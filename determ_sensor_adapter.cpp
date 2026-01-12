/**
 * @file determ_sensor_adapter.cpp
 * @author Hao Chen (hao.chen@determtech.com)
 * @brief Implementation of DetermSensor adapter with advanced buffered synchronization
 * @version 2.0.0
 * @date 2025-12-26
 *
 * @copyright Copyright (c) 2025 Determ Inc. All rights reserved
 *
 * COMPLETE VERSION - All fixes integrated for continuously-running streaming server
 */

#include "capture/sensor/determ_sensor_adapter.h"
#include <capture/capture_client.h>
#include <capture/sensor/depth_mask.h>
#include <capture/utility.h>
#include <mutex>  // For std::call_once

using namespace determ::capture;

DetermSensorAdapter::DetermSensorAdapter() 
    : m_sensor_started(false), 
      m_quit_capture_loop(false),
      m_sync_stats_counter(0),
      m_callback_processing_enabled(false)  // Start disabled
{
    m_determ_sensor = std::make_unique<determ::capture::DetermSensor>();
}

DetermSensorAdapter::~DetermSensorAdapter()
{
    // Ensure hardware is stopped on destruction
    if (m_sensor_started)
    {
        StopSensorHardware();
    }
}

bool DetermSensorAdapter::Init(std::shared_ptr<CaptureClient> client)
{
    m_client = client;
    m_config_path = client->GetSensorConfigPath();

    // Read the sensor config file (RealSense format)
    if (!ReadSensorConfigFromFile())
    {
        system::SYSTEM_LOG_ERROR("Failed to read the sensor config file");
        m_client->PushMessageToMessageBus(CaptureMessageType::MSG_ERROR, 
            "Failed to read the sensor config file");
        return false;
    }

    // Determine color format and depth-to-color projection
    if (client->GetStreamDataType() == StreamDataType::SDT_10BIT_RAW)
    {
        m_is_depth_to_color_projection_needed = true;
        m_color_format = ColorFormat::CF_BGRA;
        m_gstreamer_color_format = std::string("P010_10LE");
    }
    else
    {
        m_is_depth_to_color_projection_needed = false;
        if (m_sensor_config->m_color_format == "YUYV")
        {
            m_color_format = ColorFormat::CF_YUYV;
            m_gstreamer_color_format = std::string("YUY2");
        }
        else
        {
            system::SYSTEM_LOG_ERROR("Invalid color format");
            return false;
        }
    }

    // Initialize the underlying DetermSensor
    if (m_determ_sensor->Initialize() != 0)
    {
        system::SYSTEM_LOG_ERROR("Failed to initialize DetermSensor");
        m_client->PushMessageToMessageBus(CaptureMessageType::MSG_ERROR, 
            "Failed to initialize DetermSensor");
        return false;
    }

    // Configure DetermSensor based on the RealSense-style config
    if (!ConfigureDetermSensor())
    {
        system::SYSTEM_LOG_ERROR("Failed to configure DetermSensor");
        m_client->PushMessageToMessageBus(CaptureMessageType::MSG_ERROR, 
            "Failed to configure DetermSensor");
        return false;
    }

    // ═══════════════════════════════════════════════════════════
    // CONFIGURE SYNCHRONIZATION PARAMETERS
    // ═══════════════════════════════════════════════════════════
    // Set sync tolerance 
    m_determ_sensor->SetSyncTolerance(16);
    
    // Set buffer size (larger buffer = better sync success rate)
    // ~1 second of buffering = 30 frames per stream
    m_determ_sensor->SetSyncBufferSize(50);

    system::SYSTEM_LOG_INFO("Synchronization configured: tolerance=16ms, buffer_size=30");

    // ═══════════════════════════════════════════════════════════
    // CALIBRATION LOADING - CRITICAL FOR AVOIDING BUFFER ISSUES
    // ═══════════════════════════════════════════════════════════
    std::string calibration_file = m_config_path + DETERM_SENSOR_CONFIG_FILE;
    
    // Try to load calibration file FIRST
    // This avoids the start/stop cycle in GetSensorIntrinsicAndExtinsic()
    // which clears Raspberry Pi buffers
    bool calibration_loaded = m_determ_sensor->LoadCalibration(calibration_file);
    
    if (calibration_loaded)
    {
        system::SYSTEM_LOG_INFO("✓ Loaded calibration from: {}", calibration_file);
        
        // Convert loaded calibration to sensor config format
        if (!ConvertCalibrationToSensorConfig())
        {
            system::SYSTEM_LOG_ERROR("Failed to convert calibration to sensor config");
            return false;
        }
    }
    else
    {
        system::SYSTEM_LOG_WARNING("⚠ No calibration file found at: {}", calibration_file);
        system::SYSTEM_LOG_INFO("Will extract calibration from sensor (requires temporary start/stop)");
        
        // GetSensorIntrinsicAndExtinsic() will temporarily start/stop sensors
        // This clears Raspberry Pi buffers!
        if (!GetSensorIntrinsicAndExtinsic())
        {
            system::SYSTEM_LOG_ERROR("Failed to get sensor intrinsic and extrinsic parameters");
            m_client->PushMessageToMessageBus(CaptureMessageType::MSG_ERROR, 
                "Failed to get sensor intrinsic and extrinsic parameters");
            return false;
        }
        
        // CRITICAL FIX: Re-configure color sensor to re-setup buffers
        // GetSensorIntrinsicAndExtinsic() calls Stop() which clears m_requests
        system::SYSTEM_LOG_INFO("Re-configuring color sensor after calibration extraction...");
        std::string color_format = MapColorFormat(m_color_format);
        if (m_determ_sensor->SetColorSensorConfig(
                m_sensor_config->m_color_resolution.m_width,
                m_sensor_config->m_color_resolution.m_height,
                color_format) != 0)
        {
            system::SYSTEM_LOG_ERROR("Failed to re-configure color sensor buffers");
            m_client->PushMessageToMessageBus(CaptureMessageType::MSG_ERROR, 
                "Failed to re-configure color sensor buffers");
            return false;
        }
        system::SYSTEM_LOG_INFO("✓ Color sensor buffers re-initialized after calibration extraction");
    }

    // Print configuration
    m_sensor_config->Print();
    
    system::SYSTEM_LOG_INFO("✓ DetermSensor adapter initialized successfully");
    
    return true;
}

void DetermSensorAdapter::Run()
{
    // ═══════════════════════════════════════════════════════════
    // CRITICAL: Ensure previous capture thread is stopped cleanly
    // ═══════════════════════════════════════════════════════════
    if (m_sensor_data_capturing_thread.joinable())
    {
        system::SYSTEM_LOG_INFO("Stopping previous capture thread...");
        m_quit_capture_loop = true;
        
        // Give threads time to see the quit flag
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        m_sensor_data_capturing_thread.join();
        system::SYSTEM_LOG_INFO("✓ Previous capture thread stopped");
    }

    // Reset FPS counter
    m_fps.Reset(m_client->GetCaptureConfig()->m_fps_interval);
    m_quit_capture_loop = false;

    // ═══════════════════════════════════════════════════════════
    // Register frame callback ONCE during first Run() call
    // After that, the callback stays registered and active
    // ═══════════════════════════════════════════════════════════
    static std::once_flag callback_registered;
    std::call_once(callback_registered, [this]() {
        system::SYSTEM_LOG_INFO("Registering frame callback (one-time setup)...");
        
        m_determ_sensor->setFrameCallback(
            [this](const determ::capture::SynchronizedFrameSet& frameset) {
                // ═══════════════════════════════════════════════════════════
                // CRITICAL: Multi-level safety checks before processing
                // ═══════════════════════════════════════════════════════════
                
                // Check 1: Is callback processing enabled?
                if (!m_callback_processing_enabled.load()) {
                    return;  // Callback disabled during disconnect
                }
                
                // Check 2: Should we quit?
                if (m_quit_capture_loop.load()) {
                    return;  // Quit flag set
                }
                
                // Check 3: Do we have valid frames?
                const auto* color_frame = frameset.getFrame("color");
                const auto* depth_frame = frameset.getFrame("depth");

                if (!color_frame || !depth_frame) {
                    return;  // Invalid frames
                }

                // Print sync statistics periodically
                static std::atomic<int> callback_counter{0};
                if (++callback_counter % 100 == 0) {
                    system::SYSTEM_LOG_INFO("Buffered Sync #{}: {:.2f} ms", 
                        frameset.sync_id, frameset.max_time_diff_ms);
                }

                // Create RGBD data structure
                try {
                    std::shared_ptr<ImageRawData> rgbd_data = std::make_shared<ImageRawData>(
                        m_color_format,
                        color_frame->width, color_frame->height,
                        depth_frame->width, depth_frame->height);

                    // Copy frame data
                    memcpy(rgbd_data->m_color, color_frame->data.data(), color_frame->data.size());
                    memcpy(rgbd_data->m_depth, depth_frame->data.data(), depth_frame->data.size());

                    // Apply depth filtering if needed
                    if (!m_is_depth_to_color_projection_needed) {
                        FilterDepthWithMask(
                            reinterpret_cast<uint16_t*>(rgbd_data->m_depth), 
                            depth_frame->data.size() / sizeof(uint16_t));
                    }

                    // Push to client queue
                    m_client->m_image_raw_data_queue->m_image_raw_data_queue_mtx.lock();
                    if (m_client->m_image_raw_data_queue->m_image_raw_data_queue.size() < K_MAX_QUEUE_SIZE) {
                        m_client->m_image_raw_data_queue->m_image_raw_data_queue.push(rgbd_data);
                    } else {
                        m_client->m_image_raw_data_queue->m_image_raw_data_queue.pop();
                        m_client->m_image_raw_data_queue->m_image_raw_data_queue.push(rgbd_data);
                    }
                    m_client->m_image_raw_data_queue->m_image_raw_data_queue_mtx.unlock();

                    // Trigger processing if needed
                    if (m_is_depth_to_color_projection_needed) {
                        m_client->OnOneRGBDFrameCaptured();
                    }

                    // Calculate FPS
                    if (m_client->GetCaptureConfig()->m_fps_output) {
                        if (m_fps.CalculateFPS()) {
                            system::SYSTEM_LOG_INFO("Thread data gathering fps: {}", m_fps.GetFPS());
                        }
                    }
                } catch (const std::exception& e) {
                    system::SYSTEM_LOG_ERROR("Exception in frame callback: {}", e.what());
                }
            }
        );
        
        system::SYSTEM_LOG_INFO("✓ Frame callback registered");
    });

    // Start the capture management thread
    m_sensor_data_capturing_thread = std::thread([this]() {
        system::SYSTEM_LOG_INFO("╔══════════════════════════════════════════════════════════╗");
        system::SYSTEM_LOG_INFO("║   Capture Thread Started (Client Connected)             ║");
        system::SYSTEM_LOG_INFO("╚══════════════════════════════════════════════════════════╝");

        // Enable callback processing
        m_callback_processing_enabled = true;
        system::SYSTEM_LOG_INFO("✓ Callback processing enabled");

        // Start synchronized capture (idempotent - safe to call multiple times)
        if (!m_determ_sensor->StartSynchronizedCapture())
        {
            system::SYSTEM_LOG_ERROR("Failed to start synchronized capture");
            m_quit_capture_loop = true;
            m_callback_processing_enabled = false;
            return;
        }

        system::SYSTEM_LOG_INFO("Buffered synchronization active (callback mode)");

        // Main thread loop - just waits and monitors
        int stats_counter = 0;
        while (!m_quit_capture_loop.load()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            // Print stats every 5 seconds
            if (++stats_counter % 5 == 0 && !m_quit_capture_loop.load()) {
                m_determ_sensor->printSyncStats(false);
            }
        }

        // Disable callback processing before exiting
        m_callback_processing_enabled = false;
        system::SYSTEM_LOG_INFO("✓ Callback processing disabled");
        system::SYSTEM_LOG_INFO("Capture thread exiting...");
    });

    system::SYSTEM_LOG_INFO("✓ Capture thread started");
}

void DetermSensorAdapter::Release()
{
    // On Release (application shutdown), actually stop the hardware
    StopSensorHardware();
    system::SYSTEM_LOG_DEBUG("Release DetermSensor Adapter object");
}

bool DetermSensorAdapter::StartSensor()
{
    // ═══════════════════════════════════════════════════════════
    // CRITICAL: Enhanced guard for continuously-running servers
    // Checks BOTH the flag AND actual sensor state
    // This handles cases where sensors start during Init() but flag isn't set
    // ═══════════════════════════════════════════════════════════
    if (m_sensor_started.load() || m_determ_sensor->IsRunning()) {
        system::SYSTEM_LOG_INFO("╔══════════════════════════════════════════════════════════╗");
        system::SYSTEM_LOG_INFO("║   Sensors Already Running - Skipping Redundant Start    ║");
        system::SYSTEM_LOG_INFO("╠══════════════════════════════════════════════════════════╣");
        system::SYSTEM_LOG_INFO("║ This is NORMAL for continuously-running streaming       ║");
        system::SYSTEM_LOG_INFO("║ servers. Sensors were started at application init       ║");
        system::SYSTEM_LOG_INFO("║ and remain active across client connect/disconnect.     ║");
        system::SYSTEM_LOG_INFO("╚══════════════════════════════════════════════════════════╝");
        
        // Sync the flag with actual state
        if (!m_sensor_started.load() && m_determ_sensor->IsRunning()) {
            system::SYSTEM_LOG_INFO("Syncing m_sensor_started flag with actual sensor state");
            m_sensor_started = true;
        }
        
        return true;  // Return success without doing anything
    }

    system::SYSTEM_LOG_INFO("Starting sensors for the FIRST TIME...");

    // Read current exposure and white balance settings from config
    nlohmann::json json_data;
    std::string config_file = m_config_path + DETERM_SENSOR_CONFIG_FILE;
    if (!determ::utility::Json::ReadJsonFromFile(config_file, json_data))
    {
        system::SYSTEM_LOG_ERROR("Failed to read sensor exposure and white balance settings from {}", config_file);
        m_client->PushMessageToMessageBus(CaptureMessageType::MSG_ERROR, 
            "Failed to read sensor exposure and white balance settings");
        return false;
    }
    else
    {
        m_sensor_config->m_auto_exposure = json_data["Auto Exposure"].get<bool>();
        m_sensor_config->m_exposure = json_data["Exposure"].get<int>();
        m_sensor_config->m_white_balance = json_data["White Balance"].get<int>();
    }

    // Apply color control settings BEFORE starting sensors
    if (!SetDetermSensorColorControl(
            m_sensor_config->m_auto_exposure, 
            m_sensor_config->m_exposure, 
            m_sensor_config->m_white_balance))
    {
        system::SYSTEM_LOG_ERROR("Failed to set sensor color controls");
        m_client->PushMessageToMessageBus(CaptureMessageType::MSG_ERROR, 
            "Failed to set sensor color controls");
        return false;
    }

    // Start the DetermSensor - this starts both hardware sensors
    if (m_determ_sensor->Start() != 0)
    {
        system::SYSTEM_LOG_ERROR("Failed to start DetermSensor");
        m_client->PushMessageToMessageBus(CaptureMessageType::MSG_ERROR, 
            "Failed to start DetermSensor");
        return false;
    }

    m_sensor_started = true;
    system::SYSTEM_LOG_INFO("✓ DetermSensor adapter started successfully");
    system::SYSTEM_LOG_INFO("✓ Hardware sensors running - ready for buffered synchronized capture");

    return true;
}

bool DetermSensorAdapter::StopSensor()
{
    // ═══════════════════════════════════════════════════════════
    // CRITICAL: Intelligent stop behavior for streaming server
    // 
    // In continuously-running streaming mode, we DON'T actually
    // stop the hardware sensors when clients disconnect. Instead,
    // we just stop the capture thread (which can be restarted).
    // 
    // This allows instant reconnection without hardware restart.
    // ═══════════════════════════════════════════════════════════
    
    if (!m_sensor_started)
    {
        system::SYSTEM_LOG_INFO("Sensors not started, nothing to stop");
        return true;
    }

    system::SYSTEM_LOG_INFO("╔════════════════════════════════════════════════════════╗");
    system::SYSTEM_LOG_INFO("║   StopSensor() Called (Client Disconnect)             ║");
    system::SYSTEM_LOG_INFO("╠════════════════════════════════════════════════════════╣");
    system::SYSTEM_LOG_INFO("║ For continuously-running streaming server:             ║");
    system::SYSTEM_LOG_INFO("║   • Stopping capture thread                            ║");
    system::SYSTEM_LOG_INFO("║   • Flushing data queue                                ║");
    system::SYSTEM_LOG_INFO("║   • Hardware sensors REMAIN ACTIVE                     ║");
    system::SYSTEM_LOG_INFO("║   • Next client can connect instantly!                 ║");
    system::SYSTEM_LOG_INFO("╚════════════════════════════════════════════════════════╝");

    // ═══════════════════════════════════════════════════════════
    // CRITICAL: Disable callback processing IMMEDIATELY
    // This prevents the callback from doing ANY work during shutdown
    // ═══════════════════════════════════════════════════════════
    m_callback_processing_enabled = false;
    system::SYSTEM_LOG_INFO("✓ Callback processing disabled");
    
    // Set quit flag
    m_quit_capture_loop = true;
    
    // Wait for any in-flight callbacks to complete
    // The callback checks m_callback_processing_enabled at the start,
    // so any callbacks that started before we set it to false will
    // complete, and new callbacks will immediately return
    system::SYSTEM_LOG_INFO("Waiting for in-flight callbacks to finish...");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Stop the capture thread
    if (m_sensor_data_capturing_thread.joinable())
    {
        m_sensor_data_capturing_thread.join();
        system::SYSTEM_LOG_INFO("✓ Capture thread stopped");
    }

    // Flush the data queue for clean start with next client
    m_client->m_image_raw_data_queue->m_image_raw_data_queue_mtx.lock();
    int flushed_count = 0;
    while (!m_client->m_image_raw_data_queue->m_image_raw_data_queue.empty())
    {
        m_client->m_image_raw_data_queue->m_image_raw_data_queue.pop();
        flushed_count++;
    }
    m_client->m_image_raw_data_queue->m_image_raw_data_queue_mtx.unlock();
    
    if (flushed_count > 0)
    {
        system::SYSTEM_LOG_INFO("✓ Flushed {} frames from queue", flushed_count);
    }

    // DON'T stop the hardware sensors!
    // DON'T call m_determ_sensor->Stop()!
    // DON'T set m_sensor_started = false!
    
    system::SYSTEM_LOG_INFO("✓ Ready for next client (hardware remains active)");
    
    // Note: We keep m_sensor_started = true so the hardware stays active
    // m_callback_processing_enabled will be set to true again when Run() is called

    return true;
}

// Add a new method for ACTUAL hardware shutdown (called from destructor or Release())
bool DetermSensorAdapter::StopSensorHardware()
{
    if (!m_sensor_started)
    {
        return true;
    }

    system::SYSTEM_LOG_INFO("Stopping hardware sensors (application shutdown)...");

    // Stop the capture thread
    if (m_sensor_data_capturing_thread.joinable())
    {
        m_quit_capture_loop = true;
        m_sensor_data_capturing_thread.join();
    }

    // Now actually stop the hardware
    m_determ_sensor->Stop();

    m_sensor_started = false;
    system::SYSTEM_LOG_INFO("✓ Hardware sensors stopped");

    return true;
}

bool DetermSensorAdapter::ReadSensorConfigFromFile()
{
    nlohmann::json json_data;

    // Read sensor config (use RealSense config file format for compatibility)
    std::string config_file = m_config_path + DETERM_SENSOR_CONFIG_FILE;
    if (!determ::utility::Json::ReadJsonFromFile(config_file, json_data))
    {
        system::SYSTEM_LOG_ERROR("Failed to read the sensor config file: {}", config_file);
        return false;
    }
    else
    {
        SensorConfigFactory sensor_config_factory;
        m_sensor_config = sensor_config_factory.CreateRealSenseSensorConfig(json_data);
    }

    return true;
}

bool DetermSensorAdapter::GetSensorIntrinsicAndExtinsic()
{
    // ═══════════════════════════════════════════════════════════
    // CRITICAL: Check if calibration was already loaded from file
    // This avoids the start/stop cycle that clears buffers
    // ═══════════════════════════════════════════════════════════
    if (m_determ_sensor->HasCalibration())
    {
        system::SYSTEM_LOG_INFO("Calibration already loaded from file, converting to sensor config format");
        return ConvertCalibrationToSensorConfig();
    }

    // No calibration file found, need to extract from sensor
    system::SYSTEM_LOG_WARNING("No calibration file found, extracting from sensor (requires temporary start/stop)");
    
    if (m_sensor_started)
    {
        system::SYSTEM_LOG_ERROR("Data streaming is running. Please stop data streaming first.");
        return false;
    }

    // Start sensor temporarily to get calibration
    if (m_determ_sensor->Start() != 0)
    {
        system::SYSTEM_LOG_ERROR("Failed to start DetermSensor for calibration extraction");
        return false;
    }

    // TODO: Extract calibration from running RealSense sensor
    // For now, the calibration should be in the DetermSensor after Start()
    
    // Stop the sensor (this will clear Raspberry Pi buffers!)
    m_determ_sensor->Stop();

    // If we have calibration loaded, convert it
    if (m_determ_sensor->HasCalibration())
    {
        return ConvertCalibrationToSensorConfig();
    }
    else
    {
        system::SYSTEM_LOG_ERROR("Failed to get calibration from DetermSensor");
        return false;
    }
}

bool DetermSensorAdapter::ConvertCalibrationToSensorConfig()
{
    if (!m_determ_sensor->HasCalibration())
    {
        system::SYSTEM_LOG_ERROR("No calibration data available");
        return false;
    }

    // Get calibration data from DetermSensor
    const auto& color_intr = m_determ_sensor->GetColorIntrinsics();
    const auto& depth_intr = m_determ_sensor->GetDepthIntrinsics();
    const auto& depth_to_color_extr = m_determ_sensor->GetDepthToColorExtrinsics();

    // Convert color intrinsics
    SensorDistortion color_distortion;
    color_distortion.Init(
        color_intr.coeffs[0], color_intr.coeffs[1],
        color_intr.coeffs[4], 0.0,
        0.0, 0.0,
        color_intr.coeffs[2], color_intr.coeffs[3]);
    
    m_sensor_config->m_color_intrinsic.Init(
        color_intr.width, color_intr.height,
        color_intr.fx, color_intr.fy,
        color_intr.ppx, color_intr.ppy,
        color_distortion);

    // Convert depth intrinsics
    SensorDistortion depth_distortion;
    depth_distortion.Init(
        depth_intr.coeffs[0], depth_intr.coeffs[1],
        depth_intr.coeffs[4], 0.0,
        0.0, 0.0,
        depth_intr.coeffs[2], depth_intr.coeffs[3]);
    
    m_sensor_config->m_depth_intrinsic.Init(
        depth_intr.width, depth_intr.height,
        depth_intr.fx, depth_intr.fy,
        depth_intr.ppx, depth_intr.ppy,
        depth_distortion);

    // Convert extrinsics (depth to color)
    float transformation_matrix[16] = {
        depth_to_color_extr.rotation[0], depth_to_color_extr.rotation[1], 
        depth_to_color_extr.rotation[2], depth_to_color_extr.translation[0],
        
        depth_to_color_extr.rotation[3], depth_to_color_extr.rotation[4], 
        depth_to_color_extr.rotation[5], depth_to_color_extr.translation[1],
        
        depth_to_color_extr.rotation[6], depth_to_color_extr.rotation[7], 
        depth_to_color_extr.rotation[8], depth_to_color_extr.translation[2],
        
        0, 0, 0, 1
    };
    
    m_sensor_config->m_depth_to_color_extrinsic.Init(transformation_matrix);

    system::SYSTEM_LOG_INFO("✓ Converted DetermSensor calibration to sensor config format");
    return true;
}

bool DetermSensorAdapter::ConfigureDetermSensor()
{
    // Enable streams
    std::vector<std::string> streams = {"color", "depth"};
    if (m_determ_sensor->EnableStream(streams) != 0)
    {
        system::SYSTEM_LOG_ERROR("Failed to enable streams");
        return false;
    }

    // Configure color sensor
    std::string color_format = MapColorFormat(m_color_format);
    if (m_determ_sensor->SetColorSensorConfig(
            m_sensor_config->m_color_resolution.m_width,
            m_sensor_config->m_color_resolution.m_height,
            color_format) != 0)
    {
        system::SYSTEM_LOG_ERROR("Failed to configure color sensor");
        return false;
    }

    // Configure depth sensor
    if (m_determ_sensor->SetDepthSensorConfig(
            m_sensor_config->m_depth_resolution.m_width,
            m_sensor_config->m_depth_resolution.m_height) != 0)
    {
        system::SYSTEM_LOG_ERROR("Failed to configure depth sensor");
        return false;
    }

    system::SYSTEM_LOG_INFO("✓ DetermSensor configured successfully");
    return true;
}

std::string DetermSensorAdapter::MapColorFormat(ColorFormat format)
{
    switch (format)
    {
        case ColorFormat::CF_YUYV:
            return "YUYV";
        case ColorFormat::CF_BGRA:
            return "BGRA";
        case ColorFormat::CF_RGB:
            return "RGB";
        default:
            return "YUYV";
    }
}

bool DetermSensorAdapter::SetDetermSensorColorControl(bool is_auto, int exposure_time, int white_balance)
{
    if (!m_determ_sensor->SetColorSensorControl(is_auto, 
            static_cast<float>(exposure_time), 
            static_cast<float>(white_balance)))
    {
        system::SYSTEM_LOG_ERROR("Failed to set DetermSensor color control");
        return false;
    }

    system::SYSTEM_LOG_INFO("✓ Set DetermSensor color control to {}", is_auto ? "auto" : "manual");
    return true;
}

// Note: Getters are inline in header (GetColorWidth, GetColorHeight, etc.)
// Only implement non-inline methods here