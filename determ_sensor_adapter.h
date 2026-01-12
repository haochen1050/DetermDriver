/**
 * @file determ_sensor_adapter.h
 * @author Hao Chen (hao.chen@determtech.com)
 * @brief DetermSensor adapter 
 * @version 1.0.0
 * @date 2025-11-15
 *
 * @copyright Copyright (c) 2025 Determ Inc. All rights reserved
 *
 */

#pragma once

#include <memory>
#include <thread>
#include <mutex>
#include <map>
#include <condition_variable>
#include <atomic>

#include <capture/sensor/sensor.h>
#include <capture/sensor/sensor_config.h>
#include <capture/const.h>
#include <capture/data_structure.h>

// Include the new DetermSensor implementation
#include "capture/sensor/determ_sensor.h"

namespace determ {
namespace capture {

/**
 * @brief Adapter for DetermSensor (Raspberry Pi Camera + RealSense D421)
 * 
 * This adapter integrates the DetermSensor into the existing capture framework,
 * providing compatibility with the RealSense-based configuration system while
 * utilizing advanced buffered synchronization for robust RGBD capture.
 * 
 * Key Features:
 * - Advanced buffered synchronization with adaptive parameters
 * - Automatic synchronization mode selection (HYBRID by default)
 * - Real-time sync performance monitoring
 * - Calibration management compatible with existing framework
 * - Comprehensive logging and diagnostics
 * - Drop-in replacement for RealSenseSensor
 */
class DetermSensorAdapter : public Sensor
{
public:
    DetermSensorAdapter();
    virtual ~DetermSensorAdapter();

    /**
     * @brief Initialize the adapter with capture client
     * 
     * Sets up:
     * - Sensor configuration from RealSense-compatible config file
     * - Advanced buffered synchronization parameters
     * - Calibration loading and conversion
     * 
     * @param client Shared pointer to capture client
     * @return true if initialization successful
     */
    bool Init(std::shared_ptr<CaptureClient> client) override;

    /**
     * @brief Start the main capture loop with buffered synchronization
     * 
     * Launches capture thread that:
     * - Uses StartSynchronizedCapture() for continuous buffered sync
     * - Monitors sync statistics and performance
     * - Handles frame queuing and processing
     * - Reports sync quality metrics periodically
     */
    void Run() override;

    /**
     * @brief Release adapter resources
     */
    void Release() override;

    /**
     * @brief Start the sensor with configured parameters
     * 
     * Applies exposure and white balance settings, then starts
     * both Raspberry Pi camera and RealSense depth sensor.
     * 
     * @return true if sensor started successfully
     */
    bool StartSensor() override;

    /**
     * @brief Stop the sensor and capture threads
     * 
     * Cleanly shuts down synchronized capture and prints
     * final synchronization statistics.
     * 
     * @return true if sensor stopped successfully
     */
    bool StopSensor() override;

    /**
     * @brief Read sensor configuration from JSON file
     * 
     * Uses RealSense-compatible format for easy migration.
     * 
     * @return true if config read successfully
     */
    bool ReadSensorConfigFromFile() override;

    /**
     * @brief Get sensor intrinsic and extrinsic parameters
     * 
     * Attempts to extract calibration from sensor or load from file.
     * 
     * @return true if calibration obtained successfully
     */
    bool GetSensorIntrinsicAndExtinsic() override;

    /**
     * @brief Get the sensor config object
     *
     * @return std::shared_ptr<RealSenseSensorConfig> The sensor config object
     */
    inline std::shared_ptr<RealSenseSensorConfig> GetSensorConfig() { return m_sensor_config; }

    /**
     * @brief Get the Depth Cut Range
     *
     * @return CutRange The depth cut range
     */
    CutRange GetDepthCutRange() override { return m_sensor_config->m_cut_range; }

    /**
     * @brief Get the Width of the color sensor image video
     *
     * @return int The width of the color sensor image video
     */
    int GetColorWidth() override { return m_sensor_config->m_color_intrinsic.m_width; }

    /**
     * @brief Get the Height of the color sensor image video
     *
     * @return int The height of the color sensor image video
     */
    int GetColorHeight() override { return m_sensor_config->m_color_intrinsic.m_height; }

    /**
     * @brief Get the Width of the depth sensor image video
     *
     * @return int The width of the depth sensor image video
     */
    int GetDepthWidth() override { return m_sensor_config->m_depth_intrinsic.m_width; }

    /**
     * @brief Get the Height of the depth sensor image video
     *
     * @return int The height of the depth sensor image video
     */
    int GetDepthHeight() override { return m_sensor_config->m_depth_intrinsic.m_height; }

    /**
     * @brief Set color sensor control parameters
     * 
     * Configures Raspberry Pi camera exposure and white balance.
     * Converts Kelvin white balance to normalized RGB gains.
     * 
     * @param enableAuto Enable automatic exposure/white balance
     * @param exposure_value Manual exposure time in microseconds (1-10000)
     * @param whitebalance_value White balance in Kelvin (2800-6500)
     * @return true if controls set successfully
     */
    bool SetDetermSensorColorControl(bool enableAuto, int exposure_value, int whitebalance_value);

    /**
     * @brief Is the sensor started
     *
     * @return true If the sensor is started
     * @return false If the sensor is not started
     */
    inline bool IsSensorStarted() { return m_sensor_started; }

private:
    /**
     * @brief Configure DetermSensor with advanced buffered synchronization
     * 
     * Configures:
     * - Stream enable/disable
     * - Resolution and format settings
     * - Sync tolerance (adaptive based on FPS)
     * - Buffer size (adaptive based on expected latency)
     * 
     * Calculates optimal parameters:
     * - sync_tolerance_ms = 1000.0 / fps (1 frame period)
     * - sync_buffer_size = (500ms / frame_period_ms) + 5 frames
     * - Memory footprint estimation
     * 
     * @return true if configuration successful
     */
    bool ConfigureDetermSensor();

    /**
     * @brief Convert calibration from DetermSensor to framework format
     * 
     * Converts native RealSense rs2_intrinsics/rs2_extrinsics to
     * SensorIntrinsic/SensorExtrinsic structures used by framework.
     * 
     * @return true if conversion successful
     */
    bool ConvertCalibrationToSensorConfig();

    /**
     * @brief Map framework ColorFormat to DetermSensor format string
     * 
     * @param format Framework color format enum
     * @return DetermSensor format string (e.g., "YUYV", "RGB888")
     */
    std::string MapColorFormat(ColorFormat format);

    /**
     * @brief Convert white balance from Kelvin to normalized value
     * 
     * Mapping:
     * - 2800K (warm) → 0.5 (more red gain)
     * - 4500K (neutral) → 1.0 (balanced)
     * - 6500K (cool) → 2.0 (more blue gain)
     * 
     * @param kelvin White balance temperature in Kelvin
     * @return Normalized white balance value (0.5 - 2.0)
     */
    float ConvertWhiteBalanceKelvinToNormalized(int kelvin);

    bool StopSensorHardware(); ///< Stop the underlying sensor hardware

private:
    std::shared_ptr<CaptureClient> m_client;

    // Core sensor instance with advanced synchronization
    std::unique_ptr<determ::capture::DetermSensor> m_determ_sensor;

    // Sensor state
    std::atomic<bool> m_sensor_started;
    std::atomic<bool> m_quit_capture_loop;

    // Sensor config (RealSense format for compatibility)
    std::string m_config_path;
    std::shared_ptr<RealSenseSensorConfig> m_sensor_config;

    // Data capturing thread
    std::thread m_sensor_data_capturing_thread;

    // Enabled streams configuration
    std::vector<std::string> m_enabled_streams;

    // Statistics tracking
    int m_sync_stats_counter;  ///< Counter for periodic sync stats logging

    std::atomic<bool> m_callback_processing_enabled;  ///< Flag to enable/disable callback processing
};

} // namespace capture
} // namespace determ