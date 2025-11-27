#include "sensor/raspberry_sensor.h"

using determ::sensor::RaspberrySensor;
using determ::sensor::TimestampedRaspberryFrame;
RaspberrySensor::RaspberrySensor()
    : m_cm(nullptr),
      m_camera(nullptr),
      m_stream(nullptr),
      m_is_running(false),
      m_current_buffer_index(0),
      m_resolution(1920, 1080),
      m_pixel_format(formats::RGB888),
      m_bytes_per_pixel(3),
      m_auto_mode(true),
      m_manual_exposure_time(10000.0f),
      m_manual_white_balance(1.0f),
      m_auto_focus_enabled(true),
      m_manual_focus_position(0.0f)
{
}

RaspberrySensor::~RaspberrySensor()
{
  // Ensure camera is stopped first
  Stop();

  // Give system time to complete cleanup
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Release camera
  if (m_camera) {
    m_camera->release();
    m_camera.reset();
  }
           
  // Stop camera manager last
  if (m_cm) {
    m_cm->stop();
    m_cm.reset();
  }

  // Give final moment for system cleanup
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

int RaspberrySensor::Initialize()
{
  // Initialize camera manager
  m_cm = std::make_unique<CameraManager>();
  int ret = m_cm->start();
  if (ret) {
    std::cerr << "Failed to start camera manager: " << ret << std::endl;
    return ret;
  }

  // Get available cameras
  auto cameras = m_cm->cameras();
  if (cameras.empty()) {
    std::cerr << "No cameras available" << std::endl;
    return -1;
  }

  // Use the first available camera (typically the Pi camera)
  m_camera = cameras[0];
  std::cout << "Using camera: " << m_camera->id() << std::endl;

  // Acquire the camera
  ret = m_camera->acquire();
  if (ret) {
    std::cerr << "Failed to acquire sensor: " << ret << std::endl;
    return ret;
  }

  std::cout << "Raspberry v3 Camera initialized successfully" << std::endl;
  return 0;
}

PixelFormat RaspberrySensor::StringToPixelFormat(const std::string &format)
{
  if (format == "RGB888" || format == "RGB") {
    return formats::RGB888;
  } else if (format == "BGR888" || format == "BGR") {
    return formats::BGR888;
  } else if (format == "YUYV") {
    return formats::YUYV;
  } else if (format == "NV12") {
    return formats::NV12;
  } else {
    std::cerr << "Unknown format: " << format << ", defaulting to RGB888" << std::endl;
    return formats::RGB888;
  }
}

float RaspberrySensor::GetBytesPerPixel(const PixelFormat &format)
{
  if (format == formats::RGB888 || format == formats::BGR888) {
    return 3;
  } else if (format == formats::YUYV) {
    return 2;
  } else if (format == formats::NV12) {
    return 1.5;  // 12 bits per pixel
  } else {
    return 3;  // Default to 3 bytes per pixel
  }
}

size_t RaspberrySensor::GetFrameSize(const PixelFormat &format, const Size &resolution)
{
  if (format == formats::RGB888 || format == formats::BGR888) {
    return resolution.width * resolution.height * 3;
  } else if (format == formats::YUYV) {
    return resolution.width * resolution.height * 2;
  } else if (format == formats::NV12) {
    // Y plane: width * height
    // UV plane: (width * height) / 2 (interleaved U and V)
    // Total: width * height * 1.5
    return (resolution.width * resolution.height * 3) / 2;
  } else {
    return resolution.width * resolution.height * 3;  // Default
  }
}

void RaspberrySensor::CalculateWhiteBalanceGains(float white_balance, float &red_gain, float &blue_gain)
{
  // Convert white balance value to red and blue gains
  // white_balance: 0.5 = warm (more red), 1.0 = neutral, 2.0 = cool (more blue)

  if (white_balance < 1.0f) {
    // Warmer: increase red, decrease blue
    red_gain = 1.0f + (1.0f - white_balance) * 0.8f;  // Max red gain ~1.4
    blue_gain = white_balance;                        // Blue gain decreases
  } else {
    // Cooler: decrease red, increase blue
    red_gain = 1.0f / white_balance;                   // Red gain decreases
    blue_gain = 1.0f + (white_balance - 1.0f) * 0.8f;  // Max blue gain ~1.8
  }

  // Clamp gains to reasonable ranges
  red_gain = std::max(0.5f, std::min(2.0f, red_gain));
  blue_gain = std::max(0.5f, std::min(2.0f, blue_gain));
}

int RaspberrySensor::SetSensorConfig(int width, int height, const std::string &format)
{
  if (m_is_running) {
    std::cerr << "Cannot change configuration while camera is running" << std::endl;
    return -1;
  }

  std::cout << "Setting sensor configuration:" << std::endl;
  std::cout << "  Width: " << width << std::endl;
  std::cout << "  Height: " << height << std::endl;
  std::cout << "  Format: " << format << std::endl;

  m_resolution = Size(width, height);
  m_pixel_format = StringToPixelFormat(format);
  m_bytes_per_pixel = GetBytesPerPixel(m_pixel_format);

  // Generate camera configuration for video capture
  m_config = m_camera->generateConfiguration({StreamRole::VideoRecording});
  if (!m_config) {
    std::cerr << "Failed to generate configuration" << std::endl;
    return -1;
  }

  // Configure the stream
  StreamConfiguration &stream_config = m_config->at(0);
  stream_config.pixelFormat = m_pixel_format;
  stream_config.size = m_resolution;

  // Validate the configuration
  CameraConfiguration::Status validation = m_config->validate();
  if (validation == CameraConfiguration::Invalid) {
    std::cerr << "Configuration invalid" << std::endl;
    return -1;
  }
  if (validation == CameraConfiguration::Adjusted) {
    std::cout << "Configuration adjusted" << std::endl;
    // Update our stored values with the adjusted configuration
    m_resolution = stream_config.size;
    m_pixel_format = stream_config.pixelFormat;
    m_bytes_per_pixel = GetBytesPerPixel(m_pixel_format);
  }

  // Apply the configuration
  int ret = m_camera->configure(m_config.get());
  if (ret) {
    std::cerr << "Failed to configure camera: " << ret << std::endl;
    return ret;
  }

  // Get the configured stream
  m_stream = stream_config.stream();

  std::cout << "Sensor configured:" << std::endl;
  std::cout << "  Format: " << m_pixel_format.toString() << std::endl;
  std::cout << "  Size: " << m_resolution.toString() << std::endl;
  std::cout << "  Bytes per pixel: " << m_bytes_per_pixel << std::endl;

  // ALWAYS enable autofocus after any resolution change
  std::cout << "Enabling autofocus for new resolution..." << std::endl;
  m_auto_focus_enabled = true;  // Force autofocus to be enabled

  // Set up autofocus controls
  if (!SetFocusControl(true)) {
    std::cerr << "Warning: Failed to enable autofocus, but continuing..." << std::endl;
  }

  // Setup buffers first
  ret = SetupBuffers();
  if (ret != 0) {
    return ret;
  }

  // After buffers are set up, trigger autofocus for the new resolution
  std::cout << "Triggering autofocus for resolution " << width << "x" << height << "..." << std::endl;

  // Give the system a moment to stabilize
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Trigger autofocus (this will work better after buffers are ready)
  TriggerAutofocus();

  // Give autofocus more time to settle for new resolution
  std::cout << "Waiting for autofocus to settle..." << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::cout << "Autofocus setup complete for resolution " << width << "x" << height << std::endl;

  return 0;
}

bool RaspberrySensor::SetSensorColorControl(bool is_auto, float exposure_time, float white_balance)
{
  if (!m_camera) {
    std::cerr << "Camera not initialized. Call Initialize() first." << std::endl;
    return false;
  }

  m_auto_mode = is_auto;
  m_manual_exposure_time = exposure_time;
  m_manual_white_balance = white_balance;

  // Clear previous controls
  m_current_controls.clear();

  // Get available controls to check what's supported
  const ControlInfoMap &supported_controls = m_camera->controls();

  if (is_auto) {
    std::cout << "Setting auto exposure and white balance mode" << std::endl;

    // For auto mode, we mainly control white balance mode
    // Raspberry Pi cameras handle exposure automatically by default

    // Set auto white balance mode if supported
    if (supported_controls.find(&controls::AwbMode) != supported_controls.end()) {
      m_current_controls.set(controls::AwbMode, controls::AwbAuto);
      std::cout << "  Auto white balance mode set" << std::endl;
    }

    // Try to set exposure mode preferences (optional)
    if (supported_controls.find(&controls::AeExposureMode) != supported_controls.end()) {
      m_current_controls.set(controls::AeExposureMode, controls::ExposureNormal);
      std::cout << "  Exposure mode set to normal" << std::endl;
    }

    // Try to set metering mode (optional)
    if (supported_controls.find(&controls::AeMeteringMode) != supported_controls.end()) {
      m_current_controls.set(controls::AeMeteringMode, controls::MeteringMatrix);
      std::cout << "  Metering mode set to matrix" << std::endl;
    }

    std::cout << "Auto mode enabled successfully" << std::endl;

  } else {
    std::cout << "Setting manual exposure and white balance" << std::endl;

    // Set manual exposure time if supported
    if (supported_controls.find(&controls::ExposureTime) != supported_controls.end()) {
      int64_t exposure_us = static_cast<int64_t>(exposure_time);

      // Check if the exposure time is within supported range
      const ControlInfo &exp_info = supported_controls.at(&controls::ExposureTime);
      int64_t min_exp = exp_info.min().get<int64_t>();
      int64_t max_exp = exp_info.max().get<int64_t>();

      // Clamp exposure to supported range
      exposure_us = std::max(min_exp, std::min(max_exp, exposure_us));

      m_current_controls.set(controls::ExposureTime, exposure_us);
      std::cout << "  Exposure time: " << exposure_us << " Î¼s (range: " << min_exp << "-" << max_exp << ")"
                << std::endl;
    } else {
      std::cerr << "  Warning: ExposureTime control not supported" << std::endl;
    }

    // Set analog gain if supported
    if (supported_controls.find(&controls::AnalogueGain) != supported_controls.end()) {
      m_current_controls.set(controls::AnalogueGain, 1.0f);
      std::cout << "  Analog gain set to 1.0" << std::endl;
    }

    // Set digital gain if supported
    if (supported_controls.find(&controls::DigitalGain) != supported_controls.end()) {
      m_current_controls.set(controls::DigitalGain, 1.0f);
      std::cout << "  Digital gain set to 1.0" << std::endl;
    }

    // Set manual white balance if supported
    if (supported_controls.find(&controls::ColourGains) != supported_controls.end()) {
      float red_gain, blue_gain;
      CalculateWhiteBalanceGains(white_balance, red_gain, blue_gain);
      m_current_controls.set(controls::ColourGains, {red_gain, blue_gain});
      std::cout << "  White balance gains: R=" << red_gain << ", B=" << blue_gain << std::endl;
    } else {
      std::cerr << "  Warning: ColourGains control not supported" << std::endl;
    }

    std::cout << "Manual mode set successfully" << std::endl;
  }

  return true;
}

// Add this helper method to your class to debug available controls
void RaspberrySensor::PrintAvailableControls()
{
  if (!m_camera) {
    std::cerr << "Camera not initialized" << std::endl;
    return;
  }

  std::cout << "\n========== AVAILABLE CAMERA CONTROLS ==========" << std::endl;

  const ControlInfoMap &controls = m_camera->controls();

  std::cout << "Total controls available: " << controls.size() << std::endl;

  // List important controls for debugging
  std::vector<const ControlId *> important_controls = {
      &controls::ExposureTime,   &controls::AnalogueGain,   &controls::DigitalGain, &controls::AeEnable,
      &controls::AeExposureMode, &controls::AeMeteringMode, &controls::AwbEnable,   &controls::AwbMode,
      &controls::ColourGains,    &controls::Brightness,     &controls::Contrast,    &controls::Saturation};

  for (const auto *control_id : important_controls) {
    auto it = controls.find(control_id);
    if (it != controls.end()) {
      const ControlInfo &info = it->second;
      std::cout << "âœ“ " << control_id->name();
      if (!info.values().empty()) {
        std::cout << " (range: " << info.min().toString() << " - " << info.max().toString() << ")";
      }
      std::cout << std::endl;
    } else {
      std::cout << "âœ— " << control_id->name() << " (not supported)" << std::endl;
    }
  }

  std::cout << "============================================\n" << std::endl;
}

int RaspberrySensor::SetupBuffers()
{
  // Create frame buffer allocator
  m_allocator = std::make_unique<FrameBufferAllocator>(m_camera);

  int ret = m_allocator->allocate(m_stream);
  if (ret < 0) {
    std::cerr << "Failed to allocate buffers: " << ret << std::endl;
    return ret;
  }

  std::cout << "Allocated " << m_allocator->buffers(m_stream).size() << " buffers" << std::endl;

  // Reset buffer index for cycling
  m_current_buffer_index = 0;

  // Create capture requests
  m_requests.clear();
  for (unsigned int i = 0; i < m_allocator->buffers(m_stream).size(); ++i) {
    std::unique_ptr<Request> request = m_camera->createRequest();
    if (!request) {
      std::cerr << "Failed to create request" << std::endl;
      return -1;
    }

    const std::unique_ptr<FrameBuffer> &buffer = m_allocator->buffers(m_stream)[i];
    ret = request->addBuffer(m_stream, buffer.get());
    if (ret < 0) {
      std::cerr << "Failed to add buffer to request: " << ret << std::endl;
      return ret;
    }

    // Requests will be managed manually without signals

    m_requests.push_back(std::move(request));
  }

  return 0;
}

int RaspberrySensor::ApplyControlsToRequest(Request *request, const ControlList &controls)
{
  if (!request) {
    return -1;
  }

  // Apply controls to the request
  for (const auto &[id, value] : controls) {
    request->controls().set(id, value);
  }

  return 0;
}

int RaspberrySensor::Start()
{
  if (m_is_running) {
    std::cout << "Camera is already running" << std::endl;
    return 0;
  }

  if (!m_config) {
    std::cerr << "Camera not configured. Call SetSensorConfig first." << std::endl;
    return -1;
  }

  // Start the camera
  int ret = m_camera->start();
  if (ret) {
    std::cerr << "Failed to start camera: " << ret << std::endl;
    return ret;
  }

  m_is_running = true;

  // ALWAYS ensure autofocus is enabled and triggered when starting
  std::cout << "Camera started, initializing autofocus..." << std::endl;

  // Force autofocus to be enabled
  m_auto_focus_enabled = true;
  SetFocusControl(true);

  // Give camera a moment to stabilize
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // Trigger initial autofocus
  TriggerAutofocus();

  // Wait for autofocus to complete
  std::cout << "Waiting for initial autofocus to complete..." << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  std::cout << "Camera started successfully with autofocus enabled" << std::endl;
  return 0;
}

int RaspberrySensor::Stop()
{
  if (!m_is_running) {
    return 0;
  }

  m_is_running = false;

  // Give any pending operations a moment to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Stop the camera
  int ret = m_camera->stop();
  if (ret) {
    std::cerr << "Failed to stop camera: " << ret << std::endl;
    // Continue with cleanup even if stop fails
  }

  // Clear requests to release any references to buffers
  m_requests.clear();

  // Reset buffer allocator to release buffers properly
  if (m_allocator) {
    m_allocator.reset();
  }

  // Reset buffer index
  m_current_buffer_index = 0;

  std::cout << "Camera stopped successfully" << std::endl;
  return 0;
}

TimestampedRaspberryFrame RaspberrySensor::GetTimestampedFrame()
{
    TimestampedRaspberryFrame result;
    
    if (!m_is_running) {
        std::cerr << "Camera is not running. Call Start() first." << std::endl;
        return result;
    }

    // Use pre-allocated requests from the buffer pool
    if (m_requests.empty()) {
        std::cerr << "No requests available. Call SetupBuffers() first." << std::endl;
        return result;
    }

    // Get the current request from the pool (circular buffer)
    Request *request = m_requests[m_current_buffer_index].get();
    
    // Reuse the request if it was previously used
    // This clears the previous state and makes it ready for queueing again
    request->reuse(Request::ReuseFlag::ReuseBuffers);
    
    // Apply current controls to the request
    if (!m_current_controls.empty()) {
        request->controls().clear();
        ApplyControlsToRequest(request, m_current_controls);
    }

    // Capture timestamp just before queueing
    auto frame_timestamp = std::chrono::high_resolution_clock::now();
    
    // Queue the request
    int ret = m_camera->queueRequest(request);
    if (ret < 0) {
        std::cerr << "Failed to queue request: " << ret << std::endl;
        return result;
    }

    // Advance buffer index for next frame
    m_current_buffer_index = (m_current_buffer_index + 1) % m_requests.size();

    // Wait for completion
    bool completed = false;
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(5);

    while (!completed && m_is_running) {
        auto current_time = std::chrono::steady_clock::now();
        if (current_time - start_time > timeout) {
            std::cerr << "Frame capture timeout" << std::endl;
            return result;
        }

        if (request->status() == Request::RequestComplete) {
            completed = true;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (!completed) {
        return result;
    }

    // Extract frame data using the separate method
    FrameBuffer *frame_buffer = request->buffers().begin()->second;
    if (!frame_buffer) {
        return result;
    }

    std::vector<uint8_t> frame_data = ExtractFrameData(frame_buffer);
    
    // Set the result
    result.data = std::move(frame_data);
    result.timestamp = frame_timestamp;
    result.sequence_number = ++m_frame_sequence;
    
    return result;
}

// Extract the frame data extraction logic into a separate method
std::vector<uint8_t> RaspberrySensor::ExtractFrameData(FrameBuffer *frame_buffer)
{
    // Get the memory mapped buffer
    const std::vector<FrameBuffer::Plane> &planes = frame_buffer->planes();
    if (planes.empty()) {
        std::cerr << "No planes in buffer" << std::endl;
        return {};
    }

    // Handle different formats differently
    if (m_pixel_format == formats::RGB888 || m_pixel_format == formats::BGR888 || m_pixel_format == formats::YUYV) {
        const FrameBuffer::Plane &plane = planes[0];

        // Map the buffer memory
        void *memory = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
        if (memory == MAP_FAILED) {
            std::cerr << "Failed to mmap buffer" << std::endl;
            return {};
        }

        // Calculate expected frame size
        size_t expected_size = GetFrameSize(m_pixel_format, m_resolution);
        size_t actual_size = std::min(expected_size, static_cast<size_t>(plane.length));

        // Copy the frame data
        std::vector<uint8_t> frame_data(actual_size);
        std::memcpy(frame_data.data(), memory, actual_size);

        // Unmap the memory
        munmap(memory, plane.length);

        return frame_data;
    } else if (m_pixel_format == formats::NV12) {
        // NV12 format handling
        size_t y_size = m_resolution.width * m_resolution.height;
        size_t uv_size = y_size / 2;  // UV plane is half the size of Y plane
        size_t total_size = y_size + uv_size;

        std::vector<uint8_t> frame_data(total_size);

        if (planes.size() == 1) {
            // Single plane NV12: Y data followed by interleaved UV data
            const FrameBuffer::Plane &plane = planes[0];

            void *memory = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
            if (memory == MAP_FAILED) {
                std::cerr << "Failed to mmap NV12 buffer" << std::endl;
                return {};
            }

            // Copy the entire NV12 data (Y + UV)
            size_t copy_size = std::min(total_size, static_cast<size_t>(plane.length));
            std::memcpy(frame_data.data(), memory, copy_size);

            munmap(memory, plane.length);

            return frame_data;
        } else if (planes.size() == 2) {
            // Two planes NV12: separate Y and UV planes
            const FrameBuffer::Plane &y_plane = planes[0];
            const FrameBuffer::Plane &uv_plane = planes[1];

            // Map Y plane
            void *y_memory = mmap(nullptr, y_plane.length, PROT_READ, MAP_SHARED, y_plane.fd.get(), 0);
            if (y_memory == MAP_FAILED) {
                std::cerr << "Failed to mmap Y plane of NV12 buffer" << std::endl;
                return {};
            }

            // Map UV plane
            void *uv_memory = mmap(nullptr, uv_plane.length, PROT_READ, MAP_SHARED, uv_plane.fd.get(), 0);
            if (uv_memory == MAP_FAILED) {
                std::cerr << "Failed to mmap UV plane of NV12 buffer" << std::endl;
                munmap(y_memory, y_plane.length);
                return {};
            }

            // Copy Y plane data
            size_t y_copy_size = std::min(y_size, static_cast<size_t>(y_plane.length));
            std::memcpy(frame_data.data(), y_memory, y_copy_size);

            // Copy UV plane data
            size_t uv_copy_size = std::min(uv_size, static_cast<size_t>(uv_plane.length));
            std::memcpy(frame_data.data() + y_size, uv_memory, uv_copy_size);

            // Unmap both planes
            munmap(y_memory, y_plane.length);
            munmap(uv_memory, uv_plane.length);

            return frame_data;
        } else {
            std::cerr << "Unexpected number of planes for NV12: " << planes.size() << std::endl;
            return {};
        }
    }

    return {};  // Unsupported format or no data available
}

// Keep original GetFrame for backward compatibility
std::vector<uint8_t> RaspberrySensor::GetFrame()
{
    TimestampedRaspberryFrame timestamped_frame = GetTimestampedFrame();
    return std::move(timestamped_frame.data);
}

void RaspberrySensor::PrintSupportedFormats()
{
  if (!m_camera) {
    std::cerr << "Camera not initialized. Call Initialize() first." << std::endl;
    return;
  }

  std::cout << "\n========== CAMERA CAPABILITIES ==========" << std::endl;
  std::cout << "Camera: " << m_camera->id() << std::endl;

  // Define stream roles to test
  std::vector<std::pair<StreamRole, std::string>> roles = {{StreamRole::StillCapture, "Still Capture"},
                                                           {StreamRole::VideoRecording, "Video Recording"},
                                                           {StreamRole::Viewfinder, "Viewfinder"}};

  // Common pixel formats to test
  std::vector<std::pair<PixelFormat, std::string>> formats = {
      {formats::RGB888, "RGB888"}, {formats::BGR888, "BGR888"}, {formats::YUYV, "YUYV"}, {formats::NV12, "NV12"}};

  // Common resolutions to test
  std::vector<std::pair<Size, std::string>> test_resolutions = {{Size(640, 480), "640x480"},
                                                                {Size(800, 600), "800x600"},
                                                                {Size(1024, 768), "1024x768"},
                                                                {Size(1280, 720), "1280x720 (720p)"},
                                                                {Size(1920, 1080), "1920x1080 (1080p)"},
                                                                {Size(2592, 1944), "2592x1944"},
                                                                {Size(3840, 2160), "3840x2160 (4K)"},
                                                                {Size(4608, 2592), "4608x2592 (Max Pi Camera v3)"}};

  for (const auto &[role, role_name] : roles) {
    std::cout << "\n--- " << role_name << " ---" << std::endl;

    // Generate configuration for this role
    std::unique_ptr<CameraConfiguration> test_config = m_camera->generateConfiguration({role});
    if (!test_config) {
      std::cout << "  Configuration not supported for this role" << std::endl;
      continue;
    }

    if (test_config->empty()) {
      std::cout << "  No streams available for this role" << std::endl;
      continue;
    }

    StreamConfiguration &stream_config = test_config->at(0);

    // Test different formats
    for (const auto &[format, format_name] : formats) {
      std::cout << "\n  Format: " << format_name << std::endl;

      bool format_supported = false;
      std::vector<std::string> supported_resolutions;

      // Test different resolutions for this format
      for (const auto &[resolution, res_name] : test_resolutions) {
        // Set the format and resolution
        stream_config.pixelFormat = format;
        stream_config.size = resolution;

        // Validate the configuration
        CameraConfiguration::Status validation = test_config->validate();

        if (validation != CameraConfiguration::Invalid) {
          // Check if the format/resolution was accepted or adjusted
          if (stream_config.pixelFormat == format) {
            format_supported = true;
            if (stream_config.size == resolution) {
              supported_resolutions.push_back(res_name);
            } else {
              // Resolution was adjusted
              std::string adjusted = std::to_string(stream_config.size.width) + "x" +
                                     std::to_string(stream_config.size.height) + " (adjusted from " + res_name + ")";
              supported_resolutions.push_back(adjusted);
            }
          }
        }
      }

      if (format_supported) {
        std::cout << "    Supported resolutions:" << std::endl;
        for (const auto &res : supported_resolutions) {
          std::cout << "      - " << res << std::endl;
        }
      } else {
        std::cout << "    Format not supported" << std::endl;
      }
    }
  }

  // Also show the raw sensor capabilities if available
  std::cout << "\n--- Raw Sensor Information ---" << std::endl;

  // Get camera properties (not controls)
  const ControlList &properties = m_camera->properties();

  // Try to get sensor resolution if available
  if (properties.contains(properties::PixelArraySize.id())) {
    const auto &array_size = properties.get(properties::PixelArraySize);
    std::cout << "Sensor pixel array size: " << array_size.value().width << "x" << array_size.value().height
              << std::endl;
  }

  // Show camera model if available
  if (properties.contains(properties::Model.id())) {
    const auto &model = properties.get(properties::Model);
    std::cout << "Camera model: " << model.value() << std::endl;
  }

  // Get camera controls for frame duration limits
  const ControlList &controls = m_camera->controls();

  // Show frame duration limits if available
  if (controls.contains(controls::FrameDurationLimits.id())) {
    const auto &limits = controls.get(controls::FrameDurationLimits);
    auto span = limits.value();
    if (span.size() >= 2) {
      double min_fps = 1000000.0 / span[1];  // Convert from microseconds
      double max_fps = 1000000.0 / span[0];
      std::cout << "Frame rate range: " << min_fps << " - " << max_fps << " fps" << std::endl;
    }
  }

  std::cout << "\n========================================\n" << std::endl;
}

bool RaspberrySensor::SetFocusControl(bool auto_focus, float manual_focus_position)
{
  if (!m_camera) {
    std::cerr << "Camera not initialized. Call Initialize() first." << std::endl;
    return false;
  }

  m_auto_focus_enabled = auto_focus;
  m_manual_focus_position = manual_focus_position;

  const ControlInfoMap &supported_controls = m_camera->controls();

  if (auto_focus) {
    std::cout << "Setting autofocus mode" << std::endl;

    // Set autofocus mode if supported
    if (supported_controls.find(&controls::AfMode) != supported_controls.end()) {
      m_current_controls.set(controls::AfMode, controls::AfModeAuto);
      std::cout << "  Autofocus mode set to Auto" << std::endl;
    } else {
      std::cerr << "  Warning: AfMode control not supported" << std::endl;
      return false;
    }

    // Set autofocus range (optional)
    if (supported_controls.find(&controls::AfRange) != supported_controls.end()) {
      m_current_controls.set(controls::AfRange, controls::AfRangeNormal);
      std::cout << "  Autofocus range set to Normal" << std::endl;
    }

    // Set autofocus speed (optional)
    if (supported_controls.find(&controls::AfSpeed) != supported_controls.end()) {
      m_current_controls.set(controls::AfSpeed, controls::AfSpeedNormal);
      std::cout << "  Autofocus speed set to Normal" << std::endl;
    }

  } else {
    std::cout << "Setting manual focus" << std::endl;

    // Set manual focus mode
    if (supported_controls.find(&controls::AfMode) != supported_controls.end()) {
      m_current_controls.set(controls::AfMode, controls::AfModeManual);
      std::cout << "  Focus mode set to Manual" << std::endl;
    }

    // Set lens position for manual focus
    if (supported_controls.find(&controls::LensPosition) != supported_controls.end()) {
      const ControlInfo &lens_info = supported_controls.at(&controls::LensPosition);
      float min_pos = lens_info.min().get<float>();
      float max_pos = lens_info.max().get<float>();

      // Clamp position to supported range
      float clamped_position = std::max(min_pos, std::min(max_pos, manual_focus_position));

      m_current_controls.set(controls::LensPosition, clamped_position);
      std::cout << "  Lens position set to: " << clamped_position << " (range: " << min_pos << "-" << max_pos << ")"
                << std::endl;
    } else {
      std::cerr << "  Warning: LensPosition control not supported" << std::endl;
    }
  }

  return true;
}

bool RaspberrySensor::TriggerAutofocus()
{
  if (!m_camera) {
    std::cerr << "Camera not initialized" << std::endl;
    return false;
  }

  const ControlInfoMap &supported_controls = m_camera->controls();

  // Trigger autofocus if supported
  if (supported_controls.find(&controls::AfTrigger) != supported_controls.end()) {
    ControlList trigger_controls;
    trigger_controls.set(controls::AfTrigger, controls::AfTriggerStart);

    // Apply the trigger through a temporary request
    std::unique_ptr<Request> trigger_request = m_camera->createRequest();
    if (trigger_request) {
      ApplyControlsToRequest(trigger_request.get(), trigger_controls);
      int ret = m_camera->queueRequest(trigger_request.get());
      if (ret == 0) {
        std::cout << "Autofocus triggered" << std::endl;
        return true;
      }
    }
  }

  std::cerr << "Failed to trigger autofocus - control not supported" << std::endl;
  return false;
}

bool RaspberrySensor::RefreshAutofocus()
{
  if (!m_is_running) {
    std::cerr << "Camera must be running to refresh autofocus" << std::endl;
    return false;
  }

  std::cout << "Refreshing autofocus..." << std::endl;

  // Force autofocus to be enabled
  m_auto_focus_enabled = true;
  SetFocusControl(true);

  // Trigger autofocus
  bool success = TriggerAutofocus();

  if (success) {
    // Wait for autofocus to settle
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    std::cout << "Autofocus refresh complete" << std::endl;
  }

  return success;
}

void RaspberrySensor::PrintFocusControls()
{
  if (!m_camera) {
    std::cerr << "Camera not initialized" << std::endl;
    return;
  }

  std::cout << "\n========== FOCUS CONTROLS ==========" << std::endl;

  const ControlInfoMap &controls = m_camera->controls();

  // Check focus-related controls
  std::vector<const ControlId *> focus_controls = {&controls::AfMode,    &controls::AfRange, &controls::AfSpeed,
                                                   &controls::AfTrigger, &controls::AfState, &controls::LensPosition};

  for (const auto *control_id : focus_controls) {
    auto it = controls.find(control_id);
    if (it != controls.end()) {
      const ControlInfo &info = it->second;
      std::cout << "âœ“ " << control_id->name();
      if (!info.values().empty()) {
        std::cout << " (range: " << info.min().toString() << " - " << info.max().toString() << ")";
      }
      std::cout << std::endl;
    } else {
      std::cout << "âœ— " << control_id->name() << " (not supported)" << std::endl;
    }
  }

  std::cout << "===================================\n" << std::endl;
}