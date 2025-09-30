#include "sensor/realsense_sensor.h"

using determ::sensor::RealsenseSensor;
using determ::sensor::RealsenseFrameSet;
// using determ::sensor::RealsenseTimestampedFrame;

RealsenseSensor::RealsenseSensor()
    : m_is_initialized(false), m_is_running(false), m_is_configured(false), m_width(640), m_height(480), m_fps(30)
{
  // Initialize stream mapping
  m_stream_mapping["depth"] = RS2_STREAM_DEPTH;
  m_stream_mapping["infrared1"] = RS2_STREAM_INFRARED;
  m_stream_mapping["infrared2"] = RS2_STREAM_INFRARED;
}

RealsenseSensor::~RealsenseSensor()
{
  // Ensure sensor is stopped first
  Stop();

  // Give system time to complete cleanup
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Pipeline cleanup is handled automatically by rs2::pipeline destructor
  std::cout << "RealSense sensor destroyed" << std::endl;
}

int RealsenseSensor::Initialize()
{
  try {
    std::cout << "Initializing RealSense sensor..." << std::endl;

    // Get connected devices
    rs2::device_list devices = m_ctx.query_devices();
    if (devices.size() == 0) {
      std::cerr << "No RealSense devices found" << std::endl;
      return -1;
    }

    // Use the first available device
    m_device = devices[0];

      // Enable hardware timestamps
    for (auto&& sensor : m_device.query_sensors()) {
        if (sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) {
            sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f);
            std::cout << "Hardware global time enabled" << std::endl;
        }
    }

    // Get device information
    m_device_name = m_device.get_info(RS2_CAMERA_INFO_NAME);
    m_serial_number = m_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    m_firmware_version = m_device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);

    std::cout << "Connected to RealSense device:" << std::endl;
    std::cout << "  Name: " << m_device_name << std::endl;
    std::cout << "  Serial: " << m_serial_number << std::endl;
    std::cout << "  Firmware: " << m_firmware_version << std::endl;

    m_is_initialized = true;
    std::cout << "RealSense sensor initialized successfully" << std::endl;
    return 0;

  } catch (const rs2::error& e) {
    std::cerr << "RealSense error during initialization: " << e.what() << std::endl;
    return -1;
  } catch (const std::exception& e) {
    std::cerr << "Standard error during initialization: " << e.what() << std::endl;
    return -1;
  }
}

rs2_stream RealsenseSensor::StringToStreamType(const std::string& stream_name)
{
  if (stream_name == "depth") {
    return RS2_STREAM_DEPTH;
  } else if (stream_name == "infrared1") {
    return RS2_STREAM_INFRARED;
  } else if (stream_name == "infrared2") {
    return RS2_STREAM_INFRARED;
  } else {
    std::cerr << "Unknown stream type: " << stream_name << ", defaulting to depth" << std::endl;
    return RS2_STREAM_DEPTH;
  }
}

std::string RealsenseSensor::StreamTypeToString(rs2_stream stream_type)
{
  switch (stream_type) {
    case RS2_STREAM_DEPTH:
      return "depth";
    case RS2_STREAM_INFRARED:
      return "infrared";
    default:
      return "unknown";
  }
}

size_t RealsenseSensor::GetFrameSize(rs2_stream stream_type, int width, int height)
{
  int bytes_per_pixel = GetBytesPerPixel(stream_type);
  return width * height * bytes_per_pixel;
}

int RealsenseSensor::GetBytesPerPixel(rs2_stream stream_type)
{
  switch (stream_type) {
    case RS2_STREAM_DEPTH:
      return 2;  // 16-bit depth data
    case RS2_STREAM_INFRARED:
      return 1;  // 8-bit infrared data
    default:
      return 1;
  }
}

bool RealsenseSensor::IsStreamSupported(rs2_stream stream_type, int width, int height, int fps)
{
  try {
    // Check if any sensor supports this stream configuration
    for (auto&& sensor : m_device.query_sensors()) {
      for (auto&& profile : sensor.get_stream_profiles()) {
        if (auto video_profile = profile.as<rs2::video_stream_profile>()) {
          if (video_profile.stream_type() == stream_type && video_profile.width() == width &&
              video_profile.height() == height && video_profile.fps() == fps) {
            return true;
          }
        }
      }
    }
    return false;
  } catch (const rs2::error& e) {
    return false;
  }
}

int RealsenseSensor::SetSensorConfig(int width, int height, const std::vector<std::string>& streams)
{
  if (m_is_running) {
    std::cerr << "Cannot change configuration while sensor is running" << std::endl;
    return -1;
  }

  if (!m_is_initialized) {
    std::cerr << "Sensor not initialized. Call Initialize() first." << std::endl;
    return -1;
  }

  std::cout << "Setting sensor configuration:" << std::endl;
  std::cout << "  Width: " << width << std::endl;
  std::cout << "  Height: " << height << std::endl;
  std::cout << "  Streams: ";
  for (const auto& stream : streams) {
    std::cout << stream << " ";
  }
  std::cout << std::endl;

  m_width = width;
  m_height = height;
  m_enabled_streams = streams;

  try {
    // Configure streams
    for (const auto& stream_name : streams) {
      if (stream_name == "depth") {
        if (!IsStreamSupported(RS2_STREAM_DEPTH, width, height, m_fps)) {
          std::cerr << "Depth stream " << width << "x" << height << "@" << m_fps << "fps not supported" << std::endl;
          return -1;
        }
        m_config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, m_fps);
        std::cout << "  Enabled depth stream: " << width << "x" << height << "@" << m_fps << "fps" << std::endl;

      } else if (stream_name == "infrared1") {
        if (!IsStreamSupported(RS2_STREAM_INFRARED, width, height, m_fps)) {
          std::cerr << "Infrared stream " << width << "x" << height << "@" << m_fps << "fps not supported" << std::endl;
          return -1;
        }
        m_config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, m_fps);
        std::cout << "  Enabled infrared1 stream: " << width << "x" << height << "@" << m_fps << "fps" << std::endl;

      } else if (stream_name == "infrared2") {
        if (!IsStreamSupported(RS2_STREAM_INFRARED, width, height, m_fps)) {
          std::cerr << "Infrared stream " << width << "x" << height << "@" << m_fps << "fps not supported" << std::endl;
          return -1;
        }
        m_config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, m_fps);
        std::cout << "  Enabled infrared2 stream: " << width << "x" << height << "@" << m_fps << "fps" << std::endl;

      } else {
        std::cerr << "Unsupported stream type: " << stream_name << std::endl;
        return -1;
      }
    }

    m_is_configured = true;
    std::cout << "Sensor configuration completed successfully" << std::endl;
    return 0;

  } catch (const rs2::error& e) {
    std::cerr << "RealSense error during configuration: " << e.what() << std::endl;
    return -1;
  }
}

int RealsenseSensor::Start()
{
  if (m_is_running) {
    std::cout << "Sensor is already running" << std::endl;
    return 0;
  }

  if (!m_is_configured) {
    std::cerr << "Sensor not configured. Call SetSensorConfig first." << std::endl;
    return -1;
  }

  try {
    std::cout << "Starting RealSense sensor..." << std::endl;

    // Start the pipeline with the configured streams
    m_pipeline.start(m_config);

    // Wait for frames to stabilize
    std::cout << "Waiting for sensor to stabilize..." << std::endl;
    for (int i = 0; i < 30; ++i) {
      rs2::frameset frames = m_pipeline.wait_for_frames();
    }

    m_is_running = true;
    std::cout << "RealSense sensor started successfully" << std::endl;
    return 0;

  } catch (const rs2::error& e) {
    std::cerr << "RealSense error during start: " << e.what() << std::endl;
    return -1;
  }
}

int RealsenseSensor::Stop()
{
  if (!m_is_running) {
    return 0;
  }

  try {
    std::cout << "Stopping RealSense sensor..." << std::endl;

    m_pipeline.stop();
    m_is_running = false;

    std::cout << "RealSense sensor stopped successfully" << std::endl;
    return 0;

  } catch (const rs2::error& e) {
    std::cerr << "RealSense error during stop: " << e.what() << std::endl;
    return -1;
  }
}

std::map<std::string, std::vector<uint8_t>> RealsenseSensor::GetFrame() {
    std::map<std::string, std::vector<uint8_t>> frame_data;
    
    if (!m_is_running) {
        std::cerr << "Sensor is not running. Call Start() first." << std::endl;
        return frame_data;
    }

    try {
        rs2::frameset frames = m_pipeline.wait_for_frames(5000);

        // Clear previous timestamps and capture new ones
        // m_last_hardware_timestamps.clear();

        for (const auto& stream_name : m_enabled_streams) {
            rs2::frame frame;

            if (stream_name == "depth") {
                frame = frames.get_depth_frame();
            } else if (stream_name == "infrared1") {
                frame = frames.get_infrared_frame(1);
            } else if (stream_name == "infrared2") {
                frame = frames.get_infrared_frame(2);
            }

            if (frame) {

                auto video_frame = frame.as<rs2::video_frame>();
                int width = video_frame.get_width();
                int height = video_frame.get_height();
                int bytes_per_pixel = video_frame.get_bytes_per_pixel();

                const uint8_t* frame_ptr = static_cast<const uint8_t*>(frame.get_data());
                size_t frame_size = width * height * bytes_per_pixel;

                std::vector<uint8_t> data(frame_size);
                std::memcpy(data.data(), frame_ptr, frame_size);

                frame_data[stream_name] = std::move(data);
            }
        }

        return frame_data;

    } catch (const rs2::error& e) {
        std::cerr << "RealSense error during frame capture: " << e.what() << std::endl;
        return frame_data;
    }
}

RealsenseFrameSet RealsenseSensor::GetTimestampedFrameSet()
{
    RealsenseFrameSet frameset;
    
    if (!m_is_running) {
        std::cerr << "Sensor is not running. Call Start() first." << std::endl;
        return frameset;
    }

    try {
        rs2::frameset frames = m_pipeline.wait_for_frames(5000);
        frameset.frameset_sequence = ++m_frameset_sequence;

        // m_last_hardware_timestamps.clear();

        for (const auto& stream_name : m_enabled_streams) {
            rs2::frame frame;

            if (stream_name == "depth") {
                frame = frames.get_depth_frame();
            } else if (stream_name == "infrared1") {
                frame = frames.get_infrared_frame(1);
            } else if (stream_name == "infrared2") {
                frame = frames.get_infrared_frame(2);
            }

            if (frame) {
                RealsenseTimestampedFrame timestamped_frame;
                
                // Get hardware timestamp (should be in global time domain)
                timestamped_frame.timestamp_ms = frame.get_timestamp();
                timestamped_frame.sequence_number = ++m_frame_sequence;
                timestamped_frame.stream_name = stream_name;

                // Store for backward compatibility
                // m_last_hardware_timestamps[stream_name] = timestamped_frame.timestamp_ms;

                // Extract frame data
                auto video_frame = frame.as<rs2::video_frame>();
                int width = video_frame.get_width();
                int height = video_frame.get_height();
                int bytes_per_pixel = video_frame.get_bytes_per_pixel();

                const uint8_t* frame_ptr = static_cast<const uint8_t*>(frame.get_data());
                size_t frame_size = width * height * bytes_per_pixel;

                timestamped_frame.data.resize(frame_size);
                std::memcpy(timestamped_frame.data.data(), frame_ptr, frame_size);

                frameset.frames[stream_name] = std::move(timestamped_frame);
            }
        }

        return frameset;

    } catch (const rs2::error& e) {
        std::cerr << "RealSense error during frame capture: " << e.what() << std::endl;
        return frameset;
    }
}

// // Add method to get last hardware timestamps
// std::map<std::string, double> RealsenseSensor::getLastHardwareTimestamps() const {
//     return m_last_hardware_timestamps;
// }

void RealsenseSensor::PrintSupportedFormats()
{
  if (!m_is_initialized) {
    std::cerr << "Sensor not initialized. Call Initialize() first." << std::endl;
    return;
  }

  std::cout << "\n========== REALSENSE SENSOR CAPABILITIES ==========" << std::endl;
  std::cout << "Device: " << GetDeviceInfo() << std::endl;

  try {
    // Common resolutions to test
    std::vector<std::pair<std::pair<int, int>, std::string>> test_resolutions = {{{424, 240}, "424x240"},
                                                                                 {{480, 270}, "480x270"},
                                                                                 {{640, 360}, "640x360"},
                                                                                 {{640, 480}, "640x480 (VGA)"},
                                                                                 {{848, 480}, "848x480"},
                                                                                 {{1280, 720}, "1280x720 (720p)"},
                                                                                 {{1920, 1080}, "1920x1080 (1080p)"}};

    // Test different stream types
    std::vector<std::pair<rs2_stream, std::string>> stream_types = {{RS2_STREAM_DEPTH, "Depth"},
                                                                    {RS2_STREAM_INFRARED, "Infrared"}};

    for (auto&& sensor : m_device.query_sensors()) {
      std::cout << "\n--- Sensor: " << sensor.get_info(RS2_CAMERA_INFO_NAME) << " ---" << std::endl;

      for (const auto& [stream_type, stream_name] : stream_types) {
        std::cout << "\n  Stream: " << stream_name << std::endl;

        bool stream_supported = false;
        std::vector<std::string> supported_resolutions;

        for (const auto& [resolution, res_name] : test_resolutions) {
          int width = resolution.first;
          int height = resolution.second;

          if (IsStreamSupported(stream_type, width, height, 30)) {
            stream_supported = true;
            supported_resolutions.push_back(res_name + " @30fps");
          } else if (IsStreamSupported(stream_type, width, height, 15)) {
            stream_supported = true;
            supported_resolutions.push_back(res_name + " @15fps");
          } else if (IsStreamSupported(stream_type, width, height, 6)) {
            stream_supported = true;
            supported_resolutions.push_back(res_name + " @6fps");
          }
        }

        if (stream_supported) {
          std::cout << "    Supported configurations:" << std::endl;
          for (const auto& res : supported_resolutions) {
            std::cout << "      - " << res << std::endl;
          }
        } else {
          std::cout << "    Stream not supported" << std::endl;
        }
      }
    }

  } catch (const rs2::error& e) {
    std::cerr << "Error querying capabilities: " << e.what() << std::endl;
  }

  std::cout << "\n================================================\n" << std::endl;
}

void RealsenseSensor::PrintAvailableControls()
{
  if (!m_is_initialized) {
    std::cerr << "Sensor not initialized. Call Initialize() first." << std::endl;
    return;
  }

  std::cout << "\n========== REALSENSE SENSOR CONTROLS ==========" << std::endl;

  try {
    for (auto&& sensor : m_device.query_sensors()) {
      std::cout << "\n--- Sensor: " << sensor.get_info(RS2_CAMERA_INFO_NAME) << " ---" << std::endl;

      // List of important controls to check
      std::vector<rs2_option> important_options = {
          RS2_OPTION_EXPOSURE,           RS2_OPTION_GAIN,        RS2_OPTION_ENABLE_AUTO_EXPOSURE,
          RS2_OPTION_LASER_POWER,        RS2_OPTION_DEPTH_UNITS, RS2_OPTION_EMITTER_ENABLED};

      for (rs2_option option : important_options) {
        if (sensor.supports(option)) {
          try {
            rs2::option_range range = sensor.get_option_range(option);
            float current_value = sensor.get_option(option);

            std::cout << "✓ " << rs2_option_to_string(option) << " (current: " << current_value
                      << ", range: " << range.min << "-" << range.max << ", step: " << range.step << ")" << std::endl;
          } catch (const rs2::error& e) {
            std::cout << "✓ " << rs2_option_to_string(option) << " (supported but error reading: " << e.what() << ")"
                      << std::endl;
          }
        } else {
          std::cout << "✗ " << rs2_option_to_string(option) << " (not supported)" << std::endl;
        }
      }
    }

  } catch (const rs2::error& e) {
    std::cerr << "Error querying controls: " << e.what() << std::endl;
  }

  std::cout << "\n=============================================\n" << std::endl;
}

bool RealsenseSensor::SetDepthControl(bool auto_exposure, float manual_exposure, float gain)
{
  if (!m_is_initialized) {
    std::cerr << "Sensor not initialized. Call Initialize() first." << std::endl;
    return false;
  }

  try {
    // Find the depth sensor
    for (auto&& sensor : m_device.query_sensors()) {
      if (sensor.supports(RS2_OPTION_DEPTH_UNITS)) {  // This indicates depth sensor
        std::cout << "Setting depth sensor controls:" << std::endl;

        if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
          sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, auto_exposure ? 1.0f : 0.0f);
          std::cout << "  Auto exposure: " << (auto_exposure ? "enabled" : "disabled") << std::endl;
        }

        if (!auto_exposure) {
          if (sensor.supports(RS2_OPTION_EXPOSURE)) {
            rs2::option_range exp_range = sensor.get_option_range(RS2_OPTION_EXPOSURE);
            float clamped_exposure = std::max(exp_range.min, std::min(exp_range.max, manual_exposure));
            sensor.set_option(RS2_OPTION_EXPOSURE, clamped_exposure);
            std::cout << "  Manual exposure: " << clamped_exposure << " μs" << std::endl;
          }

          if (sensor.supports(RS2_OPTION_GAIN)) {
            rs2::option_range gain_range = sensor.get_option_range(RS2_OPTION_GAIN);
            float clamped_gain = std::max(gain_range.min, std::min(gain_range.max, gain));
            sensor.set_option(RS2_OPTION_GAIN, clamped_gain);
            std::cout << "  Gain: " << clamped_gain << std::endl;
          }
        }

        return true;
      }
    }

    std::cerr << "Depth sensor not found" << std::endl;
    return false;

  } catch (const rs2::error& e) {
    std::cerr << "Error setting depth controls: " << e.what() << std::endl;
    return false;
  }
}

bool RealsenseSensor::SetInfraredControl(bool auto_exposure, float manual_exposure, float gain)
{
  if (!m_is_initialized) {
    std::cerr << "Sensor not initialized. Call Initialize() first." << std::endl;
    return false;
  }

  try {
    // Find the infrared sensor
    for (auto&& sensor : m_device.query_sensors()) {
      if (sensor.supports(RS2_OPTION_EXPOSURE) && !sensor.supports(RS2_OPTION_DEPTH_UNITS)) {
        std::cout << "Setting infrared sensor controls:" << std::endl;

        if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
          sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, auto_exposure ? 1.0f : 0.0f);
          std::cout << "  Auto exposure: " << (auto_exposure ? "enabled" : "disabled") << std::endl;
        }

        if (!auto_exposure) {
          if (sensor.supports(RS2_OPTION_EXPOSURE)) {
            rs2::option_range exp_range = sensor.get_option_range(RS2_OPTION_EXPOSURE);
            float clamped_exposure = std::max(exp_range.min, std::min(exp_range.max, manual_exposure));
            sensor.set_option(RS2_OPTION_EXPOSURE, clamped_exposure);
            std::cout << "  Manual exposure: " << clamped_exposure << " μs" << std::endl;
          }

          if (sensor.supports(RS2_OPTION_GAIN)) {
            rs2::option_range gain_range = sensor.get_option_range(RS2_OPTION_GAIN);
            float clamped_gain = std::max(gain_range.min, std::min(gain_range.max, gain));
            sensor.set_option(RS2_OPTION_GAIN, clamped_gain);
            std::cout << "  Gain: " << clamped_gain << std::endl;
          }
        }

        return true;
      }
    }

    std::cerr << "Infrared sensor not found" << std::endl;
    return false;

  } catch (const rs2::error& e) {
    std::cerr << "Error setting infrared controls: " << e.what() << std::endl;
    return false;
  }
}

bool RealsenseSensor::SetLaserPower(float power)
{
  if (!m_is_initialized) {
    std::cerr << "Sensor not initialized. Call Initialize() first." << std::endl;
    return false;
  }

  try {
    for (auto&& sensor : m_device.query_sensors()) {
      if (sensor.supports(RS2_OPTION_LASER_POWER)) {
        rs2::option_range power_range = sensor.get_option_range(RS2_OPTION_LASER_POWER);
        float clamped_power = std::max(power_range.min, std::min(power_range.max, power));

        sensor.set_option(RS2_OPTION_LASER_POWER, clamped_power);
        std::cout << "Laser power set to: " << clamped_power << " mW (range: " << power_range.min << "-"
                  << power_range.max << ")" << std::endl;
        return true;
      }
    }

    std::cerr << "Laser power control not supported" << std::endl;
    return false;

  } catch (const rs2::error& e) {
    std::cerr << "Error setting laser power: " << e.what() << std::endl;
    return false;
  }
}

bool RealsenseSensor::SetDepthUnits(float units)
{
  if (!m_is_initialized) {
    std::cerr << "Sensor not initialized. Call Initialize() first." << std::endl;
    return false;
  }

  try {
    for (auto&& sensor : m_device.query_sensors()) {
      if (sensor.supports(RS2_OPTION_DEPTH_UNITS)) {
        rs2::option_range units_range = sensor.get_option_range(RS2_OPTION_DEPTH_UNITS);
        float clamped_units = std::max(units_range.min, std::min(units_range.max, units));

        sensor.set_option(RS2_OPTION_DEPTH_UNITS, clamped_units);
        std::cout << "Depth units set to: " << clamped_units << " meters/unit (range: " << units_range.min << "-"
                  << units_range.max << ")" << std::endl;
        return true;
      }
    }

    std::cerr << "Depth units control not supported" << std::endl;
    return false;

  } catch (const rs2::error& e) {
    std::cerr << "Error setting depth units: " << e.what() << std::endl;
    return false;
  }
}

std::string RealsenseSensor::GetDeviceInfo() const
{
  if (!m_is_initialized) {
    return "Device not initialized";
  }

  return m_device_name + " (S/N: " + m_serial_number + ", FW: " + m_firmware_version + ")";
}