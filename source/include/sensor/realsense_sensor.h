// Add these to your realsense_sensor.h file

#pragma once

#include <librealsense2/rs.hpp>
#include <vector>
#include <string>
#include <map>
#include <chrono>
#include <memory>
#include <iostream>
#include <thread>

namespace determ {
namespace sensor {

/**
 * @brief Timestamped frame from RealSense with hardware global time
 */
struct RealsenseTimestampedFrame {
    std::vector<uint8_t> data;
    double timestamp_ms;          // Hardware global time timestamp
    uint64_t sequence_number;
    std::string stream_name;
    
    uint64_t getTimestampUs() const {
        return static_cast<uint64_t>(timestamp_ms * 1000.0);
    }
    
    double getTimestampMs() const {
        return timestamp_ms;
    }
    
    // Convert to system time point for synchronization
    std::chrono::high_resolution_clock::time_point getTimestampAsTimePoint() const {
        auto duration_ms = std::chrono::milliseconds(static_cast<long long>(timestamp_ms));
        return std::chrono::high_resolution_clock::time_point(duration_ms);
    }
};

/**
 * @brief Set of synchronized frames from RealSense
 */
struct RealsenseFrameSet {
    std::map<std::string, RealsenseTimestampedFrame> frames;
    uint64_t frameset_sequence;
    
    std::chrono::high_resolution_clock::time_point getPrimaryTimestamp() const {
        if (!frames.empty()) {
            return frames.begin()->second.getTimestampAsTimePoint();
        }
        return std::chrono::high_resolution_clock::now();
    }
    
    double getPrimaryTimestampMs() const {
        if (!frames.empty()) {
            return frames.begin()->second.getTimestampMs();
        }
        return 0.0;
    }
};

class RealsenseSensor {
public:
    RealsenseSensor();
    ~RealsenseSensor();

    // Initialization and configuration
    int Initialize();
    int SetSensorConfig(int width, int height, const std::vector<std::string>& streams);
    
    // Control methods
    int Start();
    int Stop();
    
    // Frame capture methods
    std::map<std::string, std::vector<uint8_t>> GetFrame();  // Original method
    RealsenseFrameSet GetTimestampedFrameSet();              // New timestamped method
    
    // // Hardware timestamp access
    // std::map<std::string, double> getLastHardwareTimestamps() const;
    
    // Sensor controls
    bool SetDepthControl(bool auto_exposure, float manual_exposure = 8500.0f, float gain = 16.0f);
    bool SetInfraredControl(bool auto_exposure, float manual_exposure = 8500.0f, float gain = 16.0f);
    bool SetLaserPower(float power);  // 0.0 to 360.0 (mW)
    bool SetDepthUnits(float units);  // meters per unit (default 0.001 = 1mm)
    
    // Information methods
    void PrintSupportedFormats();
    void PrintAvailableControls();
    std::pair<int, int> GetResolution() const { return {m_width, m_height}; }
    std::vector<std::string> GetEnabledStreams() const { return m_enabled_streams; }
    std::string GetDeviceInfo() const;
    
    // Stream utilities
    bool IsStreamSupported(rs2_stream stream_type, int width, int height, int fps);
    
private:
    // Helper methods
    rs2_stream StringToStreamType(const std::string& stream_name);
    std::string StreamTypeToString(rs2_stream stream_type);
    size_t GetFrameSize(rs2_stream stream_type, int width, int height);
    int GetBytesPerPixel(rs2_stream stream_type);
    
    // Member variables
    rs2::context m_ctx;
    rs2::device m_device;
    rs2::pipeline m_pipeline;
    rs2::config m_config;
    
    bool m_is_initialized;
    bool m_is_running;
    bool m_is_configured;
    
    int m_width, m_height, m_fps;
    std::vector<std::string> m_enabled_streams;
    std::map<std::string, rs2_stream> m_stream_mapping;
    
    // Device information
    std::string m_device_name;
    std::string m_serial_number;
    std::string m_firmware_version;
    
    // Timestamp tracking
    // std::map<std::string, double> m_last_hardware_timestamps;
    uint64_t m_frame_sequence = 0;
    uint64_t m_frameset_sequence = 0;
};

} // namespace sensor
} // namespace determ