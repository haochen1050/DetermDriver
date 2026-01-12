
/**
 * @file realsense_d421_sensor.h
 * @author Hao Chen (hao.chen@determtech.com)
 * RealSense D421 Sensor Driver for Determ Sensor Set
 * @version 1.0.0
 * @date 2025-07-03
 *
 * @copyright Copyright (c) 2025 Determ Inc. All rights reserved
 *
 */
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
    std::map<std::string, std::vector<uint8_t>> GetFrame();  // Legacy: returns frame data only
    rs2::frameset GetFrameset();                             // New: returns native RealSense frameset with timestamps
    
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
};

} // namespace sensor
} // namespace determ