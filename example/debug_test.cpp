#include "sensor/determ_sensor.h"
#include <iostream>
#include <chrono>
#include <thread>

using namespace determ::sensor;

void testSensorInitialization() {
    std::cout << "\n=== Testing Sensor Initialization ===" << std::endl;
    
    DetermSensor sensor;
    
    // Test initialization
    std::cout << "Initializing sensor..." << std::endl;
    int ret = sensor.Initialize();
    if (ret != 0) {
        std::cerr << "FAILED: Sensor initialization returned: " << ret << std::endl;
        return;
    }
    std::cout << "SUCCESS: Sensor initialized" << std::endl;
    
    // Test stream enabling
    std::vector<std::string> streams = {"color", "depth", "infrared1", "infrared2"};
    std::cout << "Enabling streams..." << std::endl;
    ret = sensor.EnableStream(streams);
    if (ret != 0) {
        std::cerr << "FAILED: Stream enabling returned: " << ret << std::endl;
        return;
    }
    std::cout << "SUCCESS: Streams enabled" << std::endl;
    
    // Test configuration
    std::cout << "Configuring color sensor..." << std::endl;
    ret = sensor.SetColorSensorConfig(640, 480, "RGB888");
    if (ret != 0) {
        std::cerr << "FAILED: Color config returned: " << ret << std::endl;
        return;
    }
    std::cout << "SUCCESS: Color sensor configured" << std::endl;
    
    std::cout << "Configuring depth sensor..." << std::endl;
    ret = sensor.SetDepthSensorConfig(640, 480);
    if (ret != 0) {
        std::cerr << "FAILED: Depth config returned: " << ret << std::endl;
        return;
    }
    std::cout << "SUCCESS: Depth sensor configured" << std::endl;
    
    // Test hardware timestamp capability
    std::cout << "Testing hardware timestamp capability..." << std::endl;
    bool has_hw_timestamps = sensor.hasHardwareTimestamps();
    std::cout << "Hardware timestamps available: " << (has_hw_timestamps ? "YES" : "NO") << std::endl;
    
    // Test sensor start
    std::cout << "Starting sensor..." << std::endl;
    ret = sensor.Start();
    if (ret != 0) {
        std::cerr << "FAILED: Sensor start returned: " << ret << std::endl;
        return;
    }
    std::cout << "SUCCESS: Sensor started" << std::endl;
    
    // Wait for sensor to stabilize
    std::cout << "Waiting for sensor stabilization..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    std::cout << "Initialization test completed successfully!" << std::endl;
}

void testFrameCapture() {
    std::cout << "\n=== Testing Frame Capture ===" << std::endl;
    
    DetermSensor sensor;
    
    // Initialize and configure
    if (sensor.Initialize() != 0) {
        std::cerr << "FAILED: Could not initialize sensor" << std::endl;
        return;
    }
    
    std::vector<std::string> streams = {"color", "depth", "infrared1", "infrared2"};
    if (sensor.EnableStream(streams) != 0) {
        std::cerr << "FAILED: Could not enable streams" << std::endl;
        return;
    }
    
    if (sensor.SetColorSensorConfig(640, 480, "RGB888") != 0 ||
        sensor.SetDepthSensorConfig(640, 480) != 0) {
        std::cerr << "FAILED: Could not configure sensors" << std::endl;
        return;
    }
    
    if (sensor.Start() != 0) {
        std::cerr << "FAILED: Could not start sensor" << std::endl;
        return;
    }
    
    // Wait for stabilization
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Test frame capture
    std::cout << "Testing frame capture (10 attempts)..." << std::endl;
    
    for (int i = 0; i < 10; i++) {
        std::cout << "\nAttempt " << (i+1) << ":" << std::endl;
        
        auto start_time = std::chrono::high_resolution_clock::now();
        auto frame_data = sensor.GetFrame();
        auto end_time = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        std::cout << "  Capture took: " << duration.count() << "ms" << std::endl;
        std::cout << "  Streams returned: " << frame_data.size() << std::endl;
        
        if (frame_data.empty()) {
            std::cout << "  WARNING: No frame data returned" << std::endl;
        } else {
            for (const auto& [stream_name, data] : frame_data) {
                std::cout << "    " << stream_name << ": " << data.size() << " bytes" << std::endl;
            }
        }
        
        // Test hardware timestamps if available
        if (sensor.hasHardwareTimestamps()) {
            auto hw_timestamps = sensor.getLastHardwareTimestamps();
            std::cout << "  Hardware timestamps: " << hw_timestamps.size() << std::endl;
            for (const auto& [stream, timestamp] : hw_timestamps) {
                std::cout << "    " << stream << ": " << timestamp << "ms" << std::endl;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    sensor.Stop();
    std::cout << "Frame capture test completed!" << std::endl;
}

void testTimestampAccuracy() {
    std::cout << "\n=== Testing Timestamp Accuracy ===" << std::endl;
    
    DetermSensor sensor;
    
    // Quick setup
    if (sensor.Initialize() != 0 || 
        sensor.EnableStream({"color", "depth"}) != 0 ||
        sensor.SetColorSensorConfig(640, 480, "RGB888") != 0 ||
        sensor.SetDepthSensorConfig(640, 480) != 0 ||
        sensor.Start() != 0) {
        std::cerr << "FAILED: Setup failed" << std::endl;
        return;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    std::cout << "Capturing frames for timestamp analysis..." << std::endl;
    
    for (int i = 0; i < 5; i++) {
        auto software_timestamp = std::chrono::high_resolution_clock::now();
        auto frame_data = sensor.GetFrame();
        
        if (!frame_data.empty()) {
            std::cout << "\nFrame set " << (i+1) << ":" << std::endl;
            
            auto sw_timestamp_ms = std::chrono::duration_cast<std::chrono::microseconds>(
                software_timestamp.time_since_epoch()).count() / 1000.0;
            
            std::cout << "  Software timestamp: " << std::fixed << std::setprecision(3) << sw_timestamp_ms << "ms" << std::endl;
            
            if (sensor.hasHardwareTimestamps()) {
                auto hw_timestamps = sensor.getLastHardwareTimestamps();
                for (const auto& [stream, hw_ts] : hw_timestamps) {
                    double diff = std::abs(hw_ts - sw_timestamp_ms);
                    std::cout << "  " << stream << " HW timestamp: " << hw_ts << "ms (diff: " << diff << "ms)" << std::endl;
                }
            }
        } else {
            std::cout << "Frame " << (i+1) << ": No data captured" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    
    sensor.Stop();
    std::cout << "Timestamp accuracy test completed!" << std::endl;
}

void testStreamFormats() {
    std::cout << "\n=== Testing Stream Formats ===" << std::endl;
    
    DetermSensor sensor;
    
    if (sensor.Initialize() != 0) {
        std::cerr << "FAILED: Could not initialize" << std::endl;
        return;
    }
    
    // Print supported formats
    std::cout << "Supported formats:" << std::endl;
    sensor.PrintSupportedFormats();
    
    // Test individual streams
    std::vector<std::string> test_streams = {"color", "depth", "infrared1", "infrared2"};
    
    for (const auto& stream : test_streams) {
        std::cout << "\nTesting stream: " << stream << std::endl;
        
        if (sensor.EnableStream({stream}) != 0) {
            std::cout << "  FAILED: Could not enable " << stream << std::endl;
            continue;
        }
        
        if (stream == "color") {
            if (sensor.SetColorSensorConfig(640, 480, "RGB888") != 0) {
                std::cout << "  FAILED: Could not configure color" << std::endl;
                continue;
            }
        } else {
            if (sensor.SetDepthSensorConfig(640, 480) != 0) {
                std::cout << "  FAILED: Could not configure depth" << std::endl;
                continue;
            }
        }
        
        if (sensor.Start() != 0) {
            std::cout << "  FAILED: Could not start for " << stream << std::endl;
            sensor.Stop();
            continue;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        auto frame_data = sensor.GetFrame();
        if (frame_data.find(stream) != frame_data.end()) {
            std::cout << "  SUCCESS: " << stream << " captured " << frame_data[stream].size() << " bytes" << std::endl;
        } else {
            std::cout << "  FAILED: " << stream << " not in captured data" << std::endl;
        }
        
        sensor.Stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

int main() {
    std::cout << "=== DetermSensor Debug Test Suite ===" << std::endl;
    
    try {
        // Run tests in sequence
        testSensorInitialization();
        testFrameCapture();
        testTimestampAccuracy();
        testStreamFormats();
        
        std::cout << "\n=== All Tests Completed ===" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "ERROR: Test failed with exception: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}