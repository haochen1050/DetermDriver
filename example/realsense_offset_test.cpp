/**
 * @file realsense_offset_test.cpp
 * @brief Simple test to measure offset between RealSense global time and system time
 * @author Lei Gao (lei.gao@determtech.com)
 * @version 1.0.0
 * @date 2025-07-03
 */

#include <librealsense2/rs.hpp>
#include <chrono>
#include <iostream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <thread>
#include <fstream>

class RealsenseOffsetTester {
public:
    RealsenseOffsetTester() {
        std::cout << "RealSense Global Time Offset Tester" << std::endl;
        std::cout << "===================================" << std::endl;
    }
    
    bool initialize() {
        try {
            // Get connected devices
            rs2::device_list devices = ctx_.query_devices();
            if (devices.size() == 0) {
                std::cerr << "No RealSense devices found" << std::endl;
                return false;
            }
            
            device_ = devices[0];
            
            std::cout << "Found device: " << device_.get_info(RS2_CAMERA_INFO_NAME) 
                      << " (S/N: " << device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << ")" << std::endl;
            
            // Configure pipeline for minimal streams to reduce overhead
            config_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
            
            // Enable hardware timestamps
            for (auto&& sensor : device_.query_sensors()) {
                if (sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) {
                    sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f);
                    std::cout << "Hardware global time enabled" << std::endl;
                }
            }
            
            return true;
            
        } catch (const rs2::error& e) {
            std::cerr << "RealSense error during initialization: " << e.what() << std::endl;
            return false;
        }
    }
    
    void runOffsetTest(int num_samples = 100, int interval_ms = 50) {
        std::cout << "\nStarting offset measurement..." << std::endl;
        std::cout << "Samples: " << num_samples << ", Interval: " << interval_ms << "ms" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        
        try {
            // Start pipeline
            pipeline_.start(config_);
            
            // Wait for frames to stabilize
            std::cout << "Stabilizing camera..." << std::endl;
            for (int i = 0; i < 10; ++i) {
                pipeline_.wait_for_frames();
            }
            
            std::vector<OffsetMeasurement> measurements;
            measurements.reserve(num_samples);
            
            // Collect measurements
            std::cout << "\nCollecting measurements..." << std::endl;
            std::cout << "Sample | System Time (us) | RealSense Time (us) | Offset (us) | Offset (ms)" << std::endl;
            std::cout << "-------|------------------|---------------------|-------------|------------" << std::endl;
            
            for (int i = 0; i < num_samples; ++i) {
                OffsetMeasurement measurement = takeMeasurement();
                measurements.push_back(measurement);
                
                // Print every 10th measurement or first/last few
                if (i < 5 || i >= num_samples - 5 || (i + 1) % 10 == 0) {
                    std::cout << std::setw(6) << (i + 1) << " | "
                              << std::setw(16) << measurement.system_time_us << " | "
                              << std::setw(19) << measurement.realsense_time_us << " | "
                              << std::setw(11) << measurement.offset_us << " | "
                              << std::setw(10) << std::fixed << std::setprecision(3) 
                              << (measurement.offset_us / 1000.0) << std::endl;
                } else if (i == 5 && num_samples > 15) {
                    std::cout << "  ...  |       ...        |        ...          |     ...     |    ..." << std::endl;
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
            }
            
            pipeline_.stop();
            
            // Analyze results
            analyzeResults(measurements);
            
            // Save to file
            saveResults(measurements, "realsense_offset_test.csv");
            
        } catch (const rs2::error& e) {
            std::cerr << "RealSense error during test: " << e.what() << std::endl;
        }
    }
    
private:
    struct OffsetMeasurement {
        uint64_t system_time_us;
        uint64_t realsense_time_us;
        int64_t offset_us;
        double timestamp_domain;
        
        OffsetMeasurement() : system_time_us(0), realsense_time_us(0), offset_us(0), timestamp_domain(0) {}
    };
    
    OffsetMeasurement takeMeasurement() {
        OffsetMeasurement measurement;

        // Get system time before waiting for frame
        auto before_system = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // Wait for frames
        rs2::frameset frames = pipeline_.wait_for_frames();

        // Get system time right after
        auto after_system = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // Explicitly get the depth frame
        rs2::frame depth = frames.get_depth_frame();

        // Get RealSense timestamp (ms → µs)
        double realsense_timestamp_ms = depth.get_timestamp();
        rs2_timestamp_domain domain = depth.get_frame_timestamp_domain();

        // Use midpoint between before/after as system reference
        // measurement.system_time_us = (before_system + after_system) / 2;
        measurement.system_time_us = /*before_system; */ after_system;

        measurement.realsense_time_us = static_cast<uint64_t>(realsense_timestamp_ms * 1000.0);
        measurement.offset_us = static_cast<int64_t>(measurement.realsense_time_us) -
                            static_cast<int64_t>(measurement.system_time_us);
        measurement.timestamp_domain = static_cast<double>(domain);

        return measurement;
    }

    
    void analyzeResults(const std::vector<OffsetMeasurement>& measurements) {
        if (measurements.empty()) {
            std::cout << "\nNo measurements to analyze!" << std::endl;
            return;
        }
        
        std::cout << "\n========== OFFSET ANALYSIS RESULTS ==========" << std::endl;
        
        // Check timestamp domain consistency
        rs2_timestamp_domain domain = static_cast<rs2_timestamp_domain>(measurements[0].timestamp_domain);
        std::cout << "Timestamp Domain: ";
        switch(domain) {
            case RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK:
                std::cout << "HARDWARE_CLOCK" << std::endl;
                break;
            case RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME:
                std::cout << "SYSTEM_TIME" << std::endl;
                break;
            case RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME:
                std::cout << "GLOBAL_TIME" << std::endl;
                break;
            default:
                std::cout << "UNKNOWN (" << domain << ")" << std::endl;
        }
        
        // Calculate statistics
        std::vector<int64_t> offsets;
        for (const auto& m : measurements) {
            offsets.push_back(m.offset_us);
        }
        
        std::sort(offsets.begin(), offsets.end());
        
        int64_t min_offset = offsets.front();
        int64_t max_offset = offsets.back();
        int64_t median_offset = offsets[offsets.size() / 2];
        
        // Calculate mean
        int64_t sum = 0;
        for (int64_t offset : offsets) {
            sum += offset;
        }
        double mean_offset = static_cast<double>(sum) / offsets.size();
        
        // Calculate standard deviation
        double variance = 0;
        for (int64_t offset : offsets) {
            double diff = offset - mean_offset;
            variance += diff * diff;
        }
        double std_dev = std::sqrt(variance / offsets.size());
        
        // Calculate drift (change over time)
        int64_t first_offset = measurements.front().offset_us;
        int64_t last_offset = measurements.back().offset_us;
        int64_t total_drift = last_offset - first_offset;
        
        // Print results
        std::cout << "\nOffset Statistics (microseconds):" << std::endl;
        std::cout << "  Mean:     " << std::setw(10) << std::fixed << std::setprecision(1) << mean_offset << " us (" 
                  << std::setprecision(3) << (mean_offset / 1000.0) << " ms)" << std::endl;
        std::cout << "  Median:   " << std::setw(10) << median_offset << " us (" 
                  << std::setprecision(3) << (median_offset / 1000.0) << " ms)" << std::endl;
        std::cout << "  Min:      " << std::setw(10) << min_offset << " us (" 
                  << std::setprecision(3) << (min_offset / 1000.0) << " ms)" << std::endl;
        std::cout << "  Max:      " << std::setw(10) << max_offset << " us (" 
                  << std::setprecision(3) << (max_offset / 1000.0) << " ms)" << std::endl;
        std::cout << "  Range:    " << std::setw(10) << (max_offset - min_offset) << " us (" 
                  << std::setprecision(3) << ((max_offset - min_offset) / 1000.0) << " ms)" << std::endl;
        std::cout << "  Std Dev:  " << std::setw(10) << std::setprecision(1) << std_dev << " us (" 
                  << std::setprecision(3) << (std_dev / 1000.0) << " ms)" << std::endl;
        std::cout << "  Drift:    " << std::setw(10) << total_drift << " us (" 
                  << std::setprecision(3) << (total_drift / 1000.0) << " ms)" << std::endl;
        
        // Recommendations
        std::cout << "\nRecommendations:" << std::endl;
        if (domain != RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME) {
            std::cout << "  ⚠️  WARNING: Not using GLOBAL_TIME domain! Synchronization will be poor." << std::endl;
        }
        
        double range_ms = (max_offset - min_offset) / 1000.0;
        if (range_ms > 50.0) {
            std::cout << "  ⚠️  Large offset variation (" << std::setprecision(1) << range_ms 
                      << "ms). Consider increasing sync tolerance." << std::endl;
        }
        
        double abs_mean_ms = std::abs(mean_offset) / 1000.0;
        if (abs_mean_ms > 10.0) {
            std::cout << "  📝 Large mean offset (" << std::setprecision(1) << abs_mean_ms 
                      << "ms). Implement offset compensation." << std::endl;
        }
        
        std::cout << "  📊 Suggested sync tolerance: " << std::setprecision(1) << (range_ms + 10.0) << "ms" << std::endl;
        std::cout << "  🔧 Suggested offset compensation: " << std::setprecision(1) << (mean_offset / 1000.0) << "ms" << std::endl;
        
        std::cout << "=============================================" << std::endl;
    }
    
    void saveResults(const std::vector<OffsetMeasurement>& measurements, const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open " << filename << " for writing" << std::endl;
            return;
        }
        
        file << "Sample,SystemTime_us,RealsenseTime_us,Offset_us,Offset_ms,Domain" << std::endl;
        
        for (size_t i = 0; i < measurements.size(); ++i) {
            const auto& m = measurements[i];
            file << (i + 1) << ","
                 << m.system_time_us << ","
                 << m.realsense_time_us << ","
                 << m.offset_us << ","
                 << std::fixed << std::setprecision(3) << (m.offset_us / 1000.0) << ","
                 << static_cast<int>(m.timestamp_domain) << std::endl;
        }
        
        file.close();
        std::cout << "\nResults saved to: " << filename << std::endl;
    }
    
private:
    rs2::context ctx_;
    rs2::device device_;
    rs2::pipeline pipeline_;
    rs2::config config_;
};

int main(int argc, char* argv[]) {
    int num_samples = 100;
    int interval_ms = 50;
    
    // Parse command line arguments
    if (argc > 1) {
        num_samples = std::atoi(argv[1]);
        if (num_samples <= 0) num_samples = 100;
    }
    
    if (argc > 2) {
        interval_ms = std::atoi(argv[2]);
        if (interval_ms <= 0) interval_ms = 50;
    }
    
    std::cout << "Usage: " << argv[0] << " [num_samples] [interval_ms]" << std::endl;
    std::cout << "Default: " << argv[0] << " 100 50" << std::endl;
    std::cout << "Current: " << argv[0] << " " << num_samples << " " << interval_ms << "\n" << std::endl;
    
    try {
        RealsenseOffsetTester tester;
        
        if (!tester.initialize()) {
            return -1;
        }
        
        tester.runOffsetTest(num_samples, interval_ms);
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}

/* 
CMakeLists.txt addition:

find_package(realsense2 REQUIRED)

add_executable(realsense_offset_test realsense_offset_test.cpp)
target_link_libraries(realsense_offset_test 
    ${realsense2_LIBRARY}
)
*/