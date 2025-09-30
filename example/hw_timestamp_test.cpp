/**
 * @file hw_timestamp_test.cpp
 * @brief Hardware timestamp testing utility for DS1307 RTC and RealSense integration
 */

#include "sensor/synchronized_camera.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <signal.h>
#include <thread>
#include <chrono>
#include <atomic>

using namespace determ::sensor;

std::atomic<bool> g_running{true};

void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    g_running = false;
}

int main() {
    std::cout << "Hardware Timestamp Test Utility" << std::endl;
    std::cout << "===============================" << std::endl;
    std::cout << "Testing DS1307 RTC and RealSense hardware timestamps" << std::endl;
    
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        // Create configuration for timestamp testing
        SyncConfig config;
        config.enabled_streams = {"color", "infrared1", "infrared2"};
        
        // Minimal configuration for testing
        config.color.width = 640;
        config.color.height = 480;
        config.color.format = "RGB888";
        config.depth.width = 640;
        config.depth.height = 480;
        
        // Tight sync settings for timestamp accuracy testing
        config.sync.tolerance_ms = 5.0;
        config.sync.hw_tolerance_ms = 1.0;
        config.sync.prefer_hardware_timestamps = true;
        config.sync.enable_stats = true;
        
        // Create application
        SynchronizedCameraApp app(config);
        
        // Initialize
        std::cout << "\nInitializing cameras..." << std::endl;
        if (app.Initialize() != 0) {
            std::cerr << "Failed to initialize cameras" << std::endl;
            return -1;
        }
        
        // Check hardware timestamp availability
        bool hw_available = app.isHardwareTimestampingAvailable();
        std::cout << "Hardware timestamping: " << (hw_available ? "Available" : "Not available") << std::endl;
        
        if (hw_available) {
            std::cout << "Syncing RTC with system time..." << std::endl;
            app.syncRTCWithSystemTime();
        }
        
        // Start capture
        if (app.Start() != 0) {
            std::cerr << "Failed to start capture" << std::endl;
            return -1;
        }
        
        std::cout << "Starting hardware timestamp test..." << std::endl;
        std::cout << "Collecting 100 synchronized frame sets..." << std::endl;
        
        // Test data collection
        std::vector<double> sync_precisions;
        std::vector<bool> hw_timestamp_flags;
        int total_frames = 0;
        int hw_frames = 0;
        
        // Create log file
        std::ofstream log_file("hw_timestamp_test.log");
        log_file << "Frame,Timestamp_Type,Sync_Precision_ms,Stream_Count" << std::endl;
        
        // Collect test data
        for (int i = 0; i < 100 && g_running; ++i) {
            SynchronizedFrameSet frame_set;
            
            if (app.captureFrame(frame_set, 2000)) {
                total_frames++;
                sync_precisions.push_back(frame_set.max_time_diff_ms);
                hw_timestamp_flags.push_back(frame_set.all_hardware_timestamped);
                
                if (frame_set.all_hardware_timestamped) {
                    hw_frames++;
                }
                
                // Log data
                log_file << i << "," 
                         << (frame_set.all_hardware_timestamped ? "HW" : "SW") << ","
                         << std::fixed << std::setprecision(3) << frame_set.max_time_diff_ms << ","
                         << frame_set.frames.size() << std::endl;
                
                // Print progress
                if ((i + 1) % 20 == 0) {
                    std::cout << "Progress: " << (i + 1) << "/100 frames captured" << std::endl;
                }
            } else {
                std::cout << "Timeout waiting for frame " << i << std::endl;
            }
        }
        
        log_file.close();
        
        // Stop capture
        app.Stop();
        
        // Analyze results
        std::cout << "\n=== Hardware Timestamp Test Results ===" << std::endl;
        std::cout << "Total frames captured: " << total_frames << std::endl;
        std::cout << "Hardware timestamped frames: " << hw_frames << std::endl;
        std::cout << "Hardware timestamp usage: " << std::fixed << std::setprecision(1) 
                  << (100.0 * hw_frames / total_frames) << "%" << std::endl;
        
        if (!sync_precisions.empty()) {
            // Calculate statistics
            double sum = 0.0, hw_sum = 0.0, sw_sum = 0.0;
            double min_val = sync_precisions[0], max_val = sync_precisions[0];
            double hw_min = 999.0, hw_max = 0.0, sw_min = 999.0, sw_max = 0.0;
            int hw_count = 0, sw_count = 0;
            
            for (size_t i = 0; i < sync_precisions.size(); ++i) {
                double val = sync_precisions[i];
                sum += val;
                min_val = std::min(min_val, val);
                max_val = std::max(max_val, val);
                
                if (hw_timestamp_flags[i]) {
                    hw_sum += val;
                    hw_count++;
                    hw_min = std::min(hw_min, val);
                    hw_max = std::max(hw_max, val);
                } else {
                    sw_sum += val;
                    sw_count++;
                    sw_min = std::min(sw_min, val);
                    sw_max = std::max(sw_max, val);
                }
            }
            
            double avg_precision = sum / sync_precisions.size();
            
            std::cout << "\nSynchronization Precision Analysis:" << std::endl;
            std::cout << "  Overall average: " << std::setprecision(3) << avg_precision << "ms" << std::endl;
            std::cout << "  Overall range: " << min_val << "ms - " << max_val << "ms" << std::endl;
            
            if (hw_count > 0) {
                double hw_avg = hw_sum / hw_count;
                std::cout << "  Hardware timestamp average: " << hw_avg << "ms" << std::endl;
                std::cout << "  Hardware timestamp range: " << hw_min << "ms - " << hw_max << "ms" << std::endl;
            }
            
            if (sw_count > 0) {
                double sw_avg = sw_sum / sw_count;
                std::cout << "  Software timestamp average: " << sw_avg << "ms" << std::endl;
                std::cout << "  Software timestamp range: " << sw_min << "ms - " << sw_max << "ms" << std::endl;
            }
            
            // Improvement analysis
            if (hw_count > 0 && sw_count > 0) {
                double hw_avg = hw_sum / hw_count;
                double sw_avg = sw_sum / sw_count;
                double improvement = ((sw_avg - hw_avg) / sw_avg) * 100.0;
                
                std::cout << "\nHardware Timestamp Effectiveness:" << std::endl;
                std::cout << "  Precision improvement: " << std::setprecision(1) 
                          << improvement << "% better with hardware timestamps" << std::endl;
            }
        }
        
        // Print final application stats
        std::cout << "\nApplication Statistics:" << std::endl;
        app.printStats(true);
        
        std::cout << "\nTest log saved to: hw_timestamp_test.log" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }
    
    std::cout << "Hardware timestamp test completed successfully!" << std::endl;
    return 0;
}