/**
 * @file sync_benchmark.cpp
 * @brief Synchronization performance benchmark utility
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
    std::cout << "\nReceived signal " << signal << ", stopping benchmark..." << std::endl;
    g_running = false;
}

int main(int argc, char* argv[]) {
    std::cout << "Enhanced Camera Synchronization Benchmark" << std::endl;
    std::cout << "=========================================" << std::endl;
    
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    int benchmark_duration = 60; // Default 60 seconds
    if (argc > 1) {
        benchmark_duration = std::atoi(argv[1]);
    }
    
    try {
        // Create optimized configuration for benchmarking
        SyncConfig config;
        config.enabled_streams = {"color", "infrared1", "infrared2"};
        
        // Optimized settings for performance
        config.color.width = 640;
        config.color.height = 480;
        config.color.format = "RGB888";
        config.depth.width = 640;
        config.depth.height = 480;
        
        // Enhanced sync settings
        config.sync.tolerance_ms = 5.0;
        config.sync.hw_tolerance_ms = 1.0;
        config.sync.prefer_hardware_timestamps = true;
        config.sync.buffer_size = 20;
        config.sync.enable_stats = true;
        
        // Performance optimization
        config.performance.high_priority = true;
        config.performance.queue_size = 15;
        
        // Create application
        SynchronizedCameraApp app(config);
        
        std::cout << "Initializing cameras for benchmark..." << std::endl;
        if (app.Initialize() != 0) {
            std::cerr << "Failed to initialize cameras" << std::endl;
            return -1;
        }
        
        // Check hardware capabilities
        bool hw_available = app.isHardwareTimestampingAvailable();
        std::cout << "Hardware timestamping: " << (hw_available ? "Available" : "Not available") << std::endl;
        
        if (hw_available) {
            app.syncRTCWithSystemTime();
        }
        
        if (app.Start() != 0) {
            std::cerr << "Failed to start capture" << std::endl;
            return -1;
        }
        
        std::cout << "Starting " << benchmark_duration << " second benchmark..." << std::endl;
        
        // Benchmark variables
        auto start_time = std::chrono::high_resolution_clock::now();
        auto end_time = start_time + std::chrono::seconds(benchmark_duration);
        
        std::vector<double> sync_times;
        std::vector<double> frame_intervals;
        int total_captured = 0;
        int hw_frames = 0;
        int timeouts = 0;
        
        uint64_t last_timestamp = 0;
        
        // Benchmark loop
        while (g_running && std::chrono::high_resolution_clock::now() < end_time) {
            SynchronizedFrameSet frame_set;
            
            if (app.captureFrame(frame_set, 100)) {
                total_captured++;
                sync_times.push_back(frame_set.max_time_diff_ms);
                
                if (frame_set.all_hardware_timestamped) {
                    hw_frames++;
                }
                
                // Calculate frame interval
                if (last_timestamp > 0) {
                    double interval = (frame_set.reference_timestamp_us - last_timestamp) / 1000.0;
                    frame_intervals.push_back(interval);
                }
                last_timestamp = frame_set.reference_timestamp_us;
                
            } else {
                timeouts++;
            }
        }
        
        app.Stop();
        
        // Calculate benchmark results
        auto actual_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - start_time);
        double duration_sec = actual_duration.count() / 1000.0;
        
        std::cout << "\n=== Benchmark Results ===" << std::endl;
        std::cout << "Duration: " << std::fixed << std::setprecision(1) << duration_sec << " seconds" << std::endl;
        std::cout << "Total frames captured: " << total_captured << std::endl;
        std::cout << "Hardware timestamped frames: " << hw_frames << std::endl;
        std::cout << "Timeouts: " << timeouts << std::endl;
        std::cout << "Capture rate: " << std::setprecision(1) << (total_captured / duration_sec) << " FPS" << std::endl;
        std::cout << "Hardware timestamp usage: " << std::setprecision(1) 
                  << (100.0 * hw_frames / total_captured) << "%" << std::endl;
        
        if (!sync_times.empty()) {
            // Synchronization precision analysis
            double sum = 0.0, min_sync = sync_times[0], max_sync = sync_times[0];
            for (double t : sync_times) {
                sum += t;
                min_sync = std::min(min_sync, t);
                max_sync = std::max(max_sync, t);
            }
            double avg_sync = sum / sync_times.size();
            
            std::cout << "\nSynchronization Analysis:" << std::endl;
            std::cout << "  Average precision: " << std::setprecision(3) << avg_sync << "ms" << std::endl;
            std::cout << "  Best precision: " << min_sync << "ms" << std::endl;
            std::cout << "  Worst precision: " << max_sync << "ms" << std::endl;
        }
        
        if (!frame_intervals.empty()) {
            // Frame interval analysis
            double sum = 0.0;
            for (double t : frame_intervals) {
                sum += t;
            }
            double avg_interval = sum / frame_intervals.size();
            
            std::cout << "\nFrame Timing Analysis:" << std::endl;
            std::cout << "  Average frame interval: " << std::setprecision(1) << avg_interval << "ms" << std::endl;
            std::cout << "  Effective FPS: " << (1000.0 / avg_interval) << std::endl;
        }
        
        // Application-level statistics
        std::cout << "\nApplication Statistics:" << std::endl;
        app.printStats(true);
        
        // Save detailed results
        std::ofstream results("sync_benchmark_results.txt");
        results << "Enhanced Camera Synchronization Benchmark Results\n";
        results << "================================================\n";
        results << "Duration: " << duration_sec << " seconds\n";
        results << "Total frames: " << total_captured << "\n";
        results << "Hardware frames: " << hw_frames << "\n";
        results << "Capture rate: " << (total_captured / duration_sec) << " FPS\n";
        results << "Hardware usage: " << (100.0 * hw_frames / total_captured) << "%\n";
        
        if (!sync_times.empty()) {
            double sum = 0.0, min_sync = sync_times[0], max_sync = sync_times[0];
            for (double t : sync_times) {
                sum += t;
                min_sync = std::min(min_sync, t);
                max_sync = std::max(max_sync, t);
            }
            results << "Average sync precision: " << (sum / sync_times.size()) << "ms\n";
            results << "Best sync precision: " << min_sync << "ms\n";
            results << "Worst sync precision: " << max_sync << "ms\n";
        }
        
        results.close();
        std::cout << "\nDetailed results saved to: sync_benchmark_results.txt" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Benchmark exception: " << e.what() << std::endl;
        return -1;
    }
    
    std::cout << "Benchmark completed successfully!" << std::endl;
    return 0;
}

// Usage: ./sync_benchmark [duration_in_seconds]
// Example: ./sync_benchmark 120  # Run for 2 minutes