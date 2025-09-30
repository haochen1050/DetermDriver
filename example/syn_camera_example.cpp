#include "sensor/synchronized_camera.h"
#include <iostream>
#include <iomanip>
#include <signal.h>

using namespace determ::sensor;

// Global application instance for signal handling
std::unique_ptr<SynchronizedCameraApp> g_app;

void signalHandler(int signal) {
    std::cout << "\n🛑 Received signal " << signal << ", shutting down..." << std::endl;
    if (g_app) {
        g_app->Stop();
    }
    exit(0);
}

/**
 * @brief Simple example demonstrating synchronized camera capture
 */
class SimpleSyncExample {
public:
    static int run() {
        std::cout << "🌈 === Simple Synchronized Camera Example ===" << std::endl;
        std::cout << "D421 Depth + Pi Camera → Synchronized Capture" << std::endl;
        std::cout << "=============================================" << std::endl;
        
        // Setup signal handling
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);
        
        try {
            // Create simple configuration
            SyncConfig config;
            config.enabled_streams = {"color", "depth"};  // Enable both streams
            
            // Color sensor configuration
            config.color.width = 640;
            config.color.height = 480;
            config.color.format = "RGB888";
            config.color.auto_exposure = true;
            config.color.auto_focus = true;
            
            // Depth sensor configuration
            config.depth.width = 640;
            config.depth.height = 480;
            config.depth.auto_exposure = true;
            
            // Synchronization configuration
            config.sync.tolerance_ms = 50.0;  // 50ms tolerance
            config.sync.buffer_size = 20;     // Smaller buffer
            config.sync.enable_stats = true;
            
            // Create application
            g_app = std::make_unique<SynchronizedCameraApp>(config);
            
            // Initialize
            std::cout << "\n🔧 Initializing cameras..." << std::endl;
            int ret = g_app->Initialize();
            if (ret != 0) {
                std::cerr << "❌ Failed to initialize: " << ret << std::endl;
                return ret;
            }
            
            // Start capture
            std::cout << "🎬 Starting synchronized capture..." << std::endl;
            ret = g_app->Start();
            if (ret != 0) {
                std::cerr << "❌ Failed to start: " << ret << std::endl;
                return ret;
            }
            
            std::cout << "✅ Capture started successfully!" << std::endl;
            std::cout << "\n📸 Capturing 50 synchronized frames..." << std::endl;
            
            // Capture frames
            int successful_captures = 0;
            for (int i = 0; i < 50; ++i) {
                SynchronizedFrameSet frame_set;
                
                if (g_app->captureFrame(frame_set, 1000)) {  // 1 second timeout
                    successful_captures++;
                    
                    std::cout << "\rFrame " << std::setw(2) << (i + 1) 
                              << "/50 | Sync: " << std::fixed << std::setprecision(1)
                              << frame_set.max_time_diff_ms << "ms"
                              << " | Streams: " << frame_set.frames.size()
                              << "          " << std::flush;
                    
                    // Optional: Save every 10th frame
                    if ((i + 1) % 10 == 0) {
                        saveFrame(frame_set, i + 1);
                    }
                } else {
                    std::cout << "\n⚠️  Failed to capture frame " << (i + 1) << std::endl;
                }
            }
            
            std::cout << "\n\n📊 Capture completed!" << std::endl;
            std::cout << "Successful captures: " << successful_captures << "/50" << std::endl;
            
            // Print statistics
            g_app->printStats(true);
            
            // Stop application
            g_app->Stop();
            g_app.reset();
            
            std::cout << "✅ Example completed successfully" << std::endl;
            return 0;
            
        } catch (const std::exception& e) {
            std::cerr << "❌ Error: " << e.what() << std::endl;
            if (g_app) {
                g_app->Stop();
                g_app.reset();
            }
            return -1;
        }
    }

private:
    static void saveFrame(const SynchronizedFrameSet& frame_set, int frame_number) {
        try {
            std::string base_filename = "frame_" + std::to_string(frame_number);
            
            // Save color frame if available
            if (frame_set.hasStream("color")) {
                const auto* color_frame = frame_set.getFrame("color");
                std::string filename = base_filename + "_color.ppm";
                saveColorFrame(*color_frame, filename);
            }
            
            // Save depth frame if available
            if (frame_set.hasStream("depth")) {
                const auto* depth_frame = frame_set.getFrame("depth");
                std::string filename = base_filename + "_depth.pgm";
                saveDepthFrame(*depth_frame, filename);
            }
            
            // Save metadata
            std::string meta_filename = base_filename + "_info.txt";
            saveMetadata(frame_set, meta_filename);
            
        } catch (const std::exception& e) {
            std::cerr << "⚠️  Failed to save frame " << frame_number << ": " << e.what() << std::endl;
        }
    }
    
    static void saveColorFrame(const TimestampedFrame& frame, const std::string& filename) {
        std::ofstream file(filename, std::ios::binary);
        if (!file) return;
        
        // Write PPM header
        file << "P6\n" << frame.width << " " << frame.height << "\n255\n";
        
        // Write pixel data
        file.write(reinterpret_cast<const char*>(frame.data.data()), frame.data.size());
    }
    
    static void saveDepthFrame(const TimestampedFrame& frame, const std::string& filename) {
        std::ofstream file(filename, std::ios::binary);
        if (!file) return;
        
        // Convert 16-bit depth to 8-bit for PGM
        file << "P5\n" << frame.width << " " << frame.height << "\n255\n";
        
        for (size_t i = 0; i < frame.data.size(); i += 2) {
            uint16_t depth_value = *reinterpret_cast<const uint16_t*>(&frame.data[i]);
            uint8_t scaled_value = static_cast<uint8_t>(std::min(255, depth_value / 20));  // Scale depth
            file.write(reinterpret_cast<const char*>(&scaled_value), 1);
        }
    }
    
    static void saveMetadata(const SynchronizedFrameSet& frame_set, const std::string& filename) {
        std::ofstream file(filename);
        if (!file) return;
        
        file << "Frame Set Information\n";
        file << "====================\n";
        file << "Sync ID: " << frame_set.sync_id << "\n";
        file << "Average Timestamp: " << std::fixed << std::setprecision(3) 
             << frame_set.avg_timestamp_ms << " ms\n";
        file << "Max Time Difference: " << frame_set.max_time_diff_ms << " ms\n";
        file << "Number of Streams: " << frame_set.frames.size() << "\n\n";
        
        for (const auto& [stream_name, frame] : frame_set.frames) {
            file << stream_name << ":\n";
            file << "  Timestamp: " << frame.getTimestampMs() << " ms\n";
            file << "  Frame Number: " << frame.frame_number << "\n";
            file << "  Size: " << frame.width << "x" << frame.height << "\n";
            file << "  Format: " << frame.format << "\n";
            file << "  Data Size: " << frame.data.size() << " bytes\n\n";
        }
    }
};

/**
 * @brief Continuous capture example
 */
class ContinuousCaptureExample {
public:
    static int run() {
        std::cout << "🔄 === Continuous Synchronized Capture ===" << std::endl;
        
        SyncConfig config;
        config.enabled_streams = {"color", "depth"};
        config.sync.tolerance_ms = 33.0;  // 33ms tolerance
        
        SynchronizedCameraApp app(config);
        
        if (app.Initialize() != 0 || app.Start() != 0) {
            return -1;
        }
        
        // Set up callback for continuous processing
        app.setFrameCallback([](const SynchronizedFrameSet& frame_set) {
            static int frame_count = 0;
            frame_count++;
            
            if (frame_count % 30 == 0) {  // Print every 30 frames
                std::cout << "\r📊 Frame " << std::setw(6) << frame_count 
                          << " | Sync: " << std::fixed << std::setprecision(1)
                          << frame_set.max_time_diff_ms << "ms"
                          << " | Streams: " << frame_set.frames.size()
                          << "          " << std::flush;
            }
        });
        
        std::cout << "🔄 Running continuous capture for 10 seconds..." << std::endl;
        std::cout << "Press Ctrl+C to stop early" << std::endl;
        
        // Run for 10 seconds
        std::this_thread::sleep_for(std::chrono::seconds(10));
        
        std::cout << "\n📊 Final statistics:" << std::endl;
        app.printStats(true);
        
        return 0;
    }
};

int main(int argc, char* argv[]) {
    try {
        if (argc > 1 && std::string(argv[1]) == "--continuous") {
            return ContinuousCaptureExample::run();
        } else {
            return SimpleSyncExample::run();
        }
    } catch (const std::exception& e) {
        std::cerr << "❌ Fatal error: " << e.what() << std::endl;
        return -1;
    }
}