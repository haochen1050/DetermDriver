// simple_sync_visualizer.cpp
// Simple synchronized visualization of color and depth streams using DetermSensor

#include "sensor/determ_sensor.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>

using namespace determ::sensor;

class SyncVisualizer {
public:
    SyncVisualizer() : 
        display_width_(640), 
        display_height_(480),
        stats_panel_height_(120),
        frame_count_(0) {
        
        // Create display window
        cv::namedWindow("Synchronized Color + Depth", cv::WINDOW_AUTOSIZE);
        
        // Initialize text properties
        font_face_ = cv::FONT_HERSHEY_SIMPLEX;
        font_scale_ = 0.6;
        font_thickness_ = 1;
        
        std::cout << "SyncVisualizer initialized" << std::endl;
        std::cout << "Controls: [Q]uit | [S]ave screenshot | [R]eset stats" << std::endl;
    }
    
    ~SyncVisualizer() {
        cv::destroyAllWindows();
    }
    
    void visualizeFrames(const SynchronizedFrameSet& frameset) {
        frame_count_++;
        
        // Create side-by-side display (color + depth + stats)
        int total_width = display_width_ * 2;
        int total_height = display_height_ + stats_panel_height_;
        cv::Mat display_image = cv::Mat::zeros(total_height, total_width, CV_8UC3);
        
        // Process and display color frame
        cv::Mat color_display = processColorFrame(frameset);
        color_display.copyTo(display_image(cv::Rect(0, 0, display_width_, display_height_)));
        
        // Process and display depth frame
        cv::Mat depth_display = processDepthFrame(frameset);
        depth_display.copyTo(display_image(cv::Rect(display_width_, 0, display_width_, display_height_)));
        
        // Add frame information overlays
        addFrameOverlay(display_image, frameset, "color", 0, 0);
        addFrameOverlay(display_image, frameset, "depth", display_width_, 0);
        
        // Add synchronization statistics panel
        addSyncStatsPanel(display_image, frameset);
        
        // Show the display
        cv::imshow("Synchronized Color + Depth", display_image);
        
        // Handle keyboard input
        char key = cv::waitKey(1) & 0xFF;
        if (key == 's') {
            saveScreenshot(display_image, frameset);
        }
    }
    
private:
    cv::Mat processColorFrame(const SynchronizedFrameSet& frameset) {
        if (frameset.frames.find("color") == frameset.frames.end()) {
            return createPlaceholderImage("No Color Stream", cv::Scalar(50, 50, 100));
        }
        
        // Get color frame data from the map
        const auto& color_frames = frameset.frames.at("color");
        
        // Convert raw data to OpenCV Mat
        // Assuming RGB888 format for simplicity
        int width = 1920;  // You'll need to get actual dimensions
        int height = 1080;
        
        if (color_frames.size() != static_cast<size_t>(width * height * 3)) {
            // Try to detect format/size from data size
            if (color_frames.size() == 640 * 480 * 3) {
                width = 640; height = 480;
            } else if (color_frames.size() == 1280 * 720 * 3) {
                width = 1280; height = 720;
            }
        }
        
        cv::Mat color_mat(height, width, CV_8UC3);
        if (color_frames.size() >= static_cast<size_t>(width * height * 3)) {
            std::memcpy(color_mat.data, color_frames.data(), width * height * 3);
            cv::cvtColor(color_mat, color_mat, cv::COLOR_RGB2BGR); // Convert RGB to BGR for OpenCV
        } else {
            color_mat = createPlaceholderImage("Invalid Color Data", cv::Scalar(0, 0, 255));
        }
        
        return color_mat;
    }
    
    cv::Mat processDepthFrame(const SynchronizedFrameSet& frameset) {
        if (frameset.frames.find("depth") == frameset.frames.end()) {
            return createPlaceholderImage("No Depth Stream", cv::Scalar(100, 50, 50));
        }
        
        const auto& depth_data = frameset.frames.at("depth");
        
        // Assume standard depth resolution
        int width = 640;
        int height = 480;
        
        if (depth_data.size() != static_cast<size_t>(width * height * 2)) {
            // Try other common resolutions
            if (depth_data.size() == 848 * 480 * 2) {
                width = 848; height = 480;
            } else if (depth_data.size() == 424 * 240 * 2) {
                width = 424; height = 240;
            }
        }
        
        cv::Mat depth_raw(height, width, CV_16UC1);
        if (depth_data.size() >= static_cast<size_t>(width * height * 2)) {
            std::memcpy(depth_raw.data, depth_data.data(), width * height * 2);
        } else {
            return createPlaceholderImage("Invalid Depth Data", cv::Scalar(255, 0, 0));
        }
        
        // Convert depth to colorized visualization
        cv::Mat depth_8bit, depth_colored;
        
        // Normalize depth values for visualization
        double min_val, max_val;
        cv::minMaxLoc(depth_raw, &min_val, &max_val);
        
        if (max_val > min_val && max_val > 0) {
            // Scale to 8-bit range
            depth_raw.convertTo(depth_8bit, CV_8UC1, 255.0 / max_val);
            
            // Apply colormap for better visualization
            cv::applyColorMap(depth_8bit, depth_colored, cv::COLORMAP_JET);
            
            // Mask out invalid depth values (0 depth)
            cv::Mat mask = depth_raw == 0;
            depth_colored.setTo(cv::Scalar(0, 0, 0), mask);
        } else {
            depth_colored = createPlaceholderImage("No Depth Data", cv::Scalar(128, 128, 128));
        }
        
        return depth_colored;
    }
    
    cv::Mat createPlaceholderImage(const std::string& message, const cv::Scalar& color) {
        cv::Mat placeholder(display_height_, display_width_, CV_8UC3, color);
        
        // Add text message
        cv::Size text_size = cv::getTextSize(message, font_face_, font_scale_, font_thickness_, nullptr);
        cv::Point text_pos((placeholder.cols - text_size.width) / 2, 
                          (placeholder.rows + text_size.height) / 2);
        
        cv::putText(placeholder, message, text_pos, font_face_, font_scale_, 
                   cv::Scalar(255, 255, 255), font_thickness_);
        
        return placeholder;
    }
    
    void addFrameOverlay(cv::Mat& image, const SynchronizedFrameSet& frameset, 
                        const std::string& stream_name, int x_offset, int y_offset) {
        
        std::vector<std::string> info_lines;
        
        // Stream name
        std::string display_name = (stream_name == "color") ? "COLOR" : "DEPTH";
        info_lines.push_back(display_name);
        
        // Frameset ID - use a simple counter if not available
        static uint64_t frame_counter = 0;
        info_lines.push_back("Set #" + std::to_string(++frame_counter));
        
        // Timestamp (show current time in milliseconds for sync verification)
        auto now = std::chrono::high_resolution_clock::now();
        uint64_t timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();
        uint64_t timestamp_frac = timestamp_ms % 1000;
        info_lines.push_back("TS: " + std::to_string(timestamp_frac) + "ms");
        
        // Data size if available
        if (frameset.frames.find(stream_name) != frameset.frames.end()) {
            size_t data_size = frameset.frames.at(stream_name).size();
            info_lines.push_back("Size: " + std::to_string(data_size / 1024) + "KB");
        }
        
        // Create semi-transparent background
        cv::Rect info_rect(x_offset + 5, y_offset + 5, 150, info_lines.size() * 20 + 10);
        cv::Mat overlay;
        image.copyTo(overlay);
        cv::rectangle(overlay, info_rect, cv::Scalar(0, 0, 0), -1);
        cv::addWeighted(image, 0.7, overlay, 0.3, 0, image);
        
        // Draw text lines
        for (size_t i = 0; i < info_lines.size(); ++i) {
            cv::putText(image, info_lines[i], 
                       cv::Point(x_offset + 10, y_offset + 25 + i * 20), 
                       font_face_, font_scale_, cv::Scalar(255, 255, 255), font_thickness_);
        }
    }
    
    void addSyncStatsPanel(cv::Mat& image, const SynchronizedFrameSet& frameset) {
        // Stats panel at bottom
        cv::Rect stats_rect(0, display_height_, display_width_ * 2, stats_panel_height_);
        cv::rectangle(image, stats_rect, cv::Scalar(40, 40, 40), -1);
        
        std::vector<std::string> stats_lines;
        
        // Frame count and sync info
        static uint64_t display_frame_count = 0;
        stats_lines.push_back("Frame Count: " + std::to_string(frame_count_) + 
                             "  |  Display #" + std::to_string(++display_frame_count));
        
        // Sync quality - use a simple indicator since we don't have access to timing details
        std::string sync_quality = "ACTIVE";
        cv::Scalar quality_color = cv::Scalar(0, 255, 0); // Green for active
        
        if (frameset.frames.size() == 2) {
            sync_quality = "SYNCHRONIZED";
            quality_color = cv::Scalar(0, 255, 0); // Green
        } else if (frameset.frames.size() == 1) {
            sync_quality = "PARTIAL";
            quality_color = cv::Scalar(0, 255, 255); // Yellow
        } else {
            sync_quality = "NO DATA";
            quality_color = cv::Scalar(0, 0, 255); // Red
        }
        
        stats_lines.push_back("Sync Quality: " + sync_quality + 
                             "  |  Streams: " + std::to_string(frameset.frames.size()));
        
        // Stream status
        std::string stream_status = "Streams: ";
        stream_status += frameset.frames.find("color") != frameset.frames.end() ? "COLOR✓ " : "COLOR✗ ";
        stream_status += frameset.frames.find("depth") != frameset.frames.end() ? "DEPTH✓" : "DEPTH✗";
        stats_lines.push_back(stream_status);
        
        // Current timestamp
        auto now = std::chrono::high_resolution_clock::now();
        uint64_t current_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();
        stats_lines.push_back("Current Time: " + std::to_string(current_ms % 10000) + "ms");
        
        // Draw stats text
        int y_start = display_height_ + 20;
        for (size_t i = 0; i < stats_lines.size(); ++i) {
            cv::Scalar text_color = (i == 1) ? quality_color : cv::Scalar(255, 255, 255);
            cv::putText(image, stats_lines[i], 
                       cv::Point(10, y_start + i * 25), 
                       font_face_, font_scale_, text_color, font_thickness_);
        }
    }
    
    void saveScreenshot(const cv::Mat& image, const SynchronizedFrameSet& frameset) {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        
        static uint64_t screenshot_counter = 0;
        std::ostringstream filename;
        filename << "sync_capture_" << (++screenshot_counter) << "_" 
                << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".png";
        
        cv::imwrite(filename.str(), image);
        std::cout << "Screenshot saved: " << filename.str() << std::endl;
    }
    
    // Member variables
    int display_width_, display_height_;
    int stats_panel_height_;
    uint64_t frame_count_;
    int font_face_;
    double font_scale_;
    int font_thickness_;
};

// Main demonstration function
int main() {
    std::cout << "=== DetermSensor Synchronized Color+Depth Visualization ===" << std::endl;
    
    // Initialize sensor
    DetermSensor sensor;
    
    if (sensor.Initialize() != 0) {
        std::cerr << "Failed to initialize DetermSensor" << std::endl;
        return -1;
    }
    
    // Configure streams
    std::vector<std::string> streams = {"color", "depth"};
    if (sensor.EnableStream(streams) != 0) {
        std::cerr << "Failed to enable streams" << std::endl;
        return -1;
    }
    
    // Configure sensors
    sensor.SetColorSensorConfig(1280, 720, "RGB888");
    sensor.SetDepthSensorConfig(640, 480);
    
    // Set auto controls
    sensor.SetColorSensorControl(true);  // Auto exposure/white balance
    sensor.SetDepthSensorControl(true);  // Auto exposure
    
    // Start capture
    if (sensor.Start() != 0) {
        std::cerr << "Failed to start sensor" << std::endl;
        return -1;
    }
    
    // Configure synchronization
    sensor.SetSynchronizationMode(SyncMode::SOFTWARE_SYNC);
    sensor.SetSyncTolerance(20.0); // 20ms tolerance
    
    // Create visualizer
    SyncVisualizer visualizer;
    
    std::cout << "Starting synchronized capture..." << std::endl;
    std::cout << "Press 'Q' to quit, 'S' to save screenshot" << std::endl;
    
    // Main capture loop
    int timeout_count = 0;
    const int max_timeouts = 50;
    
    while (timeout_count < max_timeouts) {
        // Get synchronized frames
        auto frameset = sensor.GetSynchronizedFrames(100.0); // 100ms timeout
        
        if (!frameset.frames.empty()) {
            timeout_count = 0; // Reset timeout counter
            visualizer.visualizeFrames(frameset);
            
            // Check for quit command
            char key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27) { // 'q' or ESC
                std::cout << "Quit requested" << std::endl;
                break;
            }
        } else {
            timeout_count++;
            if (timeout_count % 10 == 0) {
                std::cout << "Waiting for synchronized frames... (" << timeout_count << ")" << std::endl;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    if (timeout_count >= max_timeouts) {
        std::cout << "Too many timeouts, exiting..." << std::endl;
    }
    
    // Cleanup
    sensor.Stop();
    std::cout << "Synchronized visualization completed" << std::endl;
    
    return 0;
}