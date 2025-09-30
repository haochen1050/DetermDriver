#include "sensor/determ_sensor.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>

using namespace determ::sensor;

class DetermSensorVisualizer {
public:
    DetermSensorVisualizer() : 
        display_width_(640), display_height_(480),
        stats_height_(150), frame_counter_(0),
        last_stats_update_(std::chrono::steady_clock::now()) {
        
        // Create windows
        cv::namedWindow("DetermSensor: Color + Depth Synchronized View", cv::WINDOW_AUTOSIZE);
        
        // Initialize font for text overlay
        font_face_ = cv::FONT_HERSHEY_SIMPLEX;
        font_scale_ = 0.5;
        font_thickness_ = 1;
    }
    
    ~DetermSensorVisualizer() {
        cv::destroyAllWindows();
    }
    
    void visualizeSynchronizedFrame(const SynchronizedFrameSet& frame_set, const SyncStats& stats) {
        frame_counter_++;
        
        // Create main display image (2x1 grid: color + depth side by side + stats panel)
        cv::Mat display_image = cv::Mat::zeros(display_height_ + stats_height_, display_width_ * 2, CV_8UC3);
        
        // Process and place streams side by side
        processAndPlaceStream(display_image, frame_set, "color", 0, 0, "COLOR");
        processAndPlaceStream(display_image, frame_set, "depth", 1, 0, "DEPTH");
        
        // Add statistics panel at bottom
        addStatisticsPanel(display_image, stats, frame_set);
        
        // Add sync quality indicator
        addSyncQualityIndicator(display_image, frame_set);
        
        // Show the combined image
        cv::imshow("DetermSensor: Color + Depth Synchronized View", display_image);
        
        // Handle key presses (non-blocking)
        char key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) { // 'q' or ESC to quit
            std::cout << "Quit key pressed" << std::endl;
        } else if (key == 's') { // 's' to save screenshot
            saveScreenshot(display_image, frame_set);
        } else if (key == 'r') { // 'r' to reset stats
            std::cout << "Reset stats requested" << std::endl;
        // } else if (key == 'c') { // 'c' to calibrate clocks
        //     std::cout << "Clock calibration requested" << std::endl;
        } else if (key == '1') { // '1' for software sync
            std::cout << "Software sync mode requested" << std::endl;
        } else if (key == '2') { // '2' for hardware sync
            std::cout << "Hardware sync mode requested" << std::endl;
        } else if (key == '3') { // '3' for hybrid sync
            std::cout << "Hybrid sync mode requested" << std::endl;
        }
    }
    
private:
    void processAndPlaceStream(cv::Mat& display_image, const SynchronizedFrameSet& frame_set, 
                              const std::string& stream_name, int grid_x, int grid_y, 
                              const std::string& display_name) {
        cv::Mat stream_display;
        
        if (frame_set.hasStream(stream_name)) {
            if (stream_name == "color") {
                stream_display = processColorFrame(*frame_set.getFrame(stream_name));
            } else if (stream_name == "depth") {
                stream_display = processDepthFrame(*frame_set.getFrame(stream_name));
            }
        } else {
            // Create placeholder for missing stream
            cv::Scalar placeholder_color = getPlaceholderColor(stream_name);
            stream_display = createPlaceholderImage("No " + display_name + " Stream", placeholder_color);
        }
        
        // Resize to fit grid cell
        cv::resize(stream_display, stream_display, cv::Size(display_width_, display_height_));
        
        // Calculate position in grid (side by side)
        int x_pos = grid_x * display_width_;
        int y_pos = grid_y * display_height_;
        
        // Copy to main display
        stream_display.copyTo(display_image(cv::Rect(x_pos, y_pos, display_width_, display_height_)));
        
        // Add frame info overlay
        addFrameInfoOverlay(display_image, frame_set, x_pos, y_pos, stream_name, display_name);
    }
    
    cv::Mat processColorFrame(const TimestampedFrame& frame) {
        cv::Mat color_mat;
        
        if (frame.format == "RGB888") {
            cv::Mat rgb_mat(frame.height, frame.width, CV_8UC3, (void*)frame.data.data());
            color_mat = rgb_mat.clone();
        } else if (frame.format == "BGR888") {
            color_mat = cv::Mat(frame.height, frame.width, CV_8UC3, (void*)frame.data.data()).clone();
        } else if (frame.format == "YUYV") {
            cv::Mat yuyv_mat(frame.height, frame.width, CV_8UC2, (void*)frame.data.data());
            cv::cvtColor(yuyv_mat, color_mat, cv::COLOR_YUV2BGR_YUYV);
        } else if (frame.format == "NV12") {
            cv::Mat nv12_mat(frame.height * 3/2, frame.width, CV_8UC1, (void*)frame.data.data());
            cv::cvtColor(nv12_mat, color_mat, cv::COLOR_YUV2BGR_NV12);
        } else {
            color_mat = createPlaceholderImage("Unknown Color Format: " + frame.format, cv::Scalar(0, 0, 255));
        }
        
        return color_mat;
    }
    
    cv::Mat processDepthFrame(const TimestampedFrame& frame) {
        if (frame.format != "Z16") {
            return createPlaceholderImage("Unsupported Depth Format: " + frame.format, cv::Scalar(255, 0, 0));
        }
        
        cv::Mat depth_raw(frame.height, frame.width, CV_16UC1, (void*)frame.data.data());
        cv::Mat depth_8bit, depth_colored;
        
        // Normalize depth values for better visualization
        double min_val, max_val;
        cv::minMaxLoc(depth_raw, &min_val, &max_val);
        
        if (max_val > min_val) {
            // Use a reasonable depth range (e.g., 0.5m to 5m for typical indoor scenes)
            double depth_min = std::max(min_val, 500.0);  // 0.5m in mm
            double depth_max = std::min(max_val, 5000.0); // 5m in mm
            
            depth_raw.convertTo(depth_8bit, CV_8UC1, 255.0 / (depth_max - depth_min), -depth_min * 255.0 / (depth_max - depth_min));
            
            // Clamp values
            cv::threshold(depth_8bit, depth_8bit, 255, 255, cv::THRESH_TRUNC);
            cv::threshold(depth_8bit, depth_8bit, 0, 0, cv::THRESH_TOZERO);
        } else {
            depth_8bit = cv::Mat::zeros(frame.height, frame.width, CV_8UC1);
        }
        
        // Apply colormap for better depth visualization
        cv::applyColorMap(depth_8bit, depth_colored, cv::COLORMAP_JET);
        
        // Mask out invalid depth values (set to black)
        cv::Mat mask = depth_raw == 0;
        depth_colored.setTo(cv::Scalar(0, 0, 0), mask);
        
        return depth_colored;
    }
    
    cv::Mat createPlaceholderImage(const std::string& text, const cv::Scalar& color) {
        cv::Mat placeholder(display_height_, display_width_, CV_8UC3, color);
        
        cv::Size text_size = cv::getTextSize(text, font_face_, font_scale_ * 1.2, font_thickness_ * 2, nullptr);
        cv::Point text_origin((placeholder.cols - text_size.width) / 2, 
                             (placeholder.rows + text_size.height) / 2);
        
        cv::putText(placeholder, text, text_origin, font_face_, font_scale_ * 1.2, 
                   cv::Scalar(255, 255, 255), font_thickness_ * 2);
        
        return placeholder;
    }
    
    cv::Scalar getPlaceholderColor(const std::string& stream_name) {
        if (stream_name == "color") return cv::Scalar(50, 50, 100);
        if (stream_name == "depth") return cv::Scalar(100, 50, 50);
        return cv::Scalar(75, 75, 75);
    }
    
    void addFrameInfoOverlay(cv::Mat& image, const SynchronizedFrameSet& frame_set, 
                           int x_offset, int y_offset, const std::string& stream_name, 
                           const std::string& display_name) {
        if (!frame_set.hasStream(stream_name)) {
            return;
        }
        
        const TimestampedFrame* frame = frame_set.getFrame(stream_name);
        
        std::vector<std::string> info_lines;
        info_lines.push_back(display_name);
        info_lines.push_back(std::to_string(frame->width) + "x" + std::to_string(frame->height));
        info_lines.push_back("F#" + std::to_string(frame->frame_number));
        
        // Format timestamp (show milliseconds for sync testing)
        auto timestamp_ms = frame->getPrimaryTimestampMs(); 
        auto timestamp_sec = static_cast<uint64_t>(timestamp_ms / 1000);
        auto timestamp_frac = static_cast<uint64_t>(timestamp_ms) % 1000;
        
        std::ostringstream ts_stream;
        ts_stream << std::setfill('0') << std::setw(3) << timestamp_frac << "ms";
        info_lines.push_back(ts_stream.str());
        
        // Show if hardware or software timestamp
        info_lines.push_back(frame->has_hardware_timestamp ? "HW" : "SW");
        
        // Draw compact info overlay
        cv::Rect info_rect(x_offset + 5, y_offset + 5, 120, info_lines.size() * 18 + 8);
        cv::Mat overlay;
        image.copyTo(overlay);
        cv::rectangle(overlay, info_rect, cv::Scalar(0, 0, 0), -1);
        cv::addWeighted(image, 0.6, overlay, 0.4, 0, image);
        
        for (size_t i = 0; i < info_lines.size(); ++i) {
            cv::putText(image, info_lines[i], 
                       cv::Point(x_offset + 8, y_offset + 20 + i * 18), 
                       font_face_, font_scale_, cv::Scalar(255, 255, 255), font_thickness_);
        }
    }
    
    void addStatisticsPanel(cv::Mat& image, const SyncStats& stats, const SynchronizedFrameSet& frame_set) {
        cv::Rect stats_rect(0, display_height_, display_width_ * 2, stats_height_);
        cv::rectangle(image, stats_rect, cv::Scalar(30, 30, 30), -1);
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_stats_update_);
        if (elapsed.count() > 500) {
            const_cast<SyncStats&>(stats).calculateFPS();
            last_stats_update_ = now;
        }
        
        std::vector<std::string> stats_lines;
        
        std::ostringstream row1;
        row1 << "SYNCHRONIZED: " << stats.total_frames_synchronized 
             << "  |  DROPPED: " << stats.total_frames_dropped
             << "  |  SYNC RATE: " << std::fixed << std::setprecision(1)
             << (100.0 * stats.total_frames_synchronized / std::max(1UL, stats.total_frames_captured)) << "%";
        stats_lines.push_back(row1.str());
        
        std::ostringstream row2;
        row2 << "SYNC FPS: " << std::setprecision(1) << stats.sync_fps
             << "  |  SYNC TIME - AVG: " << std::setprecision(2) << stats.avg_sync_time_ms << "ms"
             << "  |  CURRENT: " << frame_set.max_time_diff_ms << "ms";
        stats_lines.push_back(row2.str());
        
        std::ostringstream row3;
        row3 << "FRAME SET #" << frame_set.sync_id << "  |  STREAMS: " << frame_set.frames.size();
        stats_lines.push_back(row3.str());
        
        std::ostringstream row4;
        row4 << "STREAM FRAMES: ";
        bool first = true;
        for (const auto& [stream, count] : stats.frames_per_stream) {
            if (!first) row4 << " | ";
            row4 << stream << ":" << count;
            first = false;
        }
        stats_lines.push_back(row4.str());
        
        stats_lines.push_back("CONTROLS: [Q]uit | [S]ave | [R]eset | [C]alibrate | [1]SW [2]HW [3]Hybrid");
        
        int y_start = display_height_ + 15;
        for (size_t i = 0; i < stats_lines.size(); ++i) {
            cv::putText(image, stats_lines[i], 
                       cv::Point(15, y_start + i * 22), 
                       font_face_, font_scale_, cv::Scalar(255, 255, 255), font_thickness_);
        }
    }
    
    void addSyncQualityIndicator(cv::Mat& image, const SynchronizedFrameSet& frame_set) {
        cv::Scalar indicator_color;
        std::string quality_text;
        
        if (frame_set.max_time_diff_ms < 5.0) {
            indicator_color = cv::Scalar(0, 255, 0); // Green
            quality_text = "EXCELLENT";
        } else if (frame_set.max_time_diff_ms < 16.0) {
            indicator_color = cv::Scalar(0, 255, 255); // Yellow
            quality_text = "GOOD";
        } else if (frame_set.max_time_diff_ms < 33.0) {
            indicator_color = cv::Scalar(0, 165, 255); // Orange
            quality_text = "FAIR";
        } else {
            indicator_color = cv::Scalar(0, 0, 255); // Red
            quality_text = "POOR";
        }
        
        cv::Point indicator_pos(display_width_ * 2 - 100, 25);
        cv::circle(image, indicator_pos, 8, indicator_color, -1);
        cv::putText(image, quality_text, cv::Point(indicator_pos.x - 60, indicator_pos.y + 4), 
                   font_face_, font_scale_, indicator_color, font_thickness_);
    }
    
    void saveScreenshot(const cv::Mat& image, const SynchronizedFrameSet& frame_set) {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        
        std::ostringstream filename;
        filename << "determsensor_sync_" << frame_set.sync_id << "_" 
                << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".png";
        
        cv::imwrite(filename.str(), image);
        std::cout << "Screenshot saved: " << filename.str() << std::endl;
    }
    
    int display_width_, display_height_;
    int stats_height_;
    uint64_t frame_counter_;
    std::chrono::steady_clock::time_point last_stats_update_;
    
    int font_face_;
    double font_scale_;
    int font_thickness_;
};

// Example usage function with DetermSensor
void runDetermSensorVisualization() {
    // Create and configure DetermSensor
    DetermSensor sensor;
    DetermSensorVisualizer visualizer;
    
    std::cout << "=== DetermSensor Color + Depth Synchronization Visualization ===" << std::endl;
    
    // Initialize sensor
    if (sensor.Initialize() != 0) {
        std::cerr << "Failed to initialize DetermSensor" << std::endl;
        return;
    }
    
    // Enable color and depth streams
    std::vector<std::string> streams = {"color", "depth"};
    if (sensor.EnableStream(streams) != 0) {
        std::cerr << "Failed to enable streams" << std::endl;
        return;
    }
    
    // Configure color sensor
    if (sensor.SetColorSensorConfig(640, 480, "RGB888") != 0) {
        std::cerr << "Failed to configure color sensor" << std::endl;
        return;
    }
    
    // Configure depth sensor
    if (sensor.SetDepthSensorConfig(640, 480) != 0) {
        std::cerr << "Failed to configure depth sensor" << std::endl;
        return;
    }
    
    // Set synchronization parameters
    sensor.SetSyncTolerance(10); // set tolerance
    sensor.SetSyncBufferSize(15);
    
    // Start sensor
    if (sensor.Start() != 0) {
        std::cerr << "Failed to start DetermSensor" << std::endl;
        return;
    }
    
    // Start synchronized capture with hybrid mode (best of both worlds)
    if (!sensor.StartSynchronizedCapture(SyncMode::HARDWARE_SYNC)) {
        std::cerr << "Failed to start synchronized capture" << std::endl;
        sensor.Stop();
        return;
    }
    
    std::cout << "Color and depth streams displayed side by side" << std::endl;
    std::cout << "HW/SW timestamp indicators show synchronization method" << std::endl;
    std::cout << "Press keys to control synchronization mode:" << std::endl;
    std::cout << "  1 - Software Sync" << std::endl;
    std::cout << "  2 - Hardware Sync" << std::endl; 
    std::cout << "  3 - Hybrid Sync" << std::endl;
    // std::cout << "  C - Calibrate Clocks" << std::endl;
    std::cout << "  R - Reset Statistics" << std::endl;
    std::cout << "  S - Save Screenshot" << std::endl;
    std::cout << "  Q - Quit" << std::endl;
    
    SynchronizedFrameSet frame_set;
    int consecutive_timeouts = 0;
    const int max_timeouts = 20;
    
    while (consecutive_timeouts < max_timeouts) {
        if (sensor.GetLatestSynchronizedFrames(frame_set)) {
            consecutive_timeouts = 0;
            visualizer.visualizeSynchronizedFrame(frame_set, sensor.getSyncStats());
            
            char key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27) break;
            if (key == 'r') sensor.resetSyncStats();
            // if (key == 'c') sensor.CalibrateClocks();
            if (key == '1') {
                sensor.StopSynchronizedCapture();
                sensor.StartSynchronizedCapture(SyncMode::SOFTWARE_SYNC);
            }
            if (key == '2') {
                sensor.StopSynchronizedCapture();
                sensor.StartSynchronizedCapture(SyncMode::HARDWARE_SYNC);
            }
            if (key == '3') {
                sensor.StopSynchronizedCapture();
                sensor.StartSynchronizedCapture(SyncMode::HYBRID_SYNC);
            }
        } else {
            consecutive_timeouts++;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    sensor.StopSynchronizedCapture();
    sensor.Stop();
    sensor.printSyncStats(true);
}

int main() {
    try {
        runDetermSensorVisualization();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}