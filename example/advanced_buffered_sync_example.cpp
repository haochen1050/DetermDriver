#include "sensor/determ_sensor.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <deque>

using namespace determ::capture;

/**
 * @brief Visualizer demonstrating timestamp-based synchronization
 * 
 * This example showcases the simplified synchronization approach where frames
 * from both sensors (RealSense with hardware timestamps, Raspberry Pi with 
 * software timestamps) are matched based on direct timestamp comparison.
 * 
 * Key changes from previous version:
 * - No SyncMode selection (always uses native timestamps)
 * - Emphasizes timestamp quality from each sensor
 * - Shows how hardware and software timestamps are compared
 */
class SynchronizationVisualizer {
public:
    SynchronizationVisualizer() : 
        display_width_(640), display_height_(480),
        stats_height_(200), timestamp_viz_height_(150),
        frame_counter_(0),
        last_stats_update_(std::chrono::steady_clock::now()) {
        
        // Create window for sync visualization
        cv::namedWindow("Native Timestamp Synchronization", cv::WINDOW_AUTOSIZE);
        
        font_face_ = cv::FONT_HERSHEY_SIMPLEX;
        font_scale_ = 0.45;
        font_thickness_ = 1;
        
        // Track timestamp statistics
        max_timestamp_diff_ = 0.0;
        min_timestamp_diff_ = std::numeric_limits<double>::max();
        avg_timestamp_diff_ = 0.0;
        timestamp_diff_samples_ = 0;
    }
    
    ~SynchronizationVisualizer() {
        cv::destroyAllWindows();
    }
    
    /**
     * @brief Main visualization function showing native timestamp sync
     * 
     * Layout:
     * - Top: Synchronized frames (color + depth)
     * - Middle: Timestamp visualization showing native timestamps
     * - Bottom: Detailed synchronization statistics
     */
    void visualizeSync(const SynchronizedFrameSet& frame_set, const SyncStats& stats) {
        frame_counter_++;
        
        // Update timestamp difference statistics
        if (frame_set.max_time_diff_ms > 0) {
            max_timestamp_diff_ = std::max(max_timestamp_diff_, frame_set.max_time_diff_ms);
            min_timestamp_diff_ = std::min(min_timestamp_diff_, frame_set.max_time_diff_ms);
            avg_timestamp_diff_ = (avg_timestamp_diff_ * timestamp_diff_samples_ + frame_set.max_time_diff_ms) 
                                  / (timestamp_diff_samples_ + 1);
            timestamp_diff_samples_++;
        }
        
        // Total height: streams + timestamp viz + stats
        int total_height = display_height_ + timestamp_viz_height_ + stats_height_;
        cv::Mat display_image = cv::Mat::zeros(total_height, display_width_ * 2, CV_8UC3);
        
        // 1. Top section: Synchronized frames
        processAndPlaceStream(display_image, frame_set, "color", 0, 0, "COLOR (SW-TS)");
        processAndPlaceStream(display_image, frame_set, "depth", 1, 0, "DEPTH (HW-TS)");
        
        // 2. Middle section: Timestamp visualization
        addTimestampVisualization(display_image, frame_set);
        
        // 3. Bottom section: Statistics
        addDetailedStatistics(display_image, stats, frame_set);
        
        // 4. Add sync quality indicator
        addSyncQualityIndicator(display_image, frame_set);
        
        // Show the combined visualization
        cv::imshow("Native Timestamp Synchronization", display_image);
        
        char key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) {
            std::cout << "Quit key pressed" << std::endl;
        } else if (key == 's') {
            saveDetailedScreenshot(display_image, frame_set);
        }
    }
    
    /**
     * @brief Explain the simplified synchronization approach
     */
    void demonstrateSyncConcepts() {
        std::cout << "\n=== Native Timestamp Synchronization ===" << std::endl;
        std::cout << "\n1. TIMESTAMP SOURCES:" << std::endl;
        std::cout << "   RealSense D421:" << std::endl;
        std::cout << "   - Hardware timestamps from sensor" << std::endl;
        std::cout << "   - GLOBAL_TIME domain (system clock)" << std::endl;
        std::cout << "   - High precision, minimal jitter" << std::endl;
        std::cout << "   - Retrieved via: rs_frame.get_timestamp()" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "   Raspberry Pi Camera:" << std::endl;
        std::cout << "   - Software timestamps from capture time" << std::endl;
        std::cout << "   - std::chrono::high_resolution_clock" << std::endl;
        std::cout << "   - Captured at frame acquisition" << std::endl;
        std::cout << "   - Retrieved via: TimestampedRaspberryFrame.timestamp" << std::endl;
        
        std::cout << "\n2. SYNCHRONIZATION ALGORITHM:" << std::endl;
        std::cout << "   Step 1: Capture Raspberry Pi frame → get software timestamp" << std::endl;
        std::cout << "   Step 2: Capture RealSense frameset → get hardware timestamps" << std::endl;
        std::cout << "   Step 3: Convert both to microseconds since epoch" << std::endl;
        std::cout << "   Step 4: Calculate time difference between all frames" << std::endl;
        std::cout << "   Step 5: Accept if max_diff ≤ sync_tolerance (default 16.67ms)" << std::endl;
        
        std::cout << "\n3. TIMESTAMP CONVERSION:" << std::endl;
        std::cout << "   RealSense (ms) → microseconds:" << std::endl;
        std::cout << "     timestamp_us = hw_timestamp_ms * 1000.0" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "   Raspberry Pi (time_point) → microseconds:" << std::endl;
        std::cout << "     timestamp_us = duration_cast<microseconds>(" << std::endl;
        std::cout << "                      timestamp.time_since_epoch()).count()" << std::endl;
        
        std::cout << "\n4. SYNC QUALITY METRICS:" << std::endl;
        std::cout << "   max_time_diff_ms: Maximum timestamp difference" << std::endl;
        std::cout << "   - Excellent: < 2ms" << std::endl;
        std::cout << "   - Good: 2-8ms" << std::endl;
        std::cout << "   - Acceptable: 8-16.67ms (1 frame @ 60fps)" << std::endl;
        std::cout << "   - Poor: > 16.67ms" << std::endl;
        
        std::cout << "\n5. ADVANTAGES OF THIS APPROACH:" << std::endl;
        std::cout << "   ✓ Simple and maintainable code" << std::endl;
        std::cout << "   ✓ Uses each sensor's best timestamp source" << std::endl;
        std::cout << "   ✓ No clock calibration needed" << std::endl;
        std::cout << "   ✓ Direct microsecond comparison" << std::endl;
        std::cout << "   ✓ Clear sync quality metrics" << std::endl;
        
        std::cout << "\n6. KEY PARAMETERS:" << std::endl;
        std::cout << "   sync_tolerance_ms: Maximum acceptable time difference" << std::endl;
        std::cout << "   - Default: 16.67ms (suitable for 30fps)" << std::endl;
        std::cout << "   - For 60fps: consider 8.33ms" << std::endl;
        std::cout << "   - For 15fps: consider 33.33ms" << std::endl;
        std::cout << "================================================\n" << std::endl;
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
            stream_display = createPlaceholderImage("No " + display_name, cv::Scalar(50, 50, 80));
        }
        
        cv::resize(stream_display, stream_display, cv::Size(display_width_, display_height_));
        
        int x_pos = grid_x * display_width_;
        int y_pos = grid_y * display_height_;
        
        stream_display.copyTo(display_image(cv::Rect(x_pos, y_pos, display_width_, display_height_)));
        
        addStreamInfoOverlay(display_image, frame_set, x_pos, y_pos, stream_name, display_name);
    }
    
    cv::Mat processColorFrame(const TimestampedFrame& frame) {
        cv::Mat color_mat;
        
        if (frame.format == "RGB888") {
            cv::Mat rgb_mat(frame.height, frame.width, CV_8UC3, (void*)frame.data.data());
            cv::cvtColor(rgb_mat, color_mat, cv::COLOR_RGB2BGR);
        } else if (frame.format == "BGR888") {
            color_mat = cv::Mat(frame.height, frame.width, CV_8UC3, (void*)frame.data.data()).clone();
        } else if (frame.format == "YUYV") {
            cv::Mat yuyv_mat(frame.height, frame.width, CV_8UC2, (void*)frame.data.data());
            cv::cvtColor(yuyv_mat, color_mat, cv::COLOR_YUV2BGR_YUYV);
        } else if (frame.format == "NV12") {
            cv::Mat nv12_mat(frame.height * 3/2, frame.width, CV_8UC1, (void*)frame.data.data());
            cv::cvtColor(nv12_mat, color_mat, cv::COLOR_YUV2BGR_NV12);
        } else {
            color_mat = createPlaceholderImage("Unknown Format", cv::Scalar(0, 0, 100));
        }
        
        return color_mat;
    }
    
    cv::Mat processDepthFrame(const TimestampedFrame& frame) {
        if (frame.format != "Z16") {
            return createPlaceholderImage("Unsupported Format", cv::Scalar(100, 0, 0));
        }
        
        cv::Mat depth_raw(frame.height, frame.width, CV_16UC1, (void*)frame.data.data());
        cv::Mat depth_8bit, depth_colored;
        
        double min_val, max_val;
        cv::minMaxLoc(depth_raw, &min_val, &max_val);
        
        if (max_val > min_val) {
            double depth_min = std::max(min_val, 500.0);
            double depth_max = std::min(max_val, 5000.0);
            
            depth_raw.convertTo(depth_8bit, CV_8UC1, 255.0 / (depth_max - depth_min), 
                              -depth_min * 255.0 / (depth_max - depth_min));
            
            cv::threshold(depth_8bit, depth_8bit, 255, 255, cv::THRESH_TRUNC);
            cv::threshold(depth_8bit, depth_8bit, 0, 0, cv::THRESH_TOZERO);
        } else {
            depth_8bit = cv::Mat::zeros(frame.height, frame.width, CV_8UC1);
        }
        
        cv::applyColorMap(depth_8bit, depth_colored, cv::COLORMAP_JET);
        cv::Mat mask = depth_raw == 0;
        depth_colored.setTo(cv::Scalar(0, 0, 0), mask);
        
        return depth_colored;
    }
    
    /**
     * @brief Visualize native timestamps from both sensors
     * 
     * This shows:
     * - Raw timestamps from each sensor
     * - Timestamp difference
     * - Timestamp type (hardware vs software)
     */
    void addTimestampVisualization(cv::Mat& image, const SynchronizedFrameSet& frame_set) {
        int y_start = display_height_;
        cv::Rect ts_rect(0, y_start, display_width_ * 2, timestamp_viz_height_);
        
        // Dark background
        cv::rectangle(image, ts_rect, cv::Scalar(20, 20, 30), -1);
        
        // Title
        cv::putText(image, "NATIVE TIMESTAMP SYNCHRONIZATION", 
                   cv::Point(15, y_start + 20), font_face_, font_scale_ * 1.1, 
                   cv::Scalar(100, 200, 255), font_thickness_ + 1);
        
        int info_y = y_start + 45;
        int line_spacing = 20;
        
        // Get timestamps from each stream
        std::map<std::string, std::pair<uint64_t, bool>> timestamps; // timestamp_us, is_hardware
        
        for (const auto& [stream_name, frame] : frame_set.frames) {
            timestamps[stream_name] = {frame.getTimestampUs(), frame.has_hardware_timestamp};
        }
        
        // Display timestamp information
        cv::putText(image, "Timestamp Sources:", 
                   cv::Point(20, info_y), font_face_, font_scale_, 
                   cv::Scalar(200, 200, 200), font_thickness_);
        info_y += line_spacing;
        
        for (const auto& [stream_name, ts_info] : timestamps) {
            std::ostringstream ts_text;
            ts_text << "  " << stream_name << ": " 
                   << std::fixed << std::setprecision(3) << (ts_info.first / 1000.0) << " ms "
                   << (ts_info.second ? "[HW]" : "[SW]");
            
            cv::Scalar color = ts_info.second ? cv::Scalar(100, 255, 100) : cv::Scalar(100, 200, 255);
            cv::putText(image, ts_text.str(), 
                       cv::Point(40, info_y), font_face_, font_scale_, 
                       color, font_thickness_);
            info_y += line_spacing;
        }
        
        // Show timestamp difference
        info_y += 5;
        std::ostringstream diff_text;
        diff_text << "Time Difference: " << std::fixed << std::setprecision(2) 
                 << frame_set.max_time_diff_ms << " ms";
        
        cv::Scalar diff_color;
        if (frame_set.max_time_diff_ms < 2.0) {
            diff_color = cv::Scalar(0, 255, 0); // Green
        } else if (frame_set.max_time_diff_ms < 8.0) {
            diff_color = cv::Scalar(0, 255, 255); // Yellow
        } else if (frame_set.max_time_diff_ms < 16.67) {
            diff_color = cv::Scalar(0, 165, 255); // Orange
        } else {
            diff_color = cv::Scalar(0, 0, 255); // Red
        }
        
        cv::putText(image, diff_text.str(), 
                   cv::Point(20, info_y), font_face_, font_scale_, 
                   diff_color, font_thickness_ + 1);
        
        // Visual timeline representation
        int timeline_y = y_start + timestamp_viz_height_ - 15;
        cv::putText(image, "HW = Hardware timestamp (RealSense) | SW = Software timestamp (Raspberry Pi)", 
                   cv::Point(20, timeline_y), font_face_, font_scale_ * 0.85, 
                   cv::Scalar(150, 150, 150), font_thickness_);
    }
    
    /**
     * @brief Add detailed statistics
     */
    void addDetailedStatistics(cv::Mat& image, const SyncStats& stats, 
                              const SynchronizedFrameSet& frame_set) {
        int y_start = display_height_ + timestamp_viz_height_;
        cv::Rect stats_rect(0, y_start, display_width_ * 2, stats_height_);
        cv::rectangle(image, stats_rect, cv::Scalar(25, 25, 25), -1);
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_stats_update_);
        if (elapsed.count() > 500) {
            const_cast<SyncStats&>(stats).calculateFPS();
            last_stats_update_ = now;
        }
        
        std::vector<std::string> stat_lines;
        
        // Title
        stat_lines.push_back("=== SYNCHRONIZATION STATISTICS (NATIVE TIMESTAMPS) ===");
        stat_lines.push_back("");
        
        // Row 1: Frame counts
        std::ostringstream row1;
        row1 << "CAPTURED: " << stats.total_frames_captured 
             << "  |  SYNCHRONIZED: " << stats.total_frames_synchronized
             << "  |  DROPPED: " << stats.total_frames_dropped
             << "  |  SUCCESS RATE: " << std::fixed << std::setprecision(1)
             << (100.0 * stats.total_frames_synchronized / std::max(1UL, stats.total_frames_captured)) << "%";
        stat_lines.push_back(row1.str());
        
        // Row 2: Timing statistics
        std::ostringstream row2;
        row2 << "SYNC TIME - AVG: " << std::setprecision(2) << stats.avg_sync_time_ms << "ms"
             << "  |  MIN: " << stats.min_sync_time_ms << "ms"
             << "  |  MAX: " << stats.max_sync_time_ms << "ms"
             << "  |  CURRENT: " << frame_set.max_time_diff_ms << "ms";
        stat_lines.push_back(row2.str());
        
        // Row 3: Performance metrics
        std::ostringstream row3;
        row3 << "PERFORMANCE - CAPTURE FPS: " << std::setprecision(1) << stats.capture_fps
             << "  |  SYNC FPS: " << stats.sync_fps;
        stat_lines.push_back(row3.str());
        
        // Row 4: Timestamp difference statistics
        std::ostringstream row4;
        row4 << "TIMESTAMP DIFF - AVG: " << std::setprecision(2) << avg_timestamp_diff_ << "ms"
             << "  |  MIN: " << min_timestamp_diff_ << "ms"
             << "  |  MAX: " << max_timestamp_diff_ << "ms"
             << "  |  SAMPLES: " << timestamp_diff_samples_;
        stat_lines.push_back(row4.str());
        
        // Row 5: Current frame set info
        std::ostringstream row5;
        row5 << "CURRENT FRAME SET #" << frame_set.sync_id 
             << "  |  STREAMS: " << frame_set.frames.size()
             << "  |  AVG TIMESTAMP: " << std::setprecision(2) << frame_set.avg_timestamp_ms << "ms";
        stat_lines.push_back(row5.str());
        
        stat_lines.push_back("");
        
        // Row 6: Per-stream frame counts
        std::ostringstream row6;
        row6 << "STREAM FRAMES: ";
        bool first = true;
        for (const auto& [stream, count] : stats.frames_per_stream) {
            if (!first) row6 << " | ";
            row6 << stream << ":" << count;
            first = false;
        }
        stat_lines.push_back(row6.str());
        
        stat_lines.push_back("");
        stat_lines.push_back("CONTROLS: [Q]uit | [S]ave Screenshot");
        
        // Render statistics
        int text_y = y_start + 20;
        int line_spacing = 18;
        
        for (size_t i = 0; i < stat_lines.size(); ++i) {
            cv::Scalar text_color = (i == 0) ? cv::Scalar(100, 200, 255) : cv::Scalar(220, 220, 220);
            double scale = (i == 0) ? font_scale_ * 1.1 : font_scale_;
            
            cv::putText(image, stat_lines[i], 
                       cv::Point(15, text_y + i * line_spacing), 
                       font_face_, scale, text_color, font_thickness_);
        }
    }
    
    void addStreamInfoOverlay(cv::Mat& image, const SynchronizedFrameSet& frame_set, 
                             int x_offset, int y_offset, const std::string& stream_name, 
                             const std::string& display_name) {
        if (!frame_set.hasStream(stream_name)) return;
        
        const TimestampedFrame* frame = frame_set.getFrame(stream_name);
        
        std::vector<std::string> info_lines;
        info_lines.push_back(display_name);
        info_lines.push_back(std::to_string(frame->width) + "x" + std::to_string(frame->height));
        info_lines.push_back("F#" + std::to_string(frame->frame_number));
        
        // Show timestamp with microsecond precision
        auto timestamp_ms = frame->getPrimaryTimestampMs();
        auto timestamp_us = static_cast<uint64_t>(timestamp_ms * 1000.0) % 1000;
        
        std::ostringstream ts_stream;
        ts_stream << std::fixed << std::setprecision(0) << timestamp_ms << "." 
                  << std::setfill('0') << std::setw(3) << timestamp_us << "ms";
        info_lines.push_back(ts_stream.str());
        
        info_lines.push_back(frame->has_hardware_timestamp ? "HW-TS" : "SW-TS");
        
        // Compact info overlay
        cv::Rect info_rect(x_offset + 5, y_offset + 5, 130, info_lines.size() * 18 + 10);
        cv::Mat overlay;
        image.copyTo(overlay);
        cv::rectangle(overlay, info_rect, cv::Scalar(0, 0, 0), -1);
        cv::addWeighted(image, 0.5, overlay, 0.5, 0, image);
        
        for (size_t i = 0; i < info_lines.size(); ++i) {
            cv::putText(image, info_lines[i], 
                       cv::Point(x_offset + 8, y_offset + 22 + i * 18), 
                       font_face_, font_scale_, cv::Scalar(255, 255, 255), font_thickness_);
        }
    }
    
    void addSyncQualityIndicator(cv::Mat& image, const SynchronizedFrameSet& frame_set) {
        cv::Scalar indicator_color;
        std::string quality_text;
        
        // Quality based on timestamp difference
        if (frame_set.max_time_diff_ms < 2.0) {
            indicator_color = cv::Scalar(0, 255, 0); // Green - Excellent
            quality_text = "EXCELLENT";
        } else if (frame_set.max_time_diff_ms < 8.0) {
            indicator_color = cv::Scalar(0, 255, 255); // Yellow - Good
            quality_text = "GOOD";
        } else if (frame_set.max_time_diff_ms < 16.67) {
            indicator_color = cv::Scalar(0, 165, 255); // Orange - Acceptable
            quality_text = "ACCEPTABLE";
        } else {
            indicator_color = cv::Scalar(0, 0, 255); // Red - Poor
            quality_text = "POOR";
        }
        
        cv::Point indicator_pos(display_width_ * 2 - 110, 30);
        cv::circle(image, indicator_pos, 10, indicator_color, -1);
        cv::circle(image, indicator_pos, 10, cv::Scalar(255, 255, 255), 1);
        
        cv::putText(image, quality_text, cv::Point(indicator_pos.x - 75, indicator_pos.y + 5), 
                   font_face_, font_scale_, indicator_color, font_thickness_);
        
        // Show time difference below indicator
        std::ostringstream diff_text;
        diff_text << std::fixed << std::setprecision(2) << frame_set.max_time_diff_ms << "ms";
        cv::putText(image, diff_text.str(), cv::Point(indicator_pos.x - 30, indicator_pos.y + 25), 
                   font_face_, font_scale_ * 0.9, cv::Scalar(200, 200, 200), font_thickness_);
    }
    
    cv::Mat createPlaceholderImage(const std::string& text, const cv::Scalar& color) {
        cv::Mat placeholder(display_height_, display_width_, CV_8UC3, color);
        
        cv::Size text_size = cv::getTextSize(text, font_face_, font_scale_ * 1.5, font_thickness_ * 2, nullptr);
        cv::Point text_origin((placeholder.cols - text_size.width) / 2, 
                             (placeholder.rows + text_size.height) / 2);
        
        cv::putText(placeholder, text, text_origin, font_face_, font_scale_ * 1.5, 
                   cv::Scalar(255, 255, 255), font_thickness_ * 2);
        
        return placeholder;
    }
    
    void saveDetailedScreenshot(const cv::Mat& image, const SynchronizedFrameSet& frame_set) {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        
        std::ostringstream filename;
        filename << "native_sync_" << frame_set.sync_id << "_"
                << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".png";
        
        cv::imwrite(filename.str(), image);
        
        // Also save detailed text log
        std::ostringstream log_filename;
        log_filename << "native_sync_" << frame_set.sync_id << "_"
                    << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".txt";
        
        std::ofstream log_file(log_filename.str());
        log_file << "Native Timestamp Synchronization Snapshot\n";
        log_file << "=========================================\n\n";
        log_file << "Frame Set ID: " << frame_set.sync_id << "\n";
        log_file << "Max Time Difference: " << frame_set.max_time_diff_ms << " ms\n";
        log_file << "Average Timestamp: " << frame_set.avg_timestamp_ms << " ms\n\n";
        
        log_file << "Frame Timestamps:\n";
        for (const auto& [stream_name, frame] : frame_set.frames) {
            log_file << "  " << stream_name << ": " 
                    << (frame.getTimestampUs() / 1000.0) << " ms "
                    << (frame.has_hardware_timestamp ? "[Hardware]" : "[Software]") << "\n";
        }
        
        log_file << "\nTimestamp Difference Statistics:\n";
        log_file << "  Average: " << avg_timestamp_diff_ << " ms\n";
        log_file << "  Minimum: " << min_timestamp_diff_ << " ms\n";
        log_file << "  Maximum: " << max_timestamp_diff_ << " ms\n";
        
        log_file.close();
        
        std::cout << "Screenshot saved: " << filename.str() << std::endl;
        std::cout << "Log saved: " << log_filename.str() << std::endl;
    }
    
    int display_width_, display_height_;
    int stats_height_, timestamp_viz_height_;
    uint64_t frame_counter_;
    std::chrono::steady_clock::time_point last_stats_update_;
    
    int font_face_;
    double font_scale_;
    int font_thickness_;
    
    // Timestamp difference tracking
    double max_timestamp_diff_;
    double min_timestamp_diff_;
    double avg_timestamp_diff_;
    uint64_t timestamp_diff_samples_;
};

/**
 * @brief Main demonstration of native timestamp synchronization
 */
void runNativeTimestampSyncDemo() {
    DetermSensor sensor;
    SynchronizationVisualizer visualizer;
    
    std::cout << "\n╔════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Native Timestamp Synchronization Demonstration           ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════════╝\n" << std::endl;
    
    // Show conceptual explanation first
    visualizer.demonstrateSyncConcepts();
    
    std::cout << "\nPress ENTER to start live demonstration..." << std::endl;
    std::cin.get();
    
    // Initialize sensor
    if (sensor.Initialize() != 0) {
        std::cerr << "Failed to initialize DetermSensor" << std::endl;
        return;
    }
    
    // Enable streams
    std::vector<std::string> streams = {"color", "depth"};
    if (sensor.EnableStream(streams) != 0) {
        std::cerr << "Failed to enable streams" << std::endl;
        return;
    }
    
    // Configure sensors
    if (sensor.SetColorSensorConfig(640, 480, "YUYV") != 0) {
        std::cerr << "Failed to configure color sensor" << std::endl;
        return;
    }
    
    if (sensor.SetDepthSensorConfig(640, 480) != 0) {
        std::cerr << "Failed to configure depth sensor" << std::endl;
        return;
    }
    
    // Configure sync parameters (no mode selection needed!)
    std::cout << "\nConfiguring synchronization..." << std::endl;
    sensor.SetSyncTolerance(16.67);  // 1 frame @ 60fps tolerance
    
    std::cout << "  Sync tolerance: 16.67 ms (allows matching within ~1 frame @ 60fps)" << std::endl;
    std::cout << "  Using native timestamps:" << std::endl;
    std::cout << "    - RealSense: Hardware timestamps (GLOBAL_TIME)" << std::endl;
    std::cout << "    - Raspberry Pi: Software timestamps (high_resolution_clock)" << std::endl;
    
    // Start sensor
    if (sensor.Start() != 0) {
        std::cerr << "Failed to start DetermSensor" << std::endl;
        return;
    }
    
    // Start synchronized capture (no mode parameter!)
    std::cout << "\nStarting synchronized capture..." << std::endl;
    if (!sensor.StartSynchronizedCapture()) {
        std::cerr << "Failed to start synchronized capture" << std::endl;
        sensor.Stop();
        return;
    }
    
    std::cout << "\n╔════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  VISUALIZATION GUIDE                                       ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║  TOP SECTION: Synchronized frame output                   ║" << std::endl;
    std::cout << "║  - Left: Color stream (software timestamp)                ║" << std::endl;
    std::cout << "║  - Right: Depth stream (hardware timestamp)               ║" << std::endl;
    std::cout << "║  - Overlays show timestamp type (HW-TS vs SW-TS)          ║" << std::endl;
    std::cout << "║                                                            ║" << std::endl;
    std::cout << "║  MIDDLE SECTION: Timestamp visualization                  ║" << std::endl;
    std::cout << "║  - Shows native timestamps from each sensor               ║" << std::endl;
    std::cout << "║  - [HW] = Hardware timestamp (RealSense)                  ║" << std::endl;
    std::cout << "║  - [SW] = Software timestamp (Raspberry Pi)               ║" << std::endl;
    std::cout << "║  - Time difference colored by sync quality                ║" << std::endl;
    std::cout << "║                                                            ║" << std::endl;
    std::cout << "║  BOTTOM SECTION: Statistics                                ║" << std::endl;
    std::cout << "║  - Frame counts and success rates                         ║" << std::endl;
    std::cout << "║  - Timestamp difference statistics                        ║" << std::endl;
    std::cout << "║  - Performance metrics (FPS)                              ║" << std::endl;
    std::cout << "║                                                            ║" << std::endl;
    std::cout << "║  TOP RIGHT: Sync quality indicator                        ║" << std::endl;
    std::cout << "║  - Green: Excellent (< 2ms)                               ║" << std::endl;
    std::cout << "║  - Yellow: Good (2-8ms)                                   ║" << std::endl;
    std::cout << "║  - Orange: Acceptable (8-16.67ms)                         ║" << std::endl;
    std::cout << "║  - Red: Poor (> 16.67ms)                                  ║" << std::endl;
    std::cout << "║                                                            ║" << std::endl;
    std::cout << "║  CONTROLS:                                                 ║" << std::endl;
    std::cout << "║  - Q or ESC: Quit                                          ║" << std::endl;
    std::cout << "║  - S: Save screenshot + log file                           ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════════╝\n" << std::endl;
    
    SynchronizedFrameSet frame_set;
    int consecutive_timeouts = 0;
    const int max_timeouts = 50;
    
    while (consecutive_timeouts < max_timeouts) {
        if (sensor.GetLatestSynchronizedFrames(frame_set)) {
            consecutive_timeouts = 0;
            
            // Visualize with native timestamp information
            visualizer.visualizeSync(frame_set, sensor.getSyncStats());
            
            char key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27) {
                std::cout << "\nExiting..." << std::endl;
                break;
            }
        } else {
            consecutive_timeouts++;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    if (consecutive_timeouts >= max_timeouts) {
        std::cout << "\nTimeout: No synchronized frames received" << std::endl;
    }
    
    // Cleanup
    sensor.StopSynchronizedCapture();
    sensor.Stop();
    
    // Show final statistics
    std::cout << "\n╔════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  FINAL SYNCHRONIZATION STATISTICS                          ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════════╝\n" << std::endl;
    sensor.printSyncStats(true);
    
    std::cout << "\nKey Observations:" << std::endl;
    std::cout << "  1. Both sensors provide high-quality timestamps" << std::endl;
    std::cout << "  2. Direct timestamp comparison is simple and effective" << std::endl;
    std::cout << "  3. No clock calibration needed with GLOBAL_TIME domain" << std::endl;
    std::cout << "  4. Sync quality depends on frame rate consistency" << std::endl;
}

int main() {
    try {
        runNativeTimestampSyncDemo();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}