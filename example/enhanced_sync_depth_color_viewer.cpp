/**
 * @file enhanced_sync_depth_color_viewer.cpp
 * @brief Enhanced synchronized depth and color frame viewer with hardware timestamp support
 */

#include "sensor/synchronized_camera.h"
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <thread>
#include <chrono>
#include <atomic>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace determ::sensor;
using namespace cv;

// Global application instance
std::unique_ptr<SynchronizedCameraApp> g_app;

void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    if (g_app) {
        g_app->Stop();
    }
    exit(0);
}

/**
 * @brief Enhanced Synchronized Frame Viewer with Hardware Timestamp Support
 */
class EnhancedSynchronizedFrameViewer {
private:
    // Current frames
    Mat current_depth_;
    Mat current_color_;
    
    // Display parameters
    int display_width_ = 640;
    int display_height_ = 480;
    int panel_height_ = 140;  // Increased for additional hardware timestamp info
    
    // Visualization modes
    enum DepthColorMode {
        DEPTH_GRAYSCALE = 0,
        DEPTH_COLORMAP_JET = 1,
        DEPTH_COLORMAP_HOT = 2,
        DEPTH_COLORMAP_COOL = 3,
        DEPTH_COLORMAP_RAINBOW = 4
    };
    DepthColorMode depth_mode_ = DEPTH_COLORMAP_JET;
    
    // Processing parameters
    float max_depth_display_ = 7.0f;  // Maximum depth for visualization (meters)
    float min_depth_display_ = 0.3f;  // Minimum depth for visualization (meters)
    bool auto_exposure_adjustment_ = false;
    
    // Statistics
    std::atomic<int> frame_count_{0};
    std::chrono::time_point<std::chrono::high_resolution_clock> last_stats_time_;
    float fps_ = 0.0f;
    double depth_min_val_ = 0.0;
    double depth_max_val_ = 0.0;
    int valid_depth_pixels_ = 0;
    int total_depth_pixels_ = 0;
    double avg_depth_ = 0.0;
    
    // Enhanced sync statistics with hardware timestamp info
    uint64_t last_frame_timestamp_ = 0;
    double frame_interval_ms_ = 0.0;
    int sync_drops_ = 0;
    int hardware_timestamp_frames_ = 0;
    int total_processed_frames_ = 0;
    double avg_sync_precision_ms_ = 0.0;
    double best_sync_precision_ms_ = 999.0;
    double worst_sync_precision_ms_ = 0.0;
    
public:
    EnhancedSynchronizedFrameViewer() {
        last_stats_time_ = std::chrono::high_resolution_clock::now();
        
        // Create main display window
        namedWindow("Enhanced Synchronized Depth & Color Viewer", WINDOW_AUTOSIZE);
        
        std::cout << "Enhanced Synchronized Frame Viewer initialized with hardware timestamp support" << std::endl;
        std::cout << "  Display resolution: " << display_width_ << "x" << display_height_ << std::endl;
    }
    
    ~EnhancedSynchronizedFrameViewer() {
        destroyAllWindows();
    }
    
    /**
     * @brief Process enhanced synchronized frames with hardware timestamp analysis
     */
    bool processFrames(const SynchronizedFrameSet& frame_set) {
        const auto* depth_frame = frame_set.getFrame("depth");
        const auto* color_frame = frame_set.getFrame("color");
        
        if (!depth_frame || !color_frame) {
            sync_drops_++;
            std::cerr << "Missing depth or color frame (drops: " << sync_drops_ << ")" << std::endl;
            return false;
        }
        
        total_processed_frames_++;
        
        // Track hardware timestamp usage
        if (frame_set.all_hardware_timestamped) {
            hardware_timestamp_frames_++;
        }
        
        // Update sync precision statistics
        updateSyncPrecisionStats(frame_set);
        
        // Convert frame data to OpenCV Mats
        current_depth_ = Mat(depth_frame->height, depth_frame->width, CV_16UC1, 
                            const_cast<void*>(reinterpret_cast<const void*>(depth_frame->data.data())));
        
        Mat rgb_mat(color_frame->height, color_frame->width, CV_8UC3,
           const_cast<void*>(reinterpret_cast<const void*>(color_frame->data.data())));
        current_color_ = rgb_mat.clone(); 
        
        // Resize frames to display resolution if needed
        if (current_color_.size() != cv::Size(display_width_, display_height_)) {
            resize(current_color_, current_color_, cv::Size(display_width_, display_height_));
        }
        if (current_depth_.size() != cv::Size(display_width_, display_height_)) {
            resize(current_depth_, current_depth_, cv::Size(display_width_, display_height_));
        }
        
        // Calculate frame statistics
        calculateDepthStatistics();
        
        // Update timing statistics using hardware timestamps when available
        uint64_t timestamp_us;
        if (frame_set.all_hardware_timestamped) {
            timestamp_us = frame_set.reference_timestamp_us;
        } else {
            // Fallback to software timestamp
            timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                depth_frame->timestamp.time_since_epoch()).count();
        }
        updateTimingStats(timestamp_us);
        
        // Update performance statistics
        updateStats();
        
        return true;
    }
    
    /**
     * @brief Update synchronization precision statistics
     */
    void updateSyncPrecisionStats(const SynchronizedFrameSet& frame_set) {
        // Update running average of sync precision
        double precision = frame_set.max_time_diff_ms;
        avg_sync_precision_ms_ = (avg_sync_precision_ms_ * (total_processed_frames_ - 1) + precision) / total_processed_frames_;
        
        // Track best and worst sync precision
        if (precision < best_sync_precision_ms_) {
            best_sync_precision_ms_ = precision;
        }
        if (precision > worst_sync_precision_ms_) {
            worst_sync_precision_ms_ = precision;
        }
    }
    
    /**
     * @brief Calculate depth frame statistics
     */
    void calculateDepthStatistics() {
        if (current_depth_.empty()) return;
    
        // Reset counters
        valid_depth_pixels_ = 0;
        total_depth_pixels_ = current_depth_.rows * current_depth_.cols;
        double depth_sum = 0.0;
        
        // Get min/max for the entire image
        minMaxLoc(current_depth_, &depth_min_val_, &depth_max_val_);
        
        // Count valid pixels using the same range as the working visualization (300-7000mm)
        for (int y = 0; y < current_depth_.rows; ++y) {
            for (int x = 0; x < current_depth_.cols; ++x) {
                uint16_t depth_mm = current_depth_.at<uint16_t>(y, x);
                // Use same range as the working unsynchronized version
                if (depth_mm > 300 && depth_mm < 7000) {
                    valid_depth_pixels_++;
                    depth_sum += depth_mm;
                }
            }
        }
        
        avg_depth_ = valid_depth_pixels_ > 0 ? depth_sum / valid_depth_pixels_ : 0.0;
    }
    
    /**
     * @brief Update timing statistics
     */
    void updateTimingStats(uint64_t current_timestamp) {
        if (last_frame_timestamp_ > 0) {
            frame_interval_ms_ = (current_timestamp - last_frame_timestamp_) / 1000.0; // us to ms
        }
        last_frame_timestamp_ = current_timestamp;
    }
    
    /**
     * @brief Create colorized depth image
     */
    Mat createColorizedDepth() {
        if (current_depth_.empty()) {
        return Mat::zeros(display_height_, display_width_, CV_8UC3);
        }
        
        Mat result;
        
        // Convert 16-bit depth to 8-bit with the same parameters
        current_depth_.convertTo(result, CV_8UC1, 255.0 / (7000 - 300), -300 * 255.0 / (7000 - 300));
        
        // Convert to 3-channel for consistency
        cvtColor(result, result, COLOR_GRAY2BGR);
        
        // Apply colormap for better depth visualization
        Mat depth_colored;
        cvtColor(result, depth_colored, COLOR_BGR2GRAY);
        
        // Apply the selected colormap
        switch (depth_mode_) {
            case DEPTH_GRAYSCALE:
                // Keep as BGR (already converted above)
                break;
            case DEPTH_COLORMAP_JET:
                applyColorMap(depth_colored, result, COLORMAP_JET);
                break;
            case DEPTH_COLORMAP_HOT:
                applyColorMap(depth_colored, result, COLORMAP_HOT);
                break;
            case DEPTH_COLORMAP_COOL:
                applyColorMap(depth_colored, result, COLORMAP_COOL);
                break;
            case DEPTH_COLORMAP_RAINBOW:
                applyColorMap(depth_colored, result, COLORMAP_RAINBOW);
                break;
        }
        
        return result;
    }
    
    /**
     * @brief Create enhanced status panel with hardware timestamp information
     */
    Mat createEnhancedStatusPanel() {
        Mat panel = Mat::zeros(panel_height_, display_width_ * 2, CV_8UC3);
        
        // Title
        putText(panel, "Enhanced Synchronized Depth & Color Viewer (Hardware Timestamps)", 
                cv::Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
        
        // Performance stats (left column)
        std::string fps_text = "FPS: " + std::to_string(static_cast<int>(fps_));
        putText(panel, fps_text, cv::Point(10, 45), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));
        
        std::string interval_text = "Frame Interval: " + std::to_string(frame_interval_ms_).substr(0, 5) + "ms";
        putText(panel, interval_text, cv::Point(10, 65), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));
        
        std::string drops_text = "Sync Drops: " + std::to_string(sync_drops_);
        Scalar drop_color = sync_drops_ > 0 ? Scalar(0, 0, 255) : Scalar(0, 255, 0);
        putText(panel, drops_text, cv::Point(10, 85), FONT_HERSHEY_SIMPLEX, 0.5, drop_color);
        
        // Hardware timestamp stats (left column, continued)
        double hw_percentage = total_processed_frames_ > 0 ? 
            (100.0 * hardware_timestamp_frames_ / total_processed_frames_) : 0.0;
        std::string hw_text = "HW Timestamps: " + std::to_string(static_cast<int>(hw_percentage)) + "%";
        Scalar hw_color = hw_percentage > 50 ? Scalar(0, 255, 0) : Scalar(0, 255, 255);
        putText(panel, hw_text, cv::Point(10, 105), FONT_HERSHEY_SIMPLEX, 0.5, hw_color);
        
        // Depth stats (middle column)
        std::string valid_pixels = "Valid Depth: " + std::to_string(valid_depth_pixels_) + "/" + std::to_string(total_depth_pixels_);
        putText(panel, valid_pixels, cv::Point(280, 45), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0));
        
        std::string avg_depth_text = "Avg Depth: " + std::to_string(avg_depth_ / 1000.0).substr(0, 4) + "m";
        putText(panel, avg_depth_text, cv::Point(280, 65), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0));
        
        std::string range_text = "Range: " + std::to_string(depth_min_val_/1000.0).substr(0, 4) + 
                                "-" + std::to_string(depth_max_val_/1000.0).substr(0, 4) + "m";
        putText(panel, range_text, cv::Point(280, 85), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0));
        
        // Synchronization precision stats (middle column, continued)
        std::string sync_precision = "Sync Precision: " + std::to_string(avg_sync_precision_ms_).substr(0, 4) + "ms";
        putText(panel, sync_precision, cv::Point(280, 105), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0));
        
        std::string best_sync = "Best: " + std::to_string(best_sync_precision_ms_).substr(0, 4) + "ms";
        putText(panel, best_sync, cv::Point(280, 125), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 255, 0));
        
        // Display settings (right column)
        std::string depth_mode_names[] = {"GRAYSCALE", "JET", "HOT", "COOL", "RAINBOW"};
        std::string mode_text = "Depth Mode: " + depth_mode_names[depth_mode_];
        putText(panel, mode_text, cv::Point(550, 45), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255));
        
        std::string range_setting = "Display Range: " + std::to_string(min_depth_display_).substr(0, 3) + 
                                   "-" + std::to_string(max_depth_display_).substr(0, 3) + "m";
        putText(panel, range_setting, cv::Point(550, 65), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255));
        
        // Frame processing info
        std::string processed_text = "Processed: " + std::to_string(total_processed_frames_);
        putText(panel, processed_text, cv::Point(550, 85), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255));
        
        // Controls (far right)
        putText(panel, "Controls: 1-5:Modes, +/-:Range, S:Stats, Q:Quit", 
                cv::Point(800, 45), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(200, 200, 200));
        
        putText(panel, "Hardware timestamping improves sync precision", 
                cv::Point(800, 65), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(100, 255, 100));
        
        return panel;
    }
    
    /**
     * @brief Display enhanced visualization with hardware timestamp info
     */
    void displayVisualization() {
        if (current_color_.empty() || current_depth_.empty()) {
            return;
        }
        
        // Create colorized depth
        Mat colorized_depth = createColorizedDepth();
        
        // Create combined display
        Mat combined_display = Mat::zeros(display_height_ + panel_height_, display_width_ * 2, CV_8UC3);
        
        // Copy color image (left side)
        current_color_.copyTo(combined_display(cv::Rect(0, 0, display_width_, display_height_)));
        
        // Copy depth image (right side)
        colorized_depth.copyTo(combined_display(cv::Rect(display_width_, 0, display_width_, display_height_)));
        
        // Add labels with hardware timestamp indicators
        putText(combined_display, "RGB Camera (Pi v3)", cv::Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255), 2);
        putText(combined_display, "Depth Camera (RealSense)", cv::Point(display_width_ + 10, 30), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255), 2);
        
        // Add hardware timestamp status indicators
        if (hardware_timestamp_frames_ > 0) {
            putText(combined_display, "HW SYNC", cv::Point(10, display_height_ - 10), 
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
            putText(combined_display, "HW SYNC", cv::Point(display_width_ + 10, display_height_ - 10), 
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
        }
        
        // Add enhanced status panel
        Mat status_panel = createEnhancedStatusPanel();
        status_panel.copyTo(combined_display(cv::Rect(0, display_height_, display_width_ * 2, panel_height_)));
        
        // Show combined display
        imshow("Enhanced Synchronized Depth & Color Viewer", combined_display);
        
        // Handle keyboard input
        int key = waitKey(1) & 0xFF;
        handleKeyboard(key);
    }
    
    /**
     * @brief Handle keyboard input
     */
    void handleKeyboard(int key) {
        switch (key) {
            case '1':
                depth_mode_ = DEPTH_GRAYSCALE;
                std::cout << "Depth mode: GRAYSCALE" << std::endl;
                break;
            case '2':
                depth_mode_ = DEPTH_COLORMAP_JET;
                std::cout << "Depth mode: JET" << std::endl;
                break;
            case '3':
                depth_mode_ = DEPTH_COLORMAP_HOT;
                std::cout << "Depth mode: HOT" << std::endl;
                break;
            case '4':
                depth_mode_ = DEPTH_COLORMAP_COOL;
                std::cout << "Depth mode: COOL" << std::endl;
                break;
            case '5':
                depth_mode_ = DEPTH_COLORMAP_RAINBOW;
                std::cout << "Depth mode: RAINBOW" << std::endl;
                break;
                
            case '+':
            case '=':
                max_depth_display_ = std::min(max_depth_display_ + 0.5f, 10.0f);
                std::cout << "Max depth range: " << max_depth_display_ << "m" << std::endl;
                break;
                
            case '-':
                max_depth_display_ = std::max(max_depth_display_ - 0.5f, 1.0f);
                std::cout << "Max depth range: " << max_depth_display_ << "m" << std::endl;
                break;
                
            case 's':
            case 'S':
                printDetailedStats();
                break;
                
            case 'a':
            case 'A':
                auto_exposure_adjustment_ = !auto_exposure_adjustment_;
                std::cout << "Auto exposure adjustment: " << (auto_exposure_adjustment_ ? "ON" : "OFF") << std::endl;
                break;
                
            case 'q':
            case 'Q':
            case 27: // ESC
                std::cout << "\nQuitting..." << std::endl;
                if (g_app) {
                    g_app->Stop();
                }
                exit(0);
                break;
        }
    }
    
    /**
     * @brief Update performance statistics
     */
    void updateStats() {
        frame_count_++;
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_stats_time_);
        
        if (duration.count() > 1000) {
            fps_ = frame_count_ * 1000.0f / duration.count();
            frame_count_ = 0;
            last_stats_time_ = now;
        }
    }
    
    /**
     * @brief Print detailed statistics including hardware timestamp analysis
     */
    void printDetailedStats() {
        std::cout << "\n=== Enhanced Synchronized Frame Viewer Statistics ===" << std::endl;
        std::cout << "Performance:" << std::endl;
        std::cout << "  FPS: " << fps_ << std::endl;
        std::cout << "  Frame interval: " << frame_interval_ms_ << "ms" << std::endl;
        std::cout << "  Sync drops: " << sync_drops_ << std::endl;
        
        std::cout << "\nHardware Timestamp Analysis:" << std::endl;
        std::cout << "  Total processed frames: " << total_processed_frames_ << std::endl;
        std::cout << "  Hardware timestamped frames: " << hardware_timestamp_frames_ << std::endl;
        double hw_percentage = total_processed_frames_ > 0 ? 
            (100.0 * hardware_timestamp_frames_ / total_processed_frames_) : 0.0;
        std::cout << "  Hardware timestamp usage: " << std::fixed << std::setprecision(1) 
                  << hw_percentage << "%" << std::endl;
        
        std::cout << "\nSynchronization Precision:" << std::endl;
        std::cout << "  Average sync precision: " << std::fixed << std::setprecision(3) 
                  << avg_sync_precision_ms_ << "ms" << std::endl;
        std::cout << "  Best sync precision: " << best_sync_precision_ms_ << "ms" << std::endl;
        std::cout << "  Worst sync precision: " << worst_sync_precision_ms_ << "ms" << std::endl;
        
        std::cout << "\nDepth Analysis:" << std::endl;
        std::cout << "  Valid depth pixels: " << valid_depth_pixels_ << "/" << total_depth_pixels_ 
                  << " (" << std::fixed << std::setprecision(1) 
                  << (100.0f * valid_depth_pixels_ / total_depth_pixels_) << "%)" << std::endl;
        std::cout << "  Average depth: " << std::fixed << std::setprecision(3) 
                  << (avg_depth_ / 1000.0) << "m" << std::endl;
        std::cout << "  Depth range: " << (depth_min_val_ / 1000.0) << "m - " 
                  << (depth_max_val_ / 1000.0) << "m" << std::endl;
        std::cout << "================================================" << std::endl;
    }
    
    /**
     * @brief Get hardware timestamp usage percentage
     */
    double getHardwareTimestampUsage() const {
        return total_processed_frames_ > 0 ? 
            (100.0 * hardware_timestamp_frames_ / total_processed_frames_) : 0.0;
    }
    
    /**
     * @brief Get average synchronization precision
     */
    double getAverageSyncPrecision() const {
        return avg_sync_precision_ms_;
    }
};

/**
 * @brief Main application
 */
int main() {
    std::cout << "=== Enhanced Synchronized Depth & Color Frame Viewer ===" << std::endl;
    std::cout << "Real-time visualization with hardware timestamp support" << std::endl;
    std::cout << "DS1307 RTC + RealSense hardware timestamps for improved sync precision" << std::endl;
    std::cout << "=======================================================" << std::endl;
    
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        // Create enhanced configuration with hardware timestamp optimization
        SyncConfig config;
        config.enabled_streams = {"color", "depth"};
        
        // Color sensor (Pi Camera v3) - optimized for hardware timestamps
        config.color.width = 640;
        config.color.height = 480;
        config.color.format = "RGB888";
        config.color.auto_exposure = true;
        config.color.auto_focus = true;
        
        // Depth sensor (RealSense D421) - hardware timestamps enabled
        config.depth.width = 640;
        config.depth.height = 480;
        config.depth.auto_exposure = true;
        config.depth.laser_power = 100.0f;  // Increased for better depth quality
        
        // Enhanced sync settings for hardware timestamps
        config.sync.tolerance_ms = 10.0;           // Tighter tolerance with hardware timestamps
        config.sync.hw_tolerance_ms = 2.0;         // Very tight for pure hardware sync
        config.sync.prefer_hardware_timestamps = true;
        config.sync.buffer_size = 15;
        config.sync.enable_stats = true;
        
        // Performance optimizations
        config.performance.high_priority = true;
        config.performance.queue_size = 10;
        
        // Create enhanced application and viewer
        g_app = std::make_unique<SynchronizedCameraApp>(config);
        EnhancedSynchronizedFrameViewer viewer;
        
        // Initialize and check hardware timestamp capabilities
        std::cout << "\nInitializing enhanced camera system..." << std::endl;
        if (g_app->Initialize() != 0) {
            std::cerr << "Failed to initialize cameras" << std::endl;
            return -1;
        }
        
        // Check and report hardware timestamp capabilities
        bool hw_available = g_app->isHardwareTimestampingAvailable();
        std::cout << "Hardware timestamping: " << (hw_available ? "Available" : "Not available") << std::endl;
        
        if (hw_available) {
            std::cout << "Attempting to sync RTC with system time..." << std::endl;
            if (g_app->syncRTCWithSystemTime()) {
                std::cout << "RTC synchronized successfully" << std::endl;
            } else {
                std::cout << "RTC sync failed or not available" << std::endl;
            }
        }
        
        // Print camera information
        g_app->printCameraInfo();
        
        if (g_app->Start() != 0) {
            std::cerr << "Failed to start capture" << std::endl;
            return -1;
        }
        
        std::cout << "Cameras started successfully!" << std::endl;
        std::cout << "\nControls:" << std::endl;
        std::cout << "  1-5 - Switch depth colorization modes" << std::endl;
        std::cout << "  +/- - Adjust depth display range" << std::endl;
        std::cout << "  S   - Print detailed statistics" << std::endl;
        std::cout << "  A   - Toggle auto exposure adjustment" << std::endl;
        std::cout << "  Q/ESC - Quit application" << std::endl;
        std::cout << "\nStarting enhanced real-time frame visualization..." << std::endl;
        
        // Variables for frame handling and timing
        SynchronizedFrameSet frame_set;
        auto last_print_time = std::chrono::high_resolution_clock::now();
        const auto print_interval = std::chrono::seconds(15); // Print stats every 15 seconds
        int total_frame_count = 0;
        
        // Main processing loop
        while (true) {
            // Get synchronized frame set with timeout
            bool got_frame = g_app->captureFrame(frame_set, 100); // 100ms timeout
            
            if (got_frame) {
                // Process the enhanced synchronized frames
                if (viewer.processFrames(frame_set)) {
                    // Display enhanced visualization
                    viewer.displayVisualization();
                    total_frame_count++;
                    
                    // Print periodic statistics with hardware timestamp analysis
                    auto now = std::chrono::high_resolution_clock::now();
                    if (now - last_print_time >= print_interval) {
                        std::cout << "\n=== Periodic Status Update ===" << std::endl;
                        std::cout << "Total frames processed: " << total_frame_count << std::endl;
                        std::cout << "Hardware timestamp usage: " << std::fixed << std::setprecision(1) 
                                  << viewer.getHardwareTimestampUsage() << "%" << std::endl;
                        std::cout << "Average sync precision: " << std::fixed << std::setprecision(3)
                                  << viewer.getAverageSyncPrecision() << "ms" << std::endl;
                        
                        // Print application-level sync statistics
                        g_app->printStats(false);
                        last_print_time = now;
                    }
                } else {
                    std::cerr << "Failed to process frame set" << std::endl;
                }
            } else {
                // Timeout or no frames available
                // Handle UI events during timeout
                int key = cv::waitKey(1) & 0xFF;
                if (key == 'q' || key == 'Q' || key == 27) { // ESC key
                    std::cout << "\nUser requested quit..." << std::endl;
                    break;
                }
                
                // Small delay to prevent excessive CPU usage
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            
            // Check for application termination
            if (!g_app) {
                std::cout << "Application stopped" << std::endl;
                break;
            }
        }
        
        // Cleanup
        std::cout << "\nCleaning up..." << std::endl;
        g_app->Stop();
        
        // Final comprehensive statistics
        std::cout << "\n=== Final Enhanced Statistics ===" << std::endl;
        std::cout << "Total frames processed: " << total_frame_count << std::endl;
        viewer.printDetailedStats();
        
        // Print final application-level statistics
        std::cout << "\nApplication-level synchronization statistics:" << std::endl;
        g_app->printStats(true);
        
        // Hardware timestamp effectiveness analysis
        const auto& stats = g_app->getStats();
        if (stats.hardware_timestamped_frames > 0) {
            double hw_percentage = 100.0 * stats.hardware_timestamped_frames / stats.total_frames_synchronized;
            std::cout << "\nHardware Timestamp Effectiveness Analysis:" << std::endl;
            std::cout << "  Hardware timestamp usage: " << std::fixed << std::setprecision(1) 
                      << hw_percentage << "%" << std::endl;
            
            if (stats.avg_hw_sync_time_ms < stats.avg_sync_time_ms) {
                double improvement = ((stats.avg_sync_time_ms - stats.avg_hw_sync_time_ms) / stats.avg_sync_time_ms) * 100.0;
                std::cout << "  Sync precision improvement: " << std::setprecision(1) 
                          << improvement << "% better with hardware timestamps" << std::endl;
            }
            
            std::cout << "  Best hardware sync: " << std::setprecision(3) 
                      << stats.min_hw_sync_time_ms << "ms" << std::endl;
            std::cout << "  Best overall sync: " << stats.min_sync_time_ms << "ms" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Exception in main: " << e.what() << std::endl;
        
        if (g_app) {
            g_app->Stop();
        }
        
        return -1;
    } catch (...) {
        std::cerr << "Unknown exception in main!" << std::endl;
        
        if (g_app) {
            g_app->Stop();
        }
        
        return -1;
    }
    
    std::cout << "Enhanced application completed successfully!" << std::endl;
    return 0;
}