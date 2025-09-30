#pragma once

#include "determ_sensor.h"
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <map>
#include <vector>
#include <memory>
#include <functional>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>

namespace determ {
namespace sensor {

/**
 * @brief Timestamped frame data structure with software timestamps only
 */
struct TimestampedFrame {
    std::string stream_name;
    std::vector<uint8_t> data;
    
    // Timestamp information
    std::chrono::high_resolution_clock::time_point software_timestamp;
    double hardware_timestamp_ms = 0.0;  // Hardware timestamp from sensor
    bool has_hardware_timestamp = false;
    
    uint64_t frame_number;
    int width, height;
    std::string format;
    
    TimestampedFrame();
    TimestampedFrame(const std::string& name, std::vector<uint8_t>&& frame_data, 
                    int w, int h, const std::string& fmt);
    
    // Get the primary timestamp to use for synchronization
    double getPrimaryTimestampMs() const {
        return has_hardware_timestamp ? hardware_timestamp_ms : getSoftwareTimestampMs();
    }
    
    // Get software timestamp in milliseconds
    double getSoftwareTimestampMs() const {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            software_timestamp.time_since_epoch()).count() / 1000.0;
    }
    
    // Get timestamp in microseconds since epoch (for compatibility)
    uint64_t getTimestampUs() const {
        if (has_hardware_timestamp) {
            return static_cast<uint64_t>(hardware_timestamp_ms * 1000.0);
        } else {
            return std::chrono::duration_cast<std::chrono::microseconds>(
                software_timestamp.time_since_epoch()).count();
        }
    }
    
    // Get timestamp in milliseconds for compatibility
    double getTimestampMs() const {
        return getPrimaryTimestampMs();
    }
    
    // Set hardware timestamp (typically from RealSense)
    void setHardwareTimestamp(double hw_timestamp_ms) {
        hardware_timestamp_ms = hw_timestamp_ms;
        has_hardware_timestamp = true;
    }
};

/**
 * @brief Synchronized frame set
 */
struct SynchronizedFrameSet {
    std::map<std::string, TimestampedFrame> frames;
    double avg_timestamp_ms;
    double max_time_diff_ms;
    uint64_t sync_id;
    uint64_t reference_timestamp_us;
    
    SynchronizedFrameSet();
    
    bool hasStream(const std::string& stream_name) const;
    const TimestampedFrame* getFrame(const std::string& stream_name) const;
    void calculateSyncStats();
};

/**
 * @brief Configuration for synchronized camera application
 */
struct SyncConfig {
    // Stream configuration
    std::vector<std::string> enabled_streams;
    
    // Color sensor configuration
    struct {
        int width = 1280;
        int height = 720;
        std::string format = "RGB888";
        bool auto_exposure = true;
        float manual_exposure = 10000.0f;
        float white_balance = 1.0f;
        bool auto_focus = true;
        float manual_focus = 0.0f;
    } color;
    
    // Depth sensor configuration
    struct {
        int width = 640;
        int height = 480;
        bool auto_exposure = true;
        float manual_exposure = 5000.0f;
        float gain = 1.0f;
        float laser_power = 50.0f;
        float depth_units = 0.001f;
    } depth;
    
    // Synchronization parameters
    struct {
        double tolerance_ms = 33.0;         // Tolerance for frame synchronization in milliseconds
        size_t buffer_size = 30;
        int timeout_ms = 1000;
        bool drop_unsynced = true;
        bool enable_stats = true;
    } sync;
    
    // Performance parameters
    struct {
        int capture_threads = 2;
        bool high_priority = true;
        int queue_size = 10;
    } performance;
};

/**
 * @brief Synchronization statistics
 */
struct SyncStats {
    uint64_t total_frames_captured = 0;
    uint64_t total_frames_synchronized = 0;
    uint64_t total_frames_dropped = 0;
    
    double avg_sync_time_ms = 0.0;
    double min_sync_time_ms = std::numeric_limits<double>::max();
    double max_sync_time_ms = 0.0;
    
    double capture_fps = 0.0;
    double sync_fps = 0.0;
    
    std::map<std::string, uint64_t> frames_per_stream;
    
    std::chrono::high_resolution_clock::time_point start_time;
    std::chrono::high_resolution_clock::time_point last_update;
    
    SyncStats();
    void updateStats(const SynchronizedFrameSet& frame_set);
    void calculateFPS();
    void printStats(bool detailed = false) const;
};

/**
 * @brief Synchronized camera application class with software timestamping
 */
class SynchronizedCameraApp {
public:
    using FrameCallback = std::function<void(const SynchronizedFrameSet&)>;
    
    explicit SynchronizedCameraApp(const SyncConfig& config = SyncConfig{});
    ~SynchronizedCameraApp();
    
    // Main application lifecycle
    int Initialize();
    int Start();
    int Stop();
    
    // Configuration methods
    void setConfig(const SyncConfig& config) { config_ = config; }
    const SyncConfig& getConfig() const { return config_; }
    
    // Frame callback registration
    void setFrameCallback(FrameCallback callback) { frame_callback_ = callback; }
    
    // Synchronous frame capture
    bool captureFrame(SynchronizedFrameSet& frame_set, int timeout_ms = 1000);
    
    // Statistics and monitoring
    const SyncStats& getStats() const { return stats_; }
    void printStats(bool detailed = false) const { stats_.printStats(detailed); }
    void resetStats();
    
    // Camera control methods
    bool refreshColorAutofocus();
    bool setColorExposure(bool auto_mode, float exposure = 10000.0f, float white_balance = 1.0f);
    bool setDepthExposure(bool auto_mode, float exposure = 5000.0f, float gain = 1.0f);
    bool setLaserPower(float power);
    
    // Utility methods
    void printCameraInfo() const;
    void printSupportedFormats() const;
    bool isRunning() const { return is_running_; }

private:
    // Internal methods
    int initializeSensor();
    int configureSensor();
    void captureWorker();
    void synchronizationWorker();
    
    // Frame synchronization
    void addFrameToBuffer(TimestampedFrame&& frame);
    bool findSynchronizedFrames(SynchronizedFrameSet& frame_set);
    void cleanupOldFrames();
    
    // Thread management
    void setThreadPriority();
    void startWorkerThreads();
    void stopWorkerThreads();
    
    // Member variables
    SyncConfig config_;
    std::unique_ptr<DetermSensor> sensor_;
    
    // Threading
    std::atomic<bool> is_running_{false};
    std::atomic<bool> is_initialized_{false};
    std::vector<std::thread> worker_threads_;
    
    // Frame synchronization
    std::map<std::string, std::queue<TimestampedFrame>> frame_buffers_;
    std::queue<SynchronizedFrameSet> synchronized_frames_;
    
    // Thread synchronization
    mutable std::mutex buffer_mutex_;
    mutable std::mutex sync_mutex_;
    std::condition_variable sync_cv_;
    
    // Callback and statistics
    FrameCallback frame_callback_;
    mutable SyncStats stats_;
    
    // Frame dimensions (updated during configuration)
    std::map<std::string, std::pair<int, int>> stream_dimensions_;
    std::map<std::string, std::string> stream_formats_;
};

} // namespace sensor
} // namespace determ