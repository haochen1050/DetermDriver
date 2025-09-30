// Enhanced determ_sensor.h with integrated synchronization algorithms

#pragma once

#include "sensor/raspberry_sensor.h"
#include "sensor/realsense_sensor.h"
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
#include <algorithm>
#include <numeric>

namespace determ {
namespace sensor {

// Forward declarations
class RealsenseSensor;
class RaspberrySensor;

/**
 * @brief Enhanced timestamped frame with better synchronization support
 */
struct TimestampedFrame {
    std::string stream_name;
    std::vector<uint8_t> data;
    
    // Timestamp information
    std::chrono::high_resolution_clock::time_point software_timestamp;
    double hardware_timestamp_ms = 0.0;
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
    
    // Set hardware timestamp (typically from RealSense)
    void setHardwareTimestamp(double hw_timestamp_ms) {
        hardware_timestamp_ms = hw_timestamp_ms;
        has_hardware_timestamp = true;
    }
};

/**
 * @brief Enhanced synchronized frame set with better statistics
 */
struct SynchronizedFrameSet {
    std::map<std::string, TimestampedFrame> frames;
    std::chrono::high_resolution_clock::time_point sync_timestamp;
    double avg_timestamp_ms = 0.0;
    double max_time_diff_ms = 0.0;
    uint64_t sync_id = 0;
    uint64_t reference_timestamp_us = 0;
    uint64_t frameset_sequence = 0;
    
    SynchronizedFrameSet();
    
    bool hasStream(const std::string& stream_name) const;
    const TimestampedFrame* getFrame(const std::string& stream_name) const;
    void calculateSyncStats();
    
    // Get primary timestamp for compatibility
    double getPrimaryTimestampMs() const {
        return avg_timestamp_ms;
    }
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
 * @brief Synchronization modes
 */
enum class SyncMode {
    SOFTWARE_SYNC,   // Use system timestamps only
    HARDWARE_SYNC,   // Use hardware timestamps with clock calibration
    HYBRID_SYNC      // Use hardware when available, fallback to software
};

/**
 * @brief Enhanced DetermSensor with integrated synchronization algorithms
 */
class DetermSensor {
public:
    using FrameCallback = std::function<void(const SynchronizedFrameSet&)>;
    
    // Valid stream names
    // static const std::vector<std::string> kMValidStreams;
    const std::vector<std::string> kMValidStreams = {"color", "depth", "infrared1", "infrared2"};

    DetermSensor();
    ~DetermSensor();

    // Core lifecycle methods
    int Initialize();
    int Start();
    int Stop();

    // Stream management
    int EnableStream(const std::vector<std::string>& streams);
    bool IsValidStream(const std::string& stream_name) const;

    // Synchronization configuration
    bool SetSynchronizationMode(SyncMode mode);
    bool SetSyncTolerance(double tolerance_ms);
    bool SetSyncBufferSize(size_t buffer_size);
    
    // Enhanced frame capture with synchronization algorithms
    SynchronizedFrameSet GetSynchronizedFrames(double timeout_ms = 50.0);
    bool GetLatestSynchronizedFrames(SynchronizedFrameSet& frameset);
    
    // Continuous synchronized capture
    bool StartSynchronizedCapture(SyncMode mode = SyncMode::HYBRID_SYNC);
    bool StopSynchronizedCapture();
    
    // Frame callback for real-time processing
    void setFrameCallback(FrameCallback callback) { m_frame_callback = callback; }

    // Sensor configuration
    int SetColorSensorConfig(int width, int height, const std::string& format);
    int SetDepthSensorConfig(int width, int height);

    // Sensor controls
    bool SetColorSensorControl(bool is_auto, float exposure_time = 10000.0f, float white_balance = 1.0f);
    bool SetColorFocusControl(bool auto_focus, float manual_focus_position = 0.0f);
    bool TriggerColorAutofocus();
    bool RefreshColorAutofocus();
    bool SetDepthSensorControl(bool auto_exposure, float manual_exposure = 8500.0f, float gain = 16.0f);
    bool SetInfraredSensorControl(bool auto_exposure, float manual_exposure = 8500.0f, float gain = 16.0f);
    bool SetLaserPower(float power);
    bool SetDepthUnits(float units);

    // Frame capture (legacy interface)
    std::map<std::string, std::vector<uint8_t>> GetFrame();

    // Information and diagnostics
    void PrintSupportedFormats();
    void PrintAvailableControls();
    void PrintColorFocusControls();
    std::string GetColorSensorInfo() const;
    std::string GetDepthSensorInfo() const;
    
    // Statistics and monitoring
    const SyncStats& getSyncStats() const { return m_sync_stats; }
    void printSyncStats(bool detailed = false) const { m_sync_stats.printStats(detailed); }
    void resetSyncStats();

    // Status queries
    bool IsRunning() const { return m_is_running; }
    bool IsColorSensorEnabled() const { return m_color_enabled; }
    bool IsDepthSensorEnabled() const { return m_depth_enabled; }
    bool IsInfraredSensorEnabled() const { return m_infrared_enabled; }

private:
    // Initialization helpers
    int InitializeColorSensor();
    int InitializeDepthSensor();
    int StartColorSensor();
    int StartDepthSensor();
    int StopColorSensor();
    int StopDepthSensor();

    // Synchronization algorithms
    bool CaptureFramesWithSoftwareSync(SynchronizedFrameSet& frameset);
    bool CaptureFramesWithHardwareSync(SynchronizedFrameSet& frameset);
    bool CaptureFramesWithHybridSync(SynchronizedFrameSet& frameset);
    
    // Advanced synchronization methods
    void addFrameToBuffer(TimestampedFrame&& frame);
    bool findSynchronizedFrames(SynchronizedFrameSet& frame_set);
    void cleanupOldFrames();
    
    // Continuous capture thread
    void SynchronizedCaptureThread();

    // Sensor instances
    std::unique_ptr<RaspberrySensor> m_raspberry_sensor;
    std::unique_ptr<RealsenseSensor> m_realsense_sensor;

    // State management
    std::atomic<bool> m_is_initialized;
    std::atomic<bool> m_is_running;
    std::atomic<bool> m_sync_capture_active;

    // Stream configuration
    std::vector<std::string> m_enabled_streams;
    bool m_color_enabled;
    bool m_depth_enabled;
    bool m_infrared_enabled;

    // Sensor configuration
    int m_color_width, m_color_height;
    std::string m_color_format;
    bool m_color_configured;
    int m_depth_width, m_depth_height;
    bool m_depth_configured;

    // Synchronization configuration
    SyncMode m_sync_mode;
    double m_sync_tolerance_ms;
    size_t m_sync_buffer_size;

    // Frame buffering for advanced synchronization
    std::map<std::string, std::queue<TimestampedFrame>> m_frame_buffers;
    std::map<std::string, std::pair<int, int>> m_stream_dimensions;
    std::map<std::string, std::string> m_stream_formats;
    mutable std::mutex m_buffer_mutex;

    // Synchronized capture management
    std::thread m_sync_capture_thread;
    SynchronizedFrameSet m_latest_frameset;
    std::atomic<bool> m_new_frameset_available;
    mutable std::mutex m_latest_frameset_mutex;
    std::condition_variable m_frameset_cv;
    std::atomic<uint64_t> m_frameset_sequence;

    // Frame callback
    FrameCallback m_frame_callback;

    // Statistics
    mutable SyncStats m_sync_stats;
};

} // namespace sensor
} // namespace determ