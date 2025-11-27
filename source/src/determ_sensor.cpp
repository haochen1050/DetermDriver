#include "sensor/determ_sensor.h"
#include <cmath>
#include <numeric>
#include <algorithm>


using determ::capture::DetermSensor;
using determ::capture::SynchronizedFrameSet;
using determ::capture::SyncMode;
using determ::capture::TimestampedFrame;
using determ::capture::SyncStats;
using determ::sensor::RaspberrySensor;
using determ::sensor::RealsenseSensor;
// // Valid stream names
// const std::vector<std::string> DetermSensor::kMValidStreams = {
//     "color", "depth", "infrared1", "infrared2"
// };

// TimestampedFrame implementation
TimestampedFrame::TimestampedFrame() 
    : frame_number(0), width(0), height(0) {}

TimestampedFrame::TimestampedFrame(const std::string& name, std::vector<uint8_t>&& frame_data, 
                                 int w, int h, const std::string& fmt)
    : stream_name(name), data(std::move(frame_data)), 
      software_timestamp(std::chrono::high_resolution_clock::now()),
      width(w), height(h), format(fmt) {
    static std::atomic<uint64_t> global_frame_counter{0};
    frame_number = ++global_frame_counter;
}

// SynchronizedFrameSet implementation
SynchronizedFrameSet::SynchronizedFrameSet() 
    : avg_timestamp_ms(0), max_time_diff_ms(0), sync_id(0), reference_timestamp_us(0) {}

bool SynchronizedFrameSet::hasStream(const std::string& stream_name) const {
    return frames.find(stream_name) != frames.end();
}

const TimestampedFrame* SynchronizedFrameSet::getFrame(const std::string& stream_name) const {
    auto it = frames.find(stream_name);
    return (it != frames.end()) ? &it->second : nullptr;
}

void SynchronizedFrameSet::calculateSyncStats() {
    if (frames.empty()) return;
    
    uint64_t sum_timestamp = 0;
    uint64_t min_timestamp = std::numeric_limits<uint64_t>::max();
    uint64_t max_timestamp = 0;
    
    for (const auto& [name, frame] : frames) {
        uint64_t ts = frame.getTimestampUs();
        sum_timestamp += ts;
        min_timestamp = std::min(min_timestamp, ts);
        max_timestamp = std::max(max_timestamp, ts);
    }
    
    reference_timestamp_us = sum_timestamp / frames.size();
    avg_timestamp_ms = reference_timestamp_us / 1000.0;
    max_time_diff_ms = static_cast<double>(max_timestamp - min_timestamp) / 1000.0;
    
    static std::atomic<uint64_t> sync_counter{0};
    sync_id = ++sync_counter;
}

// SyncStats implementation
SyncStats::SyncStats() {
    start_time = last_update = std::chrono::high_resolution_clock::now();
}

void SyncStats::updateStats(const SynchronizedFrameSet& frame_set) {
    total_frames_synchronized++;
    
    if (frame_set.max_time_diff_ms < min_sync_time_ms) {
        min_sync_time_ms = frame_set.max_time_diff_ms;
    }
    if (frame_set.max_time_diff_ms > max_sync_time_ms) {
        max_sync_time_ms = frame_set.max_time_diff_ms;
    }
    
    // Update running average
    avg_sync_time_ms = (avg_sync_time_ms * (total_frames_synchronized - 1) + 
                       frame_set.max_time_diff_ms) / total_frames_synchronized;
    
    // Update per-stream counters
    for (const auto& [stream_name, frame] : frame_set.frames) {
        frames_per_stream[stream_name]++;
    }
}

void SyncStats::calculateFPS() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
    
    if (duration.count() > 0) {
        double seconds = duration.count() / 1000.0;
        capture_fps = total_frames_captured / seconds;
        sync_fps = total_frames_synchronized / seconds;
    }
    
    last_update = now;
}

void SyncStats::printStats(bool detailed) const {
    std::cout << "\nðŸ“Š === Synchronization Statistics ===" << std::endl;
    std::cout << "   Captured: " << total_frames_captured << " frames" << std::endl;
    std::cout << "   Synchronized: " << total_frames_synchronized << " frames" << std::endl;
    std::cout << "   Dropped: " << total_frames_dropped << " frames" << std::endl;
    std::cout << "   Sync rate: " << std::fixed << std::setprecision(1) 
              << (100.0 * total_frames_synchronized / std::max(1UL, total_frames_captured)) << "%" << std::endl;
    
    std::cout << "   Sync timing: avg=" << std::setprecision(2) << avg_sync_time_ms 
              << "ms, min=" << min_sync_time_ms 
              << "ms, max=" << max_sync_time_ms << "ms" << std::endl;
    
    std::cout << "   Performance: capture=" << std::setprecision(1) << capture_fps 
              << " FPS, sync=" << sync_fps << " FPS" << std::endl;
    
    if (detailed) {
        std::cout << "   Per-stream frames:" << std::endl;
        for (const auto& [stream, count] : frames_per_stream) {
            std::cout << "     " << stream << ": " << count << std::endl;
        }
    }
    std::cout << "=============================================" << std::endl;
}

// DetermSensor Implementation
DetermSensor::DetermSensor()
    : m_is_initialized(false),
      m_is_running(false),
      m_sync_capture_active(false),
      m_color_enabled(false),
      m_depth_enabled(false),
      m_infrared_enabled(false),
      m_color_width(1920),
      m_color_height(1080),
      m_color_format("YUYV"),
      m_color_configured(false),
      m_depth_width(640),
      m_depth_height(480),
      m_depth_configured(false),
      m_sync_mode(SyncMode::HYBRID_SYNC),
      m_sync_tolerance_ms(16.67),
      m_sync_buffer_size(30),
      m_new_frameset_available(false),
      m_frameset_sequence(0)
{
    // Create sensor instances
    m_raspberry_sensor = std::make_unique<RaspberrySensor>();
    m_realsense_sensor = std::make_unique<RealsenseSensor>();
}

DetermSensor::~DetermSensor() {
    StopSynchronizedCapture();
    Stop();

    // Give system time to complete cleanup
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Reset sensor instances
    m_raspberry_sensor.reset();
    m_realsense_sensor.reset();

    std::cout << "DetermSensor destroyed" << std::endl;
}


int DetermSensor::Initialize()
{
  if (m_is_initialized) {
    std::cout << "DetermSensor already initialized" << std::endl;
    return 0;
  }

  std::cout << "Initializing DetermSensor..." << std::endl;

  // Initialize both sensors
  int color_result = InitializeColorSensor();
  int depth_result = InitializeDepthSensor();

  // Check if at least one sensor initialized successfully
  if (color_result != 0 && depth_result != 0) {
    std::cerr << "Failed to initialize any sensors" << std::endl;
    return -1;
  }

  m_is_initialized = true;
  std::cout << "DetermSensor initialized successfully" << std::endl;

  // Print initialization summary
  std::cout << "Sensor initialization summary:" << std::endl;
  std::cout << "  Color sensor (Raspberry Pi): " << (color_result == 0 ? "âœ“ Success" : "âœ— Failed") << std::endl;
  std::cout << "  Depth sensor (RealSense): " << (depth_result == 0 ? "âœ“ Success" : "âœ— Failed") << std::endl;

  return 0;
}

int DetermSensor::InitializeColorSensor()
{
  if (!m_raspberry_sensor) {
    return -1;
  }

  int ret = m_raspberry_sensor->Initialize();
  if (ret == 0) {
    std::cout << "Color sensor (Raspberry Pi Camera) initialized successfully" << std::endl;
  } else {
    std::cerr << "Failed to initialize color sensor: " << ret << std::endl;
  }

  return ret;
}

int DetermSensor::InitializeDepthSensor()
{
  if (!m_realsense_sensor) {
    return -1;
  }

  int ret = m_realsense_sensor->Initialize();
  if (ret == 0) {
    std::cout << "Depth sensor (RealSense) initialized successfully" << std::endl;
  } else {
    std::cerr << "Failed to initialize depth sensor: " << ret << std::endl;
  }

  return ret;
}

bool DetermSensor::IsValidStream(const std::string& stream_name) const
{
  return std::find(kMValidStreams.begin(), kMValidStreams.end(), stream_name) != kMValidStreams.end();
}

int DetermSensor::EnableStream(const std::vector<std::string>& streams)
{
  if (m_is_running) {
    std::cerr << "Cannot change stream configuration while sensors are running" << std::endl;
    return -1;
  }

  if (!m_is_initialized) {
    std::cerr << "Sensors not initialized. Call Initialize() first." << std::endl;
    return -1;
  }

  // Validate all streams first
  for (const auto& stream : streams) {
    if (!IsValidStream(stream)) {
      std::cerr << "Invalid stream type: " << stream << std::endl;
      std::cerr << "Valid streams are: ";
      for (const auto& valid : kMValidStreams) {
        std::cerr << valid << " ";
      }
      std::cerr << std::endl;
      return -1;
    }
  }

  std::cout << "Enabling streams: ";
  for (const auto& stream : streams) {
    std::cout << stream << " ";
  }
  std::cout << std::endl;

  // Reset stream flags
  m_color_enabled = false;
  m_depth_enabled = false;
  m_infrared_enabled = false;

  // Set stream flags based on enabled streams
  for (const auto& stream : streams) {
    if (stream == "color") {
      m_color_enabled = true;
    } else if (stream == "depth") {
      m_depth_enabled = true;
      m_infrared_enabled = false;  // Ensure we don't have conflicting streams
    } else if (stream == "infrared1" || stream == "infrared2") {
      m_infrared_enabled = true;
    }
  }

  m_enabled_streams = streams;

  std::cout << "Stream configuration:" << std::endl;
  std::cout << "  Color stream: " << (m_color_enabled ? "enabled" : "disabled") << std::endl;
  std::cout << "  Depth stream: " << (m_depth_enabled ? "enabled" : "disabled") << std::endl;
  std::cout << "  Infrared streams: " << (m_infrared_enabled ? "enabled" : "disabled") << std::endl;

  return 0;
}

bool DetermSensor::SetSynchronizationMode(SyncMode mode) {
    if (m_sync_capture_active) {
        std::cerr << "Cannot change sync mode while synchronized capture is active" << std::endl;
        return false;
    }
    
    m_sync_mode = mode;
    std::cout << "Synchronization mode set to: ";
    switch (mode) {
        case SyncMode::SOFTWARE_SYNC:
            std::cout << "SOFTWARE_SYNC" << std::endl;
            break;
        case SyncMode::HARDWARE_SYNC:
            std::cout << "HARDWARE_SYNC" << std::endl;
            break;
        case SyncMode::HYBRID_SYNC:
            std::cout << "HYBRID_SYNC" << std::endl;
            break;
    }
    return true;
}

bool DetermSensor::SetSyncTolerance(double tolerance_ms) {
    if (tolerance_ms <= 0 || tolerance_ms > 1000) {
        std::cerr << "Invalid sync tolerance: " << tolerance_ms << " ms" << std::endl;
        return false;
    }
    
    m_sync_tolerance_ms = tolerance_ms;
    std::cout << "Sync tolerance set to: " << tolerance_ms << " ms" << std::endl;
    return true;
}

bool DetermSensor::SetSyncBufferSize(size_t buffer_size) {
    if (buffer_size < 5 || buffer_size > 1000) {
        std::cerr << "Invalid buffer size: " << buffer_size << std::endl;
        return false;
    }
    
    m_sync_buffer_size = buffer_size;
    std::cout << "Sync buffer size set to: " << buffer_size << std::endl;
    return true;
}

// bool DetermSensor::CalibrateClocks() {
//     if (!m_is_running) {
//         std::cerr << "Sensors must be running to calibrate clocks" << std::endl;
//         return false;
//     }
    
//     if (m_sync_mode == SyncMode::SOFTWARE_SYNC) {
//         std::cout << "Clock calibration not needed for SOFTWARE_SYNC mode" << std::endl;
//         return true;
//     }
    
//     // With RealSense using RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME, 
//     // timestamps are already in OS system clock domain
//     std::cout << "RealSense using GLOBAL_TIME domain - no calibration needed" << std::endl;
//     m_clock_sync.m_is_calibrated = true;
//     return true;
// }

// INTEGRATED SYNCHRONIZATION ALGORITHMS
SynchronizedFrameSet DetermSensor::GetSynchronizedFrames(double timeout_ms) {
    SynchronizedFrameSet frameset;
    frameset.frameset_sequence = ++m_frameset_sequence;
    
    if (!m_is_running) {
        std::cerr << "Sensors not running" << std::endl;
        return frameset;
    }
    
    bool success = false;
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::duration<double, std::milli>(timeout_ms);
    
    while (!success && (std::chrono::steady_clock::now() - start_time < timeout)) {
        switch (m_sync_mode) {
            case SyncMode::SOFTWARE_SYNC:
                success = CaptureFramesWithSoftwareSync(frameset);
                break;
            case SyncMode::HARDWARE_SYNC:
                success = CaptureFramesWithHardwareSync(frameset);
                break;
            case SyncMode::HYBRID_SYNC:
                success = CaptureFramesWithHybridSync(frameset);
                break;
        }
        
        if (!success) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    // Update statistics if successful
    if (success && !frameset.frames.empty()) {
        m_sync_stats.updateStats(frameset);
    }
    
    return frameset;
}

bool DetermSensor::CaptureFramesWithSoftwareSync(SynchronizedFrameSet& frameset) {
    // Capture timestamp as close to frame acquisition as possible
    auto capture_timestamp = std::chrono::high_resolution_clock::now();
    
    std::vector<std::pair<std::string, std::chrono::high_resolution_clock::time_point>> timestamps;
    
    // Capture from color sensor (Raspberry Pi)
    if (m_color_enabled && m_raspberry_sensor) {
        auto color_frame = m_raspberry_sensor->GetFrame();
        if (!color_frame.empty()) {
            // Get dimensions and format for color stream
            auto dim_it = m_stream_dimensions.find("color");
            auto fmt_it = m_stream_formats.find("color");
            
            int width = (dim_it != m_stream_dimensions.end()) ? dim_it->second.first : m_color_width;
            int height = (dim_it != m_stream_dimensions.end()) ? dim_it->second.second : m_color_height;
            std::string format = (fmt_it != m_stream_formats.end()) ? fmt_it->second : m_color_format;
            
            TimestampedFrame frame("color", std::move(color_frame), width, height, format);
            frame.software_timestamp = capture_timestamp;
            frameset.frames["color"] = std::move(frame);
            timestamps.emplace_back("color", capture_timestamp);
            
            m_sync_stats.total_frames_captured++;
        }
    }
    
    // Capture from depth/IR sensors (RealSense)
    if ((m_depth_enabled || m_infrared_enabled) && m_realsense_sensor) {
        auto rs_frames = m_realsense_sensor->GetFrame();
        
        for (const auto& [stream_name, frame_data] : rs_frames) {
            if (std::find(m_enabled_streams.begin(), m_enabled_streams.end(), stream_name) != m_enabled_streams.end()) {
                // Get dimensions and format
                auto dim_it = m_stream_dimensions.find(stream_name);
                auto fmt_it = m_stream_formats.find(stream_name);
                
                int width = (dim_it != m_stream_dimensions.end()) ? dim_it->second.first : m_depth_width;
                int height = (dim_it != m_stream_dimensions.end()) ? dim_it->second.second : m_depth_height;
                std::string format = (fmt_it != m_stream_formats.end()) ? fmt_it->second : 
                                   (stream_name == "depth" ? "Z16" : "Y8");
                
                TimestampedFrame frame(stream_name, std::vector<uint8_t>(frame_data), width, height, format);
                frame.software_timestamp = capture_timestamp;
                frameset.frames[stream_name] = std::move(frame);
                timestamps.emplace_back(stream_name, capture_timestamp);
                
                m_sync_stats.total_frames_captured++;
            }
        }
    }
    
    if (timestamps.empty()) {
        return false;
    }
    
    // Calculate synchronization statistics
    auto min_time = std::min_element(timestamps.begin(), timestamps.end(),
        [](const auto& a, const auto& b) { return a.second < b.second; })->second;
    auto max_time = std::max_element(timestamps.begin(), timestamps.end(),
        [](const auto& a, const auto& b) { return a.second < b.second; })->second;
    
    frameset.sync_timestamp = min_time + (max_time - min_time) / 2;
    frameset.max_time_diff_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
        max_time - min_time).count();
    
    frameset.calculateSyncStats();
    
    return frameset.max_time_diff_ms <= m_sync_tolerance_ms;
}

bool DetermSensor::CaptureFramesWithHardwareSync(SynchronizedFrameSet& frameset) {
    std::vector<std::pair<std::string, std::chrono::high_resolution_clock::time_point>> timestamps;
    
    // Capture from color sensor (software timestamp)
    if (m_color_enabled && m_raspberry_sensor) {
        auto rpi_frame = m_raspberry_sensor->GetTimestampedFrame();
        if (!rpi_frame.data.empty()) {
            auto dim_it = m_stream_dimensions.find("color");
            auto fmt_it = m_stream_formats.find("color");
            
            int width = (dim_it != m_stream_dimensions.end()) ? dim_it->second.first : m_color_width;
            int height = (dim_it != m_stream_dimensions.end()) ? dim_it->second.second : m_color_height;
            std::string format = (fmt_it != m_stream_formats.end()) ? fmt_it->second : m_color_format;
            
            TimestampedFrame frame("color", std::move(rpi_frame.data), width, height, format);
            frame.software_timestamp = rpi_frame.timestamp;
            frameset.frames["color"] = std::move(frame);
            timestamps.emplace_back("color", rpi_frame.timestamp);
            
            m_sync_stats.total_frames_captured++;
        }
    }
    
    // Capture from depth/IR sensors (hardware timestamp already in GLOBAL_TIME domain)
    if ((m_depth_enabled || m_infrared_enabled) && m_realsense_sensor) {
        auto rs_frameset = m_realsense_sensor->GetTimestampedFrameSet();
        
        for (const auto& [stream_name, rs_frame] : rs_frameset.frames) {
            if (std::find(m_enabled_streams.begin(), m_enabled_streams.end(), stream_name) != m_enabled_streams.end()) {
                auto dim_it = m_stream_dimensions.find(stream_name);
                auto fmt_it = m_stream_formats.find(stream_name);
                
                int width = (dim_it != m_stream_dimensions.end()) ? dim_it->second.first : m_depth_width;
                int height = (dim_it != m_stream_dimensions.end()) ? dim_it->second.second : m_depth_height;
                std::string format = (fmt_it != m_stream_formats.end()) ? fmt_it->second : 
                                   (stream_name == "depth" ? "Z16" : "Y8");
                
                TimestampedFrame frame(stream_name, std::vector<uint8_t>(rs_frame.data), width, height, format);
                frame.setHardwareTimestamp(rs_frame.timestamp_ms);
                
                // Since RealSense uses GLOBAL_TIME domain, convert directly to system time_point
                auto hardware_duration_us = std::chrono::microseconds(static_cast<int64_t>(rs_frame.timestamp_ms * 1000.0));
                frame.software_timestamp = std::chrono::high_resolution_clock::time_point(hardware_duration_us);
                
                frameset.frames[stream_name] = std::move(frame);
                timestamps.emplace_back(stream_name, frame.software_timestamp);
                
                m_sync_stats.total_frames_captured++;
            }
        }
    }
    
    if (timestamps.empty()) {
        return false;
    }
    
    // Calculate synchronization statistics using timestamps
    auto min_time = std::min_element(timestamps.begin(), timestamps.end(),
        [](const auto& a, const auto& b) { return a.second < b.second; })->second;
    auto max_time = std::max_element(timestamps.begin(), timestamps.end(),
        [](const auto& a, const auto& b) { return a.second < b.second; })->second;
    
    frameset.sync_timestamp = min_time + (max_time - min_time) / 2;
    frameset.max_time_diff_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
        max_time - min_time).count();
    
    frameset.calculateSyncStats();
    
    return frameset.max_time_diff_ms <= m_sync_tolerance_ms;
}

bool DetermSensor::CaptureFramesWithHybridSync(SynchronizedFrameSet& frameset) {
    // Since RealSense uses GLOBAL_TIME domain, hardware sync should always work
    // Fall back to software sync only if there's an error
    if (CaptureFramesWithHardwareSync(frameset)) {
        return true;
    } else {
        return CaptureFramesWithSoftwareSync(frameset);
    }
}

// ADVANCED BUFFERED SYNCHRONIZATION METHODS
void DetermSensor::addFrameToBuffer(TimestampedFrame&& frame) {
    std::lock_guard<std::mutex> lock(m_buffer_mutex);
    
    auto& buffer = m_frame_buffers[frame.stream_name];
    
    // Limit buffer size
    while (buffer.size() >= m_sync_buffer_size) {
        buffer.pop();
        m_sync_stats.total_frames_dropped++;
    }
    
    buffer.push(std::move(frame));
}

bool DetermSensor::findSynchronizedFrames(SynchronizedFrameSet& frame_set) {
    std::lock_guard<std::mutex> lock(m_buffer_mutex);
    
    // Check if we have frames in all required streams
    for (const auto& stream : m_enabled_streams) {
        if (m_frame_buffers[stream].empty()) {
            return false;
        }
    }
    
    // Use the first enabled stream as reference
    std::map<std::string, TimestampedFrame> candidate_frames;
    
    const auto& reference_stream = m_enabled_streams[0];
    auto& reference_buffer = m_frame_buffers[reference_stream];
    const auto& reference_frame = reference_buffer.front();
    candidate_frames[reference_stream] = reference_frame;
    
    // Use the primary timestamp of reference frame
    double reference_timestamp_ms = reference_frame.getPrimaryTimestampMs();
    
    bool valid_sync = true;
    
    // Find best matching frames for other streams
    for (size_t i = 1; i < m_enabled_streams.size(); ++i) {
        const auto& stream_name = m_enabled_streams[i];
        auto& buffer = m_frame_buffers[stream_name];
        
        TimestampedFrame* best_match = nullptr;
        double min_time_diff = std::numeric_limits<double>::max();
        
        // Search through buffer for best temporal match
        std::queue<TimestampedFrame> temp_queue;
        while (!buffer.empty()) {
            TimestampedFrame current_frame = buffer.front();
            buffer.pop();
            
            // Compare using primary timestamps
            double current_timestamp_ms = current_frame.getPrimaryTimestampMs();
            double time_diff = std::abs(current_timestamp_ms - reference_timestamp_ms);
            
            if (time_diff < min_time_diff && time_diff <= m_sync_tolerance_ms) {
                if (best_match) {
                    temp_queue.push(*best_match);
                }
                best_match = new TimestampedFrame(std::move(current_frame));
                min_time_diff = time_diff;
            } else {
                temp_queue.push(current_frame);
            }
        }
        
        // Restore remaining frames to buffer
        buffer = std::move(temp_queue);
        
        if (best_match) {
            candidate_frames[stream_name] = *best_match;
            delete best_match;
        } else {
            valid_sync = false;
            break;
        }
    }
    
    if (valid_sync) {
        // Remove used reference frame
        if (!m_frame_buffers[reference_stream].empty()) {
            m_frame_buffers[reference_stream].pop();
        }
        
        frame_set.frames = std::move(candidate_frames);
        frame_set.calculateSyncStats();
        return true;
    }
    
    return false;
}

void DetermSensor::cleanupOldFrames() {
    std::lock_guard<std::mutex> lock(m_buffer_mutex);
    
    auto now = std::chrono::high_resolution_clock::now();
    double max_age_ms = m_sync_tolerance_ms * 3;
    
    for (auto& [stream_name, buffer] : m_frame_buffers) {
        std::queue<TimestampedFrame> cleaned_buffer;
        
        while (!buffer.empty()) {
            const auto& frame = buffer.front();
            double age_ms = std::chrono::duration<double, std::milli>(
                now - frame.software_timestamp).count();
            
            if (age_ms < max_age_ms) {
                cleaned_buffer.push(frame);
            } else {
                m_sync_stats.total_frames_dropped++;
            }
            
            buffer.pop();
        }
        
        buffer = std::move(cleaned_buffer);
    }
}

// CONTINUOUS SYNCHRONIZED CAPTURE
bool DetermSensor::StartSynchronizedCapture(SyncMode mode) {
    if (m_sync_capture_active) {
        std::cout << "Synchronized capture already active" << std::endl;
        return true;
    }
    
    if (!m_is_running) {
        std::cerr << "Sensors must be running to start synchronized capture" << std::endl;
        return false;
    }
    
    SetSynchronizationMode(mode);
    
    // Initialize frame buffers
    for (const auto& stream : m_enabled_streams) {
        m_frame_buffers[stream] = std::queue<TimestampedFrame>();
    }
    
    // Reset statistics
    m_sync_stats = SyncStats();
    
    m_sync_capture_active = true;
    m_sync_capture_thread = std::thread(&DetermSensor::SynchronizedCaptureThread, this);
    
    std::cout << "Synchronized capture started with mode: ";
    switch (m_sync_mode) {
        case SyncMode::SOFTWARE_SYNC: std::cout << "SOFTWARE_SYNC"; break;
        case SyncMode::HARDWARE_SYNC: std::cout << "HARDWARE_SYNC"; break;
        case SyncMode::HYBRID_SYNC: std::cout << "HYBRID_SYNC"; break;
    }
    std::cout << " (RealSense GLOBAL_TIME domain)" << std::endl;
    
    return true;
}

bool DetermSensor::StopSynchronizedCapture() {
    if (!m_sync_capture_active) {
        return true;
    }
    
    m_sync_capture_active = false;
    m_frameset_cv.notify_all();
    
    if (m_sync_capture_thread.joinable()) {
        m_sync_capture_thread.join();
    }
    
    // Clear frame buffers
    {
        std::lock_guard<std::mutex> lock(m_buffer_mutex);
        m_frame_buffers.clear();
    }
    
    // Print final statistics
    m_sync_stats.calculateFPS();
    std::cout << "Synchronized capture stopped" << std::endl;
    printSyncStats(true);
    
    return true;
}

void DetermSensor::SynchronizedCaptureThread() {
    std::cout << "Synchronized capture thread started" << std::endl;
    
    while (m_sync_capture_active) {
        try {
            // Get synchronized frames using the selected algorithm
            auto frameset = GetSynchronizedFrames(50.0);
            
            if (!frameset.frames.empty()) {
                // Store latest frameset
                {
                    std::lock_guard<std::mutex> lock(m_latest_frameset_mutex);
                    m_latest_frameset = std::move(frameset);
                    m_new_frameset_available = true;
                }
                m_frameset_cv.notify_all();
                
                // Call user callback if registered
                if (m_frame_callback) {
                    m_frame_callback(m_latest_frameset);
                }
            }
            
            // Periodic cleanup of old frames
            static int cleanup_counter = 0;
            if (++cleanup_counter % 100 == 0) {
                cleanupOldFrames();
            }
            
        } catch (const std::exception& e) {
            if (m_sync_capture_active) {
                std::cerr << "Synchronized capture thread error: " << e.what() << std::endl;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    std::cout << "Synchronized capture thread stopped" << std::endl;
}

bool DetermSensor::GetLatestSynchronizedFrames(SynchronizedFrameSet& frameset) {
    std::unique_lock<std::mutex> lock(m_latest_frameset_mutex);
    
    if (!m_new_frameset_available.load()) {
        if (!m_frameset_cv.wait_for(lock, std::chrono::milliseconds(100), 
                                   [this] { return m_new_frameset_available.load(); })) {
            return false;
        }
    }
    
    frameset = m_latest_frameset;
    m_new_frameset_available.store(false);
    return true;
}

void DetermSensor::resetSyncStats() {
    m_sync_stats = SyncStats();
    std::cout << "Synchronization statistics reset" << std::endl;
}


int DetermSensor::SetColorSensorConfig(int width, int height, const std::string& format)
{
  if (m_is_running) {
    std::cerr << "Cannot change color sensor configuration while running" << std::endl;
    return -1;
  }

  if (!m_color_enabled) {
    std::cerr << "Color stream not enabled. Call EnableStream() first." << std::endl;
    return -1;
  }

  if (!m_raspberry_sensor) {
    std::cerr << "Color sensor not available" << std::endl;
    return -1;
  }

  std::cout << "Setting color sensor configuration: " << width << "x" << height << " " << format << std::endl;

  int ret = m_raspberry_sensor->SetSensorConfig(width, height, format);
  if (ret == 0) {
    m_color_width = width;
    m_color_height = height;
    m_color_format = format;
    m_color_configured = true;
    std::cout << "Color sensor configured successfully" << std::endl;
  } else {
    std::cerr << "Failed to configure color sensor: " << ret << std::endl;
  }

  return ret;
}

int DetermSensor::SetDepthSensorConfig(int width, int height)
{
  if (m_is_running) {
    std::cerr << "Cannot change depth sensor configuration while running" << std::endl;
    return -1;
  }

  if (!m_depth_enabled && !m_infrared_enabled) {
    std::cerr << "Depth or infrared streams not enabled. Call EnableStream() first." << std::endl;
    return -1;
  }

  if (!m_realsense_sensor) {
    std::cerr << "Depth sensor not available" << std::endl;
    return -1;
  }

  std::cout << "Setting depth sensor configuration: " << width << "x" << height << std::endl;

  // Build stream list for RealSense
  std::vector<std::string> realsense_streams;
  for (const auto& stream : m_enabled_streams) {
    if (stream == "depth" || stream == "infrared1" || stream == "infrared2") {
      realsense_streams.push_back(stream);
    }
  }

  if (realsense_streams.empty()) {
    std::cerr << "No depth/infrared streams to configure" << std::endl;
    return -1;
  }

  int ret = m_realsense_sensor->SetSensorConfig(width, height, realsense_streams);
  if (ret == 0) {
    m_depth_width = width;
    m_depth_height = height;
    m_depth_configured = true;
    std::cout << "Depth sensor configured successfully" << std::endl;
  } else {
    std::cerr << "Failed to configure depth sensor: " << ret << std::endl;
  }

  return ret;
}

bool DetermSensor::SetColorSensorControl(bool is_auto, float exposure_time, float white_balance)
{
  if (!m_color_enabled || !m_raspberry_sensor) {
    std::cerr << "Color sensor not available or not enabled" << std::endl;
    return false;
  }

  std::cout << "Setting color sensor controls - Auto: " << (is_auto ? "true" : "false") << std::endl;
  return m_raspberry_sensor->SetSensorColorControl(is_auto, exposure_time, white_balance);
}

bool DetermSensor::SetColorFocusControl(bool auto_focus, float manual_focus_position)
{
  if (!m_color_enabled || !m_raspberry_sensor) {
    std::cerr << "Color sensor not available or not enabled" << std::endl;
    return false;
  }

  std::cout << "Setting color sensor focus control - Auto: " << (auto_focus ? "true" : "false") << std::endl;
  return m_raspberry_sensor->SetFocusControl(auto_focus, manual_focus_position);
}

bool DetermSensor::TriggerColorAutofocus()
{
  if (!m_color_enabled || !m_raspberry_sensor) {
    std::cerr << "Color sensor not available or not enabled" << std::endl;
    return false;
  }

  return m_raspberry_sensor->TriggerAutofocus();
}

bool DetermSensor::RefreshColorAutofocus()
{
  if (!m_color_enabled || !m_raspberry_sensor) {
    std::cerr << "Color sensor not available or not enabled" << std::endl;
    return false;
  }

  return m_raspberry_sensor->RefreshAutofocus();
}

bool DetermSensor::SetDepthSensorControl(bool auto_exposure, float manual_exposure, float gain)
{
  if (!m_depth_enabled || !m_realsense_sensor) {
    std::cerr << "Depth sensor not available or not enabled" << std::endl;
    return false;
  }

  std::cout << "Setting depth sensor controls - Auto exposure: " << (auto_exposure ? "true" : "false") << std::endl;
  return m_realsense_sensor->SetDepthControl(auto_exposure, manual_exposure, gain);
}

bool DetermSensor::SetInfraredSensorControl(bool auto_exposure, float manual_exposure, float gain)
{
  if (!m_infrared_enabled || !m_realsense_sensor) {
    std::cerr << "Infrared sensor not available or not enabled" << std::endl;
    return false;
  }

  std::cout << "Setting infrared sensor controls - Auto exposure: " << (auto_exposure ? "true" : "false") << std::endl;
  return m_realsense_sensor->SetInfraredControl(auto_exposure, manual_exposure, gain);
}

bool DetermSensor::SetLaserPower(float power)
{
  if (!m_realsense_sensor) {
    std::cerr << "RealSense sensor not available" << std::endl;
    return false;
  }

  return m_realsense_sensor->SetLaserPower(power);
}

bool DetermSensor::SetDepthUnits(float units)
{
  if (!m_realsense_sensor) {
    std::cerr << "RealSense sensor not available" << std::endl;
    return false;
  }

  return m_realsense_sensor->SetDepthUnits(units);
}

int DetermSensor::Start()
{
  if (m_is_running) {
    std::cout << "DetermSensor is already running" << std::endl;
    return 0;
  }

  if (!m_is_initialized) {
    std::cerr << "Sensors not initialized. Call Initialize() first." << std::endl;
    return -1;
  }

  if (m_enabled_streams.empty()) {
    std::cerr << "No streams enabled. Call EnableStream() first." << std::endl;
    return -1;
  }

  std::cout << "Starting DetermSensor..." << std::endl;

  int color_result = 0;
  int depth_result = 0;

  // Start color sensor if enabled
  if (m_color_enabled) {
    color_result = StartColorSensor();
  }

  // Start depth sensor if enabled
  if (m_depth_enabled || m_infrared_enabled) {
    depth_result = StartDepthSensor();
  }

  // Check if at least one sensor started successfully
  bool color_success = !m_color_enabled || (color_result == 0);
  bool depth_success = !(m_depth_enabled || m_infrared_enabled) || (depth_result == 0);

  if (!color_success && !depth_success) {
    std::cerr << "Failed to start any enabled sensors" << std::endl;
    return -1;
  }

  m_is_running = true;
  std::cout << "DetermSensor started successfully" << std::endl;

  // Print start summary
  std::cout << "Sensor start summary:" << std::endl;
  if (m_color_enabled) {
    std::cout << "  Color sensor: " << (color_result == 0 ? "âœ“ Success" : "âœ— Failed") << std::endl;
  }
  if (m_depth_enabled || m_infrared_enabled) {
    std::cout << "  Depth/IR sensor: " << (depth_result == 0 ? "âœ“ Success" : "âœ— Failed") << std::endl;
  }

  return 0;
}

int DetermSensor::StartColorSensor()
{
  if (!m_raspberry_sensor || !m_color_configured) {
    std::cerr << "Color sensor not configured. Call SetColorSensorConfig() first." << std::endl;
    return -1;
  }

  int ret = m_raspberry_sensor->Start();
  if (ret == 0) {
    std::cout << "Color sensor started successfully" << std::endl;
  } else {
    std::cerr << "Failed to start color sensor: " << ret << std::endl;
  }

  return ret;
}

int DetermSensor::StartDepthSensor()
{
  if (!m_realsense_sensor || !m_depth_configured) {
    std::cerr << "Depth sensor not configured. Call SetDepthSensorConfig() first." << std::endl;
    return -1;
  }

  int ret = m_realsense_sensor->Start();
  if (ret == 0) {
    std::cout << "Depth sensor started successfully" << std::endl;
  } else {
    std::cerr << "Failed to start depth sensor: " << ret << std::endl;
  }

  return ret;
}

int DetermSensor::Stop()
{
  if (!m_is_running) {
    return 0;
  }

  std::cout << "Stopping DetermSensor..." << std::endl;

  int color_result = 0;
  int depth_result = 0;

  // Stop color sensor if enabled
  if (m_color_enabled) {
    color_result = StopColorSensor();
  }

  // Stop depth sensor if enabled
  if (m_depth_enabled || m_infrared_enabled) {
    depth_result = StopDepthSensor();
  }

  m_is_running = false;
  std::cout << "DetermSensor stopped successfully" << std::endl;

  // Print stop summary
  std::cout << "Sensor stop summary:" << std::endl;
  if (m_color_enabled) {
    std::cout << "  Color sensor: " << (color_result == 0 ? "âœ“ Success" : "âœ— Failed") << std::endl;
  }
  if (m_depth_enabled || m_infrared_enabled) {
    std::cout << "  Depth/IR sensor: " << (depth_result == 0 ? "âœ“ Success" : "âœ— Failed") << std::endl;
  }

  return (color_result == 0 && depth_result == 0) ? 0 : -1;
}

int DetermSensor::StopColorSensor()
{
  if (!m_raspberry_sensor) {
    return 0;
  }

  int ret = m_raspberry_sensor->Stop();
  if (ret == 0) {
    std::cout << "Color sensor stopped successfully" << std::endl;
  } else {
    std::cerr << "Failed to stop color sensor: " << ret << std::endl;
  }

  return ret;
}

int DetermSensor::StopDepthSensor()
{
  if (!m_realsense_sensor) {
    return 0;
  }

  int ret = m_realsense_sensor->Stop();
  if (ret == 0) {
    std::cout << "Depth sensor stopped successfully" << std::endl;
  } else {
    std::cerr << "Failed to stop depth sensor: " << ret << std::endl;
  }

  return ret;
}



std::map<std::string, std::vector<uint8_t>> DetermSensor::GetFrame() {
    std::map<std::string, std::vector<uint8_t>> combined_frames;
    
    if (!IsRunning()) {
        std::cerr << "DetermSensor is not running. Call Start() first." << std::endl;
        return combined_frames;
    }
    
    try {
        // Get frame from RaspberrySensor (color stream)
        if (m_raspberry_sensor && IsColorSensorEnabled()) {
            auto color_frame = m_raspberry_sensor->GetFrame();
            if (!color_frame.empty()) {
                combined_frames["color"] = std::move(color_frame);
            }
        }
        
        // Get frames from RealsenseSensor (depth, infrared streams)
        if (m_realsense_sensor && (IsDepthSensorEnabled() || IsInfraredSensorEnabled())) {
            auto depth_frames = m_realsense_sensor->GetFrame();
            
            for (const auto& [stream_name, frame_data] : depth_frames) {
                if (std::find(m_enabled_streams.begin(), m_enabled_streams.end(), stream_name) != m_enabled_streams.end()) {
                    combined_frames[stream_name] = frame_data;
                }
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error capturing frames: " << e.what() << std::endl;
    }
    
    return combined_frames;
}

// std::map<std::string, double> DetermSensor::getLastHardwareTimestamps() const {
//     std::map<std::string, double> timestamps;
    
//     // Get hardware timestamps from RealSense if available
//     if (m_realsense_sensor && (IsDepthSensorEnabled() || IsInfraredSensorEnabled()) && IsRunning()) {
//         auto realsense_timestamps = m_realsense_sensor->getLastHardwareTimestamps();
//         timestamps.insert(realsense_timestamps.begin(), realsense_timestamps.end());
//     }
    
//     // RaspberrySensor (color stream) doesn't provide hardware timestamps
//     return timestamps;
// }

void DetermSensor::PrintSupportedFormats()
{
  std::cout << "\n========== DETERM SENSOR CAPABILITIES ==========" << std::endl;

  if (m_raspberry_sensor) {
    std::cout << "\n--- COLOR SENSOR (RASPBERRY PI CAMERA) ---" << std::endl;
    m_raspberry_sensor->PrintSupportedFormats();
  }

  if (m_realsense_sensor) {
    std::cout << "\n--- DEPTH SENSOR (REALSENSE) ---" << std::endl;
    m_realsense_sensor->PrintSupportedFormats();
  }

  std::cout << "===============================================\n" << std::endl;
}

void DetermSensor::PrintAvailableControls()
{
  std::cout << "\n========== DETERM SENSOR CONTROLS ==========" << std::endl;

  if (m_raspberry_sensor) {
    std::cout << "\n--- COLOR SENSOR CONTROLS ---" << std::endl;
    m_raspberry_sensor->PrintAvailableControls();
  }

  if (m_realsense_sensor) {
    std::cout << "\n--- DEPTH SENSOR CONTROLS ---" << std::endl;
    m_realsense_sensor->PrintAvailableControls();
  }

  std::cout << "==========================================\n" << std::endl;
}

void DetermSensor::PrintColorFocusControls()
{
  if (m_raspberry_sensor) {
    std::cout << "\n--- COLOR SENSOR FOCUS CONTROLS ---" << std::endl;
    m_raspberry_sensor->PrintFocusControls();
  } else {
    std::cout << "Color sensor not available" << std::endl;
  }
}

std::string DetermSensor::GetColorSensorInfo() const
{
  if (!m_raspberry_sensor) {
    return "Color sensor not available";
  }

  return "Raspberry Pi Camera - Resolution: " + std::to_string(m_color_width) + "x" + std::to_string(m_color_height) +
         ", Format: " + m_color_format;
}

std::string DetermSensor::GetDepthSensorInfo() const
{
  if (!m_realsense_sensor) {
    return "Depth sensor not available";
  }

  return m_realsense_sensor->GetDeviceInfo() + " - Resolution: " + std::to_string(m_depth_width) + "x" +
         std::to_string(m_depth_height);
}