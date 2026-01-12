#include "sensor/determ_sensor.h"
#include <cmath>
#include <numeric>
#include <algorithm>


using determ::capture::DetermSensor;
using determ::capture::SynchronizedFrameSet;
using determ::capture::TimestampedFrame;
using determ::capture::SyncStats;
using determ::sensor::RaspberrySensor;
using determ::sensor::RealsenseSensor;
using determ::capture::CalibrationData;

// ============================================================================
// TimestampedFrame implementation
// ============================================================================

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

// ============================================================================
// SynchronizedFrameSet implementation
// ============================================================================

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

// ============================================================================
// SyncStats implementation
// ============================================================================

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
    std::cout << "\n === Synchronization Statistics ===" << std::endl;
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

// ============================================================================
// DetermSensor Implementation
// ============================================================================

DetermSensor::DetermSensor()
    : m_is_initialized(false),
      m_is_running(false),
      m_sync_capture_active(false),
      m_color_enabled(false),
      m_depth_enabled(false),
      m_color_width(0),
      m_color_height(0),
      m_color_format("YUYV"),
      m_color_configured(false),
      m_depth_width(0),
      m_depth_height(0),
      m_depth_configured(false),
      m_sync_tolerance_ms(16.67),
      m_sync_buffer_size(30),
      m_new_frameset_available(false),
      m_frameset_sequence(0),
      m_has_calibration(false)
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

// ============================================================================
// Initialization
// ============================================================================

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
  std::cout << "  Color sensor (Raspberry Pi): " << (color_result == 0 ? "Success" : " Failed") << std::endl;
  std::cout << "  Depth sensor (RealSense): " << (depth_result == 0 ? "Success" : " Failed") << std::endl;

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

// ============================================================================
// Stream Management
// ============================================================================

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

  // Set stream flags based on enabled streams
  for (const auto& stream : streams) {
    if (stream == "color") {
      m_color_enabled = true;
    } else if (stream == "depth" || stream == "infrared1" || stream == "infrared2") {
      m_depth_enabled = true;
    }
  }

  m_enabled_streams = streams;

  std::cout << "Stream configuration:" << std::endl;
  std::cout << "  Color stream: " << (m_color_enabled ? "enabled" : "disabled") << std::endl;
  std::cout << "  Depth/IR streams: " << (m_depth_enabled ? "enabled" : "disabled") << std::endl;

  return 0;
}

// ============================================================================
// Synchronization Configuration
// ============================================================================

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

// ============================================================================
// ADVANCED BUFFERED SYNCHRONIZED FRAME CAPTURE 
// ============================================================================

SynchronizedFrameSet DetermSensor::GetSynchronizedFrames(double timeout_ms) {
    SynchronizedFrameSet frameset;
    frameset.frameset_sequence = ++m_frameset_sequence;
    
    if (!m_is_running) {
        std::cerr << "Sensors not running" << std::endl;
        return frameset;
    }
    
    auto start_time = std::chrono::steady_clock::now();
    auto timeout_duration = std::chrono::duration<double, std::milli>(timeout_ms);
    
    // Strategy: Keep capturing and buffering frames until we find a good match
    // This handles variable frame arrival times and jitter
    
    while (std::chrono::steady_clock::now() - start_time < timeout_duration) {
        
        // 1. Capture new frames and add to buffers
        captureAndBufferFrames();
        
        // 2. Try to find synchronized frames from buffers
        if (findSynchronizedFrames(frameset)) {
            // Success! We found a synchronized set
            m_sync_stats.updateStats(frameset);
            return frameset;
        }
        
        // 3. Clean up old frames periodically
        static int cleanup_counter = 0;
        if (++cleanup_counter % 10 == 0) {
            cleanupOldFrames();
        }
        
        // Small sleep to avoid busy waiting
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    
    // Timeout - return empty frameset
    return frameset;
}

// ============================================================================
// CAPTURE AND BUFFER FRAMES (captures from both sensors and adds to queues)
// ============================================================================

void DetermSensor::captureAndBufferFrames() {
    // 1. Capture from Raspberry Pi (color sensor)
    if (m_color_enabled && m_raspberry_sensor) {
        try {
            auto rpi_frame = m_raspberry_sensor->GetTimestampedFrame();
            if (!rpi_frame.data.empty()) {
                auto dim_it = m_stream_dimensions.find("color");
                auto fmt_it = m_stream_formats.find("color");
                
                int width = (dim_it != m_stream_dimensions.end()) ? dim_it->second.first : m_color_width;
                int height = (dim_it != m_stream_dimensions.end()) ? dim_it->second.second : m_color_height;
                std::string format = (fmt_it != m_stream_formats.end()) ? fmt_it->second : m_color_format;
                
                TimestampedFrame frame("color", std::move(rpi_frame.data), width, height, format);
                frame.software_timestamp = rpi_frame.timestamp;
                
                // Add to buffer
                addFrameToBuffer(std::move(frame));
                m_sync_stats.total_frames_captured++;
            }
        } catch (const std::exception& e) {
            // Continue on error - other sensor may still work
        }
    }
    
    // 2. Capture from RealSense (depth sensor)
    if (m_depth_enabled && m_realsense_sensor) {
        try {
            rs2::frameset rs_frameset = m_realsense_sensor->GetFrameset();
            
            if (rs_frameset) {
                for (const auto& stream_name : m_enabled_streams) {
                    if (stream_name == "color") continue;  // Skip color, handled by Raspberry Pi
                    
                    rs2::frame rs_frame;
                    
                    if (stream_name == "depth") {
                        rs_frame = rs_frameset.get_depth_frame();
                    } else if (stream_name == "infrared1") {
                        rs_frame = rs_frameset.get_infrared_frame(1);
                    } else if (stream_name == "infrared2") {
                        rs_frame = rs_frameset.get_infrared_frame(2);
                    }
                    
                    if (rs_frame) {
                        auto dim_it = m_stream_dimensions.find(stream_name);
                        auto fmt_it = m_stream_formats.find(stream_name);
                        
                        auto video_frame = rs_frame.as<rs2::video_frame>();
                        int width = (dim_it != m_stream_dimensions.end()) ? dim_it->second.first : video_frame.get_width();
                        int height = (dim_it != m_stream_dimensions.end()) ? dim_it->second.second : video_frame.get_height();
                        std::string format = (fmt_it != m_stream_formats.end()) ? fmt_it->second : 
                                           (stream_name == "depth" ? "Z16" : "Y8");
                        
                        // Extract frame data
                        const uint8_t* frame_ptr = static_cast<const uint8_t*>(rs_frame.get_data());
                        size_t frame_size = width * height * video_frame.get_bytes_per_pixel();
                        std::vector<uint8_t> data(frame_size);
                        std::memcpy(data.data(), frame_ptr, frame_size);
                        
                        TimestampedFrame frame(stream_name, std::move(data), width, height, format);
                        
                        // Get hardware timestamp from RealSense (in milliseconds)
                        double hw_timestamp_ms = rs_frame.get_timestamp();
                        frame.setHardwareTimestamp(hw_timestamp_ms);
                        
                        // Convert to microseconds
                        uint64_t timestamp_us = static_cast<uint64_t>(hw_timestamp_ms * 1000.0);
                        auto hardware_duration_us = std::chrono::microseconds(timestamp_us);
                        frame.software_timestamp = std::chrono::high_resolution_clock::time_point(hardware_duration_us);
                        
                        // Add to buffer
                        addFrameToBuffer(std::move(frame));
                        m_sync_stats.total_frames_captured++;
                    }
                }
            }
        } catch (const std::exception& e) {
            // Continue on error
        }
    }
}

// ============================================================================
// BUFFERED SYNCHRONIZATION METHODS (if needed for advanced use)
// ============================================================================

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
    
    // Advanced matching algorithm: Find the best temporal match across all streams

    
    std::map<std::string, TimestampedFrame> best_match;
    double best_max_diff = std::numeric_limits<double>::max();
    
    // Find stream with minimum buffer size (most constrained) to use as reference
    std::string reference_stream;
    size_t min_buffer_size = std::numeric_limits<size_t>::max();
    
    for (const auto& stream : m_enabled_streams) {
        if (m_frame_buffers[stream].size() < min_buffer_size) {
            min_buffer_size = m_frame_buffers[stream].size();
            reference_stream = stream;
        }
    }
    
    if (reference_stream.empty() || m_frame_buffers[reference_stream].empty()) {
        return false;
    }
    
    // Try to match frames using each frame in the reference buffer
    auto& ref_buffer = m_frame_buffers[reference_stream];
    std::queue<TimestampedFrame> temp_ref_buffer;
    
    // Limit search depth to avoid excessive computation
    int search_depth = std::min(static_cast<int>(ref_buffer.size()), 10);
    
    for (int ref_idx = 0; ref_idx < search_depth && !ref_buffer.empty(); ++ref_idx) {
        TimestampedFrame ref_frame = ref_buffer.front();
        ref_buffer.pop();
        temp_ref_buffer.push(ref_frame);
        
        double ref_timestamp_ms = ref_frame.getPrimaryTimestampMs();
        
        // Try to find matching frames in other streams
        std::map<std::string, TimestampedFrame> candidate_match;
        candidate_match[reference_stream] = ref_frame;
        
        bool valid_match = true;
        double max_diff = 0.0;
        
        for (const auto& stream : m_enabled_streams) {
            if (stream == reference_stream) continue;
            
            // Search this stream's buffer for best temporal match
            TimestampedFrame* best_stream_match = nullptr;
            double min_time_diff = std::numeric_limits<double>::max();
            
            auto& stream_buffer = m_frame_buffers[stream];
            std::queue<TimestampedFrame> temp_stream_buffer;
            
            while (!stream_buffer.empty()) {
                TimestampedFrame current_frame = stream_buffer.front();
                stream_buffer.pop();
                
                double current_timestamp_ms = current_frame.getPrimaryTimestampMs();
                double time_diff = std::abs(current_timestamp_ms - ref_timestamp_ms);
                
                if (time_diff < min_time_diff) {
                    if (best_stream_match) {
                        temp_stream_buffer.push(*best_stream_match);
                        delete best_stream_match;
                    }
                    best_stream_match = new TimestampedFrame(std::move(current_frame));
                    min_time_diff = time_diff;
                } else {
                    temp_stream_buffer.push(current_frame);
                }
            }
            
            // Restore buffer
            stream_buffer = std::move(temp_stream_buffer);
            
            if (best_stream_match) {
                if (min_time_diff <= m_sync_tolerance_ms) {
                    candidate_match[stream] = *best_stream_match;
                    max_diff = std::max(max_diff, min_time_diff);
                    delete best_stream_match;
                } else {
                    // No match within tolerance for this stream
                    valid_match = false;
                    delete best_stream_match;
                    break;
                }
            } else {
                // No frame found in buffer
                valid_match = false;
                break;
            }
        }
        
        // Check if this is a better match than previous best
        if (valid_match && candidate_match.size() == m_enabled_streams.size()) {
            if (max_diff < best_max_diff) {
                best_max_diff = max_diff;
                best_match = std::move(candidate_match);
                
                // If we found an excellent match (< 2ms), use it immediately
                if (best_max_diff < 2.0) {
                    break;
                }
            }
        }
    }
    
    // Restore reference buffer
    while (!temp_ref_buffer.empty()) {
        ref_buffer.push(temp_ref_buffer.front());
        temp_ref_buffer.pop();
    }
    
    // If we found a valid match, use it
    if (!best_match.empty() && best_match.size() == m_enabled_streams.size()) {
        // Remove the matched frames from buffers
        for (const auto& [stream_name, matched_frame] : best_match) {
            auto& buffer = m_frame_buffers[stream_name];
            std::queue<TimestampedFrame> temp_buffer;
            
            bool removed = false;
            while (!buffer.empty()) {
                TimestampedFrame current = buffer.front();
                buffer.pop();
                
                // Match by frame number (unique identifier)
                if (!removed && current.frame_number == matched_frame.frame_number) {
                    removed = true;
                    // Don't add to temp buffer - this is the one we're using
                } else {
                    temp_buffer.push(std::move(current));
                }
            }
            
            buffer = std::move(temp_buffer);
        }
        
        frame_set.frames = std::move(best_match);
        frame_set.calculateSyncStats();
        return true;
    }
    
    return false;
}

void DetermSensor::cleanupOldFrames() {
    std::lock_guard<std::mutex> lock(m_buffer_mutex);
    
    if (m_frame_buffers.empty()) return;
    
    // Calculate reference timestamp (most recent frame across all buffers)
    uint64_t newest_timestamp_us = 0;
    
    for (const auto& [stream_name, buffer] : m_frame_buffers) {
        if (!buffer.empty()) {
            // Peek at frames to find newest timestamp
            std::queue<TimestampedFrame> temp = buffer;
            while (!temp.empty()) {
                uint64_t ts = temp.front().getTimestampUs();
                newest_timestamp_us = std::max(newest_timestamp_us, ts);
                temp.pop();
            }
        }
    }
    
    if (newest_timestamp_us == 0) return;
    
    // Remove frames older than 3x tolerance window
    double max_age_us = m_sync_tolerance_ms * 1000.0 * 3.0;
    
    for (auto& [stream_name, buffer] : m_frame_buffers) {
        std::queue<TimestampedFrame> cleaned_buffer;
        int dropped = 0;
        
        while (!buffer.empty()) {
            TimestampedFrame frame = buffer.front();
            buffer.pop();
            
            uint64_t frame_ts = frame.getTimestampUs();
            double age_us = static_cast<double>(newest_timestamp_us - frame_ts);
            
            if (age_us < max_age_us) {
                cleaned_buffer.push(std::move(frame));
            } else {
                dropped++;
                m_sync_stats.total_frames_dropped++;
            }
        }
        
        buffer = std::move(cleaned_buffer);
    }
}

// ============================================================================
// CONTINUOUS SYNCHRONIZED CAPTURE
// ============================================================================

bool DetermSensor::StartSynchronizedCapture() {
    if (m_sync_capture_active) {
        std::cout << "Synchronized capture already active" << std::endl;
        return true;
    }
    
    if (!m_is_running) {
        std::cerr << "Sensors must be running to start synchronized capture" << std::endl;
        return false;
    }
    
    // Initialize frame buffers
    for (const auto& stream : m_enabled_streams) {
        m_frame_buffers[stream] = std::queue<TimestampedFrame>();
    }
    
    // Reset statistics
    m_sync_stats = SyncStats();
    
    m_sync_capture_active = true;
    m_sync_capture_thread = std::thread(&DetermSensor::SynchronizedCaptureThread, this);
    
    std::cout << "Synchronized capture started using native timestamps" << std::endl;
    std::cout << "  RealSense: Hardware timestamps (GLOBAL_TIME domain)" << std::endl;
    std::cout << "  Raspberry Pi: Software timestamps (high_resolution_clock)" << std::endl;
    std::cout << "  Sync tolerance: " << m_sync_tolerance_ms << " ms" << std::endl;
    
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
            // Get synchronized frames using native timestamps
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

// ============================================================================
// Sensor Configuration
// ============================================================================

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
    
    // Store dimensions and format for synchronization
    m_stream_dimensions["color"] = {width, height};
    m_stream_formats["color"] = format;
    
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

  if (!m_depth_enabled) {
    std::cerr << "Depth streams not enabled. Call EnableStream() first." << std::endl;
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
    
    // Store dimensions and formats for synchronization
    for (const auto& stream : realsense_streams) {
      m_stream_dimensions[stream] = {width, height};
      if (stream == "depth") {
        m_stream_formats[stream] = "Z16";
      } else if (stream == "infrared1" || stream == "infrared2") {
        m_stream_formats[stream] = "Y8";
      }
    }
    
    std::cout << "Depth sensor configured successfully" << std::endl;
  } else {
    std::cerr << "Failed to configure depth sensor: " << ret << std::endl;
  }

  return ret;
}

// ============================================================================
// Sensor Controls
// ============================================================================

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

// ============================================================================
// Start/Stop
// ============================================================================

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
  if (m_depth_enabled) {
    depth_result = StartDepthSensor();
  }

  // Check if at least one sensor started successfully
  bool color_success = !m_color_enabled || (color_result == 0);
  bool depth_success = !m_depth_enabled || (depth_result == 0);

  if (!color_success && !depth_success) {
    std::cerr << "Failed to start any enabled sensors" << std::endl;
    return -1;
  }

  m_is_running = true;
  std::cout << "DetermSensor started successfully" << std::endl;

  // Print start summary
  std::cout << "Sensor start summary:" << std::endl;
  if (m_color_enabled) {
    std::cout << "  Color sensor: " << (color_result == 0 ? "Success" : " Failed") << std::endl;
  }
  if (m_depth_enabled) {
    std::cout << "  Depth sensor: " << (depth_result == 0 ? "Success" : " Failed") << std::endl;
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
  if (m_depth_enabled) {
    depth_result = StopDepthSensor();
  }

  m_is_running = false;
  std::cout << "DetermSensor stopped successfully" << std::endl;

  // Print stop summary
  std::cout << "Sensor stop summary:" << std::endl;
  if (m_color_enabled) {
    std::cout << "  Color sensor: " << (color_result == 0 ? "Success" : " Failed") << std::endl;
  }
  if (m_depth_enabled) {
    std::cout << "  Depth sensor: " << (depth_result == 0 ? "Success" : " Failed") << std::endl;
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

// ============================================================================
// Legacy Frame Capture (non-synchronized)
// ============================================================================

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
        
        // Get frames from RealsenseSensor (depth streams)
        if (m_realsense_sensor && IsDepthSensorEnabled()) {
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

// ============================================================================
// Information and Diagnostics
// ============================================================================

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

// ============================================================================
// CALIBRATION METHODS
// ============================================================================

void CalibrationData::print() const {
    std::cout << "\n========== CALIBRATION DATA ==========" << std::endl;
    
    std::cout << "Color Camera Intrinsics:" << std::endl;
    std::cout << "  Resolution: " << color_intrinsics.width << "x" << color_intrinsics.height << std::endl;
    std::cout << "  Focal length (fx, fy): " << color_intrinsics.fx << ", " << color_intrinsics.fy << std::endl;
    std::cout << "  Principal point (ppx, ppy): " << color_intrinsics.ppx << ", " << color_intrinsics.ppy << std::endl;
    std::cout << "  Distortion coeffs: [" 
              << color_intrinsics.coeffs[0] << ", " << color_intrinsics.coeffs[1] << ", "
              << color_intrinsics.coeffs[2] << ", " << color_intrinsics.coeffs[3] << ", " 
              << color_intrinsics.coeffs[4] << "]" << std::endl;
    
    std::cout << "\nDepth Camera Intrinsics:" << std::endl;
    std::cout << "  Resolution: " << depth_intrinsics.width << "x" << depth_intrinsics.height << std::endl;
    std::cout << "  Focal length (fx, fy): " << depth_intrinsics.fx << ", " << depth_intrinsics.fy << std::endl;
    std::cout << "  Principal point (ppx, ppy): " << depth_intrinsics.ppx << ", " << depth_intrinsics.ppy << std::endl;
    std::cout << "  Distortion coeffs: [" 
              << depth_intrinsics.coeffs[0] << ", " << depth_intrinsics.coeffs[1] << ", "
              << depth_intrinsics.coeffs[2] << ", " << depth_intrinsics.coeffs[3] << ", " 
              << depth_intrinsics.coeffs[4] << "]" << std::endl;
    
    std::cout << "\nDepth-to-Color Extrinsics:" << std::endl;
    std::cout << "  Rotation matrix:" << std::endl;
    std::cout << "    [" << depth_to_color.rotation[0] << ", " << depth_to_color.rotation[1] << ", " << depth_to_color.rotation[2] << "]" << std::endl;
    std::cout << "    [" << depth_to_color.rotation[3] << ", " << depth_to_color.rotation[4] << ", " << depth_to_color.rotation[5] << "]" << std::endl;
    std::cout << "    [" << depth_to_color.rotation[6] << ", " << depth_to_color.rotation[7] << ", " << depth_to_color.rotation[8] << "]" << std::endl;
    std::cout << "  Translation (meters): [" 
              << depth_to_color.translation[0] << ", " 
              << depth_to_color.translation[1] << ", " 
              << depth_to_color.translation[2] << "]" << std::endl;
    
    std::cout << "======================================\n" << std::endl;
}

bool DetermSensor::LoadCalibration(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open calibration file: " << filepath << std::endl;
        return false;
    }
    
    try {
        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string json = buffer.str();
        file.close();
        
        // Parse color intrinsics
        size_t pos = json.find("\"color_intrinsic\"");
        if (pos != std::string::npos) {
            m_calibration.color_intrinsics.width = parseJsonInt(json, "width", pos);
            m_calibration.color_intrinsics.height = parseJsonInt(json, "height", pos);
            m_calibration.color_intrinsics.fx = parseJsonFloat(json, "fx", pos);
            m_calibration.color_intrinsics.fy = parseJsonFloat(json, "fy", pos);
            m_calibration.color_intrinsics.ppx = parseJsonFloat(json, "cx", pos);
            m_calibration.color_intrinsics.ppy = parseJsonFloat(json, "cy", pos);
            
            size_t dist_pos = json.find("\"distortion\"", pos);
            if (dist_pos != std::string::npos) {
                m_calibration.color_intrinsics.coeffs[0] = parseJsonFloat(json, "k1", dist_pos);
                m_calibration.color_intrinsics.coeffs[1] = parseJsonFloat(json, "k2", dist_pos);
                m_calibration.color_intrinsics.coeffs[2] = parseJsonFloat(json, "p1", dist_pos);
                m_calibration.color_intrinsics.coeffs[3] = parseJsonFloat(json, "p2", dist_pos);
                m_calibration.color_intrinsics.coeffs[4] = parseJsonFloat(json, "k3", dist_pos);
            }
            m_calibration.color_intrinsics.model = RS2_DISTORTION_BROWN_CONRADY;
        }
        
        // Parse depth intrinsics
        pos = json.find("\"depth_intrinsic\"");
        if (pos != std::string::npos) {
            m_calibration.depth_intrinsics.width = parseJsonInt(json, "width", pos);
            m_calibration.depth_intrinsics.height = parseJsonInt(json, "height", pos);
            m_calibration.depth_intrinsics.fx = parseJsonFloat(json, "fx", pos);
            m_calibration.depth_intrinsics.fy = parseJsonFloat(json, "fy", pos);
            m_calibration.depth_intrinsics.ppx = parseJsonFloat(json, "cx", pos);
            m_calibration.depth_intrinsics.ppy = parseJsonFloat(json, "cy", pos);
            
            size_t dist_pos = json.find("\"distortion\"", pos);
            if (dist_pos != std::string::npos) {
                m_calibration.depth_intrinsics.coeffs[0] = parseJsonFloat(json, "k1", dist_pos);
                m_calibration.depth_intrinsics.coeffs[1] = parseJsonFloat(json, "k2", dist_pos);
                m_calibration.depth_intrinsics.coeffs[2] = parseJsonFloat(json, "p1", dist_pos);
                m_calibration.depth_intrinsics.coeffs[3] = parseJsonFloat(json, "p2", dist_pos);
                m_calibration.depth_intrinsics.coeffs[4] = parseJsonFloat(json, "k3", dist_pos);
            }
            m_calibration.depth_intrinsics.model = RS2_DISTORTION_BROWN_CONRADY;
        }
        
        // Parse extrinsics
        pos = json.find("\"depth_to_color_extrinsic\"");
        if (pos != std::string::npos) {
            size_t arr_start = json.find("[", pos);
            size_t arr_end = json.find("]", arr_start);
            std::string extr_str = json.substr(arr_start + 1, arr_end - arr_start - 1);
            
            std::istringstream iss(extr_str);
            std::string val;
            std::vector<float> extr_values;
            
            // Parse all values from the array
            while (std::getline(iss, val, ',') && extr_values.size() < 16) {
                extr_values.push_back(std::stof(val));
            }
            
            if (extr_values.size() == 16) {
                // Extract 3x3 rotation matrix from 4x4 transformation matrix
                // Row 0: [0, 1, 2, skip 3]
                m_calibration.depth_to_color.rotation[0] = extr_values[0];
                m_calibration.depth_to_color.rotation[1] = extr_values[1];
                m_calibration.depth_to_color.rotation[2] = extr_values[2];
                
                // Row 1: [4, 5, 6, skip 7]
                m_calibration.depth_to_color.rotation[3] = extr_values[4];
                m_calibration.depth_to_color.rotation[4] = extr_values[5];
                m_calibration.depth_to_color.rotation[5] = extr_values[6];
                
                // Row 2: [8, 9, 10, skip 11]
                m_calibration.depth_to_color.rotation[6] = extr_values[8];
                m_calibration.depth_to_color.rotation[7] = extr_values[9];
                m_calibration.depth_to_color.rotation[8] = extr_values[10];
                
                // Extract translation from column 3 (indices 3, 7, 11)
                m_calibration.depth_to_color.translation[0] = extr_values[3];
                m_calibration.depth_to_color.translation[1] = extr_values[7];
                m_calibration.depth_to_color.translation[2] = extr_values[11];
                
                // Note: indices 12, 13, 14 are 0s and index 15 is 1 (homogeneous coordinates)
            } else {
                std::cerr << "Warning: depth_to_color_extrinsic has " << extr_values.size() 
                          << " elements, expected 16" << std::endl;
            }
        }
        
        m_has_calibration = true;
        std::cout << "Calibration loaded from: " << filepath << std::endl;
        m_calibration.print();
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error loading calibration: " << e.what() << std::endl;
        m_has_calibration = false;
        return false;
    }
}

bool DetermSensor::SaveCalibration(const std::string& filepath) const {
    if (!m_has_calibration) {
        std::cerr << "No calibration to save" << std::endl;
        return false;
    }
    
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to create file: " << filepath << std::endl;
        return false;
    }
    
    file << "{\n";
    file << "    \"color_intrinsic\": {\n";
    file << "        \"width\": " << m_calibration.color_intrinsics.width << ",\n";
    file << "        \"height\": " << m_calibration.color_intrinsics.height << ",\n";
    file << "        \"fx\": " << m_calibration.color_intrinsics.fx << ",\n";
    file << "        \"fy\": " << m_calibration.color_intrinsics.fy << ",\n";
    file << "        \"cx\": " << m_calibration.color_intrinsics.ppx << ",\n";
    file << "        \"cy\": " << m_calibration.color_intrinsics.ppy << ",\n";
    file << "        \"distortion\": {\n";
    file << "            \"model\": \"brown_conrady\",\n";
    file << "            \"k1\": " << m_calibration.color_intrinsics.coeffs[0] << ",\n";
    file << "            \"k2\": " << m_calibration.color_intrinsics.coeffs[1] << ",\n";
    file << "            \"k3\": " << m_calibration.color_intrinsics.coeffs[4] << ",\n";
    file << "            \"p1\": " << m_calibration.color_intrinsics.coeffs[2] << ",\n";
    file << "            \"p2\": " << m_calibration.color_intrinsics.coeffs[3] << "\n";
    file << "        }\n";
    file << "    },\n\n";
    
    file << "    \"depth_intrinsic\": {\n";
    file << "        \"width\": " << m_calibration.depth_intrinsics.width << ",\n";
    file << "        \"height\": " << m_calibration.depth_intrinsics.height << ",\n";
    file << "        \"fx\": " << m_calibration.depth_intrinsics.fx << ",\n";
    file << "        \"fy\": " << m_calibration.depth_intrinsics.fy << ",\n";
    file << "        \"cx\": " << m_calibration.depth_intrinsics.ppx << ",\n";
    file << "        \"cy\": " << m_calibration.depth_intrinsics.ppy << ",\n";
    file << "        \"distortion\": {\n";
    file << "            \"model\": \"brown_conrady\",\n";
    file << "            \"k1\": " << m_calibration.depth_intrinsics.coeffs[0] << ",\n";
    file << "            \"k2\": " << m_calibration.depth_intrinsics.coeffs[1] << ",\n";
    file << "            \"k3\": " << m_calibration.depth_intrinsics.coeffs[4] << ",\n";
    file << "            \"p1\": " << m_calibration.depth_intrinsics.coeffs[2] << ",\n";
    file << "            \"p2\": " << m_calibration.depth_intrinsics.coeffs[3] << "\n";
    file << "        }\n";
    file << "    },\n\n";
    
    file << "    \"depth_to_color_extrinsic\": [\n";
    // Build 4x4 transformation matrix from rotation and translation
    file << "        " << m_calibration.depth_to_color.rotation[0] << ",\n";
    file << "        " << m_calibration.depth_to_color.rotation[1] << ",\n";
    file << "        " << m_calibration.depth_to_color.rotation[2] << ",\n";
    file << "        " << m_calibration.depth_to_color.translation[0] << ",\n";

    file << "        " << m_calibration.depth_to_color.rotation[3] << ",\n";
    file << "        " << m_calibration.depth_to_color.rotation[4] << ",\n";
    file << "        " << m_calibration.depth_to_color.rotation[5] << ",\n";
    file << "        " << m_calibration.depth_to_color.translation[1] << ",\n";

    file << "        " << m_calibration.depth_to_color.rotation[6] << ",\n";
    file << "        " << m_calibration.depth_to_color.rotation[7] << ",\n";
    file << "        " << m_calibration.depth_to_color.rotation[8] << ",\n";
    file << "        " << m_calibration.depth_to_color.translation[2] << ",\n";

    file << "        0,\n";
    file << "        0,\n";
    file << "        0,\n";
    file << "        1\n";
    file << "    ]\n";
    file << "}\n";
    
    file.close();
    std::cout << "Calibration saved to: " << filepath << std::endl;
    return true;
}

int DetermSensor::parseJsonInt(const std::string& json, const std::string& key, size_t start_pos) {
    size_t pos = json.find("\"" + key + "\"", start_pos);
    if (pos == std::string::npos) return 0;
    
    size_t colon = json.find(":", pos);
    size_t comma = json.find(",", colon);
    return std::stoi(json.substr(colon + 1, comma - colon - 1));
}

float DetermSensor::parseJsonFloat(const std::string& json, const std::string& key, size_t start_pos) {
    size_t pos = json.find("\"" + key + "\"", start_pos);
    if (pos == std::string::npos) return 0.0f;
    
    size_t colon = json.find(":", pos);
    size_t comma = json.find(",", colon);
    if (comma == std::string::npos) {
        comma = json.find("}", colon);
    }
    return std::stof(json.substr(colon + 1, comma - colon - 1));
}