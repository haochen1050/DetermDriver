#include "sensor/synchronized_camera.h"
#include <algorithm>

namespace determ {
namespace sensor {

// TimestampedFrame implementation - Software timestamp only
TimestampedFrame::TimestampedFrame() 
    : frame_number(0), width(0), height(0) {}

TimestampedFrame::TimestampedFrame(const std::string& name, std::vector<uint8_t>&& frame_data, 
                                 int w, int h, const std::string& fmt)
    : stream_name(name), data(std::move(frame_data)), 
      software_timestamp(std::chrono::high_resolution_clock::now()),  // Changed from 'timestamp'
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
    std::cout << "\n📊 === Synchronization Statistics ===" << std::endl;
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

// SynchronizedCameraApp implementation
SynchronizedCameraApp::SynchronizedCameraApp(const SyncConfig& config)
    : config_(config), sensor_(std::make_unique<DetermSensor>()) {
    std::cout << "SynchronizedCameraApp created with software timestamp support" << std::endl;
}

SynchronizedCameraApp::~SynchronizedCameraApp() {
    Stop();
    std::cout << "SynchronizedCameraApp destroyed" << std::endl;
}

int SynchronizedCameraApp::Initialize() {
    if (is_initialized_) {
        std::cout << "Application already initialized" << std::endl;
        return 0;
    }
    
    std::cout << "🚀 Initializing Synchronized Camera Application..." << std::endl;
    
    // Step 1: Initialize the sensor
    int ret = initializeSensor();
    if (ret != 0) {
        std::cerr << "Failed to initialize sensor: " << ret << std::endl;
        return ret;
    }
    
    // Step 2: Configure the sensor
    ret = configureSensor();
    if (ret != 0) {
        std::cerr << "Failed to configure sensor: " << ret << std::endl;
        return ret;
    }
    
    // Step 3: Initialize frame buffers
    for (const auto& stream : config_.enabled_streams) {
        frame_buffers_[stream] = std::queue<TimestampedFrame>();
    }
    
    is_initialized_ = true;
    std::cout << "Synchronized Camera Application initialized successfully" << std::endl;
    
    return 0;
}

int SynchronizedCameraApp::Start() {
    if (!is_initialized_) {
        std::cerr << "Application not initialized. Call Initialize() first." << std::endl;
        return -1;
    }
    
    if (is_running_) {
        std::cout << "Application already running" << std::endl;
        return 0;
    }
    
    std::cout << "Starting synchronized capture..." << std::endl;
    
    // Start the sensor
    int ret = sensor_->Start();
    if (ret != 0) {
        std::cerr << "Failed to start sensor: " << ret << std::endl;
        return ret;
    }
    
    // Reset statistics
    stats_ = SyncStats();
    
    // Start worker threads
    is_running_ = true;
    startWorkerThreads();
    
    std::cout << "Synchronized capture started successfully" << std::endl;
    return 0;
}

int SynchronizedCameraApp::Stop() {
    if (!is_running_) {
        return 0;
    }
    
    std::cout << "Stopping synchronized capture..." << std::endl;
    
    // Stop worker threads first
    stopWorkerThreads();
    
    // Stop the sensor
    int ret = 0;
    if (sensor_) {
        ret = sensor_->Stop();
        if (ret != 0) {
            std::cerr << "Warning: Failed to stop sensor cleanly: " << ret << std::endl;
        }
    }
    
    is_running_ = false;
    
    // Clear frame buffers
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        frame_buffers_.clear();
    }
    
    {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        while (!synchronized_frames_.empty()) {
            synchronized_frames_.pop();
        }
    }
    
    std::cout << "Synchronized capture stopped" << std::endl;
    
    // Print final statistics
    if (config_.sync.enable_stats) {
        stats_.calculateFPS();
        printStats(true);
    }
    
    return ret;
}

int SynchronizedCameraApp::initializeSensor() {
    std::cout << "Initializing cameras..." << std::endl;
    
    int ret = sensor_->Initialize();
    if (ret != 0) {
        return ret;
    }
    
    std::cout << "Cameras initialized" << std::endl;
    return 0;
}

int SynchronizedCameraApp::configureSensor() {
    std::cout << "Configuring camera streams..." << std::endl;
    
    // Enable streams
    int ret = sensor_->EnableStream(config_.enabled_streams);
    if (ret != 0) {
        return ret;
    }
    
    // Configure color sensor if enabled
    bool has_color = std::find(config_.enabled_streams.begin(), config_.enabled_streams.end(), "color") != config_.enabled_streams.end();
    if (has_color) {
        ret = sensor_->SetColorSensorConfig(config_.color.width, config_.color.height, config_.color.format);
        if (ret != 0) {
            return ret;
        }
        
        // Set color controls
        sensor_->SetColorSensorControl(config_.color.auto_exposure, config_.color.manual_exposure, config_.color.white_balance);
        sensor_->SetColorFocusControl(config_.color.auto_focus, config_.color.manual_focus);
        
        // Store dimensions
        stream_dimensions_["color"] = {config_.color.width, config_.color.height};
        stream_formats_["color"] = config_.color.format;
    }
    
    // Configure depth sensor if enabled
    bool has_depth_streams = false;
    for (const auto& stream : config_.enabled_streams) {
        if (stream == "depth" || stream == "infrared1" || stream == "infrared2") {
            has_depth_streams = true;
            break;
        }
    }
    
    if (has_depth_streams) {
        ret = sensor_->SetDepthSensorConfig(config_.depth.width, config_.depth.height);
        if (ret != 0) {
            return ret;
        }
        
        // Set depth controls
        sensor_->SetDepthSensorControl(config_.depth.auto_exposure, config_.depth.manual_exposure, config_.depth.gain);
        sensor_->SetLaserPower(config_.depth.laser_power);
        sensor_->SetDepthUnits(config_.depth.depth_units);
        
        // Store dimensions for depth streams
        for (const auto& stream : config_.enabled_streams) {
            if (stream == "depth" || stream == "infrared1" || stream == "infrared2") {
                stream_dimensions_[stream] = {config_.depth.width, config_.depth.height};
                stream_formats_[stream] = (stream == "depth") ? "Z16" : "Y8";
            }
        }
    }
    
    std::cout << "Camera streams configured" << std::endl;
    return 0;
}

void SynchronizedCameraApp::startWorkerThreads() {
    // Start capture worker
    worker_threads_.emplace_back(&SynchronizedCameraApp::captureWorker, this);
    
    // Start synchronization worker
    worker_threads_.emplace_back(&SynchronizedCameraApp::synchronizationWorker, this);
    
    // Set thread priorities if requested
    if (config_.performance.high_priority) {
        setThreadPriority();
    }
    
    std::cout << "Worker threads started (" << worker_threads_.size() << " threads)" << std::endl;
}

void SynchronizedCameraApp::stopWorkerThreads() {
    is_running_ = false;
    
    // Notify all waiting threads
    sync_cv_.notify_all();
    
    // Join all worker threads
    for (auto& thread : worker_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    worker_threads_.clear();
    
    std::cout << "Worker threads stopped" << std::endl;
}

void SynchronizedCameraApp::captureWorker() {
    std::cout << "Capture worker started" << std::endl;
    
    bool has_hw_timestamps = sensor_->hasHardwareTimestamps();
    std::cout << "Sensor provides " << (has_hw_timestamps ? "mixed hardware/software" : "software only") << " timestamps" << std::endl;
    
    while (is_running_) {
        try {
            // Capture software timestamp as close to frame acquisition as possible
            auto capture_timestamp = std::chrono::high_resolution_clock::now();
            
            // Capture frames from all enabled streams
            auto frame_data = sensor_->GetFrame();
            
            // Get hardware timestamps if available (for RealSense streams)
            std::map<std::string, double> hw_timestamps;
            if (has_hw_timestamps) {
                hw_timestamps = sensor_->getLastHardwareTimestamps();
            }
            
            if (!frame_data.empty()) {
                for (const auto& [stream_name, data] : frame_data) {
                    if (std::find(config_.enabled_streams.begin(), config_.enabled_streams.end(), stream_name) != config_.enabled_streams.end()) {
                        
                        // Get dimensions and format
                        auto dim_it = stream_dimensions_.find(stream_name);
                        auto fmt_it = stream_formats_.find(stream_name);
                        
                        int width = (dim_it != stream_dimensions_.end()) ? dim_it->second.first : 0;
                        int height = (dim_it != stream_dimensions_.end()) ? dim_it->second.second : 0;
                        std::string format = (fmt_it != stream_formats_.end()) ? fmt_it->second : "unknown";
                        
                        // Create timestamped frame
                        TimestampedFrame frame(stream_name, std::vector<uint8_t>(data), width, height, format);
                        frame.software_timestamp = capture_timestamp;
                        
                        // Set hardware timestamp if available (RealSense streams)
                        auto hw_it = hw_timestamps.find(stream_name);
                        if (hw_it != hw_timestamps.end()) {
                            frame.setHardwareTimestamp(hw_it->second);
                        }
                        
                        addFrameToBuffer(std::move(frame));
                        stats_.total_frames_captured++;
                    }
                }
            }
            
        } catch (const std::exception& e) {
            if (is_running_) {
                std::cerr << "Capture worker error: " << e.what() << std::endl;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    
    std::cout << "Capture worker stopped" << std::endl;
}

void SynchronizedCameraApp::synchronizationWorker() {
    std::cout << "Synchronization worker started" << std::endl;
    
    while (is_running_) {
        try {
            SynchronizedFrameSet frame_set;
            
            if (findSynchronizedFrames(frame_set)) {
                // Update statistics
                stats_.updateStats(frame_set);
                
                // Add to synchronized frames queue
                {
                    std::lock_guard<std::mutex> lock(sync_mutex_);
                    if (synchronized_frames_.size() >= static_cast<size_t>(config_.performance.queue_size)) {
                        synchronized_frames_.pop();  // Drop oldest
                        stats_.total_frames_dropped++;
                    }
                    synchronized_frames_.push(frame_set);
                }
                
                // Notify waiting threads
                sync_cv_.notify_one();
                
                // Call callback if registered
                if (frame_callback_) {
                    frame_callback_(frame_set);
                }
            }
            
            // Cleanup old frames periodically
            cleanupOldFrames();
            
        } catch (const std::exception& e) {
            if (is_running_) {
                std::cerr << "Synchronization worker error: " << e.what() << std::endl;
            }
        }
        
        // Small delay for synchronization worker
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    std::cout << "Synchronization worker stopped" << std::endl;
}

void SynchronizedCameraApp::addFrameToBuffer(TimestampedFrame&& frame) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    auto& buffer = frame_buffers_[frame.stream_name];
    
    // Limit buffer size
    while (buffer.size() >= config_.sync.buffer_size) {
        buffer.pop();
        stats_.total_frames_dropped++;
    }
    
    buffer.push(std::move(frame));
}

bool SynchronizedCameraApp::findSynchronizedFrames(SynchronizedFrameSet& frame_set) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    // Check if we have frames in all required streams
    for (const auto& stream : config_.enabled_streams) {
        if (frame_buffers_[stream].empty()) {
            return false;  // Not all streams have frames
        }
    }
    
    // Use primary timestamp synchronization
    std::map<std::string, TimestampedFrame> candidate_frames;
    
    const auto& reference_stream = config_.enabled_streams[0];
    auto& reference_buffer = frame_buffers_[reference_stream];
    const auto& reference_frame = reference_buffer.front();
    candidate_frames[reference_stream] = reference_frame;
    
    bool valid_sync = true;
    for (size_t i = 1; i < config_.enabled_streams.size(); ++i) {
        const auto& stream_name = config_.enabled_streams[i];
        auto& buffer = frame_buffers_[stream_name];
        
        TimestampedFrame* best_match = nullptr;
        double min_time_diff = std::numeric_limits<double>::max();
        
        std::queue<TimestampedFrame> temp_queue;
        while (!buffer.empty()) {
            TimestampedFrame current_frame = buffer.front();
            buffer.pop();
            
            double time_diff = std::abs(current_frame.getPrimaryTimestampMs() - reference_frame.getPrimaryTimestampMs());  // Use new method
            
            if (time_diff < min_time_diff && time_diff <= config_.sync.tolerance_ms) {
                if (best_match) {
                    temp_queue.push(*best_match);
                }
                best_match = new TimestampedFrame(std::move(current_frame));
                min_time_diff = time_diff;
            } else {
                temp_queue.push(current_frame);
            }
        }
        
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
        for (const auto& stream : config_.enabled_streams) {
            if (!frame_buffers_[stream].empty()) {
                frame_buffers_[stream].pop();
            }
        }
        
        frame_set.frames = std::move(candidate_frames);
        frame_set.calculateSyncStats();
        return true;
    }
    
    return false;
}

void SynchronizedCameraApp::cleanupOldFrames() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    auto now = std::chrono::high_resolution_clock::now();
    double max_age_ms = config_.sync.tolerance_ms * 3;
    
    for (auto& [stream_name, buffer] : frame_buffers_) {
        std::queue<TimestampedFrame> cleaned_buffer;
        
        while (!buffer.empty()) {
            const auto& frame = buffer.front();
            double age_ms = std::chrono::duration<double, std::milli>(now - frame.software_timestamp).count();  // Changed from 'timestamp'
            
            if (age_ms < max_age_ms) {
                cleaned_buffer.push(frame);
            } else {
                stats_.total_frames_dropped++;
            }
            
            buffer.pop();
        }
        
        buffer = std::move(cleaned_buffer);
    }
}

bool SynchronizedCameraApp::captureFrame(SynchronizedFrameSet& frame_set, int timeout_ms) {
    std::unique_lock<std::mutex> lock(sync_mutex_);
    
    // Wait for synchronized frame or timeout
    if (sync_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
                         [this] { return !synchronized_frames_.empty() || !is_running_; })) {
        
        if (!synchronized_frames_.empty()) {
            frame_set = synchronized_frames_.front();
            synchronized_frames_.pop();
            return true;
        }
    }
    
    return false;
}

void SynchronizedCameraApp::setThreadPriority() {
    // Platform-specific thread priority setting would go here
    // For now, just a placeholder
    std::cout << "Thread priority optimization enabled" << std::endl;
}

bool SynchronizedCameraApp::refreshColorAutofocus() {
    if (!sensor_) return false;
    return sensor_->RefreshColorAutofocus();
}

bool SynchronizedCameraApp::setColorExposure(bool auto_mode, float exposure, float white_balance) {
    if (!sensor_) return false;
    return sensor_->SetColorSensorControl(auto_mode, exposure, white_balance);
}

bool SynchronizedCameraApp::setDepthExposure(bool auto_mode, float exposure, float gain) {
    if (!sensor_) return false;
    return sensor_->SetDepthSensorControl(auto_mode, exposure, gain);
}

bool SynchronizedCameraApp::setLaserPower(float power) {
    if (!sensor_) return false;
    return sensor_->SetLaserPower(power);
}

void SynchronizedCameraApp::printCameraInfo() const {
    if (!sensor_) {
        std::cout << "No sensor available" << std::endl;
        return;
    }
    
    std::cout << "\nCamera Information" << std::endl;
    std::cout << "Color Sensor: " << sensor_->GetColorSensorInfo() << std::endl;
    std::cout << "Depth Sensor: " << sensor_->GetDepthSensorInfo() << std::endl;
}

void SynchronizedCameraApp::printSupportedFormats() const {
    if (!sensor_) {
        std::cout << "No sensor available" << std::endl;
        return;
    }
    
    sensor_->PrintSupportedFormats();
}

void SynchronizedCameraApp::resetStats() {
    stats_ = SyncStats();
    std::cout << "Statistics reset" << std::endl;
}

} // namespace sensor
} // namespace determ