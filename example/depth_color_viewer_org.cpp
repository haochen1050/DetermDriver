#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <libcamera/libcamera.h>
#include <thread>
#include <mutex>
#include <queue>
#include <chrono>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <memory>
#include <sys/mman.h>
#include <unistd.h>

using namespace libcamera;

// Calibration data structure
struct CalibratedCameraSystem {
    // Stereo calibration parameters
    cv::Mat R, T, E, F;  // Rotation, Translation, Essential, Fundamental matrices
    cv::Mat R1, R2, P1, P2, Q;  // Rectification and projection matrices
    cv::Mat map1_rgb, map2_rgb, map1_ir, map2_ir;  // Rectification maps
    
    // Individual camera parameters
    cv::Mat rgb_camera_matrix, rgb_distortion;
    cv::Mat ir_camera_matrix, ir_distortion;
    
    double baseline;
    cv::Size image_size;
    bool calibrated;
    
    CalibratedCameraSystem() : baseline(0), calibrated(false) {}
    
    // Load calibration from XML file
    bool loadCalibration(const std::string& calibration_file = "stereo_calibration.xml") {
        try {
            cv::FileStorage fs(calibration_file, cv::FileStorage::READ);
            
            if (!fs.isOpened()) {
                std::cerr << "Cannot open calibration file: " << calibration_file << std::endl;
                return false;
            }
            
            // Load stereo calibration matrices
            fs["rotation_matrix"] >> R;
            fs["translation_vector"] >> T;
            fs["essential_matrix"] >> E;
            fs["fundamental_matrix"] >> F;
            fs["rectification_matrix_1"] >> R1;
            fs["rectification_matrix_2"] >> R2;
            fs["projection_matrix_1"] >> P1;
            fs["projection_matrix_2"] >> P2;
            fs["disparity_to_depth_mapping_matrix"] >> Q;
            
            // Load rectification maps
            fs["rectification_map_1_rgb"] >> map1_rgb;
            fs["rectification_map_2_rgb"] >> map2_rgb;
            fs["rectification_map_1_ir"] >> map1_ir;
            fs["rectification_map_2_ir"] >> map2_ir;
            
            // Load camera matrices
            fs["rgb_camera_matrix"] >> rgb_camera_matrix;
            fs["rgb_distortion"] >> rgb_distortion;
            fs["ir_camera_matrix"] >> ir_camera_matrix;
            fs["ir_distortion"] >> ir_distortion;
            
            // Load other parameters
            fs["baseline"] >> baseline;
            
            int width, height;
            fs["image_width"] >> width;
            fs["image_height"] >> height;
            image_size = cv::Size(width, height);
            
            fs.release();
            
            calibrated = true;
            
            std::cout << "✅ Calibration loaded successfully!" << std::endl;
            std::cout << "   Baseline: " << baseline << " mm" << std::endl;
            std::cout << "   Image size: " << image_size.width << "x" << image_size.height << std::endl;
            
            return true;
            
        } catch (const cv::Exception& e) {
            std::cerr << "OpenCV error loading calibration: " << e.what() << std::endl;
            return false;
        } catch (const std::exception& e) {
            std::cerr << "Error loading calibration: " << e.what() << std::endl;
            return false;
        }
    }
    
    // Apply rectification to RGB frame
    cv::Mat rectifyRGB(const cv::Mat& rgb_frame) {
        if (!calibrated || map1_rgb.empty() || map2_rgb.empty()) {
            return rgb_frame.clone();
        }
        
        cv::Mat rectified;
        cv::remap(rgb_frame, rectified, map1_rgb, map2_rgb, cv::INTER_LINEAR);
        return rectified;
    }
    
    // Apply rectification to IR/Depth frame
    cv::Mat rectifyIR(const cv::Mat& ir_frame) {
        if (!calibrated || map1_ir.empty() || map2_ir.empty()) {
            return ir_frame.clone();
        }
        
        cv::Mat rectified;
        cv::remap(ir_frame, rectified, map1_ir, map2_ir, cv::INTER_LINEAR);
        return rectified;
    }
};

// Frame data structure
struct FrameData {
    cv::Mat data;
    cv::Mat rectified_data;
    double timestamp;
    long long frame_number;
    bool is_rectified;
    
    FrameData() : timestamp(0), frame_number(0), is_rectified(false) {}
    
    FrameData(const cv::Mat& img, double ts, long long fn) 
        : data(img.clone()), timestamp(ts), frame_number(fn), is_rectified(false) {}
};

// Synchronized frame structure
struct SynchronizedFrame {
    FrameData depth;
    FrameData color;
    double avg_timestamp;
    double time_diff_ms;
    
    SynchronizedFrame() : avg_timestamp(0), time_diff_ms(0) {}
    
    SynchronizedFrame(const FrameData& d, const FrameData& c) 
        : depth(d), color(c) {
        avg_timestamp = (depth.timestamp + color.timestamp) / 2.0;
        time_diff_ms = std::abs(depth.timestamp - color.timestamp);
    }
};

// libcamera wrapper class
class LibcameraCapture {
private:
    std::unique_ptr<CameraManager> camera_manager_;
    std::shared_ptr<Camera> camera_;
    std::unique_ptr<CameraConfiguration> config_;
    std::unique_ptr<FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<Request>> requests_;
    
    int width_, height_, fps_;
    bool initialized_;
    std::atomic<bool> capturing_;
    
    std::mutex frame_mutex_;
    cv::Mat latest_frame_;
    double latest_timestamp_;
    bool new_frame_available_;
    
    Stream* stream_;

public:
    LibcameraCapture(int width = 640, int height = 480, int fps = 30) 
        : width_(width), height_(height), fps_(fps), initialized_(false), 
          capturing_(false), latest_timestamp_(0), new_frame_available_(false), stream_(nullptr) {}
    
    ~LibcameraCapture() {
        stop();
        cleanup();
    }
    
    bool initialize() {
        try {
            // Initialize camera manager
            camera_manager_ = std::make_unique<CameraManager>();
            int ret = camera_manager_->start();
            if (ret) {
                std::cerr << "Failed to start camera manager" << std::endl;
                return false;
            }
            
            // Get available cameras
            auto cameras = camera_manager_->cameras();
            if (cameras.empty()) {
                std::cerr << "No cameras found" << std::endl;
                return false;
            }
            
            // Use the first available camera
            camera_ = cameras[0];
            
            // Acquire the camera
            ret = camera_->acquire();
            if (ret) {
                std::cerr << "Failed to acquire camera" << std::endl;
                return false;
            }
            
            // Generate camera configuration
            config_ = camera_->generateConfiguration({StreamRole::Viewfinder});
            if (!config_) {
                std::cerr << "Failed to generate camera configuration" << std::endl;
                return false;
            }
            
            // Configure stream
            StreamConfiguration& stream_config = config_->at(0);
            stream_config.size.width = width_;
            stream_config.size.height = height_;
            stream_config.pixelFormat = formats::RGB888;  // Use RGB888 instead of BGR888
            
            // Validate configuration
            CameraConfiguration::Status validation = config_->validate();
            if (validation == CameraConfiguration::Invalid) {
                std::cerr << "Invalid camera configuration" << std::endl;
                return false;
            }
            
            if (validation == CameraConfiguration::Adjusted) {
                std::cout << "Camera configuration adjusted:" << std::endl;
                std::cout << "  Size: " << stream_config.size.width << "x" << stream_config.size.height << std::endl;
                width_ = stream_config.size.width;
                height_ = stream_config.size.height;
            }
            
            // Apply configuration
            ret = camera_->configure(config_.get());
            if (ret) {
                std::cerr << "Failed to configure camera" << std::endl;
                return false;
            }
            
            // Get the stream
            stream_ = stream_config.stream();
            
            // Allocate buffers
            allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
            ret = allocator_->allocate(stream_);
            if (ret < 0) {
                std::cerr << "Failed to allocate buffers" << std::endl;
                return false;
            }
            
            // Create requests
            const std::vector<std::unique_ptr<FrameBuffer>>& buffers = allocator_->buffers(stream_);
            for (const auto& buffer : buffers) {
                auto request = camera_->createRequest();
                if (!request) {
                    std::cerr << "Failed to create request" << std::endl;
                    return false;
                }
                
                ret = request->addBuffer(stream_, buffer.get());
                if (ret < 0) {
                    std::cerr << "Failed to add buffer to request" << std::endl;
                    return false;
                }
                
                requests_.push_back(std::move(request));
            }
            
            // Connect request completed signal
            camera_->requestCompleted.connect(this, &LibcameraCapture::requestComplete);
            
            // Set frame rate control
            ControlList controls;
            controls.set(controls::FrameDurationLimits, 
                        libcamera::Span<const int64_t, 2>({1000000 / fps_, 1000000 / fps_}));
            
            ret = camera_->start(&controls);
            if (ret) {
                std::cerr << "Failed to start camera" << std::endl;
                return false;
            }
            
            initialized_ = true;
            
            std::cout << "✅ libcamera initialized successfully!" << std::endl;
            std::cout << "   Resolution: " << width_ << "x" << height_ << std::endl;
            std::cout << "   FPS: " << fps_ << std::endl;
            std::cout << "   Camera: " << camera_->id() << std::endl;
            
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "libcamera initialization error: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool start() {
        if (!initialized_) {
            std::cerr << "Camera not initialized" << std::endl;
            return false;
        }
        
        capturing_ = true;
        
        // Queue initial requests
        for (auto& request : requests_) {
            camera_->queueRequest(request.get());
        }
        
        std::cout << "✅ libcamera capture started" << std::endl;
        return true;
    }
    
    void stop() {
        if (capturing_) {
            capturing_ = false;
            
            if (camera_) {
                camera_->stop();
            }
            
            std::cout << "libcamera capture stopped" << std::endl;
        }
    }
    
    bool getFrame(cv::Mat& frame, double& timestamp) {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        
        if (!new_frame_available_ || latest_frame_.empty()) {
            return false;
        }
        
        frame = latest_frame_.clone();
        timestamp = latest_timestamp_;
        new_frame_available_ = false;
        
        return true;
    }
    
    bool isCapturing() const {
        return capturing_;
    }
    
    cv::Size getFrameSize() const {
        return cv::Size(width_, height_);
    }

private:
    void requestComplete(Request* request) {
        if (!capturing_) {
            return;
        }
        
        if (request->status() == Request::RequestComplete) {
            // Get the buffer
            const Request::BufferMap& buffers = request->buffers();
            auto it = buffers.find(stream_);
            if (it != buffers.end()) {
                FrameBuffer* buffer = it->second;
                
                // Process the frame
                processFrame(buffer);
                
                // Requeue the request for continuous capture
                if (capturing_) {
                    request->reuse(Request::ReuseBuffers);
                    camera_->queueRequest(request);
                }
            }
        }
    }
    
    void processFrame(FrameBuffer* buffer) {
        // Get buffer data
        const FrameBuffer::Plane& plane = buffer->planes()[0];
        
        // Map the buffer to get the data
        void* data = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
        if (data == MAP_FAILED) {
            std::cerr << "Failed to map buffer" << std::endl;
            return;
        }
        
        // Get timestamp
        double timestamp = getCurrentTimestampMs();
        
        // Create OpenCV Mat from buffer data
        // Note: libcamera RGB888 format is actually BGR888 ordered in memory for this camera
        cv::Mat bgr_frame(height_, width_, CV_8UC3, data);
        
        // Store the frame (no color conversion needed - data is already in BGR format)
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_ = bgr_frame.clone();  // Deep copy
            latest_timestamp_ = timestamp;
            new_frame_available_ = true;
        }
        
        // Unmap the buffer
        munmap(data, plane.length);
    }
    
    double getCurrentTimestampMs() {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
        return static_cast<double>(millis);
    }
    
    void cleanup() {
        // Stop requests
        requests_.clear();
        
        // Release allocator
        if (allocator_) {
            allocator_.reset();
        }
        
        // Release camera
        if (camera_) {
            camera_->release();
            camera_.reset();
        }
        
        // Stop camera manager
        if (camera_manager_) {
            camera_manager_->stop();
            camera_manager_.reset();
        }
    }
};

class SynchronizedCameraCapture {
private:
    // Calibration system
    CalibratedCameraSystem calibration_;
    
    // Camera configurations
    int depth_fps_;
    int color_fps_;
    double sync_tolerance_ms_;
    
    // RealSense components
    rs2::pipeline rs_pipeline_;
    rs2::config rs_config_;
    
    // libcamera color camera
    std::unique_ptr<LibcameraCapture> libcamera_;
    
    // Thread-safe queues and buffers
    std::queue<FrameData> depth_queue_;
    std::queue<FrameData> color_queue_;
    std::queue<SynchronizedFrame> synced_queue_;
    
    std::deque<FrameData> depth_buffer_;
    std::deque<FrameData> color_buffer_;
    
    // Threading components
    std::atomic<bool> running_;
    std::vector<std::thread> threads_;
    
    // Mutexes for thread safety
    mutable std::mutex depth_queue_mutex_;
    mutable std::mutex color_queue_mutex_;
    mutable std::mutex synced_queue_mutex_;
    mutable std::mutex buffer_mutex_;
    
    // Condition variables for synchronization
    std::condition_variable synced_cv_;
    
    // Constants
    static constexpr size_t MAX_QUEUE_SIZE = 10;
    static constexpr size_t MAX_BUFFER_SIZE = 60;

public:
    SynchronizedCameraCapture(int depth_fps = 30, int color_fps = 30, 
                            double sync_tolerance_ms = 33.0,
                            const std::string& calibration_file = "stereo_calibration.xml")
        : depth_fps_(depth_fps), color_fps_(color_fps), sync_tolerance_ms_(sync_tolerance_ms), 
          running_(false) {
        
        // Load calibration
        if (!calibration_.loadCalibration(calibration_file)) {
            std::cerr << "⚠️  Warning: Failed to load calibration. Running without rectification." << std::endl;
        }
        
        initRealSense();
        initLibcamera();
    }
    
    ~SynchronizedCameraCapture() {
        cleanup();
    }

private:
    void initRealSense() {
        try {
            // Configure depth stream only (D421 has no internal color camera)
            rs_config_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, depth_fps_);
            
            std::cout << "RealSense D421 configured for depth stream only" << std::endl;
            
            // Start the pipeline
            auto profile = rs_pipeline_.start(rs_config_);
            
            // Configure depth sensor options
            auto device = profile.get_device();
            auto depth_sensor = device.first<rs2::depth_sensor>();
            
            // Set high accuracy preset
            if (depth_sensor.supports(RS2_OPTION_VISUAL_PRESET)) {
                depth_sensor.set_option(RS2_OPTION_VISUAL_PRESET, 4); // High accuracy
            }
            
            // Enable infrared emitter
            if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
                depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1);
            }
            
            std::cout << "✅ RealSense D421 initialized at " << depth_fps_ << " FPS" << std::endl;
            
        } catch (const rs2::error& e) {
            std::cerr << "Failed to initialize RealSense: " << e.what() << std::endl;
            throw;
        }
    }
    
    void initLibcamera() {
        try {
            std::cout << "Initializing Pi Camera v3 via libcamera..." << std::endl;
            
            libcamera_ = std::make_unique<LibcameraCapture>(640, 480, color_fps_);
            
            if (!libcamera_->initialize()) {
                throw std::runtime_error("Failed to initialize libcamera");
            }
            
            std::cout << "✅ Pi Camera v3 initialized successfully!" << std::endl;
            
        } catch (const std::exception& e) {
            std::cerr << "Failed to initialize libcamera: " << e.what() << std::endl;
            throw;
        }
    }
    
    double getCurrentTimestampMs() {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
        return static_cast<double>(millis);
    }
    
    void captureDepthFrames() {
        while (running_) {
            try {
                auto frames = rs_pipeline_.wait_for_frames(1000);
                auto depth_frame = frames.get_depth_frame();
                
                if (depth_frame) {
                    double timestamp = getCurrentTimestampMs();
                    
                    // Convert to OpenCV Mat
                    cv::Mat depth_image(cv::Size(depth_frame.get_width(), depth_frame.get_height()),
                                      CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
                    
                    FrameData frame_data(depth_image, timestamp, depth_frame.get_frame_number());
                    
                    // Apply rectification if calibrated
                    if (calibration_.calibrated) {
                        frame_data.rectified_data = calibration_.rectifyIR(depth_image);
                        frame_data.is_rectified = true;
                    }
                    
                    // Add to queue
                    {
                        std::lock_guard<std::mutex> lock(depth_queue_mutex_);
                        if (depth_queue_.size() >= MAX_QUEUE_SIZE) {
                            depth_queue_.pop();
                        }
                        depth_queue_.push(frame_data);
                    }
                }
                
            } catch (const rs2::error& e) {
                if (running_) {
                    std::cerr << "Depth capture error: " << e.what() << std::endl;
                }
                break;
            }
        }
    }
    
    void captureColorFrames() {
        while (running_) {
            try {
                cv::Mat color_image;
                double timestamp;
                
                if (libcamera_->getFrame(color_image, timestamp)) {
                    FrameData frame_data(color_image, timestamp, static_cast<long long>(timestamp));
                    
                    // Apply rectification if calibrated
                    if (calibration_.calibrated) {
                        frame_data.rectified_data = calibration_.rectifyRGB(color_image);
                        frame_data.is_rectified = true;
                    }
                    
                    {
                        std::lock_guard<std::mutex> lock(color_queue_mutex_);
                        if (color_queue_.size() >= MAX_QUEUE_SIZE) {
                            color_queue_.pop();
                        }
                        color_queue_.push(frame_data);
                    }
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(1000 / color_fps_));
                
            } catch (const std::exception& e) {
                if (running_) {
                    std::cerr << "Color capture error: " << e.what() << std::endl;
                }
                break;
            }
        }
    }
    
    void synchronizeFrames() {
        while (running_) {
            try {
                FrameData depth_frame, color_frame;
                bool have_depth = false, have_color = false;
                
                // Get frames from queues
                {
                    std::lock_guard<std::mutex> lock(depth_queue_mutex_);
                    if (!depth_queue_.empty()) {
                        depth_frame = depth_queue_.front();
                        depth_queue_.pop();
                        have_depth = true;
                    }
                }
                
                {
                    std::lock_guard<std::mutex> lock(color_queue_mutex_);
                    if (!color_queue_.empty()) {
                        color_frame = color_queue_.front();
                        color_queue_.pop();
                        have_color = true;
                    }
                }
                
                // Add frames to buffers and find synchronized pairs
                {
                    std::lock_guard<std::mutex> lock(buffer_mutex_);
                    
                    if (have_depth) {
                        if (depth_buffer_.size() >= MAX_BUFFER_SIZE) {
                            depth_buffer_.pop_front();
                        }
                        depth_buffer_.push_back(depth_frame);
                    }
                    
                    if (have_color) {
                        if (color_buffer_.size() >= MAX_BUFFER_SIZE) {
                            color_buffer_.pop_front();
                        }
                        color_buffer_.push_back(color_frame);
                    }
                    
                    findSynchronizedPairs();
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                
            } catch (const std::exception& e) {
                if (running_) {
                    std::cerr << "Synchronization error: " << e.what() << std::endl;
                }
                break;
            }
        }
    }
    
    void findSynchronizedPairs() {
        for (auto depth_it = depth_buffer_.begin(); depth_it != depth_buffer_.end();) {
            const FrameData& depth_frame = *depth_it;
            
            FrameData* best_match = nullptr;
            double min_time_diff = std::numeric_limits<double>::max();
            auto best_color_it = color_buffer_.end();
            
            // Find best matching color frame
            for (auto color_it = color_buffer_.begin(); color_it != color_buffer_.end(); ++color_it) {
                double time_diff = std::abs(depth_frame.timestamp - color_it->timestamp);
                
                if (time_diff < min_time_diff && time_diff <= sync_tolerance_ms_) {
                    min_time_diff = time_diff;
                    best_match = &(*color_it);
                    best_color_it = color_it;
                }
            }
            
            if (best_match) {
                // Create synchronized frame
                SynchronizedFrame synced_frame(depth_frame, *best_match);
                synced_frame.time_diff_ms = min_time_diff;
                
                // Add to synced queue
                {
                    std::lock_guard<std::mutex> lock(synced_queue_mutex_);
                    if (synced_queue_.size() >= 5) {
                        synced_queue_.pop();
                    }
                    synced_queue_.push(synced_frame);
                }
                
                synced_cv_.notify_one();
                
                // Remove used frames
                depth_it = depth_buffer_.erase(depth_it);
                color_buffer_.erase(best_color_it);
            } else {
                ++depth_it;
            }
        }
    }

public:
    void startCapture() {
        running_ = true;
        
        // Start libcamera
        if (!libcamera_->start()) {
            std::cerr << "Failed to start libcamera capture" << std::endl;
            return;
        }
        
        threads_.emplace_back(&SynchronizedCameraCapture::captureDepthFrames, this);
        threads_.emplace_back(&SynchronizedCameraCapture::captureColorFrames, this);
        threads_.emplace_back(&SynchronizedCameraCapture::synchronizeFrames, this);
        
        std::cout << "✅ Synchronized capture started. ";
        if (calibration_.calibrated) {
            std::cout << "Using stereo calibration with " << calibration_.baseline << "mm baseline.";
        } else {
            std::cout << "Running without calibration.";
        }
        std::cout << std::endl;
    }
    
    void stopCapture() {
        running_ = false;
        synced_cv_.notify_all();
        
        // Stop libcamera
        if (libcamera_) {
            libcamera_->stop();
        }
        
        for (auto& thread : threads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
        threads_.clear();
        
        std::cout << "Synchronized capture stopped." << std::endl;
    }
    
    bool getSynchronizedFrame(SynchronizedFrame& frame, int timeout_ms = 1000) {
        std::unique_lock<std::mutex> lock(synced_queue_mutex_);
        
        if (synced_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
                               [this] { return !synced_queue_.empty() || !running_; })) {
            
            if (!synced_queue_.empty()) {
                frame = synced_queue_.front();
                synced_queue_.pop();
                return true;
            }
        }
        
        return false;
    }
    
    // Save frame pair
    void saveFramePair(const SynchronizedFrame& frame, 
                      const std::string& save_dir = "frames", 
                      const std::string& frame_id = "") {
        
        std::string id = frame_id.empty() ? std::to_string(static_cast<long long>(frame.avg_timestamp)) : frame_id;
        
        // Create directory
        std::string mkdir_cmd = "mkdir -p " + save_dir;
        system(mkdir_cmd.c_str());
        
        // Save original frames
        cv::imwrite(save_dir + "/depth_original_" + id + ".png", frame.depth.data);
        cv::imwrite(save_dir + "/color_original_" + id + ".jpg", frame.color.data);
        
        // Save rectified frames if available
        if (frame.depth.is_rectified) {
            cv::imwrite(save_dir + "/depth_rectified_" + id + ".png", frame.depth.rectified_data);
        }
        if (frame.color.is_rectified) {
            cv::imwrite(save_dir + "/color_rectified_" + id + ".jpg", frame.color.rectified_data);
        }
        
        std::cout << "💾 Saved frame pair " << id << " (sync: " << frame.time_diff_ms << "ms)" << std::endl;
    }
    
    void cleanup() {
        stopCapture();
        
        try {
            rs_pipeline_.stop();
        } catch (...) {}
        
        if (libcamera_) {
            libcamera_.reset();
        }
        
        std::cout << "Resources cleaned up." << std::endl;
    }
};

// Main function
int main() {
    try {
        std::cout << "=== D421 Depth + Pi Camera v3 Synchronized Capture ===" << std::endl;
        
        // Initialize synchronized capture system
        SynchronizedCameraCapture capture(
            30, 30,                    // FPS for depth and color
            16.0,                      // Sync tolerance (ms)
            "stereo_calibration.xml"   // Calibration file
        );
        
        // Start capture
        capture.startCapture();
        
        std::cout << "\n🚀 Starting synchronized visualization..." << std::endl;
        std::cout << "Controls:" << std::endl;
        std::cout << "  's' - Save frame pair" << std::endl;
        std::cout << "  'r' - Toggle rectified/original view" << std::endl;
        std::cout << "  't' - Toggle timestamp printing" << std::endl;
        std::cout << "  'q' - Quit" << std::endl;
        std::cout << "\n=== Frame Statistics ===" << std::endl;
        
        int frame_counter = 0;
        bool print_timestamps = true;
        bool show_rectified = true;
        
        // Statistics tracking
        std::vector<double> sync_diffs;
        double total_sync_time = 0;
        double min_sync = std::numeric_limits<double>::max();
        double max_sync = 0;
        
        while (true) {
            SynchronizedFrame synced_frame;
            
            if (capture.getSynchronizedFrame(synced_frame, 1000)) {
                
                // Print timestamp info
                if (print_timestamps) {
                    std::cout << std::fixed << std::setprecision(3);
                    std::cout << "Frame #" << std::setw(4) << frame_counter 
                              << " | Sync: " << std::setw(6) << synced_frame.time_diff_ms << "ms" << std::endl;
                }
                
                // Update statistics
                sync_diffs.push_back(synced_frame.time_diff_ms);
                total_sync_time += synced_frame.time_diff_ms;
                min_sync = std::min(min_sync, synced_frame.time_diff_ms);
                max_sync = std::max(max_sync, synced_frame.time_diff_ms);
                
                // Prepare frames for display
                cv::Mat depth_display, color_display;
                
                // Choose which frames to display (original or rectified)
                if (show_rectified && synced_frame.depth.is_rectified) {
                    depth_display = synced_frame.depth.rectified_data.clone();
                    color_display = synced_frame.color.rectified_data.clone();
                } else {
                    depth_display = synced_frame.depth.data.clone();
                    color_display = synced_frame.color.data.clone();
                }
                
                // Convert depth to colormap for visualization
                cv::Mat depth_8bit, depth_colormap;
                depth_display.convertTo(depth_8bit, CV_8UC1, 255.0/5000.0);
                cv::applyColorMap(depth_8bit, depth_colormap, cv::COLORMAP_JET);
                
                // Resize frames to ensure they're the same size
                cv::Size display_size(640, 480);
                cv::resize(depth_colormap, depth_colormap, display_size);
                cv::resize(color_display, color_display, display_size);
                
                // Create combined display (side by side)
                cv::Mat combined_display(display_size.height, display_size.width * 2, CV_8UC3);
                
                // Copy depth to left side
                depth_colormap.copyTo(combined_display(cv::Rect(0, 0, display_size.width, display_size.height)));
                
                // Copy color to right side  
                color_display.copyTo(combined_display(cv::Rect(display_size.width, 0, display_size.width, display_size.height)));
                
                // Add labels
                cv::putText(combined_display, "D421 Depth", cv::Point(10, 30), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                cv::putText(combined_display, "Pi Camera v3", cv::Point(display_size.width + 10, 30), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                
                // Add calibration status on depth side
                std::string calib_status = show_rectified ? "RECTIFIED" : "ORIGINAL";
                cv::putText(combined_display, calib_status, cv::Point(10, 60), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                
                // Add statistics on color side (right side)
                std::string frame_info = "Frame #" + std::to_string(frame_counter);
                cv::putText(combined_display, frame_info, cv::Point(display_size.width + 10, 60), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
                
                std::string sync_info = "Sync: " + std::to_string(synced_frame.time_diff_ms).substr(0, 5) + "ms";
                cv::putText(combined_display, sync_info, cv::Point(display_size.width + 10, 90), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
                
                if (frame_counter > 0) {
                    std::string avg_sync = "Avg: " + std::to_string(total_sync_time / frame_counter).substr(0, 5) + "ms";
                    cv::putText(combined_display, avg_sync, cv::Point(display_size.width + 10, 120), 
                               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
                    
                    std::string min_max = "Min/Max: " + std::to_string(min_sync).substr(0, 4) + "/" + std::to_string(max_sync).substr(0, 4) + "ms";
                    cv::putText(combined_display, min_max, cv::Point(display_size.width + 10, 150), 
                               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
                }
                
                // Add control instructions at bottom
                cv::putText(combined_display, "Controls: 's'=Save 'r'=Rect/Orig 't'=Timestamps 'q'=Quit", 
                           cv::Point(10, display_size.height - 10), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                
                // Display the combined window
                cv::imshow("D421 Depth + Pi Camera v3 - Synchronized View", combined_display);
                
                // Handle keyboard input
                char key = cv::waitKey(1) & 0xFF;
                
                if (key == 'q') {
                    break;
                } else if (key == 's') {
                    capture.saveFramePair(synced_frame, "frames", 
                                        std::to_string(frame_counter));
                } else if (key == 'r') {
                    show_rectified = !show_rectified;
                    std::cout << "📐 Switched to " << (show_rectified ? "rectified" : "original") << " view" << std::endl;
                } else if (key == 't') {
                    print_timestamps = !print_timestamps;
                    std::cout << "⏰ Timestamp printing " << (print_timestamps ? "ON" : "OFF") << std::endl;
                }
                
                frame_counter++;
                
                // Print summary statistics every 100 frames
                if (frame_counter % 100 == 0) {
                    std::cout << "\n📊 === Statistics Summary ===" << std::endl;
                    std::cout << "   Frames processed: " << frame_counter << std::endl;
                    std::cout << "   Average sync time: " << (total_sync_time / frame_counter) << "ms" << std::endl;
                    std::cout << "   Min/Max sync: " << min_sync << "/" << max_sync << "ms" << std::endl;
                    std::cout << "=============================\n" << std::endl;
                }
                
            } else {
                std::cout << "⏳ Waiting for synchronized frame..." << std::endl;
            }
        }
        
        // Final statistics
        std::cout << "\n🏁 === Final Statistics ===" << std::endl;
        std::cout << "Total frames: " << frame_counter << std::endl;
        if (frame_counter > 0) {
            std::cout << "Overall average sync: " << (total_sync_time / frame_counter) << "ms" << std::endl;
            std::cout << "Min/Max sync: " << min_sync << "/" << max_sync << "ms" << std::endl;
            
            // Calculate percentiles
            std::sort(sync_diffs.begin(), sync_diffs.end());
            if (!sync_diffs.empty()) {
                std::cout << "Median sync: " << sync_diffs[sync_diffs.size() / 2] << "ms" << std::endl;
                std::cout << "90th percentile: " << sync_diffs[sync_diffs.size() * 0.9] << "ms" << std::endl;
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
        return -1;
    }
    
    cv::destroyAllWindows();
    return 0;
}