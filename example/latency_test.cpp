// fps_test.cpp - Test different frame rates on Pi Camera V3
#include <libcamera/camera_manager.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/libcamera.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include <algorithm>
#include <chrono>
#include <cmath>  // Add this for std::pow and std::sqrt
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <signal.h>
#include <thread>
#include <vector>

using namespace libcamera;

class FrameRateTester {
public:
    struct FrameInfo {
        uint64_t capture_time_us;
        uint64_t completion_time_us;
        uint64_t interval_us;  // Time since previous frame
        uint32_t frame_id;
        
        FrameInfo() : capture_time_us(0), completion_time_us(0), interval_us(0), frame_id(0) {}
    };

private:
    std::unique_ptr<CameraManager> camera_manager;
    std::shared_ptr<Camera> camera;
    std::unique_ptr<CameraConfiguration> config;
    std::unique_ptr<FrameBufferAllocator> allocator;
    Stream* stream;
    
    std::vector<FrameInfo> frames;
    bool running;
    uint32_t frame_counter;
    size_t current_buffer_index;
    uint64_t last_capture_time;
    
    Size resolution;
    PixelFormat pixel_format;
    float target_fps;

public:
    FrameRateTester() : running(false), frame_counter(0), current_buffer_index(0), last_capture_time(0) {}

    ~FrameRateTester() {
        Cleanup();
    }

    uint64_t GetTimestamp() const {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count();
    }

    bool Initialize(int width, int height, const std::string& format, float fps) {
        resolution = Size(width, height);
        target_fps = fps;
        
        if (format == "YUYV") {
            pixel_format = formats::YUYV;
        } else if (format == "RGB888") {
            pixel_format = formats::RGB888;
        } else {
            pixel_format = formats::YUYV;
        }

        camera_manager = std::make_unique<CameraManager>();
        int ret = camera_manager->start();
        if (ret) {
            std::cerr << "Failed to start camera manager" << std::endl;
            return false;
        }

        auto cameras = camera_manager->cameras();
        if (cameras.empty()) {
            std::cerr << "No cameras found" << std::endl;
            return false;
        }

        // Find Pi camera
        for (auto& cam : cameras) {
            std::string cam_id = cam->id();
            if (cam_id.find("imx708") != std::string::npos || 
                cam_id.find("/base/soc/") != std::string::npos) {
                camera = cam;
                break;
            }
        }
        
        if (!camera) {
            std::cerr << "No Pi camera found" << std::endl;
            return false;
        }

        ret = camera->acquire();
        if (ret) {
            std::cerr << "Failed to acquire camera" << std::endl;
            return false;
        }

        config = camera->generateConfiguration({StreamRole::VideoRecording});
        if (!config) {
            std::cerr << "Failed to generate configuration" << std::endl;
            return false;
        }

        StreamConfiguration& stream_config = config->at(0);
        stream_config.pixelFormat = pixel_format;
        stream_config.size = resolution;

        CameraConfiguration::Status validation = config->validate();
        if (validation == CameraConfiguration::Invalid) {
            std::cerr << "Invalid configuration" << std::endl;
            return false;
        }
        
        if (validation == CameraConfiguration::Adjusted) {
            resolution = stream_config.size;
            pixel_format = stream_config.pixelFormat;
        }

        ret = camera->configure(config.get());
        if (ret) {
            std::cerr << "Failed to configure camera" << std::endl;
            return false;
        }

        stream = stream_config.stream();

        std::cout << "Camera configured:" << std::endl;
        std::cout << "  Resolution: " << resolution.toString() << std::endl;
        std::cout << "  Format: " << pixel_format.toString() << std::endl;
        std::cout << "  Target FPS: " << target_fps << std::endl;

        allocator = std::make_unique<FrameBufferAllocator>(camera);
        ret = allocator->allocate(stream);
        if (ret < 0) {
            std::cerr << "Failed to allocate buffers" << std::endl;
            return false;
        }

        std::cout << "Allocated " << allocator->buffers(stream).size() << " buffers" << std::endl;
        return true;
    }

    void SetFrameRateControls() {
        // Try to set frame duration limits (simplified approach)
        const ControlInfoMap& controls = camera->controls();
        
        if (controls.find(&controls::FrameDurationLimits) != controls.end()) {
            std::cout << "Note: FrameDurationLimits control available but not set (API compatibility)" << std::endl;
        } else {
            std::cout << "Note: Frame rate will be controlled by software pacing" << std::endl;
        }
        
        std::cout << "Using software frame pacing for " << target_fps << " fps target" << std::endl;
    }

    FrameInfo CaptureFrame() {
        FrameInfo info;
        info.frame_id = ++frame_counter;

        std::unique_ptr<Request> request = camera->createRequest();
        if (!request) {
            return info;
        }

        auto& buffers = allocator->buffers(stream);
        if (buffers.empty()) {
            return info;
        }

        const std::unique_ptr<FrameBuffer>& buffer = buffers[current_buffer_index];
        current_buffer_index = (current_buffer_index + 1) % buffers.size();

        int ret = request->addBuffer(stream, buffer.get());
        if (ret < 0) {
            return info;
        }

        // Set basic controls
        ControlList controls;
        const ControlInfoMap& supported = camera->controls();
        
        try {
            if (supported.find(&controls::AwbMode) != supported.end()) {
                controls.set(controls::AwbMode, controls::AwbAuto);
            }
            
            for (const auto& [id, value] : controls) {
                request->controls().set(id, value);
            }
        } catch (...) {
            // Continue without controls
        }

        info.capture_time_us = GetTimestamp();
        
        // Calculate interval from last frame
        if (last_capture_time > 0) {
            info.interval_us = info.capture_time_us - last_capture_time;
        }
        last_capture_time = info.capture_time_us;
        
        ret = camera->queueRequest(request.get());
        if (ret < 0) {
            return info;
        }

        // Wait for completion
        bool completed = false;
        auto start_wait = std::chrono::steady_clock::now();
        auto timeout = std::chrono::milliseconds(200);
        
        Request* req_ptr = request.get();

        while (!completed && running) {
            if (req_ptr->status() == Request::RequestComplete) {
                info.completion_time_us = GetTimestamp();
                completed = true;
                break;
            }

            if (std::chrono::steady_clock::now() - start_wait > timeout) {
                break;
            }

            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

        return info;
    }

    void TestFrameRate(int num_frames = 100, bool verbose = false) {
        std::cout << "\n========== TESTING " << target_fps << " FPS ==========" << std::endl;
        
        int ret = camera->start();
        if (ret) {
            std::cerr << "Failed to start camera" << std::endl;
            return;
        }

        SetFrameRateControls();
        running = true;
        frames.clear();
        frames.reserve(num_frames);
        last_capture_time = 0;

        // Warmup
        for (int i = 0; i < 5 && running; ++i) {
            CaptureFrame();
        }

        std::cout << "Capturing " << num_frames << " frames..." << std::endl;
        auto test_start = std::chrono::steady_clock::now();
        
        for (int i = 0; i < num_frames && running; ++i) {
            auto frame_info = CaptureFrame();
            
            if (frame_info.completion_time_us > 0) {
                frames.push_back(frame_info);
                
                if (verbose && i % 10 == 0) {
                    double actual_fps = (frame_info.interval_us > 0) ? 
                        1000000.0 / frame_info.interval_us : 0.0;
                    std::cout << "Frame " << frame_info.frame_id 
                              << ": interval=" << frame_info.interval_us/1000.0 << "ms"
                              << ", fps=" << std::fixed << std::setprecision(1) << actual_fps << std::endl;
                }
            }
            
            // Frame pacing - sleep to target frame rate
            if (target_fps > 0) {
                uint64_t target_interval_us = static_cast<uint64_t>(1000000.0f / target_fps);
                uint64_t elapsed_us = GetTimestamp() - frame_info.capture_time_us;
                
                if (elapsed_us < target_interval_us) {
                    std::this_thread::sleep_for(std::chrono::microseconds(target_interval_us - elapsed_us));
                }
            }
        }

        auto test_end = std::chrono::steady_clock::now();
        auto test_duration = std::chrono::duration_cast<std::chrono::milliseconds>(test_end - test_start);
        
        camera->stop();
        running = false;
        
        // Calculate statistics
        if (!frames.empty()) {
            // Remove first frame (no interval)
            std::vector<uint64_t> intervals;
            for (size_t i = 1; i < frames.size(); ++i) {
                if (frames[i].interval_us > 0) {
                    intervals.push_back(frames[i].interval_us);
                }
            }
            
            if (!intervals.empty()) {
                std::sort(intervals.begin(), intervals.end());
                
                double mean_interval_us = std::accumulate(intervals.begin(), intervals.end(), 0.0) / intervals.size();
                double median_interval_us = intervals[intervals.size() / 2];
                double min_interval_us = intervals.front();
                double max_interval_us = intervals.back();
                
                double actual_fps = 1000000.0 / mean_interval_us;
                double median_fps = 1000000.0 / median_interval_us;
                double max_fps = 1000000.0 / min_interval_us;
                double min_fps = 1000000.0 / max_interval_us;
                
                std::cout << "\n========== RESULTS ==========" << std::endl;
                std::cout << std::fixed << std::setprecision(2);
                std::cout << "Target FPS:     " << target_fps << std::endl;
                std::cout << "Actual FPS:     " << actual_fps << " (mean)" << std::endl;
                std::cout << "Median FPS:     " << median_fps << std::endl;
                std::cout << "FPS Range:      " << min_fps << " - " << max_fps << std::endl;
                std::cout << "Mean interval:  " << mean_interval_us/1000.0 << " ms" << std::endl;
                std::cout << "Interval range: " << min_interval_us/1000.0 << " - " << max_interval_us/1000.0 << " ms" << std::endl;
                std::cout << "Total frames:   " << frames.size() << std::endl;
                std::cout << "Test duration:  " << test_duration.count() << " ms" << std::endl;
                
                // Assessment
                double fps_accuracy = (actual_fps / target_fps) * 100.0;
                std::cout << "\n========== ASSESSMENT ==========" << std::endl;
                std::cout << "FPS Accuracy:   " << fps_accuracy << "%" << std::endl;
                
                if (fps_accuracy >= 95.0) {
                    std::cout << "EXCELLENT: Target frame rate achieved" << std::endl;
                } else if (fps_accuracy >= 80.0) {
                    std::cout << "GOOD: Close to target frame rate" << std::endl;
                } else if (fps_accuracy >= 60.0) {
                    std::cout << "FAIR: Significantly below target" << std::endl;
                } else {
                    std::cout << "POOR: Cannot sustain target frame rate" << std::endl;
                }
                
                // Check frame rate stability
                double interval_variance = 0.0;
                for (uint64_t interval : intervals) {
                    interval_variance += std::pow(interval - mean_interval_us, 2);
                }
                interval_variance /= intervals.size();
                double interval_stddev = std::sqrt(interval_variance);
                double stability = 100.0 * (1.0 - (interval_stddev / mean_interval_us));
                
                std::cout << "Frame stability: " << stability << "%" << std::endl;
                if (stability >= 95.0) {
                    std::cout << "Very stable frame timing" << std::endl;
                } else if (stability >= 85.0) {
                    std::cout << "Good frame timing stability" << std::endl;
                } else {
                    std::cout << "Variable frame timing - may affect sync" << std::endl;
                }
            }
        }
    }

    void Stop() { running = false; }

private:
    void Cleanup() {
        running = false;
        if (camera) {
            try { camera->stop(); } catch (...) {}
            try { camera->release(); } catch (...) {}
            camera.reset();
        }
        if (allocator) allocator.reset();
        if (camera_manager) {
            try { camera_manager->stop(); } catch (...) {}
            camera_manager.reset();
        }
    }
};

FrameRateTester* g_tester = nullptr;
void signal_handler(int) { if (g_tester) g_tester->Stop(); }

int main(int argc, char* argv[]) {
    std::vector<float> test_fps = {15, 30, 60, 90, 120};
    int width = 1280;
    int height = 720;
    std::string format = "YUYV";
    int num_frames = 50;
    bool verbose = false;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-w" && i + 1 < argc) width = std::atoi(argv[++i]);
        else if (arg == "-h" && i + 1 < argc) height = std::atoi(argv[++i]);
        else if (arg == "-f" && i + 1 < argc) format = argv[++i];
        else if (arg == "-n" && i + 1 < argc) num_frames = std::atoi(argv[++i]);
        else if (arg == "--fps" && i + 1 < argc) {
            test_fps.clear();
            test_fps.push_back(std::atof(argv[++i]));
        }
        else if (arg == "-v") verbose = true;
        else if (arg == "--help") {
            std::cout << "Pi Camera V3 Frame Rate Test\n";
            std::cout << "Usage: " << argv[0] << " [options]\n\n";
            std::cout << "Options:\n";
            std::cout << "  -w <width>     Frame width (default: 1280)\n";
            std::cout << "  -h <height>    Frame height (default: 720)\n";
            std::cout << "  -f <format>    Format: YUYV, RGB888 (default: YUYV)\n";
            std::cout << "  -n <frames>    Frames per test (default: 50)\n";
            std::cout << "  --fps <rate>   Test specific FPS (default: test multiple)\n";
            std::cout << "  -v             Verbose output\n";
            std::cout << "  --help         Show help\n\n";
            std::cout << "Tests whether Pi Camera V3 can achieve target frame rates\n";
            return 0;
        }
    }

    signal(SIGINT, signal_handler);

    std::cout << "Pi Camera V3 Frame Rate Test" << std::endl;
    std::cout << "============================" << std::endl;
    std::cout << "Resolution: " << width << "x" << height << std::endl;
    std::cout << "Format: " << format << std::endl;
    std::cout << "Frames per test: " << num_frames << std::endl;

    for (float fps : test_fps) {
        FrameRateTester tester;
        g_tester = &tester;

        if (!tester.Initialize(width, height, format, fps)) {
            std::cerr << "Failed to initialize camera for " << fps << " fps test" << std::endl;
            continue;
        }

        tester.TestFrameRate(num_frames, verbose);
        
        std::cout << "\nPress Enter to continue to next test (or Ctrl+C to exit)...";
        std::cin.get();
    }

    std::cout << "\nFrame rate testing completed!" << std::endl;
    return 0;
}