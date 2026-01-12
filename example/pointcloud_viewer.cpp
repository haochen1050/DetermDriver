#include "sensor/determ_sensor.h"
#include <opencv2/opencv.hpp>
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <cmath>

using namespace determ::capture;
using namespace cv;
using json = nlohmann::json;

/**
 * @brief Thread-safe point cloud data structure
 */
struct PointCloud {
    std::vector<float> vertices;
    std::vector<uint8_t> colors;
    mutable std::mutex data_mutex;
    std::atomic<size_t> point_count{0};
    
    void clear() {
        std::lock_guard<std::mutex> lock(data_mutex);
        vertices.clear();
        colors.clear();
        point_count.store(0);
    }
    
    void setData(std::vector<float>&& new_vertices, std::vector<uint8_t>&& new_colors) {
        std::lock_guard<std::mutex> lock(data_mutex);
        vertices = std::move(new_vertices);
        colors = std::move(new_colors);
        point_count.store(vertices.size() / 3);
    }
    
    void getData(std::vector<float>& out_vertices, std::vector<uint8_t>& out_colors) const {
        std::lock_guard<std::mutex> lock(data_mutex);
        out_vertices = vertices;
        out_colors = colors;
    }
};

/**
 * @brief OpenGL Point Cloud Visualizer
 */
class PointCloudVisualizer {
private:
    static std::atomic<PointCloudVisualizer*> instance_;
    static std::mutex instance_mutex_;
    
    DetermSensor* sensor_;
    PointCloud point_cloud_;
    
    // Camera parameters
    std::atomic<float> camera_distance_{1.5f};
    std::atomic<float> camera_angle_x_{20.0f};
    std::atomic<float> camera_angle_y_{0.0f};
    std::atomic<float> camera_target_x_{0.0f};
    std::atomic<float> camera_target_y_{0.0f};
    std::atomic<float> camera_target_z_{0.8f};
    
    // Mouse interaction
    std::mutex mouse_mutex_;
    int last_mouse_x_ = 0;
    int last_mouse_y_ = 0;
    bool mouse_left_pressed_ = false;
    bool mouse_right_pressed_ = false;
    
    // Visualization parameters
    std::atomic<int> downsample_factor_{2};
    std::atomic<float> max_depth_{2.5f};
    std::atomic<float> min_depth_{0.3f};
    std::atomic<int> point_size_{2};
    std::atomic<bool> show_axes_{true};
    std::atomic<bool> auto_rotate_{false};
    
    // Calibration parameters (from config)
    rs2_intrinsics depth_intrinsics_;
    rs2_intrinsics color_intrinsics_;
    rs2_extrinsics depth_to_color_;
    std::atomic<bool> has_calibration_{false};
    
    // Statistics
    std::atomic<int> fps_counter_{0};
    std::chrono::steady_clock::time_point last_fps_time_;
    std::atomic<float> current_fps_{0.0f};
    
    std::atomic<bool> opengl_initialized_{false};
    
public:
    std::atomic<bool> should_exit_{false};
    
    PointCloudVisualizer(DetermSensor* sensor) : sensor_(sensor) {
        last_fps_time_ = std::chrono::steady_clock::now();
        
        // Load calibration from sensor
        if (sensor_->HasCalibration()) {
            depth_intrinsics_ = sensor_->GetDepthIntrinsics();
            color_intrinsics_ = sensor_->GetColorIntrinsics();
            depth_to_color_ = sensor_->GetDepthToColorExtrinsics();
            has_calibration_.store(true);
            std::cout << "Calibration loaded from sensor" << std::endl;
        } else {
            std::cerr << "Warning: No calibration available" << std::endl;
        }
        
        {
            std::lock_guard<std::mutex> lock(instance_mutex_);
            instance_.store(this);
        }
    }
    
    ~PointCloudVisualizer() {
        should_exit_.store(true);
        {
            std::lock_guard<std::mutex> lock(instance_mutex_);
            if (instance_.load() == this) {
                instance_.store(nullptr);
            }
        }
    }
    
    void processFrames(const SynchronizedFrameSet& frame_set) {
        if (should_exit_.load()) return;
        
        const auto* depth_frame = frame_set.getFrame("depth");
        const auto* color_frame = frame_set.getFrame("color");
        
        if (!depth_frame || !color_frame) {
            return;
        }
        
        try {
            // Validate frame dimensions
            if (depth_frame->width == 0 || depth_frame->height == 0 ||
                color_frame->width == 0 || color_frame->height == 0) {
                return;
            }
            
            // Convert color frame to RGB using OpenCV
            Mat color_img;
            if (color_frame->format == "YUYV") {
                Mat yuyv_img(color_frame->height, color_frame->width, CV_8UC2,
                            const_cast<uint8_t*>(color_frame->data.data()));
                Mat bgr_img;
                cv::cvtColor(yuyv_img, bgr_img, cv::COLOR_YUV2BGR_YUYV);
                cv::cvtColor(bgr_img, color_img, cv::COLOR_BGR2RGB);
            } else if (color_frame->format == "RGB888" || color_frame->format == "RGB") {
                color_img = Mat(color_frame->height, color_frame->width, CV_8UC3,
                               const_cast<uint8_t*>(color_frame->data.data())).clone();
            } else if (color_frame->format == "BGR888" || color_frame->format == "BGR") {
                Mat bgr_img(color_frame->height, color_frame->width, CV_8UC3,
                           const_cast<uint8_t*>(color_frame->data.data()));
                cv::cvtColor(bgr_img, color_img, cv::COLOR_BGR2RGB);
            } else {
                color_img = Mat(color_frame->height, color_frame->width, CV_8UC3, Scalar(128, 128, 128));
            }
            
            // Create depth Mat
            Mat depth_img(depth_frame->height, depth_frame->width, CV_16UC1,
                         const_cast<uint16_t*>(reinterpret_cast<const uint16_t*>(depth_frame->data.data())));
            
            generatePointCloud(depth_img, color_img);
            updateFPS();
            
        } catch (const std::exception& e) {
            std::cerr << "Error processing frames: " << e.what() << std::endl;
        }
    }
    
    void generatePointCloud(const Mat& depth_img, const Mat& color_img) {
        if (should_exit_.load() || !has_calibration_.load()) {
            return;
        }
        
        std::vector<float> new_vertices;
        std::vector<uint8_t> new_colors;
        
        // Estimate capacity
        int estimated_points = (depth_img.rows / downsample_factor_.load()) *
                              (depth_img.cols / downsample_factor_.load()) / 4;
        new_vertices.reserve(estimated_points * 3);
        new_colors.reserve(estimated_points * 3);
        
        // Get parameters
        int downsample = downsample_factor_.load();
        float max_depth = max_depth_.load();
        float min_depth = min_depth_.load();
        
        // Process depth image
        for (int v = 0; v < depth_img.rows; v += downsample) {
            for (int u = 0; u < depth_img.cols; u += downsample) {
                uint16_t depth_raw = depth_img.at<uint16_t>(v, u);
                
                if (depth_raw == 0) continue;
                
                // Convert depth from mm to meters
                float depth_m = depth_raw / 1000.0f;
                
                if (depth_m < min_depth || depth_m > max_depth) continue;
                
                // Deproject depth pixel to 3D point
                float x_depth = (u - depth_intrinsics_.ppx) * depth_m / depth_intrinsics_.fx;
                float y_depth = (v - depth_intrinsics_.ppy) * depth_m / depth_intrinsics_.fy;
                float z_depth = depth_m;
                
                // Transform to color camera coordinate system
                float x_color = depth_to_color_.rotation[0] * x_depth +
                              depth_to_color_.rotation[1] * y_depth +
                              depth_to_color_.rotation[2] * z_depth +
                              depth_to_color_.translation[0];
                float y_color = depth_to_color_.rotation[3] * x_depth +
                              depth_to_color_.rotation[4] * y_depth +
                              depth_to_color_.rotation[5] * z_depth +
                              depth_to_color_.translation[1];
                float z_color = depth_to_color_.rotation[6] * x_depth +
                              depth_to_color_.rotation[7] * y_depth +
                              depth_to_color_.rotation[8] * z_depth +
                              depth_to_color_.translation[2];
                
                Vec3b color(128, 128, 128);
                
                if (z_color > 0.1f) {
                    // Project to color image
                    float u_color = color_intrinsics_.fx * x_color / z_color + color_intrinsics_.ppx;
                    float v_color = color_intrinsics_.fy * y_color / z_color + color_intrinsics_.ppy;
                    
                    int u_int = static_cast<int>(u_color + 0.5f);
                    int v_int = static_cast<int>(v_color + 0.5f);
                    
                    if (u_int >= 0 && u_int < color_img.cols &&
                        v_int >= 0 && v_int < color_img.rows) {
                        color = color_img.at<Vec3b>(v_int, u_int);
                    }
                    
                    // Add point (use color camera coordinates)
                    new_vertices.push_back(x_color);
                    new_vertices.push_back(-y_color);  // Flip Y for OpenGL
                    new_vertices.push_back(z_color);
                    new_colors.push_back(color[0]);  // R
                    new_colors.push_back(color[1]);  // G
                    new_colors.push_back(color[2]);  // B
                }
            }
        }
        
        point_cloud_.setData(std::move(new_vertices), std::move(new_colors));
    }
    
    void updateFPS() {
        fps_counter_.fetch_add(1);
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_fps_time_);
        
        if (duration.count() > 1000) {
            current_fps_.store(fps_counter_.load() * 1000.0f / duration.count());
            fps_counter_.store(0);
            last_fps_time_ = now;
        }
    }
    
    void initOpenGL(int argc, char** argv) {
        glutInit(&argc, argv);
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
        glutInitWindowSize(1280, 800);
        glutCreateWindow("DetermSensor Point Cloud Viewer - OpenGL");
        
        glutDisplayFunc(displayCallback);
        glutReshapeFunc(reshapeCallback);
        glutKeyboardFunc(keyboardCallback);
        glutMouseFunc(mouseCallback);
        glutMotionFunc(motionCallback);
        glutTimerFunc(16, timerCallback, 0);
        
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        
        opengl_initialized_.store(true);
        
        printControls();
    }
    
    void printControls() {
        std::cout << "\n=== DetermSensor Point Cloud Viewer - OpenGL ===" << std::endl;
        std::cout << "Controls:" << std::endl;
        std::cout << "  Mouse Left Drag: Rotate view" << std::endl;
        std::cout << "  Mouse Right Drag: Pan view" << std::endl;
        std::cout << "  Mouse Wheel: Zoom in/out" << std::endl;
        std::cout << "  A: Toggle coordinate axes" << std::endl;
        std::cout << "  R: Toggle auto-rotate" << std::endl;
        std::cout << "  +/-: Adjust point density" << std::endl;
        std::cout << "  [/]: Adjust point size" << std::endl;
        std::cout << "  ,/.: Adjust depth range" << std::endl;
        std::cout << "  Space: Reset view" << std::endl;
        std::cout << "  Q/ESC: Quit" << std::endl;
        std::cout << "===============================================" << std::endl;
    }
    
    // Static callbacks
    static void displayCallback() {
        auto* inst = instance_.load();
        if (inst && inst->opengl_initialized_.load() && !inst->should_exit_.load()) {
            inst->render();
        }
    }
    
    static void reshapeCallback(int w, int h) {
        auto* inst = instance_.load();
        if (inst && inst->opengl_initialized_.load() && !inst->should_exit_.load()) {
            inst->reshape(w, h);
        }
    }
    
    static void keyboardCallback(unsigned char key, int x, int y) {
        auto* inst = instance_.load();
        if (inst && inst->opengl_initialized_.load() && !inst->should_exit_.load()) {
            inst->keyboard(key, x, y);
        }
    }
    
    static void mouseCallback(int button, int state, int x, int y) {
        auto* inst = instance_.load();
        if (inst && inst->opengl_initialized_.load() && !inst->should_exit_.load()) {
            inst->mouse(button, state, x, y);
        }
    }
    
    static void motionCallback(int x, int y) {
        auto* inst = instance_.load();
        if (inst && inst->opengl_initialized_.load() && !inst->should_exit_.load()) {
            inst->motion(x, y);
        }
    }
    
    static void timerCallback(int value) {
        (void)value;
        auto* inst = instance_.load();
        if (inst && inst->opengl_initialized_.load() && !inst->should_exit_.load()) {
            glutPostRedisplay();
            glutTimerFunc(16, timerCallback, 0);
        }
    }
    
    void render() {
        if (should_exit_.load() || !opengl_initialized_.load()) return;
        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();
        
        // Auto-rotate
        if (auto_rotate_.load()) {
            float new_angle = camera_angle_y_.load() + 0.3f;
            if (new_angle > 360.0f) new_angle -= 360.0f;
            camera_angle_y_.store(new_angle);
        }
        
        // Setup camera
        float dist = camera_distance_.load();
        float angle_x = camera_angle_x_.load();
        float angle_y = camera_angle_y_.load();
        float target_x = camera_target_x_.load();
        float target_y = camera_target_y_.load();
        float target_z = camera_target_z_.load();
        
        gluLookAt(
            target_x + dist * sin(angle_y * M_PI / 180.0f) * cos(angle_x * M_PI / 180.0f),
            target_y + dist * sin(angle_x * M_PI / 180.0f),
            target_z + dist * cos(angle_y * M_PI / 180.0f) * cos(angle_x * M_PI / 180.0f),
            target_x, target_y, target_z,
            0.0f, 1.0f, 0.0f
        );
        
        // Draw axes
        if (show_axes_.load()) {
            glLineWidth(3.0f);
            glBegin(GL_LINES);
            // X - Red
            glColor3f(1.0f, 0.0f, 0.0f);
            glVertex3f(0.0f, 0.0f, 0.0f);
            glVertex3f(0.3f, 0.0f, 0.0f);
            // Y - Green
            glColor3f(0.0f, 1.0f, 0.0f);
            glVertex3f(0.0f, 0.0f, 0.0f);
            glVertex3f(0.0f, 0.3f, 0.0f);
            // Z - Blue
            glColor3f(0.0f, 0.0f, 1.0f);
            glVertex3f(0.0f, 0.0f, 0.0f);
            glVertex3f(0.0f, 0.0f, 0.3f);
            glEnd();
        }
        
        // Draw point cloud
        std::vector<float> vertices;
        std::vector<uint8_t> colors;
        point_cloud_.getData(vertices, colors);
        
        if (!vertices.empty() && !colors.empty()) {
            glPointSize(static_cast<GLfloat>(point_size_.load()));
            glBegin(GL_POINTS);
            
            size_t point_count = std::min(vertices.size() / 3, colors.size() / 3);
            for (size_t i = 0; i < point_count; ++i) {
                glColor3ub(colors[i*3], colors[i*3+1], colors[i*3+2]);
                glVertex3f(vertices[i*3], vertices[i*3+1], vertices[i*3+2]);
            }
            
            glEnd();
        }
        
        drawStats();
        glutSwapBuffers();
    }
    
    void drawStats() {
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT), 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        
        glDisable(GL_DEPTH_TEST);
        
        // Background
        glColor4f(0.0f, 0.0f, 0.0f, 0.75f);
        glBegin(GL_QUADS);
        glVertex2f(10, 10);
        glVertex2f(350, 10);
        glVertex2f(350, 130);
        glVertex2f(10, 130);
        glEnd();
        
        glColor3f(1.0f, 1.0f, 1.0f);
        
        char text[256];
        snprintf(text, sizeof(text), "FPS: %.1f", current_fps_.load());
        renderText(20, 30, text);
        
        snprintf(text, sizeof(text), "Points: %zu (DS: %dx)", 
                point_cloud_.point_count.load(), downsample_factor_.load());
        renderText(20, 50, text);
        
        snprintf(text, sizeof(text), "Point Size: %dpx", point_size_.load());
        renderText(20, 70, text);
        
        snprintf(text, sizeof(text), "Depth: %.1f - %.1fm", min_depth_.load(), max_depth_.load());
        renderText(20, 90, text);
        
        renderText(20, 110, has_calibration_.load() ? "Calibrated" : "No Calibration");
        
        glEnable(GL_DEPTH_TEST);
        
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
    }
    
    void renderText(float x, float y, const char* text) {
        glRasterPos2f(x, y);
        for (const char* c = text; *c != '\0'; c++) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
        }
    }
    
    void reshape(int width, int height) {
        if (height == 0) height = 1;
        glViewport(0, 0, width, height);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(60.0, static_cast<double>(width) / height, 0.01, 100.0);
        glMatrixMode(GL_MODELVIEW);
    }
    
    void keyboard(unsigned char key, int x, int y) {
        (void)x; (void)y;
        
        switch (key) {
            case 'q': case 'Q': case 27:
                should_exit_.store(true);
                exit(0);
                break;
            case 'a': case 'A':
                show_axes_.store(!show_axes_.load());
                break;
            case 'r': case 'R':
                auto_rotate_.store(!auto_rotate_.load());
                break;
            case '+': case '=':
                if (downsample_factor_.load() > 1) {
                    downsample_factor_.fetch_sub(1);
                }
                break;
            case '-': case '_':
                if (downsample_factor_.load() < 8) {
                    downsample_factor_.fetch_add(1);
                }
                break;
            case '[':
                if (point_size_.load() > 1) {
                    point_size_.fetch_sub(1);
                }
                break;
            case ']':
                if (point_size_.load() < 10) {
                    point_size_.fetch_add(1);
                }
                break;
            case ',':
                max_depth_.store(max_depth_.load() - 0.1f);
                if (max_depth_.load() < min_depth_.load()) {
                    max_depth_.store(min_depth_.load() + 0.1f);
                }
                break;
            case '.':
                max_depth_.store(max_depth_.load() + 0.1f);
                break;
            case ' ':
                camera_distance_.store(1.5f);
                camera_angle_x_.store(20.0f);
                camera_angle_y_.store(0.0f);
                break;
        }
    }
    
    void mouse(int button, int state, int x, int y) {
        std::lock_guard<std::mutex> lock(mouse_mutex_);
        
        if (button == GLUT_LEFT_BUTTON) {
            mouse_left_pressed_ = (state == GLUT_DOWN);
        } else if (button == GLUT_RIGHT_BUTTON) {
            mouse_right_pressed_ = (state == GLUT_DOWN);
        } else if (button == 3) {  // Wheel up
            camera_distance_.store(std::max(0.1f, camera_distance_.load() * 0.9f));
        } else if (button == 4) {  // Wheel down
            camera_distance_.store(std::min(10.0f, camera_distance_.load() * 1.1f));
        }
        
        last_mouse_x_ = x;
        last_mouse_y_ = y;
    }
    
    void motion(int x, int y) {
        std::lock_guard<std::mutex> lock(mouse_mutex_);
        
        if (mouse_left_pressed_) {
            float dx = x - last_mouse_x_;
            float dy = y - last_mouse_y_;
            
            float new_angle_y = camera_angle_y_.load() + dx * 0.5f;
            float new_angle_x = camera_angle_x_.load() + dy * 0.5f;
            
            new_angle_x = std::clamp(new_angle_x, -89.0f, 89.0f);
            
            camera_angle_y_.store(new_angle_y);
            camera_angle_x_.store(new_angle_x);
        } else if (mouse_right_pressed_) {
            float dx = (x - last_mouse_x_) * 0.01f;
            float dy = (y - last_mouse_y_) * 0.01f;
            
            camera_target_x_.store(camera_target_x_.load() + dx);
            camera_target_y_.store(camera_target_y_.load() - dy);
        }
        
        last_mouse_x_ = x;
        last_mouse_y_ = y;
    }
};

// Static member initialization
std::atomic<PointCloudVisualizer*> PointCloudVisualizer::instance_{nullptr};
std::mutex PointCloudVisualizer::instance_mutex_;

int main(int argc, char** argv) {
    std::cout << "=== DetermSensor Point Cloud Viewer - OpenGL ===" << std::endl;
    
    try {
        // Initialize sensor
        DetermSensor sensor;
        
        if (sensor.Initialize() != 0) {
            std::cerr << "Failed to initialize sensor" << std::endl;
            return -1;
        }
        
        // Enable streams
        std::vector<std::string> streams = {"color", "depth"};
        if (sensor.EnableStream(streams) != 0) {
            std::cerr << "Failed to enable streams" << std::endl;
            return -1;
        }
        
        // Configure sensors
        if (sensor.SetColorSensorConfig(640, 480, "YUYV") != 0 ||
            sensor.SetDepthSensorConfig(640, 480) != 0) {
            std::cerr << "Failed to configure sensors" << std::endl;
            return -1;
        }
        
        // Load calibration
        std::string config_path = "../config/determ_sensor_config.json";
        if (!sensor.LoadCalibration(config_path)) {
            std::cerr << "Warning: Failed to load calibration" << std::endl;
        }
        
        // Start sensor
        if (sensor.Start() != 0) {
            std::cerr << "Failed to start sensor" << std::endl;
            return -1;
        }
        
        // Start synchronized capture
        sensor.SetSyncTolerance(20.0);
        if (!sensor.StartSynchronizedCapture()) {
            std::cerr << "Failed to start synchronized capture" << std::endl;
            return -1;
        }
        
        // Initialize visualizer
        auto visualizer = std::make_unique<PointCloudVisualizer>(&sensor);
        visualizer->initOpenGL(argc, argv);
        
        // Start processing thread
        std::atomic<bool> running{true};
        std::thread processing_thread([&]() {
            SynchronizedFrameSet frame_set;
            while (running.load() && !visualizer->should_exit_.load()) {
                if (sensor.GetLatestSynchronizedFrames(frame_set)) {
                    visualizer->processFrames(frame_set);
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
        });
        
        std::cout << "Starting visualization..." << std::endl;
        
        // Start OpenGL main loop
        glutMainLoop();
        
        // Cleanup
        std::cout << "Shutting down..." << std::endl;
        running.store(false);
        if (processing_thread.joinable()) {
            processing_thread.join();
        }
        
        sensor.StopSynchronizedCapture();
        sensor.Stop();
        
        std::cout << "Shutdown complete" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}