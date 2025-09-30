#include "sensor/determ_sensor.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <memory>

using namespace determ::sensor;
using namespace cv;
using json = nlohmann::json;

/**
 * @brief Thread-safe calibration data loader
 */
struct CalibrationData {
    // Color camera (Raspberry Pi) parameters
    Mat color_camera_matrix;
    Mat color_distortion;
    
    // Depth camera (RealSense) parameters
    Mat depth_camera_matrix;
    Mat depth_distortion;
    
    // Transformation from depth to color coordinate system
    Mat rotation_matrix;
    Mat translation_vector;
    
    // Cached parameters for fast computation (thread-safe)
    std::atomic<float> color_fx{0}, color_fy{0}, color_cx{0}, color_cy{0};
    std::atomic<float> depth_fx{0}, depth_fy{0}, depth_cx{0}, depth_cy{0};
    std::atomic<float> tx{0}, ty{0}, tz{0}; // translation in meters
    
    // Rotation matrix elements for fast access
    float R[9]; // 3x3 rotation matrix flattened
    std::mutex R_mutex; // Protect rotation matrix access
    
    std::atomic<bool> is_loaded{false};
    std::string source = "None";
    std::mutex source_mutex;
    
    bool loadFromJSON(const std::string& json_path) {
        try {
            std::ifstream file(json_path);
            if (!file.is_open()) {
                std::cerr << "Cannot open calibration file: " << json_path << std::endl;
                return false;
            }
            
            json cal_data;
            file >> cal_data;
            
            std::cout << "Loading calibration from: " << json_path << std::endl;
            
            // Validate JSON structure
            if (!cal_data.contains("color_camera_matrix") || 
                !cal_data.contains("depth_camera_intrinsics") ||
                !cal_data.contains("rotation_matrix") ||
                !cal_data.contains("translation_vector")) {
                std::cerr << "Invalid calibration file format" << std::endl;
                return false;
            }
            
            // Load color camera parameters with validation
            auto color_matrix = cal_data["color_camera_matrix"];
            if (color_matrix.size() != 3 || color_matrix[0].size() != 3) {
                std::cerr << "Invalid color camera matrix format" << std::endl;
                return false;
            }
            
            color_camera_matrix = (Mat_<double>(3, 3) << 
                color_matrix[0][0], color_matrix[0][1], color_matrix[0][2],
                color_matrix[1][0], color_matrix[1][1], color_matrix[1][2],
                color_matrix[2][0], color_matrix[2][1], color_matrix[2][2]);
            
            auto color_dist = cal_data["color_distortion"][0];
            if (color_dist.size() != 5) {
                std::cerr << "Invalid color distortion format" << std::endl;
                return false;
            }
            
            color_distortion = (Mat_<double>(1, 5) << 
                color_dist[0], color_dist[1], color_dist[2], color_dist[3], color_dist[4]);
            
            // Load depth camera parameters
            auto depth_intrinsics = cal_data["depth_camera_intrinsics"];
            depth_camera_matrix = (Mat_<double>(3, 3) << 
                depth_intrinsics["fx"], 0.0, depth_intrinsics["cx"],
                0.0, depth_intrinsics["fy"], depth_intrinsics["cy"],
                0.0, 0.0, 1.0);
            
            depth_distortion = Mat::zeros(1, 5, CV_64F);
            
            // Load transformation matrices with validation
            auto rot_matrix = cal_data["rotation_matrix"];
            if (rot_matrix.size() != 3 || rot_matrix[0].size() != 3) {
                std::cerr << "Invalid rotation matrix format" << std::endl;
                return false;
            }
            
            rotation_matrix = (Mat_<double>(3, 3) << 
                rot_matrix[0][0], rot_matrix[0][1], rot_matrix[0][2],
                rot_matrix[1][0], rot_matrix[1][1], rot_matrix[1][2],
                rot_matrix[2][0], rot_matrix[2][1], rot_matrix[2][2]);
            
            auto trans_vector = cal_data["translation_vector"];
            if (trans_vector.size() != 3 || trans_vector[0].size() != 1) {
                std::cerr << "Invalid translation vector format" << std::endl;
                return false;
            }
            
            translation_vector = (Mat_<double>(3, 1) << 
                trans_vector[0][0], trans_vector[1][0], trans_vector[2][0]);
            
            // Cache parameters for fast access
            cacheParameters();
            
            {
                std::lock_guard<std::mutex> lock(source_mutex);
                source = "JSON: " + json_path;
            }
            is_loaded.store(true);
            
            std::cout << "Calibration loaded successfully!" << std::endl;
            std::cout << "Color camera fx: " << color_fx.load() << ", fy: " << color_fy.load() << std::endl;
            std::cout << "Depth camera fx: " << depth_fx.load() << ", fy: " << depth_fy.load() << std::endl;
            std::cout << "Translation: " << tx.load() << ", " << ty.load() << ", " << tz.load() << std::endl;
            
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "Error loading calibration: " << e.what() << std::endl;
            return false;
        }
    }
    
    void cacheParameters() {
        // Validate matrix types and sizes before accessing
        if (color_camera_matrix.type() != CV_64F || color_camera_matrix.rows != 3 || color_camera_matrix.cols != 3) {
            std::cerr << "Invalid color camera matrix format for caching" << std::endl;
            return;
        }
        
        if (depth_camera_matrix.type() != CV_64F || depth_camera_matrix.rows != 3 || depth_camera_matrix.cols != 3) {
            std::cerr << "Invalid depth camera matrix format for caching" << std::endl;
            return;
        }
        
        if (rotation_matrix.type() != CV_64F || rotation_matrix.rows != 3 || rotation_matrix.cols != 3) {
            std::cerr << "Invalid rotation matrix format for caching" << std::endl;
            return;
        }
        
        if (translation_vector.type() != CV_64F || translation_vector.rows != 3 || translation_vector.cols != 1) {
            std::cerr << "Invalid translation vector format for caching" << std::endl;
            return;
        }
        
        // Color camera parameters
        color_fx.store(static_cast<float>(color_camera_matrix.at<double>(0, 0)));
        color_fy.store(static_cast<float>(color_camera_matrix.at<double>(1, 1)));
        color_cx.store(static_cast<float>(color_camera_matrix.at<double>(0, 2)));
        color_cy.store(static_cast<float>(color_camera_matrix.at<double>(1, 2)));
        
        // Depth camera parameters
        depth_fx.store(static_cast<float>(depth_camera_matrix.at<double>(0, 0)));
        depth_fy.store(static_cast<float>(depth_camera_matrix.at<double>(1, 1)));
        depth_cx.store(static_cast<float>(depth_camera_matrix.at<double>(0, 2)));
        depth_cy.store(static_cast<float>(depth_camera_matrix.at<double>(1, 2)));
        
        // Translation
        tx.store(static_cast<float>(translation_vector.at<double>(0)));
        ty.store(static_cast<float>(translation_vector.at<double>(1)));
        tz.store(static_cast<float>(translation_vector.at<double>(2)));
        
        // Rotation matrix elements for fast access (thread-safe)
        {
            std::lock_guard<std::mutex> lock(R_mutex);
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    R[i*3 + j] = static_cast<float>(rotation_matrix.at<double>(i, j));
                }
            }
        }
    }
    
    bool loadDefaultCalibration() {
        std::cout << "Loading default calibration..." << std::endl;
        
        try {
            // Your provided calibration values
            color_camera_matrix = (Mat_<double>(3, 3) << 
                541.15412629, 0.0, 322.76223738,
                0.0, 541.18394017, 245.13495424,
                0.0, 0.0, 1.0);
            
            color_distortion = (Mat_<double>(1, 5) << 
                -5.94205850e-02, 7.24135370e-02, 5.12143246e-04, -1.89740685e-05, 7.05105297e-02);
            
            depth_camera_matrix = (Mat_<double>(3, 3) << 
                461.32928467, 0.0, 324.59243774,
                0.0, 461.32928467, 242.13339233,
                0.0, 0.0, 1.0);
            
            depth_distortion = Mat::zeros(1, 5, CV_64F);
            
            rotation_matrix = (Mat_<double>(3, 3) << 
                0.99998518, -0.00526811, 0.00137678,
                0.00518194, 0.99838997, 0.05648548,
                -0.00167213, -0.05647751, 0.99840247);
            
            translation_vector = (Mat_<double>(3, 1) << 
                0.0374266, 0.00087347, 0.00149728);
            
            cacheParameters();
            {
                std::lock_guard<std::mutex> lock(source_mutex);
                source = "Default (embedded)";
            }
            is_loaded.store(true);
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "Error loading default calibration: " << e.what() << std::endl;
            return false;
        }
    }
    
    // Thread-safe getters
    std::string getSource() const {
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(source_mutex));
        return source;
    }
    
    void getRotationMatrix(float R_copy[9]) const {
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(R_mutex));
        std::memcpy(R_copy, R, sizeof(float) * 9);
    }
};

/**
 * @brief Thread-safe point cloud structure
 */
struct PointCloud {
    std::vector<float> vertices;
    std::vector<uint8_t> colors;
    mutable std::mutex data_mutex;
    std::atomic<int> point_count{0};
    
    void clear() {
        std::lock_guard<std::mutex> lock(data_mutex);
        vertices.clear();
        colors.clear();
        point_count.store(0);
    }
    
    void reserve(size_t estimated_points) {
        std::lock_guard<std::mutex> lock(data_mutex);
        vertices.reserve(estimated_points * 3);
        colors.reserve(estimated_points * 3);
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
    
    size_t getPointCount() const {
        return point_count.load();
    }
};

/**
 * @brief Thread-safe point cloud visualizer using OpenGL
 */
class PointCloudVisualizer {
private:
    static std::atomic<PointCloudVisualizer*> instance_;
    static std::mutex instance_mutex_;
    
    CalibrationData calibration_;
    PointCloud point_cloud_;
    DetermSensor* sensor_;
    
    // Visualization parameters (thread-safe)
    std::atomic<float> camera_distance_{1.5f};
    std::atomic<float> camera_angle_x_{20.0f};
    std::atomic<float> camera_angle_y_{0.0f};
    std::atomic<float> camera_target_x_{0.0f};
    std::atomic<float> camera_target_y_{0.0f}; 
    std::atomic<float> camera_target_z_{0.8f};
    
    // Mouse interaction (protected by mutex)
    mutable std::mutex mouse_mutex_;
    int last_mouse_x_ = 0;
    int last_mouse_y_ = 0;
    bool mouse_pressed_ = false;
    bool right_mouse_pressed_ = false;
    
    // Processing parameters (atomic for thread safety)
    std::atomic<int> downsample_factor_{2};
    std::atomic<float> max_depth_{2.0f};
    std::atomic<float> min_depth_{0.2f};
    std::atomic<int> point_size_{2};
    std::atomic<bool> use_calibration_{true};
    std::atomic<bool> show_axes_{true};
    std::atomic<bool> auto_rotate_{false};
    
    // Statistics (thread-safe)
    std::atomic<int> fps_counter_{0};
    std::chrono::high_resolution_clock::time_point last_fps_time_;
    std::atomic<float> current_fps_{0.0f};
    
    // OpenGL state
    std::atomic<bool> opengl_initialized_{false};
    
public:
    std::atomic<bool> should_exit_{false}; // Made public for thread access;
    
public:
    PointCloudVisualizer(DetermSensor* sensor) : sensor_(sensor) {
        last_fps_time_ = std::chrono::high_resolution_clock::now();
        
        // Try to load calibration
        std::string config_path = "../config/rgb_to_depth_calibration.json";
        if (std::filesystem::exists(config_path)) {
            if (!calibration_.loadFromJSON(config_path)) {
                calibration_.loadDefaultCalibration();
            }
        } else {
            std::cout << "Config file not found: " << config_path << std::endl;
            calibration_.loadDefaultCalibration();
        }
        
        std::cout << "Using calibration: " << calibration_.getSource() << std::endl;
        
        // Set instance at the very end of construction
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
            // Convert frame data to OpenCV matrices with validation
            if (depth_frame->width == 0 || depth_frame->height == 0 || 
                color_frame->width == 0 || color_frame->height == 0) {
                std::cerr << "Invalid frame dimensions" << std::endl;
                return;
            }
            
            if (depth_frame->data.empty() || color_frame->data.empty()) {
                std::cerr << "Empty frame data" << std::endl;
                return;
            }
            
            Mat depth_img(depth_frame->height, depth_frame->width, CV_16UC1, 
                         const_cast<void*>(reinterpret_cast<const void*>(depth_frame->data.data())));
            
            Mat color_img(color_frame->height, color_frame->width, CV_8UC3,
                         const_cast<void*>(reinterpret_cast<const void*>(color_frame->data.data())));
            
            // Validate matrices
            if (depth_img.empty() || color_img.empty()) {
                std::cerr << "Failed to create OpenCV matrices from frame data" << std::endl;
                return;
            }
            
            generatePointCloud(depth_img, color_img);
            updateFPS();
            
        } catch (const std::exception& e) {
            std::cerr << "Error processing frames: " << e.what() << std::endl;
        }
    }
    
    void generatePointCloud(const Mat& depth_img, const Mat& color_img) {
        if (should_exit_.load() || !calibration_.is_loaded.load()) {
            return;
        }
        
        // Work with local vectors to avoid locking during processing
        std::vector<float> new_vertices;
        std::vector<uint8_t> new_colors;
        
        // Estimate capacity to avoid reallocations
        int estimated_points = (depth_img.rows / downsample_factor_.load()) * 
                              (depth_img.cols / downsample_factor_.load()) / 4; // Rough estimate
        new_vertices.reserve(estimated_points * 3);
        new_colors.reserve(estimated_points * 3);
        
        // Get atomic values once for consistency
        int downsample = downsample_factor_.load();
        float max_depth = max_depth_.load();
        float min_depth = min_depth_.load();
        bool use_calib = use_calibration_.load();
        
        // Get cached calibration parameters
        float c_fx = calibration_.color_fx.load();
        float c_fy = calibration_.color_fy.load();
        float c_cx = calibration_.color_cx.load();
        float c_cy = calibration_.color_cy.load();
        
        float d_fx = calibration_.depth_fx.load();
        float d_fy = calibration_.depth_fy.load();
        float d_cx = calibration_.depth_cx.load();
        float d_cy = calibration_.depth_cy.load();
        
        float t_x = calibration_.tx.load();
        float t_y = calibration_.ty.load();
        float t_z = calibration_.tz.load();
        
        float R_copy[9];
        calibration_.getRotationMatrix(R_copy);
        
        for (int v = 0; v < depth_img.rows; v += downsample) {
            for (int u = 0; u < depth_img.cols; u += downsample) {
                // Bounds checking
                if (v >= depth_img.rows || u >= depth_img.cols) continue;
                
                uint16_t depth_raw = depth_img.at<uint16_t>(v, u);
                
                if (depth_raw == 0) continue;
                
                // Convert depth from mm to meters
                float depth_m = depth_raw / 1000.0f;
                
                if (depth_m < min_depth || depth_m > max_depth) continue;
                
                // Generate 3D point in depth camera coordinates
                float x_depth = (u - d_cx) * depth_m / d_fx;
                float y_depth = (v - d_cy) * depth_m / d_fy;
                float z_depth = depth_m;
                
                Vec3b color(128, 128, 128); // Default gray
                
                if (use_calib) {
                    // Transform point from depth to color coordinate system
                    // P_color = R * P_depth + T
                    float x_color = R_copy[0] * x_depth + R_copy[1] * y_depth + R_copy[2] * z_depth + t_x;
                    float y_color = R_copy[3] * x_depth + R_copy[4] * y_depth + R_copy[5] * z_depth + t_y;
                    float z_color = R_copy[6] * x_depth + R_copy[7] * y_depth + R_copy[8] * z_depth + t_z;
                    
                    if (z_color > 0.1f) { // Point must be in front of color camera
                        // Project to color image coordinates
                        float u_color = c_fx * x_color / z_color + c_cx;
                        float v_color = c_fy * y_color / z_color + c_cy;
                        
                        // Sample color from color image with bounds checking
                        int u_int = static_cast<int>(u_color + 0.5f);
                        int v_int = static_cast<int>(v_color + 0.5f);
                        
                        if (u_int >= 0 && u_int < color_img.cols && v_int >= 0 && v_int < color_img.rows) {
                            color = color_img.at<Vec3b>(v_int, u_int);
                        }
                        
                        // Use color camera coordinate system for display
                        new_vertices.push_back(x_color);
                        new_vertices.push_back(-y_color);
                        new_vertices.push_back(z_color);
                        new_colors.push_back(color[2]); // BGR to RGB
                        new_colors.push_back(color[1]);
                        new_colors.push_back(color[0]);
                    }
                } else {
                    // No calibration - use depth coordinates with misaligned color
                    if (u < color_img.cols && v < color_img.rows) {
                        color = color_img.at<Vec3b>(v, u);
                    }
                    
                    new_vertices.push_back(x_depth);
                    new_vertices.push_back(-y_depth);
                    new_vertices.push_back(z_depth);
                    new_colors.push_back(color[2]); // BGR to RGB
                    new_colors.push_back(color[1]);
                    new_colors.push_back(color[0]);
                }
            }
        }
        
        // Atomically update the point cloud
        point_cloud_.setData(std::move(new_vertices), std::move(new_colors));
    }
    
    void updateFPS() {
        fps_counter_.fetch_add(1);
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_time_);
        
        if (duration.count() > 1000) {
            current_fps_.store(fps_counter_.load() * 1000.0f / duration.count());
            fps_counter_.store(0);
            last_fps_time_ = now;
        }
    }
    
    void initOpenGL(int argc, char** argv) {
        glutInit(&argc, argv);
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
        glutInitWindowSize(1200, 800);
        glutCreateWindow("DetermSensor Point Cloud Viewer - Fixed");
        
        glutDisplayFunc(displayCallback);
        glutReshapeFunc(reshapeCallback);
        glutKeyboardFunc(keyboardCallback);
        glutMouseFunc(mouseCallback);
        glutMotionFunc(motionCallback);
        glutTimerFunc(16, timerCallback, 0);
        
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
        
        opengl_initialized_.store(true);
        
        std::cout << "\n=== DetermSensor Point Cloud Viewer (FIXED) ===" << std::endl;
        std::cout << "Controls:" << std::endl;
        std::cout << "  Mouse Left: Rotate view" << std::endl;
        std::cout << "  Mouse Right: Pan view" << std::endl;
        std::cout << "  Mouse Wheel: Zoom" << std::endl;
        std::cout << "  C: Toggle calibration" << std::endl;
        std::cout << "  A: Toggle axes" << std::endl;
        std::cout << "  R: Auto-rotate" << std::endl;
        std::cout << "  +/-: Point density" << std::endl;
        std::cout << "  P: Point size" << std::endl;
        std::cout << "  Space: Reset view" << std::endl;
        std::cout << "  Q: Quit" << std::endl;
        std::cout << "===========================================" << std::endl;
    }
    
    // Static callback functions with proper null checking
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
        (void)value; // Suppress unused parameter warning
        auto* inst = instance_.load();
        if (inst && inst->opengl_initialized_.load() && !inst->should_exit_.load()) {
            glutPostRedisplay(); 
            glutTimerFunc(16, timerCallback, 0); 
        }
    }
    
    void render() {
        if (should_exit_.load() || !opengl_initialized_.load()) return;
        
        try {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glLoadIdentity();
            
            // Set up camera with thread-safe access
            if (auto_rotate_.load()) {
                float new_angle = camera_angle_y_.load() + 0.5f;
                if (new_angle > 360.0f) new_angle -= 360.0f;
                camera_angle_y_.store(new_angle);
            }
            
            float cam_dist = camera_distance_.load();
            float cam_angle_x = camera_angle_x_.load();
            float cam_angle_y = camera_angle_y_.load();
            float cam_target_x = camera_target_x_.load();
            float cam_target_y = camera_target_y_.load();
            float cam_target_z = camera_target_z_.load();
            
            gluLookAt(
                cam_target_x + cam_dist * sin(cam_angle_y * M_PI / 180.0f) * cos(cam_angle_x * M_PI / 180.0f),
                cam_target_y + cam_dist * sin(cam_angle_x * M_PI / 180.0f),
                cam_target_z + cam_dist * cos(cam_angle_y * M_PI / 180.0f) * cos(cam_angle_x * M_PI / 180.0f),
                cam_target_x, cam_target_y, cam_target_z,
                0.0f, 1.0f, 0.0f
            );
            
            // Draw coordinate axes
            if (show_axes_.load()) {
                glLineWidth(3.0f);
                glBegin(GL_LINES);
                // X axis - Red
                glColor3f(1.0f, 0.0f, 0.0f);
                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3f(0.3f, 0.0f, 0.0f);
                // Y axis - Green
                glColor3f(0.0f, 1.0f, 0.0f);
                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3f(0.0f, 0.3f, 0.0f);
                // Z axis - Blue
                glColor3f(0.0f, 0.0f, 1.0f);
                glVertex3f(0.0f, 0.0f, 0.0f);
                glVertex3f(0.0f, 0.0f, 0.3f);
                glEnd();
                glLineWidth(1.0f);
            }
            
            // Render point cloud
            std::vector<float> vertices_copy;
            std::vector<uint8_t> colors_copy;
            point_cloud_.getData(vertices_copy, colors_copy);
            
            if (!vertices_copy.empty() && !colors_copy.empty()) {
                glPointSize(static_cast<GLfloat>(point_size_.load()));
                glBegin(GL_POINTS);
                
                size_t point_count = vertices_copy.size() / 3;
                size_t color_count = colors_copy.size() / 3;
                size_t render_count = std::min(point_count, color_count);
                
                for (size_t i = 0; i < render_count; ++i) {
                    size_t vertex_idx = i * 3;
                    size_t color_idx = i * 3;
                    
                    if (vertex_idx + 2 < vertices_copy.size() && color_idx + 2 < colors_copy.size()) {
                        glColor3ub(colors_copy[color_idx], colors_copy[color_idx + 1], colors_copy[color_idx + 2]);
                        glVertex3f(vertices_copy[vertex_idx], vertices_copy[vertex_idx + 1], vertices_copy[vertex_idx + 2]);
                    }
                }
                
                glEnd();
            }
            
            // Draw stats
            drawStats();
            
            glutSwapBuffers();
            
        } catch (const std::exception& e) {
            std::cerr << "Render error: " << e.what() << std::endl;
        }
    }
    
    void drawStats() {
        try {
            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            glLoadIdentity();
            glOrtho(0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT), 0, -1, 1);
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();
            
            glDisable(GL_DEPTH_TEST);
            
            // Background
            glColor4f(0.0f, 0.0f, 0.0f, 0.7f);
            glBegin(GL_QUADS);
            glVertex2f(10, 10);
            glVertex2f(400, 10);
            glVertex2f(400, 160);
            glVertex2f(10, 160);
            glEnd();
            
            glColor3f(1.0f, 1.0f, 1.0f);
            
            // Stats text
            std::string fps_text = "FPS: " + std::to_string(static_cast<int>(current_fps_.load()));
            renderText(20, 30, fps_text.c_str());
            
            std::string points_text = "Points: " + std::to_string(point_cloud_.getPointCount()) + 
                                     " (downsample: " + std::to_string(downsample_factor_.load()) + "x)";
            renderText(20, 50, points_text.c_str());
            
            std::string point_size_text = "Point Size: " + std::to_string(point_size_.load()) + "px";
            renderText(20, 70, point_size_text.c_str());
            
            std::string calib_text = use_calibration_.load() ? "Calibration: ON" : "Calibration: OFF";
            renderText(20, 90, calib_text.c_str());
            
            std::string source_text = "Source: " + calibration_.getSource();
            renderText(20, 110, source_text.c_str());
            
            renderText(20, 130, "Status: FIXED - Thread-Safe");
            
            glEnable(GL_DEPTH_TEST);
            
            glPopMatrix();
            glMatrixMode(GL_PROJECTION);
            glPopMatrix();
            glMatrixMode(GL_MODELVIEW);
            
        } catch (const std::exception& e) {
            std::cerr << "Stats draw error: " << e.what() << std::endl;
        }
    }
    
    void renderText(float x, float y, const char* text) {
        if (!text) return;
        
        glRasterPos2f(x, y);
        for (const char* c = text; *c != '\0'; c++) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
        }
    }
    
    void reshape(int width, int height) {
        if (should_exit_.load() || !opengl_initialized_.load()) return;
        
        try {
            if (height == 0) height = 1; // Prevent division by zero
            
            glViewport(0, 0, width, height);
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluPerspective(60.0, static_cast<double>(width) / static_cast<double>(height), 0.01, 100.0);
            glMatrixMode(GL_MODELVIEW);
        } catch (const std::exception& e) {
            std::cerr << "Reshape error: " << e.what() << std::endl;
        }
    }
    
    void keyboard(unsigned char key, int x, int y) {
        (void)x; // Suppress unused parameter warning
        (void)y; // Suppress unused parameter warning
        
        if (should_exit_.load()) return;
        
        try {
            switch (key) {
                case 'q': case 'Q': case 27: // ESC
                    should_exit_.store(true);
                    std::cout << "Exiting..." << std::endl;
                    exit(0);
                    break;
                case 'c': case 'C':
                    use_calibration_.store(!use_calibration_.load());
                    std::cout << "Calibration: " << (use_calibration_.load() ? "ON" : "OFF") << std::endl;
                    break;
                case 'a': case 'A':
                    show_axes_.store(!show_axes_.load());
                    std::cout << "Axes: " << (show_axes_.load() ? "ON" : "OFF") << std::endl;
                    break;
                case 'r': case 'R':
                    auto_rotate_.store(!auto_rotate_.load());
                    std::cout << "Auto-rotate: " << (auto_rotate_.load() ? "ON" : "OFF") << std::endl;
                    break;
                case '+': case '=':
                    // Decrease downsample factor (more points)
                    if (downsample_factor_.load() > 1) {
                        downsample_factor_.fetch_sub(1);
                        std::cout << "Point density increased (downsample: " << downsample_factor_.load() << "x)" << std::endl;
                    }
                    break;
                case '-': case '_':
                    // Increase downsample factor (fewer points)
                    if (downsample_factor_.load() < 8) {
                        downsample_factor_.fetch_add(1);
                        std::cout << "Point density decreased (downsample: " << downsample_factor_.load() << "x)" << std::endl;
                    }
                    break;
                case 'p': case 'P':
                    // Cycle through point sizes 1-5
                    {
                        int current_size = point_size_.load();
                        int new_size = (current_size % 5) + 1;
                        point_size_.store(new_size);
                        std::cout << "Point size: " << new_size << "px" << std::endl;
                    }
                    break;
                case ' ':
                    // Reset view
                    camera_distance_.store(1.5f);
                    camera_angle_x_.store(20.0f);
                    camera_angle_y_.store(0.0f);
                    std::cout << "View reset" << std::endl;
                    break;
            }
        } catch (const std::exception& e) {
            std::cerr << "Keyboard error: " << e.what() << std::endl;
        }
    }
    
    void mouse(int button, int state, int x, int y) {
        if (should_exit_.load()) return;
        
        try {
            std::lock_guard<std::mutex> lock(mouse_mutex_);
            
            if (button == GLUT_LEFT_BUTTON) {
                mouse_pressed_ = (state == GLUT_DOWN);
            } else if (button == GLUT_RIGHT_BUTTON) {
                right_mouse_pressed_ = (state == GLUT_DOWN);
            } else if (button == 3) { // Wheel up
                float current_dist = camera_distance_.load();
                float new_dist = current_dist * 0.9f;
                if (new_dist < 0.1f) new_dist = 0.1f;
                camera_distance_.store(new_dist);
            } else if (button == 4) { // Wheel down
                float current_dist = camera_distance_.load();
                float new_dist = current_dist * 1.1f;
                if (new_dist > 10.0f) new_dist = 10.0f;
                camera_distance_.store(new_dist);
            }
            last_mouse_x_ = x;
            last_mouse_y_ = y;
        } catch (const std::exception& e) {
            std::cerr << "Mouse error: " << e.what() << std::endl;
        }
    }
    
    void motion(int x, int y) {
        if (should_exit_.load()) return;
        
        try {
            std::lock_guard<std::mutex> lock(mouse_mutex_);
            
            if (mouse_pressed_) {
                float dx = x - last_mouse_x_;
                float dy = y - last_mouse_y_;
                
                float new_angle_y = camera_angle_y_.load() + dx * 0.5f;
                float new_angle_x = camera_angle_x_.load() + dy * 0.5f;
                
                if (new_angle_x > 89.0f) new_angle_x = 89.0f;
                if (new_angle_x < -89.0f) new_angle_x = -89.0f;
                
                camera_angle_y_.store(new_angle_y);
                camera_angle_x_.store(new_angle_x);
            } else if (right_mouse_pressed_) {
                float dx = (x - last_mouse_x_) * 0.01f;
                float dy = (y - last_mouse_y_) * 0.01f;
                
                camera_target_x_.store(camera_target_x_.load() + dx);
                camera_target_y_.store(camera_target_y_.load() - dy);
            }
            
            last_mouse_x_ = x;
            last_mouse_y_ = y;
        } catch (const std::exception& e) {
            std::cerr << "Motion error: " << e.what() << std::endl;
        }
    }
};

// Static member definitions
std::atomic<PointCloudVisualizer*> PointCloudVisualizer::instance_{nullptr};
std::mutex PointCloudVisualizer::instance_mutex_;

int main(int argc, char** argv) {
    std::cout << "=== DetermSensor Point Cloud Viewer (FIXED) ===" << std::endl;
    
    try {
        // Initialize DetermSensor
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
        if (sensor.SetColorSensorConfig(640, 480, "RGB888") != 0) {
            std::cerr << "Failed to configure color sensor" << std::endl;
            return -1;
        }
        
        if (sensor.SetDepthSensorConfig(640, 480) != 0) {
            std::cerr << "Failed to configure depth sensor" << std::endl;
            return -1;
        }
        
        // Start sensor
        if (sensor.Start() != 0) {
            std::cerr << "Failed to start sensor" << std::endl;
            return -1;
        }
        
        // Start synchronized capture
        if (!sensor.StartSynchronizedCapture(SyncMode::SOFTWARE_SYNC)) {
            std::cerr << "Failed to start synchronized capture" << std::endl;
            return -1;
        }
        
        // Initialize visualizer with proper RAII
        std::unique_ptr<PointCloudVisualizer> visualizer;
        try {
            visualizer = std::make_unique<PointCloudVisualizer>(&sensor);
            visualizer->initOpenGL(argc, argv);
        } catch (const std::exception& e) {
            std::cerr << "Failed to initialize visualizer: " << e.what() << std::endl;
            return -1;
        }
        
        // Start processing thread
        std::atomic<bool> running{true};
        std::thread processing_thread([&]() {
            SynchronizedFrameSet frame_set;
            while (running.load() && !visualizer->should_exit_.load()) {
                try {
                    if (sensor.GetLatestSynchronizedFrames(frame_set)) {
                        visualizer->processFrames(frame_set);
                    } else {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Processing thread error: " << e.what() << std::endl;
                    break;
                }
            }
        });
        
        std::cout << "Starting OpenGL main loop..." << std::endl;
        
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
        
        std::cout << "Shutdown complete." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}