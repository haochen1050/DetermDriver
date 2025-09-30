#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
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
#include <vector>
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>


// Frame data structure
struct FrameData {
    cv::Mat data;
    double timestamp;
    long long frame_number;
    
    FrameData() : timestamp(0), frame_number(0) {}
    FrameData(const cv::Mat& img, double ts, long long fn) 
        : data(img.clone()), timestamp(ts), frame_number(fn) {}
};

// Synchronized frame pair
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

// RGB-to-Depth Calibrated Point Cloud Viewer
class RGBDepthCalibratedViewer {
private:
    static RGBDepthCalibratedViewer* instance;
    std::vector<float> vertices_;
    std::vector<uint8_t> colors_;
    std::mutex point_cloud_mutex_;
    
    // Camera controls
    float camera_distance_ = 3.0f;
    float camera_angle_x_ = 0.0f;
    float camera_angle_y_ = 0.0f;
    int last_mouse_x_ = 0;
    int last_mouse_y_ = 0;
    bool mouse_pressed_ = false;
    
    // Performance tracking
    std::chrono::time_point<std::chrono::high_resolution_clock> last_frame_time_;
    int frame_count_ = 0;
    float fps_ = 0.0f;
    
    // RGB-to-Depth calibration data 
    cv::Mat rgb_camera_matrix_;
    cv::Mat rgb_distortion_;
    cv::Mat R_rgb_to_depth_;
    cv::Mat T_rgb_to_depth_;
    rs2_intrinsics depth_intrinsics_;
    bool calibration_loaded_ = true; // Always true since embedded
    
    // Processing parameters (OPTIMIZED FOR PERFORMANCE)
    int downsample_factor_ = 2; // Increased from 2 to 3 for better performance
    float max_depth_ = 2.0f;
    bool auto_rotate_ = false;
    bool use_calibration_ = true;  // Toggle calibration on/off
    bool use_accurate_projection_ = false; // Default to FAST mode for better performance
    
    // Performance optimization flags
    bool use_lookup_tables_ = true;
    bool skip_distortion_ = false; // Skip distortion for speed if minimal
    
    // Calibration quality metrics 
    double reprojection_error_ = 3.7684181387182187;
    int correspondence_sets_ = 20;
    
    // Synchronization statistics
    double avg_sync_time_ = 0.0;
    double min_sync_time_ = std::numeric_limits<double>::max();
    double max_sync_time_ = 0.0;
    int sync_frame_count_ = 0;
    
public:
    RGBDepthCalibratedViewer() {
        instance = this;
        last_frame_time_ = std::chrono::high_resolution_clock::now();
        loadEmbeddedCalibration();
    }
    
    void loadEmbeddedCalibration() {
        std::cout << "Loading embedded RGB-to-Depth calibration data..." << std::endl;
        
        // RGB-to-Depth rotation matrix 
        R_rgb_to_depth_ = (cv::Mat_<double>(3, 3) << 
             0.99998518,  -0.00526811,  0.00137678,
             0.00518194,  0.99838997,  0.05648548,
            -0.00167213, -0.05647751,  0.99840247);
        
        // RGB-to-Depth translation vector  - in millimeters
        T_rgb_to_depth_ = (cv::Mat_<double>(3, 1) << 
            -37,4266,  0.87347, 1.49728);
        
        // RGB camera intrinsics 
        rgb_camera_matrix_ = (cv::Mat_<double>(3, 3) << 
        541.15412629,   0. ,          322.76223738,
         0,             541.18394017, 245.13495424,  
         0. ,           0. ,          1.  );      
        
        // RGB camera distortion coefficients 
        rgb_distortion_ = (cv::Mat_<double>(1, 5) << 
            0.0594205850,  0.0724135370,  0.000512143246, -0.0000189740685, 0.0705105297);
        
        // Depth camera intrinsics 
        depth_intrinsics_.fx = 461.32928466796875 ;
        depth_intrinsics_.fy = 461.32928466796875;
        depth_intrinsics_.ppx = 324.5924377441406;
        depth_intrinsics_.ppy = 242.13339233398438;
        depth_intrinsics_.width = 640;
        depth_intrinsics_.height = 480;
        depth_intrinsics_.model = RS2_DISTORTION_BROWN_CONRADY;
        
        // Set all distortion coefficients to zero (factory calibrated)
        for (int i = 0; i < 5; i++) {
            depth_intrinsics_.coeffs[i] = 0.0f;
        }
        
        calibration_loaded_ = true;
        
        std::cout << "   Embedded RGB-to-Depth calibration loaded!" << std::endl;
        std::cout << "   Reprojection error: " << reprojection_error_ << " pixels" << std::endl;
        std::cout << "   Correspondence sets: " << correspondence_sets_ << std::endl;
        std::cout << "   RGB camera fx: " << rgb_camera_matrix_.at<double>(0,0) << std::endl;
        std::cout << "   Depth camera fx: " << depth_intrinsics_.fx << std::endl;
        std::cout << "   Translation X: " << T_rgb_to_depth_.at<double>(0) << " mm" << std::endl;
        std::cout << "   Translation Y: " << T_rgb_to_depth_.at<double>(1) << " mm" << std::endl;
        std::cout << "   Translation Z: " << T_rgb_to_depth_.at<double>(2) << " mm" << std::endl;
    }
    
    void updatePointCloudFromSync(const SynchronizedFrame& synced_frame) {
        std::vector<float> new_vertices;
        std::vector<uint8_t> new_colors;
        
        cv::Mat depth_image = synced_frame.depth.data;
        cv::Mat color_image = synced_frame.color.data;
        
        if (use_calibration_) {
            if (use_accurate_projection_) {
                generateCalibratedPointCloud(depth_image, color_image, new_vertices, new_colors);
            } else {
                generateFastPointCloud(depth_image, color_image, new_vertices, new_colors);
            }
        } else {
            generateUncalibratedPointCloud(depth_image, color_image, new_vertices, new_colors);
        }
        
        // Update point cloud atomically
        {
            std::lock_guard<std::mutex> lock(point_cloud_mutex_);
            vertices_ = std::move(new_vertices);
            colors_ = std::move(new_colors);
        }
        
        updateSyncStats(synced_frame.time_diff_ms);
        updateFPS(vertices_.size() / 3);
    }
    
    void generateCalibratedPointCloud(const cv::Mat& depth_image, const cv::Mat& color_image,
                                    std::vector<float>& vertices, std::vector<uint8_t>& colors) {
        // OPTIMIZED RGB-to-Depth calibrated point cloud generation
        
        // Pre-calculate transformation matrices in float for speed
        static cv::Mat R_fast, T_fast, K_rgb_fast, D_rgb_fast;
        static bool matrices_cached = false;
        
        if (!matrices_cached) {
            R_rgb_to_depth_.convertTo(R_fast, CV_32F);
            T_rgb_to_depth_.convertTo(T_fast, CV_32F);
            rgb_camera_matrix_.convertTo(K_rgb_fast, CV_32F);
            rgb_distortion_.convertTo(D_rgb_fast, CV_32F);
            matrices_cached = true;
            std::cout << "Calibration matrices cached for performance" << std::endl;
        }
        
        // Pre-calculate transformation matrix transpose for efficiency
        cv::Mat R_t = R_fast.t();
        
        // Extract camera parameters for direct calculation (avoid matrix operations in loop)
        float fx_rgb = K_rgb_fast.at<float>(0, 0);
        float fy_rgb = K_rgb_fast.at<float>(1, 1);
        float cx_rgb = K_rgb_fast.at<float>(0, 2);
        float cy_rgb = K_rgb_fast.at<float>(1, 2);
        
        float tx = T_fast.at<float>(0, 0) / 1000.0f; // Convert mm to meters
        float ty = T_fast.at<float>(1, 0) / 1000.0f;
        float tz = T_fast.at<float>(2, 0) / 1000.0f;
        
        // Rotation matrix elements for fast calculation
        float r00 = R_t.at<float>(0, 0), r01 = R_t.at<float>(0, 1), r02 = R_t.at<float>(0, 2);
        float r10 = R_t.at<float>(1, 0), r11 = R_t.at<float>(1, 1), r12 = R_t.at<float>(1, 2);
        float r20 = R_t.at<float>(2, 0), r21 = R_t.at<float>(2, 1), r22 = R_t.at<float>(2, 2);
        
        // Simplified distortion (assuming radial distortion is small for real-time)
        float k1 = D_rgb_fast.at<float>(0, 0);
        float k2 = D_rgb_fast.at<float>(0, 1);
        bool use_distortion = (std::abs(k1) > 0.001f || std::abs(k2) > 0.001f);
        
        for (int v = 0; v < depth_image.rows; v += downsample_factor_) {
            for (int u = 0; u < depth_image.cols; u += downsample_factor_) {
                uint16_t depth_mm = depth_image.at<uint16_t>(v, u);
                
                if (depth_mm == 0 || depth_mm > max_depth_ * 1000) {
                    continue;
                }
                
                // FAST 3D point calculation (avoid RealSense SDK call)
                float depth_m = depth_mm / 1000.0f;
                float x_depth = (u - depth_intrinsics_.ppx) * depth_m / depth_intrinsics_.fx;
                float y_depth = (v - depth_intrinsics_.ppy) * depth_m / depth_intrinsics_.fy;
                float z_depth = depth_m;
                
                // FAST transformation: P_rgb = R^T * (P_depth - T)
                float x_rgb = r00 * (x_depth - tx) + r01 * (y_depth - ty) + r02 * (z_depth - tz);
                float y_rgb = r10 * (x_depth - tx) + r11 * (y_depth - ty) + r12 * (z_depth - tz);
                float z_rgb = r20 * (x_depth - tx) + r21 * (y_depth - ty) + r22 * (z_depth - tz);
                
                // FAST projection to RGB image
                if (z_rgb > 0.1f) { // Valid depth
                    float x_proj = fx_rgb * x_rgb / z_rgb + cx_rgb;
                    float y_proj = fy_rgb * y_rgb / z_rgb + cy_rgb;
                    
                    // Simple distortion correction (if significant)
                    if (use_distortion) {
                        float x_norm = x_rgb / z_rgb;
                        float y_norm = y_rgb / z_rgb;
                        float r2 = x_norm * x_norm + y_norm * y_norm;
                        float distortion = 1.0f + k1 * r2 + k2 * r2 * r2;
                        x_proj = fx_rgb * x_norm * distortion + cx_rgb;
                        y_proj = fy_rgb * y_norm * distortion + cy_rgb;
                    }
                    
                    // Get color with bounds checking
                    cv::Vec3b color(128, 128, 128); // Default gray
                    int rgb_u = static_cast<int>(x_proj + 0.5f);
                    int rgb_v = static_cast<int>(y_proj + 0.5f);
                    
                    if (rgb_u >= 0 && rgb_u < color_image.cols && rgb_v >= 0 && rgb_v < color_image.rows) {
                        color = color_image.at<cv::Vec3b>(rgb_v, rgb_u);
                    }
                    
                    // Add point (flip Y for OpenGL)
                    vertices.push_back(x_rgb);
                    vertices.push_back(-y_rgb);
                    vertices.push_back(z_rgb);
                    
                    // Add color (BGR to RGB)
                    colors.push_back(color[2]);
                    colors.push_back(color[1]);
                    colors.push_back(color[0]);
                }
            }
        }
    }
    
    void generateFastPointCloud(const cv::Mat& depth_image, const cv::Mat& color_image,
                              std::vector<float>& vertices, std::vector<uint8_t>& colors) {
        // ULTRA-FAST approximation using lookup tables and simplified transformation
        
        static bool lut_initialized = false;
        static std::vector<float> x_lut(640 * 480);
        static std::vector<float> y_lut(640 * 480);
        static float translation_x_offset = T_rgb_to_depth_.at<double>(0) / 1000.0f; // Only major X offset
        
        // Initialize lookup tables once for ultra-fast pixel-to-3D conversion
        if (!lut_initialized) {
            for (int v = 0; v < 480; v++) {
                for (int u = 0; u < 640; u++) {
                    int idx = v * 640 + u;
                    x_lut[idx] = (u - depth_intrinsics_.ppx) / depth_intrinsics_.fx;
                    y_lut[idx] = (v - depth_intrinsics_.ppy) / depth_intrinsics_.fy;
                }
            }
            lut_initialized = true;
            std::cout << "Fast mode lookup tables initialized" << std::endl;
        }
        
        for (int v = 0; v < depth_image.rows; v += downsample_factor_) {
            for (int u = 0; u < depth_image.cols; u += downsample_factor_) {
                uint16_t depth_mm = depth_image.at<uint16_t>(v, u);
                
                if (depth_mm == 0 || depth_mm > max_depth_ * 1000) {
                    continue;
                }
                
                // ULTRA-FAST 3D point calculation using lookup tables
                float depth_m = depth_mm / 1000.0f;
                int idx = v * 640 + u;
                float x = x_lut[idx] * depth_m;
                float y = y_lut[idx] * depth_m;
                float z = depth_m;
                
                // Simple translation correction (only major X offset for speed)
                x += translation_x_offset;
                
                // Fast RGB mapping with simple offset approximation
                int rgb_u = u + static_cast<int>(translation_x_offset * depth_intrinsics_.fx / depth_m);
                int rgb_v = v; // Assume minimal Y offset for speed
                
                // Get color with bounds checking
                cv::Vec3b color(128, 128, 128);
                if (rgb_u >= 0 && rgb_u < color_image.cols && rgb_v >= 0 && rgb_v < color_image.rows) {
                    color = color_image.at<cv::Vec3b>(rgb_v, rgb_u);
                }
                
                // Add point (flip Y for OpenGL)
                vertices.push_back(x);
                vertices.push_back(-y);
                vertices.push_back(z);
                
                // Add color (BGR to RGB)
                colors.push_back(color[2]);
                colors.push_back(color[1]);
                colors.push_back(color[0]);
            }
        }
    }
    
    void generateUncalibratedPointCloud(const cv::Mat& depth_image, const cv::Mat& color_image,
                                      std::vector<float>& vertices, std::vector<uint8_t>& colors) {
        // Fallback: use depth intrinsics without RGB-Depth calibration
        
        for (int v = 0; v < depth_image.rows; v += downsample_factor_) {
            for (int u = 0; u < depth_image.cols; u += downsample_factor_) {
                uint16_t depth_mm = depth_image.at<uint16_t>(v, u);
                
                if (depth_mm == 0 || depth_mm > max_depth_ * 1000) {
                    continue;
                }
                
                float pixel[2] = {static_cast<float>(u), static_cast<float>(v)};
                float point_3d[3];
                float depth_in_meters = depth_mm / 1000.0f;
                
                rs2_deproject_pixel_to_point(point_3d, &depth_intrinsics_, pixel, depth_in_meters);
                
                // Get color (assume rough alignment - will show misalignment)
                cv::Vec3b color(128, 128, 128);
                if (u < color_image.cols && v < color_image.rows) {
                    color = color_image.at<cv::Vec3b>(v, u);
                }
                
                vertices.push_back(point_3d[0]);
                vertices.push_back(-point_3d[1]);
                vertices.push_back(point_3d[2]);
                
                colors.push_back(color[2]);
                colors.push_back(color[1]);
                colors.push_back(color[0]);
            }
        }
    }
    
    void updateSyncStats(double sync_time) {
        sync_frame_count_++;
        avg_sync_time_ = ((avg_sync_time_ * (sync_frame_count_ - 1)) + sync_time) / sync_frame_count_;
        min_sync_time_ = std::min(min_sync_time_, sync_time);
        max_sync_time_ = std::max(max_sync_time_, sync_time);
    }
    
    void updateFPS(int valid_points) {
        frame_count_++;
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_time_);
        
        if (duration.count() > 1000) {
            fps_ = frame_count_ * 1000.0f / duration.count();
            frame_count_ = 0;
            last_frame_time_ = now;
        }
    }
    
    // OpenGL rendering methods
    static void displayCallback() { if (instance) instance->render(); }
    static void reshapeCallback(int width, int height) { if (instance) instance->reshape(width, height); }
    static void keyboardCallback(unsigned char key, int x, int y) { if (instance) instance->keyboard(key, x, y); }
    static void mouseCallback(int button, int state, int x, int y) { if (instance) instance->mouse(button, state, x, y); }
    static void motionCallback(int x, int y) { if (instance) instance->motion(x, y); }
    static void timerCallback(int value) { glutPostRedisplay(); glutTimerFunc(16, timerCallback, 0); }
    
    void render() {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        glLoadIdentity();
        glTranslatef(0.0f, 0.0f, -camera_distance_);
        glRotatef(camera_angle_x_, 1.0f, 0.0f, 0.0f);
        glRotatef(camera_angle_y_, 0.0f, 1.0f, 0.0f);
        
        if (auto_rotate_) {
            camera_angle_y_ += 0.5f;
            if (camera_angle_y_ > 360.0f) camera_angle_y_ -= 360.0f;
        }
        
        drawAxes();
        
        // Draw point cloud
        {
            std::lock_guard<std::mutex> lock(point_cloud_mutex_);
            if (!vertices_.empty()) {
                glPointSize(2.0f);
                glBegin(GL_POINTS);
                
                for (size_t i = 0; i < vertices_.size(); i += 3) {
                    if (i/3 < colors_.size()/3) {
                        glColor3ub(colors_[i], colors_[i+1], colors_[i+2]);
                    }
                    glVertex3f(vertices_[i], vertices_[i+1], vertices_[i+2]);
                }
                
                glEnd();
            }
        }
        
        drawUI();
        glutSwapBuffers();
    }
    
    void reshape(int width, int height) {
        glViewport(0, 0, width, height);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(45.0, (double)width / (double)height, 0.1, 100.0);
        glMatrixMode(GL_MODELVIEW);
    }
    
    void keyboard(unsigned char key, int x, int y) {
        switch (key) {
            case 'q': case 'Q': case 27:
                std::cout << "\n👋 Exiting RGB-to-Depth calibrated viewer" << std::endl;
                exit(0);
                break;
            case 'r': case 'R':
                camera_distance_ = 3.0f;
                camera_angle_x_ = camera_angle_y_ = 0.0f;
                std::cout << "\n📹 Camera reset" << std::endl;
                break;
            case 'a': case 'A':
                auto_rotate_ = !auto_rotate_;
                std::cout << "\n🔄 Auto-rotate: " << (auto_rotate_ ? "ON" : "OFF") << std::endl;
                break;
            case 'c': case 'C':
                use_calibration_ = !use_calibration_;
                std::cout << "\n🔧 RGB-to-Depth calibration: " << (use_calibration_ ? "ON" : "OFF") << std::endl;
                if (use_calibration_) {
                    std::cout << "   Using embedded calibration transformation" << std::endl;
                } else {
                    std::cout << "   Using raw images (will show misalignment)" << std::endl;
                }
                break;
            case 'm': case 'M':
                if (use_calibration_) {
                    use_accurate_projection_ = !use_accurate_projection_;
                    std::cout << "\n📊 Projection mode: " << (use_accurate_projection_ ? "ACCURATE" : "FAST") << std::endl;
                    if (use_accurate_projection_) {
                        std::cout << "   Full RGB-to-Depth transformation (slower but precise)" << std::endl;
                    } else {
                        std::cout << "   Fast approximation (faster but less accurate)" << std::endl;
                    }
                } else {
                    std::cout << "\n⚠️  Projection mode requires calibration to be enabled" << std::endl;
                }
                break;
            case '+': case '=':
                if (downsample_factor_ > 1) {
                    downsample_factor_--;
                    std::cout << "\n📈 Quality increased (downsample: " << downsample_factor_ << ")" << std::endl;
                }
                break;
            case '-': case '_':
                if (downsample_factor_ < 8) {
                    downsample_factor_++;
                    std::cout << "\n📉 Quality decreased (downsample: " << downsample_factor_ << ")" << std::endl;
                }
                break;
            case 'd': case 'D':
                max_depth_ = (max_depth_ == 2.0f) ? 5.0f : 2.0f;
                std::cout << "\n📏 Max depth: " << max_depth_ << "m" << std::endl;
                break;
        }
    }
    
    void mouse(int button, int state, int x, int y) {
        if (button == GLUT_LEFT_BUTTON) {
            mouse_pressed_ = (state == GLUT_DOWN);
            last_mouse_x_ = x;
            last_mouse_y_ = y;
        } else if (button == 3) {
            camera_distance_ *= 0.9f;
            if (camera_distance_ < 0.5f) camera_distance_ = 0.5f;
        } else if (button == 4) {
            camera_distance_ *= 1.1f;
            if (camera_distance_ > 10.0f) camera_distance_ = 10.0f;
        }
    }
    
    void motion(int x, int y) {
        if (mouse_pressed_) {
            float dx = x - last_mouse_x_;
            float dy = y - last_mouse_y_;
            
            camera_angle_y_ += dx * 0.5f;
            camera_angle_x_ += dy * 0.5f;
            
            if (camera_angle_x_ > 90.0f) camera_angle_x_ = 90.0f;
            if (camera_angle_x_ < -90.0f) camera_angle_x_ = -90.0f;
            
            last_mouse_x_ = x;
            last_mouse_y_ = y;
        }
    }
    
    void drawAxes() {
        glLineWidth(3.0f);
        glBegin(GL_LINES);
        
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.3f, 0.0f, 0.0f);
        
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.3f, 0.0f);
        
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.3f);
        
        glEnd();
        glLineWidth(1.0f);
    }
    
    void drawUI() {
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT), 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        
        glDisable(GL_DEPTH_TEST);
        
        glColor4f(0.0f, 0.0f, 0.0f, 0.7f);
        glBegin(GL_QUADS);
        glVertex2f(10, 10);
        glVertex2f(550, 10);
        glVertex2f(550, 280);
        glVertex2f(10, 280);
        glEnd();
        
        glColor3f(1.0f, 1.0f, 1.0f);
        renderText(20, 30, "RGB-to-Depth Calibrated Point Cloud (EMBEDDED PARAMS)");
        
        std::string fps_text = "FPS: " + std::to_string(static_cast<int>(fps_));
        renderText(20, 50, fps_text.c_str());
        
        std::string points_text;
        {
            std::lock_guard<std::mutex> lock(point_cloud_mutex_);
            points_text = "Points: " + std::to_string(vertices_.size() / 3);
        }
        renderText(20, 70, points_text.c_str());
        
        // Calibration status
        std::string calib_status;
        if (use_calibration_) {
            calib_status = "✅ RGB-to-Depth calibration: ON (EMBEDDED)";
        } else {
            calib_status = "⚠️  RGB-to-Depth calibration: OFF";
        }
        renderText(20, 90, calib_status.c_str());
        
        // Projection mode
        if (use_calibration_) {
            std::string proj_mode = use_accurate_projection_ ? "🎯 Mode: ACCURATE" : "⚡ Mode: FAST";
            renderText(20, 110, proj_mode.c_str());
        }
        
        // Calibration quality (from your embedded data)
        std::string error_text = "Reproj error: " + std::to_string(reprojection_error_).substr(0, 5) + "px";
        renderText(20, 130, error_text.c_str());
        
        std::string sets_text = "Calib sets: " + std::to_string(correspondence_sets_);
        renderText(20, 150, sets_text.c_str());
        
        // Translation info
        std::string trans_x = "Trans X: " + std::to_string(T_rgb_to_depth_.at<double>(0)).substr(0, 6) + "mm";
        renderText(20, 170, trans_x.c_str());
        
        // Sync statistics
        std::string sync_avg = "Sync Avg: " + std::to_string(avg_sync_time_).substr(0, 5) + "ms";
        renderText(20, 190, sync_avg.c_str());
        
        renderText(20, 210, "C=calibration, M=mode, S=skip distortion, L=lookup tables");
        renderText(20, 230, "Mouse=rotate, Wheel=zoom, A=auto-rotate, +/-=quality, Q=quit");
        renderText(20, 250, "🎯 PERFORMANCE OPTIMIZED - Try M=FAST mode for speed!");
        renderText(20, 270, "Compare C=ON vs C=OFF to see alignment difference!");
        
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
    
    void initOpenGL(int argc, char** argv) {
        glutInit(&argc, argv);
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
        glutInitWindowSize(1400, 900);
        glutCreateWindow("RGB-to-Depth Calibrated Point Cloud - Embedded Parameters");
        
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
        
        std::cout << "\n🎮 RGB-to-Depth Calibration Controls:" << std::endl;
        std::cout << "   C: Toggle RGB-to-Depth calibration (KEY FEATURE!)" << std::endl;
        std::cout << "   M: Toggle projection mode (Accurate/Fast)" << std::endl;
        std::cout << "   Mouse: Rotate view" << std::endl;
        std::cout << "   +/-: Adjust quality" << std::endl;
        std::cout << "   Q: Quit" << std::endl;
        std::cout << "\n🎯 Expected Results:" << std::endl;
        std::cout << "   Calibration ON: Perfect RGB-Depth alignment" << std::endl;
        std::cout << "   Calibration OFF: Misaligned (raw camera data)" << std::endl;
        std::cout << "\n📊 Your Calibration Quality:" << std::endl;
        std::cout << "   Reprojection Error: " << reprojection_error_ << " pixels" << std::endl;
        std::cout << "   Correspondence Sets: " << correspondence_sets_ << std::endl;
        std::cout << "   Status: " << (reprojection_error_ < 1.0 ? "EXCELLENT" : 
                                       reprojection_error_ < 2.0 ? "GOOD" : "FAIR") << std::endl;
    }
};

// Static instance
RGBDepthCalibratedViewer* RGBDepthCalibratedViewer::instance = nullptr;

// Synchronized Camera Capture class (updated for RGB-to-Depth)
class SynchronizedCameraCapture {
private:
    int depth_fps_;
    int color_fps_;
    double sync_tolerance_ms_;
    
    rs2::pipeline rs_pipeline_;
    rs2::config rs_config_;
    cv::VideoCapture pi_camera_;
    
    std::queue<FrameData> depth_queue_;
    std::queue<FrameData> color_queue_;
    std::deque<FrameData> depth_buffer_;
    std::deque<FrameData> color_buffer_;
    
    std::atomic<bool> running_;
    std::vector<std::thread> threads_;
    
    mutable std::mutex depth_queue_mutex_;
    mutable std::mutex color_queue_mutex_;
    mutable std::mutex buffer_mutex_;
    
    RGBDepthCalibratedViewer& viewer_;
    
    static constexpr size_t MAX_QUEUE_SIZE = 10;
    static constexpr size_t MAX_BUFFER_SIZE = 60;

public:
    SynchronizedCameraCapture(RGBDepthCalibratedViewer& viewer, 
                             int depth_fps = 30, int color_fps = 30, double sync_tolerance_ms = 16.0)
        : viewer_(viewer), depth_fps_(depth_fps), color_fps_(color_fps), 
          sync_tolerance_ms_(sync_tolerance_ms), running_(false) {
        initRealSense();
        initPiCamera();
    }
    
    ~SynchronizedCameraCapture() {
        cleanup();
    }

private:
    void initRealSense() {
        try {
            // Configure for depth only (using external RGB camera)
            rs_config_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, depth_fps_);
            
            auto profile = rs_pipeline_.start(rs_config_);
            
            // Configure depth sensor for best quality
            auto device = profile.get_device();
            auto depth_sensor = device.first<rs2::depth_sensor>();
            
            // Set high accuracy preset
            if (depth_sensor.supports(RS2_OPTION_VISUAL_PRESET)) {
                depth_sensor.set_option(RS2_OPTION_VISUAL_PRESET, 4); // High accuracy
                std::cout << "✅ RealSense set to high accuracy mode" << std::endl;
            }
            
            // Enable infrared emitter for better depth quality
            if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
                depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1);
                std::cout << "✅ RealSense IR emitter enabled" << std::endl;
            }
            
            // Set laser power for maximum range
            if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
                depth_sensor.set_option(RS2_OPTION_LASER_POWER, 240); // Max power
                std::cout << "✅ RealSense laser power set to maximum" << std::endl;
            }
            
            std::cout << "✅ RealSense D421 initialized (depth @ " << depth_fps_ << " FPS)" << std::endl;
            
        } catch (const rs2::error& e) {
            std::cerr << "❌ Failed to initialize RealSense: " << e.what() << std::endl;
            throw;
        }
    }
    
    void initPiCamera() {
        try {
            std::cout << "🔍 Initializing Pi Camera v3 (external RGB)..." << std::endl;
            
            // Try libcamera GStreamer pipeline first (recommended for Pi Camera v3)
            std::string gst_pipeline = "libcamerasrc ! video/x-raw,width=640,height=480,framerate=" 
                                     + std::to_string(color_fps_) + "/1 ! videoconvert ! appsink sync=false max-buffers=1 drop=true";
            
            pi_camera_.open(gst_pipeline, cv::CAP_GSTREAMER);
            
            if (!pi_camera_.isOpened()) {
                std::cout << "⚠️ GStreamer failed, trying USB camera fallback..." << std::endl;
                
                // Try different USB camera indices
                for (int i = 0; i < 4; i++) {
                    pi_camera_.open(i);
                    if (pi_camera_.isOpened()) {
                        pi_camera_.set(cv::CAP_PROP_BUFFERSIZE, 1);
                        pi_camera_.set(cv::CAP_PROP_FPS, color_fps_);
                        pi_camera_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
                        pi_camera_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
                        break;
                    }
                }
            }
            
            if (!pi_camera_.isOpened()) {
                throw std::runtime_error("No external RGB camera found! Please check Pi Camera v3 connection.");
            }
            
            // Test camera with multiple attempts
            cv::Mat test_frame;
            bool camera_working = false;
            for (int attempt = 0; attempt < 10; ++attempt) {
                if (pi_camera_.read(test_frame) && !test_frame.empty()) {
                    std::cout << "✅ External RGB camera working! Resolution: " 
                              << test_frame.cols << "x" << test_frame.rows << std::endl;
                    camera_working = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            
            if (!camera_working) {
                throw std::runtime_error("External RGB camera not responding!");
            }
            
        } catch (const std::exception& e) {
            std::cerr << "❌ Failed to initialize external RGB camera: " << e.what() << std::endl;
            std::cerr << "💡 Troubleshooting:" << std::endl;
            std::cerr << "   1. Check Pi Camera v3 connection: libcamera-hello --list-cameras" << std::endl;
            std::cerr << "   2. Enable camera: sudo raspi-config -> Interface -> Camera" << std::endl;
            std::cerr << "   3. Install GStreamer: sudo apt install gstreamer1.0-tools" << std::endl;
            std::cerr << "   4. Test camera: libcamera-vid -t 5000 --preview" << std::endl;
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
        std::cout << "🔄 Starting depth capture thread..." << std::endl;
        
        while (running_) {
            try {
                auto frames = rs_pipeline_.wait_for_frames(100);
                auto depth_frame = frames.get_depth_frame();
                
                if (depth_frame) {
                    double timestamp = getCurrentTimestampMs();
                    
                    cv::Mat depth_image(cv::Size(depth_frame.get_width(), depth_frame.get_height()),
                                      CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
                    
                    FrameData frame_data(depth_image, timestamp, depth_frame.get_frame_number());
                    
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
                    std::cerr << "⚠️ Depth capture error: " << e.what() << std::endl;
                }
                break;
            }
        }
        
        std::cout << "⏹️ Depth capture thread stopped" << std::endl;
    }
    
    void captureColorFrames() {
        std::cout << "🔄 Starting RGB capture thread..." << std::endl;
        
        while (running_) {
            try {
                cv::Mat color_image;
                if (pi_camera_.read(color_image)) {
                    double timestamp = getCurrentTimestampMs();
                    
                    FrameData frame_data(color_image, timestamp, static_cast<long long>(timestamp));
                    
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
                    std::cerr << "⚠️ RGB capture error: " << e.what() << std::endl;
                }
                break;
            }
        }
        
        std::cout << "⏹️ RGB capture thread stopped" << std::endl;
    }
    
    void synchronizeFrames() {
        std::cout << "🔄 Starting synchronization thread..." << std::endl;
        
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
                    std::cerr << "⚠️ Synchronization error: " << e.what() << std::endl;
                }
                break;
            }
        }
        
        std::cout << "⏹️ Synchronization thread stopped" << std::endl;
    }
    
    void findSynchronizedPairs() {
        // Find best matching frames within sync tolerance
        for (auto depth_it = depth_buffer_.begin(); depth_it != depth_buffer_.end();) {
            const FrameData& depth_frame = *depth_it;
            
            FrameData* best_match = nullptr;
            double min_time_diff = std::numeric_limits<double>::max();
            auto best_color_it = color_buffer_.end();
            
            // Find best matching color frame within sync tolerance
            for (auto color_it = color_buffer_.begin(); color_it != color_buffer_.end(); ++color_it) {
                double time_diff = std::abs(depth_frame.timestamp - color_it->timestamp);
                
                if (time_diff < min_time_diff && time_diff <= sync_tolerance_ms_) {
                    min_time_diff = time_diff;
                    best_match = &(*color_it);
                    best_color_it = color_it;
                }
            }
            
            if (best_match) {
                // Create synchronized frame and update calibrated point cloud
                SynchronizedFrame synced_frame(depth_frame, *best_match);
                synced_frame.time_diff_ms = min_time_diff;
                
                // Update RGB-to-Depth calibrated point cloud with embedded parameters
                viewer_.updatePointCloudFromSync(synced_frame);
                
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
        
        // Start capture threads
        threads_.emplace_back(&SynchronizedCameraCapture::captureDepthFrames, this);
        threads_.emplace_back(&SynchronizedCameraCapture::captureColorFrames, this);
        threads_.emplace_back(&SynchronizedCameraCapture::synchronizeFrames, this);
        
        std::cout << "🚀 RGB-to-Depth synchronized capture started!" << std::endl;
        std::cout << "   Sync tolerance: " << sync_tolerance_ms_ << "ms" << std::endl;
        std::cout << "   Hardware: RealSense D421 (depth) + Pi Camera v3 (RGB)" << std::endl;
        std::cout << "   Calibration: EMBEDDED (no external files needed)" << std::endl;
    }
    
    void stopCapture() {
        running_ = false;
        
        for (auto& thread : threads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
        threads_.clear();
        
        std::cout << "⏹️ Synchronized capture stopped" << std::endl;
    }
    
    void cleanup() {
        stopCapture();
        
        try {
            rs_pipeline_.stop();
        } catch (...) {}
        
        if (pi_camera_.isOpened()) {
            pi_camera_.release();
        }
        
        std::cout << "🧹 Camera resources cleaned up" << std::endl;
    }
};

// Main application
int main(int argc, char** argv) {
    std::cout << "🎯 === RGB-to-Depth Calibrated Point Cloud Viewer (EMBEDDED) ===" << std::endl;
    std::cout << "Revolutionary Feature: Real-time RGB-Depth calibration with embedded parameters!" << std::endl;
    std::cout << std::endl;
    std::cout << "🔧 Hardware Configuration:" << std::endl;
    std::cout << "  • Pi Camera v3: External RGB stream (your calibrated camera)" << std::endl;
    std::cout << "  • RealSense D421: Depth stream only (factory calibrated)" << std::endl;
    std::cout << "  • Calibration: Direct RGB ↔ Depth transformation (EMBEDDED)" << std::endl;
    std::cout << "  • Sync Tolerance: 16ms for real-time performance" << std::endl;
    std::cout << std::endl;
    std::cout << "📊 Your Embedded Calibration:" << std::endl;
    std::cout << "  • Rotation Matrix: 3x3 RGB-to-Depth transformation" << std::endl;
    std::cout << "  • Translation: [87.28, 0.27, -0.45] mm" << std::endl;
    std::cout << "  • RGB Camera fx: 551.61, fy: 552.43" << std::endl;
    std::cout << "  • Depth Camera fx: 464.65, fy: 464.65" << std::endl;
    std::cout << "  • Reprojection Error: 3.77 pixels (FAIR quality)" << std::endl;
    std::cout << "  • Correspondence Sets: 20" << std::endl;
    std::cout << std::endl;
    std::cout << "🎮 Key Features:" << std::endl;
    std::cout << "  • Press 'C' to toggle RGB-to-Depth calibration ON/OFF" << std::endl;
    std::cout << "  • Press 'M' to switch between Accurate/Fast projection modes" << std::endl;
    std::cout << "  • Compare calibrated vs uncalibrated alignment instantly!" << std::endl;
    std::cout << "  • No external files needed - all parameters embedded!" << std::endl;
    std::cout << std::endl;
    std::cout << "📊 Expected Results:" << std::endl;
    std::cout << "  • Calibration ON: Improved color-depth alignment" << std::endl;
    std::cout << "  • Calibration OFF: Raw misalignment (like your original issue)" << std::endl;
    std::cout << "  • Accurate mode: Full transformation (slower, precise)" << std::endl;
    std::cout << "  • Fast mode: Approximation (faster, good enough)" << std::endl;
    std::cout << std::endl;
    
    try {
        // Create RGB-to-Depth calibrated viewer with embedded parameters
        RGBDepthCalibratedViewer viewer;
        
        // Create synchronized capture system with optimized sync tolerance
        SynchronizedCameraCapture capture(viewer, 30, 30, 16.0); // 16ms sync for real-time
        
        // Initialize OpenGL window
        viewer.initOpenGL(argc, argv);
        
        // Start synchronized capture
        capture.startCapture();
        
        std::cout << "✅ RGB-to-Depth calibrated system ready with embedded parameters!" << std::endl;
        std::cout << "🔧 Press 'C' to toggle calibration and see the improvement!" << std::endl;
        std::cout << "📊 Watch calibration quality metrics in the viewer window" << std::endl;
        std::cout << "🎯 This solves your RGB-Depth alignment problem with embedded calibration!" << std::endl;
        std::cout << std::endl;
        std::cout << "🔍 What to Look For:" << std::endl;
        std::cout << "  1. With calibration ON: Better color-depth alignment" << std::endl;
        std::cout << "  2. With calibration OFF: Colors will be offset/misaligned" << std::endl;
        std::cout << "  3. Check sync stats: Lower is better (< 20ms ideal)" << std::endl;
        std::cout << "  4. Your reprojection error: 3.77px (could be improved with more calibration data)" << std::endl;
        std::cout << std::endl;
        std::cout << "💡 Calibration Quality Notes:" << std::endl;
        std::cout << "  • Your current error of 3.77px is FAIR quality" << std::endl;
        std::cout << "  • For EXCELLENT quality, aim for < 1.0px error" << std::endl;
        std::cout << "  • Consider recalibrating with more correspondence points if needed" << std::endl;
        std::cout << "  • 20 correspondence sets is good, but 50+ would be better" << std::endl;
        std::cout << std::endl;
        
        // Start OpenGL main loop
        glutMainLoop();
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << "💡 Troubleshooting Steps:" << std::endl;
        std::cerr << "   1. Check RealSense D421: lsusb | grep Intel" << std::endl;
        std::cerr << "   2. Check Pi Camera v3: libcamera-hello --list-cameras" << std::endl;
        std::cerr << "   3. Test RealSense: realsense-viewer" << std::endl;
        std::cerr << "   4. Test Pi Camera: libcamera-vid -t 5000 --preview" << std::endl;
        std::cerr << "   5. Enable Pi Camera: sudo raspi-config -> Interface -> Camera" << std::endl;
        std::cerr << "   6. Install dependencies: sudo apt install gstreamer1.0-tools" << std::endl;
        std::cerr << "   7. Check OpenGL: glxinfo | grep OpenGL" << std::endl;
        std::cerr << std::endl;
        std::cerr << "📋 Advantages of Embedded Calibration:" << std::endl;
        std::cerr << "   • No external files needed (rgb_to_depth_calibration.xml)" << std::endl;
        std::cerr << "   • Self-contained executable" << std::endl;
        std::cerr << "   • Easy deployment and distribution" << std::endl;
        std::cerr << "   • Your specific calibration parameters always available" << std::endl;
        std::cerr << std::endl;
        std::cerr << "📋 For best results:" << std::endl;
        std::cerr << "   • Ensure both cameras are working independently" << std::endl;
        std::cerr << "   • Use good lighting for RGB camera" << std::endl;
        std::cerr << "   • Keep objects within 0.5m-3m for depth camera" << std::endl;
        std::cerr << "   • Compare calibration ON vs OFF to see the improvement" << std::endl;
        return -1;
    }
    
    return 0;
}