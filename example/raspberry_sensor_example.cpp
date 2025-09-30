/**
 * @file raspberry_sensor_example.cpp
 * @brief Example usage of RaspberrySensor class with OpenCV visualization
 */
#include <opencv2/opencv.hpp>

#include "sensor/raspberry_sensor.h"

// Example usage
int main(int argc, char* argv[])
{
  determ::sensor::RaspberrySensor rpi_sensor;

  std::cout << "Initializing raspberry v3 camera ..." << std::endl;

  // Initialize the camera system
  if (rpi_sensor.Initialize() != 0) {
    return -1;
  }

  // Print all supported formats and resolutions
  rpi_sensor.PrintSupportedFormats();

  // Set sensor configuration
  // RaspberrySensor support RGB888, BGR888, YUYV
  // Uncomment the line above rpi_sensor.PrintSupportedFormats(); to check supported formats and resolutions
  std::cout << "Configuring raspberry v3 camera ..." << std::endl;
  if (argc == 1) {
    // Default configuration: 640x480 resolution, RGB888 format
    std::cout << "Using default configuration: 640x480 RGB888" << std::endl;
    if (rpi_sensor.SetSensorConfig(640, 480, "RGB888") != 0) {
      return -1;
    }
  } else if (argc == 2) {
    // Single argument: use it as the format, default resolution 640x480
    std::cout << "Using default resolution 640x480 with format: " << argv[1] << std::endl;
    if (rpi_sensor.SetSensorConfig(640, 480, std::string(argv[1])) != 0) {
      return -1;
    }
  } else if (argc == 4) {
    int width = std::stoi(argv[1]);
    int height = std::stoi(argv[2]);
    std::string format = argv[3];

    std::cout << "Using configuration: " << width << "x" << height << " " << format << std::endl;
    if (rpi_sensor.SetSensorConfig(width, height, format) != 0) {
      return -1;
    }
  } else {
    std::cerr << "Usage: " << argv[0] << " [width height format]" << std::endl;
    return -1;
  }

  // Enable auto exposure and white balance

  /*
  Parameter Guide:

  exposure_time (in microseconds):
  - 1000-5000:   Very fast, for bright conditions or moving subjects
  - 5000-15000:  Normal daylight conditions
  - 15000-30000: Indoor or overcast conditions
  - 30000+:      Low light conditions

  white_balance:
  - 0.5-0.8:     Warm (tungsten/incandescent lighting)
  - 0.8-1.2:     Neutral (daylight)
  - 1.2-2.0:     Cool (shade/cloudy conditions)

  Usage patterns:
  1. Start with auto mode: SetSensorColorControl(true, 0, 0)
  2. Switch to manual for specific conditions
  3. Adjust exposure based on lighting
  4. Adjust white balance based on light source type
  */

  bool auto_mode = true;
  float exposure_time = 0;  // Ignored in auto mode
  float white_balance = 0;  // Ignored in auto mode

  // bool auto_mode = false;
  // float exposure_time = 15000.0f;  // 15ms exposure time (in microseconds)
  // float white_balance = 1.8f;      // Slightly cool white balance (>1.0 = cooler, <1.0 = warmer)

  if (!rpi_sensor.SetSensorColorControl(auto_mode, exposure_time, white_balance)) {
    std::cerr << "Failed to set auto color control" << std::endl;
    return -1;
  }

  // Start video capture
  std::cout << "Starting capture ..." << std::endl;
  if (rpi_sensor.Start() != 0) {
    return -1;
  }

  std::cout << "Camera ready. Displaying video stream..." << std::endl;
  std::cout << "Press 'q' or ESC to quit" << std::endl;

  // Check if running over SSH
  const char* ssh_client = std::getenv("SSH_CLIENT");
  const char* ssh_connection = std::getenv("SSH_CONNECTION");
  if (ssh_client || ssh_connection) {
    std::cout << "WARNING: Running over SSH detected!" << std::endl;
    std::cout << "This may limit window sizing capabilities." << std::endl;
    std::cout << "Try running directly on Pi desktop for full control." << std::endl;
  }

  // Create OpenCV window with normal sizing (not auto-resize)
  cv::namedWindow("Pi Camera Stream", cv::WINDOW_NORMAL);

  // Force window to be resizable (sometimes helps on Pi)
  cv::setWindowProperty("Pi Camera Stream", cv::WND_PROP_ASPECT_RATIO, cv::WINDOW_FREERATIO);

  // Constants for window sizing
  const int kFixedWindowWidth = 800;

  // Variables for window sizing calculation
  int display_width = kFixedWindowWidth;
  int display_height = 600;
  bool window_size_set = false;

  // Capture and display frames
  int frame_count = 0;
  auto start_time = std::chrono::steady_clock::now();

  while (true) {
    std::vector<uint8_t> frame_data = rpi_sensor.GetFrame();

    if (!frame_data.empty()) {
      frame_count++;

      // Get frame dimensions
      Size resolution = rpi_sensor.GetResolution();
      int width = resolution.width;
      int height = resolution.height;
      int bytes_per_pixel = rpi_sensor.GetBytesPerPixel();

      std::cout << "Frame " << frame_count << " captured, size: "
                << frame_data.size() << " bytes, bytes per pixel: "
                << bytes_per_pixel << std::endl;

      // Create OpenCV Mat from the frame data
      cv::Mat frame;
      if (rpi_sensor.GetPixelFormat() == formats::RGB888) {
        cv::Mat rgb_frame(height, width, CV_8UC3, frame_data.data());
        frame = rgb_frame.clone();
      } else if (rpi_sensor.GetPixelFormat() == formats::BGR888) {
        cv::Mat bgr_frame(height, width, CV_8UC3, frame_data.data());
        cv::cvtColor(bgr_frame, frame, cv::COLOR_BGR2RGB);
      } else if (rpi_sensor.GetPixelFormat() == formats::YUYV) {
        // YUYV format - convert to BGR for OpenCV
        cv::Mat yuyv_frame(height, width, CV_8UC2, frame_data.data());
        cv::cvtColor(yuyv_frame, frame, cv::COLOR_YUV2BGR_YUYV);
      } else if (rpi_sensor.GetPixelFormat() == formats::NV12) {
        // NV12 format - convert to BGR for OpenCV
        cv::Mat nv12_frame(height * 3 / 2, width, CV_8UC1, frame_data.data());
        // cv::Mat y_plane = nv12_frame(cv::Rect(0, 0, width, height));
        // cv::Mat uv_plane = nv12_frame(cv::Rect(0, height, width, height / 2));
        cv::cvtColor(nv12_frame, frame, cv::COLOR_YUV2BGR_NV12);
      } else {
        // Fallback - treat as grayscale or custom format
        frame = cv::Mat(height, width, CV_8UC(bytes_per_pixel), frame_data.data()).clone();
      }

      // Calculate display dimensions based on aspect ratio (only once)
      if (!window_size_set) {
        double aspect_ratio = static_cast<double>(frame.rows) / static_cast<double>(frame.cols);
        display_height = static_cast<int>(display_width * aspect_ratio);

        // CRITICAL: Display the frame FIRST, then resize
        // This ensures the window is properly realized before resizing
        cv::Mat temp_frame;
        cv::resize(frame, temp_frame, cv::Size(display_width, display_height));
        cv::imshow("Pi Camera Stream", temp_frame);
        cv::waitKey(100);  // Allow window to be created and displayed

        // Get initial window size after display
        cv::Rect initial_rect = cv::getWindowImageRect("Pi Camera Stream");
        std::cout << "Initial window size after display: " << initial_rect.width << "x" << initial_rect.height
                  << std::endl;

        // Now resize with window manager decoration compensation
        // Add typical decoration sizes to compensate for window manager
        int compensation_width = 64;   // Typical horizontal decorations
        int compensation_height = 48;  // Typical vertical decorations (title bar, etc.)

        int adjusted_width = display_width + compensation_width;
        int adjusted_height = display_height + compensation_height;

        std::cout << "Attempting resize with compensation: " << adjusted_width << "x" << adjusted_height << std::endl;
        cv::resizeWindow("Pi Camera Stream", adjusted_width, adjusted_height);
        cv::waitKey(200);

        // Check final result
        cv::Rect final_rect = cv::getWindowImageRect("Pi Camera Stream");
        std::cout << "Final window size: " << final_rect.width << "x" << final_rect.height << std::endl;

        // Update our target dimensions to match actual result
        display_width = final_rect.width;
        display_height = final_rect.height;

        if (abs(final_rect.width - kFixedWindowWidth) < 100) {
          std::cout << "Window resize successful!" << std::endl;
        } else {
          std::cout << "Window size constrained, using actual size: " << display_width << "x" << display_height
                    << std::endl;
        }
        std::cout << "========================\n" << std::endl;

        window_size_set = true;
      }

      // Resize frame to fit the fixed window dimensions
      cv::Mat resized_frame;
      cv::resize(frame, resized_frame, cv::Size(display_width, display_height));

      auto current_time = std::chrono::steady_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
      double fps = (frame_count * 1000.0) / duration.count();
      std::string info = "Frame: " + std::to_string(frame_count) + " | FPS: " + std::to_string(fps).substr(0, 5);
      cv::putText(resized_frame, info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

      // // // Display the resized frame
      cv::imshow("Pi Camera Stream", resized_frame);

      // Check for exit condition
      int key = cv::waitKey(1) & 0xFF;
      if (key == 'q' || key == 27) {  // 'q' or ESC key
        break;
      }

      // Print periodic status
      if (frame_count % 100 == 0) {
        std::cout << info << std::endl;
      }

    } else {
      std::cerr << "Failed to get frame, retrying..." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  auto end_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

  std::cout << "Displayed " << frame_count << " frames in " << duration.count() << "ms" << std::endl;
  std::cout << "Average FPS: " << (frame_count * 1000.0) / duration.count() << std::endl;

  // Cleanup
  cv::destroyAllWindows();
  rpi_sensor.Stop();

  return 0;
}