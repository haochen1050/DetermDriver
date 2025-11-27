// /**
//  * @file determ_sensor_example.cpp
//  * @brief Example usage of both RaspberryCamera and RealsenseSensor class with OpenCV visualization
//  */

#include <signal.h>

#include <chrono>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "sensor/determ_sensor.h"

using namespace determ::capture;

// Global flag for graceful shutdown
volatile bool g_running = true;

void SignalHandler(int signum)
{
  std::cout << "\nInterrupt signal (" << signum << ") received. Shutting down gracefully..." << std::endl;
  g_running = false;
}

cv::Mat ConvertToOpenCv(const std::vector<uint8_t>& data, int width, int height, const std::string& format)
{
  if (data.empty()) {
    return cv::Mat::zeros(height, width, CV_8UC3);
  }

  cv::Mat result;

  if (format == "RGB888" || format == "RGB") {
    // RGB data - convert to BGR for OpenCV
    cv::Mat rgb_mat(height, width, CV_8UC3, const_cast<uint8_t*>(data.data()));
    // cv::cvtColor(rgb_mat, result, cv::COLOR_RGB2BGR);
    result = rgb_mat.clone();  // Direct copy, OpenCV handles RGB as BGR internally
  } else if (format == "BGR888" || format == "BGR") {
    // BGR data - direct copy
    result = cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(data.data())).clone();
  } else if (format == "depth") {
    // 16-bit depth data - convert to 8-bit for visualization
    cv::Mat depth_16(height, width, CV_16UC1, const_cast<uint8_t*>(data.data()));

    depth_16.convertTo(result, CV_8UC1, 255.0 / (7000 - 300), -300 * 255.0 / (7000 - 300));

    // Convert to 3-channel for consistency
    cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);

    // Apply colormap for better depth visualization
    cv::Mat depth_colored;
    cv::cvtColor(result, depth_colored, cv::COLOR_BGR2GRAY);
    cv::applyColorMap(depth_colored, result, cv::COLORMAP_JET);
  } else if (format == "infrared") {
    // 8-bit infrared data
    cv::Mat ir_mat(height, width, CV_8UC1, const_cast<uint8_t*>(data.data()));
    cv::cvtColor(ir_mat, result, cv::COLOR_GRAY2BGR);
  } else {
    // Unknown format - create empty image
    result = cv::Mat::zeros(height, width, CV_8UC3);
  }

  return result;
}

cv::Mat CreateGridDisplay(const std::map<std::string, cv::Mat>& images, const cv::Size& grid_size)
{
  // Calculate individual image size for the grid
  int img_width = grid_size.width / 2;
  int img_height = grid_size.height / 2;

  // Create the final grid image
  cv::Mat grid_image = cv::Mat::zeros(grid_size.height, grid_size.width, CV_8UC3);

  // Helper function to resize and place image
  auto place_image = [&](const cv::Mat& img, int x, int y, const std::string& label) {
    cv::Mat resized_img;
    if (!img.empty()) {
      cv::resize(img, resized_img, cv::Size(img_width, img_height));
    } else {
      resized_img = cv::Mat::zeros(img_height, img_width, CV_8UC3);
    }

    // Copy to grid position
    cv::Rect roi(x * img_width, y * img_height, img_width, img_height);
    resized_img.copyTo(grid_image(roi));

    // Add label
    cv::putText(grid_image, label, cv::Point(x * img_width + 10, y * img_height + 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(255, 255, 255), 2);
  };

  // Place images in 2x2 grid
  // Top-left: Color
  auto color_it = images.find("color");
  place_image(color_it != images.end() ? color_it->second : cv::Mat(), 0, 0, "Color");

  // Top-right: Depth
  auto depth_it = images.find("depth");
  place_image(depth_it != images.end() ? depth_it->second : cv::Mat(), 1, 0, "Depth");

  // Bottom-left: Infrared 1
  auto ir1_it = images.find("infrared1");
  place_image(ir1_it != images.end() ? ir1_it->second : cv::Mat(), 0, 1, "Infrared 1");

  // Bottom-right: Infrared 2
  auto ir2_it = images.find("infrared2");
  place_image(ir2_it != images.end() ? ir2_it->second : cv::Mat(), 1, 1, "Infrared 2");

  return grid_image;
}

cv::Mat CreateSideBySideDisplay(const std::map<std::string, cv::Mat>& images, const cv::Size& grid_size)
{
  // Calculate individual image size for the grid
  int img_width = grid_size.width / 2;
  int img_height = grid_size.height / 2;

  // Create the final grid image
  cv::Mat grid_image = cv::Mat::zeros(grid_size.height, grid_size.width, CV_8UC3);

  // Helper function to resize and place image
  auto place_image = [&](const cv::Mat& img, int x, int y, const std::string& label) {
    cv::Mat resized_img;
    if (!img.empty()) {
      cv::resize(img, resized_img, cv::Size(img_width, img_height));
    } else {
      resized_img = cv::Mat::zeros(img_height, img_width, CV_8UC3);
    }

    // Copy to grid position
    cv::Rect roi(x * img_width, y * img_height, img_width, img_height);
    resized_img.copyTo(grid_image(roi));

    // Add label
    cv::putText(grid_image, label, cv::Point(x * img_width + 10, y * img_height + 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(255, 255, 255), 2);
  };

  // Place images in side by side
  // Top-left: Color
  auto color_it = images.find("color");
  place_image(color_it != images.end() ? color_it->second : cv::Mat(), 0, 0, "Color");

  // Top-right: Depth
  auto depth_it = images.find("depth");
  place_image(depth_it != images.end() ? depth_it->second : cv::Mat(), 1, 0, "Depth");

  return grid_image;
}

int main(int argc, char* argv[])
{
  int display_mode = 0;
  if (argc > 1) {
    try {
      display_mode = std::stoi(argv[1]);
    } catch (const std::exception& e) {
      std::cerr << "Invalid display mode argument. Using default (0)." << std::endl;
    }
  }

  if (display_mode > 1) {
    std::cout << "Display mode 0 : show all the streams as square grid " << std::endl;
    std::cout << "Display mode 1 : show color and depth side by side " << std::endl;
    std::cout << "Now display mode is not 0 or 1. Set to default as 0 " << std::endl;
    display_mode = 0;
  }

  // Set up signal handler for graceful shutdown
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  std::cout << "=== DetermSensor Multi-Stream Demo ===" << std::endl;

  // Create DetermSensor instance
  DetermSensor sensor;

  // Initialize the sensor
  std::cout << "\n1. Initializing sensors..." << std::endl;
  if (sensor.Initialize() != 0) {
    std::cerr << "Failed to initialize sensors!" << std::endl;
    return -1;
  }

  // Print capabilities
  std::cout << "\n2. Sensor capabilities:" << std::endl;
  sensor.PrintSupportedFormats();

  // Enable all streams
  std::cout << "\n3. Enabling all streams..." << std::endl;
  std::vector<std::string> streams;
  if (display_mode == 0) {
    streams = {"color", "depth", "infrared1", "infrared2"};
  } else {
    streams = {"color", "depth"};
  }
  if (sensor.EnableStream(streams) != 0) {
    std::cerr << "Failed to enable streams!" << std::endl;
    return -1;
  }

  // Configure color sensor (1280x720 RGB)
  std::cout << "\n4. Configuring color sensor..." << std::endl;
  if (sensor.SetColorSensorConfig(1280, 720, "RGB888") != 0) {
    std::cerr << "Failed to configure color sensor!" << std::endl;
    return -1;
  }

  // Configure depth sensor (640x480)
  std::cout << "\n5. Configuring depth sensor..." << std::endl;
  if (sensor.SetDepthSensorConfig(640, 480) != 0) {
    std::cerr << "Failed to configure depth sensor!" << std::endl;
    return -1;
  }

  // Set auto controls
  std::cout << "\n6. Setting automatic controls..." << std::endl;

  // Auto exposure and white balance for color sensor
  sensor.SetColorSensorControl(true);  // Auto mode
  sensor.SetColorFocusControl(true);   // Auto focus

  // Auto exposure for depth and infrared sensors
  sensor.SetDepthSensorControl(true);     // Auto exposure for depth
  sensor.SetInfraredSensorControl(true);  // Auto exposure for infrared

  // Optional: Set laser power for better depth sensing
  sensor.SetLaserPower(150.0f);  // 150mW laser power

  // Start capture
  std::cout << "\n7. Starting capture..." << std::endl;
  if (sensor.Start() != 0) {
    std::cerr << "Failed to start sensors!" << std::endl;
    return -1;
  }

  // Create OpenCV window
  const std::string kWindowName = "DetermSensor - Multi-Stream View";
  cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);

  // Display grid size
  const cv::Size kGridSize(1280, 960);  // 2x2 grid, each cell 640x480

  std::cout << "\n8. Starting live display..." << std::endl;
  std::cout << "Press 'q' or 'ESC' to quit, 'f' to trigger autofocus" << std::endl;

  // Performance monitoring
  auto last_time = std::chrono::steady_clock::now();
  int frame_count = 0;
  double fps = 0.0;

  // Main capture loop
  while (g_running) {
    auto start_time = std::chrono::steady_clock::now();

    // Get frames from all streams
    auto frame_data = sensor.GetFrame();

    if (frame_data.empty()) {
      std::cerr << "No frame data received" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      continue;
    }

    // Convert frames to OpenCV format
    std::map<std::string, cv::Mat> opencv_images;

    // Convert color frame
    if (frame_data.find("color") != frame_data.end()) {
      opencv_images["color"] = ConvertToOpenCv(frame_data["color"], 1280, 720, "RGB888");
    }

    // Convert depth frame
    if (frame_data.find("depth") != frame_data.end()) {
      opencv_images["depth"] = ConvertToOpenCv(frame_data["depth"], 640, 480, "depth");
    }

    // Convert infrared frames
    if (frame_data.find("infrared1") != frame_data.end()) {
      opencv_images["infrared1"] = ConvertToOpenCv(frame_data["infrared1"], 640, 480, "infrared");
    }

    if (frame_data.find("infrared2") != frame_data.end()) {
      opencv_images["infrared2"] = ConvertToOpenCv(frame_data["infrared2"], 640, 480, "infrared");
    }

    // Add FPS information
    std::string fps_text = "FPS: " + std::to_string(fps).substr(0, 5);

    if (display_mode == 0) {
      // Create grid display
      cv::Mat grid_display = CreateGridDisplay(opencv_images, kGridSize);

      cv::putText(grid_display, fps_text, cv::Point(kGridSize.width - 150, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                  cv::Scalar(0, 255, 0), 2);

      // Show the grid
      cv::imshow(kWindowName, grid_display);
    } else {
      cv::Mat side_by_side_display = CreateSideBySideDisplay(opencv_images, kGridSize);

      cv::putText(side_by_side_display, fps_text, cv::Point(kGridSize.width - 150, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                  cv::Scalar(0, 255, 0), 2);

      // Show the grid
      cv::imshow(kWindowName, side_by_side_display);
    }

    // Handle keyboard input
    int key = cv::waitKey(1) & 0xFF;
    if (key == 'q' || key == 27) {  // 'q' or ESC
      std::cout << "User requested quit" << std::endl;
      break;
    } else if (key == 'f') {  // 'f' for focus
      std::cout << "Triggering autofocus..." << std::endl;
      sensor.TriggerColorAutofocus();
    } else if (key == 'r') {  // 'r' for refresh focus
      std::cout << "Refreshing autofocus..." << std::endl;
      sensor.RefreshColorAutofocus();
    }

    // Calculate FPS
    frame_count++;
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_time).count();

    if (elapsed >= 1) {  // Update FPS every second
      fps = static_cast<double>(frame_count) / elapsed;
      frame_count = 0;
      last_time = current_time;

      // Print status every 5 seconds
      static int status_counter = 0;
      status_counter++;
      if (status_counter >= 5) {
        std::cout << "Status: Running at " << fps << " FPS, Streams: " << frame_data.size() << "/" << streams.size()
                  << std::endl;
        status_counter = 0;
      }
    }

    // Small delay to prevent excessive CPU usage
    auto frame_end_time = std::chrono::steady_clock::now();
    auto frame_duration = std::chrono::duration_cast<std::chrono::milliseconds>(frame_end_time - start_time);

    // Target ~30 FPS (33ms per frame)
    const int kTargetFrameTime = 33;
    if (frame_duration.count() < kTargetFrameTime) {
      std::this_thread::sleep_for(std::chrono::milliseconds(kTargetFrameTime - frame_duration.count()));
    }
  }

  // Cleanup
  std::cout << "\n9. Stopping sensors..." << std::endl;
  sensor.Stop();

  cv::destroyAllWindows();

  std::cout << "DetermSensor demo completed successfully!" << std::endl;
  std::cout << "\nFinal Statistics:" << std::endl;
  std::cout << "  Color sensor info: " << sensor.GetColorSensorInfo() << std::endl;
  std::cout << "  Depth sensor info: " << sensor.GetDepthSensorInfo() << std::endl;

  return 0;
}