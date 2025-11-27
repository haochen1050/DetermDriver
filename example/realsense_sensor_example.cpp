/**
 * @file realsense_sensor_example.cpp
 * @brief Example usage of RealsenseSensor class with OpenCV visualization
 */

#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "sensor/realsense_d421_sensor.h"

// Function declarations
void SaveCurrentFrames(const std::map<std::string, std::vector<uint8_t>>& frame_data, int frame_number, int width,
                       int height);

int main(int argc, char* argv[])
{
  determ::sensor::RealsenseSensor rs_sensor;

  // Initialize the sensor
  std::cout << "=== Initializing RealSense Sensor ===" << std::endl;
  int ret = rs_sensor.Initialize();
  if (ret != 0) {
    std::cerr << "Failed to initialize sensor" << std::endl;
    return -1;
  }

  // Print capabilities
  std::cout << "\n=== Sensor Capabilities ===" << std::endl;
  rs_sensor.PrintSupportedFormats();
  rs_sensor.PrintAvailableControls();

  // Configure sensor for depth and infrared streams
  std::cout << "\n=== Configuring Sensor ===" << std::endl;
  std::vector<std::string> streams;
  if (argc > 1) {
    // Use command line arguments for streams
    for (int i = 1; i < argc; ++i) {
      streams.push_back(argv[i]);
    }
  } else {
    // Default configuration: depth and both infrared streams
    streams = {"depth", "infrared1", "infrared2"};
  }
  ret = rs_sensor.SetSensorConfig(640, 480, streams);
  if (ret != 0) {
    std::cerr << "Failed to configure sensor" << std::endl;
    return -1;
  }

  // Set sensor controls
  std::cout << "\n=== Setting Sensor Controls ===" << std::endl;
  rs_sensor.SetDepthControl(true);     // Auto exposure
  rs_sensor.SetInfraredControl(true);  // Auto exposure for IR
  rs_sensor.SetLaserPower(240.0f);     // Set laser power to 240mW
  rs_sensor.SetDepthUnits(0.001f);     // 1mm per unit

  // Start capturing
  std::cout << "\n=== Starting Capture ===" << std::endl;
  ret = rs_sensor.Start();
  if (ret != 0) {
    std::cerr << "Failed to start sensor" << std::endl;
    return -1;
  }

  // Get resolution for OpenCV Mat creation
  auto [width, height] = rs_sensor.GetResolution();
  std::cout << "Resolution: " << width << "x" << height << std::endl;

  // Create OpenCV windows
  cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Infrared 1", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Infrared 2", cv::WINDOW_AUTOSIZE);

  // Position windows side by side
  cv::moveWindow("Depth", 0, 0);
  cv::moveWindow("Infrared 1", width + 50, 0);
  cv::moveWindow("Infrared 2", 2 * (width + 50), 0);

  std::cout << "\n=== Starting Video Stream ===" << std::endl;
  std::cout << "Press 'q' or ESC to quit, 's' to save frames" << std::endl;

  // Performance monitoring
  auto start_time = std::chrono::steady_clock::now();
  int frame_count = 0;
  bool running = true;

  std::string depth_info;

  while (running) {
    // Capture frame
    auto frame_data = rs_sensor.GetFrame();

    if (frame_data.empty()) {
      std::cerr << "Failed to get frame data" << std::endl;
      continue;
    }

    // Process and display each stream
    for (const auto& [stream_name, data] : frame_data) {
      if (data.empty()) continue;

      cv::Mat display_mat;

      if (stream_name == "depth") {
        // Convert 16-bit depth data to OpenCV Mat
        cv::Mat depth_mat(height, width, CV_16UC1, reinterpret_cast<void*>(const_cast<uint8_t*>(data.data())));

        // Create visualization of depth data
        cv::Mat depth_8bit;

        // Scale to 0-255 range, ignoring invalid depth (0)
        cv::Mat valid_mask = depth_mat > 0;
        depth_mat.convertTo(depth_8bit, CV_8UC1, 255.0 / (7000 - 300), -300 * 255.0 / (7000 - 300));
        depth_8bit.setTo(0, ~valid_mask);  // Set invalid pixels to black
        // Apply colormap for better visualization
        cv::applyColorMap(depth_8bit, display_mat, cv::COLORMAP_JET);

        auto current_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        double fps = (frame_count * 1000.0) / duration.count();

        depth_info = "Depth Camera: " + std::to_string(frame_count) + " | FPS: " + std::to_string(fps).substr(0, 5);
        cv::putText(display_mat, depth_info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0),
                    2);

        cv::imshow("Depth", display_mat);

      } else if (stream_name == "infrared1" || stream_name == "infrared2") {
        // Convert 8-bit infrared data to OpenCV Mat
        cv::Mat ir_mat(height, width, CV_8UC1, const_cast<uint8_t*>(data.data()));

        // Apply histogram equalization for better contrast
        cv::Mat ir_equalized;
        cv::equalizeHist(ir_mat, ir_equalized);

        // Convert to 3-channel for consistent display
        cv::cvtColor(ir_equalized, display_mat, cv::COLOR_GRAY2BGR);

        // Add stream label
        std::string label = (stream_name == "infrared1") ? "IR Camera 1" : "IR Camera 2";
        cv::putText(display_mat, label, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        std::string window_name = (stream_name == "infrared1") ? "Infrared 1" : "Infrared 2";
        cv::imshow(window_name, display_mat);
      }
    }

    // Handle keyboard input
    int key = cv::waitKey(1) & 0xFF;

    if (key == 'q' || key == 27) {  // 'q' or ESC key
      running = false;

    } else if (key == 's') {  // 's' key to save current frames
      SaveCurrentFrames(frame_data, frame_count, width, height);
      std::cout << "Saved frames " << frame_count << std::endl;
    }

    frame_count++;

    // Display FPS every 30 frames
    if (frame_count % 30 == 0) {
      std::cout << depth_info << std::endl;
    }
  }

  // Cleanup
  std::cout << "\n=== Stopping Capture ===" << std::endl;
  cv::destroyAllWindows();
  rs_sensor.Stop();

  std::cout << "RealSense visualization completed successfully" << std::endl;
  std::cout << "Total frames processed: " << frame_count << std::endl;
  return 0;
}

// Function to save current frames to files
void SaveCurrentFrames(const std::map<std::string, std::vector<uint8_t>>& frame_data, int frame_number, int width,
                       int height)
{
  for (const auto& [stream_name, data] : frame_data) {
    if (data.empty()) continue;

    std::string timestamp = std::to_string(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
            .count());

    if (stream_name == "depth") {
      // Save depth as both raw data and colorized image
      std::string raw_filename = "depth_" + timestamp + "_frame_" + std::to_string(frame_number) + ".raw";
      std::string img_filename = "depth_" + timestamp + "_frame_" + std::to_string(frame_number) + ".png";

      // Save raw 16-bit depth data
      std::ofstream raw_file(raw_filename, std::ios::binary);
      raw_file.write(reinterpret_cast<const char*>(data.data()), data.size());
      raw_file.close();

      // Save colorized depth image
      cv::Mat depth_mat(height, width, CV_16UC1, reinterpret_cast<void*>(const_cast<uint8_t*>(data.data())));
      cv::Mat depth_8bit, depth_colored;

      cv::Mat valid_mask = depth_mat > 0;
      depth_mat.convertTo(depth_8bit, CV_8UC1, 255.0 / (7000 - 300), -300 * 255.0 / (7000 - 300));
      depth_8bit.setTo(0, ~valid_mask);
      cv::applyColorMap(depth_8bit, depth_colored, cv::COLORMAP_JET);
      cv::imwrite(img_filename, depth_colored);

    } else if (stream_name.find("infrared") != std::string::npos) {
      std::string filename = stream_name + "_" + timestamp + "_frame_" + std::to_string(frame_number) + ".png";

      // Save infrared as PNG
      cv::Mat ir_mat(height, width, CV_8UC1, const_cast<uint8_t*>(data.data()));
      cv::imwrite(filename, ir_mat);
    }
  }
}

// Example of how to save frames to files
void SaveFramesToFiles(const std::map<std::string, std::vector<uint8_t>>& frame_data, int frame_number, int width,
                       int height)
{
  for (const auto& [stream_name, data] : frame_data) {
    std::string filename = stream_name + "_frame_" + std::to_string(frame_number);

    if (stream_name == "depth") {
      filename += ".raw";  // Save as raw 16-bit data
      std::ofstream file(filename, std::ios::binary);
      file.write(reinterpret_cast<const char*>(data.data()), data.size());
      file.close();

    } else if (stream_name.find("infrared") != std::string::npos) {
      filename += ".pgm";  // Save as PGM (grayscale image format)
      std::ofstream file(filename);
      file << "P5\n" << width << " " << height << "\n255\n";
      file.write(reinterpret_cast<const char*>(data.data()), data.size());
      file.close();
    }

    std::cout << "Saved " << stream_name << " frame to " << filename << std::endl;
  }
}
