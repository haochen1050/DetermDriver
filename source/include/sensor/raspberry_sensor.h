/**
 * @file raspberry_sensor.h
 * @author Lei Gao (lei.gao@determtech.com)
 * @brief Raspberry Sensor driver for Determ Sensor Set
 * @version 1.0.0
 * @date 2025-07-03
 *
 * @copyright Copyright (c) 2025 Determ Inc. All rights reserved
 *
 */
#pragma once

#include <fcntl.h>
#include <libcamera/camera_manager.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/libcamera.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>

using namespace libcamera;

namespace determ
{
  namespace sensor
  {
    struct TimestampedRaspberryFrame {
      std::vector<uint8_t> data;
      std::chrono::high_resolution_clock::time_point timestamp;
      uint64_t sequence_number;
      
      uint64_t getTimestampUs() const {
          return std::chrono::duration_cast<std::chrono::microseconds>(
              timestamp.time_since_epoch()).count();
      }
      
      double getTimestampMs() const {
          return std::chrono::duration_cast<std::chrono::microseconds>(
              timestamp.time_since_epoch()).count() / 1000.0;
      }
    };

    class RaspberrySensor
    {
     public:
      RaspberrySensor();
      ~RaspberrySensor();

      // Initialize the sensor system
      int Initialize();

      // Set sensor sensor configuration (resolution and format)
      int SetSensorConfig(int width, int height, const std::string& format);

      // Set sensor color control (exposure and white balance)
      bool SetSensorColorControl(bool is_auto, float exposure_time = 0.0f, float white_balance = 0.0f);

      // Start video capture
      int Start();

      // Stop video capture
      int Stop();

      // Get a frame (blocking call that returns raw frame data)
      std::vector<uint8_t> GetFrame();
      TimestampedRaspberryFrame GetTimestampedFrame();

      // Print all supported resolutions and formats
      void PrintSupportedFormats();

      // Print available sensor controls
      void PrintAvailableControls();

      bool SetFocusControl(bool auto_focus, float manual_focus_position = 0.0f);
      bool TriggerAutofocus();
      bool RefreshAutofocus();
      void PrintFocusControls();

      // Helper methods
      Size GetResolution() const { return m_resolution; }
      PixelFormat GetPixelFormat() const { return m_pixel_format; }
      int GetBytesPerPixel() const { return m_bytes_per_pixel; }



     private:
      int SetupBuffers();
      float GetBytesPerPixel(const PixelFormat& format);
      size_t GetFrameSize(const PixelFormat& format, const Size& resolution);
      PixelFormat StringToPixelFormat(const std::string& format);

      int ApplyControlsToRequest(Request* request, const ControlList& controls);
      void CalculateWhiteBalanceGains(float white_balance, float& red_gain, float& blue_gain);

      std::vector<uint8_t> ExtractFrameData(FrameBuffer *frame_buffer);

     private:
      std::unique_ptr<CameraManager> m_cm;
      std::shared_ptr<Camera> m_camera;
      std::unique_ptr<CameraConfiguration> m_config;
      std::unique_ptr<FrameBufferAllocator> m_allocator;
      std::vector<std::unique_ptr<Request>> m_requests;
      Stream* m_stream;
      uint64_t m_frame_sequence = 0;

      // Video streaming members
      bool m_is_running;
      size_t m_current_buffer_index;

      // Sensor configuration
      Size m_resolution;
      PixelFormat m_pixel_format;
      int m_bytes_per_pixel;

      // Color control settings
      ControlList m_current_controls;
      bool m_auto_mode;
      float m_manual_exposure_time;
      float m_manual_white_balance;

      // Focus control
      bool m_auto_focus_enabled;
      float m_manual_focus_position;
    };
  }  // namespace sensor
}  // namespace determ