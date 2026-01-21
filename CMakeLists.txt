cmake_minimum_required(VERSION 3.16)
project(determsensor VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add compiler flags for better error reporting and optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/bin)

# Find required packages
find_package(PkgConfig REQUIRED)

# Find libcamera using pkg-config
pkg_check_modules(LIBCAMERA REQUIRED libcamera)
if(NOT LIBCAMERA_FOUND)
    message(FATAL_ERROR "libcamera not found. Please install libcamera-dev package.")
endif()
message(STATUS "Found libcamera: ${LIBCAMERA_VERSION}")

# # Find RealSense SDK
# find_package(realsense2 REQUIRED)
# if(NOT realsense2_FOUND)
#     message(FATAL_ERROR "librealsense2 not found. Please install librealsense2-dev package.")
# endif()
# message(STATUS "Found RealSense: ${realsense2_VERSION}")

# Find OpenCV package
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found. Please install libopencv-dev package.")
endif()
message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

# Find OpenGL and GLUT packages
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
if(NOT OPENGL_FOUND)
    message(FATAL_ERROR "OpenGL not found. Please install libgl1-mesa-dev package.")
endif()
if(NOT GLUT_FOUND)
    message(FATAL_ERROR "GLUT not found. Please install freeglut3-dev package.")
endif()

# Find threading library
find_package(Threads REQUIRED)

# Add thirdparty dependencies
include(3rdparty/add_thirdparty.cmake)

# Enhanced synchronization system features
message(STATUS "")
message(STATUS "Enhanced Camera Synchronization Configuration:")
message(STATUS "  Hardware timestamping: DS1307 RTC + RealSense")
message(STATUS "  Enhanced sync algorithm: Enabled")
message(STATUS "  Mixed timestamp support: Enabled")
message(STATUS "")

add_subdirectory(source)
add_subdirectory(example)