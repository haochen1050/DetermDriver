# Enhanced DetermSensor library with hardware timestamp support

# Collect all source files
file(GLOB_RECURSE SENSOR_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

# Collect all header files
file(GLOB_RECURSE SENSOR_HEADERS 
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
)

# Create the enhanced sensor library
add_library(${PROJECT_NAME} SHARED ${SENSOR_SOURCES} ${SENSOR_HEADERS})

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${LIBCAMERA_INCLUDE_DIRS}
    ${realsense2_INCLUDE_DIR}
)

# Link directories
target_link_directories(${PROJECT_NAME} PUBLIC
    ${LIBCAMERA_LIBRARY_DIRS}
    ${realsense2_LIBRARY_DIR}
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PUBLIC
    ${LIBCAMERA_LIBRARIES}
    ${realsense2_LIBRARY}
    Threads::Threads
    rt  # Real-time library for RTC access
)

# Add compiler definitions for enhanced features
target_compile_definitions(${PROJECT_NAME} PRIVATE
    -DENHANCED_SYNCHRONIZATION=1
    -DHARDWARE_TIMESTAMP_SUPPORT=1
)

# Set library properties
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    OUTPUT_NAME_DEBUG "${PROJECT_NAME}d"
    OUTPUT_NAME_RELEASE "${PROJECT_NAME}"
    PUBLIC_HEADER "${SENSOR_HEADERS}"
)

# Installation
include(GNUInstallDirs)

# Install library
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/sensor
)

message(STATUS "Enhanced DetermSensor library configured:")
message(STATUS "  Sources: ${SENSOR_SOURCES}")
message(STATUS "  Headers: ${SENSOR_HEADERS}")
message(STATUS "  Hardware timestamp support: Enabled")
message(STATUS "  Enhanced synchronization: Enabled")