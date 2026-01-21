

macro(determ_sensor_app APP_NAME)
    add_executable(${APP_NAME})
    
    target_sources(${APP_NAME} PRIVATE "${APP_NAME}.cpp")
    
    # Link all required libraries
    target_link_libraries(${APP_NAME} PUBLIC
        determsensor
        ${OpenCV_LIBS}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        Threads::Threads
        rt  # For RTC access in enhanced examples
    )
    
    # Include directories
    target_include_directories(${APP_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../source/include
        ${OpenCV_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
    )
    
    # Add compiler definitions for enhanced features
    target_compile_definitions(${APP_NAME} PRIVATE
        -DENHANCED_SYNCHRONIZATION=1
        -DHARDWARE_TIMESTAMP_SUPPORT=1
    )
    
    set_target_properties(${APP_NAME} PROPERTIES
        FOLDER "example/"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
        INSTALL_RPATH_USE_LINK_PATH TRUE
    )
    
    get_target_property(CURRENT_INSTALL_RPATH ${APP_NAME} INSTALL_RPATH)
    list(APPEND CURRENT_INSTALL_RPATH "$ORIGIN")
    set_target_properties(${APP_NAME} PROPERTIES INSTALL_RPATH "${CURRENT_INSTALL_RPATH}")
endmacro()

# Enhanced synchronized camera examples
# determ_sensor_app(enhanced_sync_depth_color_viewer)
# determ_sensor_app(enhanced_sync_example)
# determ_sensor_app(hw_timestamp_test)
# determ_sensor_app(sync_benchmark)

# Original examples (maintained for compatibility)
determ_sensor_app(raspberry_sensor_example)
determ_sensor_app(realsense_sensor_example)
# determ_sensor_app(determ_sensor_example)
# determ_sensor_app(pointcloud_viewer)
# determ_sensor_app(pointcloud_viewer_org)
# determ_sensor_app(depth_color_viewer)
# determ_sensor_app(depth_color_viewer_org)
# determ_sensor_app(latency_test)
# determ_sensor_app(realsense_offset_test)
# determ_sensor_app(test_calibration_loader)
# determ_sensor_app(simple_sync_visualizer)
# determ_sensor_app(sync_comparison_demo)
determ_sensor_app(advanced_buffered_sync_example)

# Add the existing synchronized camera example (if it exists)
# if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/syn_camera_example.cpp")
#     determ_sensor_app(syn_camera_example)
# endif()

