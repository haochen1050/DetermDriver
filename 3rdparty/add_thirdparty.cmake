# third-party library integration
set(THIRDPARTY_DIR "${CMAKE_CURRENT_LIST_DIR}")

function(determ_import_3rdparty_library name)
    # cmake_parse_arguments(arg "PUBLIC;HEADER;INCLUDE_ALL" "INCLUDE_DIR;LIB_DIR;SHARED_LIB_DIR;MODULE" "LIBRARIES;SHARED_LIBRARIES" ${ARGN})
    cmake_parse_arguments(arg "" "INCLUDE_DIR;LIB_DIR;BIN_DIR;MODULE" "STATIC_LIBRARIES;SHARED_LIBRARIES;THIRD_LIBRARIES" ${ARGN})

    if(arg_UNPARSED_ARGUMENTS)
        message(STATUS "Unparsed: ${arg_UNPARSED_ARGUMENTS}")
        message(FATAL_ERROR "Invalid syntax: determ_import_3rdparty_library(${name} ${ARGN})")
    endif()
    add_library(${name} INTERFACE)

    # link and install the include directory
    if(arg_INCLUDE_DIR)
        set (incl_path ${arg_INCLUDE_DIR})
        
        # check if the include directory ends with '/'
        if (arg_INCLUDE_DIR MATCHES "(.*)/$")
            set(incl_path ${CMAKE_MATCH_1})
        else()
            message(STATUS "Invalid include directory: ${arg_INCLUDE_DIR}")  
            message(FATAL_ERROR "The include directory should end with '/'")  
        endif()

        # set the include directory
        target_include_directories(${name} SYSTEM INTERFACE $<BUILD_INTERFACE:${incl_path}>)
        # set the thirdparty include directory if exists
        if(EXISTS ${incl_path}/thirdparty)
            target_include_directories(${name} SYSTEM INTERFACE $<BUILD_INTERFACE:${incl_path}/thirdparty>)
        else(EXSTS ${incl_path}/3rdparty)
            target_include_directories(${name} SYSTEM INTERFACE $<BUILD_INTERFACE:${incl_path}/3rdparty>)
        endif()
        # set the thirdparty include directory for the thirdparty libraries
        if(arg_THIRD_LIBRARIES)
            foreach(arg_LIBRARY IN LISTS arg_THIRD_LIBRARIES)
                # set the thirdparty include directory
                target_include_directories(${name} SYSTEM INTERFACE $<BUILD_INTERFACE:${incl_path}/3rdparty/3rdparty_${arg_LIBRARY}>)
            endforeach()
        endif()

        # install the include directory
        install(DIRECTORY ${arg_INCLUDE_DIR}
            DESTINATION ${CMAKE_INSTALL_PREFIX}/include/3rdparty/${name}
            FILES_MATCHING
                PATTERN "*.h"
                PATTERN "*.hpp"
        )

        # set the include directory for the installed library
        target_include_directories(${name} INTERFACE $<INSTALL_INTERFACE:include/3rdparty/${name}>)
        # set the thirdparty include directory for the installed library if exists
        if(EXISTS ${incl_path}/thirdparty)
            target_include_directories(${name} INTERFACE $<INSTALL_INTERFACE:include/3rdparty/${name}/thirdparty>)
        elseif(EXISTS ${incl_path}/3rdparty)
            target_include_directories(${name} INTERFACE $<INSTALL_INTERFACE:include/3rdparty/${name}/3rdparty>)       
        endif()
        # set the thirdparty include directory for the thirdparty libraries
        if(arg_THIRD_LIBRARIES)
            foreach(arg_LIBRARY IN LISTS arg_THIRD_LIBRARIES)
                # set the thirdparty include directory
                target_include_directories(${name} INTERFACE $<INSTALL_INTERFACE:include/3rdparty/${name}/3rdparty/3rdparty_${arg_LIBRARY}>)
            endforeach()
        endif()
    endif()

    # link and install the static libraries
    if(arg_STATIC_LIBRARIES)
        list(LENGTH arg_STATIC_LIBRARIES libcount)
        foreach(arg_LIBRARY IN LISTS arg_STATIC_LIBRARIES)
            # set the static library filename
            set(library_filename ${CMAKE_STATIC_LIBRARY_PREFIX}${arg_LIBRARY}${CMAKE_STATIC_LIBRARY_SUFFIX})
            # message(STATUS "Link static library: ${library_filename}")

            # link the static library
            target_link_libraries(${name} INTERFACE "$<BUILD_INTERFACE:${arg_LIB_DIR}/${library_filename}>")

            # install the static library
            install(FILES ${arg_LIB_DIR}/${library_filename}
                    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
            )

            # set the link for the installed library
            target_link_libraries(${name} INTERFACE $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/lib/${library_filename}>)
        endforeach()
    endif()

    # link and install the shared libraries
    if(arg_SHARED_LIBRARIES)
        list(LENGTH arg_SHARED_LIBRARIES libcount)
        foreach(arg_LIBRARY IN LISTS arg_SHARED_LIBRARIES)
            # set the shared library filename
            set(library_filename ${CMAKE_SHARED_LIBRARY_PREFIX}${arg_LIBRARY}${CMAKE_SHARED_LIBRARY_SUFFIX})
            message(STATUS "Link shared library: ${library_filename}")

            # link the shared library
            if(WIN32)
                # if the arg_LIBRARY is not libcurl
                if(NOT arg_LIBRARY STREQUAL "libcurl")
                    # for windows, we still need to link the static library for building
                    set(static_library_filename ${CMAKE_STATIC_LIBRARY_PREFIX}${arg_LIBRARY}${CMAKE_STATIC_LIBRARY_SUFFIX})
                    target_link_libraries(${name} INTERFACE 
                            "$<BUILD_INTERFACE:${arg_LIB_DIR}/${static_library_filename}>")
                    # install the static library
                    install(FILES ${arg_LIB_DIR}/${static_library_filename}
                            DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
                    )
                endif()
                # install the shared library
                install(FILES ${arg_BIN_DIR}/${library_filename}
                        DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
                        )
                # if the arg_LIBRARY is not libcurl
                if(NOT arg_LIBRARY STREQUAL "libcurl")
                    # set the link for the installed library
                    target_link_libraries(${name} INTERFACE 
                            $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/lib/${static_library_filename}>)
                endif()
            else()
                # for linux, we only need to link the shared library
                target_link_libraries(${name} INTERFACE
                        "$<BUILD_INTERFACE:${arg_LIB_DIR}/${library_filename}>")

                # install the shared library
                install(DIRECTORY ${arg_LIB_DIR}/
                    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
                    FILES_MATCHING
                    PATTERN "*${library_filename}*"
                    PATTERN "*/cmake" EXCLUDE
                    PATTERN "*/pkgconfig" EXCLUDE
                    PATTERN "*/libk4a1.4" EXCLUDE
                )
                # set the link for the installed library
                target_link_libraries(${name} INTERFACE
                        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/lib/${library_filename}>)
            endif()
        endforeach()
    endif()

    # install the target
    install(TARGETS ${name} EXPORT ${CMAKE_FILE_BASE_NAME}-${arg_MODULE}-targets)
    add_library(${PROJECT_NAME}::${name} ALIAS ${name})
endfunction()

# REALSENSE Driver
set(REALSENSE_LIB_NAME realsense)

set(REALSENSE_ROOT ${THIRDPARTY_DIR}/realsense)
message(STATUS "Build for Jetson -> ${REALSENSE_ROOT}")

set(REALSENSE_INCLUDE_DIR    ${REALSENSE_ROOT}/include/) #Note: the include directory is end with '/'
set(REALSENSE_LIB_DIR        ${REALSENSE_ROOT}/lib)

set(REALSENSE_SHARED_LIBRARIES  realsense2)

determ_import_3rdparty_library(${REALSENSE_LIB_NAME}
    INCLUDE_DIR      ${REALSENSE_INCLUDE_DIR}
    LIB_DIR          ${REALSENSE_LIB_DIR}
    SHARED_LIBRARIES ${REALSENSE_SHARED_LIBRARIES}
    MODULE           capture
)