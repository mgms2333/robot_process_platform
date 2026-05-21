cmake_minimum_required(VERSION 3.22)

project(robot_process_platform VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(ROBOT_PROCESS_PLATFORM_ENABLE_HYROBOT_SDK "Enable Huayan robot SDK integration." OFF)
set(HYROBOT_SDK_ROOT "" CACHE PATH "Path to Huayan robot SDK root directory.")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(robot_process_platform_core
    src/core/json_utils.cpp
    src/core/task_types.cpp
    src/device/interfaces/camera_interface.cpp
    src/device/interfaces/robot_interface.cpp
    src/device/robot/hy_robot.cpp
    src/plugin/template_types.cpp
    src/platform/device_manager.cpp
    src/platform/logger.cpp
    src/platform/system_config.cpp
    src/platform/system_service.cpp
    src/platform/task_manager.cpp
    src/platform/template_manager.cpp
    src/runtime/block_executor.cpp
    src/runtime/block_queue.cpp
    src/runtime/block_scheduler.cpp
    src/runtime/execution_context.cpp
    src/runtime/runtime_service.cpp
    src/runtime/state_machine.cpp
    src/runtime/task_runner.cpp
)

target_include_directories(robot_process_platform_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set_target_properties(robot_process_platform_core PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

if(ROBOT_PROCESS_PLATFORM_ENABLE_HYROBOT_SDK)
    if(NOT HYROBOT_SDK_ROOT)
        message(FATAL_ERROR "HYROBOT_SDK_ROOT must be set when ROBOT_PROCESS_PLATFORM_ENABLE_HYROBOT_SDK=ON")
    endif()

    target_compile_definitions(robot_process_platform_core
        PRIVATE
            ROBOT_PROCESS_PLATFORM_USE_HYROBOT_SDK=1
    )

    target_include_directories(robot_process_platform_core
        PRIVATE
            ${HYROBOT_SDK_ROOT}/include
    )

    target_link_libraries(robot_process_platform_core
        PRIVATE
            ${HYROBOT_SDK_ROOT}/Linux/libHR_Pro.so
    )

    set_property(TARGET robot_process_platform_core
        APPEND PROPERTY BUILD_RPATH ${HYROBOT_SDK_ROOT}/Linux
    )
endif()

add_library(robot_process_platform_palletizing_template SHARED
    plugins/palletizing/src/palletizing_template.cpp
    plugins/palletizing/src/palletizing_plugin_entry.cpp
)

target_include_directories(robot_process_platform_palletizing_template
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/plugins/palletizing/include
)

target_link_libraries(robot_process_platform_palletizing_template
    PUBLIC
        robot_process_platform_core
)

set_target_properties(robot_process_platform_palletizing_template PROPERTIES
    OUTPUT_NAME palletizing_template
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/plugins
)

add_executable(robot_process_platform_runner
    apps/runner/main.cpp
)

target_include_directories(robot_process_platform_runner
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(robot_process_platform_runner
    PRIVATE
        robot_process_platform_core
)
