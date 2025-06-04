cmake_minimum_required(VERSION 3.16)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 20)

project(ros2_plugin)

if(DEFINED ENV{MUJOCO_PATH})
    set(MUJOCO_PATH $ENV{MUJOCO_PATH})
else()
    find_program(MUJOCO_PROGRAM simulate)
    cmake_path(GET MUJOCO_PROGRAM PARENT_PATH MUJOCO_PATH)
endif()

if(MUJOCO_PATH)
    message(STATUS "Mujoco found at: ${MUJOCO_PATH}")
else()
    message(FATAL_ERROR
        "Mujoco program not found at expected path: ${MUJOCO_PATH}\n"
        "Define the MUJOCO_PATH environment variable or add the binary folder to the PATH"
    )
endif()

cmake_path(GET MUJOCO_PATH PARENT_PATH MUJOCO_LIB)
link_directories("${MUJOCO_LIB}/lib")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_library(ros_plugin SHARED)
target_sources(ros_plugin PRIVATE "src/ros2.cpp" "src/register.cpp")
target_include_directories(ros_plugin PRIVATE "${MUJOCO_PATH}/../include")

target_link_libraries(ros_plugin mujoco)
ament_target_dependencies(ros_plugin
    rclcpp
    example_interfaces
)

install(TARGETS ros_plugin
    EXPORT export_ros_plugin
    LIBRARY DESTINATION ${MUJOCO_PATH}/mujoco_plugin
)
