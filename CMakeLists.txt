cmake_minimum_required(VERSION 3.16)

project(ros2_plugin)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(example_interfaces REQUIRED)
else()
  message(ERROR "ROS2 not found.")
endif()

set(MUJOCO_ACTUATOR_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../..
                            ${CMAKE_CURRENT_SOURCE_DIR}/../../src
)

add_library(ros_plugin SHARED)
target_sources(ros_plugin PRIVATE "ros2.cpp" "register.cpp")
target_include_directories(ros_plugin PRIVATE ${MUJOCO_ACTUATOR_INCLUDE})

target_compile_definitions(ros_plugin PRIVATE ROS2_ENABLED)
target_link_libraries(ros_plugin mujoco)
ament_target_dependencies(ros_plugin
    rclcpp
    example_interfaces
)

target_compile_options(
  ros_plugin
  PRIVATE ${AVX_COMPILE_OPTIONS}
          ${MUJOCO_MACOS_COMPILE_OPTIONS}
          ${EXTRA_COMPILE_OPTIONS}
          ${MUJOCO_CXX_FLAGS}
)
target_link_options(
  ros_plugin
  PRIVATE
  ${MUJOCO_MACOS_LINK_OPTIONS}
  ${EXTRA_LINK_OPTIONS}
)
