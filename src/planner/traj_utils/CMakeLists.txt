cmake_minimum_required(VERSION 3.8)
project(traj_utils)

# 保证默认 Release（上游 catkin 用 -O3），可被 -DCMAKE_BUILD_TYPE= 覆盖
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Assignment.msg"
  "msg/DataDisp.msg"
  "msg/PolyTraj.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# C++ library (ROS1: catkin library 'traj_utils' from planning_visualization.cpp;
# the bare name is now taken by the rosidl interface aggregate target, so the
# library target is named after the source)
add_library(planning_visualization src/planning_visualization.cpp)
target_include_directories(planning_visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(planning_visualization
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs)
target_link_libraries(planning_visualization Eigen3::Eigen)

# Consume the message interfaces generated in this same package (mixed
# interface package pattern; Humble equivalent of newer distros'
# ${rosidl_generate_interfaces_targets})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(planning_visualization "${cpp_typesupport_target}")

install(TARGETS planning_visualization
  EXPORT export_traj_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_traj_utils HAS_LIBRARY_TARGET)
ament_export_dependencies(
  eigen3_cmake_module Eigen3
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  rosidl_default_runtime)

ament_package()
