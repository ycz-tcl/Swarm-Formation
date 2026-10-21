cmake_minimum_required(VERSION 3.8)
project(poscmd_2_odom)

# 保证默认 Release（上游 catkin 用 -O3），可被 -DCMAKE_BUILD_TYPE= 覆盖
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(quadrotor_msgs REQUIRED)

add_executable(poscmd_2_odom src/poscmd_2_odom.cpp)
ament_target_dependencies(poscmd_2_odom
  rclcpp
  nav_msgs
)
target_link_libraries(poscmd_2_odom
  Eigen3::Eigen
  # ROS1: catkin quadrotor_msgs — ROS2: rosidl aggregate typesupport target
  quadrotor_msgs::quadrotor_msgs
)

install(TARGETS poscmd_2_odom DESTINATION lib/${PROJECT_NAME})

ament_package()
