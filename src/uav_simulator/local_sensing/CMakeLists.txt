cmake_minimum_required(VERSION 3.8)
project(local_sensing_node)

# 保证默认 Release（上游 catkin 用 Release），可被 -DCMAKE_BUILD_TYPE= 覆盖
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# ROS1: set(ENABLE_CUDA false) — 结构保留，默认 false
set(ENABLE_CUDA false)

if(ENABLE_CUDA)
  # CUDA 变体（src/pcl_render_node.cpp + src/depth_render.cu，依赖 cv_bridge/
  # image_transport/dynamic_reconfigure）未迁移到 ROS2，文件保留但默认不编译；
  # 如需启用须先迁移 CUDA 路径。
  message(FATAL_ERROR "ENABLE_CUDA=true is not ported to ROS2 (Humble) yet")
else()
  add_executable(pcl_render_node src/pointcloud_render_node.cpp)
  ament_target_dependencies(pcl_render_node
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    pcl_conversions
  )
  target_include_directories(pcl_render_node PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(pcl_render_node Eigen3::Eigen ${PCL_LIBRARIES})

  install(TARGETS pcl_render_node DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
