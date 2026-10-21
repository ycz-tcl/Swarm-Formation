cmake_minimum_required(VERSION 3.8)
project(swarm_bridge)

# 保证默认 Release（上游 catkin 用 -O3），可被 -DCMAKE_BUILD_TYPE= 覆盖
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(traj_utils REQUIRED)

add_executable(bridge_node
  src/bridge_node.cpp
  )
ament_target_dependencies(bridge_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)
target_link_libraries(bridge_node
  # ROS1: catkin traj_utils — ROS2: rosidl aggregate typesupport target
  traj_utils::traj_utils
)

install(TARGETS bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
