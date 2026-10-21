cmake_minimum_required(VERSION 3.8)
project(ego_planner)

# 保证默认 Release（上游 catkin 用 -O3），可被 -DCMAKE_BUILD_TYPE= 覆盖
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(traj_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(swarm_graph REQUIRED)

# ROS1: add_executable(ego_planner_node ego_planner_node.cpp ego_replan_fsm.cpp planner_manager.cpp)
add_executable(ego_planner_node
  src/ego_planner_node.cpp
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
  )
target_include_directories(ego_planner_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
# traj_utils 经 ament_target_dependencies 自动带上 planning_visualization 库 +
# 生成的消息 + 头文件（勿显式 target_link_libraries planning_visualization）
ament_target_dependencies(ego_planner_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  quadrotor_msgs
  plan_env
  path_searching
  traj_opt
  traj_utils
  swarm_graph)
target_link_libraries(ego_planner_node Eigen3::Eigen)

# traj_server 包含 traj_opt 的 optimizer/poly_traj_utils.hpp（header-only）
add_executable(traj_server src/traj_server.cpp)
target_include_directories(traj_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(traj_server
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  quadrotor_msgs
  traj_utils
  traj_opt)
target_link_libraries(traj_server Eigen3::Eigen)

install(TARGETS ego_planner_node traj_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
