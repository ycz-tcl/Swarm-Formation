cmake_minimum_required(VERSION 3.8)
project(traj_opt)

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
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(traj_utils REQUIRED)
find_package(swarm_graph REQUIRED)

# ROS1: catkin add_library(traj_opt src/poly_traj_optimizer.cpp) — same target
# name kept, and catkin built shared libs, so keep SHARED explicit (colcon
# default is static). 捆绑的 LBFGS/munkres 等头文件不动，随 include/ 一起安装
add_library(${PROJECT_NAME} SHARED
  src/poly_traj_optimizer.cpp
  )
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  plan_env
  path_searching
  traj_utils
  swarm_graph)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  eigen3_cmake_module
  Eigen3
  rclcpp
  plan_env
  path_searching
  traj_utils
  swarm_graph)

ament_package()
