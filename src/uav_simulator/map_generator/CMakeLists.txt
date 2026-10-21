cmake_minimum_required(VERSION 3.8)
project(map_generator)

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
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

add_executable(random_forest src/random_forest_sensing.cpp)
ament_target_dependencies(random_forest
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_conversions
)
target_include_directories(random_forest PRIVATE
  ${PCL_INCLUDE_DIRS})
target_link_libraries(random_forest
  Eigen3::Eigen
  ${PCL_LIBRARIES})

install(TARGETS random_forest DESTINATION lib/${PROJECT_NAME})

ament_package()
