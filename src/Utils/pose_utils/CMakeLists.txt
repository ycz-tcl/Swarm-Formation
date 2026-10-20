cmake_minimum_required(VERSION 3.8)
project(pose_utils)

find_package(ament_cmake REQUIRED)
find_package(Armadillo REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# No Armadillo::Armadillo imported target exists (built-in FindArmadillo
# defines variables only), so chain the found libraries directly.
target_link_libraries(${PROJECT_NAME} INTERFACE ${ARMADILLO_LIBRARIES})

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(${PROJECT_NAME}Targets)
ament_export_dependencies(Armadillo)
ament_package()
