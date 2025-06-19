cmake_minimum_required(VERSION 3.17)
project(rt_reachability LANGUAGES CXX CUDA)

# Set C++ and CUDA CMAKE variables
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_ARCHITECTURES "86")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -G -g")

# Find ROS2 dependencies (only for the node)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# CUDA Dependency
find_package(CUDAToolkit REQUIRED)

# Add include directories for headers
include_directories(
  include
)

# Grid library (C++)
add_library(grid_lib
  src/Grid.cu
)
target_include_directories(grid_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# SDF library (CUDA)
add_library(sdf_lib
  src/SDF.cu
)
target_include_directories(sdf_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(sdf_lib PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>
)
target_link_libraries(sdf_lib
  CUDA::cudart
)
target_compile_options(grid_lib PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>
)
target_link_libraries(grid_lib
  CUDA::cudart
)

# Node executable (depends on ROS2 and your libraries)
add_executable(sdf_node
  src/sdf_node.cpp
)
target_include_directories(sdf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sdf_node
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
)
target_link_libraries(sdf_node
  grid_lib
  sdf_lib
)

# Install targets and headers
install(
  TARGETS grid_lib sdf_lib sdf_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
