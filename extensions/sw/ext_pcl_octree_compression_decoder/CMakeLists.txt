cmake_minimum_required(VERSION 3.8)
project(ext_pcl_octree_compression_decoder)

# Default build type is Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Additional compiler options
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-O0 -g)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
endif()

# Enable extra warnings for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) 
find_package(alfa_node REQUIRED)
find_package(alib_octree REQUIRED)
find_package(alfa_msg REQUIRED)
find_package(alib_compression REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS io octree common)
find_package(pcl_conversions REQUIRED)
find_package(FLANN REQUIRED)
find_package(Qhull REQUIRED)

# Add the executable and include both main.cpp and instantiations.cpp
add_executable(ext_pcl_octree_compression_decoder src/main.cpp)

# Include directories for all necessary libraries
target_include_directories(ext_pcl_octree_compression_decoder
    PRIVATE
    ${alfa_node_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
    ${QHULL_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

# Explicitly link against the required libraries
target_link_libraries(ext_pcl_octree_compression_decoder
    ${alfa_node_LIBRARIES}   # AlfaNode library
    ${FLANN_LIBRARIES}       # FLANN
    ${QHULL_LIBRARIES}       # Qhull
    ${PCL_IO_LIBRARIES}      # PCL IO
    ${PCL_OCTREE_LIBRARIES}  # PCL Octree
    ${PCL_COMMON_LIBRARIES}  # PCL Common
)

# Link other dependencies
ament_target_dependencies(ext_pcl_octree_compression_decoder 
    rclcpp 
    alfa_node
    alib_octree
    alib_compression 
    sensor_msgs 
    alfa_msg 
    pcl_conversions
)

# Install configuration
install(TARGETS
  ext_pcl_octree_compression_decoder
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()

