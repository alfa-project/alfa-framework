cmake_minimum_required(VERSION 3.8)
project(ext_fog_decoder)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release) # Default to Release if not specified
endif()

# Additional compiler options for specific configurations or targets
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-O0 -g)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set ALFA HARDWARE EXTENSION, Comment the line below for software only extensions
#add_compile_definitions(EXT_HARDWARE)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(alfa_node REQUIRED)
find_package(alfa_msg REQUIRED)
find_package(alib_octree REQUIRED)
find_package(alib_compression REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(ext_fog_decoder src/main.cpp)
target_link_libraries(ext_fog_decoder ${PCL_LIBRARIES})
ament_target_dependencies(ext_fog_decoder rclcpp alfa_node sensor_msgs alfa_msg pcl_conversions alib_octree alib_compression)

install(TARGETS
  ext_fog_decoder
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
