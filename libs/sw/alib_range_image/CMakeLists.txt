cmake_minimum_required(VERSION 3.8)
project(ext_fric_64_speed)

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
add_compile_definitions(EXT_HARDWARE)

# find dependencies
find_package(ament_cmake REQUIRED)


find_package(rclcpp REQUIRED)

find_package(alfa_node REQUIRED)
find_package(alfa_msg REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(ext_fric_64_speed src/main.cpp)
target_link_libraries(ext_fric_64_speed ${PCL_LIBRARIES})
ament_target_dependencies(ext_fric_64_speed rclcpp alfa_node sensor_msgs alfa_msg pcl_conversions)

install(TARGETS
  ext_fric_64_speed
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
