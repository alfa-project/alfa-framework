cmake_minimum_required(VERSION 3.8)
project(ext_patchwork)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -w)
endif()

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(alfa_node REQUIRED)
find_package(alfa_msg REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(ext_patchwork src/patchwork.cpp src/patchwork.hpp)
target_link_libraries(ext_patchwork ${PCL_LIBRARIES})
ament_target_dependencies(ext_patchwork rclcpp alfa_node sensor_msgs alfa_msg pcl_conversions)

install(TARGETS
ext_patchwork
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()