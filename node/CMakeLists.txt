cmake_minimum_required(VERSION 3.8)
project(alfa_node)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release) # Default to Release if not specified
endif()

# Additional compiler options for specific configurations or targets
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-O0 -g)
  add_compile_definitions(ALFA_VERBOSE)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(rclcpp REQUIRED)
find_package(alfa_msg REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)

# let the compiler search for headers in the include folder
include_directories(include)

# define a library target called alfa_node
set(HEADER_FILES include/alfa_node.hpp)
add_library(alfa_node src/alfa_node.cpp ${HEADER_FILES})

#add_compile_definitions(ALFA_VERBOSE)

# Exports the library
ament_export_targets(alfa_node HAS_LIBRARY_TARGET)
target_link_libraries(alfa_node ${PCL_LIBRARIES})
ament_target_dependencies(alfa_node rclcpp sensor_msgs alfa_msg pcl_conversions pcl_msgs)
 
install(
  DIRECTORY include/
  DESTINATION include
)
 
install(
  TARGETS alfa_node
  EXPORT alfa_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
