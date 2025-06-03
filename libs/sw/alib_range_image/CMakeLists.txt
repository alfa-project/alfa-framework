cmake_minimum_required(VERSION 3.8)
project(alib_range_image)

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
# add_compile_definitions(EXT_HARDWARE)
add_compile_definitions(HDL_64)

# find dependencies
find_package(ament_cmake REQUIRED)


find_package(rclcpp REQUIRED)

find_package(alfa_node REQUIRED)
find_package(alfa_msg REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(pcl_conversions REQUIRED)

# let the compiler search for headers in the include folder
include_directories(include)

# define a library target called alib_range_image
set(HEADER_FILES include/alib_range_image.hpp)
add_library(alib_range_image src/alib_range_image.cpp ${HEADER_FILES})

# Exports the library
ament_export_targets(alib_range_image HAS_LIBRARY_TARGET)
target_link_libraries(alib_range_image ${PCL_LIBRARIES})
ament_target_dependencies(alib_range_image rclcpp alfa_node sensor_msgs alfa_msg pcl_conversions)

# let the co
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS alib_range_image
  EXPORT alib_range_image
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
