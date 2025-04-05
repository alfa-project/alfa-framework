cmake_minimum_required(VERSION 3.8)
project(alib_extmem)

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
find_package(rclcpp REQUIRED)

find_package(alfa_node REQUIRED)
find_package(alfa_msg REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(pcl_conversions REQUIRED)

# let the compiler search for headers in the include folder
include_directories(include)

# define a library target called alib_extmem
set(HEADER_FILES include/alib_extmem.hpp)
add_library(alib_extmem src/alib_extmem.cpp ${HEADER_FILES})

# Exports the library
ament_export_targets(alib_extmem HAS_LIBRARY_TARGET)
target_link_libraries(alib_extmem ${PCL_LIBRARIES})
ament_target_dependencies(alib_extmem alfa_node alfa_msg)
 
install(
  DIRECTORY include/
  DESTINATION include
)
 
install(
  TARGETS alib_extmem
  EXPORT alib_extmem
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
