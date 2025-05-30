cmake_minimum_required(VERSION 3.8)
project(ext_dior)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-O0 -g)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(alfa_node REQUIRED)
find_package(alfa_msg REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io kdtree)
#find_package(FLANN REQUIRED PATHS /usr)
find_package(Qhull REQUIRED)

# Add the executable and include both main.cpp and instantiations.cpp
add_executable(ext_dior src/main.cpp)

target_include_directories(ext_dior
    PRIVATE
    ${alfa_node_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
    ${QHULL_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

# Explicitly link against the required libraries
target_link_libraries(ext_dior
    ${alfa_node_LIBRARIES}   # AlfaNode library
    ${FLANN_LIBRARIES}       # FLANN
    ${QHULL_LIBRARIES}       # Qhull
    ${PCL_IO_LIBRARIES}      # PCL IO
    ${PCL_OCTREE_LIBRARIES}  # PCL Octree
    ${PCL_COMMON_LIBRARIES}  # PCL Common
)

ament_target_dependencies(ext_dior
    rclcpp 
    alfa_node 
    sensor_msgs 
    alfa_msg 
    pcl_conversions
)

install(TARGETS
  ext_dior
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
