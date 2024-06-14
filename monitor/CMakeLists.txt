cmake_minimum_required(VERSION 3.5)

project(alfa_monitor VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_language(C)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(alfa_node REQUIRED)
find_package(alfa_msg REQUIRED)

find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(VTK REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(include)
include_directories(
    SYSTEM
    ${PCL_INCLUDE_DIRS}
)

set(PROJECT_SOURCES
        src/main.cpp
        src/pclviewer.cpp
        src/boxsettings.cpp
        src/noisegenerator.cpp
        src/configwindow.cpp
        src/metricwindow.cpp
        src/rosinterface.cpp

        include/pclviewer.h
        include/boxsettings.h
        include/noisegenerator.h
        include/style_sheat.h
        include/clouds.h
        include/metricwindow.h
        include/configwindow.h
        include/rosinterface.h

        src/ui/pclviewer.ui
        src/ui/boxsettings.ui
        src/ui/noisegenerator.ui
        src/ui/configwindow.ui
        src/ui/metricwindow.ui
)

add_executable(alfa_monitor ${PROJECT_SOURCES})
target_link_libraries(alfa_monitor Qt${QT_VERSION_MAJOR}::Widgets ${PCL_LIBRARIES})
ament_target_dependencies(alfa_monitor rclcpp alfa_node sensor_msgs rosbag2_cpp alfa_msg pcl_conversions)

install(TARGETS
  alfa_monitor
  DESTINATION lib/${PROJECT_NAME}/
)


ament_package()
