inherit ros_distro_${ROS_DISTRO}
inherit ros_superflore_generated

DESCRIPTION = "ALFA Octree Library"
SECTION = "devel"
LICENSE = "Apache-2.0"
FILE_DIR := "${@os.path.dirname(d.getVar('FILE'))}"
ALFA_FRAMEWORK_PATH = "${FILE_DIR}/../../.."
ALFA_OCTREE_PATH = "${ALFA_FRAMEWORK_PATH}/libs/sw/alib_octree"
LIC_FILES_CHKSUM = "file://${ALFA_OCTREE_PATH}/package.xml;beginline=8;endline=8;md5=35b29ecc7cd77b8c23d7f8cebb480660"

PREFERENCE = "-1"

ROS_CN = "alib_octree"
ROS_BPN = "alib-octree"

ROS_BUILDTOOL_DEPENDS = " \
	ament-cmake-native \
"

ROS_BUILDTOOL_EXPORT_DEPENDS = ""

ROS_BUILD_DEPENDS = " \
	rclcpp \
	rclcpp-components \
	sensor-msgs \
	alfa-msg \
	pcl-conversions \
	pcl-msgs \
	alfa-node \
"

ROS_EXPORT_DEPENDS = " \
	rclcpp \
	rclcpp-components \
	sensor-msgs \
	alfa-msg \
	pcl-conversions \
	pcl-msgs \
	alfa-node \
"

ROS_EXEC_DEPENDS = " \
	rclcpp \
	rclcpp-components \
	sensor-msgs \
	alfa-msg \
	pcl-conversions \
	pcl-msgs \
	alfa-node \
"

ROS_TEST_DEPENDS = " \
	ament-cmake-gtest \
	ament-lint-auto \
	ament-lint-common \
"

DEPENDS = "${ROS_BUILD_DEPENDS} ${ROS_BUILDTOOL_DEPENDS}"
DEPENDS += "${ROS_EXPORT_DEPENDS} ${ROS_BUILDTOOL_EXPORT_DEPENDS}"

RDEPENDS:${PN} += "${ROS_EXEC_DEPENDS}"
FILES:${PN} += "/usr/share/alib_octree/*"

SRC_URI = "file://${ALFA_OCTREE_PATH}/package.xml \
		   file://${ALFA_OCTREE_PATH}/CMakeLists.txt \
		   file://${ALFA_OCTREE_PATH}/include/alib_octree.hpp \
		   file://${ALFA_OCTREE_PATH}/src/alib_octree.cpp \
		   "

SRCREV = "${AUTOREV}"
S = "${WORKDIR}/${ALFA_OCTREE_PATH}"

ROS_BUILD_TYPE = "ament_cmake"
inherit ros_${ROS_BUILD_TYPE}

