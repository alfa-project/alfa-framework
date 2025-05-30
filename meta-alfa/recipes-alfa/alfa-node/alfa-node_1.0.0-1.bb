inherit ros_distro_${ROS_DISTRO}
inherit ros_superflore_generated

DESCRIPTION = "ALFA Node"
SECTION = "devel"
LICENSE = "Apache-2.0"
FILE_DIR := "${@os.path.dirname(d.getVar('FILE'))}"
ALFA_FRAMEWORK_PATH = "${FILE_DIR}/../../.."
ALFA_NODE_PATH = "${ALFA_FRAMEWORK_PATH}/node"
LIC_FILES_CHKSUM = "file://${ALFA_NODE_PATH}/package.xml;beginline=8;endline=8;md5=35b29ecc7cd77b8c23d7f8cebb480660"

PREFERENCE = "-1"

ROS_CN = "alfa-node"
ROS_BPN = "alfa-node"

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
"

ROS_EXPORT_DEPENDS = " \
	rclcpp \
	rclcpp-components \
	sensor-msgs \
	alfa-msg \
	pcl-conversions \
	pcl-msgs \
"

ROS_EXEC_DEPENDS = " \
	rclcpp \
	rclcpp-components \
	sensor-msgs \
	alfa-msg \
	pcl-conversions \
	pcl-msgs \
"

ROS_TEST_DEPENDS = " \
	ament-cmake-gtest \
	ament-lint-auto \
	ament-lint-common \
"

DEPENDS = "${ROS_BUILD_DEPENDS} ${ROS_BUILDTOOL_DEPENDS}"
DEPENDS += "${ROS_EXPORT_DEPENDS} ${ROS_BUILDTOOL_EXPORT_DEPENDS}"

RDEPENDS:${PN} += "${ROS_EXEC_DEPENDS}"
FILES:${PN} += "/usr/share/alfa_node/*"

SRC_URI = "file://${ALFA_NODE_PATH}/package.xml \
		   file://${ALFA_NODE_PATH}/CMakeLists.txt \
		   file://${ALFA_NODE_PATH}/include/alfa_node.hpp \
		   file://${ALFA_NODE_PATH}/include/alfa_defines.hpp \
		   file://${ALFA_NODE_PATH}/include/alfa_structs.hpp \
		   file://${ALFA_NODE_PATH}/src/alfa_node.cpp \
			 file://${ALFA_NODE_PATH}/src/alfa_hw.cpp \
			 file://${ALFA_NODE_PATH}/src/alfa_metrics.cpp \
			 file://${ALFA_NODE_PATH}/src/alfa_parameters.cpp \
			 file://${ALFA_NODE_PATH}/src/alfa_pointclouds.cpp \
			 file://${ALFA_NODE_PATH}/src/alfa_verbose.cpp \
		   "

SRCREV = "${AUTOREV}"
S = "${WORKDIR}/${ALFA_NODE_PATH}"

ROS_BUILD_TYPE = "ament_cmake"
inherit ros_${ROS_BUILD_TYPE}

