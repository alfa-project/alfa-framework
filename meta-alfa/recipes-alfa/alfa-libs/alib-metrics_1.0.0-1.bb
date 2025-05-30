inherit ros_distro_${ROS_DISTRO}
inherit ros_superflore_generated

DESCRIPTION = "ALFA Metrics Library"
SECTION = "devel"
LICENSE = "Apache-2.0"
FILE_DIR := "${@os.path.dirname(d.getVar('FILE'))}"
ALFA_FRAMEWORK_PATH = "${FILE_DIR}/../../.."
ALFA_LIB_PATH = "${ALFA_FRAMEWORK_PATH}/libs/sw/"
ALFA_LIB_NAME = "alib_metrics"
LIC_FILES_CHKSUM = "file://${ALFA_LIB_PATH}/${ALFA_LIB_NAME}/package.xml;beginline=8;endline=8;md5=35b29ecc7cd77b8c23d7f8cebb480660"

PREFERENCE = "-1"

ROS_CN = "${ALFA_LIB_NAME}"
ROS_BPN := "${@d.getVar('ROS_CN').replace('_', '-')}"

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
FILES:${PN} += "/usr/share/${ALFA_LIB_NAME}/*"

SRC_URI = "file://${ALFA_LIB_PATH}/${ROS_CN}/package.xml \
		   file://${ALFA_LIB_PATH}/${ROS_CN}/CMakeLists.txt \
		   file://${ALFA_LIB_PATH}/${ROS_CN}/include/alib_metrics.hpp \
		   file://${ALFA_LIB_PATH}/${ROS_CN}/src/alib_metrics.cpp \
		   "

SRCREV = "${AUTOREV}"
S = "${WORKDIR}/${ALFA_LIB_PATH}/${ROS_CN}"

ROS_BUILD_TYPE = "ament_cmake"
inherit ros_${ROS_BUILD_TYPE}

