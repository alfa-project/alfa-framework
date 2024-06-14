inherit ros_distro_${ROS_DISTRO}
inherit ros_superflore_generated

DESCRIPTION = "ALFA Distance Filter Extension"
SECTION = "devel"
LICENSE = "Apache-2.0"
FILE_DIR := "${@os.path.dirname(d.getVar('FILE'))}"
ALFA_FRAMEWORK_PATH = "${FILE_DIR}/../../.."
ALFA_EXT_PATH = "${ALFA_FRAMEWORK_PATH}/extensions/sw"
LIC_FILES_CHKSUM = "file://${ALFA_EXT_PATH}/ext_distance_filter/package.xml;beginline=8;endline=8;md5=12c26a18c7f493fdc7e8a93b16b7c04f"

ROS_CN = "ext-distance-filter"
ROS_BPN = "ext-distance-filter"

ROS_BUILDTOOL_DEPENDS = " \
	ament-cmake-native \
"

ROS_BUILDTOOL_EXPORT_DEPENDS = ""

ROS_BUILD_DEPENDS = " \
	rclcpp \
	rclcpp-components \
	alfa-node \
	sensor-msgs \
	alfa-msg \
	pcl-conversions \
	pcl-msgs \
"

ROS_EXPORT_DEPENDS = " \
	rclcpp \
	rclcpp-components \
	alfa-node \
	sensor-msgs \
	alfa-msg \
	pcl-conversions \
	pcl-msgs \
"

ROS_EXEC_DEPENDS = " \
	rclcpp \
	rclcpp-components \
	alfa-node \
	sensor-msgs \
	alfa-msg \
	pcl-conversions \
	pcl-msgs \
"

ROS_TEST_DEPENDS = " \
	ament-lint-auto \
	ament-lint-common \
"

DEPENDS = "${ROS_BUILD_DEPENDS} ${ROS_BUILDTOOL_DEPENDS}"
DEPENDS += "${ROS_EXPORT_DEPENDS} ${ROS_BUILDTOOL_EXPORT_DEPENDS}"

RDEPENDS:${PN} += "${ROS_EXEC_DEPENDS}"
FILES:${PN} += "/usr/share/ext_distance_filter/* /usr/lib/ext_distance_filter/*"

SRC_URI = "file://${ALFA_EXT_PATH}/ext_distance_filter/package.xml \
		   file://${ALFA_EXT_PATH}/ext_distance_filter/CMakeLists.txt \
		   file://${ALFA_EXT_PATH}/ext_distance_filter/src/main.cpp \
		   "

SRCREV = "${AUTOREV}"
S = "${WORKDIR}/${ALFA_EXT_PATH}/ext_distance_filter"

ROS_BUILD_TYPE = "ament_cmake"
inherit ros_${ROS_BUILD_TYPE}