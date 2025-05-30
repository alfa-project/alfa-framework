inherit ros_distro_${ROS_DISTRO}
inherit ros_superflore_generated

DESCRIPTION = "ALFA Octree compression PCL Extension"
SECTION = "devel"
LICENSE = "Apache-2.0"
FILE_DIR := "${@os.path.dirname(d.getVar('FILE'))}"
ALFA_FRAMEWORK_PATH = "${FILE_DIR}/../../.."
ALFA_EXT_PATH = "${ALFA_FRAMEWORK_PATH}/extensions/sw"
ALFA_EXT_NAME = "ext_pcl_octree_compression_encoder"
LIC_FILES_CHKSUM = "file://${ALFA_EXT_PATH}/${ALFA_EXT_NAME}/package.xml;beginline=8;endline=8;md5=12c26a18c7f493fdc7e8a93b16b7c04f"

ROS_CN = "${ALFA_EXT_NAME}"
ROS_BPN = "ext-pcl-octree-compression"


ROS_BUILDTOOL_DEPENDS = " \
	ament-cmake-native \
"

ROS_BUILDTOOL_EXPORT_DEPENDS = ""

DEPENDS += "qhull"

ROS_BUILD_DEPENDS = " \
	rclcpp \
	rclcpp-components \
	qhull \
	alfa-node \
	alib-octree \
	alib-compression \
	sensor-msgs \
	alfa-msg \
	pcl-conversions \
	pcl-msgs \
"

ROS_EXPORT_DEPENDS = " \
	rclcpp \
	qhull \
	rclcpp-components \
	alfa-node \
	alib-octree \
	alib-compression \
	sensor-msgs \
	alfa-msg \
	pcl-conversions \
	pcl-msgs \
"

ROS_EXEC_DEPENDS = " \
	rclcpp \
	rclcpp-components \
	qhull \
	alfa-node \
	alib-compression \
	alib-octree \
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
FILES:${PN} += "/usr/share/${ROS_CN}/* /usr/lib/${ROS_CN}/*"

SRC_URI = "file://${ALFA_EXT_PATH}/${ROS_CN}/package.xml \
		   file://${ALFA_EXT_PATH}/${ROS_CN}/CMakeLists.txt \
		   file://${ALFA_EXT_PATH}/${ROS_CN}/src/main.cpp \
		   "

SRCREV = "${AUTOREV}"
S = "${WORKDIR}/${ALFA_EXT_PATH}/${ROS_CN}"

ROS_BUILD_TYPE = "ament_cmake"
inherit ros_${ROS_BUILD_TYPE}
