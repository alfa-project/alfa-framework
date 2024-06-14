inherit ros_distro_${ROS_DISTRO}
inherit ros_superflore_generated

DESCRIPTION = "ALFA Messages"
SECTION = "devel"
LICENSE = "Apache-2.0"
FILE_DIR := "${@os.path.dirname(d.getVar('FILE'))}"
ALFA_FRAMEWORK_PATH = "${FILE_DIR}/../../.."
ALFA_MSG_PATH = "${ALFA_FRAMEWORK_PATH}/msgs"
LIC_FILES_CHKSUM = "file://${ALFA_MSG_PATH}/package.xml;beginline=8;endline=8;md5=12c26a18c7f493fdc7e8a93b16b7c04f"

ROS_CN = "alfa-msg"
ROS_BPN = "alfa-msg"

ROS_BUILDTOOL_DEPENDS = " \
	ament-cmake-native \
	rosidl-default-generators-native \
	rosidl-parser-native \
	rosidl-adapter-native \
	rosidl-typesupport-fastrtps-cpp-native \
	rosidl-typesupport-fastrtps-c-native \
"

ROS_BUILDTOOL_EXPORT_DEPENDS = ""

ROS_BUILD_DEPENDS = " \
	builtin-interfaces \
	rosidl-typesupport-c \
	rosidl-typesupport-cpp \
"

ROS_EXPORT_DEPENDS = ""

ROS_EXEC_DEPENDS = " \
	builtin-interfaces \
	rosidl-default-runtime \
"

# Currently informational only -- see http://www.ros.org/reps/rep-0149.html#dependency-tags.
ROS_TEST_DEPENDS = " \
	ament-cmake-gtest \
	ament-lint-auto \
	ament-lint-common \
"

DEPENDS = "${ROS_BUILD_DEPENDS} ${ROS_BUILDTOOL_DEPENDS}"
DEPENDS += "${ROS_EXPORT_DEPENDS} ${ROS_BUILDTOOL_EXPORT_DEPENDS}"

RDEPENDS:${PN} += "${ROS_EXEC_DEPENDS}"
FILES:${PN} += "/usr/share/alfa_msg/*"

SRC_URI = "file://${ALFA_MSG_PATH}/package.xml \
		   file://${ALFA_MSG_PATH}/CMakeLists.txt \
		   file://${ALFA_MSG_PATH}/srv/AlfaConfigure.srv \
		   file://${ALFA_MSG_PATH}/msg/AlfaAlivePing.msg \
		   file://${ALFA_MSG_PATH}/msg/AlfaMetrics.msg \
		   file://${ALFA_MSG_PATH}/msg/ConfigMessage.msg \
		   file://${ALFA_MSG_PATH}/msg/MetricMessage.msg \
		   "
SRCREV = "${AUTOREV}"
S = "${WORKDIR}/${ALFA_MSG_PATH}"

ROS_BUILD_TYPE = "ament_cmake"
inherit ros_${ROS_BUILD_TYPE}

