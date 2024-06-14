SUMMARY = "Recipe for build ALFA memory dd Linux kernel module"
SECTION = "devel"
LICENSE = "Apache-2.0"
FILE_DIR := "${@os.path.dirname(d.getVar('FILE'))}"
ALFA_FRAMEWORK_PATH = "${FILE_DIR}/../../.."
ALFA_DD_PATH = "${ALFA_FRAMEWORK_PATH}/drivers/alfa_mem"
LIC_FILES_CHKSUM = "file://${ALFA_FRAMEWORK_PATH}/LICENSE;md5=2b42edef8fa55315f34f2370b4715ca9"

inherit module

INHIBIT_PACKAGE_STRIP = "1"

SRC_URI = "file://${ALFA_DD_PATH}/alfa_mem.c \
		   file://${ALFA_DD_PATH}/Makefile \
		   "

SRCREV = "${AUTOREV}"
S = "${WORKDIR}/${ALFA_DD_PATH}"
