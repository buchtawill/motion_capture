#
# This file is the mocap-hdmi-memcp recipe.
#

SUMMARY = "HDMI display prototype for OV9281 V4L2 capture (CPU memcpy to /dev/fb0)"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

DEPENDS = "mocap-common"

SRC_URI = "file://mocap-hdmi-memcp.cpp \
           file://Makefile \
		  "

S = "${WORKDIR}"

do_compile() {
	     oe_runmake
}

do_install() {
	     install -d ${D}${bindir}
	     install -m 0755 mocap-hdmi-memcp ${D}${bindir}
}
