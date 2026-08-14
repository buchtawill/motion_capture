#
# This file is the mocap-hdmi-drm recipe.
#

SUMMARY = "Zero-copy HDMI display for OV9281 V4L2 capture (DRM/KMS NV12 scanout)"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

DEPENDS = "mocap-common libdrm"

# pkg-config (via the Makefile) needs pkgconfig-native on the build PATH, and
# libdrm's headers live in ${includedir}/libdrm (xf86drm.h -> <drm.h>).
inherit pkgconfig

SRC_URI = "file://main.cpp \
           file://signals.hpp \
           file://signals.cpp \
           file://drm_display.hpp \
           file://drm_display.cpp \
           file://box_overlay.hpp \
           file://box_overlay.cpp \
           file://v4l2_capture.hpp \
           file://v4l2_capture.cpp \
           file://watchdog.hpp \
           file://watchdog.cpp \
           file://test_pattern.hpp \
           file://test_pattern.cpp \
           file://Makefile \
		  "

S = "${WORKDIR}"

do_compile() {
	     oe_runmake
}

do_install() {
	     install -d ${D}${bindir}
	     install -m 0755 mocap-hdmi-drm ${D}${bindir}
}
