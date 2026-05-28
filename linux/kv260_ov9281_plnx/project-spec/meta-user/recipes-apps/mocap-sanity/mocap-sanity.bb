#
# This file is the mocap-sanity recipe.
#

SUMMARY = "Single-frame V4L2 capture sanity app (8-bpp RAW)"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

DEPENDS = "mocap-common"

SRC_URI = "file://mocap-sanity.cpp \
           file://Makefile \
		  "

S = "${WORKDIR}"

do_compile() {
	     oe_runmake
}

do_install() {
	     install -d ${D}${bindir}
	     install -m 0755 mocap-sanity ${D}${bindir}
}
