#
# This file is the mocap-perf recipe.
#

SUMMARY = "Max-FPS V4L2 capture benchmark (software FPS measurement)"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://mocap-perf.cpp \
           file://argparse.hpp \
           file://Makefile \
		  "

S = "${WORKDIR}"

do_compile() {
	     oe_runmake
}

do_install() {
	     install -d ${D}${bindir}
	     install -m 0755 mocap-perf ${D}${bindir}
}
