#
# mocap-common: shared header-only code for the mocap apps.
#
# Stages vendored third-party single-header libraries (and, in future,
# common drivers) into the sysroot under ${includedir}/mocap/. Consumers
# add `DEPENDS = "mocap-common"` and `#include <mocap/...>`; no -I flag is
# needed since the recipe sysroot's includedir is on the default search path.
#

SUMMARY = "Common header-only code for the mocap apps (argparse, stb, OV9281 pipeline config)"
SECTION = "PETALINUX/libs"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://argparse.hpp \
           file://stb_image_write.h \
           file://ov9281_pipeline.hpp \
          "

S = "${WORKDIR}"

do_install() {
	install -d ${D}${includedir}/mocap
	install -m 0644 ${S}/argparse.hpp ${D}${includedir}/mocap/
	install -m 0644 ${S}/stb_image_write.h ${D}${includedir}/mocap/
	install -m 0644 ${S}/ov9281_pipeline.hpp ${D}${includedir}/mocap/
}

# Header-only: the staged headers belong in the -dev package, and the main
# (target) package is empty. Build-time consumers get the headers via DEPENDS.
FILES:${PN}-dev = "${includedir}/mocap"
ALLOW_EMPTY:${PN} = "1"
