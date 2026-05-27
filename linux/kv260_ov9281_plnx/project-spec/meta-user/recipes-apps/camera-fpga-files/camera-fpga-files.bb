#
# Install the FPGA bitstream (.bit.bin) and the camera device-tree overlay
# (.dtbo) into /home/root so they can be loaded at runtime with fpgautil.
#
# The .bit.bin is tracked as a static source (regenerate from Vivado and copy
# into files/ whenever the PL design is re-synthesized). The .dtbo is pulled
# from the device-tree recipe's deploy output so it always matches the build.
#

SUMMARY = "Camera FPGA bitstream and device-tree overlay for /home/root"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://kv260_ov9281_proj.bit.bin"

S = "${WORKDIR}"

# The overlay comes out of the device-tree recipe's do_deploy.
do_install[depends] += "device-tree:do_deploy"

DTBO = "${DEPLOY_DIR_IMAGE}/devicetree/camera-overlay.dtbo"

do_install() {
    install -d ${D}/home/root
    install -m 0644 ${WORKDIR}/kv260_ov9281_proj.bit.bin ${D}/home/root/
    install -m 0644 ${DTBO} ${D}/home/root/camera-overlay.dtbo
}

FILES:${PN} += "/home/root/kv260_ov9281_proj.bit.bin /home/root/camera-overlay.dtbo"

# Pure data files, nothing to QA for architecture.
INSANE_SKIP:${PN} += "arch"
