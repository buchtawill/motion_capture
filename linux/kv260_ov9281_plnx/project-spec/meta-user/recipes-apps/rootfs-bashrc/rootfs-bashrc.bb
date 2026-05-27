#
# Install a default .bashrc + .profile into /home/root and /home/petalinux.
#

SUMMARY = "Default .bashrc/.profile for the root and petalinux home directories"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://dot-bashrc \
           file://dot-profile \
          "

S = "${WORKDIR}"

do_install() {
    install -d ${D}/home/root
    install -m 0644 ${WORKDIR}/dot-bashrc ${D}/home/root/.bashrc
    install -m 0644 ${WORKDIR}/dot-profile ${D}/home/root/.profile

    install -d ${D}/home/petalinux
    install -m 0644 ${WORKDIR}/dot-bashrc ${D}/home/petalinux/.bashrc
    install -m 0644 ${WORKDIR}/dot-profile ${D}/home/petalinux/.profile
}

FILES:${PN} += "/home/root/.bashrc /home/root/.profile \
                /home/petalinux/.bashrc /home/petalinux/.profile"

# Installed 0644 (world-readable), so both root and the petalinux user can read
# it without any postinst chown. (OE 2024.1 dropped 'exit 1' postinst deferral;
# use pkg_postinst_ontarget:${PN} if on-target work is ever genuinely needed.)
