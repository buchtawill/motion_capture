FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append = " file://bsp.cfg"
KERNEL_FEATURES:append = " bsp.cfg"
SRC_URI += "file://user_2026-05-14-03-02-00.cfg \
            file://user_2026-05-26-01-29-00.cfg \
            file://user_2026-05-26-01-47-00.cfg \
            "

