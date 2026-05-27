FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append = " file://bsp.cfg"
KERNEL_FEATURES:append = " bsp.cfg"
SRC_URI += "file://user_2026-05-14-03-02-00.cfg \
            file://user_2026-05-26-01-29-00.cfg \
            file://user_2026-05-26-01-47-00.cfg \
            "

# Mono RAW8 (Y8_1X8) support in the Xilinx CSI-2 RX, for the OV9281.
SRC_URI += "file://0001-xilinx-csi2rxss-add-Y8_1X8-mono-RAW8-mbus-code.patch"

