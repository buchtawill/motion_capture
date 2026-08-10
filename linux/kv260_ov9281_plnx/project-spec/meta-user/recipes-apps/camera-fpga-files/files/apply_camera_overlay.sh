#!/bin/sh
# Load the OV9281 / mocap capture pipeline: PL bitstream + device-tree overlay.
#
# The FPGA region accepts only ONE overlay at a time. Re-running this script
# (or booting with an overlay already applied) makes a fresh `fpgautil -o` fail
# with "Region already has overlay applied" (err -22). So we must FULLY remove
# any resident overlay before applying. `fpgautil -R` removes the last overlay
# it applied, but does not clear overlays applied by a previous process/boot;
# the reliable teardown is to rmdir every configfs overlay node.
set -e

BIT=/home/root/kv260_ov9281_proj.bit.bin
DTBO=/home/root/mocap-pipeline-overlay.dtbo
OVL_DIR=/sys/kernel/config/device-tree/overlays

# Tear down any resident overlay(s) so the region is free.
if [ -d "$OVL_DIR" ]; then
	for o in "$OVL_DIR"/*/; do
		[ -d "$o" ] && rmdir "$o" 2>/dev/null || true
	done
fi

# Single apply: bitstream through the FPGA manager + overlay in one shot.
/usr/bin/fpgautil -b "$BIT" -o "$DTBO"
