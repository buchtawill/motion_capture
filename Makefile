# Top-level Makefile for motion_capture project (KV260 / OV9281)

VIVADO_DIR := vivado/kv260_ov9281
VITIS_DIR  := vitis/kv260_ov9281
LINUX_DIR  := linux/kv260_ov9281_plnx

all:
	@echo "Available targets:"
	@echo "  vivado     — clean + bitstream + bit-bin"
	@echo "  vitis      — clean + build_all"
	@echo "  linux      — clean + build + package + deploy"
	@echo "  hw         — vivado + vitis"
	@echo "  full       — vivado + vitis + linux"

vivado:
	$(MAKE) -C $(VIVADO_DIR) clean
	$(MAKE) -C $(VIVADO_DIR) bitstream
	$(MAKE) -C $(VIVADO_DIR) bit-bin

vitis:
	$(MAKE) -C $(VITIS_DIR) clean
	$(MAKE) -C $(VITIS_DIR) build_all

linux:
	$(MAKE) -C $(LINUX_DIR) clean
	$(MAKE) -C $(LINUX_DIR) all

hw: vivado vitis

full: vivado vitis linux

clean:
	$(MAKE) -C $(VIVADO_DIR) clean
	$(MAKE) -C $(VITIS_DIR) clean
	$(MAKE) -C $(LINUX_DIR) clean

.PHONY: all vivado vitis linux hw full clean
