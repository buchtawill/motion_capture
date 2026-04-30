# PetaLinux build / NFS-rootfs deployment for KV260 (Zynq UltraScale+).
# Run `make config` once to set Root filesystem type = NFS in the menu.

NFS_DIR    ?= /home/will/Desktop/motion_capture/linux/nfs-mount-point
BOOT_DEST  ?= /srv/tftp
FSBL       ?= zynqmp_fsbl.elf
IMAGES_DIR := images/linux

.PHONY: all config build package deploy deploy-rootfs deploy-boot clean

all: build package deploy

config:
	petalinux-config --get-hw-description ./../../vivado/kv260_ov9281/kv260_ov9281_proj/kv260_ov9281_proj.xsa

build:
	petalinux-build

package:
	petalinux-package --boot --force \
		--fsbl $(IMAGES_DIR)/$(FSBL) \
		--u-boot

deploy: deploy-rootfs deploy-boot

deploy-rootfs: $(IMAGES_DIR)/rootfs.tar.gz
	@test -d $(NFS_DIR) || { echo "NFS_DIR $(NFS_DIR) does not exist"; exit 1; }
	sudo rm -rf $(NFS_DIR)/*
	sudo tar xzf $(IMAGES_DIR)/rootfs.tar.gz -C $(NFS_DIR)

clean:
	petalinux-build -x mrproper
