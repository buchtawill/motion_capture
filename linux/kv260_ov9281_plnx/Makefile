# PetaLinux build / NFS-rootfs deployment for KV260 (Zynq UltraScale+).
# Run `make config` once to set Root filesystem type = NFS in the menu.

NFS_DIR    ?= /home/will/Desktop/motion_capture/linux/nfs-mount-point
FSBL       ?= zynqmp_fsbl.elf
IMAGES_DIR := images/linux

ROOT_DIR   := $(NFS_DIR)/home/root
DTBO       := build/tmp/deploy/images/xilinx-k26-kv/devicetree/mocap-pipeline-overlay.dtbo
BITBIN     := ../../vivado/kv260_ov9281/kv260_ov9281_proj/kv260_ov9281_proj.bit.bin

.PHONY: all config build package deploy deploy-rootfs deploy-boot deploy-overlay clean

all: build package deploy

config:
	petalinux-config --get-hw-description ./../../vivado/kv260_ov9281/kv260_ov9281_proj/kv260_ov9281_proj.xsa

silent-config:
	petalinux-config --get-hw-description ./../../vivado/kv260_ov9281/kv260_ov9281_proj/kv260_ov9281_proj.xsa --silentconfig

build:
	@start=$$(date +%s); \
	petalinux-build; rc=$$?; \
	end=$$(date +%s); elapsed=$$((end - start)); \
	printf 'petalinux-build took %dm%02ds\n' $$((elapsed / 60)) $$((elapsed % 60)); \
	exit $$rc

package:
	petalinux-package --boot --force \
		--fsbl $(IMAGES_DIR)/$(FSBL) \
		--fpga $(IMAGES_DIR)/system_wrapper.bit \
		--u-boot

deploy: deploy-rootfs deploy-boot #deploy-overlay

deploy-rootfs: $(IMAGES_DIR)/rootfs.tar.gz
	@test -d $(NFS_DIR) || { echo "NFS_DIR $(NFS_DIR) does not exist"; exit 1; }
	sudo /usr/bin/find $(NFS_DIR) -mindepth 1 -delete
	sudo /usr/bin/tar xzf $(IMAGES_DIR)/rootfs.tar.gz -C $(NFS_DIR)

deploy-boot:
	@test -f /tftpboot/pxelinux.cfg/default \
		|| { echo "/tftpboot/pxelinux.cfg/default not found"; exit 1; }
	sed -i '/^[[:space:]]*INITRD\b/Id' /tftpboot/pxelinux.cfg/default

# deploy-overlay: $(DTBO) $(BITBIN)
# 	@test -d $(ROOT_DIR) || { echo "$(ROOT_DIR) does not exist (deploy-rootfs first)"; exit 1; }
# 	sudo /usr/bin/cp $(DTBO) $(ROOT_DIR)/
# 	sudo /usr/bin/cp $(BITBIN) $(ROOT_DIR)/

clean:
	petalinux-build -x distclean
	rm -rf /tftpboot/* ./app_binaries