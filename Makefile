# Top-level Makefile for motion_capture project

PROJ ?= zybo_ov9281

all:
	@echo "Makefile for vivado and vitis build"
	@echo "Projects are (zybo_ov9281, zybo_ov5640)"
	@echo "    vitis_build   PROJ=<project>    Create vivado project, build, export, create and build vitis"
	@echo "    vivado_build  PROJ=<project>    Create vivado project, build, export bitsream"

# Identify the directory target
# Only run if directory does not exist
vivado_build:
	$(MAKE) -C vivado/$(PROJ) clean
	$(MAKE) -C vivado/$(PROJ) build

vitis_build: vivado/$(PROJ)
	$(MAKE) -C vivado/$(PROJ) build
	$(MAKE) -C vitis/$(PROJ) build_vitis

clean:
	$(MAKE) -C vivado clean
	$(MAKE) -C vitis clean

.PHONY: all vitis_build clean