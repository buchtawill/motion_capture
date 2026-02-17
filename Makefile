# Top-level Makefile for motion_capture project

.PHONY: all vitis_build clean

all: vitis_build

# Identify the directory target
# Only run if directory does not exist
vivado/zybo_ov9281:
	@if [ ! -d "vivado/zybo_ov9281" ]; then \
		$(MAKE) -C vivado project; \
	fi

vitis_build: vivado/zybo_ov9281
	$(MAKE) -C vivado build
	$(MAKE) -C vitis build

clean:
	$(MAKE) -C vivado clean
	$(MAKE) -C vitis clean
