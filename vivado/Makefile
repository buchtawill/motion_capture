PROJ ?= zybo_ov9281

all:
	@echo "Makefile for vivado projects."
	@echo "    proj  PROJ=<project>    Create specified vivado project (zybo_ov9281, zybo_ov5640)"
	@echo "    build PROJ=<project>    Build and export the bitstream for the specified project"

clean:
	$(MAKE) -C zybo_ov5640 clean
	$(MAKE) -C zybo_ov9281 clean

proj:
	$(MAKE) -C $(PROJ) proj

build:
	$(MAKE) -C $(PROJ) build
