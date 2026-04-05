# Usage:
#   make [TB=<testbench>]        — compile + run  (default TB=tb_isp_math)
#   make compile [TB=...]        — compile only
#   make run     [TB=...]        — run only (must compile first)
#   make clean   [TB=...]        — remove output dir for this TB
#   make cleanall                — remove all output dirs

TB       ?= tb_isp_math
SNAPSHOT := tb_sim
OUT_DIR  := sim_out/$(TB)

HDL_DIR  := $(abspath ../hdl)
SIM_DIR  := $(abspath .)

# ---------------------------------------------------------------------------
# Per-testbench source lists
# ---------------------------------------------------------------------------
SRCS_tb_frame_rate := \
	$(HDL_DIR)/isp/isp_top.sv \
	$(HDL_DIR)/isp/isp_wrapper.v \
	$(SIM_DIR)/tb_frame_rate.sv

SRCS_tb_isp_math := \
	$(HDL_DIR)/isp/rdl_out/rtl/isp_regs_pkg.sv \
	$(HDL_DIR)/isp/rdl_out/rtl/isp_regs.sv \
	$(HDL_DIR)/isp/isp_math_top.sv \
	$(HDL_DIR)/isp/isp_math_wrapper.v \
	$(SIM_DIR)/tb_isp_math.sv

SRCS := $(SRCS_$(TB))

ifeq ($(SRCS),)
$(error Unknown TB '$(TB)'. Add a SRCS_$(TB) entry to the Makefile.)
endif

# ---------------------------------------------------------------------------

.PHONY: all compile run clean cleanall

all: compile run

compile:
	mkdir -p $(OUT_DIR)
	cd $(OUT_DIR) && xvlog --sv $(SRCS) --log xvlog.log
	cd $(OUT_DIR) && xelab $(TB) -snapshot $(SNAPSHOT) -debug typical --log xelab.log

run:
	cd $(OUT_DIR) && xsim $(SNAPSHOT) --runall --log xsim.log

clean:
	rm -rf $(OUT_DIR)

cleanall:
	rm -rf sim_out
