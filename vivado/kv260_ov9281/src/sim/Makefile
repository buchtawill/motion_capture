# Usage:
#   make [TB=<testbench>]        — compile + run  (default TB=tb_isp_histogram)
#   make compile [TB=...]        — compile only
#   make run     [TB=...]        — run only (must compile first)
#   make clean   [TB=...]        — remove output dir for this TB
#   make cleanall                — remove all output dirs

TB       ?= tb_isp_histogram
SNAPSHOT := tb_sim
OUT_DIR  := sim_out/$(TB)

SIM_DIR  := $(abspath .)
# Shared cores live in ip_repo (moved out of the kv260 project). stream_fifo is
# now in ip_repo/common; the ISP cores in ip_repo/isp. The mocap_wrapper sim
# lives in ip_repo/mocap/sim (self-contained; run it from there).
COMMON_DIR := $(abspath ../../../ip_repo/common)
ISP_DIR    := $(abspath ../../../ip_repo/isp)

# ---------------------------------------------------------------------------
# Per-testbench source lists
# ---------------------------------------------------------------------------
SRCS_tb_frame_rate := \
	$(ISP_DIR)/rdl_out/rtl/isp_regs_pkg.sv \
	$(ISP_DIR)/rdl_out/rtl/isp_regs.sv \
	$(COMMON_DIR)/stream_fifo.sv \
	$(ISP_DIR)/isp_histogram.sv \
	$(ISP_DIR)/isp_math_top.sv \
	$(ISP_DIR)/isp_math_wrapper.v \
	$(SIM_DIR)/tb_frame_rate.sv

INCDIRS_tb_frame_rate := $(ISP_DIR)

SRCS_tb_isp_math := \
	$(ISP_DIR)/rdl_out/rtl/isp_regs_pkg.sv \
	$(ISP_DIR)/rdl_out/rtl/isp_regs.sv \
	$(COMMON_DIR)/stream_fifo.sv \
	$(ISP_DIR)/isp_histogram.sv \
	$(ISP_DIR)/isp_math_top.sv \
	$(ISP_DIR)/isp_math_wrapper.v \
	$(SIM_DIR)/tb_isp_math.sv

# Per-TB include directories (used for `include "isp_regs_defines.svh"` etc.)
INCDIRS_tb_isp_math := $(ISP_DIR)

SRCS_tb_stream_fifo := \
	$(COMMON_DIR)/stream_fifo.sv \
	$(SIM_DIR)/tb_stream_fifo.sv

SRCS_tb_isp_histogram := \
	$(COMMON_DIR)/stream_fifo.sv \
	$(ISP_DIR)/isp_histogram.sv \
	$(SIM_DIR)/tb_isp_histogram.sv

# NOTE: the mocap_wrapper sim moved to ip_repo/mocap/sim (self-contained).
#   cd ../../../ip_repo/mocap/sim && make sim   (or: make verify)

SRCS    := $(SRCS_$(TB))
INCDIRS := $(INCDIRS_$(TB))

ifeq ($(SRCS),)
$(error Unknown TB '$(TB)'. Add a SRCS_$(TB) entry to the Makefile.)
endif

XVLOG_INC := $(addprefix -i ,$(INCDIRS))

# ---------------------------------------------------------------------------

.PHONY: all compile sim_fst sim sim_wdb clean

TBS := $(patsubst $(SIM_DIR)/%.sv,%,$(wildcard $(SIM_DIR)/tb_*.sv))

all:
	@echo "Available testbench targets:"
	@$(foreach tb,$(TBS),echo "  make sim TB=$(tb)";)

compile:
	mkdir -p $(OUT_DIR)
	cd $(OUT_DIR) && xvlog --sv $(XVLOG_INC) $(SRCS) --log xvlog.log
	cd $(OUT_DIR) && xelab $(TB) -snapshot $(SNAPSHOT) -debug all --log xelab.log

sim: clean compile sim_fst

sim_fst:
	cd $(OUT_DIR) && xsim $(SNAPSHOT) -tclbatch $(SIM_DIR)/wave_vcd.tcl --log xsim.log && \
	vcd2fst -v ./waves.vcd -f ./$(TB).fst

sim_wdb:
	cd $(OUT_DIR) && xsim $(SNAPSHOT) -tclbatch $(SIM_DIR)/wave_wdb.tcl --wdb $(TB).wdb --log xsim.log

clean:
	rm -rf sim_out
	rm -f *.jou *.log
	rm -rf .Xil xsim.dir

