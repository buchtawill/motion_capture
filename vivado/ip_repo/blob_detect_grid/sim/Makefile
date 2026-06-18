# Usage:
#   make [TB=<testbench>]        — compile + run  (default TB=tb_blob_detect_grid)
#   make compile [TB=...]        — compile only
#   make run     [TB=...]        — run only (must compile first)
#   make clean   [TB=...]        — remove output dir for this TB
#   make cleanall                — remove all output dirs
#   make gen_stim                — generate stimulus hex files
#   make model                   — run the Python reference model
#   make compare                 — run the Python comparator
#   make verify                  — full flow: gen_stim -> model -> sim -> compare

TB       ?= tb_blob_detect_grid
SNAPSHOT := tb_sim
OUT_DIR  := sim_out/$(TB)

HDL_DIR  := $(abspath ../hdl)
RDL_DIR  := $(abspath ../rdl/rdl_out/rtl)
SIM_DIR  := $(abspath .)
MDL_DIR  := $(abspath ../model)

VIVADO_SETTINGS := /tools/Xilinx/Vivado/2024.1/settings64.sh
MOCAP_ENV       := /home/will/Desktop/motion_capture/mocap_env/bin/activate

# ---------------------------------------------------------------------------
# Per-testbench source lists
# ---------------------------------------------------------------------------
SRCS_tb_blob_detect_grid := \
	$(RDL_DIR)/blob_detect_grid_regs_pkg.sv \
	$(RDL_DIR)/blob_detect_grid_regs.sv \
	$(HDL_DIR)/stream_fifo.sv \
	$(HDL_DIR)/cell_accumulator.sv \
	$(HDL_DIR)/grid_scanner.sv \
	$(HDL_DIR)/blob_emitter.sv \
	$(HDL_DIR)/blob_detect_grid_top.sv \
	$(HDL_DIR)/blob_detect_grid_wrapper.v \
	$(SIM_DIR)/tb_blob_detect_grid.sv

INCDIRS_tb_blob_detect_grid := $(HDL_DIR)

SRCS    := $(SRCS_$(TB))
INCDIRS := $(INCDIRS_$(TB))

ifeq ($(SRCS),)
$(error Unknown TB '$(TB)'. Add a SRCS_$(TB) entry to the Makefile.)
endif

XVLOG_INC := $(addprefix -i ,$(INCDIRS))

# ---------------------------------------------------------------------------

.PHONY: all compile sim_fst sim sim_wdb clean cleanall gen_stim model compare verify

TBS := $(patsubst $(SIM_DIR)/%.sv,%,$(wildcard $(SIM_DIR)/tb_*.sv))

all:
	@echo "Available testbench targets:"
	@$(foreach tb,$(TBS),echo "  make sim TB=$(tb)";)

compile:
	mkdir -p $(OUT_DIR)
	bash -c ". $(VIVADO_SETTINGS) && cd $(OUT_DIR) && xvlog --sv $(XVLOG_INC) $(SRCS) --log xvlog.log"
	bash -c ". $(VIVADO_SETTINGS) && cd $(OUT_DIR) && xelab $(TB) -snapshot $(SNAPSHOT) -debug all --log xelab.log"

sim: clean compile sim_fst

sim_fst:
	bash -c ". $(VIVADO_SETTINGS) && cd $(OUT_DIR) && xsim $(SNAPSHOT) -tclbatch $(SIM_DIR)/wave_vcd.tcl --log xsim.log && vcd2fst -v ./waves.vcd -f ./$(TB).fst"

sim_wdb:
	bash -c ". $(VIVADO_SETTINGS) && cd $(OUT_DIR) && xsim $(SNAPSHOT) -tclbatch $(SIM_DIR)/wave_wdb.tcl --wdb $(TB).wdb --log xsim.log"

gen_stim:
	mkdir -p $(OUT_DIR)
	bash -c ". $(MOCAP_ENV) && python $(MDL_DIR)/gen_stimulus.py --output-dir $(OUT_DIR)/"

model:
	mkdir -p $(OUT_DIR)
	bash -c ". $(MOCAP_ENV) && python $(MDL_DIR)/blob_detect_grid_model.py --output-dir $(OUT_DIR)/"

compare:
	bash -c ". $(MOCAP_ENV) && python $(MDL_DIR)/compare.py --rtl-out $(OUT_DIR)/blobs_rtl.hex --model-out $(OUT_DIR)/blobs_model.hex"

verify: gen_stim model compile sim_fst compare

clean:
	rm -rf sim_out
	rm -f *.jou *.log
	rm -rf .Xil xsim.dir

cleanall: clean
