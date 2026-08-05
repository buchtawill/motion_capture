# =============================================================================
# mocap_wrapper simulation
# =============================================================================
# Usage:
#   make sim         — clean + compile + run (FST waveform)
#   make compile     — compile only
#   make sim_fst     — run only (must compile first)
#   make verify      — gen stimulus + golden models, run, cosim-compare
#   make clean       — remove sim_out/
#
# DUT is the SystemVerilog core `mocap_top` (mocap_wrapper.v is the thin IP-
# Integrator shell and is compiled here too, only to confirm it elaborates).
#
# Cross-IP reuse (all unmodified):
#   - stream_fifo : ../../common
#   - isp_histogram : ../../isp          (two instances, ping-pong)
#   - blob core   : ../../blob_detect_rle/hdl (run_extractor/row_merger/blob_table)
#   - blob + hist Python goldens : ../../blob_detect_rle/model
#
# NOTE on stream_fifo: isp_histogram, the blob core, AND mocap_top all
# instantiate `stream_fifo` by name. Exactly ONE copy of the module may appear
# in the compile list, so we pull the shared common/stream_fifo.sv and must NOT
# also add blob_detect_rle/hdl/stream_fifo.sv (duplicate module definition).
# =============================================================================

TB       ?= tb_mocap_wrapper
SNAPSHOT := tb_sim
OUT_DIR  := sim_out/$(TB)

SIM_DIR    := $(abspath .)
HDL_DIR    := $(abspath ../hdl)
RDL_DIR    := $(abspath ../rdl/rdl_out/rtl)
INC_RDL    := $(abspath ../rdl)
COMMON_DIR := $(abspath ../../common)
ISP_DIR    := $(abspath ../../isp)
BLOB_DIR   := $(abspath ../../blob_detect_rle)
BLOB_MODEL := $(BLOB_DIR)/model

VIVADO_SETTINGS := /tools/Xilinx/Vivado/2024.1/settings64.sh
MOCAP_ENV       := /home/will/Desktop/motion_capture/mocap_env/bin/activate

SRCS_tb_mocap_wrapper := \
	$(RDL_DIR)/mocap_regs_pkg.sv \
	$(RDL_DIR)/mocap_regs.sv \
	$(COMMON_DIR)/stream_fifo.sv \
	$(ISP_DIR)/isp_histogram.sv \
	$(BLOB_DIR)/hdl/run_extractor.sv \
	$(BLOB_DIR)/hdl/row_merger.sv \
	$(BLOB_DIR)/hdl/blob_table.sv \
	$(HDL_DIR)/mocap_top.sv \
	$(HDL_DIR)/mocap_wrapper.v \
	$(SIM_DIR)/tb_mocap_wrapper.sv

INCDIRS_tb_mocap_wrapper := $(INC_RDL)

SRCS    := $(SRCS_$(TB))
INCDIRS := $(INCDIRS_$(TB))

ifeq ($(SRCS),)
$(error Unknown TB '$(TB)'. Add a SRCS_$(TB) entry to the Makefile.)
endif

XVLOG_INC := $(addprefix -i ,$(INCDIRS))

.PHONY: all compile sim_fst sim stim verify clean cleanall

all:
	@echo "Usage: make sim   |   make verify"

# tb_mocap_wrapper reads generated frame_*.hex stimulus via $readmemh at
# runtime, so regenerate it (+ golden models) into OUT_DIR before compiling.
stim:
	mkdir -p $(OUT_DIR)
	bash -c ". $(MOCAP_ENV) && python $(BLOB_MODEL)/gen_stimulus.py --output-dir $(abspath $(OUT_DIR))"
	bash -c ". $(MOCAP_ENV) && python $(BLOB_MODEL)/run_models.py    --dir $(abspath $(OUT_DIR))"
	bash -c ". $(MOCAP_ENV) && python $(BLOB_MODEL)/isp_hist_model.py --dir $(abspath $(OUT_DIR))"

compile: stim
	mkdir -p $(OUT_DIR)
	bash -c ". $(VIVADO_SETTINGS) && cd $(OUT_DIR) && xvlog --sv $(XVLOG_INC) $(SRCS) --log xvlog.log"
	bash -c ". $(VIVADO_SETTINGS) && cd $(OUT_DIR) && xelab $(TB) -snapshot $(SNAPSHOT) -debug all --log xelab.log"

sim_fst:
	bash -c ". $(VIVADO_SETTINGS) && cd $(OUT_DIR) && xsim $(SNAPSHOT) -tclbatch $(SIM_DIR)/wave_vcd.tcl --log xsim.log && vcd2fst -v ./waves.vcd -f ./$(TB).fst"

sim: clean compile sim_fst

# verify: the SV self-checks in the TB are the primary pass/fail gate; the
# cosim compare against the Python goldens is a secondary datapath check.
OUT_ABS := $(abspath $(OUT_DIR))

verify: sim
	@echo "--- cosim compare: blobs ---"
	@bash -c '. $(MOCAP_ENV) && cd $(BLOB_MODEL) && for f in $(OUT_ABS)/blobs_rtl_*.hex; do \
		idx=$$(basename $$f | sed -E "s/blobs_rtl_([0-9]+)\.hex/\1/"); \
		model=$(OUT_ABS)/blobs_model_$$idx.json; \
		if [ -f $$model ]; then echo "[$$idx] blobs:"; python compare.py --rtl-output $$f --model-output $$model || true; fi; \
	done'
	@echo "--- cosim compare: histograms ---"
	@bash -c '. $(MOCAP_ENV) && for f in $(OUT_ABS)/hist_rtl_*.hex; do \
		idx=$$(basename $$f | sed -E "s/hist_rtl_([0-9]+)\.hex/\1/"); \
		model=$(OUT_ABS)/hist_model_$$idx.json; \
		if [ -f $$model ]; then \
			echo "[$$idx] hist: $$(python -c "import json; a=[int(l,16) for l in open('\''$$f'\'')]; b=json.load(open('\''$$model'\''))['\''hist'\'']; print('\''MATCH'\'' if a==b else '\''MISMATCH'\'')")"; \
		fi; \
	done'

clean:
	rm -rf sim_out
	rm -f *.jou *.log
	rm -rf .Xil xsim.dir

cleanall: clean
