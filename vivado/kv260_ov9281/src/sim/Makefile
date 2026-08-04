# Usage:
#   make [TB=<testbench>]        — compile + run  (default TB=tb_isp_histogram)
#   make compile [TB=...]        — compile only
#   make run     [TB=...]        — run only (must compile first)
#   make clean   [TB=...]        — remove output dir for this TB
#   make cleanall                — remove all output dirs

TB       ?= tb_isp_histogram
SNAPSHOT := tb_sim
OUT_DIR  := sim_out/$(TB)

HDL_DIR  := $(abspath ../hdl)
SIM_DIR  := $(abspath .)
# ISP cores now live in the shared ip_repo (moved out of the kv260 project).
ISP_DIR  := $(abspath ../../../ip_repo/isp)
MOCAP_DIR := $(abspath ../hdl/mocap)
BLOB_DIR  := $(abspath ../../../ip_repo/blob_detect_rle)

# ---------------------------------------------------------------------------
# Per-testbench source lists
# ---------------------------------------------------------------------------
SRCS_tb_frame_rate := \
	$(ISP_DIR)/rdl_out/rtl/isp_regs_pkg.sv \
	$(ISP_DIR)/rdl_out/rtl/isp_regs.sv \
	$(HDL_DIR)/stream_fifo.sv \
	$(ISP_DIR)/isp_histogram.sv \
	$(ISP_DIR)/isp_math_top.sv \
	$(ISP_DIR)/isp_math_wrapper.v \
	$(SIM_DIR)/tb_frame_rate.sv

INCDIRS_tb_frame_rate := $(ISP_DIR)

SRCS_tb_isp_math := \
	$(ISP_DIR)/rdl_out/rtl/isp_regs_pkg.sv \
	$(ISP_DIR)/rdl_out/rtl/isp_regs.sv \
	$(HDL_DIR)/stream_fifo.sv \
	$(ISP_DIR)/isp_histogram.sv \
	$(ISP_DIR)/isp_math_top.sv \
	$(ISP_DIR)/isp_math_wrapper.v \
	$(SIM_DIR)/tb_isp_math.sv

# Per-TB include directories (used for `include "isp_regs_defines.svh"` etc.)
INCDIRS_tb_isp_math := $(ISP_DIR)

SRCS_tb_stream_fifo := \
	$(HDL_DIR)/stream_fifo.sv \
	$(SIM_DIR)/tb_stream_fifo.sv

SRCS_tb_isp_histogram := \
	$(HDL_DIR)/stream_fifo.sv \
	$(ISP_DIR)/isp_histogram.sv \
	$(SIM_DIR)/tb_isp_histogram.sv

# mocap_wrapper: mocap regblock + isp_histogram (snoop) + blob_detect_rle core
# (reused unmodified) + the wrapper itself. NOTE: only the blob core's
# stream_fifo.sv is included here -- isp_histogram internally reuses the same
# module name, so pulling in both HDL_DIR's and BLOB_DIR's stream_fifo.sv
# would error on duplicate module definition.
SRCS_tb_mocap_wrapper := \
	$(MOCAP_DIR)/rdl_out/rtl/mocap_regs_pkg.sv \
	$(MOCAP_DIR)/rdl_out/rtl/mocap_regs.sv \
	$(BLOB_DIR)/hdl/stream_fifo.sv \
	$(ISP_DIR)/isp_histogram.sv \
	$(BLOB_DIR)/hdl/run_extractor.sv \
	$(BLOB_DIR)/hdl/row_merger.sv \
	$(BLOB_DIR)/hdl/blob_table.sv \
	$(MOCAP_DIR)/mocap_wrapper.sv \
	$(SIM_DIR)/tb_mocap_wrapper.sv

INCDIRS_tb_mocap_wrapper := $(MOCAP_DIR)

SRCS    := $(SRCS_$(TB))
INCDIRS := $(INCDIRS_$(TB))

ifeq ($(SRCS),)
$(error Unknown TB '$(TB)'. Add a SRCS_$(TB) entry to the Makefile.)
endif

XVLOG_INC := $(addprefix -i ,$(INCDIRS))

# ---------------------------------------------------------------------------

.PHONY: all compile sim_fst sim sim_wdb clean verify

TBS := $(patsubst $(SIM_DIR)/%.sv,%,$(wildcard $(SIM_DIR)/tb_*.sv))

all:
	@echo "Available testbench targets:"
	@$(foreach tb,$(TBS),echo "  make sim TB=$(tb)";)

compile:
ifeq ($(TB),tb_mocap_wrapper)
	$(MAKE) stim_mocap
endif
	mkdir -p $(OUT_DIR)
	cd $(OUT_DIR) && xvlog --sv $(XVLOG_INC) $(SRCS) --log xvlog.log
	cd $(OUT_DIR) && xelab $(TB) -snapshot $(SNAPSHOT) -debug all --log xelab.log

# tb_mocap_wrapper needs generated frame_*.hex stimulus (not committed to the
# repo) at $(OUT_DIR) before xsim runs $readmemh on it. `sim`'s `clean` step
# wipes $(OUT_DIR), so this regenerates it as part of `compile` (harmless
# no-op for every other TB, which self-generates stimulus in-TB).
stim_mocap:
	mkdir -p $(OUT_DIR)
	cd $(BLOB_DIR)/model && python gen_stimulus.py --output-dir $(abspath $(OUT_DIR))
	cd $(BLOB_DIR)/model && python run_models.py --dir $(abspath $(OUT_DIR))
	cd $(BLOB_DIR)/model && python isp_hist_model.py --dir $(abspath $(OUT_DIR))

sim: clean compile sim_fst

sim_fst:
	cd $(OUT_DIR) && xsim $(SNAPSHOT) -tclbatch $(SIM_DIR)/wave_vcd.tcl --log xsim.log && \
	vcd2fst -v ./waves.vcd -f ./$(TB).fst

sim_wdb:
	cd $(OUT_DIR) && xsim $(SNAPSHOT) -tclbatch $(SIM_DIR)/wave_wdb.tcl --wdb $(TB).wdb --log xsim.log

# verify: regenerate stimulus + golden models into sim_out/tb_mocap_wrapper,
# run tb_mocap_wrapper (the SV self-checks are the primary pass/fail gate),
# then cosim-compare the RTL-dumped blobs_rtl_*.hex / hist_rtl_*.hex against
# the Python golden models as a secondary check.
VERIFY_OUT_DIR_ABS := $(abspath sim_out/tb_mocap_wrapper)
BLOB_MODEL_DIR      := $(BLOB_DIR)/model

verify:
	rm -rf $(VERIFY_OUT_DIR_ABS)
	mkdir -p $(VERIFY_OUT_DIR_ABS)
	cd $(BLOB_MODEL_DIR) && python gen_stimulus.py --output-dir $(VERIFY_OUT_DIR_ABS)
	cd $(BLOB_MODEL_DIR) && python run_models.py --dir $(VERIFY_OUT_DIR_ABS)
	cd $(BLOB_MODEL_DIR) && python isp_hist_model.py --dir $(VERIFY_OUT_DIR_ABS)
	$(MAKE) compile TB=tb_mocap_wrapper
	$(MAKE) sim_fst TB=tb_mocap_wrapper
	@echo "--- cosim compare: blobs ---"
	@cd $(BLOB_MODEL_DIR) && for f in $(VERIFY_OUT_DIR_ABS)/blobs_rtl_*.hex; do \
		idx=$$(basename $$f | sed -E 's/blobs_rtl_([0-9]+)\.hex/\1/'); \
		model=$(VERIFY_OUT_DIR_ABS)/blobs_model_$$idx.json; \
		if [ -f $$model ]; then \
			echo "[$$idx] blobs:"; \
			python compare.py --rtl-output $$f --model-output $$model || true; \
		fi; \
	done
	@echo "--- cosim compare: histograms ---"
	@cd $(BLOB_MODEL_DIR) && for f in $(VERIFY_OUT_DIR_ABS)/hist_rtl_*.hex; do \
		idx=$$(basename $$f | sed -E 's/hist_rtl_([0-9]+)\.hex/\1/'); \
		model=$(VERIFY_OUT_DIR_ABS)/hist_model_$$idx.json; \
		if [ -f $$model ]; then \
			echo "[$$idx] hist: $$(python -c "import json,sys; a=[int(l,16) for l in open('$$f')]; b=json.load(open('$$model'))['hist']; print('MATCH' if a==b else 'MISMATCH')")"; \
		fi; \
	done

clean:
	rm -rf sim_out
	rm -f *.jou *.log
	rm -rf .Xil xsim.dir

