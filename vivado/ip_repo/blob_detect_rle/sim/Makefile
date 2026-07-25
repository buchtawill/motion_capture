TB       ?= tb_blob_detect_rle
SNAPSHOT := tb_sim
OUT_DIR  := sim_out/$(TB)

HDL_DIR  := $(abspath ../hdl)
RDL_DIR  := $(abspath ../rdl/rdl_out/rtl)
SIM_DIR  := $(abspath .)
MDL_DIR  := $(abspath ../model)

VIVADO_SETTINGS := /tools/Xilinx/Vivado/2024.1/settings64.sh
MOCAP_ENV       := /home/will/Desktop/motion_capture/mocap_env/bin/activate

SRCS_tb_blob_detect_rle := \
	$(RDL_DIR)/blob_detect_rle_regs_pkg.sv \
	$(RDL_DIR)/blob_detect_rle_regs.sv \
	$(HDL_DIR)/stream_fifo.sv \
	$(HDL_DIR)/run_extractor.sv \
	$(HDL_DIR)/row_merger.sv \
	$(HDL_DIR)/blob_table.sv \
	$(HDL_DIR)/blob_detect_rle_top.sv \
	$(HDL_DIR)/blob_detect_rle_wrapper.v \
	$(SIM_DIR)/tb_blob_detect_rle.sv

INCDIRS_tb_blob_detect_rle := $(HDL_DIR)

SRCS    := $(SRCS_$(TB))
INCDIRS := $(INCDIRS_$(TB))

ifeq ($(SRCS),)
$(error Unknown TB '$(TB)'. Add a SRCS_$(TB) entry to the Makefile.)
endif

XVLOG_INC := $(addprefix -i ,$(INCDIRS))

.PHONY: all compile sim_fst sim clean cleanall gen_stim model compare verify

all:
	@echo "Usage: make verify"

compile:
	mkdir -p $(OUT_DIR)
	bash -c ". $(VIVADO_SETTINGS) && cd $(OUT_DIR) && xvlog --sv $(XVLOG_INC) $(SRCS) --log xvlog.log"
	bash -c ". $(VIVADO_SETTINGS) && cd $(OUT_DIR) && xelab $(TB) -snapshot $(SNAPSHOT) -debug all --log xelab.log"

sim_fst:
	bash -c ". $(VIVADO_SETTINGS) && cd $(OUT_DIR) && xsim $(SNAPSHOT) -tclbatch $(SIM_DIR)/wave_vcd.tcl --log xsim.log && vcd2fst -v ./waves.vcd -f ./$(TB).fst"

gen_stim:
	mkdir -p $(OUT_DIR)
	bash -c ". $(MOCAP_ENV) && python $(MDL_DIR)/gen_stimulus.py --output-dir $(OUT_DIR)/"

model:
	bash -c ". $(MOCAP_ENV) && python $(MDL_DIR)/run_models.py --dir $(OUT_DIR)/"

compare:
	bash -c ". $(MOCAP_ENV) && python $(MDL_DIR)/compare_all.py --dir $(OUT_DIR)/"

verify: gen_stim model compile sim_fst compare

clean:
	rm -rf sim_out
	rm -f *.jou *.log
	rm -rf .Xil xsim.dir

cleanall: clean
