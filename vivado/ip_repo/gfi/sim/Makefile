TB       ?= tb_gfi
SNAPSHOT := tb_sim
OUT_DIR  := sim_out/$(TB)

HDL_DIR  := $(abspath ../hdl)
SIM_DIR  := $(abspath .)
MDL_DIR  := $(abspath ../model)

VIVADO_SETTINGS := /tools/Xilinx/Vivado/2024.1/settings64.sh
MOCAP_ENV       := /home/will/Desktop/motion_capture/mocap_env/bin/activate

SRCS_tb_gfi := \
	$(HDL_DIR)/gfi.sv \
	$(SIM_DIR)/tb_gfi.sv

INCDIRS_tb_gfi := $(HDL_DIR)

SRCS    := $(SRCS_$(TB))
INCDIRS := $(INCDIRS_$(TB))

ifeq ($(SRCS),)
$(error Unknown TB '$(TB)'. Add a SRCS_$(TB) entry to the Makefile.)
endif

XVLOG_INC := $(addprefix -i ,$(INCDIRS))

.PHONY: all compile sim sim_fst gen_stim model clean cleanall verify

all:
	@echo "Usage: make verify"

gen_stim:
	mkdir -p $(OUT_DIR)
	bash -c ". $(MOCAP_ENV) && python $(MDL_DIR)/gen_and_expect.py --output-dir $(OUT_DIR)/"

model: gen_stim

compile:
	mkdir -p $(OUT_DIR)
	bash -c ". $(VIVADO_SETTINGS) && cd $(OUT_DIR) && xvlog --sv $(XVLOG_INC) $(SRCS) --log xvlog.log"
	bash -c ". $(VIVADO_SETTINGS) && cd $(OUT_DIR) && xelab $(TB) -snapshot $(SNAPSHOT) -debug all --log xelab.log"

sim:
	bash -c ". $(VIVADO_SETTINGS) && cd $(OUT_DIR) && xsim $(SNAPSHOT) -runall --log xsim.log" | tee $(OUT_DIR)/sim_stdout.log

sim_fst:
	bash -c ". $(VIVADO_SETTINGS) && cd $(OUT_DIR) && xsim $(SNAPSHOT) -tclbatch $(SIM_DIR)/wave_vcd.tcl --log xsim.log && vcd2fst -v ./waves.vcd -f ./$(TB).fst"

verify: gen_stim compile sim
	@grep -q "SUCCESS: ALL TESTS PASSED" $(OUT_DIR)/sim_stdout.log && echo "VERIFY: PASS" || (echo "VERIFY: FAIL"; exit 1)

clean:
	rm -rf sim_out
	rm -f *.jou *.log
	rm -rf .Xil xsim.dir

cleanall: clean
