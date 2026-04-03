HDL_DIR  := ../src/hdl
SNAPSHOT := tb_sim
TOP      := tb_frame_rate

SRCS := \
	$(HDL_DIR)/frame_rate_counter.sv \
	$(HDL_DIR)/tb_frame_rate.sv

.PHONY: all compile run clean

all: compile run

compile:
	xvlog --sv $(SRCS)
	xelab $(TOP) -snapshot $(SNAPSHOT) -debug typical

run:
	xsim $(SNAPSHOT) --runall

clean:
	rm -rf xsim.dir xsim.jou xsim.log \
	       xvlog.log xvlog.pb \
	       xelab.log xelab.pb \
	       *.wdb *.str .Xil
