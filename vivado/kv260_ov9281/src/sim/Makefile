HDL_DIR  := ../hdl
SNAPSHOT := tb_sim
TOP      := tb_frame_rate

SRCS := \
	$(HDL_DIR)/isp/isp_top.sv \
	$(HDL_DIR)/isp/isp_wrapper.v \
	./tb_frame_rate.sv

.PHONY: all compile run clean

all: compile run

compile:
	xvlog --sv $(SRCS)
	xelab $(TOP) -snapshot $(SNAPSHOT) -debug typical

run:
	xsim $(SNAPSHOT) --runall

clean:
	rm -rf xsim.dir *.jou *.log \
		xvlog.log xvlog.pb \
		xelab.log xelab.pb \
		*.wdb *.str .Xil
