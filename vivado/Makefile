all:
	@echo "project: make the project"
	@echo "build:   Run synthesis, implementation, bitstream"

clean:
	rm -f vivado*.jou vivado*.log
	rm -rf .Xil NA zybo_ov9281

zybo_ov9281: create_proj.tcl
	vivado -mode batch -source create_proj.tcl

project: clean zybo_ov9281

build: zybo_ov9281
	vivado -mode batch -source export_bitstream.tcl

.PHONY: all clean project build
