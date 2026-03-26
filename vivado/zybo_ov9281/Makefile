all:
	@echo "project: clean and regenerate the Vivado project"
	@echo "bitstream:   regenerate project, then run synthesis/implementation/bitstream"

clean:
	rm -f vivado*.jou vivado*.log
	rm -rf .Xil NA zybo_ov9281_proj

project: clean
	vivado -mode batch -source create_proj.tcl

bitstream: project
	vivado -mode batch -source export_bitstream.tcl

.PHONY: all clean project bitstream
