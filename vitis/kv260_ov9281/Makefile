
build_all:
	vitis -s scripts/create_vitis.py

app_and_boot:
	vitis -s scripts/build_app_only.py
	xsdb scripts/program_and_boot.tcl

app_only:
	vitis -s scripts/build_app_only.py

boot:
	xsdb scripts/program_and_boot.tcl

boot_fpga:
	xsdb scripts/boot_fpga.tcl

boot_fw:
	xsdb scripts/boot_fw.tcl

dump_image:
	xsdb scripts/dump_fb.tcl

clean:
	rm -rf workspace
	rm -rf .wsdata logs

.PHONY: build_all app_and_boot app_only boot boot_fpga boot_fw dump_image clean
