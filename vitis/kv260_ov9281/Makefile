
app_and_boot:
	vitis -s scripts/build_app_only.py
	xsdb scripts/program_and_boot.tcl


boot: build_all
	xsdb scripts/program_and_boot.tcl

build_all:
	vitis -s scripts/create_vitis.py

dump_image:
	xsdb scripts/dump_fb.tcl

clean:
	rm -rf workspace
	rm -rf .wsdata logs