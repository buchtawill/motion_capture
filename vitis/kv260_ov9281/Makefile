
app_and_boot:
	vitis -s s scripts/build_app_only.py
	xsdb scripts/program_and_boot.tcl


boot: build_all
	xsdb scripts/program_and_boot.tcl

build_all:
	vitis -s scripts/create_vitis.py

mem_dump:

clean:
	rm -rf workspace
	rm -rf .wsdata logs