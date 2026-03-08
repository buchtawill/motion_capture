
app_and_boot:
	vitis -s build_app_only.py
	xsdb program_and_boot.tcl


boot: clean vitis_build
	xsdb program_and_boot.tcl

vitis_build:
	vitis -s create_vitis.py

clean:
	rm -rf workspace
	rm -rf .wsdata logs