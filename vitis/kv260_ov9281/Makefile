
boot: clean build_vitis
	xsdb boot_kv260_jtag.tcl

build_vitis:
	vitis -s create_vitis.py

clean:
	rm -rf workspace
	rm -rf .wsdata