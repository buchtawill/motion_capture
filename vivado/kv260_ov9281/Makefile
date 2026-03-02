# Use case: if you build, you also regenerate the project directory

all:
	@echo "project: make the project"
	@echo "build:   Run synthesis, implementation, bitstream"

clean:
	rm -f vivado*.jou vivado*.log
	rm -rf .Xil NA kv260_ov9281_proj

# Target is the directory itself. MAKE will check if directory exists.
kv260_ov9281_proj:
	@if [ ! -d "kv260_ov9281_proj" ]; then \
		vivado -mode batch -source create_proj.tcl; \
	else \
		echo "Project directory 'kv260_ov9281_proj' already exists. Skipping creation."; \
	fi

proj: clean kv260_ov9281_proj

build: kv260_ov9281_proj
	vivado -mode batch -source export_bitstream.tcl

.PHONY: all clean project build
