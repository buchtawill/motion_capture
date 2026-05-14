# Use case: if you build, you also regenerate the project directory

PROJ_DIR := kv260_ov9281_proj
BIT      := $(PROJ_DIR)/$(PROJ_DIR).bit
BIT_BIN  := $(PROJ_DIR)/$(PROJ_DIR).bit.bin
BIF      := $(PROJ_DIR)/fpgautil.bif

all:
	@echo "project:   make the project"
	@echo "bitstream: Run synthesis, implementation, bitstream"
	@echo "bit-bin:   Generate .bit.bin for fpgautil -b on KV260"

clean:
	rm -f vivado*.jou vivado*.log
	rm -rf .Xil NA $(PROJ_DIR)

# Target is the directory itself. MAKE will check if directory exists.
$(PROJ_DIR):
	@if [ ! -d "$(PROJ_DIR)" ]; then \
		vivado -mode batch -source create_proj.tcl; \
	else \
		echo "Project directory '$(PROJ_DIR)' already exists. Skipping creation."; \
	fi

proj: clean $(PROJ_DIR)

bitstream: $(PROJ_DIR)
	vivado -mode batch -source export_bitstream.tcl

bit-bin: $(BIT_BIN)

$(BIT_BIN): $(BIT)
	@printf 'all:\n{\n $(notdir $(BIT))\n}\n' > $(BIF)
	cd $(PROJ_DIR) && bootgen -arch zynqmp -image $(notdir $(BIF)) -process_bitstream bin 
	@echo "Generated $(BIT_BIN)"

.PHONY: all clean proj bitstream bit-bin
