# =============================================================
# PeakRDL Makefile for OV9281 ISP Register Map
# =============================================================
# Requirements:
#   pip install peakrdl 
#
# Targets:
#   make all      - generate all output products
#   make rtl      - SystemVerilog AXI4-Lite slave
#   make header   - C header for driver
#   make html     - HTML register map documentation
#   make clean    - remove all generated outputs
# =============================================================

RDL_FILE    := rdl/isp_regs.rdl
TOP         := isp_regs

RTL_DIR     := rdl/rtl
SW_DIR      := rdl/sw
HTML_DIR    := rdl/html

PEAKRDL     := peakrdl

# =============================================================
# Default target
# =============================================================
.PHONY: all
all: rtl header html
	@echo ""
	@echo "All output products generated:"
	@echo "  RTL:          $(RTL_DIR)/"
	@echo "  C header:     $(SW_DIR)/"
	@echo "  Docs:         $(HTML_DIR)/index.html"

# =============================================================
# SystemVerilog AXI4-Lite slave
# =============================================================
.PHONY: rtl
rtl: $(RTL_DIR)/$(TOP).sv

$(RTL_DIR)/$(TOP).sv: $(RDL_FILE)
	@mkdir -p $(RTL_DIR)
	$(PEAKRDL) regblock $< -o $(RTL_DIR) --top $(TOP)
	@echo "RTL generated in $(RTL_DIR)/"

# =============================================================
# C header for Linux / bare-metal driver
# =============================================================
.PHONY: header
header: $(SW_DIR)/$(TOP).h

$(SW_DIR)/$(TOP).h: $(RDL_FILE)
	@mkdir -p $(SW_DIR)
	$(PEAKRDL) c-header $< -o $(SW_DIR)/$(TOP).h --top $(TOP)
	@echo "C header generated in $(SW_DIR)/"

# =============================================================
# HTML register map documentation
# =============================================================
.PHONY: html
html: $(HTML_DIR)/index.html

$(HTML_DIR)/index.html: $(RDL_FILE)
	@mkdir -p $(HTML_DIR)
	$(PEAKRDL) html $< -o $(HTML_DIR) --top $(TOP)
	@echo "HTML docs generated — open $(HTML_DIR)/index.html"

# =============================================================
# sv2v: convert SystemVerilog output to plain Verilog
# Requires sv2v on PATH: github.com/zachjs/sv2v/releases
# =============================================================
.PHONY: verilog
verilog: $(RTL_DIR)/$(TOP).v

$(RTL_DIR)/$(TOP).v: $(RTL_DIR)/$(TOP).sv
	sv2v $(RTL_DIR)/$(TOP).sv $(RTL_DIR)/$(TOP)_pkg.sv > $(RTL_DIR)/$(TOP).v
	@echo "Plain Verilog generated: $(RTL_DIR)/$(TOP).v"

# =============================================================
# Lint with Verilator (optional sanity check)
# =============================================================
.PHONY: lint
lint: $(RTL_DIR)/$(TOP).sv
	verilator --lint-only -sv \
		$(RTL_DIR)/$(TOP)_pkg.sv \
		$(RTL_DIR)/$(TOP).sv
	@echo "Lint passed"

# =============================================================
# Clean
# =============================================================
.PHONY: clean
clean:
	rm -rf rdl/rtl rdl/sw
	@echo "Cleaned all generated outputs"