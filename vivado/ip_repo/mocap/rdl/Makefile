# =============================================================
# PeakRDL Makefile for the mocap_wrapper register map
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

RDL_FILE    := ./mocap_regs.rdl
TOP         := mocap_regs

OUT_DIR     := ./rdl_out
RTL_DIR     := $(OUT_DIR)/rtl
SW_DIR      := $(OUT_DIR)/sw
HTML_DIR    := $(OUT_DIR)/html

VENV        := ../../../../mocap_env/bin/activate
PEAKRDL     := . $(VENV) && peakrdl

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
	$(PEAKRDL) regblock $< -o $(RTL_DIR) --top $(TOP) --cpuif axi4-lite-flat --default-reset rst_n
	@# Prepend timescale to all generated SV files
	@for f in $(RTL_DIR)/*.sv; do \
		sed -i '1s/^/`timescale 1ns\/1ps\n/' $$f; \
	done
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
# Clean
# =============================================================
.PHONY: clean
clean:
	rm -rf $(OUT_DIR)
	@echo "Cleaned all generated outputs"
