# =============================================================================
# PCIe 7.0 Controller - iverilog Simulation Makefile
# =============================================================================
# Prerequisites:
#   iverilog >= 11.0   (tested with iverilog 11/12 and iverilog-git)
#   vvp                (part of iverilog distribution)
#   gtkwave            (optional, for waveform viewing)
#
# Usage:
#   make           – compile and run simulation
#   make sim       – alias for above
#   make waves     – run simulation then open waveform in GTKWave
#   make compile   – only compile, do not run
#   make clean     – remove build artifacts
#   make lint      – run Verilator lint check (requires Verilator)
# =============================================================================

# -------------------------------------------------------------------------
# Tool Paths (override on command line: make IVERILOG=/usr/bin/iverilog)
# -------------------------------------------------------------------------
IVERILOG  ?= iverilog
VVP       ?= vvp
GTKWAVE   ?= gtkwave
VERILATOR ?= verilator

# -------------------------------------------------------------------------
# Build Configuration
# -------------------------------------------------------------------------
# Top-level simulation output
SIM_TOP   = tb_pcie_top
SIM_BIN   = $(BUILD_DIR)/$(SIM_TOP).vvp
VCD_FILE  = $(BUILD_DIR)/pcie_sim.vcd

# Build directory (shared with project build/)
BUILD_DIR = ../build

# -------------------------------------------------------------------------
# Source file lists
# -------------------------------------------------------------------------
RTL_DIR = ../rtl
TB_DIR  = .

RTL_FILES = \
    $(RTL_DIR)/pcie_pkg.sv           \
    $(RTL_DIR)/pcie_ltssm.sv         \
    $(RTL_DIR)/pcie_pipe_if.sv       \
    $(RTL_DIR)/pcie_dll_tx.sv        \
    $(RTL_DIR)/pcie_dll_rx.sv        \
    $(RTL_DIR)/pcie_flow_ctrl.sv     \
    $(RTL_DIR)/pcie_tlp_tx.sv        \
    $(RTL_DIR)/pcie_tlp_rx.sv        \
    $(RTL_DIR)/pcie_cfg_space.sv     \
    $(RTL_DIR)/pcie_axi_bridge.sv    \
    $(RTL_DIR)/pcie_dma.sv           \
    $(RTL_DIR)/pcie_controller_top.sv

TB_FILES = \
    $(TB_DIR)/axi_master_bfm.sv  \
    $(TB_DIR)/pcie_rc_bfm.sv     \
    $(TB_DIR)/tb_pcie_top.sv

ALL_SRC = $(RTL_FILES) $(TB_FILES)

# -------------------------------------------------------------------------
# iverilog flags
# -------------------------------------------------------------------------
# -g2012  : Enable SystemVerilog 2012 syntax (best support in iverilog)
# -Wall   : All warnings
# -I      : Include paths for `include resolution
IFLAGS = \
    -g2012            \
    -Wall             \
    -I$(RTL_DIR)      \
    -I$(TB_DIR)

# Simulation plusargs (override: make SIM_ARGS="+timeout=100000")
SIM_ARGS ?=

# -------------------------------------------------------------------------
# Default target
# -------------------------------------------------------------------------
.PHONY: all
all: sim

# -------------------------------------------------------------------------
# Compile
# -------------------------------------------------------------------------
.PHONY: compile
compile: $(SIM_BIN)

$(SIM_BIN): $(ALL_SRC) | $(BUILD_DIR)
	@echo "------------------------------------------------------------"
	@echo " Compiling PCIe 7.0 Simulation with iverilog"
	@echo "------------------------------------------------------------"
	$(IVERILOG) $(IFLAGS) -o $@ $(ALL_SRC)
	@echo " Compilation successful: $@"

# -------------------------------------------------------------------------
# Run simulation
# -------------------------------------------------------------------------
.PHONY: sim
sim: $(SIM_BIN)
	@echo "------------------------------------------------------------"
	@echo " Running PCIe 7.0 Simulation"
	@echo "------------------------------------------------------------"
	$(VVP) $(SIM_BIN) $(SIM_ARGS)
	@echo " Simulation finished. VCD: $(VCD_FILE)"

# -------------------------------------------------------------------------
# Open waveforms in GTKWave
# -------------------------------------------------------------------------
.PHONY: waves
waves: sim
	@if [ -f $(VCD_FILE) ]; then \
	    echo "Opening GTKWave with $(VCD_FILE)"; \
	    $(GTKWAVE) $(VCD_FILE) & \
	else \
	    echo "VCD file not found: $(VCD_FILE)"; \
	    exit 1; \
	fi

# -------------------------------------------------------------------------
# Verilator lint (no simulation – structural check only)
# -------------------------------------------------------------------------
.PHONY: lint
lint:
	@echo "------------------------------------------------------------"
	@echo " Running Verilator lint (syntax/structural check)"
	@echo "------------------------------------------------------------"
	$(VERILATOR) --lint-only \
	    --sv                  \
	    -I$(RTL_DIR)          \
	    -I$(TB_DIR)           \
	    --top-module $(SIM_TOP) \
	    $(ALL_SRC)
	@echo " Lint passed."

# -------------------------------------------------------------------------
# Create build directory if it doesn't exist
# -------------------------------------------------------------------------
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# -------------------------------------------------------------------------
# Clean
# -------------------------------------------------------------------------
.PHONY: clean
clean:
	@echo " Cleaning simulation artifacts..."
	rm -f $(SIM_BIN)
	rm -f $(VCD_FILE)
	@echo " Done."

# -------------------------------------------------------------------------
# Help
# -------------------------------------------------------------------------
.PHONY: help
help:
	@echo ""
	@echo "PCIe 7.0 Controller Simulation Makefile"
	@echo "========================================"
	@echo ""
	@echo "Targets:"
	@echo "  make          – Compile and run simulation (default)"
	@echo "  make sim      – Same as 'make'"
	@echo "  make compile  – Only compile, do not run"
	@echo "  make waves    – Run simulation then view VCD in GTKWave"
	@echo "  make lint     – Run Verilator lint check"
	@echo "  make clean    – Remove build artifacts"
	@echo "  make help     – Show this message"
	@echo ""
	@echo "Variables:"
	@echo "  IVERILOG=$(IVERILOG)"
	@echo "  VVP=$(VVP)"
	@echo "  GTKWAVE=$(GTKWAVE)"
	@echo "  VERILATOR=$(VERILATOR)"
	@echo "  SIM_ARGS=$(SIM_ARGS)"
	@echo ""
