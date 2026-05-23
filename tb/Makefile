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
# Top-level simulation output (tagged per SIM_BYPASS profile)
SIM_TOP   = tb_pcie_top
STRICT    ?= 0
BYPASS_PIPE    ?= 1
BYPASS_DLL_TX  ?= 1
BYPASS_DLL_RX  ?= 1
BYPASS_TLP_TX  ?= 1
BYPASS_LCRC    ?= 1

ifeq ($(STRICT),1)
  SIM_TAG := strict
else
  SIM_TAG := p$(BYPASS_PIPE)_dt$(BYPASS_DLL_TX)_dr$(BYPASS_DLL_RX)_tlp$(BYPASS_TLP_TX)_lc$(BYPASS_LCRC)
endif

SIM_BIN   = $(BUILD_DIR)/$(SIM_TOP)_$(SIM_TAG).vvp
VCD_FILE  = $(BUILD_DIR)/pcie_sim_$(SIM_TAG).vcd

# Build directory (shared with project build/)
BUILD_DIR = ../build

# -------------------------------------------------------------------------
# Source file lists
# -------------------------------------------------------------------------
RTL_DIR = ../rtl
TB_DIR  = .

RTL_FILES = \
    $(RTL_DIR)/pcie_pkg.sv           \
    $(RTL_DIR)/pcie_cdc_sync.sv      \
    $(RTL_DIR)/pcie_async_fifo.sv    \
    $(RTL_DIR)/pcie_ltssm.sv         \
    $(RTL_DIR)/pcie_pipe_if.sv       \
    $(RTL_DIR)/pcie_flit_if.sv       \
    $(RTL_DIR)/pcie_dll_tx.sv        \
    $(RTL_DIR)/pcie_dll_rx.sv        \
    $(RTL_DIR)/pcie_flow_ctrl.sv     \
    $(RTL_DIR)/pcie_tlp_tx.sv        \
    $(RTL_DIR)/pcie_tlp_rx.sv        \
    $(RTL_DIR)/pcie_cfg_space.sv     \
    $(RTL_DIR)/pcie_axi_bridge.sv    \
    $(RTL_DIR)/pcie_dma.sv           \
    $(RTL_DIR)/pcie_pm_ctrl.sv       \
    $(RTL_DIR)/pcie_cpl_timeout.sv   \
    $(RTL_DIR)/pcie_controller_top.sv

TB_FILES = \
    $(TB_DIR)/axi_master_bfm.sv  \
    $(TB_DIR)/pcie_rc_bfm.sv     \
    $(TB_DIR)/tb_pcie_top.sv

SVA_DIR   = ../sva
SVA_ALL   = \
    $(SVA_DIR)/pcie_ltssm_assertions.sv \
    $(SVA_DIR)/pcie_dll_assertions.sv \
    $(SVA_DIR)/pcie_fc_assertions.sv \
    $(SVA_DIR)/pcie_tlp_assertions.sv \
    $(SVA_DIR)/pcie_delivery_assertions.sv

# Verilator: only delivery assertions (no ## / $past clock forms in other modules)
SVA_VERILATOR = $(SVA_DIR)/pcie_delivery_assertions.sv

SVA_FILES = $(SVA_ALL) $(TB_DIR)/pcie_sva_bind.sv

ALL_SRC     = $(RTL_FILES) $(TB_FILES)
# iverilog does not support concurrent SVA (##, |->, cover property). Default sim
# excludes SVA; use ENABLE_SVA=1 with Verilator or a commercial simulator.
ENABLE_SVA ?= 0
ifeq ($(ENABLE_SVA),1)
  SIM_SRC = $(ALL_SRC) $(SVA_FILES)
else
  SIM_SRC = $(ALL_SRC)
endif

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
    -I$(TB_DIR)       \
    $(IVERILOG_DEFS)

# Simulation plusargs (override: make SIM_ARGS="+timeout=100000")
SIM_ARGS ?=

# iverilog -D for gradual SIM_BYPASS (see tb_pcie_top.sv)
IVERILOG_DEFS :=
ifeq ($(FLIT),1)
  IVERILOG_DEFS += -DPCIE_FLIT_TEST
endif
ifeq ($(STRICT),1)
  IVERILOG_DEFS += -DPCIE_STRICT_LAYERS
else
  ifeq ($(BYPASS_PIPE),0)
    IVERILOG_DEFS += -DPCIE_BYPASS_PIPE_OFF -DPCIE_PIPE_LAYER_EN
  endif
  ifeq ($(BYPASS_DLL_TX),0)
    IVERILOG_DEFS += -DPCIE_BYPASS_DLL_TX_OFF
  endif
  ifeq ($(BYPASS_DLL_RX),0)
    IVERILOG_DEFS += -DPCIE_BYPASS_DLL_RX_OFF
  endif
  ifeq ($(BYPASS_TLP_TX),0)
    IVERILOG_DEFS += -DPCIE_BYPASS_TLP_TX_OFF
  endif
  ifeq ($(BYPASS_LCRC),0)
    IVERILOG_DEFS += -DPCIE_BYPASS_LCRC_OFF
  endif
endif

# Verilator lint: suppress known-benign warnings (iverilog-oriented RTL)
VERILATOR_LINT_FLAGS = \
    -Wno-INITIALDLY -Wno-DECLFILENAME -Wno-PINMISSING \
    -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-ZERODLY \
    -Wno-UNOPTFLAT -Wno-UNSIGNED \
    -Wno-COMBDLY -Wno-STMTDLY

# Verilator cannot lint TLP SVA ($past with explicit clock). Skip TLP bind via define.
VERILATOR_SVA_FLAGS = $(VERILATOR_LINT_FLAGS)

# -------------------------------------------------------------------------
# Default target
# -------------------------------------------------------------------------
check: lint lint-sva sim sim-pipe sim-strict
	@echo " check: lint + SVA lint + default sim + sim-pipe + sim-strict all passed."

sim-strict-check: lint lint-sva sim-strict
	@echo " sim-strict-check: lint + SVA lint + strict SIM_BYPASS=0 sim passed."

.PHONY: all sim-pipe sim-dll sim-tlp sim-lcrc sim-strict sim-features sim-features-strict sim-flit formal synth-smoke
all: sim

# -------------------------------------------------------------------------
# Compile
# -------------------------------------------------------------------------
.PHONY: compile
compile: $(SIM_BIN)

$(SIM_BIN): $(SIM_SRC) | $(BUILD_DIR)
	@echo "------------------------------------------------------------"
	@echo " Compiling PCIe 7.0 Simulation with iverilog"
	@echo "------------------------------------------------------------"
	$(IVERILOG) $(IFLAGS) -o $@ $(SIM_SRC)
	@echo " Compilation successful: $@"

# -------------------------------------------------------------------------
# Run simulation
# -------------------------------------------------------------------------
.PHONY: sim
sim: $(SIM_BIN)
	@echo "------------------------------------------------------------"
	@echo " Running PCIe 7.0 Simulation ($(SIM_TAG))"
	@echo "------------------------------------------------------------"
	$(VVP) $(SIM_BIN) +vcd=$(VCD_FILE) $(SIM_ARGS)
	@echo " Simulation finished. VCD: $(VCD_FILE)"

# Per-layer SIM_BYPASS profiles (recompile with tagged .vvp)
sim-pipe:
	$(MAKE) sim BYPASS_PIPE=0

sim-dll:
	$(MAKE) sim BYPASS_DLL_TX=0 BYPASS_DLL_RX=0

sim-tlp:
	$(MAKE) sim BYPASS_TLP_TX=0

sim-lcrc:
	$(MAKE) sim BYPASS_LCRC=0

sim-strict:
	$(MAKE) sim STRICT=1

# Tests 16–18 only (PM, cpl-timeout, MSI-X) — faster than full strict regression
sim-features:
	$(MAKE) sim SIM_ARGS="+feature_tests_only"

sim-features-strict:
	$(MAKE) sim STRICT=1 SIM_ARGS="+feature_tests_only"

sim-flit:
	$(MAKE) sim FLIT=1 STRICT=1

sim-flit-check:
	$(MAKE) sim FLIT=1 STRICT=1 SIM_ARGS="+no_vcd"

check-all: check
	$(MAKE) -C ../verilator_tb run
	@echo " check-all: tb check + Verilator DUT smoke passed."

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
.PHONY: lint lint-sva lint-sva-all check check-all
lint:
	@echo "------------------------------------------------------------"
	@echo " Running Verilator lint (syntax/structural check)"
	@echo "------------------------------------------------------------"
	$(VERILATOR) --lint-only \
	    --sv                  \
	    -I$(RTL_DIR)          \
	    -I$(TB_DIR)           \
	    --top-module $(SIM_TOP) \
	    $(VERILATOR_LINT_FLAGS) \
	    $(ALL_SRC)
	@echo " Lint passed."

# -------------------------------------------------------------------------
# Verilator lint including bound delivery SVA (no iverilog sim)
# -------------------------------------------------------------------------
lint-sva:
	@echo "------------------------------------------------------------"
	@echo " Verilator lint: RTL + TB + delivery SVA bind (other SVA: Questa only)"
	@echo "------------------------------------------------------------"
	$(VERILATOR) --lint-only --sv \
	    -I$(RTL_DIR) -I$(TB_DIR) \
	    --top-module $(SIM_TOP) \
	    $(VERILATOR_SVA_FLAGS) \
	    $(ALL_SRC) $(SVA_VERILATOR) $(TB_DIR)/pcie_sva_bind.sv
	@echo " SVA lint passed (Verilator-compatible bind)."

lint-sva-all: lint-sva
	@echo " Note: LTSSM/DLL/FC/TLP SVA require Questa/VCS (Verilator skips via +define+VERILATOR)."

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
	rm -f $(BUILD_DIR)/$(SIM_TOP)_*.vvp
	rm -f $(BUILD_DIR)/pcie_sim_*.vcd
	rm -f $(BUILD_DIR)/pcie_sim.vcd $(BUILD_DIR)/$(SIM_TOP).vvp
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
	@echo "  make lint     – Run Verilator lint check (RTL+TB)"
	@echo "  make lint-sva     – Verilator: RTL+TB+SVA bind (no TLP bind)"
	@echo "  make lint-sva-all – Alias for lint-sva on Verilator-only flows"
	@echo "  make check        – lint + lint-sva + sim + sim-pipe"
	@echo "  make check-all    – check + verilator_tb run"
	@echo "  make sim-pipe     – SIM_BYPASS_PIPE=0 (real pcie_pipe_if)"
	@echo "  make sim-dll      – DLL TX/RX layers on"
	@echo "  make sim-tlp      – TLP TX layer on"
	@echo "  make sim-lcrc     – LCRC check on (DLL RX)"
	@echo "  make sim-strict   – all bypass off (PCIE_STRICT_LAYERS)"
	@echo "  make sim-features – tests 16-18 only (+feature_tests_only)"
	@echo "  make sim-features-strict – sim-features with STRICT=1"
	@echo "  make sim-flit       – FLIT test (FLIT=1 STRICT=1, 42 checks)"
	@echo "  ENABLE_SVA=1      – iverilog: not supported; use Verilator lint-sva"
	@echo ""
	@echo "  make clean    – Remove build artifacts"
	@echo "  make help     – Show this message"
	@echo ""
	@echo "Variables:"
	@echo "  IVERILOG=$(IVERILOG)"
	@echo "  VVP=$(VVP)"
	@echo "  GTKWAVE=$(GTKWAVE)"
	@echo "  VERILATOR=$(VERILATOR)"
	@echo "  SIM_ARGS=$(SIM_ARGS)"
	@echo "  BYPASS_PIPE/DLL_TX/DLL_RX/TLP_TX/LCRC=$(BYPASS_PIPE)/$(BYPASS_DLL_TX)/$(BYPASS_DLL_RX)/$(BYPASS_TLP_TX)/$(BYPASS_LCRC)"
	@echo "  STRICT=$(STRICT)  SIM_TAG=$(SIM_TAG)"
	@echo ""
