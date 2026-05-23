# =============================================================================
# Verilator minimal DUT simulation (C++ driver, no UVM / iverilog BFMs)
# =============================================================================
#   make build   — compile Vtb_pcie_dut
#   make run     — build + run smoke (exit 0 = link_up)
#   make trace   — build with VCD + run
#   make clean   — remove obj_dir and binary
# =============================================================================

VERILATOR ?= verilator
ROOT      := $(abspath ..)
RTL_DIR   := $(ROOT)/rtl
TB_DIR    := $(ROOT)/tb
UVM_TB    := $(ROOT)/uvm_tb
BUILD_DIR := $(ROOT)/build/verilator
OBJ_DIR   := $(BUILD_DIR)/obj_dir
TOP       := tb_pcie_dut
EXE       := $(BUILD_DIR)/V$(TOP)

TRACE     ?= 0
JOBS      ?= 0

RTL_SRCS := \
	$(RTL_DIR)/pcie_pkg.sv \
	$(RTL_DIR)/pcie_cdc_sync.sv \
	$(RTL_DIR)/pcie_async_fifo.sv \
	$(RTL_DIR)/pcie_ltssm.sv \
	$(RTL_DIR)/pcie_pipe_if.sv \
	$(RTL_DIR)/pcie_flit_if.sv \
	$(RTL_DIR)/pcie_dll_tx.sv \
	$(RTL_DIR)/pcie_dll_rx.sv \
	$(RTL_DIR)/pcie_flow_ctrl.sv \
	$(RTL_DIR)/pcie_tlp_tx.sv \
	$(RTL_DIR)/pcie_tlp_rx.sv \
	$(RTL_DIR)/pcie_cfg_space.sv \
	$(RTL_DIR)/pcie_axi_bridge.sv \
	$(RTL_DIR)/pcie_dma.sv \
	$(RTL_DIR)/pcie_pm_ctrl.sv \
	$(RTL_DIR)/pcie_cpl_timeout.sv \
	$(RTL_DIR)/pcie_controller_top.sv

TB_SRCS := \
	pcie_verilator_link.sv \
	tb_pcie_dut.sv

VERILATOR_WARN := \
	-Wno-INITIALDLY -Wno-DECLFILENAME -Wno-PINMISSING \
	-Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-ZERODLY \
	-Wno-UNOPTFLAT -Wno-UNSIGNED -Wno-COMBDLY -Wno-STMTDLY \
	-Wno-SELRANGE -Wno-REDEFMACRO

VERILATOR_FLAGS := \
	--cc --exe --build --build-jobs $(JOBS) --no-timing \
	--top-module $(TOP) \
	-I$(RTL_DIR) -I$(TB_DIR) -I$(UVM_TB) \
	$(VERILATOR_WARN) \
	--Mdir $(OBJ_DIR) \
	-o $(BUILD_DIR)/V$(TOP)

ifeq ($(TRACE),1)
  VERILATOR_FLAGS += --trace -CFLAGS "-DVM_TRACE"
endif

.PHONY: help build run trace clean

help:
	@echo "Verilator DUT smoke: build run trace clean"
	@echo "  TRACE=1 make trace  — enable VCD ($(BUILD_DIR)/pcie_dut.vcd)"

build:
	@mkdir -p $(BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
	  $(RTL_SRCS) $(TB_SRCS) sim_main.cpp

run: build
	@$(EXE); ec=$$?; \
	  if [ $$ec -eq 0 ]; then echo "Verilator sim PASS"; else echo "Verilator sim FAIL"; fi; \
	  exit $$ec

trace:
	$(MAKE) run TRACE=1

clean:
	rm -rf $(BUILD_DIR)
