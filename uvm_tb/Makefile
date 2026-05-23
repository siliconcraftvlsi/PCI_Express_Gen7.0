# =============================================================================
# PCIe 7.0 UVM Testbench Makefile
# =============================================================================
# Targets:
#   make smoke        — pcie_smoke_test + full SVA bind
#   make error_inj    — pcie_error_regression_test
#   make uvm-strict   — smoke with PCIE_STRICT_LAYERS (no SIM_BYPASS)
#   make regress      — all tests × NUM_SEEDS
#
# Requires Questa/VCS (not Verilator). See ../docs/VERILATOR_FLOW.md.
# Default UVM_HOME: Questa FSE uvm-1.2 (override if needed).
# =============================================================================

SIMULATOR     ?= questa
TOP           := pcie_uvm_top
QUESTA_HOME   ?= /home/swami/intelFPGA_lite/24.1std/questa_fse
UVM_HOME      ?= $(QUESTA_HOME)/verilog_src/uvm-1.2
STRICT_LAYERS ?= 0
USE_RC_BFM    ?= 1
ROOT          := $(abspath ..)
RTL_DIR       := $(ROOT)/rtl
SVA_DIR       := $(ROOT)/sva
TB_DIR        := $(abspath .)
BUILD_DIR     := $(ROOT)/build/uvm
COV_DIR       := $(ROOT)/coverage
LOG_DIR       := $(TB_DIR)/logs
WORK_LIB      := $(BUILD_DIR)/work

TESTS := \
  pcie_smoke_test \
  pcie_error_regression_test \
  pcie_feature_regression_test

NUM_SEEDS := 5
SEED      ?= 1

RTL_FILES := \
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

SVA_FILES := \
  $(SVA_DIR)/pcie_ltssm_assertions.sv \
  $(SVA_DIR)/pcie_dll_assertions.sv \
  $(SVA_DIR)/pcie_fc_assertions.sv \
  $(SVA_DIR)/pcie_tlp_assertions.sv \
  $(SVA_DIR)/pcie_delivery_assertions.sv

TB_FILES := \
  $(TB_DIR)/pcie_pipe_if.sv \
  $(TB_DIR)/pcie_uvm_if.sv \
  $(TB_DIR)/pcie_pipe_agent_pkg.sv \
  $(TB_DIR)/pcie_ltssm_cov.sv \
  $(TB_DIR)/pcie_uvm_pkg.sv \
  $(TB_DIR)/pcie_error_inject_seq.sv \
  $(TB_DIR)/pcie_feature_uvm.sv \
  $(TB_DIR)/pcie_pipe_partner.sv \
  $(TB_DIR)/pcie_sva_bind.sv \
  $(TB_DIR)/pcie_uvm_top.sv

ifeq ($(USE_RC_BFM),1)
  TB_FILES += $(ROOT)/tb/pcie_rc_bfm.sv
endif

ALL_FILES := $(RTL_FILES) $(SVA_FILES) $(TB_FILES)
UVM_PKG   := $(UVM_HOME)/src/uvm_pkg.sv

VLOG_DEFS :=
ifeq ($(STRICT_LAYERS),1)
  VLOG_DEFS += +define+PCIE_STRICT_LAYERS
endif
ifeq ($(USE_RC_BFM),1)
  VLOG_DEFS += +define+PCIE_UVM_RC_BFM
endif

# ---------------------------------------------------------------------------
# Questa
# ---------------------------------------------------------------------------
ifeq ($(SIMULATOR),questa)

VLOG := vlog -sv -mfcu $(VLOG_DEFS) \
  +incdir+$(RTL_DIR) +incdir+$(TB_DIR) +incdir+$(UVM_HOME)/src \
  +define+UVM_NO_DEPRECATED \
  -work $(WORK_LIB) \
  $(UVM_PKG) \
  $(ALL_FILES)

VSIM := vsim -c -lib $(WORK_LIB) +incdir+$(UVM_HOME)/src \
  -coverage +UVM_VERBOSITY=UVM_LOW +acc $(TOP)

compile:
	@mkdir -p $(BUILD_DIR) $(LOG_DIR)
	$(VLOG)

smoke: compile
	$(VSIM) +UVM_TESTNAME=pcie_smoke_test +ntb_random_seed=$(SEED) \
	  -do "coverage save -onexit $(BUILD_DIR)/smoke.ucdb; run -all; quit -f" \
	  | tee $(LOG_DIR)/smoke.log

error_inj: compile
	$(VSIM) +UVM_TESTNAME=pcie_error_regression_test +ntb_random_seed=$(SEED) \
	  -do "coverage save -onexit $(BUILD_DIR)/error_inj.ucdb; run -all; quit -f" \
	  | tee $(LOG_DIR)/error_inj.log

feature_regression: compile
	$(VSIM) +UVM_TESTNAME=pcie_feature_regression_test +ntb_random_seed=$(SEED) \
	  -do "coverage save -onexit $(BUILD_DIR)/feature_regression.ucdb; run -all; quit -f" \
	  | tee $(LOG_DIR)/feature_regression.log

strict:
	$(MAKE) compile STRICT_LAYERS=1
	$(VSIM) +UVM_TESTNAME=pcie_smoke_test +ntb_random_seed=$(SEED) \
	  -do "coverage save -onexit $(BUILD_DIR)/smoke_strict.ucdb; run -all; quit -f" \
	  | tee $(LOG_DIR)/smoke_strict.log

endif

# ---------------------------------------------------------------------------
# VCS
# ---------------------------------------------------------------------------
ifeq ($(SIMULATOR),vcs)

VCS_COMPILE := vcs -sverilog -timescale=1ns/1ps $(VLOG_DEFS) \
  +incdir+$(RTL_DIR) +incdir+$(TB_DIR) +incdir+$(UVM_HOME)/src \
  -ntb_opts uvm-1.2 +define+UVM_NO_DEPRECATED \
  -cm line+cond+fsm+tgl+branch \
  $(UVM_PKG) $(ALL_FILES) -o $(BUILD_DIR)/simv

compile:
	@mkdir -p $(BUILD_DIR) $(LOG_DIR)
	$(VCS_COMPILE)

smoke: compile
	$(BUILD_DIR)/simv +UVM_TESTNAME=pcie_smoke_test +ntb_random_seed=$(SEED) \
	  +UVM_VERBOSITY=UVM_LOW -cm line+cond+fsm+tgl+branch \
	  -cm_dir $(COV_DIR)/smoke.vdb | tee $(LOG_DIR)/smoke.log

error_inj: compile
	$(BUILD_DIR)/simv +UVM_TESTNAME=pcie_error_regression_test +ntb_random_seed=$(SEED) \
	  +UVM_VERBOSITY=UVM_LOW -cm line+cond+fsm+tgl+branch \
	  -cm_dir $(COV_DIR)/error_inj.vdb | tee $(LOG_DIR)/error_inj.log

feature_regression: compile
	$(BUILD_DIR)/simv +UVM_TESTNAME=pcie_feature_regression_test +ntb_random_seed=$(SEED) \
	  +UVM_VERBOSITY=UVM_LOW -cm line+cond+fsm+tgl+branch \
	  -cm_dir $(COV_DIR)/feature_regression.vdb | tee $(LOG_DIR)/feature_regression.log

strict:
	$(MAKE) compile STRICT_LAYERS=1
	$(VSIM) +UVM_TESTNAME=pcie_smoke_test +ntb_random_seed=$(SEED) \
	  -do "coverage save -onexit $(BUILD_DIR)/smoke_strict.ucdb; run -all; quit -f" \
	  | tee $(LOG_DIR)/smoke_strict.log

endif

# ---------------------------------------------------------------------------
# Common
# ---------------------------------------------------------------------------
help:
	@echo "PCIe UVM — SIMULATOR=$(SIMULATOR) STRICT_LAYERS=$(STRICT_LAYERS)"
	@echo "  compile smoke error_inj feature_regression strict regress cov_merge clean"

regress: compile
	@mkdir -p $(LOG_DIR) $(COV_DIR)
	@: > $(LOG_DIR)/regress_summary.txt
	@for t in $(TESTS); do \
	  for seed in $$(seq 1 $(NUM_SEEDS)); do \
	    echo "=== $$t seed=$$seed ==="; \
	    $(VSIM) +UVM_TESTNAME=$$t +ntb_random_seed=$$seed \
	      -do "run -all; quit -f" 2>&1 | tee $(LOG_DIR)/$${t}_seed$${seed}.log; \
	    echo "$$t seed=$$seed: $$(grep -cE 'UVM_ERROR|UVM_FATAL' $(LOG_DIR)/$${t}_seed$${seed}.log || echo 0) errors" \
	      >> $(LOG_DIR)/regress_summary.txt; \
	  done; \
	done
	@cat $(LOG_DIR)/regress_summary.txt

cov_merge:
	@$(MAKE) -f $(ROOT)/scripts/coverage_merge.mk

clean:
	rm -rf $(BUILD_DIR) $(LOG_DIR) work *.log transcript *.ucdb *.vdb

.PHONY: help compile smoke error_inj feature_regression strict regress cov_merge clean
