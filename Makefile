PROJECT := PCI_Express_Gen7.0

.PHONY: help all sim compile lint lint-sva lint-sva-all check check-all verilator-check waves clean package docs-check
.PHONY: sim-pipe sim-dll sim-tlp sim-lcrc sim-strict sim-features sim-features-strict sim-flit formal formal-yosys-prep formal-dll synth-smoke
.PHONY: uvm-smoke uvm-error uvm-strict uvm-compile
.PHONY: verilator-build verilator-run verilator-trace verilator-clean

QUESTA_AVAILABLE := $(shell command -v vlog >/dev/null 2>&1 && echo 1)

help:
	@echo "$(PROJECT) — Verilator + iverilog (default open-source stack)"
	@echo "  Verilator-only CI:  make verilator-check   (lint + lint-sva + verilator-run)"
	@echo "  Full open-source:   make check             (+ iverilog sim; needs iverilog)"
	@echo "  check check-all sim sim-pipe sim-dll sim-tlp sim-lcrc sim-strict"
	@echo "  lint lint-sva lint-sva-all waves clean"
	@echo "  verilator-run verilator-trace — DUT link-up smoke (verilator_tb/)"
	@echo "  uvm-* — optional Questa/VCS only (skip if you have Verilator only)"

all: sim

sim:
	$(MAKE) -C tb sim

compile:
	$(MAKE) -C tb compile

lint:
	$(MAKE) -C tb lint

lint-sva:
	$(MAKE) -C tb lint-sva

lint-sva-all:
	$(MAKE) -C tb lint-sva-all

check:
	$(MAKE) -C tb check

check-all:
	$(MAKE) -C tb check-all

# Verilator-only signoff (no iverilog / Questa required)
verilator-check: lint lint-sva verilator-run
	@echo " verilator-check: lint + SVA lint + verilator-run passed."

sim-pipe:
	$(MAKE) -C tb sim-pipe

sim-dll:
	$(MAKE) -C tb sim-dll

sim-tlp:
	$(MAKE) -C tb sim-tlp

sim-lcrc:
	$(MAKE) -C tb sim-lcrc

sim-strict:
	$(MAKE) -C tb sim-strict

sim-features:
	$(MAKE) -C tb sim-features

sim-features-strict:
	$(MAKE) -C tb sim-features-strict

sim-flit:
	$(MAKE) -C tb sim-flit

FORMAL_YOSYS_DIR := formal/yosys_rtl
FORMAL_PREP_SRCS := \
	rtl/pcie_pkg.sv \
	rtl/pcie_ltssm.sv \
	rtl/pcie_dll_tx.sv \
	rtl/pcie_flow_ctrl.sv \
	formal/props/pcie_ltssm_formal_props.sv \
	formal/props/pcie_dll_formal_props.sv \
	formal/props/pcie_fc_formal_props.sv

formal-yosys-prep:
	@mkdir -p $(FORMAL_YOSYS_DIR)
	python3 scripts/formal_yosys_prep.py --pkg rtl/pcie_pkg.sv --out-dir $(FORMAL_YOSYS_DIR) \
	  $(foreach s,$(FORMAL_PREP_SRCS),--src $(s))
	@sed -i 's/RETRY_BUF_DEPTH = 2048/RETRY_BUF_DEPTH = 4/' $(FORMAL_YOSYS_DIR)/pcie_dll_tx.sv
	@sed -i '1i `define FORMAL' $(FORMAL_YOSYS_DIR)/pcie_dll_tx.sv

FORMAL_SBY := pcie_ltssm_fpv.sby pcie_dll_fpv.sby pcie_fc_fpv.sby
FORMAL_SBY_DLL := pcie_dll_fpv.sby

formal-dll: formal-yosys-prep
	@if command -v sby >/dev/null 2>&1; then \
	  (cd formal && sby -f $(FORMAL_SBY_DLL)) || exit 1; \
	else \
	  echo "SymbiYosys (sby) required — see formal/README.md"; exit 1; \
	fi

formal: formal-yosys-prep
	@if command -v sby >/dev/null 2>&1; then \
	  for f in $(FORMAL_SBY); do echo "=== formal/$$f ==="; (cd formal && sby -f "$$f") || exit 1; done; \
	  echo " formal: all SymbiYosys targets passed."; \
	else \
	  if [ "$(FORMAL_STRICT)" = "1" ]; then \
	    echo "SymbiYosys (sby) required but not installed — see formal/README.md"; exit 1; \
	  else \
	    echo "SymbiYosys (sby) not installed — see formal/README.md"; exit 0; \
	  fi; \
	fi

synth-smoke:
	@if command -v dc_shell >/dev/null 2>&1 && [ -n "$$LIB_DB" ]; then \
	  dc_shell-xg-t -f scripts/synth_dc.tcl | tee synth_results/synth_smoke.log; \
	else \
	  echo "Skip synth-smoke: need dc_shell and LIB_DB env var (see scripts/synth_dc.tcl)"; exit 0; \
	fi

sim-strict-check:
	$(MAKE) -C tb sim-strict-check

verilator-build:
	$(MAKE) -C verilator_tb build

verilator-run:
	$(MAKE) -C verilator_tb run

verilator-trace:
	$(MAKE) -C verilator_tb trace

verilator-clean:
	$(MAKE) -C verilator_tb clean

uvm-compile:
	@if [ -z "$(QUESTA_AVAILABLE)" ]; then \
	  echo "ERROR: Questa/VCS not found — see docs/VERILATOR_FLOW.md"; exit 1; fi
	$(MAKE) -C uvm_tb compile SIMULATOR=questa

uvm-smoke:
	@if [ -z "$(QUESTA_AVAILABLE)" ]; then \
	  echo "ERROR: Questa/VCS not found — see docs/VERILATOR_FLOW.md"; exit 1; fi
	$(MAKE) -C uvm_tb smoke SIMULATOR=questa

uvm-error:
	@if [ -z "$(QUESTA_AVAILABLE)" ]; then \
	  echo "ERROR: Questa/VCS not found — see docs/VERILATOR_FLOW.md"; exit 1; fi
	$(MAKE) -C uvm_tb error_inj SIMULATOR=questa

uvm-strict:
	@if [ -z "$(QUESTA_AVAILABLE)" ]; then \
	  echo "ERROR: Questa/VCS not found — see docs/VERILATOR_FLOW.md"; exit 1; fi
	$(MAKE) -C uvm_tb strict SIMULATOR=questa STRICT_LAYERS=1

waves:
	$(MAKE) -C tb waves

clean:
	$(MAKE) -C tb clean
	$(MAKE) -C uvm_tb clean
	$(MAKE) -C verilator_tb clean
	@rm -rf release

package:
	$(MAKE) -C .. package PROJECT=$(PROJECT)

docs-check:
	$(MAKE) -C .. delivery-check
