# =============================================================================
# PCIe 7.0 Controller - Synopsys Design Compiler Synthesis Script
# =============================================================================
# Usage:
#   dc_shell-xg-t -f scripts/synth_dc.tcl | tee logs/synth_dc.log
#
# Requires:
#   - RTL source in rtl/
#   - SDC constraints in constraints/pcie_controller_top.sdc
#   - Target library set via LIB_DB environment variable, or edit below
# =============================================================================

# ---------------------------------------------------------------------------
# Tool settings
# ---------------------------------------------------------------------------
set_app_var sh_enable_page_mode  false
set_app_var compile_ultra_ungroup_small_hierarchies false

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
set DESIGN_ROOT  [file normalize [file dirname [info script]]/..]
set RTL_DIR      $DESIGN_ROOT/rtl
set CONST_DIR    $DESIGN_ROOT/constraints
set RESULTS_DIR  $DESIGN_ROOT/synth_results
set REPORT_DIR   $DESIGN_ROOT/synth_results/reports

file mkdir $RESULTS_DIR
file mkdir $REPORT_DIR

# ---------------------------------------------------------------------------
# Target library (edit for your PDK)
# ---------------------------------------------------------------------------
set TARGET_LIB   [getenv LIB_DB]   ;# e.g. /pdkroot/slow_125c.db
set SYMBOL_LIB   ""

set target_library  $TARGET_LIB
set link_library    "* $TARGET_LIB"
set symbol_library  $SYMBOL_LIB

# ---------------------------------------------------------------------------
# RTL source files (in dependency order)
# ---------------------------------------------------------------------------
set RTL_FILES {
  pcie_pkg.sv
  pcie_cdc_sync.sv
  pcie_async_fifo.sv
  pcie_ltssm.sv
  pcie_pipe_if.sv
  pcie_dll_tx.sv
  pcie_dll_rx.sv
  pcie_flow_ctrl.sv
  pcie_tlp_tx.sv
  pcie_tlp_rx.sv
  pcie_cfg_space.sv
  pcie_axi_bridge.sv
  pcie_dma.sv
  pcie_pm_ctrl.sv
  pcie_cpl_timeout.sv
  pcie_controller_top.sv
}

foreach f $RTL_FILES {
  analyze -format sverilog -lib work $RTL_DIR/$f
}

# ---------------------------------------------------------------------------
# Elaborate top
# ---------------------------------------------------------------------------
set TOP_MODULE pcie_controller_top
elaborate $TOP_MODULE -lib work

# ---------------------------------------------------------------------------
# Design parameters (must match testbench / intended config)
# ---------------------------------------------------------------------------
# Example: x4 Gen5 endpoint
set_parameter MAX_GEN   5
set_parameter NUM_LANES 4
set_parameter DATA_W    256
set_parameter EN_DMA    1

# ---------------------------------------------------------------------------
# Link and check
# ---------------------------------------------------------------------------
link
check_design

# ---------------------------------------------------------------------------
# Timing constraints
# ---------------------------------------------------------------------------
read_sdc $CONST_DIR/pcie_controller_top.sdc

# ---------------------------------------------------------------------------
# Compile (ultra mode for best QoR; change to compile for faster run)
# ---------------------------------------------------------------------------
compile_ultra -no_autoungroup

# ---------------------------------------------------------------------------
# Reports
# ---------------------------------------------------------------------------
report_timing -max_paths 20 -nworst 5  > $REPORT_DIR/timing.rpt
report_area                             > $REPORT_DIR/area.rpt
report_power                            > $REPORT_DIR/power.rpt
report_design                           > $REPORT_DIR/design.rpt
report_constraint -all_violators        > $REPORT_DIR/constraints.rpt
report_clock_gating                     > $REPORT_DIR/clock_gating.rpt
report_qor                              > $REPORT_DIR/qor.rpt

# ---------------------------------------------------------------------------
# Write outputs
# ---------------------------------------------------------------------------
write -format ddc -hierarchy -output $RESULTS_DIR/${TOP_MODULE}.ddc
write -format verilog -hierarchy -output $RESULTS_DIR/${TOP_MODULE}_netlist.v
write_sdc $RESULTS_DIR/${TOP_MODULE}_mapped.sdc
write_sdf -version 3.0 $RESULTS_DIR/${TOP_MODULE}.sdf

echo "=== Synthesis complete. Check $REPORT_DIR/ for reports. ==="
