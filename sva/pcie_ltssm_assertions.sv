// =============================================================================
// PCIe 7.0 Controller - LTSSM Formal/Simulation Assertions
// PCIe Base Spec Rev 7.0 Section 4.2
// =============================================================================
// Covers: state reachability, legal transitions, timeout guards,
// no-skip rules, and power/reset control invariants.
// =============================================================================

`include "pcie_pkg.sv"

module pcie_ltssm_assertions
  import pcie_pkg::*;
#(
  parameter int unsigned NUM_LANES = 16,
  parameter int unsigned TIMEOUT_CYCLES = 24'hFFFFFF
)(
  input  logic              clk,
  input  logic              rst_n,
  input  ltssm_state_e      ltssm_state,
  input  logic              link_up,
  input  logic [3:0]        pipe_rate,
  input  logic [1:0]        pipe_width,
  input  logic [3:0]        pipe_power_down,
  input  logic              pipe_reset_n,
  input  logic [NUM_LANES-1:0] pipe_rx_valid,
  input  logic [NUM_LANES-1:0] pipe_rx_elec_idle,
  input  pcie_gen_e         negotiated_gen,
  input  logic [4:0]        negotiated_width
);

  // ---------------------------------------------------------------------------
  // Helper: one-hot state checks
  // ---------------------------------------------------------------------------
  // State must always be a valid enumeration value
  property p_ltssm_valid_state;
    @(posedge clk) disable iff (!rst_n)
    ltssm_state inside {
      DETECT_QUIET, DETECT_ACTIVE,
      POLLING_ACTIVE, POLLING_COMPLIANCE, POLLING_CONFIGURATION,
      POLLING_SPEED,
      CONFIG_LWIDTH_START, CONFIG_LWIDTH_ACCEPT,
      CONFIG_LANENUM_WAIT, CONFIG_LANENUM_ACCEPT,
      CONFIG_COMPLETE, CONFIG_IDLE,
      RECOVERY_RCVRLOCK, RECOVERY_RCVRCFG, RECOVERY_IDLE,
      RECOVERY_EQUALIZATION,
      L0, L0S_TX, L0S_RX,
      L1_ENTRY, L1_IDLE,
      L2_IDLE, L2_TX_WAKE,
      HOT_RESET, DISABLED,
      LOOPBACK_ENTRY, LOOPBACK_ACTIVE, LOOPBACK_EXIT
    };
  endproperty
  ast_ltssm_valid_state: assert property (p_ltssm_valid_state)
    else $error("LTSSM: illegal state encoding 0x%0h", ltssm_state);

  // ---------------------------------------------------------------------------
  // link_up only asserted in L0
  // ---------------------------------------------------------------------------
  property p_link_up_only_in_l0;
    @(posedge clk) disable iff (!rst_n)
    link_up |-> (ltssm_state == L0);
  endproperty
  ast_link_up_only_l0: assert property (p_link_up_only_in_l0)
    else $error("LTSSM: link_up asserted outside L0 state");

  property p_l0_implies_link_up;
    @(posedge clk) disable iff (!rst_n)
    (ltssm_state == L0) |-> link_up;
  endproperty
  ast_l0_implies_link_up: assert property (p_l0_implies_link_up)
    else $error("LTSSM: in L0 but link_up not asserted");

  // ---------------------------------------------------------------------------
  // pipe_reset_n must de-assert (go low) to enter DETECT from reset
  // ---------------------------------------------------------------------------
  property p_pipe_reset_during_detect;
    @(posedge clk) disable iff (!rst_n)
    (ltssm_state == DETECT_QUIET) |-> !pipe_reset_n;
  endproperty
  ast_pipe_reset_detect: assert property (p_pipe_reset_during_detect)
    else $error("LTSSM: pipe_reset_n not asserted in DETECT_QUIET");

  // ---------------------------------------------------------------------------
  // pipe_reset_n must be asserted (high) in L0
  // ---------------------------------------------------------------------------
  property p_pipe_active_in_l0;
    @(posedge clk) disable iff (!rst_n)
    (ltssm_state == L0) |-> pipe_reset_n;
  endproperty
  ast_pipe_active_l0: assert property (p_pipe_active_in_l0)
    else $error("LTSSM: pipe_reset_n de-asserted in L0");

  // ---------------------------------------------------------------------------
  // Negotiated gen must be <= Gen7 encoding
  // ---------------------------------------------------------------------------
  property p_neg_gen_valid;
    @(posedge clk) disable iff (!rst_n)
    link_up |-> (negotiated_gen inside {PCIE_GEN1, PCIE_GEN2, PCIE_GEN3,
                                         PCIE_GEN4, PCIE_GEN5, PCIE_GEN6,
                                         PCIE_GEN7});
  endproperty
  ast_neg_gen_valid: assert property (p_neg_gen_valid)
    else $error("LTSSM: invalid negotiated_gen 0x%0h when link_up", negotiated_gen);

  // ---------------------------------------------------------------------------
  // Negotiated lane width must be 1,2,4,8 or 16 when link is up
  // ---------------------------------------------------------------------------
  property p_neg_width_valid;
    @(posedge clk) disable iff (!rst_n)
    link_up |-> (negotiated_width inside {5'd1, 5'd2, 5'd4, 5'd8, 5'd16});
  endproperty
  ast_neg_width_valid: assert property (p_neg_width_valid)
    else $error("LTSSM: invalid negotiated_width %0d when link_up", negotiated_width);

  // ---------------------------------------------------------------------------
  // No direct jump from DETECT to L0 (must go through POLLING → CONFIG)
  // ---------------------------------------------------------------------------
  property p_no_skip_detect_to_l0;
    @(posedge clk) disable iff (!rst_n)
    (ltssm_state == DETECT_QUIET) |=>
      !(ltssm_state == L0);
  endproperty
  ast_no_skip_detect_l0: assert property (p_no_skip_detect_to_l0)
    else $error("LTSSM: illegal skip from DETECT_QUIET directly to L0");

  // ---------------------------------------------------------------------------
  // Recovery reachable from L0 only (not from DETECT/POLLING directly)
  // ---------------------------------------------------------------------------
  property p_recovery_only_from_l0_or_config;
    @(posedge clk) disable iff (!rst_n)
    $rose(ltssm_state == RECOVERY_RCVRLOCK) |->
      $past(ltssm_state, 1) inside {L0, L0S_TX, L0S_RX, CONFIG_IDLE};
  endproperty
  ast_recovery_source: assert property (p_recovery_only_from_l0_or_config)
    else $error("LTSSM: RECOVERY entered from unexpected state %0d",
                int'($past(ltssm_state, 1)));

  // ---------------------------------------------------------------------------
  // HOT_RESET can only be entered from L0 or CONFIG
  // ---------------------------------------------------------------------------
  property p_hot_reset_source;
    @(posedge clk) disable iff (!rst_n)
    $rose(ltssm_state == HOT_RESET) |->
      $past(ltssm_state, 1) inside {L0, CONFIG_IDLE, RECOVERY_IDLE};
  endproperty
  ast_hot_reset_source: assert property (p_hot_reset_source)
    else $error("LTSSM: HOT_RESET entered from unexpected state");

  // ---------------------------------------------------------------------------
  // L1 entry only from L0 (power management handshake)
  // ---------------------------------------------------------------------------
  property p_l1_entry_source;
    @(posedge clk) disable iff (!rst_n)
    $rose(ltssm_state == L1_ENTRY) |->
      $past(ltssm_state, 1) == L0;
  endproperty
  ast_l1_entry_source: assert property (p_l1_entry_source)
    else $error("LTSSM: L1_ENTRY not entered from L0");

  // ---------------------------------------------------------------------------
  // pipe_power_down must be non-zero in L1/L2
  // ---------------------------------------------------------------------------
  property p_power_down_in_l1;
    @(posedge clk) disable iff (!rst_n)
    (ltssm_state inside {L1_IDLE, L2_IDLE}) |-> (pipe_power_down != 4'h0);
  endproperty
  ast_power_down_l1: assert property (p_power_down_in_l1)
    else $error("LTSSM: pipe_power_down=0 in L1/L2 idle");

  // ---------------------------------------------------------------------------
  // Liveness: system must eventually reach L0 after reset (within timeout)
  // ---------------------------------------------------------------------------
  property p_eventually_l0;
    @(posedge clk)
    $rose(rst_n) |-> ##[1:TIMEOUT_CYCLES] (ltssm_state == L0);
  endproperty
  cov_l0_reached: cover property (p_eventually_l0);

  // ---------------------------------------------------------------------------
  // Coverage: all major states visited
  // ---------------------------------------------------------------------------
  cov_detect:    cover property (@(posedge clk) ltssm_state == DETECT_ACTIVE);
  cov_polling:   cover property (@(posedge clk) ltssm_state == POLLING_ACTIVE);
  cov_config:    cover property (@(posedge clk) ltssm_state == CONFIG_COMPLETE);
  cov_recovery:  cover property (@(posedge clk) ltssm_state == RECOVERY_RCVRLOCK);
  cov_l0:        cover property (@(posedge clk) ltssm_state == L0);
  cov_l0s_tx:    cover property (@(posedge clk) ltssm_state == L0S_TX);
  cov_l1_entry:  cover property (@(posedge clk) ltssm_state == L1_ENTRY);
  cov_l1_idle:   cover property (@(posedge clk) ltssm_state == L1_IDLE);
  cov_l2_idle:   cover property (@(posedge clk) ltssm_state == L2_IDLE);
  cov_hot_reset: cover property (@(posedge clk) ltssm_state == HOT_RESET);
  cov_disabled:  cover property (@(posedge clk) ltssm_state == DISABLED);
  cov_loopback:  cover property (@(posedge clk) ltssm_state == LOOPBACK_ACTIVE);

endmodule : pcie_ltssm_assertions
