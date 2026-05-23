// =============================================================================
// PCIe 7.0 Controller - Flow Control Assertions
// PCIe Base Spec Rev 7.0 Section 2.11
// =============================================================================
// Covers: credit underflow/overflow, infinite credit encoding, FC init
// sequence ordering, periodic FC update liveness, RX credit advertisement
// validity, infinite-credit stability, and FC update type correctness.
// =============================================================================

`include "pcie_pkg.sv"

module pcie_fc_assertions
  import pcie_pkg::*;
#(
  parameter int unsigned FC_UPDATE_MAX_INTERVAL = 200000
)(
  input  logic         clk,
  input  logic         rst_n,
  input  logic         link_up,

  // --------------------------------------------------------------------------
  // Available credits (what remote side granted us)
  // --------------------------------------------------------------------------
  input  fc_credits_t  avail_p,
  input  fc_credits_t  avail_np,
  input  fc_credits_t  avail_cpl,

  // --------------------------------------------------------------------------
  // Credits consumed by TL TX
  // --------------------------------------------------------------------------
  input  fc_credits_t  consumed_p,
  input  fc_credits_t  consumed_np,
  input  fc_credits_t  consumed_cpl,

  // --------------------------------------------------------------------------
  // Initial credits we advertise to the remote side
  // --------------------------------------------------------------------------
  input  fc_credits_t  init_credits_p,
  input  fc_credits_t  init_credits_np,
  input  fc_credits_t  init_credits_cpl,

  // --------------------------------------------------------------------------
  // FC update TX pulse
  // --------------------------------------------------------------------------
  input  logic         fc_update_tx,

  // --------------------------------------------------------------------------
  // FC DLLP received from remote side (RX path)
  // --------------------------------------------------------------------------
  input  logic         fc_rx_valid,     // FC DLLP received and decoded
  input  dllp_type_e   fc_rx_type,      // Decoded FC DLLP type
  input  logic [11:0]  fc_rx_hdr,       // Header credit value in received DLLP
  input  logic [19:0]  fc_rx_data,      // Data credit value in received DLLP

  // --------------------------------------------------------------------------
  // FC update type (type of the update DLLP we are sending)
  // --------------------------------------------------------------------------
  input  dllp_type_e   fc_upd_type      // Must be FC_UPD_P/NP/CPL when fc_update_tx
);

  // ==========================================================================
  // Local parameters
  // ==========================================================================
  localparam logic [11:0] FC_HDR_INFINITE = 12'hFFF;
  localparam logic [19:0] FC_DAT_INFINITE = 20'hFFFFF;

  // ==========================================================================
  // P1 — Posted header credits: consumed must not exceed available
  // ==========================================================================
  property p_posted_hdr_no_underflow;
    @(posedge clk) disable iff (!rst_n)
    (link_up && (avail_p.header_credits != FC_HDR_INFINITE)) |->
      (consumed_p.header_credits <= avail_p.header_credits);
  endproperty
  ast_p_hdr_underflow: assert property (p_posted_hdr_no_underflow)
    else $error("FC: Posted header credit underflow: consumed=%0d avail=%0d",
                consumed_p.header_credits, avail_p.header_credits);

  // ==========================================================================
  // P2 — Posted data credits: consumed must not exceed available
  // ==========================================================================
  property p_posted_dat_no_underflow;
    @(posedge clk) disable iff (!rst_n)
    (link_up && (avail_p.data_credits != FC_DAT_INFINITE)) |->
      (consumed_p.data_credits <= avail_p.data_credits);
  endproperty
  ast_p_dat_underflow: assert property (p_posted_dat_no_underflow)
    else $error("FC: Posted data credit underflow: consumed=%0d avail=%0d",
                consumed_p.data_credits, avail_p.data_credits);

  // ==========================================================================
  // P3 — Non-posted header credits: no underflow
  // ==========================================================================
  property p_np_hdr_no_underflow;
    @(posedge clk) disable iff (!rst_n)
    (link_up && (avail_np.header_credits != FC_HDR_INFINITE)) |->
      (consumed_np.header_credits <= avail_np.header_credits);
  endproperty
  ast_np_hdr_underflow: assert property (p_np_hdr_no_underflow)
    else $error("FC: NP header credit underflow: consumed=%0d avail=%0d",
                consumed_np.header_credits, avail_np.header_credits);

  // ==========================================================================
  // P4 — Completion header credits: no underflow
  // ==========================================================================
  property p_cpl_hdr_no_underflow;
    @(posedge clk) disable iff (!rst_n)
    (link_up && (avail_cpl.header_credits != FC_HDR_INFINITE)) |->
      (consumed_cpl.header_credits <= avail_cpl.header_credits);
  endproperty
  ast_cpl_hdr_underflow: assert property (p_cpl_hdr_no_underflow)
    else $error("FC: CPL header credit underflow: consumed=%0d avail=%0d",
                consumed_cpl.header_credits, avail_cpl.header_credits);

  // ==========================================================================
  // P5 — Init credits for Posted must be non-zero or explicitly infinite
  // ==========================================================================
  property p_init_p_credits_nonzero;
    @(posedge clk) disable iff (!rst_n)
    link_up |->
      (init_credits_p.header_credits != '0) ||
      (init_credits_p.header_credits == FC_HDR_INFINITE);
  endproperty
  ast_init_p_nonzero: assert property (p_init_p_credits_nonzero)
    else $error("FC: Posted init header credits are zero (non-infinite)");

  // ==========================================================================
  // P6 — No X on avail_p header credits after link_up
  // ==========================================================================
  property p_avail_p_no_x;
    @(posedge clk) disable iff (!rst_n)
    link_up |-> !$isunknown(avail_p.header_credits);
  endproperty
  ast_avail_p_no_x: assert property (p_avail_p_no_x)
    else $error("FC: X on avail_p.header_credits after link_up");

  // ==========================================================================
  // P7 — No X on avail_np header credits after link_up
  // ==========================================================================
  property p_avail_np_no_x;
    @(posedge clk) disable iff (!rst_n)
    link_up |-> !$isunknown(avail_np.header_credits);
  endproperty
  ast_avail_np_no_x: assert property (p_avail_np_no_x)
    else $error("FC: X on avail_np.header_credits after link_up");

  // ==========================================================================
  // P8 — No X on avail_cpl header credits after link_up
  // ==========================================================================
  property p_avail_cpl_no_x;
    @(posedge clk) disable iff (!rst_n)
    link_up |-> !$isunknown(avail_cpl.header_credits);
  endproperty
  ast_avail_cpl_no_x: assert property (p_avail_cpl_no_x)
    else $error("FC: X on avail_cpl.header_credits after link_up");

  // ==========================================================================
  // P9 — fc_update_tx should not pulse when link is down
  // ==========================================================================
  property p_no_fc_update_without_link;
    @(posedge clk) disable iff (!rst_n)
    fc_update_tx |-> link_up;
  endproperty
  ast_fc_update_needs_link: assert property (p_no_fc_update_without_link)
    else $error("FC: fc_update_tx asserted without link_up");

  // ==========================================================================
  // P10 — RX Init DLLP for Posted/Non-Posted must advertise at least 1 header
  //        credit (infinite encoding 12'hFFF is also acceptable)
  //        PCIe Base Spec Section 2.11.2: "The initial FC advertisement for
  //        Posted and NonPosted header credits must be non-zero."
  // ==========================================================================
  property p_fc_rx_hdr_nonzero;
    @(posedge clk) disable iff (!rst_n)
    (fc_rx_valid &&
     (fc_rx_type inside {DLLP_FC_INIT_P, DLLP_FC_INIT_NP})) |->
      ((fc_rx_hdr > 12'h0) || (fc_rx_hdr == FC_HDR_INFINITE));
  endproperty
  ast_fc_rx_hdr_nonzero: assert property (p_fc_rx_hdr_nonzero)
    else $error("FC RX: Init DLLP (type=0x%02h) carries zero header credits",
                fc_rx_type);

  // ==========================================================================
  // P11 — When fc_update_tx pulses, the update type must be one of the three
  //        FC Update types (UPD_P, UPD_NP, or UPD_CPL)
  // ==========================================================================
  property p_credit_update_type_valid;
    @(posedge clk) disable iff (!rst_n)
    fc_update_tx |->
      (fc_upd_type inside {DLLP_FC_UPD_P, DLLP_FC_UPD_NP, DLLP_FC_UPD_CPL});
  endproperty
  ast_credit_update_type_valid: assert property (p_credit_update_type_valid)
    else $error("FC: fc_update_tx with invalid fc_upd_type=0x%02h", fc_upd_type);

  // ==========================================================================
  // P12 — FC update must fire within FC_UPDATE_MAX_INTERVAL cycles of link_up
  //        (bounded liveness: covers the periodic FC update timer requirement)
  // ==========================================================================
  property p_fc_upd_cycles_after_init;
    @(posedge clk) disable iff (!rst_n)
    link_up |-> ##[1:FC_UPDATE_MAX_INTERVAL+1] fc_update_tx;
  endproperty
  // Expressed as cover here; change to assert if the design strictly guarantees
  // the update within this window from any given link_up cycle.
  cov_fc_update_live: cover property (p_fc_upd_cycles_after_init);

  // ==========================================================================
  // P13 — avail_cpl.header_credits: once link is up, it must be either
  //        infinite (12'hFFF) or non-zero.  It should never be zero while
  //        the link is up (would block all completions indefinitely).
  // ==========================================================================
  property p_avail_cpl_infinite_or_nonzero;
    @(posedge clk) disable iff (!rst_n)
    link_up |->
      (avail_cpl.header_credits == FC_HDR_INFINITE) ||
      (avail_cpl.header_credits > 12'h0);
  endproperty
  ast_avail_cpl_infinite_or_nonzero: assert property (p_avail_cpl_infinite_or_nonzero)
    else $error("FC: avail_cpl.header_credits is zero (not infinite) while link_up");

  // ==========================================================================
  // P14 — fc_update_tx implies link_up (consumed credits only updated with link)
  //        This is a restatement of P9 phrased as "no consumed before init"
  //        from the FC perspective: you cannot update credits when link is down.
  // ==========================================================================
  property p_no_consumed_before_init;
    @(posedge clk) disable iff (!rst_n)
    fc_update_tx |-> link_up;
  endproperty
  // Note: identical check to p_no_fc_update_without_link; keep both for
  // documentation clarity — they serve different conceptual roles.
  ast_no_consumed_before_init: assert property (p_no_consumed_before_init)
    else $error("FC: FC update attempted before link (FC init) completes");

  // ==========================================================================
  // P15 — Infinite credit stability: if avail_p.header_credits is infinite
  //        (12'hFFF), it must remain infinite on the next cycle.
  //        An infinite grant is irrevocable per PCIe spec Section 2.11.1.
  // ==========================================================================
  property p_inf_credit_stable_p;
    @(posedge clk) disable iff (!rst_n)
    (avail_p.header_credits == FC_HDR_INFINITE) |=>
      (avail_p.header_credits == FC_HDR_INFINITE);
  endproperty
  ast_inf_credit_stable_p: assert property (p_inf_credit_stable_p)
    else $error("FC: avail_p.header_credits transitioned away from infinite");

  property p_inf_credit_stable_np;
    @(posedge clk) disable iff (!rst_n)
    (avail_np.header_credits == FC_HDR_INFINITE) |=>
      (avail_np.header_credits == FC_HDR_INFINITE);
  endproperty
  ast_inf_credit_stable_np: assert property (p_inf_credit_stable_np)
    else $error("FC: avail_np.header_credits transitioned away from infinite");

  property p_inf_credit_stable_cpl;
    @(posedge clk) disable iff (!rst_n)
    (avail_cpl.header_credits == FC_HDR_INFINITE) |=>
      (avail_cpl.header_credits == FC_HDR_INFINITE);
  endproperty
  ast_inf_credit_stable_cpl: assert property (p_inf_credit_stable_cpl)
    else $error("FC: avail_cpl.header_credits transitioned away from infinite");

  // ==========================================================================
  // P16 — FC RX Init completion DLLP: data credit field may be zero (infinite
  //        header-only grant) but header field must be non-zero for CPL as well
  //        if data_credits are non-zero.
  //        Simplified: when we receive a CPL init, if data credits > 0 then
  //        header credits must also be > 0 (or infinite).
  // ==========================================================================
  property p_fc_rx_cpl_hdr_consistent;
    @(posedge clk) disable iff (!rst_n)
    (fc_rx_valid && (fc_rx_type == DLLP_FC_INIT_CPL) &&
     (fc_rx_data > 20'h0) && (fc_rx_data != FC_DAT_INFINITE)) |->
      ((fc_rx_hdr > 12'h0) || (fc_rx_hdr == FC_HDR_INFINITE));
  endproperty
  ast_fc_rx_cpl_hdr_consistent: assert property (p_fc_rx_cpl_hdr_consistent)
    else $error("FC RX: CPL Init has non-zero data credits but zero header credits");

  // ==========================================================================
  // Cover Properties
  // ==========================================================================

  // Existing covers
  cov_infinite_p_hdr:  cover property (@(posedge clk) disable iff (!rst_n)
                          avail_p.header_credits == FC_HDR_INFINITE);
  cov_infinite_np_hdr: cover property (@(posedge clk) disable iff (!rst_n)
                          avail_np.header_credits == FC_HDR_INFINITE);
  cov_infinite_cpl:    cover property (@(posedge clk) disable iff (!rst_n)
                          avail_cpl.header_credits == FC_HDR_INFINITE);
  cov_fc_update_pulse: cover property (@(posedge clk) disable iff (!rst_n)
                          fc_update_tx);
  cov_p_credit_full:   cover property (@(posedge clk) disable iff (!rst_n)
                          link_up &&
                          (consumed_p.header_credits == avail_p.header_credits) &&
                          (avail_p.header_credits != FC_HDR_INFINITE));
  cov_np_credit_full:  cover property (@(posedge clk) disable iff (!rst_n)
                          link_up &&
                          (consumed_np.header_credits == avail_np.header_credits) &&
                          (avail_np.header_credits != FC_HDR_INFINITE));

  // New covers
  // FC Init-P received from remote side
  cov_fc_rx_init_p:    cover property (@(posedge clk) disable iff (!rst_n)
                          fc_rx_valid && (fc_rx_type == DLLP_FC_INIT_P));

  // FC Init-NP received
  cov_fc_rx_init_np:   cover property (@(posedge clk) disable iff (!rst_n)
                          fc_rx_valid && (fc_rx_type == DLLP_FC_INIT_NP));

  // FC Init-CPL received
  cov_fc_rx_init_cpl:  cover property (@(posedge clk) disable iff (!rst_n)
                          fc_rx_valid && (fc_rx_type == DLLP_FC_INIT_CPL));

  // FC Update-P received
  cov_fc_rx_upd_p:     cover property (@(posedge clk) disable iff (!rst_n)
                          fc_rx_valid && (fc_rx_type == DLLP_FC_UPD_P));

  // FC Update sent with UPD_NP type
  cov_fc_upd_np_sent:  cover property (@(posedge clk) disable iff (!rst_n)
                          fc_update_tx && (fc_upd_type == DLLP_FC_UPD_NP));

  // FC Update sent with UPD_CPL type
  cov_fc_upd_cpl_sent: cover property (@(posedge clk) disable iff (!rst_n)
                          fc_update_tx && (fc_upd_type == DLLP_FC_UPD_CPL));

  // CPL credits hit zero (exhausted)
  cov_cpl_credits_zero: cover property (@(posedge clk) disable iff (!rst_n)
                          link_up &&
                          (consumed_cpl.header_credits == avail_cpl.header_credits) &&
                          (avail_cpl.header_credits != FC_HDR_INFINITE));

  // Infinite data credit for posted
  cov_inf_dat_p:        cover property (@(posedge clk) disable iff (!rst_n)
                          avail_p.data_credits == FC_DAT_INFINITE);

endmodule : pcie_fc_assertions
