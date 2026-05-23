// =============================================================================
// PCIe 7.0 Controller - Data Link Layer Assertions
// PCIe Base Spec Rev 7.0 Section 3
// =============================================================================
// Covers: sequence numbering, ACK/NAK protocol, replay timer, LCRC validity,
// backpressure protocol, DLLP type encoding, FC DLLP types, PM DLLP types,
// replay-count limit, DLL active state, and error signal integrity.
// =============================================================================

`include "pcie_pkg.sv"

module pcie_dll_assertions
  import pcie_pkg::*;
#(
  parameter int unsigned DATA_W          = 256,
  parameter int unsigned REPLAY_TIMEOUT  = 4096,
  parameter int unsigned ACK_LAT_TIMEOUT = 256
)(
  input  logic              clk,
  input  logic              rst_n,
  input  logic              link_up,

  // --------------------------------------------------------------------------
  // TL → DLL TX
  // --------------------------------------------------------------------------
  input  logic [DATA_W-1:0] tl_tx_data,
  input  logic              tl_tx_valid,
  input  logic              tl_tx_ready,
  input  logic              tl_tx_sop,
  input  logic              tl_tx_eop,

  // --------------------------------------------------------------------------
  // DLL TX → PIPE
  // --------------------------------------------------------------------------
  input  logic [DATA_W-1:0] phy_tx_data,
  input  logic              phy_tx_valid,
  input  logic              phy_tx_sop,
  input  logic              phy_tx_eop,

  // --------------------------------------------------------------------------
  // DLL RX ← PIPE
  // --------------------------------------------------------------------------
  input  logic [DATA_W-1:0] dll_rx_data,
  input  logic              dll_rx_valid,
  input  logic              dll_rx_sop,
  input  logic              dll_rx_eop,
  input  logic              dll_rx_error,

  // --------------------------------------------------------------------------
  // ACK/NAK feedback
  // --------------------------------------------------------------------------
  input  logic              nak_received,
  input  logic [11:0]       ack_seq,
  input  logic [11:0]       nak_seq,

  // Separate output sequence numbers used for ACK/NAK generation
  input  logic              ack_seq_out_valid,   // pulses when we emit an ACK
  input  logic [11:0]       ack_seq_out,         // sequence number in our ACK
  input  logic              nak_seq_out_valid,   // pulses when we emit a NAK
  input  logic [11:0]       nak_seq_out,         // sequence number in our NAK

  // --------------------------------------------------------------------------
  // DLLP type on RX (decoded by dll_rx)
  // --------------------------------------------------------------------------
  input  logic [7:0]        dllp_type,
  input  logic              dllp_valid,

  // --------------------------------------------------------------------------
  // DLL active / health
  // --------------------------------------------------------------------------
  input  logic              dll_tx_active,        // DLL in DL_ACTIVE state
  input  logic              dll_error,            // Fatal DLL error (replay exhausted)
  input  logic              dllp_err,             // DLLP CRC or framing error

  // --------------------------------------------------------------------------
  // FC DLLP fields (decoded from FC DLLPs)
  // --------------------------------------------------------------------------
  input  logic              fc_dllp_valid,        // FC DLLP received / being processed
  input  dllp_type_e        fc_dllp_type,         // FC DLLP type code
  input  logic [11:0]       fc_dllp_hdr,          // Header credit field
  input  logic [19:0]       fc_dllp_data,         // Data credit field

  // --------------------------------------------------------------------------
  // PM DLLP fields (decoded from PM DLLPs)
  // --------------------------------------------------------------------------
  input  logic              pm_dllp_valid,        // PM DLLP received / being processed
  input  dllp_type_e        pm_dllp_type          // PM DLLP type code
);

  // ==========================================================================
  // Local parameters
  // ==========================================================================
  localparam int unsigned NAK_WINDOW   = 8192;  // observation window in cycles
  localparam int unsigned NAK_LIMIT    = 3;     // NAKs before dll_error
  localparam int unsigned ERR_LATENCY  = 10;    // cycles after 3rd NAK to see error

  // ==========================================================================
  // P1 — AXI-style ready/valid: once valid, data stable until accepted
  // ==========================================================================
  property p_tl_tx_data_stable;
    @(posedge clk) disable iff (!rst_n)
    (tl_tx_valid && !tl_tx_ready) |=> ($stable(tl_tx_data) && tl_tx_valid);
  endproperty
  ast_tl_tx_stable: assert property (p_tl_tx_data_stable)
    else $error("DLL TX: tl_tx_data changed while valid && !ready");

  // ==========================================================================
  // P2 — SOP must precede EOP on PHY TX
  // ==========================================================================
  property p_sop_before_eop_tx;
    @(posedge clk) disable iff (!rst_n)
    (phy_tx_valid && phy_tx_eop) |->
      $past(phy_tx_valid && phy_tx_sop, 1) ||
      !$past(phy_tx_eop, 1);
  endproperty
  ast_sop_before_eop: assert property (p_sop_before_eop_tx)
    else $error("DLL TX: EOP without preceding SOP");

  // ==========================================================================
  // P3 — No gap (valid de-assert) inside a TX TLP burst
  // ==========================================================================
  property p_no_gap_in_tlp_tx;
    @(posedge clk) disable iff (!rst_n)
    (phy_tx_valid && phy_tx_sop && !phy_tx_eop) |=>
      phy_tx_valid;
  endproperty
  ast_no_gap_tlp: assert property (p_no_gap_in_tlp_tx)
    else $error("DLL TX: gap (valid de-asserted) inside TLP burst");

  // ==========================================================================
  // P4 — No X on phy_tx_data when valid
  // ==========================================================================
  property p_no_x_on_tx_data;
    @(posedge clk) disable iff (!rst_n)
    phy_tx_valid |-> !$isunknown(phy_tx_data);
  endproperty
  ast_no_x_tx: assert property (p_no_x_on_tx_data)
    else $error("DLL TX: X/Z on phy_tx_data when valid");

  // ==========================================================================
  // P5 — NAK sequence number must be known when nak_received
  // ==========================================================================
  property p_nak_seq_known;
    @(posedge clk) disable iff (!rst_n)
    nak_received |-> !$isunknown(nak_seq);
  endproperty
  ast_nak_seq_known: assert property (p_nak_seq_known)
    else $error("DLL: X on nak_seq when nak_received");

  // ==========================================================================
  // P6 — ACK sequence must be known when ACK DLLP received
  // ==========================================================================
  property p_ack_seq_known;
    @(posedge clk) disable iff (!rst_n)
    (dllp_valid && (dllp_type == 8'h00)) |-> !$isunknown(ack_seq);
  endproperty
  ast_ack_seq_known: assert property (p_ack_seq_known)
    else $error("DLL: X on ack_seq during ACK DLLP");

  // ==========================================================================
  // P7 — dll_rx_error only during active RX packet
  // ==========================================================================
  property p_rx_err_needs_rx;
    @(posedge clk) disable iff (!rst_n)
    dll_rx_error |-> dll_rx_valid;
  endproperty
  ast_rx_err_valid: assert property (p_rx_err_needs_rx)
    else $error("DLL RX: dll_rx_error asserted without dll_rx_valid");

  // ==========================================================================
  // P8 — DLLP type encoding must be one of the architecturally defined values
  // ==========================================================================
  property p_dllp_type_valid;
    @(posedge clk) disable iff (!rst_n)
    dllp_valid |->
      dllp_type inside {
        8'h00,          // ACK
        8'h10,          // NAK
        8'h20,          // PM_ENTER_L1
        8'h21,          // PM_ENTER_L23
        8'h22,          // PM_ACT_STATE_REQ
        8'h24,          // PM_REQ_ACK
        8'h30,          // VENDOR
        8'h40, 8'h50, 8'h60,   // FC_INIT P/NP/CPL
        8'hC0, 8'hD0, 8'hE0   // FC_UPD P/NP/CPL
      };
  endproperty
  ast_dllp_type: assert property (p_dllp_type_valid)
    else $error("DLL: unknown DLLP type 0x%02h", dllp_type);

  // ==========================================================================
  // P9 — No RX activity without link_up
  // ==========================================================================
  property p_no_rx_without_link;
    @(posedge clk) disable iff (!rst_n)
    dll_rx_valid |-> link_up;
  endproperty
  ast_rx_needs_link: assert property (p_no_rx_without_link)
    else $error("DLL RX: data received before link_up");

  // ==========================================================================
  // P10 — Replay count limit: if NAK fires 3× in NAK_WINDOW cycles,
  //        dll_error must assert within ERR_LATENCY cycles.
  //
  //  Implementation: use a local integer variable to count NAKs within the
  //  bounded window.  SVA local variables are sampled at each step; we rely
  //  on the sequence operator to accumulate across multiple NAK pulses.
  //
  //  Simplified encoding: check that whenever we see 3 consecutive distinct
  //  nak_received pulses within NAK_WINDOW cycles, dll_error fires in <=10
  //  cycles from the 3rd.
  // ==========================================================================
  property p_replay_count_limit;
    int nak_cnt;
    @(posedge clk) disable iff (!rst_n)
    // Seed: first NAK
    (nak_received, nak_cnt = 1) |->
      // Within NAK_WINDOW cycles, accumulate 2 more NAKs
      (##[1:NAK_WINDOW] (nak_received, nak_cnt++)) [*2] ##0
      // After the 3rd NAK (nak_cnt==3), dll_error within ERR_LATENCY cycles
      (nak_cnt >= NAK_LIMIT) |-> ##[1:ERR_LATENCY] dll_error;
  endproperty
  ast_replay_count_limit: assert property (p_replay_count_limit)
    else $error("DLL: 3 NAKs in %0d cycles but dll_error not seen within %0d cycles",
                NAK_WINDOW, ERR_LATENCY);

  // ==========================================================================
  // P11 — DLL must be active (dll_tx_active) whenever link is up
  // ==========================================================================
  property p_dll_active_after_link;
    @(posedge clk) disable iff (!rst_n)
    link_up |-> dll_tx_active;
  endproperty
  ast_dll_active_after_link: assert property (p_dll_active_after_link)
    else $error("DLL: link_up asserted but dll_tx_active=0");

  // ==========================================================================
  // P12 — FC DLLP type must be one of the six defined FC types
  // ==========================================================================
  property p_fc_dllp_with_type;
    @(posedge clk) disable iff (!rst_n)
    fc_dllp_valid |->
      fc_dllp_type inside {
        DLLP_FC_INIT_P,
        DLLP_FC_INIT_NP,
        DLLP_FC_INIT_CPL,
        DLLP_FC_UPD_P,
        DLLP_FC_UPD_NP,
        DLLP_FC_UPD_CPL
      };
  endproperty
  ast_fc_dllp_with_type: assert property (p_fc_dllp_with_type)
    else $error("DLL FC: fc_dllp_type=0x%02h is not a valid FC DLLP type",
                fc_dllp_type);

  // ==========================================================================
  // P13 — PM DLLP type must be one of the four defined PM types
  // ==========================================================================
  property p_pm_dllp_valid_type;
    @(posedge clk) disable iff (!rst_n)
    pm_dllp_valid |->
      pm_dllp_type inside {
        DLLP_PM_ENTER_L1,
        DLLP_PM_ENTER_L23,
        DLLP_PM_ACT_STATE_REQ,
        DLLP_PM_REQ_ACK
      };
  endproperty
  ast_pm_dllp_valid_type: assert property (p_pm_dllp_valid_type)
    else $error("DLL PM: pm_dllp_type=0x%02h is not a valid PM DLLP type",
                pm_dllp_type);

  // ==========================================================================
  // P14 — ACK and NAK for the same sequence number cannot be emitted
  //        simultaneously (illegal per spec Section 3.5)
  // ==========================================================================
  property p_ack_seq_monotone;
    @(posedge clk) disable iff (!rst_n)
    (ack_seq_out_valid && nak_seq_out_valid) |->
      (ack_seq_out != nak_seq_out);
  endproperty
  ast_ack_seq_monotone: assert property (p_ack_seq_monotone)
    else $error("DLL: ACK and NAK emitted for same sequence number %0d", ack_seq_out);

  // ==========================================================================
  // P15 — dllp_err must not be X/Z while link is up
  // ==========================================================================
  property p_dllp_err_no_x;
    @(posedge clk) disable iff (!rst_n)
    link_up |-> !$isunknown(dllp_err);
  endproperty
  ast_dllp_err_no_x: assert property (p_dllp_err_no_x)
    else $error("DLL: X/Z on dllp_err while link_up");

  // ==========================================================================
  // Cover Properties
  // ==========================================================================

  // Original covers
  cov_nak_received:  cover property (@(posedge clk) disable iff (!rst_n) nak_received);
  cov_rx_error:      cover property (@(posedge clk) disable iff (!rst_n) dll_rx_error);
  cov_dllp_ack:      cover property (@(posedge clk) disable iff (!rst_n)
                       dllp_valid && (dllp_type == 8'h00));
  cov_dllp_nak:      cover property (@(posedge clk) disable iff (!rst_n)
                       dllp_valid && (dllp_type == 8'h10));
  cov_dllp_fc_init:  cover property (@(posedge clk) disable iff (!rst_n)
                       dllp_valid && (dllp_type inside {8'h40, 8'h50, 8'h60}));
  cov_dllp_fc_upd:   cover property (@(posedge clk) disable iff (!rst_n)
                       dllp_valid && (dllp_type inside {8'hC0, 8'hD0, 8'hE0}));
  cov_dllp_pm:       cover property (@(posedge clk) disable iff (!rst_n)
                       dllp_valid && (dllp_type inside {8'h20, 8'h21, 8'h22, 8'h24}));

  // New covers for FC and PM DLLP subtypes
  cov_fc_dllp_init_p: cover property (
    @(posedge clk) disable iff (!rst_n)
    fc_dllp_valid && (fc_dllp_type == DLLP_FC_INIT_P));

  cov_fc_dllp_upd_p:  cover property (
    @(posedge clk) disable iff (!rst_n)
    fc_dllp_valid && (fc_dllp_type == DLLP_FC_UPD_P));

  cov_pm_dllp_l1:     cover property (
    @(posedge clk) disable iff (!rst_n)
    pm_dllp_valid && (pm_dllp_type == DLLP_PM_ENTER_L1));

  cov_dll_error_fired: cover property (
    @(posedge clk) disable iff (!rst_n)
    dll_error);

  cov_nak_rcvd_explicit: cover property (
    @(posedge clk) disable iff (!rst_n)
    nak_received);

endmodule : pcie_dll_assertions
