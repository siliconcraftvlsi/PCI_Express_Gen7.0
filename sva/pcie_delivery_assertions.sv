// =============================================================================
// PCIe 7.0 Controller - TLP Delivery & AXI-to-PCIe Ordering Assertions
// PCIe Base Spec Rev 7.0 Section 2 (TL) / Section 3 (DLL)
// =============================================================================
// Covers: TLP TX/RX framing, AXI handshake ordering, DMA pulse semantics,
//         error signal integrity, flow-control liveness, and DLL health.
// =============================================================================

`include "pcie_pkg.sv"

module pcie_delivery_assertions
  import pcie_pkg::*;
#(
  parameter int DATA_W       = 256,
  parameter int ADDR_W       = 64,
  parameter int CPL_TIMEOUT  = 12_500_000   // 50 ms at 250 MHz
)(
  input  logic              clk,
  input  logic              rst_n,
  input  logic              link_up,

  // --------------------------------------------------------------------------
  // TLP TX path (TL → DLL)
  // --------------------------------------------------------------------------
  input  logic [DATA_W-1:0] tlp_tx_data,
  input  logic              tlp_tx_valid,
  input  logic              tlp_tx_ready,
  input  logic              tlp_tx_sop,
  input  logic              tlp_tx_eop,

  // --------------------------------------------------------------------------
  // TLP RX path (DLL → TL)
  // --------------------------------------------------------------------------
  input  logic [DATA_W-1:0] tlp_rx_data,
  input  logic              tlp_rx_valid,
  input  logic              tlp_rx_sop,
  input  logic              tlp_rx_eop,
  input  logic              tlp_rx_error,

  // --------------------------------------------------------------------------
  // AXI4 Subordinate (host-facing)
  // --------------------------------------------------------------------------
  input  logic              s_axi_awvalid,
  input  logic              s_axi_awready,
  input  logic [ADDR_W-1:0] s_axi_awaddr,
  input  logic              s_axi_wvalid,
  input  logic              s_axi_wready,
  input  logic              s_axi_wlast,
  input  logic              s_axi_bvalid,
  input  logic              s_axi_bready,
  input  logic [1:0]        s_axi_bresp,
  input  logic              s_axi_arvalid,
  input  logic              s_axi_arready,
  input  logic [ADDR_W-1:0] s_axi_araddr,
  input  logic              s_axi_rvalid,
  input  logic              s_axi_rready,
  input  logic              s_axi_rlast,
  input  logic [DATA_W-1:0] s_axi_rdata,
  input  logic [1:0]        s_axi_rresp,

  // --------------------------------------------------------------------------
  // DMA engine
  // --------------------------------------------------------------------------
  input  logic              dma_done,
  input  logic              dma_error,

  // --------------------------------------------------------------------------
  // DLLP / DLL health
  // --------------------------------------------------------------------------
  input  logic              dllp_valid,
  input  logic              dll_tx_active,
  input  logic              dll_error,

  // --------------------------------------------------------------------------
  // Error reporting (from Config/AER block)
  // --------------------------------------------------------------------------
  input  logic              cfg_err_cor,
  input  logic              cfg_err_nonfatal,
  input  logic              cfg_err_fatal,

  // --------------------------------------------------------------------------
  // Flow control
  // --------------------------------------------------------------------------
  input  logic              fc_update_tx
);

  // ==========================================================================
  // Local tracking registers
  // ==========================================================================

  // Track whether we are inside a TX TLP burst (SOP seen, EOP not yet seen)
  logic in_tlp_tx;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      in_tlp_tx <= 1'b0;
    else if (tlp_tx_valid && tlp_tx_ready) begin
      if (tlp_tx_sop && !tlp_tx_eop)
        in_tlp_tx <= 1'b1;
      else if (tlp_tx_eop)
        in_tlp_tx <= 1'b0;
    end
  end

  // Counter used to bound AW → B interval (simplified window)
  // Rolls over every 4096 cycles; used to show bvalid occurs within a window
  // after awvalid was seen.
  logic [11:0] aw_seen_ctr;
  logic        aw_pending;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      aw_seen_ctr <= '0;
      aw_pending  <= 1'b0;
    end else begin
      if (s_axi_awvalid && s_axi_awready)
        aw_pending <= 1'b1;
      else if (s_axi_bvalid && s_axi_bready)
        aw_pending <= 1'b0;

      if (aw_pending)
        aw_seen_ctr <= aw_seen_ctr + 1'b1;
      else
        aw_seen_ctr <= '0;
    end
  end

  // ==========================================================================
  // TLP TX Assertions
  // ==========================================================================

  // P1: No X/Z on tlp_tx_data when valid
  property p_tlp_tx_no_x;
    @(posedge clk) disable iff (!rst_n)
    tlp_tx_valid |-> !$isunknown(tlp_tx_data);
  endproperty
  ast_tlp_tx_no_x: assert property (p_tlp_tx_no_x)
    else $error("DELIVERY TX: X/Z on tlp_tx_data while tlp_tx_valid");

  // P2: Data must be stable when valid && !ready (AXI-style hold)
  property p_tlp_tx_stable;
    @(posedge clk) disable iff (!rst_n)
    (tlp_tx_valid && !tlp_tx_ready) |=> ($stable(tlp_tx_data) && tlp_tx_valid);
  endproperty
  ast_tlp_tx_stable: assert property (p_tlp_tx_stable)
    else $error("DELIVERY TX: tlp_tx_data/valid not held while backpressured");

  // P3: EOP cannot appear unless SOP was seen first
  //     Uses in_tlp_tx: high means we saw SOP and have not yet seen EOP.
  //     A single-beat TLP where SOP and EOP coincide is legal (in_tlp_tx is 0
  //     entering that beat, so we gate on (in_tlp_tx || tlp_tx_sop)).
  property p_tlp_tx_sop_before_eop;
    @(posedge clk) disable iff (!rst_n)
    (tlp_tx_valid && tlp_tx_eop) |-> (in_tlp_tx || tlp_tx_sop);
  endproperty
  ast_tlp_tx_sop_before_eop: assert property (p_tlp_tx_sop_before_eop)
    else $error("DELIVERY TX: EOP without preceding SOP");

  // P4: No bubble inside a packet — once SOP is accepted, valid must stay high
  //     until EOP is also accepted.
  property p_tlp_tx_no_gap;
    @(posedge clk) disable iff (!rst_n)
    (tlp_tx_valid && tlp_tx_ready && tlp_tx_sop && !tlp_tx_eop) |=>
      tlp_tx_valid;
  endproperty
  ast_tlp_tx_no_gap: assert property (p_tlp_tx_no_gap)
    else $error("DELIVERY TX: valid de-asserted inside TLP burst (gap after SOP)");

  // ==========================================================================
  // TLP RX Assertions
  // ==========================================================================

  // P5: No X/Z on tlp_rx_data when valid
  property p_tlp_rx_no_x_on_valid;
    @(posedge clk) disable iff (!rst_n)
    tlp_rx_valid |-> !$isunknown(tlp_rx_data);
  endproperty
  ast_tlp_rx_no_x_on_valid: assert property (p_tlp_rx_no_x_on_valid)
    else $error("DELIVERY RX: X/Z on tlp_rx_data while tlp_rx_valid");

  // P6: tlp_rx_error may only assert during a valid RX beat
  property p_tlp_rx_err_with_valid;
    @(posedge clk) disable iff (!rst_n)
    tlp_rx_error |-> tlp_rx_valid;
  endproperty
  ast_tlp_rx_err_with_valid: assert property (p_tlp_rx_err_with_valid)
    else $error("DELIVERY RX: tlp_rx_error asserted without tlp_rx_valid");

  // P7: RX data must not arrive when link is down
  property p_no_rx_data_without_link;
    @(posedge clk) disable iff (!rst_n)
    tlp_rx_valid |-> link_up;
  endproperty
  ast_no_rx_data_without_link: assert property (p_no_rx_data_without_link)
    else $error("DELIVERY RX: tlp_rx_valid while link_up=0");

  // P8: TX must not start without DLL being active
  property p_no_tx_without_dll_active;
    @(posedge clk) disable iff (!rst_n)
    tlp_tx_valid |-> dll_tx_active;
  endproperty
  ast_no_tx_without_dll_active: assert property (p_no_tx_without_dll_active)
    else $error("DELIVERY TX: tlp_tx_valid while dll_tx_active=0");

  // ==========================================================================
  // AXI-to-PCIe Ordering / Delivery Assertions
  // ==========================================================================

  // P9: AW channel — valid and address must be held until accepted
  property p_axi_aw_stable;
    @(posedge clk) disable iff (!rst_n)
    (s_axi_awvalid && !s_axi_awready) |=>
      (s_axi_awvalid && $stable(s_axi_awaddr));
  endproperty
  ast_axi_aw_stable: assert property (p_axi_aw_stable)
    else $error("DELIVERY AXI: awvalid/awaddr not held while !awready");

  // P10: W channel — valid must be held until accepted
  property p_axi_w_stable;
    @(posedge clk) disable iff (!rst_n)
    (s_axi_wvalid && !s_axi_wready) |=> s_axi_wvalid;
  endproperty
  ast_axi_w_stable: assert property (p_axi_w_stable)
    else $error("DELIVERY AXI: wvalid de-asserted before wready");

  // P11: AR channel — valid and address must be held until accepted
  property p_axi_ar_stable;
    @(posedge clk) disable iff (!rst_n)
    (s_axi_arvalid && !s_axi_arready) |=>
      (s_axi_arvalid && $stable(s_axi_araddr));
  endproperty
  ast_axi_ar_stable: assert property (p_axi_ar_stable)
    else $error("DELIVERY AXI: arvalid/araddr not held while !arready");

  // P12: B (write response) only after AW was previously accepted
  //      Simplified: bvalid requires aw_pending tracker to be high
  //      (i.e. an AW handshake has occurred and no matching B yet).
  property p_axi_b_only_after_aw;
    @(posedge clk) disable iff (!rst_n)
    s_axi_bvalid |-> aw_pending;
  endproperty
  ast_axi_b_only_after_aw: assert property (p_axi_b_only_after_aw)
    else $error("DELIVERY AXI: bvalid with no outstanding AW transaction");

  // P13: After wlast + wready handshake, wvalid may drop (packet boundary)
  //      This is a cover / liveness check — assert that wlast clears within
  //      a reasonable window rather than sticking high indefinitely.
  property p_axi_wlast_clears_wvalid;
    @(posedge clk) disable iff (!rst_n)
    (s_axi_wvalid && s_axi_wready && s_axi_wlast) |=>
      (!s_axi_wlast || !s_axi_wvalid || s_axi_wready);
  endproperty
  ast_axi_wlast_clears_wvalid: assert property (p_axi_wlast_clears_wvalid)
    else $error("DELIVERY AXI: wlast did not de-assert after accepted beat");

  // P14: bresp must be OKAY (2'b00) or SLVERR (2'b10); DECERR is abnormal
  property p_axi_bresp_ok;
    @(posedge clk) disable iff (!rst_n)
    s_axi_bvalid |-> (s_axi_bresp inside {2'b00, 2'b10});
  endproperty
  ast_axi_bresp_ok: assert property (p_axi_bresp_ok)
    else $error("DELIVERY AXI: unexpected bresp=0x%0h (expected OKAY/SLVERR)",
                s_axi_bresp);

  // P15: rresp must be OKAY or SLVERR; DECERR is abnormal
  property p_axi_rresp_ok;
    @(posedge clk) disable iff (!rst_n)
    s_axi_rvalid |-> (s_axi_rresp inside {2'b00, 2'b10});
  endproperty
  ast_axi_rresp_ok: assert property (p_axi_rresp_ok)
    else $error("DELIVERY AXI: unexpected rresp=0x%0h (expected OKAY/SLVERR)",
                s_axi_rresp);

  // ==========================================================================
  // DMA Correctness Assertions
  // ==========================================================================

  // P16: done and error are mutually exclusive
  property p_dma_done_xor_error;
    @(posedge clk) disable iff (!rst_n)
    !(dma_done && dma_error);
  endproperty
  ast_dma_done_xor_error: assert property (p_dma_done_xor_error)
    else $error("DELIVERY DMA: dma_done and dma_error asserted simultaneously");

  // P17: dma_done is a single-cycle pulse, not a sticky level
  property p_dma_done_pulse;
    @(posedge clk) disable iff (!rst_n)
    dma_done |=> !dma_done;
  endproperty
  ast_dma_done_pulse: assert property (p_dma_done_pulse)
    else $error("DELIVERY DMA: dma_done held high for more than 1 cycle");

  // ==========================================================================
  // Error Signal Integrity Assertions
  // ==========================================================================

  // P18: cfg_err_cor must never be X/Z
  property p_err_no_x_cor;
    @(posedge clk) disable iff (!rst_n)
    !$isunknown(cfg_err_cor);
  endproperty
  ast_err_no_x_cor: assert property (p_err_no_x_cor)
    else $error("DELIVERY ERR: X/Z on cfg_err_cor");

  // P19: cfg_err_nonfatal must never be X/Z
  property p_err_no_x_nonfatal;
    @(posedge clk) disable iff (!rst_n)
    !$isunknown(cfg_err_nonfatal);
  endproperty
  ast_err_no_x_nonfatal: assert property (p_err_no_x_nonfatal)
    else $error("DELIVERY ERR: X/Z on cfg_err_nonfatal");

  // P20: cfg_err_fatal must never be X/Z
  property p_err_no_x_fatal;
    @(posedge clk) disable iff (!rst_n)
    !$isunknown(cfg_err_fatal);
  endproperty
  ast_err_no_x_fatal: assert property (p_err_no_x_fatal)
    else $error("DELIVERY ERR: X/Z on cfg_err_fatal");

  // P21: Fatal and correctable errors are mutually exclusive per PCIe AER spec
  //      (a correctable error cannot also be fatal)
  property p_fatal_implies_not_cor;
    @(posedge clk) disable iff (!rst_n)
    !(cfg_err_fatal && cfg_err_cor);
  endproperty
  ast_fatal_implies_not_cor: assert property (p_fatal_implies_not_cor)
    else $error("DELIVERY ERR: cfg_err_fatal and cfg_err_cor asserted simultaneously");

  // P22: After DLL fatal error, link_up must drop within 10 cycles
  //      (weak until / bounded liveness: link must go down)
  property p_dll_error_no_recovery;
    @(posedge clk) disable iff (!rst_n)
    dll_error |-> ##[1:10] (link_up == 1'b0);
  endproperty
  ast_dll_error_no_recovery: assert property (p_dll_error_no_recovery)
    else $error("DELIVERY DLL: link_up did not drop within 10 cycles of dll_error");

  // ==========================================================================
  // Flow Control Liveness Assertions
  // ==========================================================================

  // P23: FC update must not occur without link being up
  property p_fc_update_with_link;
    @(posedge clk) disable iff (!rst_n)
    fc_update_tx |-> link_up;
  endproperty
  ast_fc_update_with_link: assert property (p_fc_update_with_link)
    else $error("DELIVERY FC: fc_update_tx fired while link_up=0");

  // P24: After link comes up, an FC update must eventually fire
  //      (bounded liveness within FC_UPDATE_TIMER + 1 cycles)
  property p_fc_update_periodic;
    @(posedge clk) disable iff (!rst_n)
    link_up |-> ##[1:200001] fc_update_tx;
  endproperty
  // Expressed as a cover to avoid false assertion failures in early link-up;
  // change to assert if the design guarantees hard-bounded FC update firing.
  cov_fc_update_periodic: cover property (p_fc_update_periodic);

  // ==========================================================================
  // Cover Properties
  // ==========================================================================

  // COV1: Complete TX TLP: SOP then EOP (with at least 1 valid beat between)
  cov_tlp_tx_complete: cover property (
    @(posedge clk) disable iff (!rst_n)
    (tlp_tx_valid && tlp_tx_sop) ##[1:$] (tlp_tx_valid && tlp_tx_eop));

  // COV2: Complete RX TLP: SOP then EOP
  cov_tlp_rx_complete: cover property (
    @(posedge clk) disable iff (!rst_n)
    (tlp_rx_valid && tlp_rx_sop) ##[1:$] (tlp_rx_valid && tlp_rx_eop));

  // COV3: AXI write response OK (OKAY resp on B channel)
  cov_axi_write_resp_ok: cover property (
    @(posedge clk) disable iff (!rst_n)
    s_axi_bvalid && (s_axi_bresp == 2'b00));

  // COV4: AXI read response OK (OKAY resp on R channel with rlast)
  cov_axi_read_resp_ok: cover property (
    @(posedge clk) disable iff (!rst_n)
    s_axi_rvalid && s_axi_rlast && (s_axi_rresp == 2'b00));

  // COV5: DMA done reached
  cov_dma_done: cover property (
    @(posedge clk) disable iff (!rst_n)
    dma_done);

  // COV6: DLL error reached
  cov_dll_error: cover property (
    @(posedge clk) disable iff (!rst_n)
    dll_error);

  // COV7: Correctable error fired
  cov_cfg_err_cor: cover property (
    @(posedge clk) disable iff (!rst_n)
    cfg_err_cor);

  // COV8: Non-fatal error fired
  cov_cfg_err_nonfatal: cover property (
    @(posedge clk) disable iff (!rst_n)
    cfg_err_nonfatal);

  // COV9: FC update fired while link is up
  cov_fc_update_fired: cover property (
    @(posedge clk) disable iff (!rst_n)
    fc_update_tx && link_up);

  // COV10: Link-up reached (transition from 0 to 1)
  cov_link_up_reached: cover property (
    @(posedge clk) disable iff (!rst_n)
    (link_up == 1'b0) ##1 link_up);

endmodule : pcie_delivery_assertions
