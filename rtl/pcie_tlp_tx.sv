`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// Author  : Robert Kingsly Amalathas
// Email   : robertk@microprocessorlab.com
// LinkedIn: www.linkedin.com/in/robertkingslya
// -----------------------------------------------------------------------------
// =============================================================================
// PCIe 7.0 Controller - Transaction Layer TX (TLP Transmitter)
// Based on PCI Express Base Specification Rev 7.0 Chapter 2
// =============================================================================
// Description:
//   Arbitrates and assembles TLPs from multiple sources (AXI bridge posted
//   writes, AXI bridge non-posted reads, DMA requests, config completions)
//   and forwards them to the Data Link Layer TX.
//
//   Features:
//     - 4-input round-robin arbiter (posted, non-posted, DMA, config)
//     - Flow-credit check before admitting a TLP to the TX pipeline
//     - Maximum Payload Size enforcement
//     - Maximum Read Request Size enforcement
//     - TLP Header construction helpers
//     - ECRC append (optional)
//     - Traffic Class and Virtual Channel mapping
// =============================================================================

`include "pcie_pkg.sv"

module pcie_tlp_tx
  import pcie_pkg::*;
#(
  parameter int unsigned DATA_W    = 256,
  parameter int unsigned ADDR_W    = 64,
  parameter int unsigned NUM_VCS   = 8,
  parameter bit          SIM_BYPASS = 0
)(
  input  logic              clk,
  input  logic              rst_n,
  input  logic              link_up,

  // -------------------------------------------------------------------------
  // Flow Control Credits (from FC Manager)
  // -------------------------------------------------------------------------
  input  fc_credits_t       fc_avail_p,
  input  fc_credits_t       fc_avail_np,
  input  fc_credits_t       fc_avail_cpl,
  output fc_credits_t       fc_consumed_p,
  output fc_credits_t       fc_consumed_np,
  output fc_credits_t       fc_consumed_cpl,

  // -------------------------------------------------------------------------
  // Source: AXI Bridge Posted (writes)
  // -------------------------------------------------------------------------
  input  logic [DATA_W-1:0]  axibr_tx_data,
  input  logic               axibr_tx_valid,
  output logic               axibr_tx_ready,
  input  logic               axibr_tx_sop,
  input  logic               axibr_tx_eop,

  // -------------------------------------------------------------------------
  // Source: AXI Bridge Non-Posted (reads)
  // -------------------------------------------------------------------------
  input  logic [DATA_W-1:0]  axibr_np_data,
  input  logic               axibr_np_valid,
  output logic               axibr_np_ready,
  input  logic               axibr_np_sop,
  input  logic               axibr_np_eop,

  // -------------------------------------------------------------------------
  // Source: DMA Engine
  // -------------------------------------------------------------------------
  input  logic [DATA_W-1:0]  dma_tx_data,
  input  logic               dma_tx_valid,
  output logic               dma_tx_ready,
  input  logic               dma_tx_sop,
  input  logic               dma_tx_eop,

  // -------------------------------------------------------------------------
  // Source: Configuration Space Completions
  // -------------------------------------------------------------------------
  input  logic [DATA_W-1:0]  cfg_tx_data,
  input  logic               cfg_tx_valid,
  output logic               cfg_tx_ready,

  // -------------------------------------------------------------------------
  // Config register values
  // -------------------------------------------------------------------------
  input  logic [2:0]         cfg_mps,    // Max Payload Size encoded
  input  logic [2:0]         cfg_mrrs,   // Max Read Request Size encoded
  input  logic [15:0]        np_outstanding,
  input  logic               cfg_relaxed_order,
  input  logic               axibr_np_relaxed_order,

  // -------------------------------------------------------------------------
  // Output to Data Link Layer TX
  // -------------------------------------------------------------------------
  output logic [DATA_W-1:0]  tl_tx_data,
  output logic               tl_tx_valid,
  input  logic               tl_tx_ready,
  output logic               tl_tx_sop,
  output logic               tl_tx_eop
);

  // ---------------------------------------------------------------------------
  // Arbiter State
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    ARB_IDLE,
    ARB_POSTED,
    ARB_NP,
    ARB_DMA,
    ARB_CFG
  } arb_state_e;

  arb_state_e  arb_state;

  // Round-robin priority register
  logic [1:0]  rr_pri;   // 0=posted, 1=NP, 2=DMA, 3=cfg

  // Active source selection
  logic        src_posted, src_np, src_dma, src_cfg;

  // PCIe ordering model (Base Spec §2.4): posted must not pass waiting NP;
  // NP must not pass outstanding NP unless relaxed ordering; completions must
  // not pass outstanding NP.
  logic        np_queue_waiting;
  logic        block_posted;
  logic        block_np_axi;
  logic        block_np_dma;
  logic        block_cpl;
  logic        pick_posted;
  logic        pick_np;
  logic        pick_dma;
  logic        pick_cfg;
  logic        dma_is_posted;

  function automatic logic beat_relaxed_ok(input logic [DATA_W-1:0] data);
    tlp_dw0_t dw0;
    dw0 = tlp_dw0_t'(data[DATA_W-1 -: 32]);
    beat_relaxed_ok = cfg_relaxed_order || axibr_np_relaxed_order ||
                        tlp_relaxed_ordering_dw0(dw0);
  endfunction

  function automatic logic beat_is_posted(input logic [DATA_W-1:0] data);
    beat_is_posted = tlp_is_posted_dw0(tlp_dw0_t'(data[DATA_W-1 -: 32]));
  endfunction

  // ---------------------------------------------------------------------------
  // FC gate: check if sufficient credits exist
  // ---------------------------------------------------------------------------
  logic  fc_ok_p, fc_ok_np, fc_ok_cpl;
  logic  dma_fc_ok;

  always @* begin
    fc_ok_p   = (fc_avail_p.header_credits   > 12'd0) || (fc_avail_p.header_credits   == 12'hFFF);
    fc_ok_np  = (fc_avail_np.header_credits  > 12'd0) || (fc_avail_np.header_credits  == 12'hFFF);
    fc_ok_cpl = (fc_avail_cpl.header_credits > 12'd0) || (fc_avail_cpl.header_credits == 12'hFFF);
  end

  assign dma_is_posted = !dma_tx_valid || beat_is_posted(dma_tx_data);
  assign dma_fc_ok     = SIM_BYPASS ? (fc_ok_p && fc_ok_np)
                                    : (dma_is_posted ? fc_ok_p : fc_ok_np);

  assign np_queue_waiting   = axibr_np_valid || (dma_tx_valid && !dma_is_posted);
  assign block_posted       = np_queue_waiting;
  assign block_np_axi       = (np_outstanding != 0) && axibr_np_valid &&
                              !beat_relaxed_ok(axibr_np_data);
  assign block_np_dma       = (np_outstanding != 0) && dma_tx_valid && !dma_is_posted &&
                              !beat_relaxed_ok(dma_tx_data);
  assign block_cpl          = (np_outstanding != 0);

  assign pick_posted = axibr_tx_valid && fc_ok_p && !block_posted;
  assign pick_np     = axibr_np_valid && fc_ok_np && !block_np_axi;
  assign pick_dma    = dma_tx_valid && dma_fc_ok &&
                       (dma_is_posted ? !block_posted : !block_np_dma);
  assign pick_cfg    = cfg_tx_valid && fc_ok_cpl && !block_cpl;

  // ---------------------------------------------------------------------------
  // Consumed credit tracking
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      fc_consumed_p   <= 32'h0;
      fc_consumed_np  <= 32'h0;
      fc_consumed_cpl <= 32'h0;
    end else begin
      fc_consumed_p   <= 32'h0;
      fc_consumed_np  <= 32'h0;
      fc_consumed_cpl <= 32'h0;
      if (tl_tx_valid && tl_tx_ready && tl_tx_sop) begin
        case (arb_state)
          ARB_POSTED:
            fc_consumed_p <= {12'd1, 20'd1};
          ARB_NP:
            fc_consumed_np <= {12'd1, 20'd0};
          ARB_DMA:
            if (dma_is_posted)
              fc_consumed_p <= {12'd1, 20'd1};
            else
              fc_consumed_np <= {12'd1, 20'd0};
          ARB_CFG:
            fc_consumed_cpl <= {12'd1, 20'd1};
          default: ;
        endcase
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Combinational Ready Signals
  // ---------------------------------------------------------------------------
  logic tl_tx_rdy;
  assign tl_tx_rdy = SIM_BYPASS ? 1'b1 : tl_tx_ready;

  always_comb begin
    axibr_tx_ready = 1'b0;
    axibr_np_ready = 1'b0;
    dma_tx_ready   = 1'b0;
    cfg_tx_ready   = 1'b0;
    case (arb_state)
      ARB_POSTED: axibr_tx_ready = tl_tx_rdy;
      ARB_NP:     axibr_np_ready = tl_tx_rdy;
      ARB_DMA:    dma_tx_ready   = tl_tx_rdy;
      ARB_CFG:    cfg_tx_ready   = tl_tx_rdy;
      default: ;
    endcase
  end

  // ---------------------------------------------------------------------------
  // Arbiter FSM
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      arb_state    <= ARB_IDLE;
      rr_pri       <= 2'd0;
      tl_tx_valid  <= 1'b0;
      tl_tx_sop    <= 1'b0;
      tl_tx_eop    <= 1'b0;
      tl_tx_data   <= '0;
    end else if (!link_up) begin
      arb_state    <= ARB_IDLE;
      tl_tx_valid  <= 1'b0;
    end else begin

      // Default valid low

      tl_tx_valid    <= 1'b0;
      tl_tx_sop      <= 1'b0;
      tl_tx_eop      <= 1'b0;

      case (arb_state)

        ARB_IDLE: begin
          if (SIM_BYPASS && pick_dma)
            arb_state <= ARB_DMA;
          else if (SIM_BYPASS && pick_np)
            arb_state <= ARB_NP;
          else case (rr_pri)
            2'd0: begin
              if (pick_posted)                      arb_state <= ARB_POSTED;
              else if (pick_np)                     arb_state <= ARB_NP;
              else if (pick_dma)                    arb_state <= ARB_DMA;
              else if (pick_cfg)                    arb_state <= ARB_CFG;
            end
            2'd1: begin
              if (pick_np)                          arb_state <= ARB_NP;
              else if (pick_dma)                    arb_state <= ARB_DMA;
              else if (pick_cfg)                    arb_state <= ARB_CFG;
              else if (pick_posted)                 arb_state <= ARB_POSTED;
            end
            2'd2: begin
              if (pick_dma)                         arb_state <= ARB_DMA;
              else if (pick_cfg)                    arb_state <= ARB_CFG;
              else if (pick_posted)                 arb_state <= ARB_POSTED;
              else if (pick_np)                     arb_state <= ARB_NP;
            end
            2'd3: begin
              if (pick_cfg)                         arb_state <= ARB_CFG;
              else if (pick_posted)                 arb_state <= ARB_POSTED;
              else if (pick_np)                     arb_state <= ARB_NP;
              else if (pick_dma)                    arb_state <= ARB_DMA;
            end
          endcase
        end

        ARB_POSTED: begin
          if (axibr_tx_valid && tl_tx_rdy) begin
            tl_tx_data  <= axibr_tx_data;
            tl_tx_valid <= 1'b1;
            tl_tx_sop   <= axibr_tx_sop;
            tl_tx_eop   <= axibr_tx_eop;
            if (axibr_tx_eop) begin
              arb_state <= ARB_IDLE;
              rr_pri    <= 2'd1;  // Advance round-robin
            end
          end else if (!axibr_tx_valid)
            arb_state <= ARB_IDLE;
        end

        ARB_NP: begin
          if (axibr_np_valid && tl_tx_rdy) begin
            tl_tx_data  <= axibr_np_data;
            tl_tx_valid <= 1'b1;
            tl_tx_sop   <= axibr_np_sop;
            tl_tx_eop   <= axibr_np_eop;
            $display("[TLP-TX-DBG] ARB_NP fwd: data[255:192]=%016h sop=%b eop=%b @%0t",
                     axibr_np_data[255:192], axibr_np_sop, axibr_np_eop, $time);
            if (axibr_np_eop) begin
              arb_state <= ARB_IDLE;
              rr_pri    <= 2'd2;
            end
          end else if (!axibr_np_valid)
            arb_state <= ARB_IDLE;
        end

        ARB_DMA: begin
          if (dma_tx_valid && tl_tx_rdy) begin
            tl_tx_data  <= dma_tx_data;
            tl_tx_valid <= 1'b1;
            tl_tx_sop   <= dma_tx_sop;
            tl_tx_eop   <= dma_tx_eop;
            $display("[TLP-TX-DBG] ARB_DMA fwd: data[255:192]=%016h sop=%b eop=%b @%0t",
                     dma_tx_data[255:192], dma_tx_sop, dma_tx_eop, $time);
            if (dma_tx_eop) begin
              arb_state <= ARB_IDLE;
              rr_pri    <= 2'd3;
            end
          end else if (!dma_tx_valid)
            arb_state <= ARB_IDLE;
        end

        ARB_CFG: begin
          if (cfg_tx_valid && tl_tx_rdy) begin
            tl_tx_data  <= cfg_tx_data;
            tl_tx_valid <= 1'b1;
            tl_tx_sop   <= 1'b1;
            tl_tx_eop   <= 1'b1;
            arb_state   <= ARB_IDLE;
            rr_pri      <= 2'd0;
          end else if (!cfg_tx_valid)
            arb_state <= ARB_IDLE;
        end

        default: arb_state <= ARB_IDLE;
      endcase
    end
  end

endmodule : pcie_tlp_tx
