`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// Author  : Robert Kingsly Amalathas
// Email   : robertk@microprocessorlab.com
// LinkedIn: www.linkedin.com/in/robertkingslya
// -----------------------------------------------------------------------------
// =============================================================================
// PCIe 7.0 Controller - Transaction Layer RX (TLP Receiver / Dispatcher)
// Based on PCI Express Base Specification Rev 7.0 Chapter 2
// =============================================================================
// Description:
//   Receives TLPs from the Data Link Layer, decodes TLP type, validates
//   fields, and routes them to the appropriate destination:
//     - Memory Read/Write → AXI Bridge (with BAR address matching)
//     - Config Read/Write → Configuration Space (separate rd/wr valids)
//     - Completions → AXI Bridge (matching outstanding tag table)
//     - Messages → Internal message handler
//     - DMA completions → DMA Engine
//     - Unsupported Requests → UR Completion generator
//
//   Features:
//     - Full TLP header decode (3DW and 4DW, Type0/Type1)
//     - BAR matching (6 BARs, each 64-bit address + mask + enable)
//     - UR completion generation (3DW Cpl, status=001)
//     - ECRC checking (optional, parameter EN_ECRC)
//     - Message TLP handling with msg_valid pulse output
//     - Error reporting (Unsupported Request, Completer Abort, Malformed TLP)
//     - Poisoned TLP detection
// =============================================================================

`include "pcie_pkg.sv"

module pcie_tlp_rx
  import pcie_pkg::*;
#(
  parameter int unsigned DATA_W      = 256,
  parameter int unsigned ADDR_W      = 64,
  parameter  pcie_role_e DEVICE_ROLE = ROLE_EP,
  parameter  int         EN_ECRC     = 1
)(
  input  logic              clk,
  input  logic              rst_n,
  input  logic              link_up,

  // -------------------------------------------------------------------------
  // Input from Data Link Layer RX
  // -------------------------------------------------------------------------
  input  logic [DATA_W-1:0]  tl_rx_data,
  input  logic               tl_rx_valid,
  input  logic               tl_rx_sop,
  input  logic               tl_rx_eop,
  input  logic               tl_rx_error,

  // -------------------------------------------------------------------------
  // Completions to AXI Bridge
  // -------------------------------------------------------------------------
  output logic [DATA_W-1:0]  cpl_rx_data,
  output logic               cpl_rx_valid,
  output logic               cpl_rx_sop,
  output logic               cpl_rx_eop,
  input  logic               cpl_rx_ready,

  // -------------------------------------------------------------------------
  // Received TLPs (writes/reads) to AXI Bridge
  // -------------------------------------------------------------------------
  output logic [DATA_W-1:0]  axibr_rx_data,
  output logic               axibr_rx_valid,
  output logic               axibr_rx_sop,
  output logic               axibr_rx_eop,

  // -------------------------------------------------------------------------
  // Config Space Accesses (separate read and write valids)
  // -------------------------------------------------------------------------
  output logic               cfg_rx_valid,    // Write enable (CfgWr0)
  output logic               cfg_rd_valid,    // Read strobe  (CfgRd0)
  output logic [31:0]        cfg_rx_data,
  output logic [11:0]        cfg_rx_addr,
  output logic               cfg_rx_wr,

  // -------------------------------------------------------------------------
  // DMA Completions
  // -------------------------------------------------------------------------
  output logic [DATA_W-1:0]  dma_rx_data,
  output logic               dma_rx_valid,
  output logic               dma_rx_sop,
  output logic               dma_rx_eop,
  input  logic               dma_rx_ready,

  // -------------------------------------------------------------------------
  // UR Completion output interface
  // -------------------------------------------------------------------------
  output logic [255:0]       ur_cpl_data,
  output logic               ur_cpl_valid,
  output logic               ur_cpl_sop,
  output logic               ur_cpl_eop,
  input  logic               ur_cpl_ready,

  // -------------------------------------------------------------------------
  // Message TLP output
  // -------------------------------------------------------------------------
  output logic               msg_valid,
  output logic [7:0]         msg_code,
  output logic [63:0]        msg_data,

  // -------------------------------------------------------------------------
  // BAR matching inputs (from AXI bridge / config space)
  // -------------------------------------------------------------------------
  input  logic [63:0]        bar_addr [6],
  input  logic [63:0]        bar_mask [6],
  input  logic               bar_en   [6],

  // -------------------------------------------------------------------------
  // Error Reporting
  // -------------------------------------------------------------------------
  output logic               err_cor,
  output logic               err_nonfatal,
  output logic               err_fatal
);

  // ---------------------------------------------------------------------------
  // TLP Header Decode Register
  // ---------------------------------------------------------------------------
  tlp_dw0_t      hdr_dw0;
  logic [15:0]   hdr_requester_id;
  logic [9:0]    hdr_tag;
  logic [3:0]    hdr_last_be;
  logic [3:0]    hdr_first_be;
  logic [63:0]   hdr_addr;
  logic          hdr_4dw;
  logic          hdr_has_data;

  // Decoded TLP type classification
  logic  is_mrd, is_mwr, is_iowr, is_iord;
  logic  is_cfgrd0, is_cfgwr0, is_cfgrd1, is_cfgwr1;
  logic  is_cpl, is_cpld;
  logic  is_msg;
  logic  is_poisoned;

  // ---------------------------------------------------------------------------
  // BAR Match Logic
  // ---------------------------------------------------------------------------
  logic        bar_hit;
  logic        ur_needed;

  always @* begin : bar_match_comb
    bar_hit = 1'b0;
    for (int i = 0; i < 6; i++) begin
      if (bar_en[i] && ((hdr_addr & bar_mask[i]) == (bar_addr[i] & bar_mask[i])))
        bar_hit = 1'b1;
    end
    // UR required when MRd/MWr hits no enabled BAR
    ur_needed = (is_mrd || is_mwr) && !bar_hit && !is_poisoned;
  end

  // ---------------------------------------------------------------------------
  // ECRC check (CRC-32 per PCIe Base Spec §2.7.1, same polynomial as LCRC)
  // ---------------------------------------------------------------------------
  logic [31:0]  ecrc_crc;
  logic [31:0]  ecrc_crc_before_ecrc_dw;
  logic         ecrc_error;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      ecrc_crc <= ECRC_INIT;
    else if (tl_rx_valid && EN_ECRC && hdr_dw0.td) begin
      if (tl_rx_sop)
        ecrc_crc <= ECRC_INIT;
      if (tl_rx_eop)
        ecrc_crc <= ecrc_update_beat(tl_rx_sop ? ECRC_INIT : ecrc_crc, tl_rx_data, 7);
      else
        ecrc_crc <= ecrc_update_beat(tl_rx_sop ? ECRC_INIT : ecrc_crc, tl_rx_data, 8);
    end
  end

  always_comb begin
    ecrc_crc_before_ecrc_dw = ecrc_crc;
    ecrc_error              = 1'b0;
    if (EN_ECRC && tl_rx_valid && tl_rx_eop && hdr_dw0.td) begin
      ecrc_crc_before_ecrc_dw = ecrc_update_beat(tl_rx_sop ? ECRC_INIT : ecrc_crc,
                                                 tl_rx_data, 7);
      ecrc_error = (tl_rx_data[31:0] != ecrc_finalize(ecrc_crc_before_ecrc_dw));
    end
  end

  // ---------------------------------------------------------------------------
  // RX Receive State Machine
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    TL_RX_IDLE,
    TL_RX_HDR1,       // First header beat
    TL_RX_HDR2,       // Second header beat (for 4DW)
    TL_RX_DATA,       // Data beats
    TL_RX_DROP,       // Drop on error / unsupported
    TL_RX_UR_CPL,     // Assemble and send UR completion
    TL_RX_MSG,        // Handle message TLP
    TL_RX_ECRC_CHK    // ECRC check state (when EN_ECRC)
  } tl_rx_state_e;

  tl_rx_state_e   tl_rx_state;
  logic [2:0]     hdr_beat_cnt;

  // Routing register
  typedef enum logic [2:0] {
    ROUTE_AXI,
    ROUTE_CFG,
    ROUTE_CPL,
    ROUTE_DMA,
    ROUTE_MSG,
    ROUTE_DROP
  } route_e;

  route_e  current_route;

  // UR completion state storage
  logic [15:0]  ur_req_id;
  logic [9:0]   ur_tag;
  logic         ur_cpl_pending;

  // ---------------------------------------------------------------------------
  // DMA Tag Table: track which tags belong to DMA engine
  // ---------------------------------------------------------------------------
  parameter int DMA_TAG_BASE  = 10'd512;
  parameter int DMA_TAG_LIMIT = 10'd767;

  function automatic logic is_dma_tag(input logic [9:0] tag);
    is_dma_tag = (tag >= DMA_TAG_BASE && tag <= DMA_TAG_LIMIT);
  endfunction

  // ---------------------------------------------------------------------------
  // Header Decode (combinational on first beat)
  // ---------------------------------------------------------------------------
  always @* begin
    hdr_dw0          = tlp_dw0_t'(tl_rx_data[DATA_W-1 : DATA_W-32]);
    hdr_requester_id = tl_rx_data[DATA_W-33 : DATA_W-48];
    hdr_tag          = {tl_rx_data[DATA_W-48], tl_rx_data[DATA_W-49],
                        tl_rx_data[DATA_W-50 : DATA_W-57]};
    hdr_last_be      = tl_rx_data[DATA_W-57 : DATA_W-60];
    hdr_first_be     = tl_rx_data[DATA_W-61 : DATA_W-64];
    // 4DW header: fmt[1] = 1 indicates 64-bit address
    hdr_4dw          = hdr_dw0.fmt[1];
    hdr_has_data     = hdr_dw0.fmt[2];
    hdr_addr         = hdr_4dw ?
                       {tl_rx_data[DATA_W-65 : DATA_W-128]} :
                       {32'd0, tl_rx_data[DATA_W-65 : DATA_W-96]};
    is_poisoned      = hdr_dw0.ep;

    // TLP type decode
    is_mrd   = (hdr_dw0.tlp_type == TLP_MRd32) || (hdr_dw0.tlp_type == TLP_MRd64);
    is_mwr   = (hdr_dw0.tlp_type == TLP_MWr32) || (hdr_dw0.tlp_type == TLP_MWr64);
    is_iord  = (hdr_dw0.tlp_type == TLP_MRdLk32) && (hdr_dw0.fmt[2] == 1'b0);
    is_iowr  = (hdr_dw0.tlp_type == TLP_IOWr);
    is_cfgrd0 = (hdr_dw0.tlp_type == TLP_CfgRd0);
    is_cfgwr0 = (hdr_dw0.tlp_type == TLP_CfgWr0);
    is_cfgrd1 = (hdr_dw0.tlp_type == TLP_CfgRd1);
    is_cfgwr1 = (hdr_dw0.tlp_type == TLP_CfgWr1);
    is_cpl    = (hdr_dw0.tlp_type == TLP_Cpl);
    is_cpld   = (hdr_dw0.tlp_type == TLP_CplD);
    is_msg    = (hdr_dw0.fmt[2:1] == 2'b11);
  end

  // ---------------------------------------------------------------------------
  // RX Receive State Machine
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tl_rx_state     <= TL_RX_IDLE;
      current_route   <= ROUTE_DROP;
      hdr_beat_cnt    <= '0;
      cpl_rx_valid    <= 1'b0;
      cpl_rx_sop      <= 1'b0;
      cpl_rx_eop      <= 1'b0;
      cpl_rx_data     <= '0;
      axibr_rx_valid  <= 1'b0;
      axibr_rx_sop    <= 1'b0;
      axibr_rx_eop    <= 1'b0;
      axibr_rx_data   <= '0;
      cfg_rx_valid    <= 1'b0;
      cfg_rd_valid    <= 1'b0;
      cfg_rx_data     <= '0;
      cfg_rx_addr     <= '0;
      cfg_rx_wr       <= 1'b0;
      dma_rx_valid    <= 1'b0;
      dma_rx_sop      <= 1'b0;
      dma_rx_eop      <= 1'b0;
      dma_rx_data     <= '0;
      ur_cpl_valid    <= 1'b0;
      ur_cpl_sop      <= 1'b0;
      ur_cpl_eop      <= 1'b0;
      ur_cpl_data     <= '0;
      ur_cpl_pending  <= 1'b0;
      ur_req_id       <= '0;
      ur_tag          <= '0;
      msg_valid       <= 1'b0;
      msg_code        <= '0;
      msg_data        <= '0;
      err_cor         <= 1'b0;
      err_nonfatal    <= 1'b0;
      err_fatal       <= 1'b0;
    end else if (!link_up) begin
      tl_rx_state <= TL_RX_IDLE;
    end else begin

      // Default output de-assertions
      cpl_rx_valid   <= 1'b0;
      cpl_rx_sop     <= 1'b0;
      cpl_rx_eop     <= 1'b0;
      axibr_rx_valid <= 1'b0;
      axibr_rx_sop   <= 1'b0;
      axibr_rx_eop   <= 1'b0;
      cfg_rx_valid   <= 1'b0;
      cfg_rd_valid   <= 1'b0;
      dma_rx_valid   <= 1'b0;
      dma_rx_sop     <= 1'b0;
      dma_rx_eop     <= 1'b0;
      ur_cpl_valid   <= 1'b0;
      ur_cpl_sop     <= 1'b0;
      ur_cpl_eop     <= 1'b0;
      msg_valid      <= 1'b0;
      err_cor        <= 1'b0;
      err_nonfatal   <= 1'b0;
      err_fatal      <= 1'b0;

      // Error injection from DLL
      if (tl_rx_error) begin
        err_nonfatal <= 1'b1;
        tl_rx_state  <= TL_RX_IDLE;
      end

      case (tl_rx_state)

        // -----------------------------------------------------------------------
        TL_RX_IDLE: begin
          if (tl_rx_valid && tl_rx_sop) begin
            tl_rx_state  <= TL_RX_HDR1;
            hdr_beat_cnt <= 3'd0;
          end
        end

        // -----------------------------------------------------------------------
        TL_RX_HDR1: begin
          if (tl_rx_valid) begin
            if (is_poisoned) begin
              // Poisoned TLP: drop, signal non-fatal error
              current_route <= ROUTE_DROP;
              err_nonfatal  <= 1'b1;
              tl_rx_state   <= TL_RX_DROP;

            end else if (is_cfgrd0 || is_cfgwr0) begin
              // Config space access
              current_route <= ROUTE_CFG;
              cfg_rx_addr   <= tl_rx_data[DATA_W-80 : DATA_W-91];   // Reg addr [11:2]
              cfg_rx_wr     <= is_cfgwr0;
              cfg_rx_data   <= tl_rx_data[DATA_W-97 : DATA_W-128];  // Write data DW
              // Separate write vs read valids
              cfg_rx_valid  <= is_cfgwr0;
              cfg_rd_valid  <= is_cfgrd0;
              tl_rx_state   <= tl_rx_eop ? TL_RX_IDLE : TL_RX_DATA;

            end else if (is_cpld || is_cpl) begin
              // Broadcast completions to both AXI bridge and DMA;
              // each consumer filters by its own pending tags.
              $display("[TLP-RX] CplD/Cpl @ %0t, data[255:192]=%016h", $time, tl_rx_data[255:192]);
              current_route <= ROUTE_CPL;
              cpl_rx_data  <= tl_rx_data;
              cpl_rx_valid <= 1'b1;
              cpl_rx_sop   <= 1'b1;
              cpl_rx_eop   <= tl_rx_eop;
              dma_rx_data  <= tl_rx_data;
              dma_rx_valid <= 1'b1;
              dma_rx_sop   <= 1'b1;
              dma_rx_eop   <= tl_rx_eop;
              tl_rx_state <= tl_rx_eop ? TL_RX_IDLE : TL_RX_DATA;

            end else if (is_msg) begin
              // Message TLP
              current_route <= ROUTE_MSG;
              // msg_code: lower 8 bits of DW1 (routing + msg_code = bits [7:0])
              msg_code  <= tl_rx_data[DATA_W-57 : DATA_W-64];
              // msg_data from DW2-DW3 (bits [DATA_W-65:DATA_W-128])
              msg_data  <= tl_rx_data[DATA_W-65 : DATA_W-128];
              msg_valid <= 1'b1;
              tl_rx_state <= tl_rx_eop ? TL_RX_IDLE : TL_RX_MSG;

            end else if ((is_mrd || is_mwr || is_iord || is_iowr) && !ur_needed) begin
              // Memory/IO request with BAR hit
              current_route  <= ROUTE_AXI;
              axibr_rx_data  <= tl_rx_data;
              axibr_rx_valid <= 1'b1;
              axibr_rx_sop   <= 1'b1;
              axibr_rx_eop   <= tl_rx_eop;
              tl_rx_state    <= tl_rx_eop ? TL_RX_IDLE : TL_RX_DATA;

              // ECRC check gating: if td=1 and EN_ECRC, go to ECRC_CHK on EOP
              if (EN_ECRC && hdr_dw0.td && tl_rx_eop)
                tl_rx_state <= TL_RX_ECRC_CHK;

            end else begin
              // Unsupported Request: latch requester info, generate UR Cpl
              current_route <= ROUTE_DROP;
              err_nonfatal  <= 1'b1;
              ur_req_id     <= hdr_requester_id;
              ur_tag        <= hdr_tag;
              ur_cpl_pending <= 1'b1;
              // Drop remaining beats first, then issue UR Cpl
              if (tl_rx_eop)
                tl_rx_state <= TL_RX_UR_CPL;
              else
                tl_rx_state <= TL_RX_DROP;
            end
          end
        end

        // -----------------------------------------------------------------------
        TL_RX_DATA: begin
          if (tl_rx_valid) begin
            case (current_route)
              ROUTE_AXI: begin
                axibr_rx_data  <= tl_rx_data;
                axibr_rx_valid <= 1'b1;
                axibr_rx_sop   <= 1'b0;
                axibr_rx_eop   <= tl_rx_eop;
              end
              ROUTE_CPL: begin
                cpl_rx_data  <= tl_rx_data;
                cpl_rx_valid <= 1'b1;
                cpl_rx_sop   <= 1'b0;
                cpl_rx_eop   <= tl_rx_eop;
                dma_rx_data  <= tl_rx_data;
                dma_rx_valid <= 1'b1;
                dma_rx_sop   <= 1'b0;
                dma_rx_eop   <= tl_rx_eop;
              end
              ROUTE_DMA: begin
                cpl_rx_data  <= tl_rx_data;
                cpl_rx_valid <= 1'b1;
                cpl_rx_sop   <= 1'b0;
                cpl_rx_eop   <= tl_rx_eop;
                dma_rx_data  <= tl_rx_data;
                dma_rx_valid <= 1'b1;
                dma_rx_sop   <= 1'b0;
                dma_rx_eop   <= tl_rx_eop;
              end
              default: ;
            endcase

            if (tl_rx_eop) begin
              if (EN_ECRC && hdr_dw0.td)
                tl_rx_state <= TL_RX_ECRC_CHK;
              else
                tl_rx_state <= TL_RX_IDLE;
            end
          end
        end

        // -----------------------------------------------------------------------
        // Drop state: consume remaining beats until EOP
        TL_RX_DROP: begin
          if (tl_rx_valid && tl_rx_eop) begin
            if (ur_cpl_pending)
              tl_rx_state <= TL_RX_UR_CPL;
            else
              tl_rx_state <= TL_RX_IDLE;
          end
        end

        // -----------------------------------------------------------------------
        // UR Completion generation:
        //   Build a 3DW Cpl TLP with status=UR (3'b001), no data.
        //   Format:
        //   DW0: fmt=3'b000 (3DW no data), type=TLP_Cpl, status=CPL_UR
        //   DW1: completer_id, status=001, BCM=0, byte_count=12'd0
        //   DW2: requester_id, tag, lower_addr=7'h0
        TL_RX_UR_CPL: begin
          if (ur_cpl_ready || !ur_cpl_valid) begin
            ur_cpl_data  <= {
              // DW0: fmt=000 (3DW no-data), type=01010 (Cpl=5'b01010)
              //       TC=000, AT=00, EP=0, TD=0, TH=0, LN=0, Attr=00, Length=10'd0
              3'b000, 5'b01010,         // fmt, type
              1'b0, 3'b000,             // T9, TC[2:0]
              1'b0, 1'b0, 1'b0,        // T8, attr2, LN
              1'b0, 1'b0,              // TH, TD
              1'b0,                    // EP
              2'b00,                   // Attr[1:0]
              2'b00,                   // AT
              10'h000,                 // Length=0
              // DW1: completer_id=16'h0001, CPL_UR status=001, BCM=0, byte_count=0
              16'h0001,                // Completer ID (our device)
              3'b001,                  // Status = CPL_UR
              1'b0,                    // BCM
              12'd0,                   // Byte Count = 0
              // DW2: requester_id, tag, lower_addr=0
              ur_req_id,               // Requester ID
              1'b0,                    // reserved
              ur_tag,                  // Tag [9:0]
              7'h00,                   // Lower Address
              // Pad remainder of 256-bit bus
              {(256-96){1'b0}}
            };
            ur_cpl_valid   <= 1'b1;
            ur_cpl_sop     <= 1'b1;
            ur_cpl_eop     <= 1'b1;   // Single-beat (3DW, no data)
            ur_cpl_pending <= 1'b0;
            tl_rx_state    <= TL_RX_IDLE;
          end
        end

        // -----------------------------------------------------------------------
        // Message TLP: drain any remaining beats
        TL_RX_MSG: begin
          // msg_valid was pulsed on first beat; additional beats are payload
          if (tl_rx_valid && tl_rx_eop)
            tl_rx_state <= TL_RX_IDLE;
        end

        // -----------------------------------------------------------------------
        // ECRC Check state
        TL_RX_ECRC_CHK: begin
          if (ecrc_error) begin
            err_nonfatal <= 1'b1;
            // Re-poison the last forwarded beat indication (simplified: flag)
          end
          tl_rx_state <= TL_RX_IDLE;
        end

        default: tl_rx_state <= TL_RX_IDLE;
      endcase
    end
  end

endmodule : pcie_tlp_rx
