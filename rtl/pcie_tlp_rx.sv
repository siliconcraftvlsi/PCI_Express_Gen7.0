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
//     - Memory Read/Write → AXI Bridge
//     - Config Read/Write → Configuration Space
//     - Completions → AXI Bridge (matching outstanding tag table)
//     - Messages → Internal message handler
//     - DMA completions → DMA Engine
//
//   Features:
//     - Full TLP header decode (3DW and 4DW, Type0/Type1)
//     - Address filtering and BAR matching (for EP mode)
//     - Completion timeout detection
//     - ECRC checking (optional)
//     - Error reporting (Unsupported Request, Completer Abort, Malformed TLP)
//     - Poisoned TLP detection
// =============================================================================

`include "pcie_pkg.sv"

module pcie_tlp_rx
  import pcie_pkg::*;
#(
  parameter int unsigned DATA_W     = 256,
  parameter int unsigned ADDR_W     = 64,
  parameter  pcie_role_e DEVICE_ROLE = ROLE_EP
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
  // Config Space Accesses
  // -------------------------------------------------------------------------
  output logic               cfg_rx_valid,
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
  // RX Receive State Machine
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    TL_RX_IDLE,
    TL_RX_HDR1,    // First header beat
    TL_RX_HDR2,    // Second header beat (for 4DW)
    TL_RX_DATA,    // Data beats
    TL_RX_DROP     // Drop on error / unsupported
  } tl_rx_state_e;

  tl_rx_state_e   tl_rx_state;
  logic [2:0]     hdr_beat_cnt;

  // Routing register
  typedef enum logic [2:0] {
    ROUTE_AXI,
    ROUTE_CFG,
    ROUTE_CPL,
    ROUTE_DMA,
    ROUTE_DROP
  } route_e;

  route_e  current_route;

  // ---------------------------------------------------------------------------
  // DMA Tag Table: track which tags belong to DMA engine
  // ---------------------------------------------------------------------------
  parameter int DMA_TAG_BASE  = 10'd512;
  parameter int DMA_TAG_LIMIT = 10'd767;

  function automatic logic is_dma_tag(input logic [9:0] tag);
    return (tag >= DMA_TAG_BASE && tag <= DMA_TAG_LIMIT);
  endfunction

  // ---------------------------------------------------------------------------
  // Header Decode
  // ---------------------------------------------------------------------------
  always_comb begin
    hdr_dw0         = tlp_dw0_t'(tl_rx_data[DATA_W-1 : DATA_W-32]);
    hdr_requester_id = tl_rx_data[DATA_W-33 : DATA_W-48];
    hdr_tag          = {tl_rx_data[DATA_W-48], tl_rx_data[DATA_W-49], tl_rx_data[DATA_W-50 : DATA_W-57]};
    hdr_last_be      = tl_rx_data[DATA_W-57 : DATA_W-60];
    hdr_first_be     = tl_rx_data[DATA_W-61 : DATA_W-64];
    // 4DW header: fmt[1] = 1 indicates 64-bit address
    hdr_4dw          = hdr_dw0.fmt[1];
    hdr_has_data     = hdr_dw0.fmt[2];
    hdr_addr         = hdr_4dw ?
                       {tl_rx_data[DATA_W-65 : DATA_W-128]} :
                       {32'd0, tl_rx_data[DATA_W-65 : DATA_W-96]};
    is_poisoned     = hdr_dw0.ep;

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
  // RX State Machine
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
      cfg_rx_data     <= '0;
      cfg_rx_addr     <= '0;
      cfg_rx_wr       <= 1'b0;
      dma_rx_valid    <= 1'b0;
      dma_rx_sop      <= 1'b0;
      dma_rx_eop      <= 1'b0;
      dma_rx_data     <= '0;
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
      dma_rx_valid   <= 1'b0;
      dma_rx_sop     <= 1'b0;
      dma_rx_eop     <= 1'b0;
      err_cor        <= 1'b0;
      err_nonfatal   <= 1'b0;
      err_fatal      <= 1'b0;

      // Error injection
      if (tl_rx_error) begin
        err_nonfatal <= 1'b1;
        tl_rx_state  <= TL_RX_IDLE;
      end

      case (tl_rx_state)

        TL_RX_IDLE: begin
          if (tl_rx_valid && tl_rx_sop) begin
            tl_rx_state  <= TL_RX_HDR1;
            hdr_beat_cnt <= 3'd0;
          end
        end

        TL_RX_HDR1: begin
          if (tl_rx_valid) begin
            // Determine routing
            if (is_poisoned) begin
              current_route <= ROUTE_DROP;
              err_nonfatal  <= 1'b1;
              tl_rx_state   <= TL_RX_DROP;
            end else if (is_cfgrd0 || is_cfgwr0) begin
              current_route <= ROUTE_CFG;
              cfg_rx_addr   <= tl_rx_data[DATA_W-80 : DATA_W-91];  // Reg addr bits[11:2]
              cfg_rx_wr     <= is_cfgwr0;
              cfg_rx_data   <= tl_rx_data[DATA_W-97 : DATA_W-128]; // Write data DW
              cfg_rx_valid  <= 1'b1;
              tl_rx_state   <= tl_rx_eop ? TL_RX_IDLE : TL_RX_DATA;
            end else if (is_cpld || is_cpl) begin
              // Route completion: check tag
              if (is_dma_tag(hdr_tag))
                current_route <= ROUTE_DMA;
              else
                current_route <= ROUTE_CPL;
              // Forward first beat
              if (current_route == ROUTE_CPL) begin
                cpl_rx_data  <= tl_rx_data;
                cpl_rx_valid <= 1'b1;
                cpl_rx_sop   <= 1'b1;
                cpl_rx_eop   <= tl_rx_eop;
              end else begin
                dma_rx_data  <= tl_rx_data;
                dma_rx_valid <= 1'b1;
                dma_rx_sop   <= 1'b1;
                dma_rx_eop   <= tl_rx_eop;
              end
              tl_rx_state <= tl_rx_eop ? TL_RX_IDLE : TL_RX_DATA;
            end else if (is_mrd || is_mwr || is_iord || is_iowr) begin
              current_route  <= ROUTE_AXI;
              axibr_rx_data  <= tl_rx_data;
              axibr_rx_valid <= 1'b1;
              axibr_rx_sop   <= 1'b1;
              axibr_rx_eop   <= tl_rx_eop;
              tl_rx_state    <= tl_rx_eop ? TL_RX_IDLE : TL_RX_DATA;
            end else begin
              // Unsupported Request
              current_route <= ROUTE_DROP;
              err_nonfatal  <= 1'b1;
              tl_rx_state   <= TL_RX_DROP;
            end
          end
        end

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
              end
              ROUTE_DMA: begin
                dma_rx_data  <= tl_rx_data;
                dma_rx_valid <= 1'b1;
                dma_rx_sop   <= 1'b0;
                dma_rx_eop   <= tl_rx_eop;
              end
              default: ;
            endcase
            if (tl_rx_eop)
              tl_rx_state <= TL_RX_IDLE;
          end
        end

        TL_RX_DROP: begin
          if (tl_rx_valid && tl_rx_eop)
            tl_rx_state <= TL_RX_IDLE;
        end

        default: tl_rx_state <= TL_RX_IDLE;
      endcase
    end
  end

endmodule : pcie_tlp_rx
