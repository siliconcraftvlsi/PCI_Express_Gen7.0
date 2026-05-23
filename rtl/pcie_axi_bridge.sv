`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// Author  : Robert Kingsly Amalathas
// Email   : robertk@microprocessorlab.com
// LinkedIn: www.linkedin.com/in/robertkingslya
// -----------------------------------------------------------------------------
// =============================================================================
// PCIe 7.0 Controller - AXI Bridge
// Based on AMBA AXI4 Protocol Specification and PCIe 7.0 Base Spec
// =============================================================================
// Description:
//   Bidirectional bridge between the AXI4 application interface and the
//   PCIe Transaction Layer:
//
//   AXI Subordinate (Inbound AXI → PCIe):
//     - Accepts AXI4 write transactions and generates Memory Write TLPs
//     - Enforces Max Payload Size (MPS); splits oversized writes into multiple MWr
//     - Accepts AXI4 read transactions and generates Memory Read Request TLPs
//     - Enforces Max Read Request Size (MRRS); splits long reads into multiple MRd
//     - Per-tag state machine (TAG_FREE→PENDING→CPL_PARTIAL→CPL_DONE)
//     - Write-channel interleave fix: AW info buffered independently from W data
//
//   AXI Manager (Outbound PCIe → AXI):
//     - Receives Memory Write TLPs and drives AXI write transactions
//     - Receives Memory Read Completions with byte_count tracking
//     - Sets s_axi_rlast only when remaining byte_count reaches zero
//
//   BAR Table:
//     - Six 64-bit BARs driven by cfg_bar_wr_en writes
//     - Exported to pcie_tlp_rx for BAR address matching
//
//   Supports:
//     - INCR burst type, lengths 1–256 beats
//     - 64-bit PCIe addresses mapped to 64-bit AXI addresses
//     - AXI ID passthrough for completion matching
//     - PCIe→AXI and AXI→PCIe ordering compliance
// =============================================================================

`include "pcie_pkg.sv"

module pcie_axi_bridge
  import pcie_pkg::*;
#(
  parameter int unsigned DATA_W   = 256,
  parameter int unsigned ADDR_W   = 64,
  parameter int unsigned AXI_ID_W = 8
)(
  input  logic              clk,
  input  logic              rst_n,
  input  logic              link_up,
  input  logic [2:0]        cfg_mps,
  input  logic [2:0]        cfg_mrrs,
  input  logic              np_relaxed_order, // Attr[1] on outbound MRd/MWr headers

  // -------------------------------------------------------------------------
  // BAR configuration inputs (from config space write path)
  // -------------------------------------------------------------------------
  input  logic [31:0]       cfg_bar_wr_data,
  input  logic [2:0]        cfg_bar_idx,
  input  logic              cfg_bar_wr_en,

  // -------------------------------------------------------------------------
  // BAR table outputs (to pcie_tlp_rx for address matching)
  // -------------------------------------------------------------------------
  output logic [63:0]       bar_addr [6],
  output logic [63:0]       bar_mask [6],
  output logic              bar_en   [6],

  // -------------------------------------------------------------------------
  // AXI4 Subordinate Interface (device requests coming from SoC fabric)
  // -------------------------------------------------------------------------
  // Write Address Channel
  input  logic [AXI_ID_W-1:0]   s_axi_awid,
  input  logic [ADDR_W-1:0]     s_axi_awaddr,
  input  logic [7:0]            s_axi_awlen,
  input  logic [2:0]            s_axi_awsize,
  input  logic [1:0]            s_axi_awburst,
  input  logic                  s_axi_awvalid,
  output logic                  s_axi_awready,
  // Write Data Channel
  input  logic [DATA_W-1:0]     s_axi_wdata,
  input  logic [DATA_W/8-1:0]   s_axi_wstrb,
  input  logic                  s_axi_wlast,
  input  logic                  s_axi_wvalid,
  output logic                  s_axi_wready,
  // Write Response Channel
  output logic [AXI_ID_W-1:0]   s_axi_bid,
  output logic [1:0]            s_axi_bresp,
  output logic                  s_axi_bvalid,
  input  logic                  s_axi_bready,
  // Read Address Channel
  input  logic [AXI_ID_W-1:0]   s_axi_arid,
  input  logic [ADDR_W-1:0]     s_axi_araddr,
  input  logic [7:0]            s_axi_arlen,
  input  logic [2:0]            s_axi_arsize,
  input  logic [1:0]            s_axi_arburst,
  input  logic                  s_axi_arvalid,
  output logic                  s_axi_arready,
  // Read Data Channel
  output logic [AXI_ID_W-1:0]   s_axi_rid,
  output logic [DATA_W-1:0]     s_axi_rdata,
  output logic [1:0]            s_axi_rresp,
  output logic                  s_axi_rlast,
  output logic                  s_axi_rvalid,
  input  logic                  s_axi_rready,

  // -------------------------------------------------------------------------
  // MRd tag / completion observability (SVA bind and debug)
  // -------------------------------------------------------------------------
  output logic [9:0]            sva_pending_tag,
  output logic                  sva_tag_valid,
  output logic                  sva_cpl_received,
  output logic [9:0]            sva_cpl_tag,
  output logic [15:0]           np_outstanding,

  // -------------------------------------------------------------------------
  // AXI4 Manager Interface (PCIe completions/writes going to SoC memory)
  // -------------------------------------------------------------------------
  output logic [AXI_ID_W-1:0]   m_axi_awid,
  output logic [ADDR_W-1:0]     m_axi_awaddr,
  output logic [7:0]            m_axi_awlen,
  output logic [2:0]            m_axi_awsize,
  output logic [1:0]            m_axi_awburst,
  output logic                  m_axi_awvalid,
  input  logic                  m_axi_awready,
  output logic [DATA_W-1:0]     m_axi_wdata,
  output logic [DATA_W/8-1:0]   m_axi_wstrb,
  output logic                  m_axi_wlast,
  output logic                  m_axi_wvalid,
  input  logic                  m_axi_wready,
  input  logic [AXI_ID_W-1:0]   m_axi_bid,
  input  logic [1:0]            m_axi_bresp,
  input  logic                  m_axi_bvalid,
  output logic                  m_axi_bready,
  output logic [AXI_ID_W-1:0]   m_axi_arid,
  output logic [ADDR_W-1:0]     m_axi_araddr,
  output logic [7:0]            m_axi_arlen,
  output logic [2:0]            m_axi_arsize,
  output logic [1:0]            m_axi_arburst,
  output logic                  m_axi_arvalid,
  input  logic                  m_axi_arready,
  input  logic [AXI_ID_W-1:0]   m_axi_rid,
  input  logic [DATA_W-1:0]     m_axi_rdata,
  input  logic [1:0]            m_axi_rresp,
  input  logic                  m_axi_rlast,
  input  logic                  m_axi_rvalid,
  output logic                  m_axi_rready,

  // -------------------------------------------------------------------------
  // TLP TX interface: posted writes -> TL TX arbiter
  // -------------------------------------------------------------------------
  output logic [DATA_W-1:0]     tlp_tx_data,
  output logic                  tlp_tx_valid,
  input  logic                  tlp_tx_ready,
  output logic                  tlp_tx_sop,
  output logic                  tlp_tx_eop,

  // -------------------------------------------------------------------------
  // TLP TX interface: non-posted reads -> TL TX arbiter
  // -------------------------------------------------------------------------
  output logic [DATA_W-1:0]     tlp_np_data,
  output logic                  tlp_np_valid,
  input  logic                  tlp_np_ready,
  output logic                  tlp_np_sop,
  output logic                  tlp_np_eop,

  // -------------------------------------------------------------------------
  // Completion RX (from TL RX)
  // -------------------------------------------------------------------------
  input  logic [DATA_W-1:0]     cpl_rx_data,
  input  logic                  cpl_rx_valid,
  input  logic                  cpl_rx_sop,
  input  logic                  cpl_rx_eop,
  output logic                  cpl_rx_ready
);

  // ---------------------------------------------------------------------------
  // MPS / MRRS decode helpers
  // ---------------------------------------------------------------------------
  // Returns max payload bytes for a given cfg_mps/cfg_mrrs code
  function automatic logic [12:0] mps_bytes(input logic [2:0] code);
    case (code)
      3'd0: mps_bytes = 13'd128;
      3'd1: mps_bytes = 13'd256;
      3'd2: mps_bytes = 13'd512;
      3'd3: mps_bytes = 13'd1024;
      3'd4: mps_bytes = 13'd2048;
      3'd5: mps_bytes = 13'd4096;
      default: mps_bytes = 13'd128;
    endcase
  endfunction

  function automatic logic [12:0] mrrs_bytes(input logic [2:0] code);
    mrrs_bytes = mps_bytes(code);   // same encoding
  endfunction

  // ---------------------------------------------------------------------------
  // BAR Table (6 BARs, each 64-bit)
  // ---------------------------------------------------------------------------
  // bar_addr / bar_mask / bar_en are ports; drive them from config writes
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < 6; i++) begin
        bar_addr[i] <= '0;
        bar_mask[i] <= '0;
        bar_en[i]   <= 1'b0;
      end
    end else if (cfg_bar_wr_en && (cfg_bar_idx < 3'd6)) begin
      // Simple: lower 32 bits written; bit[0] is enable, bits[31:4] are addr
      bar_addr[cfg_bar_idx] <= {32'd0, cfg_bar_wr_data & 32'hFFFF_FFF0};
      bar_mask[cfg_bar_idx] <= {32'd0, 32'hFFFF_F000};  // 4KB granularity default
      bar_en[cfg_bar_idx]   <= cfg_bar_wr_data[0];
    end
  end

  // ---------------------------------------------------------------------------
  // Tag Pool with per-tag state machine
  // ---------------------------------------------------------------------------
  parameter int TAG_POOL_SZ  = 256;
  parameter int TAG_POOL_W   = $clog2(TAG_POOL_SZ);

  typedef enum logic [1:0] {
    TAG_FREE,
    TAG_PENDING,
    TAG_CPL_PARTIAL,
    TAG_CPL_DONE
  } tag_state_e;

  tag_state_e              tag_sm [TAG_POOL_SZ];
  logic [AXI_ID_W-1:0]    tag_id_table   [TAG_POOL_SZ];
  logic [ADDR_W-1:0]       tag_addr_table [TAG_POOL_SZ];
  logic [7:0]              tag_len_table  [TAG_POOL_SZ];
  logic [11:0]             cpl_byte_count [TAG_POOL_SZ];  // Remaining bytes in completion sequence
  logic [TAG_POOL_W-1:0]   tag_alloc;
  logic                    tag_pool_full;

  // Priority encoder: find first TAG_FREE slot
  always @* begin
    tag_alloc     = '0;
    tag_pool_full = 1'b1;
    for (int i = TAG_POOL_SZ-1; i >= 0; i--) begin
      if (tag_sm[i] == TAG_FREE) begin
        tag_alloc     = TAG_POOL_W'(i);
        tag_pool_full = 1'b0;
      end
    end
  end

  always_comb begin
    np_outstanding = 16'd0;
    for (int i = 0; i < TAG_POOL_SZ; i++) begin
      if (tag_sm[i] == TAG_PENDING || tag_sm[i] == TAG_CPL_PARTIAL)
        np_outstanding = np_outstanding + 16'd1;
    end
  end

  // ---------------------------------------------------------------------------
  // AW channel buffer (decouple AW from W to allow simultaneous arrival)
  // ---------------------------------------------------------------------------
  logic [ADDR_W-1:0]    aw_buf_addr;
  logic [7:0]           aw_buf_len;
  logic [AXI_ID_W-1:0]  aw_buf_id;
  logic [2:0]           aw_buf_size;
  logic                 aw_buf_valid;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      aw_buf_valid <= 1'b0;
      aw_buf_addr  <= '0;
      aw_buf_len   <= '0;
      aw_buf_id    <= '0;
      aw_buf_size  <= '0;
    end else begin
      if (s_axi_awvalid && s_axi_awready && !aw_buf_valid) begin
        aw_buf_addr  <= s_axi_awaddr;
        aw_buf_len   <= s_axi_awlen;
        aw_buf_id    <= s_axi_awid;
        aw_buf_size  <= s_axi_awsize;
        aw_buf_valid <= 1'b1;
      end else if (aw_buf_valid && s_axi_wvalid && s_axi_wlast) begin
        aw_buf_valid <= 1'b0;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // AXI Subordinate Write Path: AXI Write → PCIe MWr TLP (with MPS splitting)
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    AWR_IDLE,
    AWR_ADDR,
    AWR_DATA,
    AWR_SPLIT_HDR,   // Next MWr header after MPS split
    AWR_RESP
  } awr_state_e;

  awr_state_e   awr_state;
  logic [ADDR_W-1:0]   awr_addr;
  logic [7:0]          awr_len;
  logic [AXI_ID_W-1:0] awr_id;
  logic [2:0]          awr_size;
  logic [7:0]          awr_beat_cnt;
  logic                awr_is_64;
  logic [12:0]         awr_mps_limit;   // Max DW per MWr segment
  logic [9:0]          awr_seg_dw_cnt;  // DWs emitted in current MWr

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      awr_state      <= AWR_IDLE;
      s_axi_awready  <= 1'b0;
      s_axi_wready   <= 1'b0;
      s_axi_bvalid   <= 1'b0;
      s_axi_bid      <= '0;
      s_axi_bresp    <= 2'b00;
      tlp_tx_valid   <= 1'b0;
      tlp_tx_sop     <= 1'b0;
      tlp_tx_eop     <= 1'b0;
      tlp_tx_data    <= '0;
      awr_beat_cnt   <= '0;
      awr_seg_dw_cnt <= '0;
      awr_mps_limit  <= 13'd128;
    end else if (!link_up) begin
      awr_state     <= AWR_IDLE;
      s_axi_awready <= 1'b0;
      s_axi_wready  <= 1'b0;
      s_axi_bvalid  <= 1'b0;
      tlp_tx_valid  <= 1'b0;
    end else begin
      s_axi_awready <= 1'b0;
      s_axi_wready  <= 1'b0;
      tlp_tx_valid  <= 1'b0;
      tlp_tx_sop    <= 1'b0;
      tlp_tx_eop    <= 1'b0;

      case (awr_state)

        AWR_IDLE: begin
          s_axi_awready <= !aw_buf_valid;  // Accept AW when buffer free
          if (s_axi_awvalid && !aw_buf_valid) begin
            awr_addr       <= s_axi_awaddr;
            awr_len        <= s_axi_awlen;
            awr_id         <= s_axi_awid;
            awr_size       <= s_axi_awsize;
            awr_is_64      <= (s_axi_awaddr[63:32] != '0);
            awr_beat_cnt   <= '0;
            awr_mps_limit  <= mps_bytes(cfg_mps);
            awr_seg_dw_cnt <= '0;
            awr_state      <= AWR_ADDR;
          end else if (aw_buf_valid) begin
            // Drain buffered AW
            awr_addr       <= aw_buf_addr;
            awr_len        <= aw_buf_len;
            awr_id         <= aw_buf_id;
            awr_size       <= aw_buf_size;
            awr_is_64      <= (aw_buf_addr[63:32] != '0);
            awr_beat_cnt   <= '0;
            awr_mps_limit  <= mps_bytes(cfg_mps);
            awr_seg_dw_cnt <= '0;
            awr_state      <= AWR_ADDR;
          end
        end

        AWR_ADDR: begin
          // Hold MWr TLP header valid until the TL arbiter accepts it.
          if (awr_is_64) begin
            tlp_tx_data  <= {
              3'b011, 5'b00001,
              1'b0, 3'b000, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0,
              {np_relaxed_order, 1'b0}, 2'b00,
              10'(((awr_len+1) * (DATA_W/32))),
              16'h0001, 10'h000, 4'hF, 4'hF,
              awr_addr[63:2], 2'b00,
              {(DATA_W-128){1'b0}}
            };
          end else begin
            tlp_tx_data  <= {
              3'b010, 5'b00000,
              1'b0, 3'b000, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0,
              {np_relaxed_order, 1'b0}, 2'b00,
              10'(((awr_len+1) * (DATA_W/32))),
              16'h0001, 10'h000, 4'hF, 4'hF,
              awr_addr[31:2], 2'b00,
              {(DATA_W-96){1'b0}}
            };
          end
          tlp_tx_valid <= 1'b1;
          tlp_tx_sop   <= 1'b1;
          tlp_tx_eop   <= 1'b0;

          if (tlp_tx_ready) begin
            s_axi_wready   <= 1'b1;
            awr_seg_dw_cnt <= 10'd0;
            awr_state      <= AWR_DATA;
          end
        end

        AWR_DATA: begin
          s_axi_wready <= tlp_tx_ready;
          if (s_axi_wvalid) begin
            tlp_tx_data  <= s_axi_wdata;
            tlp_tx_valid <= 1'b1;
            tlp_tx_sop   <= 1'b0;
            tlp_tx_eop   <= s_axi_wlast ||
                            ({3'b0, awr_seg_dw_cnt} >= awr_mps_limit - 13'(DATA_W/32));

            if (tlp_tx_ready) begin
              awr_seg_dw_cnt <= awr_seg_dw_cnt + 10'(DATA_W/32);
              if (s_axi_wlast) begin
                awr_state    <= AWR_RESP;
                s_axi_wready <= 1'b0;
              end else if ({3'b0, awr_seg_dw_cnt} >= awr_mps_limit - 13'(DATA_W/32)) begin
                awr_addr       <= awr_addr + ADDR_W'(awr_mps_limit);
                s_axi_wready   <= 1'b0;
                awr_state      <= AWR_SPLIT_HDR;
              end else begin
                awr_beat_cnt <= awr_beat_cnt + 8'd1;
              end
            end
          end else begin
            tlp_tx_valid <= 1'b0;
          end
        end

        AWR_SPLIT_HDR: begin
          // Hold the next split MWr header until accepted.
          awr_is_64 <= (awr_addr[63:32] != '0);
          if (awr_addr[63:32] != '0) begin
            tlp_tx_data <= {
              3'b011, 5'b00001,
              1'b0, 3'b000, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0,
              {np_relaxed_order, 1'b0}, 2'b00,
              10'h001,
              16'h0001, 10'h000, 4'hF, 4'hF,
              awr_addr[63:2], 2'b00,
              {(DATA_W-128){1'b0}}
            };
          end else begin
            tlp_tx_data <= {
              3'b010, 5'b00000,
              1'b0, 3'b000, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0,
              {np_relaxed_order, 1'b0}, 2'b00,
              10'h001,
              16'h0001, 10'h000, 4'hF, 4'hF,
              awr_addr[31:2], 2'b00,
              {(DATA_W-96){1'b0}}
            };
          end
          tlp_tx_valid <= 1'b1;
          tlp_tx_sop   <= 1'b1;
          tlp_tx_eop   <= 1'b0;

          if (tlp_tx_ready) begin
            s_axi_wready   <= 1'b1;
            awr_seg_dw_cnt <= 10'd0;
            awr_state      <= AWR_DATA;
          end
        end

        AWR_RESP: begin
          if (!s_axi_bvalid) begin
            s_axi_bvalid <= 1'b1;
            s_axi_bid    <= awr_id;
            s_axi_bresp  <= 2'b00;  // OKAY
          end else if (s_axi_bready) begin
            s_axi_bvalid <= 1'b0;
            awr_state    <= AWR_IDLE;
          end
        end

        default: awr_state <= AWR_IDLE;
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // AXI Subordinate Read Path: AXI Read → PCIe MRd TLP (with MRRS splitting)
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    ARD_IDLE,
    ARD_SPLIT,    // Emit one MRd per MRRS chunk
    ARD_WAIT_CPL  // Wait for all completions for the split
  } ard_state_e;

  ard_state_e             ard_state;
  logic [ADDR_W-1:0]      ard_addr;         // Current split base address
  logic [ADDR_W-1:0]      ard_split_base;   // Original request base
  logic [7:0]             ard_len;          // AXI arlen
  logic [AXI_ID_W-1:0]   ard_id;
  logic [TAG_POOL_W-1:0]  ard_tag;
  logic                   ard_is_64;
  logic [12:0]            ard_remaining;    // Remaining bytes to issue
  logic [7:0]             ard_split_cnt;    // Number of MRd TLPs issued
  logic [15:0]            ard_cpl_wait;     // Completion wait timeout

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ard_state       <= ARD_IDLE;
      s_axi_arready   <= 1'b0;
      ard_remaining   <= '0;
      ard_split_cnt   <= '0;
      ard_cpl_wait    <= '0;
      tlp_np_valid   <= 1'b0;
      tlp_np_sop     <= 1'b0;
      tlp_np_eop     <= 1'b0;
      tlp_np_data    <= '0;
      for (int i = 0; i < TAG_POOL_SZ; i++) begin
        tag_sm[i]         <= TAG_FREE;
        cpl_byte_count[i] <= '0;
      end
    end else if (!link_up) begin
      ard_state     <= ARD_IDLE;
      s_axi_arready <= 1'b0;
      tlp_np_valid <= 1'b0;
      tlp_np_sop   <= 1'b0;
      tlp_np_eop   <= 1'b0;
      for (int i = 0; i < TAG_POOL_SZ; i++)
        tag_sm[i] <= TAG_FREE;
    end else begin
      s_axi_arready <= 1'b0;
      // s_axi_rvalid is driven solely by the completion reassembly block below
      tlp_np_valid  <= 1'b0;
      tlp_np_sop    <= 1'b0;
      tlp_np_eop    <= 1'b0;

      case (ard_state)

        ARD_IDLE: begin
          s_axi_arready <= !tag_pool_full;
          if (s_axi_arvalid && !tag_pool_full) begin
            ard_split_base  <= s_axi_araddr;
            ard_addr        <= s_axi_araddr;
            ard_len         <= s_axi_arlen;
            ard_id          <= s_axi_arid;
            ard_is_64       <= (s_axi_araddr[63:32] != '0);
            // Total bytes = (arlen+1) * (DATA_W/8)
            ard_remaining   <= 13'((s_axi_arlen + 8'd1) * (DATA_W/8));
            ard_split_cnt   <= 8'd0;
            s_axi_arready   <= 1'b0;
            ard_state       <= ARD_SPLIT;
          end
        end

        ARD_SPLIT: begin
          // Emit one MRd TLP per MRRS chunk (hold valid until accepted)
          if (!tag_pool_full) begin
            // We use tag_alloc combinatorially since we are head of line
            if (ard_is_64) begin
              tlp_np_data <= {
                3'b001, 5'b00000,
                1'b0, 3'b000, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0,
              {np_relaxed_order, 1'b0}, 2'b00,
                10'(mrrs_bytes(cfg_mrrs) >> 2),
                16'h0001, 10'(tag_alloc), 4'h0, 4'hF,
                ard_addr[63:2], 2'b00,
                {(DATA_W-128){1'b0}}
              };
            end else begin
              tlp_np_data <= {
                3'b000, 5'b00000,
                1'b0, 3'b000, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0,
              {np_relaxed_order, 1'b0}, 2'b00,
                10'(mrrs_bytes(cfg_mrrs) >> 2),
                16'h0001, 10'(tag_alloc), 4'h0, 4'hF,
                ard_addr[31:2], 2'b00,
                {(DATA_W-96){1'b0}}
              };
            end
            tlp_np_valid <= 1'b1;
            tlp_np_sop   <= 1'b1;
            tlp_np_eop   <= 1'b1;   // MRd has no data
            if (tlp_np_ready) begin
              ard_tag                     <= tag_alloc;
              tag_sm[tag_alloc]           <= TAG_PENDING;
              tag_id_table[tag_alloc]     <= ard_id;
              tag_addr_table[tag_alloc]   <= ard_addr;
              tag_len_table[tag_alloc]    <= ard_len;
              cpl_byte_count[tag_alloc]   <= 12'(ard_remaining > mrrs_bytes(cfg_mrrs)
                                                 ? mrrs_bytes(cfg_mrrs)
                                                 : ard_remaining);
              // Advance address and remaining count
              if (ard_remaining > mrrs_bytes(cfg_mrrs)) begin
                ard_remaining <= ard_remaining - mrrs_bytes(cfg_mrrs);
                ard_addr      <= ard_addr + ADDR_W'(mrrs_bytes(cfg_mrrs));
                ard_split_cnt <= ard_split_cnt + 8'd1;
                // Stay in ARD_SPLIT to emit next chunk
              end else begin
                ard_remaining <= '0;
                ard_cpl_wait  <= '0;
                ard_state     <= ARD_WAIT_CPL;
              end
            end
          end
        end

        ARD_WAIT_CPL: begin
          if (tag_sm[ard_tag] == TAG_CPL_DONE) begin
            ard_state    <= ARD_IDLE;
            ard_cpl_wait <= '0;
          end else if (ard_cpl_wait >= 16'd4096) begin
            // Recover from unmatched/spurious completions
            ard_state    <= ARD_IDLE;
            ard_cpl_wait <= '0;
            for (int i = 0; i < TAG_POOL_SZ; i++)
              tag_sm[i] <= TAG_FREE;
          end else
            ard_cpl_wait <= ard_cpl_wait + 16'd1;
        end

        default: ard_state <= ARD_IDLE;
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // Completion Reassembly: match cpl_tag, track byte_count, set rlast
  // ---------------------------------------------------------------------------
  logic [9:0]             cpl_tag;
  logic [TAG_POOL_W-1:0]  cpl_tag_idx;
  logic [11:0]            cpl_hdr_byte_count;

  // CplD tag in DW2 bits[15:6] → [175:166]; byte_count in DW1[11:0] → [203:192]
  assign cpl_tag            = cpl_rx_data[175:166];
  assign cpl_tag_idx        = cpl_tag[TAG_POOL_W-1:0];
  assign cpl_hdr_byte_count = cpl_rx_data[203:192];  // CplD DW1 byte_count[11:0]

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s_axi_rvalid <= 1'b0;
      s_axi_rlast  <= 1'b0;
      s_axi_rid    <= '0;
      s_axi_rdata  <= '0;
      s_axi_rresp  <= 2'b00;
    end else begin
      // Hold R beat until the master accepts it
      if (s_axi_rvalid && !s_axi_rready) begin
        // keep rid/rdata/rlast/rvalid
      end else begin
        s_axi_rvalid <= 1'b0;
        s_axi_rlast  <= 1'b0;

        if (s_axi_rvalid && s_axi_rready) begin
          for (int i = 0; i < TAG_POOL_SZ; i++) begin
            if (tag_sm[i] == TAG_CPL_DONE)
              tag_sm[i] <= TAG_FREE;
          end
        end

        if (cpl_rx_valid && cpl_rx_sop) begin
          $display("[AXI-BR] cpl_rx sop @ %0t tag=%0d idx=%0d sm=%0d",
                   $time, cpl_tag, cpl_tag_idx, tag_sm[cpl_tag_idx]);
          if (tag_sm[cpl_tag_idx] == TAG_PENDING ||
              tag_sm[cpl_tag_idx] == TAG_CPL_PARTIAL) begin
            if (cpl_hdr_byte_count <= 12'(DATA_W/8)) begin
              cpl_byte_count[cpl_tag_idx] <= 12'd0;
              tag_sm[cpl_tag_idx]         <= TAG_CPL_DONE;
              s_axi_rlast                 <= 1'b1;
            end else begin
              cpl_byte_count[cpl_tag_idx] <= cpl_hdr_byte_count - 12'(DATA_W/8);
              tag_sm[cpl_tag_idx]         <= TAG_CPL_PARTIAL;
              s_axi_rlast                 <= 1'b0;
            end
            s_axi_rid    <= tag_id_table[cpl_tag_idx];
            s_axi_rdata  <= cpl_rx_data[DATA_W/2-1:0];  // payload DW4+
            s_axi_rresp  <= 2'b00;
            s_axi_rvalid <= 1'b1;
          end
        end
      end
    end
  end

  assign cpl_rx_ready = 1'b1;

  // One-cycle pulses for concurrent SVA (MRd issued / CplD matched)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sva_tag_valid     <= 1'b0;
      sva_cpl_received  <= 1'b0;
      sva_pending_tag   <= '0;
      sva_cpl_tag       <= '0;
    end else begin
      sva_tag_valid    <= 1'b0;
      sva_cpl_received <= 1'b0;
      if (ard_state == ARD_SPLIT && tlp_np_valid && tlp_np_ready) begin
        sva_tag_valid   <= 1'b1;
        sva_pending_tag <= {2'b00, tag_alloc};
      end
      if (cpl_rx_valid && cpl_rx_sop &&
          (tag_sm[cpl_tag_idx] == TAG_PENDING || tag_sm[cpl_tag_idx] == TAG_CPL_PARTIAL)) begin
        sva_cpl_received <= 1'b1;
        sva_cpl_tag      <= cpl_tag;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // AXI Manager Write Path: PCIe MWr TLP → AXI Write
  // (Driven by pcie_tlp_rx axibr_rx path; simplified pass-through here)
  // ---------------------------------------------------------------------------
  assign m_axi_awid    = AXI_ID_W'(0);
  assign m_axi_awaddr  = '0;
  assign m_axi_awlen   = 8'd0;
  assign m_axi_awsize  = 3'b101;  // 32 bytes per beat (DATA_W=256)
  assign m_axi_awburst = 2'b01;   // INCR
  assign m_axi_awvalid = 1'b0;    // Driven by received TLP logic (future expansion)
  assign m_axi_wdata   = '0;
  assign m_axi_wstrb   = '0;
  assign m_axi_wlast   = 1'b0;
  assign m_axi_wvalid  = 1'b0;
  assign m_axi_bready  = 1'b1;

  // AXI Manager Read (not needed for typical EP; RPs use this for config)
  assign m_axi_arid    = AXI_ID_W'(0);
  assign m_axi_araddr  = '0;
  assign m_axi_arlen   = 8'd0;
  assign m_axi_arsize  = 3'b101;
  assign m_axi_arburst = 2'b01;
  assign m_axi_arvalid = 1'b0;
  assign m_axi_rready  = 1'b1;

endmodule : pcie_axi_bridge
