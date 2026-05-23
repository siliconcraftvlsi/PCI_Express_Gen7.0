`timescale 1ns/1ps

// =============================================================================
// PCIe 7.0 Controller - Asynchronous FIFO (Gray-code pointer CDC)
// =============================================================================
// Standard dual-clock FIFO with Gray-coded read/write pointers for safe
// multi-bit CDC.  Used for core_clk ↔ pipe_clk and core_clk ↔ aux_clk
// crossings identified in docs/cdc_rdc_checklist.md.
//
// Parameters:
//   DATA_W  — data width
//   DEPTH   — FIFO depth (must be power of 2)
//   STAGES  — synchronizer stages (2 minimum, 3 for very fast clocks)
// =============================================================================

`ifndef PCIE_ASYNC_FIFO_SV
`define PCIE_ASYNC_FIFO_SV

module pcie_async_fifo #(
  parameter int unsigned DATA_W = 32,
  parameter int unsigned DEPTH  = 16,   // must be power-of-2
  parameter int unsigned STAGES = 2
)(
  // Write side (producer clock domain)
  input  logic              wr_clk,
  input  logic              wr_rst_n,
  input  logic              wr_en,
  input  logic [DATA_W-1:0] wr_data,
  output logic              wr_full,
  output logic              wr_almost_full,

  // Read side (consumer clock domain)
  input  logic              rd_clk,
  input  logic              rd_rst_n,
  input  logic              rd_en,
  output logic [DATA_W-1:0] rd_data,
  output logic              rd_empty,
  output logic              rd_almost_full
);

  localparam int unsigned PTR_W = $clog2(DEPTH) + 1;  // extra MSB for full/empty

  // ---------------------------------------------------------------------------
  // Storage array
  // ---------------------------------------------------------------------------
  logic [DATA_W-1:0] mem [0:DEPTH-1];

  // ---------------------------------------------------------------------------
  // Pointers (binary and gray) in their native domains
  // ---------------------------------------------------------------------------
  logic [PTR_W-1:0] wr_ptr_bin, wr_ptr_gray;
  logic [PTR_W-1:0] rd_ptr_bin, rd_ptr_gray;

  // Synchronized gray pointers
  logic [PTR_W-1:0] wr_ptr_gray_sync;  // wr gray in rd domain
  logic [PTR_W-1:0] rd_ptr_gray_sync;  // rd gray in wr domain

  // ---------------------------------------------------------------------------
  // Binary → Gray
  // ---------------------------------------------------------------------------
  function automatic logic [PTR_W-1:0] bin2gray;
    input logic [PTR_W-1:0] bin;
    bin2gray = bin ^ (bin >> 1);
  endfunction

  // Gray → Binary (for full/empty logic)
  function automatic logic [PTR_W-1:0] gray2bin;
    input logic [PTR_W-1:0] gray;
    logic [PTR_W-1:0] b;
    integer i;
    begin
      b[PTR_W-1] = gray[PTR_W-1];
      for (i = PTR_W-2; i >= 0; i--)
        b[i] = b[i+1] ^ gray[i];
      gray2bin = b;
    end
  endfunction

  // ---------------------------------------------------------------------------
  // Write pointer logic (wr_clk domain)
  // ---------------------------------------------------------------------------
  always_ff @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
      wr_ptr_bin  <= '0;
      wr_ptr_gray <= '0;
    end else if (wr_en && !wr_full) begin
      wr_ptr_bin  <= wr_ptr_bin + 1;
      wr_ptr_gray <= bin2gray(wr_ptr_bin + 1);
    end
  end

  // Write to storage
  always_ff @(posedge wr_clk) begin
    if (wr_en && !wr_full)
      mem[wr_ptr_bin[PTR_W-2:0]] <= wr_data;
  end

  // ---------------------------------------------------------------------------
  // Read pointer logic (rd_clk domain)
  // ---------------------------------------------------------------------------
  always_ff @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
      rd_ptr_bin  <= '0;
      rd_ptr_gray <= '0;
    end else if (rd_en && !rd_empty) begin
      rd_ptr_bin  <= rd_ptr_bin + 1;
      rd_ptr_gray <= bin2gray(rd_ptr_bin + 1);
    end
  end

  assign rd_data = mem[rd_ptr_bin[PTR_W-2:0]];

  // ---------------------------------------------------------------------------
  // Synchronize write gray pointer to read domain
  // ---------------------------------------------------------------------------
  genvar s;
  generate
    for (s = 0; s < PTR_W; s++) begin : gen_wr_sync
      pcie_sync2 #(.STAGES(STAGES)) u_wr_sync (
        .clk_dst   (rd_clk),
        .rst_dst_n (rd_rst_n),
        .d         (wr_ptr_gray[s]),
        .q         (wr_ptr_gray_sync[s])
      );
    end

    for (s = 0; s < PTR_W; s++) begin : gen_rd_sync
      pcie_sync2 #(.STAGES(STAGES)) u_rd_sync (
        .clk_dst   (wr_clk),
        .rst_dst_n (wr_rst_n),
        .d         (rd_ptr_gray[s]),
        .q         (rd_ptr_gray_sync[s])
      );
    end
  endgenerate

  // ---------------------------------------------------------------------------
  // Full/Empty generation
  // FIFO full  : wr_ptr MSBs differ, remaining bits equal (in gray domain)
  // FIFO empty : rd_ptr == wr_ptr_sync (all bits equal in gray domain)
  // ---------------------------------------------------------------------------
  logic [PTR_W-1:0] rd_ptr_gray_sync_bin;
  logic [PTR_W-1:0] wr_ptr_gray_sync_bin;

  assign rd_ptr_gray_sync_bin = gray2bin(rd_ptr_gray_sync);
  assign wr_ptr_gray_sync_bin = gray2bin(wr_ptr_gray_sync);

  // Full (evaluated in wr domain): next wr_ptr == rd_ptr with MSB flipped
  assign wr_full = (wr_ptr_gray == {~rd_ptr_gray_sync[PTR_W-1:PTR_W-2],
                                     rd_ptr_gray_sync[PTR_W-3:0]});

  // Almost full: one slot before full
  assign wr_almost_full = (wr_ptr_bin[PTR_W-2:0] + 1 ==
                            rd_ptr_gray_sync_bin[PTR_W-2:0]) &&
                           (wr_ptr_bin[PTR_W-1] == rd_ptr_gray_sync_bin[PTR_W-1]);

  // Empty (evaluated in rd domain)
  assign rd_empty = (rd_ptr_gray == wr_ptr_gray_sync);

  // Almost full in rd domain (used for backpressure)
  assign rd_almost_full = (wr_ptr_gray_sync_bin[PTR_W-2:0] + 2 ==
                            rd_ptr_bin[PTR_W-2:0]);

endmodule : pcie_async_fifo

`endif // PCIE_ASYNC_FIFO_SV
