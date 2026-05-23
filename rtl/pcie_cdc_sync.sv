`timescale 1ns/1ps

// =============================================================================
// PCIe 7.0 Controller - CDC Synchronization Primitives
// =============================================================================
// Two modules:
//   pcie_sync2   — 2-flop synchronizer for single-bit signals
//   pcie_sync_pulse — pulse synchronizer (single-cycle pulse across domains)
// =============================================================================

`ifndef PCIE_CDC_SYNC_SV
`define PCIE_CDC_SYNC_SV

// -----------------------------------------------------------------------------
// 2-flop synchronizer — single bit
// Use for: control signals, enable bits, status flags
// Do NOT use for multi-bit buses (use async FIFO instead)
// -----------------------------------------------------------------------------
module pcie_sync2 #(
  parameter int unsigned STAGES    = 2,
  parameter logic        RESET_VAL = 1'b0
)(
  input  logic clk_dst,
  input  logic rst_dst_n,
  input  logic d,
  output logic q
);
  logic [STAGES-1:0] sync_ff;

  always_ff @(posedge clk_dst or negedge rst_dst_n) begin
    if (!rst_dst_n)
      sync_ff <= {STAGES{RESET_VAL}};
    else
      sync_ff <= {sync_ff[STAGES-2:0], d};
  end

  assign q = sync_ff[STAGES-1];

  // Instruct synthesis/CDC tool: this is a intentional multi-flop CDC path
  // pragma synthesis_off
  // cdc_signal: sync_ff
  // pragma synthesis_on

endmodule : pcie_sync2

// -----------------------------------------------------------------------------
// Pulse synchronizer — converts a 1-cycle pulse in src domain to a 1-cycle
// pulse in dst domain using the toggle/detect pattern.
// Minimum gap between source pulses: 2 × dst clock periods.
// -----------------------------------------------------------------------------
module pcie_sync_pulse (
  input  logic clk_src,
  input  logic rst_src_n,
  input  logic clk_dst,
  input  logic rst_dst_n,
  input  logic pulse_src,     // 1-cycle pulse in clk_src domain
  output logic pulse_dst      // 1-cycle pulse in clk_dst domain
);
  logic toggle_src;
  logic toggle_dst;
  logic toggle_dst_q;

  // Toggle register in source domain
  always_ff @(posedge clk_src or negedge rst_src_n) begin
    if (!rst_src_n)
      toggle_src <= 1'b0;
    else if (pulse_src)
      toggle_src <= ~toggle_src;
  end

  // 2-flop synchronizer to destination domain
  pcie_sync2 #(.STAGES(2), .RESET_VAL(1'b0)) u_sync (
    .clk_dst   (clk_dst),
    .rst_dst_n (rst_dst_n),
    .d         (toggle_src),
    .q         (toggle_dst)
  );

  // Edge detect in destination domain
  always_ff @(posedge clk_dst or negedge rst_dst_n) begin
    if (!rst_dst_n)
      toggle_dst_q <= 1'b0;
    else
      toggle_dst_q <= toggle_dst;
  end

  assign pulse_dst = toggle_dst ^ toggle_dst_q;

endmodule : pcie_sync_pulse

// -----------------------------------------------------------------------------
// Reset synchronizer — synchronizes async reset de-assertion to destination
// clock domain.  Assertion is asynchronous (fast), de-assertion is synchronous.
// -----------------------------------------------------------------------------
module pcie_rst_sync #(
  parameter int unsigned STAGES = 2
)(
  input  logic clk_dst,
  input  logic async_rst_n,   // asynchronous reset input (active-low)
  output logic sync_rst_n     // synchronous reset output for clk_dst domain
);
  logic [STAGES-1:0] rst_pipe;

  always_ff @(posedge clk_dst or negedge async_rst_n) begin
    if (!async_rst_n)
      rst_pipe <= '0;
    else
      rst_pipe <= {rst_pipe[STAGES-2:0], 1'b1};
  end

  assign sync_rst_n = rst_pipe[STAGES-1];

endmodule : pcie_rst_sync

`endif // PCIE_CDC_SYNC_SV
