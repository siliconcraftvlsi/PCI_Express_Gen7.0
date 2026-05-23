`timescale 1ns/1ps

// =============================================================================
// PCIe 6.0/7.0 FLIT Framing Layer (simplified model)
// =============================================================================
// When flit_mode=1 (Gen6+), prepends a 32-bit FLIT header on TX and strips it
// on RX. When flit_mode=0 the module is a combinatorial pass-through.
// =============================================================================

`include "pcie_pkg.sv"

module pcie_flit_if
  import pcie_pkg::*;
#(
  parameter int unsigned DATA_W = 256
)(
  input  logic              clk,
  input  logic              rst_n,
  input  logic              link_up,
  input  logic              flit_mode,

  // DLL-facing
  input  logic [DATA_W-1:0] dll_tx_data,
  input  logic              dll_tx_valid,
  output logic              dll_tx_ready,
  input  logic              dll_tx_sop,
  input  logic              dll_tx_eop,

  output logic [DATA_W-1:0] dll_rx_data,
  output logic              dll_rx_valid,
  input  logic              dll_rx_ready,
  output logic              dll_rx_sop,
  output logic              dll_rx_eop,
  output logic              dll_rx_error,

  // PIPE adapter-facing
  output logic [DATA_W-1:0] pipe_tx_data,
  output logic              pipe_tx_valid,
  input  logic              pipe_tx_ready,
  output logic              pipe_tx_sop,
  output logic              pipe_tx_eop,

  input  logic [DATA_W-1:0] pipe_rx_data,
  input  logic              pipe_rx_valid,
  output logic              pipe_rx_ready,
  input  logic              pipe_rx_sop,
  input  logic              pipe_rx_eop,
  input  logic              pipe_rx_error
);

  localparam int FLIT_HDR_W = 32;
  localparam int PAYLOAD_W  = DATA_W - FLIT_HDR_W;

  logic [7:0] tx_flit_seq;
  logic [7:0] rx_flit_seq;
  logic [FLIT_HDR_W-1:0] tx_flit_hdr;

  assign tx_flit_hdr = {8'hF0, tx_flit_seq, 8'h00, 8'h01};

  // ---------------------------------------------------------------------------
  // TX path
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      tx_flit_seq <= '0;
    else if (flit_mode && link_up && dll_tx_valid && dll_tx_ready && dll_tx_eop)
      tx_flit_seq <= tx_flit_seq + 8'd1;
  end

  generate
    if (1) begin : gen_tx
      always_comb begin
        if (!flit_mode) begin
          pipe_tx_data  = dll_tx_data;
          pipe_tx_valid = dll_tx_valid;
          pipe_tx_sop   = dll_tx_sop;
          pipe_tx_eop   = dll_tx_eop;
          dll_tx_ready  = pipe_tx_ready;
        end else begin
          pipe_tx_data  = {tx_flit_hdr, dll_tx_data[PAYLOAD_W-1:0]};
          pipe_tx_valid = dll_tx_valid;
          pipe_tx_sop   = dll_tx_sop;
          pipe_tx_eop   = dll_tx_eop;
          dll_tx_ready  = pipe_tx_ready;
        end
      end
    end
  endgenerate

  // ---------------------------------------------------------------------------
  // RX path
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      rx_flit_seq <= '0;
    else if (flit_mode && link_up && pipe_rx_valid && pipe_rx_ready && pipe_rx_eop)
      rx_flit_seq <= rx_flit_seq + 8'd1;
  end

  generate
    if (1) begin : gen_rx
      always_comb begin
        if (!flit_mode) begin
          dll_rx_data  = pipe_rx_data;
          dll_rx_valid = pipe_rx_valid;
          dll_rx_sop   = pipe_rx_sop;
          dll_rx_eop   = pipe_rx_eop;
          dll_rx_error = pipe_rx_error;
          pipe_rx_ready = dll_rx_ready;
        end else begin
          dll_rx_data  = {{FLIT_HDR_W{1'b0}}, pipe_rx_data[PAYLOAD_W-1:0]};
          dll_rx_valid = pipe_rx_valid;
          dll_rx_sop   = pipe_rx_sop;
          dll_rx_eop   = pipe_rx_eop;
          dll_rx_error = pipe_rx_error;
          pipe_rx_ready = dll_rx_ready;
        end
      end
    end
  endgenerate

endmodule : pcie_flit_if
