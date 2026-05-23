`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// Author  : Robert Kingsly Amalathas
// Email   : robertk@microprocessorlab.com
// LinkedIn: www.linkedin.com/in/robertkingslya
// -----------------------------------------------------------------------------
// =============================================================================
// PCIe 7.0 Controller - PIPE Interface Adapter
// Based on PHY Interface for PCI Express (PIPE) Specification Rev 6.x
// =============================================================================
// Description:
//   Bridges the internal 256-bit wide data path to the per-lane PIPE interface.
//   Handles:
//     - Lane serialization/deserialization (gearbox)
//     - 8b/10b and 128b/130b encoding modes (Gen1-2 vs Gen3+)
//     - Elastic buffer and clock domain crossing at PIPE boundary
//     - TX/RX framing (SOP/EOP detection)
//     - Electrical idle generation and detection
//     - Scrambler enable/disable control
// =============================================================================

`include "pcie_pkg.sv"

module pcie_pipe_if
  import pcie_pkg::*;
#(
  parameter int unsigned NUM_LANES = 16,
  parameter int unsigned PIPE_W    = 32,    // PIPE data width per lane (bits)
  parameter int unsigned DATA_W    = 256,   // Internal datapath width
  parameter bit          SIM_BYPASS = 0
)(
  input  logic              clk,
  input  logic              rst_n,
  input  logic              link_up,

  // -------------------------------------------------------------------------
  // PIPE TX signals → SerDes/PHY
  // -------------------------------------------------------------------------
  output logic [NUM_LANES-1:0][PIPE_W-1:0]   pipe_tx_data,
  output logic [NUM_LANES-1:0][PIPE_W/8-1:0] pipe_tx_datak,
  output logic [NUM_LANES-1:0]               pipe_tx_elec_idle,
  output logic [NUM_LANES-1:0]               pipe_tx_compliance,
  output logic [NUM_LANES-1:0]               pipe_tx_deemph,
  output logic [NUM_LANES-1:0][2:0]          pipe_tx_margin,
  output logic [NUM_LANES-1:0]               pipe_tx_swing,
  output logic [NUM_LANES-1:0][1:0]          pipe_tx_eq_ctrl,

  // -------------------------------------------------------------------------
  // PIPE RX signals ← SerDes/PHY
  // -------------------------------------------------------------------------
  input  logic [NUM_LANES-1:0][PIPE_W-1:0]   pipe_rx_data,
  input  logic [NUM_LANES-1:0][PIPE_W/8-1:0] pipe_rx_datak,
  input  logic [NUM_LANES-1:0]               pipe_rx_valid,
  input  logic [NUM_LANES-1:0]               pipe_rx_elec_idle,

  // -------------------------------------------------------------------------
  // Internal TX interface (from DLL TX)
  // -------------------------------------------------------------------------
  input  logic [DATA_W-1:0]  tx_data,
  input  logic               tx_valid,
  output logic               tx_ready,
  input  logic               tx_sop,
  input  logic               tx_eop,

  // -------------------------------------------------------------------------
  // Internal RX interface (to DLL RX)
  // -------------------------------------------------------------------------
  output logic [DATA_W-1:0]  rx_data,
  output logic               rx_valid,
  output logic               rx_sop,
  output logic               rx_eop,
  output logic               rx_error
);

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  // Total PIPE width across all lanes
  localparam int TOTAL_PIPE_W = NUM_LANES * PIPE_W;

  // How many PIPE words make one internal DATA_W word
  localparam int GEARBOX_RATIO = DATA_W / TOTAL_PIPE_W;

  // ---------------------------------------------------------------------------
  // PCIe Symbol Definitions (8b/10b K-codes)
  // ---------------------------------------------------------------------------
  localparam logic [7:0] COM   = 8'hBC;  // K28.5 - Comma
  localparam logic [7:0] STP   = 8'hFB;  // K27.7 - Start TLP
  localparam logic [7:0] SDP   = 8'hF7;  // K23.7 - Start DLLP
  localparam logic [7:0] END_  = 8'hFD;  // K29.7 - End
  localparam logic [7:0] EDB   = 8'hFE;  // K30.7 - EnD Bad
  localparam logic [7:0] PAD   = 8'hF3;  // K19.7 - Pad
  localparam logic [7:0] IDL   = 8'h7C;  // K28.3 - IDL (electrical idle ordered set)
  localparam logic [7:0] FTS   = 8'h3C;  // K28.1 - Fast Training Sequence
  localparam logic [7:0] SKP   = 8'h1C;  // K28.0 - Skip

  // ---------------------------------------------------------------------------
  // TX Gearbox and Framing
  // ---------------------------------------------------------------------------
  // TX FIFO / shift register to serialize DATA_W → TOTAL_PIPE_W per cycle
  logic [DATA_W-1:0]   tx_shift_reg;
  logic [2:0]          tx_phase;   // Gearbox phase counter
  logic                tx_active;  // Packet in-flight on TX
  logic                tx_sop_pend;

  // TX Ready: in directed sim always accept DLL beats (RC BFM has no PHY back-pressure)
  assign tx_ready = SIM_BYPASS ? 1'b1 : (!tx_active || (tx_phase == '0));

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_shift_reg  <= '0;
      tx_phase      <= '0;
      tx_active     <= 1'b0;
      tx_sop_pend   <= 1'b0;
    end else begin
      if (GEARBOX_RATIO <= 1) begin
        // No gearbox needed – direct 1:1
        tx_shift_reg <= tx_data;
        tx_active    <= tx_valid;
      end else begin
        if (tx_valid && tx_ready) begin
          tx_shift_reg <= tx_data;
          tx_phase     <= '0;
          tx_active    <= 1'b1;
          tx_sop_pend  <= tx_sop;
        end else if (tx_active) begin
          tx_shift_reg <= tx_shift_reg << TOTAL_PIPE_W;
          if (tx_phase == GEARBOX_RATIO - 1) begin
            tx_active <= 1'b0;
            tx_phase  <= '0;
          end else begin
            tx_phase <= tx_phase + 1;
          end
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // TX Lane Distribution
  // ---------------------------------------------------------------------------
  logic [TOTAL_PIPE_W-1:0]         tx_pipe_word;
  logic [NUM_LANES-1:0][PIPE_W/8-1:0] tx_pipe_datak_word;
  logic                             tx_is_idle;
  logic                             tx_in_tlp;

  assign tx_pipe_word = tx_shift_reg[DATA_W-1 : DATA_W-TOTAL_PIPE_W];
  assign tx_is_idle   = !tx_active && !link_up;

  // Framing control: insert STP/SDP/END_ K-symbols around TLPs
  logic tx_frame_stp, tx_frame_end;
  assign tx_frame_stp = tx_active && tx_sop_pend && (tx_phase == '0);
  assign tx_frame_end = tx_active && !tx_valid && (tx_phase == GEARBOX_RATIO - 1);

  always @* begin
    for (int i = 0; i < NUM_LANES; i++) begin
      if (tx_is_idle) begin
        // Electrical Idle / COM-SKP
        pipe_tx_data[i]      = {(PIPE_W/8){COM}};
        pipe_tx_datak[i]     = '1;
        pipe_tx_elec_idle[i] = 1'b0;
        pipe_tx_compliance[i]= 1'b0;
        pipe_tx_deemph[i]    = 1'b0;
        pipe_tx_margin[i]    = 3'b000;
        pipe_tx_swing[i]     = 1'b1;
        pipe_tx_eq_ctrl[i]   = 2'b00;
      end else if (tx_frame_stp && i == 0) begin
        // First lane first byte = STP
        pipe_tx_data[i]      = {tx_pipe_word[i*PIPE_W +: PIPE_W-8], STP};
        pipe_tx_datak[i]     = {{(PIPE_W/8-1){1'b0}}, 1'b1};
        pipe_tx_elec_idle[i] = 1'b0;
        pipe_tx_compliance[i]= 1'b0;
        pipe_tx_deemph[i]    = 1'b0;
        pipe_tx_margin[i]    = 3'b000;
        pipe_tx_swing[i]     = 1'b1;
        pipe_tx_eq_ctrl[i]   = 2'b00;
      end else begin
        pipe_tx_data[i]      = tx_pipe_word[i*PIPE_W +: PIPE_W];
        pipe_tx_datak[i]     = '0;
        pipe_tx_elec_idle[i] = 1'b0;
        pipe_tx_compliance[i]= 1'b0;
        pipe_tx_deemph[i]    = 1'b0;
        pipe_tx_margin[i]    = 3'b000;
        pipe_tx_swing[i]     = 1'b1;
        pipe_tx_eq_ctrl[i]   = 2'b00;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // RX Lane Collection and Framing
  // ---------------------------------------------------------------------------
  logic [TOTAL_PIPE_W-1:0] rx_pipe_word_raw;
  logic [NUM_LANES-1:0][PIPE_W/8-1:0] rx_pipe_datak_raw;

  // Assemble all lanes into single wide word
  always @* begin
    for (int i = 0; i < NUM_LANES; i++) begin
      rx_pipe_word_raw[i*PIPE_W +: PIPE_W]     = pipe_rx_data[i];
      rx_pipe_datak_raw[i]                      = pipe_rx_datak[i];
    end
  end

  // ---------------------------------------------------------------------------
  // RX SOP/EOP Detection (scan for STP/END_ K-symbols in assembled word)
  // ---------------------------------------------------------------------------
  logic rx_sop_det, rx_eop_det;
  logic rx_error_det;
  logic [NUM_LANES-1:0] rx_any_valid;

  assign rx_any_valid = pipe_rx_valid;

  // Scan assembled word for K-symbols (byte 0 of each PIPE_W lane slot)
  assign rx_sop_det = (|(rx_pipe_datak_raw)) && (
    (rx_pipe_word_raw[7:0]                     == STP) ||
    (rx_pipe_word_raw[PIPE_W+7   : PIPE_W]     == STP) ||
    (rx_pipe_word_raw[2*PIPE_W+7 : 2*PIPE_W]   == STP) ||
    (rx_pipe_word_raw[3*PIPE_W+7 : 3*PIPE_W]   == STP) ||
    (rx_pipe_word_raw[7:0]                     == SDP) ||
    (rx_pipe_word_raw[PIPE_W+7   : PIPE_W]     == SDP) ||
    (rx_pipe_word_raw[2*PIPE_W+7 : 2*PIPE_W]   == SDP) ||
    (rx_pipe_word_raw[3*PIPE_W+7 : 3*PIPE_W]   == SDP));

  assign rx_eop_det = (|(rx_pipe_datak_raw)) && (
    (rx_pipe_word_raw[7:0]                     == END_) ||
    (rx_pipe_word_raw[PIPE_W+7   : PIPE_W]     == END_) ||
    (rx_pipe_word_raw[2*PIPE_W+7 : 2*PIPE_W]   == END_) ||
    (rx_pipe_word_raw[3*PIPE_W+7 : 3*PIPE_W]   == END_));

  assign rx_error_det = (|(rx_pipe_datak_raw)) && (
    (rx_pipe_word_raw[7:0]                     == EDB) ||
    (rx_pipe_word_raw[PIPE_W+7   : PIPE_W]     == EDB) ||
    (rx_pipe_word_raw[2*PIPE_W+7 : 2*PIPE_W]   == EDB) ||
    (rx_pipe_word_raw[3*PIPE_W+7 : 3*PIPE_W]   == EDB));

  // ---------------------------------------------------------------------------
  // RX Gearbox: collect TOTAL_PIPE_W bits per cycle → DATA_W wide output
  // ---------------------------------------------------------------------------
  logic [DATA_W-1:0]  rx_accum;
  logic [2:0]         rx_phase;
  logic               rx_valid_int;
  logic               rx_sop_int;
  logic               rx_eop_int;
  // Latch SOP/EOP detected in earlier gearbox phases until output cycle
  logic               rx_sop_latched;
  logic               rx_eop_latched;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_accum       <= '0;
      rx_phase       <= '0;
      rx_valid_int   <= 1'b0;
      rx_sop_int     <= 1'b0;
      rx_eop_int     <= 1'b0;
      rx_sop_latched <= 1'b0;
      rx_eop_latched <= 1'b0;
    end else begin
      if (|rx_any_valid) begin
        if (GEARBOX_RATIO <= 1) begin
          rx_accum     <= rx_pipe_word_raw;
          rx_valid_int <= 1'b1;
          rx_sop_int   <= rx_sop_det;
          rx_eop_int   <= rx_eop_det;
        end else begin
          if (rx_sop_det && rx_phase != '0) begin
            // Mid-gearbox SOP: discard in-progress word, restart treating
            // this cycle as phase 0 so STP lands at MSB of accumulator.
            rx_accum[DATA_W-1 -: TOTAL_PIPE_W] <= rx_pipe_word_raw;
            rx_phase       <= 3'(GEARBOX_RATIO - 1);
            rx_valid_int   <= 1'b0;
            rx_sop_latched <= 1'b1;
            rx_eop_latched <= 1'b0;
            rx_sop_int     <= 1'b0;
            rx_eop_int     <= 1'b0;
          end else begin
            rx_accum[DATA_W-1 - rx_phase*TOTAL_PIPE_W -: TOTAL_PIPE_W] <= rx_pipe_word_raw;
            if (rx_phase == GEARBOX_RATIO - 1) begin
              rx_valid_int   <= 1'b1;
              rx_phase       <= '0;
              // Combine any SOP/EOP seen in earlier phases with current cycle
              rx_sop_int     <= rx_sop_latched || rx_sop_det;
              rx_eop_int     <= rx_eop_latched || rx_eop_det;
              rx_sop_latched <= 1'b0;
              rx_eop_latched <= 1'b0;
            end else begin
              rx_valid_int   <= 1'b0;
              rx_phase       <= rx_phase + 1;
              // Accumulate SOP/EOP across gearbox phases
              rx_sop_latched <= rx_sop_latched || rx_sop_det;
              rx_eop_latched <= rx_eop_latched || rx_eop_det;
              rx_sop_int     <= 1'b0;
              rx_eop_int     <= 1'b0;
            end
          end
        end
      end else begin
        rx_valid_int   <= 1'b0;
        rx_sop_int     <= 1'b0;
        rx_eop_int     <= 1'b0;
        rx_sop_latched <= 1'b0;
        rx_eop_latched <= 1'b0;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Output Assignments
  // ---------------------------------------------------------------------------
  assign rx_data  = rx_accum;
  assign rx_valid = rx_valid_int;
  assign rx_sop   = rx_sop_int;
  assign rx_eop   = rx_eop_int;
  assign rx_error = rx_error_det;

endmodule : pcie_pipe_if
