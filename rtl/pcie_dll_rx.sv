`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// Author  : Robert Kingsly Amalathas
// Email   : robertk@microprocessorlab.com
// LinkedIn: www.linkedin.com/in/robertkingslya
// -----------------------------------------------------------------------------
// =============================================================================
// PCIe 7.0 Controller - Data Link Layer RX
// Based on PCI Express Base Specification Rev 7.0 Section 3
// =============================================================================
// Description:
//   Implements the Receive side of the PCIe Data Link Layer:
//     - Strips and validates 12-bit sequence numbers
//     - Verifies LCRC-32 on each received TLP
//     - Issues ACK / NAK DLLPs to the remote transmitter
//     - Parses received DLLPs (ACK, NAK, FC Init/Update, PM)
//     - Validates 16-bit DLCRC on received DLLPs via calc_dlcrc()
//     - Detects duplicate TLPs (seq == expected-1) and ACKs without forwarding
//     - Detects out-of-window sequence numbers (>4096 ahead) and NAKs
//     - Drives FC DLLP parse outputs (fc_dllp_valid, type, hdr, data)
//     - Drives PM DLLP parse outputs (pm_dllp_valid, pm_dllp_type)
//     - Forwards validated TLPs to Transaction Layer
// =============================================================================

`include "pcie_pkg.sv"

module pcie_dll_rx
  import pcie_pkg::*;
#(
  parameter int unsigned DATA_W     = 256,
  parameter bit          SIM_BYPASS      = 0,  // Bypass seq checks for simulation
  parameter bit          SIM_BYPASS_LCRC = SIM_BYPASS  // Bypass LCRC only (seq may stay bypassed)
)(
  input  logic              clk,
  input  logic              rst_n,
  input  logic              link_up,

  // -------------------------------------------------------------------------
  // Input from PIPE Interface
  // -------------------------------------------------------------------------
  input  logic [DATA_W-1:0]  phy_rx_data,
  input  logic               phy_rx_valid,
  input  logic               phy_rx_sop,
  input  logic               phy_rx_eop,
  input  logic               phy_rx_error,
  input  logic               sim_lcrc_check_en,  // Directed TB: force LCRC check for one packet

  // -------------------------------------------------------------------------
  // Output to Transaction Layer
  // -------------------------------------------------------------------------
  output logic [DATA_W-1:0]  tl_rx_data,
  output logic               tl_rx_valid,
  output logic               tl_rx_sop,
  output logic               tl_rx_eop,
  output logic               tl_rx_error,

  // -------------------------------------------------------------------------
  // ACK/NAK signals to DLL TX (for generating ACK/NAK DLLPs)
  // -------------------------------------------------------------------------
  output logic               nak_out,
  output logic [11:0]        ack_seq_out,
  output logic [11:0]        nak_seq_out,

  // -------------------------------------------------------------------------
  // FC DLLP parse outputs (to flow-control manager)
  // -------------------------------------------------------------------------
  output logic               fc_dllp_valid,   // Pulse: valid FC DLLP received
  output dllp_type_e         fc_dllp_type,    // INIT/UPD P/NP/CPL type
  output logic [11:0]        fc_dllp_hdr,     // Parsed header-credit field
  output logic [19:0]        fc_dllp_data,    // Parsed data-credit field

  // -------------------------------------------------------------------------
  // PM DLLP parse outputs (to PM controller)
  // -------------------------------------------------------------------------
  output logic               pm_dllp_valid,   // Pulse: valid PM DLLP received
  output dllp_type_e         pm_dllp_type,    // PM DLLP type

  // -------------------------------------------------------------------------
  // DLLP error output
  // -------------------------------------------------------------------------
  output logic               dllp_err         // DLCRC mismatch on received DLLP
);

  // ---------------------------------------------------------------------------
  // Sequence Number Tracking
  // ---------------------------------------------------------------------------
  logic [11:0]  next_expected_seq;   // Next expected sequence number
  logic [11:0]  rx_seq_num;          // Extracted sequence number from packet

  // Sequence window: out-of-window if distance > 4096 (12-bit wrap-around safe)
  // For 12-bit seq numbers the window is [next_expected, next_expected+4095]
  logic [12:0]  seq_distance;
  assign seq_distance = {1'b0, rx_seq_num} - {1'b0, next_expected_seq};
  // Out-of-window: distance >= 4096 (using unsigned 13-bit comparison)
  logic  seq_out_of_window;
  assign seq_out_of_window = seq_distance[12] ||   // wrapped negative (behind)
                              (seq_distance >= 13'd4096);

  // Duplicate: seq_num == next_expected_seq - 1
  logic  seq_is_duplicate;
  assign seq_is_duplicate = (rx_seq_num == (next_expected_seq - 12'd1));

  // ---------------------------------------------------------------------------
  // CRC Checker
  // ---------------------------------------------------------------------------
  logic [31:0]  rx_crc_accum;        // Running LCRC accumulator
  logic [31:0]  rx_crc_received;     // LCRC extracted from tail of packet
  logic         crc_active;
  logic         crc_ok;

  // ---------------------------------------------------------------------------
  // DLLP DLCRC validation
  // ---------------------------------------------------------------------------
  // Received DLLP: bits [47:16] = type + seq/reserved, bits [15:0] = dlcrc
  // First 32 bits = phy_rx_data[47:16] = {type[7:0], fields[23:0]}
  logic [15:0]  rx_dlcrc_expected;
  logic [15:0]  rx_dlcrc_received;
  logic [31:0]  rx_dllp_crc_input;   // First 32 bits of received DLLP

  always @* begin
    // The DLLP first 32 bits are the upper 32 of a 48-bit DLLP word
    // Layout in phy_rx_data (lower 48 bits used):
    //   [47:40] = type, [39:16] = fields, [15:0] = dlcrc
    rx_dllp_crc_input = phy_rx_data[47:16];
    rx_dlcrc_received = phy_rx_data[15:0];
    rx_dlcrc_expected = calc_dlcrc(rx_dllp_crc_input);
  end

  // ---------------------------------------------------------------------------
  // Receive State Machine
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    DLL_RX_IDLE,
    DLL_RX_SEQ,        // Strip sequence number
    DLL_RX_DATA,       // Forward to TL
    DLL_RX_CRC_CHK,    // Check LCRC
    DLL_RX_DLLP        // Received a DLLP (ACK/FC/PM)
  } dll_rx_state_e;

  dll_rx_state_e  rx_state;

  // Pipeline registers
  logic [DATA_W-1:0]  rx_data_pipe;
  logic               rx_valid_pipe;
  logic               rx_sop_pipe;
  logic               rx_eop_pipe;
  logic               rx_in_tlp;
  logic               rx_in_dllp;

  // Saved previous beat for LCRC extraction
  logic [DATA_W-1:0]  rx_prev_data;

  // Flag: first data beat after seq header (drives tl_rx_sop)
  logic  first_data_beat;

  // ACK delay buffer for out-of-order
  logic [11:0]  ack_pend_seq;
  logic         ack_pend_valid;

  // ---------------------------------------------------------------------------
  // DLLP Type Extraction
  // ---------------------------------------------------------------------------
  dllp_type_e  rx_dllp_type;
  logic [11:0] rx_dllp_seq;  // For ACK/NAK DLLPs

  always @* begin
    rx_dllp_type = dllp_type_e'(phy_rx_data[47:40]);  // type in bits [47:40]
    rx_dllp_seq  = phy_rx_data[35:24];                 // seq in bits [35:24]
  end

  // ---------------------------------------------------------------------------
  // FC field extraction from received DLLP
  // FC DLLP layout (48-bit): {type[7:0], hdr_fc[11:0], data_fc[19:0], dlcrc[15:0]}
  // In phy_rx_data lower 48 bits:
  //   [47:40] = type
  //   [39:28] = hdr_fc[11:0]
  //   [27: 8] = data_fc[19:0]
  //   [ 7: 0] = dlcrc[7:0]  (note: dlcrc is [15:0], split across bits — see below)
  // Practical: store as {type[7:0], hdr_fc[11:0], data_fc[19:0], dlcrc[15:0]} = 56 bits
  // But the actual 48-bit DLLP word uses: {type, hdr[11:8], vc[3:0], hdr[7:0], data[19:8], data[7:0], dlcrc[15:8], dlcrc[7:0]}
  // Simplified extraction matches the TX packing above:
  logic [11:0]  rx_fc_hdr;
  logic [19:0]  rx_fc_data;

  always @* begin
    // Match TX FC DLLP word packing: bits[55:48]=type, [47:36]=hdr, [35:16]=data, [15:0]=dlcrc
    // The bus is DATA_W wide; FC DLLP occupies lower 56 bits
    rx_fc_hdr  = phy_rx_data[47:36];
    rx_fc_data = phy_rx_data[35:16];
  end

  // ---------------------------------------------------------------------------
  // Helper: is FC type (init or update)
  // ---------------------------------------------------------------------------
  function automatic logic is_fc_dllp_type(input dllp_type_e t);
    case (t)
      DLLP_FC_INIT_P, DLLP_FC_INIT_NP, DLLP_FC_INIT_CPL,
      DLLP_FC_UPD_P,  DLLP_FC_UPD_NP,  DLLP_FC_UPD_CPL: is_fc_dllp_type = 1'b1;
      default: is_fc_dllp_type = 1'b0;
    endcase
  endfunction

  function automatic logic is_pm_dllp_type(input dllp_type_e t);
    case (t)
      DLLP_PM_ENTER_L1, DLLP_PM_ENTER_L23,
      DLLP_PM_ACT_STATE_REQ, DLLP_PM_REQ_ACK: is_pm_dllp_type = 1'b1;
      default: is_pm_dllp_type = 1'b0;
    endcase
  endfunction

  function automatic logic is_rcvd_dllp_type(input logic [7:0] t);
    unique case (dllp_type_e'(t))
      DLLP_ACK, DLLP_NAK,
      DLLP_FC_INIT_P, DLLP_FC_INIT_NP, DLLP_FC_INIT_CPL,
      DLLP_FC_UPD_P,  DLLP_FC_UPD_NP,  DLLP_FC_UPD_CPL,
      DLLP_PM_ENTER_L1, DLLP_PM_ENTER_L23,
      DLLP_PM_ACT_STATE_REQ, DLLP_PM_REQ_ACK: is_rcvd_dllp_type = 1'b1;
      default: is_rcvd_dllp_type = 1'b0;
    endcase
  endfunction

  // ---------------------------------------------------------------------------
  // Main RX State Machine
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_state           <= DLL_RX_IDLE;
      next_expected_seq  <= '0;
      rx_crc_accum       <= LCRC_INIT;
      crc_active         <= 1'b0;
      tl_rx_valid        <= 1'b0;
      tl_rx_sop          <= 1'b0;
      tl_rx_eop          <= 1'b0;
      tl_rx_error        <= 1'b0;
      tl_rx_data         <= '0;
      nak_out            <= 1'b0;
      ack_seq_out        <= '0;
      nak_seq_out        <= '0;
      ack_pend_valid     <= 1'b0;
      rx_in_tlp          <= 1'b0;
      rx_in_dllp         <= 1'b0;
      first_data_beat    <= 1'b0;
      fc_dllp_valid      <= 1'b0;
      fc_dllp_type       <= DLLP_FC_INIT_P;
      fc_dllp_hdr        <= '0;
      fc_dllp_data       <= '0;
      pm_dllp_valid      <= 1'b0;
      pm_dllp_type       <= DLLP_PM_ENTER_L1;
      dllp_err           <= 1'b0;
    end else if (!link_up) begin
      rx_state           <= DLL_RX_IDLE;
      next_expected_seq  <= '0;
      rx_crc_accum       <= LCRC_INIT;
      tl_rx_valid        <= 1'b0;
      nak_out            <= 1'b0;
      ack_pend_valid     <= 1'b0;
      fc_dllp_valid      <= 1'b0;
      pm_dllp_valid      <= 1'b0;
      dllp_err           <= 1'b0;
    end else begin
      // Default single-cycle pulse outputs
      tl_rx_valid   <= 1'b0;
      tl_rx_sop     <= 1'b0;
      tl_rx_eop     <= 1'b0;
      tl_rx_error   <= 1'b0;
      nak_out       <= 1'b0;
      fc_dllp_valid <= 1'b0;
      pm_dllp_valid <= 1'b0;
      dllp_err      <= 1'b0;

      case (rx_state)

        DLL_RX_IDLE: begin
          // RC BFM single-beat DLLP (ACK/NAK) — DLCRC must match to avoid false ACK on zero tails
          if (phy_rx_valid && !rx_in_tlp && !rx_in_dllp &&
              (~|phy_rx_data[255:56]) &&
              (rx_dlcrc_expected == rx_dlcrc_received)) begin
            if (phy_rx_data[47:40] == 8'(DLLP_NAK)) begin
              nak_out     <= 1'b1;
              nak_seq_out <= phy_rx_data[35:24];
            end else if (phy_rx_data[47:40] == 8'(DLLP_ACK)) begin
              ack_seq_out <= phy_rx_data[35:24];
            end
          end else if (phy_rx_valid && phy_rx_sop) begin
            // Distinguish TLP vs DLLP by framing marker
            rx_seq_num   <= phy_rx_data[27:16];
            rx_data_pipe <= phy_rx_data;
            if (phy_rx_data[31:28] == 4'hA) begin
              // DLLP (SDP-style framing marker only; do not use [47:40]==0 — that
              // matches almost every STP+seq gearbox word and drops RC CplD TLPs)
              rx_state   <= DLL_RX_DLLP;
              rx_in_dllp <= 1'b1;
            end else begin
              $display("[DLL-RX] SOP TLP accepted @ %0t, data[255:224]=%08h", $time, phy_rx_data[255:224]);
              rx_state     <= DLL_RX_SEQ;
              rx_in_tlp    <= 1'b1;
              rx_crc_accum <= LCRC_INIT;
              crc_active   <= 1'b1;
            end
          end
        end

        DLL_RX_SEQ: begin
          if (SIM_BYPASS) begin
            // Bypass: accept any sequence number without validation
            rx_state        <= DLL_RX_DATA;
            first_data_beat <= 1'b1;
            tl_rx_valid     <= 1'b0;
          end else if (seq_is_duplicate) begin
            ack_seq_out <= rx_seq_num;
            rx_state    <= DLL_RX_IDLE;
            rx_in_tlp   <= 1'b0;
          end else if (seq_out_of_window) begin
            nak_out     <= 1'b1;
            nak_seq_out <= next_expected_seq;
            rx_state    <= DLL_RX_IDLE;
            rx_in_tlp   <= 1'b0;
          end else if (rx_seq_num == next_expected_seq) begin
            rx_state        <= DLL_RX_DATA;
            first_data_beat <= 1'b1;
            tl_rx_valid     <= 1'b0;
          end else begin
            nak_out     <= 1'b1;
            nak_seq_out <= next_expected_seq;
            rx_state    <= DLL_RX_IDLE;
            rx_in_tlp   <= 1'b0;
          end
        end

        DLL_RX_DATA: begin
          if (phy_rx_valid) begin
            for (int dw = 0; dw < DATA_W/32; dw++) begin
              rx_crc_accum <= calc_lcrc(rx_crc_accum, phy_rx_data[dw*32 +: 32]);
            end

            if (!phy_rx_eop) begin
              tl_rx_data      <= phy_rx_data;
              tl_rx_valid     <= 1'b1;
              tl_rx_sop       <= first_data_beat;
              tl_rx_eop       <= 1'b0;
              first_data_beat <= 1'b0;
              rx_prev_data    <= phy_rx_data;
            end else begin
              rx_crc_received <= phy_rx_data[31:0];
              rx_state        <= DLL_RX_CRC_CHK;
              tl_rx_valid     <= 1'b0;
              first_data_beat <= 1'b0;
            end
          end else begin
            tl_rx_valid <= 1'b0;
          end
        end

        DLL_RX_CRC_CHK: begin
          crc_ok <= (SIM_BYPASS_LCRC && !sim_lcrc_check_en) ||
                    (rx_crc_accum == ~rx_crc_received);
          if ((SIM_BYPASS_LCRC && !sim_lcrc_check_en) ||
              (rx_crc_accum == ~rx_crc_received)) begin
            $display("[DLL-RX] TLP forwarded to TL @ %0t, data[255:224]=%08h", $time, rx_prev_data[255:224]);
            tl_rx_data    <= rx_prev_data;
            tl_rx_valid   <= 1'b1;
            tl_rx_eop     <= 1'b1;
            tl_rx_error   <= 1'b0;
            ack_seq_out   <= next_expected_seq;
            ack_pend_seq  <= next_expected_seq;
            ack_pend_valid <= 1'b1;
            next_expected_seq <= next_expected_seq + 1;
          end else begin
            // CRC fail — NAK, discard
            tl_rx_error <= 1'b1;
            nak_out     <= 1'b1;
            nak_seq_out <= next_expected_seq;
          end
          rx_state   <= DLL_RX_IDLE;
          rx_in_tlp  <= 1'b0;
          crc_active <= 1'b0;
        end

        DLL_RX_DLLP: begin
          // Process received DLLP on EOP beat; validate DLCRC first
          if (phy_rx_valid && phy_rx_eop) begin
            // DLCRC check: compare computed CRC against received
            if (rx_dlcrc_expected != rx_dlcrc_received) begin
              // DLCRC mismatch — discard DLLP, assert error
              dllp_err   <= 1'b1;
            end else begin
              // DLCRC valid — parse DLLP type
              case (rx_dllp_type)
                DLLP_ACK: begin
                  ack_seq_out <= rx_dllp_seq;
                end

                DLLP_NAK: begin
                  nak_out     <= 1'b1;
                  nak_seq_out <= rx_dllp_seq;
                end

                DLLP_FC_INIT_P, DLLP_FC_INIT_NP, DLLP_FC_INIT_CPL,
                DLLP_FC_UPD_P,  DLLP_FC_UPD_NP,  DLLP_FC_UPD_CPL: begin
                  // Extract FC fields and drive outputs
                  fc_dllp_valid <= 1'b1;
                  fc_dllp_type  <= rx_dllp_type;
                  fc_dllp_hdr   <= rx_fc_hdr;
                  fc_dllp_data  <= rx_fc_data;
                end

                DLLP_PM_ENTER_L1, DLLP_PM_ENTER_L23,
                DLLP_PM_ACT_STATE_REQ, DLLP_PM_REQ_ACK: begin
                  pm_dllp_valid <= 1'b1;
                  pm_dllp_type  <= rx_dllp_type;
                end

                default: ;  // Unknown DLLP type — silently discard
              endcase
            end
            rx_state   <= DLL_RX_IDLE;
            rx_in_dllp <= 1'b0;
          end
        end

        default: rx_state <= DLL_RX_IDLE;
      endcase

      // Error injection: PHY-layer error on active TLP — NAK and discard
      if (phy_rx_error && rx_in_tlp) begin
        tl_rx_error <= 1'b1;
        rx_state    <= DLL_RX_IDLE;
        rx_in_tlp   <= 1'b0;
        nak_out     <= 1'b1;
        nak_seq_out <= next_expected_seq;
        crc_active  <= 1'b0;
      end
    end
  end

endmodule : pcie_dll_rx
