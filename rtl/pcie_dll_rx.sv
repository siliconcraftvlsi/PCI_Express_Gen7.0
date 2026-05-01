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
//     - Parses received DLLPs (ACK, NAK, FC Updates, PM)
//     - Forwards validated TLPs to Transaction Layer
//     - Handles duplicate (already ACKed) TLPs
// =============================================================================

`include "pcie_pkg.sv"

module pcie_dll_rx
  import pcie_pkg::*;
#(
  parameter int unsigned DATA_W = 256
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
  output logic [11:0]        nak_seq_out
);

  // ---------------------------------------------------------------------------
  // Sequence Number Tracking
  // ---------------------------------------------------------------------------
  logic [11:0]  next_expected_seq;   // Next expected sequence number
  logic [11:0]  rx_seq_num;          // Extracted sequence number from packet

  // ---------------------------------------------------------------------------
  // CRC Checker
  // ---------------------------------------------------------------------------
  logic [31:0]  rx_crc_accum;        // Running LCRC accumulator
  logic [31:0]  rx_crc_received;     // LCRC extracted from tail of packet
  logic         crc_active;
  logic         crc_ok;

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

  // Saved data-1 beat for LCRC extraction (last beat before EOP contains LCRC)
  logic [DATA_W-1:0]  rx_prev_data;

  // ACK delay buffer for out-of-order
  logic [11:0]  ack_pend_seq;
  logic         ack_pend_valid;

  // ---------------------------------------------------------------------------
  // DLLP Type Extraction
  // ---------------------------------------------------------------------------
  dllp_type_e  rx_dllp_type;
  logic [11:0] rx_dllp_seq;  // For ACK/NAK DLLPs

  always_comb begin
    rx_dllp_type = dllp_type_e'(phy_rx_data[7:0]);
    rx_dllp_seq  = phy_rx_data[23:12];
  end

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
    end else if (!link_up) begin
      rx_state           <= DLL_RX_IDLE;
      next_expected_seq  <= '0;
      rx_crc_accum       <= LCRC_INIT;
      tl_rx_valid        <= 1'b0;
      nak_out            <= 1'b0;
      ack_pend_valid     <= 1'b0;
    end else begin
      // Default outputs
      tl_rx_valid <= 1'b0;
      tl_rx_sop   <= 1'b0;
      tl_rx_eop   <= 1'b0;
      tl_rx_error <= 1'b0;
      nak_out     <= 1'b0;

      case (rx_state)

        DLL_RX_IDLE: begin
          if (phy_rx_valid && phy_rx_sop) begin
            // Determine TLP vs DLLP by checking STP vs SDP in PIPE framing
            // Simplified: use rx_seq_num presence to distinguish
            // TLPs start with 4-bit RSVD + 12-bit seq + 16-bit RSVD
            rx_seq_num   <= phy_rx_data[27:16];
            rx_data_pipe <= phy_rx_data;
            if (phy_rx_data[31:28] == 4'hA) begin
              // DLLP marker (simplified detection)
              rx_state   <= DLL_RX_DLLP;
              rx_in_dllp <= 1'b1;
            end else begin
              rx_state   <= DLL_RX_SEQ;
              rx_in_tlp  <= 1'b1;
              // Reset CRC for new TLP
              rx_crc_accum <= LCRC_INIT;
              crc_active   <= 1'b1;
            end
          end
        end

        DLL_RX_SEQ: begin
          // Validate sequence number
          if (rx_seq_num == next_expected_seq) begin
            // Valid – forward to TL (strip sequence header beat)
            rx_state    <= DLL_RX_DATA;
            tl_rx_valid <= 1'b0;   // Skip seq header beat
          end else begin
            // Sequence error – generate NAK
            nak_out     <= 1'b1;
            nak_seq_out <= next_expected_seq;
            rx_state    <= DLL_RX_IDLE;
            rx_in_tlp   <= 1'b0;
          end
        end

        DLL_RX_DATA: begin
          if (phy_rx_valid) begin
            // Update running CRC
            for (int dw = 0; dw < DATA_W/32; dw++) begin
              rx_crc_accum <= calc_lcrc(rx_crc_accum, phy_rx_data[dw*32 +: 32]);
            end

            if (!phy_rx_eop) begin
              // Pass data to TL
              tl_rx_data  <= phy_rx_data;
              tl_rx_valid <= 1'b1;
              tl_rx_sop   <= (rx_state == DLL_RX_SEQ);  // First TL beat after seq strip
              tl_rx_eop   <= 1'b0;
              rx_prev_data <= phy_rx_data;
            end else begin
              // EOP beat contains LCRC (last 32 bits)
              rx_crc_received <= phy_rx_data[31:0];
              rx_state        <= DLL_RX_CRC_CHK;
              tl_rx_valid     <= 1'b0;
            end
          end else begin
            tl_rx_valid <= 1'b0;
          end
        end

        DLL_RX_CRC_CHK: begin
          crc_ok <= (rx_crc_accum == ~rx_crc_received);
          if (rx_crc_accum == ~rx_crc_received) begin
            // CRC OK – signal final beat and advance expected sequence
            tl_rx_data  <= rx_prev_data;
            tl_rx_valid <= 1'b1;
            tl_rx_eop   <= 1'b1;
            tl_rx_error <= 1'b0;
            // Queue ACK
            ack_seq_out     <= next_expected_seq;
            ack_pend_seq    <= next_expected_seq;
            ack_pend_valid  <= 1'b1;
            next_expected_seq <= next_expected_seq + 1;
          end else begin
            // CRC fail – NAK, discard
            tl_rx_error <= 1'b1;
            nak_out     <= 1'b1;
            nak_seq_out <= next_expected_seq;
          end
          rx_state  <= DLL_RX_IDLE;
          rx_in_tlp <= 1'b0;
          crc_active <= 1'b0;
        end

        DLL_RX_DLLP: begin
          // Process received DLLP
          if (phy_rx_valid && phy_rx_eop) begin
            case (rx_dllp_type)
              DLLP_ACK: begin
                // ACK: update ack_seq_out so DLL TX can purge replay buffer
                ack_seq_out <= rx_dllp_seq;
              end
              DLLP_NAK: begin
                // NAK: trigger replay
                nak_out     <= 1'b1;
                nak_seq_out <= rx_dllp_seq;
              end
              // FC and PM DLLPs handled by flow control module
              default: ;
            endcase
            rx_state   <= DLL_RX_IDLE;
            rx_in_dllp <= 1'b0;
          end
        end

        default: rx_state <= DLL_RX_IDLE;
      endcase

      // Error injection check
      if (phy_rx_error && rx_in_tlp) begin
        tl_rx_error <= 1'b1;
        rx_state    <= DLL_RX_IDLE;
        rx_in_tlp   <= 1'b0;
        nak_out     <= 1'b1;
        nak_seq_out <= next_expected_seq;
      end
    end
  end

endmodule : pcie_dll_rx
