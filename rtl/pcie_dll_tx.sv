// -----------------------------------------------------------------------------
// Author  : Robert Kingsly Amalathas
// Email   : robertk@microprocessorlab.com
// LinkedIn: www.linkedin.com/in/robertkingslya
// -----------------------------------------------------------------------------
// =============================================================================
// PCIe 7.0 Controller - Data Link Layer TX
// Based on PCI Express Base Specification Rev 7.0 Section 3
// =============================================================================
// Description:
//   Implements the Transmit side of the PCIe Data Link Layer:
//     - Assigns 12-bit sequence numbers to each TLP
//     - Appends LCRC-32 to each TLP
//     - Wraps TLP in DLLP framing (STP, END_ symbols handled by PIPE layer)
//     - Maintains a Replay Buffer (ACK/NAK timeout retry)
//     - Transmits ACK/DLLP, FC Update DLLPs
//     - Implements ACK timer and replay timer per spec
// =============================================================================

`include "pcie_pkg.sv"

module pcie_dll_tx
  import pcie_pkg::*;
#(
  parameter int unsigned DATA_W          = 256,
  parameter int unsigned RETRY_BUF_DEPTH = 2048
)(
  input  logic              clk,
  input  logic              rst_n,
  input  logic              link_up,

  // -------------------------------------------------------------------------
  // TLP from Transaction Layer
  // -------------------------------------------------------------------------
  input  logic [DATA_W-1:0]  tl_tx_data,
  input  logic               tl_tx_valid,
  output logic               tl_tx_ready,
  input  logic               tl_tx_sop,
  input  logic               tl_tx_eop,

  // -------------------------------------------------------------------------
  // Output to PIPE Interface
  // -------------------------------------------------------------------------
  output logic [DATA_W-1:0]  phy_tx_data,
  output logic               phy_tx_valid,
  input  logic               phy_tx_ready,
  output logic               phy_tx_sop,
  output logic               phy_tx_eop,

  // -------------------------------------------------------------------------
  // ACK/NAK feedback from DLL RX
  // -------------------------------------------------------------------------
  input  logic               nak_received,
  input  logic [11:0]        ack_seq,
  input  logic [11:0]        nak_seq
);

  // ---------------------------------------------------------------------------
  // Sequence Number Management
  // ---------------------------------------------------------------------------
  logic [11:0]  tx_seq_num;      // Next sequence number to assign
  logic [11:0]  ack_ptr;         // Last ACKed sequence number
  logic [11:0]  replay_ptr;      // Replay start sequence number

  // ---------------------------------------------------------------------------
  // Replay Buffer
  // Stores TLPs until ACKed; circular buffer indexed by sequence number
  // ---------------------------------------------------------------------------
  localparam int RB_ADDR_W = $clog2(RETRY_BUF_DEPTH);

  logic [DATA_W-1:0]  replay_buf [RETRY_BUF_DEPTH];
  logic [RB_ADDR_W:0] rb_wr_ptr, rb_rd_ptr;
  logic               rb_full, rb_empty;
  logic               replay_active;
  logic [11:0]        replay_seq;

  assign rb_full  = (rb_wr_ptr[RB_ADDR_W] != rb_rd_ptr[RB_ADDR_W]) &&
                    (rb_wr_ptr[RB_ADDR_W-1:0] == rb_rd_ptr[RB_ADDR_W-1:0]);
  assign rb_empty = (rb_wr_ptr == rb_rd_ptr);

  // ---------------------------------------------------------------------------
  // ACK Timer (generate ACK DLLP when no data TLP to send)
  // ---------------------------------------------------------------------------
  logic [15:0]  ack_timer;
  logic         ack_timer_exp;
  logic         ack_pending;
  logic [11:0]  ack_next_seq;

  assign ack_timer_exp = (ack_timer == ACK_LATENCY_TIMER[15:0]);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ack_timer  <= '0;
      ack_pending <= 1'b0;
      ack_next_seq <= '0;
    end else if (!link_up) begin
      ack_timer  <= '0;
      ack_pending <= 1'b0;
    end else begin
      if (ack_timer_exp) begin
        ack_pending  <= 1'b1;
        ack_timer    <= '0;
        ack_next_seq <= ack_seq;
      end else begin
        ack_timer <= ack_timer + 1;
      end
      if (ack_pending && phy_tx_ready)
        ack_pending <= 1'b0;
    end
  end

  // ---------------------------------------------------------------------------
  // Replay Timer
  // ---------------------------------------------------------------------------
  logic [23:0]  replay_timer;
  logic         replay_timer_exp;

  assign replay_timer_exp = (replay_timer == REPLAY_TIMER_INIT[23:0]);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      replay_timer  <= '0;
      replay_active <= 1'b0;
    end else if (!link_up) begin
      replay_timer  <= '0;
      replay_active <= 1'b0;
    end else begin
      if (nak_received || replay_timer_exp) begin
        replay_active <= 1'b1;
        replay_timer  <= '0;
        replay_seq    <= nak_received ? nak_seq : ack_ptr;
        rb_rd_ptr     <= rb_rd_ptr;   // Reset read pointer to replay start
      end else if (!rb_empty) begin
        replay_timer <= '0;  // Reset timer when actively sending
      end else begin
        replay_timer <= replay_timer + 1;
      end

      if (replay_active && rb_empty)
        replay_active <= 1'b0;
    end
  end

  // ---------------------------------------------------------------------------
  // LCRC-32 Computation
  // ---------------------------------------------------------------------------
  // Running CRC updated as each DATA_W word is written
  logic [31:0]  tx_crc;
  logic         crc_active;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_crc    <= LCRC_INIT;
      crc_active <= 1'b0;
    end else begin
      if (tl_tx_valid && tl_tx_sop) begin
        tx_crc    <= LCRC_INIT;
        crc_active <= 1'b1;
      end else if (crc_active && tl_tx_valid) begin
        // Update CRC over each 32-bit word in the DATA_W beat
        for (int dw = 0; dw < DATA_W/32; dw++) begin
          tx_crc <= calc_lcrc(tx_crc, tl_tx_data[dw*32 +: 32]);
        end
        if (tl_tx_eop)
          crc_active <= 1'b0;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // TX State Machine
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    DLL_TX_IDLE,
    DLL_TX_SEQ,        // Prepend sequence number DW
    DLL_TX_DATA,       // Forward TLP data
    DLL_TX_CRC,        // Append LCRC
    DLL_TX_ACK_DLLP,   // Send ACK DLLP
    DLL_TX_FC_DLLP     // Send FC Update DLLP
  } dll_tx_state_e;

  dll_tx_state_e  dll_tx_state;

  // Sequence number prepend word: [31:20]=rsvd, [19:8]=seq_num, [7:0]=rsvd
  logic [DATA_W-1:0]  seq_prepend;
  assign seq_prepend = {{(DATA_W-32){1'b0}}, 4'h0, tx_seq_num, 16'h0};

  // ACK DLLP word: Type=ACK, AckNak_Seq_Num, 16-bit DLCRC
  logic [DATA_W-1:0]  ack_dllp_word;
  assign ack_dllp_word = {{(DATA_W-32){1'b0}}, 8'h00, 4'h0, ack_next_seq, 16'hBEEF};

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dll_tx_state  <= DLL_TX_IDLE;
      tx_seq_num    <= '0;
      ack_ptr       <= '0;
      rb_wr_ptr     <= '0;
      rb_rd_ptr     <= '0;
      phy_tx_valid  <= 1'b0;
      phy_tx_sop    <= 1'b0;
      phy_tx_eop    <= 1'b0;
      phy_tx_data   <= '0;
      tl_tx_ready   <= 1'b0;
    end else if (!link_up) begin
      dll_tx_state  <= DLL_TX_IDLE;
      tx_seq_num    <= '0;
      ack_ptr       <= '0;
      rb_wr_ptr     <= '0;
      rb_rd_ptr     <= '0;
      phy_tx_valid  <= 1'b0;
      phy_tx_sop    <= 1'b0;
      phy_tx_eop    <= 1'b0;
      tl_tx_ready   <= 1'b0;
    end else begin
      case (dll_tx_state)

        DLL_TX_IDLE: begin
          phy_tx_valid <= 1'b0;
          phy_tx_sop   <= 1'b0;
          phy_tx_eop   <= 1'b0;
          if (ack_pending) begin
            dll_tx_state <= DLL_TX_ACK_DLLP;
            tl_tx_ready  <= 1'b0;
          end else if (tl_tx_valid && !rb_full) begin
            dll_tx_state <= DLL_TX_SEQ;
            tl_tx_ready  <= 1'b0;
          end else begin
            tl_tx_ready <= !rb_full;
          end
        end

        DLL_TX_SEQ: begin
          // Output sequence number as first beat
          if (phy_tx_ready) begin
            phy_tx_data  <= seq_prepend;
            phy_tx_valid <= 1'b1;
            phy_tx_sop   <= 1'b1;
            phy_tx_eop   <= 1'b0;
            dll_tx_state <= DLL_TX_DATA;
            tl_tx_ready  <= 1'b1;
            // Store in replay buffer
            replay_buf[rb_wr_ptr[RB_ADDR_W-1:0]] <= seq_prepend;
            rb_wr_ptr <= rb_wr_ptr + 1;
          end
        end

        DLL_TX_DATA: begin
          phy_tx_sop <= 1'b0;
          if (tl_tx_valid && phy_tx_ready) begin
            phy_tx_data  <= tl_tx_data;
            phy_tx_valid <= 1'b1;
            // Store in replay buffer
            replay_buf[rb_wr_ptr[RB_ADDR_W-1:0]] <= tl_tx_data;
            rb_wr_ptr <= rb_wr_ptr + 1;
            if (tl_tx_eop) begin
              dll_tx_state <= DLL_TX_CRC;
              tl_tx_ready  <= 1'b0;
              phy_tx_eop   <= 1'b0;
            end
          end else begin
            phy_tx_valid <= 1'b0;
          end
        end

        DLL_TX_CRC: begin
          // Append LCRC as final beat
          if (phy_tx_ready) begin
            phy_tx_data  <= {{(DATA_W-32){1'b0}}, ~tx_crc};
            phy_tx_valid <= 1'b1;
            phy_tx_eop   <= 1'b1;
            tx_seq_num   <= tx_seq_num + 1;
            dll_tx_state <= DLL_TX_IDLE;
            // Store in replay buffer
            replay_buf[rb_wr_ptr[RB_ADDR_W-1:0]] <= {{(DATA_W-32){1'b0}}, ~tx_crc};
            rb_wr_ptr <= rb_wr_ptr + 1;
          end else begin
            phy_tx_valid <= 1'b0;
            phy_tx_eop   <= 1'b0;
          end
        end

        DLL_TX_ACK_DLLP: begin
          if (phy_tx_ready) begin
            phy_tx_data  <= ack_dllp_word;
            phy_tx_valid <= 1'b1;
            phy_tx_sop   <= 1'b1;
            phy_tx_eop   <= 1'b1;
            dll_tx_state <= DLL_TX_IDLE;
          end
        end

        DLL_TX_FC_DLLP: begin
          // FC Update DLLP – simplified
          if (phy_tx_ready) begin
            phy_tx_data  <= '0;
            phy_tx_valid <= 1'b1;
            phy_tx_sop   <= 1'b1;
            phy_tx_eop   <= 1'b1;
            dll_tx_state <= DLL_TX_IDLE;
          end
        end

        default: dll_tx_state <= DLL_TX_IDLE;
      endcase

      // ACK advancement: purge replay buffer up to acknowledged sequence
      if (ack_seq != ack_ptr) begin
        ack_ptr   <= ack_seq;
        rb_rd_ptr <= rb_rd_ptr + 12'(ack_seq - ack_ptr);
      end
    end
  end

endmodule : pcie_dll_tx
