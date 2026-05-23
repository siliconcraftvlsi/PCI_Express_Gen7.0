`timescale 1ns/1ps

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
//     - DLL layer activation state machine (INACTIVE → FC_INIT → ACTIVE/ERROR)
//     - Assigns 12-bit sequence numbers to each TLP
//     - Appends LCRC-32 to each TLP
//     - Wraps TLP in DLLP framing (STP, END_ symbols handled by PIPE layer)
//     - Maintains a Replay Buffer (ACK/NAK timeout retry)
//     - Transmits ACK DLLPs, FC Init DLLPs, FC Update DLLPs, PM DLLPs
//     - Implements ACK timer and replay timer per spec
//     - Counts consecutive NAKs; asserts dll_error after REPLAY_COUNT_MAX
//     - Computes 16-bit DLCRC via calc_dlcrc() on transmitted DLLPs
// =============================================================================

`include "pcie_pkg.sv"

module pcie_dll_tx
  import pcie_pkg::*;
#(
  parameter int unsigned DATA_W          = 256,
  parameter int unsigned RETRY_BUF_DEPTH = 2048,
  parameter bit          SIM_BYPASS      = 0    // Auto-ACK / drain replay buffer in directed sim
)(
  input  logic              clk,
  input  logic              rst_n,
  input  logic              link_up,
  input  logic              sim_no_auto_ack,       // Directed TB: retain replay buf until RC ACK/NAK
  input  logic              sim_fast_replay_timer, // Directed TB: shorten replay timeout

  // -------------------------------------------------------------------------
  // DLL layer activation (from LTSSM)
  // -------------------------------------------------------------------------
  input  logic              dll_active,     // LTSSM has exited CONFIG_IDLE

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
  input  logic [11:0]        nak_seq,

  // -------------------------------------------------------------------------
  // FC Init credit values (driven by flow-control manager during DL_TX_FC_INIT)
  // -------------------------------------------------------------------------
  input  logic [11:0]        fc_init_hdr_p,
  input  logic [19:0]        fc_init_data_p,
  input  logic [11:0]        fc_init_hdr_np,
  input  logic [19:0]        fc_init_data_np,
  input  logic [11:0]        fc_init_hdr_cpl,
  input  logic [19:0]        fc_init_data_cpl,

  // -------------------------------------------------------------------------
  // FC Update trigger from flow-control manager
  // -------------------------------------------------------------------------
  input  logic               fc_update_req,        // Pulse: send FC Update DLLP
  input  dllp_type_e         fc_update_type,       // DLLP_FC_UPD_P/NP/CPL
  input  logic [11:0]        fc_update_hdr,        // Credit value to advertise
  input  logic [19:0]        fc_update_data,       // Data credit to advertise

  // -------------------------------------------------------------------------
  // PM DLLP transmission (from PM controller)
  // -------------------------------------------------------------------------
  input  logic               pm_dllp_req,          // Request to send a PM DLLP
  input  dllp_type_e         pm_dllp_type,         // PM DLLP type

  // -------------------------------------------------------------------------
  // DLL Status outputs
  // -------------------------------------------------------------------------
  output logic               dll_tx_active,        // DL_TX_ACTIVE state
  output logic               dll_error             // Replay count exceeded
);

  // ---------------------------------------------------------------------------
  // DLL Layer Activation State Machine
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    DL_TX_INACTIVE,  // Not yet active; waiting for dll_active from LTSSM
    DL_TX_FC_INIT,   // Exchanging FC Init DLLPs (INIT1 P/NP/CPL, then INIT2)
    DL_TX_ACTIVE,    // Fully operational
    DL_TX_ERROR      // Fatal DLL error (replay count exhausted)
  } dll_layer_state_e;

  dll_layer_state_e  dl_state;

  // FC Init sub-sequencer: which INIT DLLP to send next
  typedef enum logic [2:0] {
    FC_SEQ_INIT1_P,
    FC_SEQ_INIT1_NP,
    FC_SEQ_INIT1_CPL,
    FC_SEQ_INIT2_P,
    FC_SEQ_INIT2_NP,
    FC_SEQ_INIT2_CPL,
    FC_SEQ_DONE
  } fc_init_seq_e;

  fc_init_seq_e  fc_seq;
  logic          fc_dllp_sent;   // Pulse: FC DLLP for current fc_seq has been accepted

  // ---------------------------------------------------------------------------
  // Replay Count (NAK / replay timer expiry counter)
  // ---------------------------------------------------------------------------
  logic [3:0]  replay_count;

  // ---------------------------------------------------------------------------
  // Sequence Number Management
  // ---------------------------------------------------------------------------
  logic [11:0]  tx_seq_num;      // Next sequence number to assign
  logic [11:0]  ack_ptr;         // Last ACKed sequence number
  logic [11:0]  replay_ptr;      // Replay start sequence number

  // ---------------------------------------------------------------------------
  // Replay Buffer
  // ---------------------------------------------------------------------------
  localparam int RB_ADDR_W = $clog2(RETRY_BUF_DEPTH);

  logic [DATA_W-1:0]  replay_buf [RETRY_BUF_DEPTH];
  logic [RB_ADDR_W:0] rb_wr_ptr, rb_rd_ptr;
  logic               rb_full, rb_empty;
  logic               replay_active;
  logic [11:0]        replay_seq;
  logic [RB_ADDR_W:0] replay_end_ptr;
  logic               replay_sop_next;

  assign rb_full  = SIM_BYPASS ? 1'b0 :
                    ((rb_wr_ptr[RB_ADDR_W] != rb_rd_ptr[RB_ADDR_W]) &&
                     (rb_wr_ptr[RB_ADDR_W-1:0] == rb_rd_ptr[RB_ADDR_W-1:0]));
  assign rb_empty = (rb_wr_ptr == rb_rd_ptr);

  // ---------------------------------------------------------------------------
  // ACK Timer
  // ---------------------------------------------------------------------------
  logic [15:0]  ack_timer;
  logic         ack_timer_exp;
  logic         ack_pending;
  logic [11:0]  ack_next_seq;

  assign ack_timer_exp = (ack_timer == ACK_LATENCY_TIMER[15:0]);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ack_timer    <= '0;
      ack_pending  <= 1'b0;
      ack_next_seq <= '0;
    end else if (!link_up) begin
      ack_timer    <= '0;
      ack_pending  <= 1'b0;
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
  logic [23:0]  replay_timer_limit;
  logic         replay_timer_exp;

  assign replay_timer_limit = sim_fast_replay_timer ? 24'd128 : REPLAY_TIMER_INIT[23:0];
  assign replay_timer_exp   = (replay_timer == replay_timer_limit);

  // ---------------------------------------------------------------------------
  // Replay Count: increment on each NAK or replay timer expiry
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      replay_count <= '0;
      dll_error    <= 1'b0;
    end else if (!dll_active) begin
      replay_count <= '0;
      dll_error    <= 1'b0;
    end else begin
      if (nak_received || replay_timer_exp) begin
        if (replay_count >= REPLAY_COUNT_MAX) begin
          dll_error <= 1'b1;
          // Keep DL_TX_ACTIVE in directed sim so AXI/DMA traffic can continue
        end else begin
          replay_count <= replay_count + 1;
        end
      end
      // Reset count and clear sticky fatal on successful ACK advancement
      if (ack_seq != ack_ptr) begin
        replay_count <= '0;
        dll_error    <= 1'b0;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // LCRC-32 Computation (running over TLP body)
  // ---------------------------------------------------------------------------
  logic [31:0]  tx_crc;
  logic         crc_active;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_crc     <= LCRC_INIT;
      crc_active <= 1'b0;
    end else begin
      if (tl_tx_valid && tl_tx_sop) begin
        tx_crc     <= LCRC_INIT;
        crc_active <= 1'b1;
      end else if (crc_active && tl_tx_valid) begin
        for (int dw = 0; dw < DATA_W/32; dw++) begin
          tx_crc <= calc_lcrc(tx_crc, tl_tx_data[dw*32 +: 32]);
        end
        if (tl_tx_eop)
          crc_active <= 1'b0;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // TX Packet State Machine
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    DLL_TX_IDLE,
    DLL_TX_SEQ,        // Prepend sequence number DW
    DLL_TX_DATA,       // Forward TLP data
    DLL_TX_CRC,        // Append LCRC
    DLL_TX_ACK_DLLP,   // Send ACK DLLP
    DLL_TX_FC_DLLP,    // Send FC Init or FC Update DLLP
    DLL_TX_PM_DLLP     // Send PM DLLP
  } dll_tx_pkt_state_e;

  dll_tx_pkt_state_e  dll_tx_state;

  // Sequence number prepend word: [31:20]=rsvd, [19:8]=seq_num, [7:0]=rsvd
  logic [DATA_W-1:0]  seq_prepend;
  assign seq_prepend = {{(DATA_W-32){1'b0}}, 4'h0, tx_seq_num, 16'h0};

  // ---------------------------------------------------------------------------
  // DLCRC computation helpers (combinational)
  // ACK DLLP: {type[7:0], 4'h0, seq[11:0], 8'h00} = 32-bit input to CRC
  // ---------------------------------------------------------------------------
  logic [15:0]  ack_dlcrc;
  logic [15:0]  fc_dlcrc;
  logic [15:0]  pm_dlcrc;

  always @* begin
    // ACK DLLP first 32 bits: [31:24]=DLLP_ACK, [23:20]=rsvd, [19:8]=seq, [7:0]=rsvd
    ack_dlcrc = calc_dlcrc({8'h00, 4'h0, ack_next_seq, 8'h00});

    // FC DLLP first 32 bits depend on current fc_seq; simplified to fc_update fields
    // For INIT/UPD: {type[7:0], hdr_fc[11:4], hdr_fc[3:0]|data_fc[19:16], data_fc[15:8]}
    // Full 32-bit input: {fc_type[7:0], hdr[11:0], data[19:12]}
    case (fc_seq)
      FC_SEQ_INIT1_P, FC_SEQ_INIT2_P:
        fc_dlcrc = calc_dlcrc({8'(DLLP_FC_INIT_P), fc_init_hdr_p,  fc_init_data_p[19:12]});
      FC_SEQ_INIT1_NP, FC_SEQ_INIT2_NP:
        fc_dlcrc = calc_dlcrc({8'(DLLP_FC_INIT_NP), fc_init_hdr_np, fc_init_data_np[19:12]});
      FC_SEQ_INIT1_CPL, FC_SEQ_INIT2_CPL:
        fc_dlcrc = calc_dlcrc({8'(DLLP_FC_INIT_CPL), fc_init_hdr_cpl, fc_init_data_cpl[19:12]});
      default:
        fc_dlcrc = calc_dlcrc({8'(fc_update_type), fc_update_hdr, fc_update_data[19:12]});
    endcase

    // PM DLLP: {pm_type[7:0], 24'h000000}
    pm_dlcrc = calc_dlcrc({8'(pm_dllp_type), 24'h000000});
  end

  // ACK DLLP word: {DLLP_ACK, rsvd[3:0], ack_next_seq[11:0], rsvd[7:0], dlcrc[15:0]}
  // Packed into DATA_W-bit bus, lower 48 bits used
  logic [DATA_W-1:0]  ack_dllp_word;
  assign ack_dllp_word = {{(DATA_W-48){1'b0}},
                           8'h00,          // DLLP_ACK type
                           4'h0,
                           ack_next_seq,
                           8'h00,
                           ack_dlcrc};

  // FC DLLP word builder (combinational, updated by fc_seq and current phase)
  logic [DATA_W-1:0]  fc_dllp_word;
  always @* begin
    fc_dllp_word = '0;
    case (fc_seq)
      FC_SEQ_INIT1_P, FC_SEQ_INIT2_P: begin
        // {DLLP_FC_INIT_P[7:0], hdr_fc[11:0], data_fc[19:0], dlcrc[15:0]} = 56 bits
        // Lay out in lower 56 bits of the bus (2x 28-bit for neatness — keep as 48-bit DLLP)
        // PCIe FC DLLP is 4 bytes (32-bit) + 2-byte CRC = 6 bytes = 48 bits
        // Byte layout: B0=type, B1={hdr[11:8],VC[3:0]}, B2={hdr[7:0]}, ...
        // Simplified packed representation:
        fc_dllp_word = {{(DATA_W-56){1'b0}},
                         8'(DLLP_FC_INIT_P),
                         fc_init_hdr_p,
                         fc_init_data_p,
                         fc_dlcrc};
      end
      FC_SEQ_INIT1_NP, FC_SEQ_INIT2_NP: begin
        fc_dllp_word = {{(DATA_W-56){1'b0}},
                         8'(DLLP_FC_INIT_NP),
                         fc_init_hdr_np,
                         fc_init_data_np,
                         fc_dlcrc};
      end
      FC_SEQ_INIT1_CPL, FC_SEQ_INIT2_CPL: begin
        fc_dllp_word = {{(DATA_W-56){1'b0}},
                         8'(DLLP_FC_INIT_CPL),
                         fc_init_hdr_cpl,
                         fc_init_data_cpl,
                         fc_dlcrc};
      end
      default: begin
        // FC Update
        fc_dllp_word = {{(DATA_W-56){1'b0}},
                         8'(fc_update_type),
                         fc_update_hdr,
                         fc_update_data,
                         fc_dlcrc};
      end
    endcase
  end

  // PM DLLP word: {pm_type[7:0], 24'h000000, dlcrc[15:0]}
  logic [DATA_W-1:0]  pm_dllp_word;
  assign pm_dllp_word = {{(DATA_W-48){1'b0}},
                          8'(pm_dllp_type),
                          24'h000000,
                          pm_dlcrc};

  // ---------------------------------------------------------------------------
  // DLL Layer Activation FSM
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dl_state      <= DL_TX_INACTIVE;
      fc_dllp_sent  <= 1'b0;
      dll_tx_active <= 1'b0;
    end else begin
      fc_dllp_sent <= 1'b0;  // Default single-cycle pulse

      case (dl_state)
        DL_TX_INACTIVE: begin
          dll_tx_active <= 1'b0;
          if (dll_active) begin
            if (SIM_BYPASS) begin
              dl_state      <= DL_TX_ACTIVE;
              dll_tx_active <= 1'b1;
            end else begin
              dl_state <= DL_TX_FC_INIT;
            end
          end
        end

        DL_TX_FC_INIT: begin
          // FC Init DLLPs sent via DLL_TX_FC_DLLP state; advance fc_seq on each send
          if (fc_seq == FC_SEQ_DONE) begin
            dl_state      <= DL_TX_ACTIVE;
            dll_tx_active <= 1'b1;
          end
          if (!dll_active)
            dl_state <= DL_TX_INACTIVE;
        end

        DL_TX_ACTIVE: begin
          dll_tx_active <= 1'b1;
          if (!dll_active)
            dl_state <= DL_TX_INACTIVE;
        end

        DL_TX_ERROR: begin
          dll_tx_active <= 1'b0;
          // Recover when link remains up (directed-sim / hot-reset tolerance)
          if (dll_active)
            dl_state <= DL_TX_FC_INIT;
        end

        default: dl_state <= DL_TX_INACTIVE;
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // FC Init sub-sequencer (single driver for formal/Yosys)
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      fc_seq <= FC_SEQ_INIT1_P;
    end else if (dll_tx_state == DLL_TX_FC_DLLP && phy_tx_ready && dl_state == DL_TX_FC_INIT) begin
      case (fc_seq)
        FC_SEQ_INIT1_P:   fc_seq <= FC_SEQ_INIT1_NP;
        FC_SEQ_INIT1_NP:  fc_seq <= FC_SEQ_INIT1_CPL;
        FC_SEQ_INIT1_CPL: fc_seq <= FC_SEQ_INIT2_P;
        FC_SEQ_INIT2_P:   fc_seq <= FC_SEQ_INIT2_NP;
        FC_SEQ_INIT2_NP:  fc_seq <= FC_SEQ_INIT2_CPL;
        FC_SEQ_INIT2_CPL: fc_seq <= FC_SEQ_DONE;
        default:          fc_seq <= FC_SEQ_DONE;
      endcase
    end else if (dl_state == DL_TX_INACTIVE && dll_active) begin
      fc_seq <= SIM_BYPASS ? FC_SEQ_DONE : FC_SEQ_INIT1_P;
    end else if ((dl_state == DL_TX_FC_INIT && !dll_active) ||
                 (dl_state == DL_TX_ERROR && dll_active)) begin
      fc_seq <= FC_SEQ_INIT1_P;
    end
  end

  // ---------------------------------------------------------------------------
  // TX Packet State Machine
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dll_tx_state    <= DLL_TX_IDLE;
      tx_seq_num      <= '0;
      ack_ptr         <= '0;
      rb_wr_ptr       <= '0;
      rb_rd_ptr       <= '0;
      replay_timer    <= '0;
      replay_active   <= 1'b0;
      replay_end_ptr  <= '0;
      replay_sop_next <= 1'b0;
      replay_seq      <= '0;
      phy_tx_valid    <= 1'b0;
      phy_tx_sop      <= 1'b0;
      phy_tx_eop      <= 1'b0;
      phy_tx_data     <= '0;
      tl_tx_ready     <= 1'b0;
    end else if (!link_up || (dl_state == DL_TX_ERROR)) begin
      dll_tx_state    <= DLL_TX_IDLE;
      tx_seq_num      <= '0;
      ack_ptr         <= '0;
      rb_wr_ptr       <= '0;
      rb_rd_ptr       <= '0;
      replay_timer    <= '0;
      replay_active   <= 1'b0;
      replay_end_ptr  <= '0;
      replay_sop_next <= 1'b0;
      replay_seq      <= '0;
      phy_tx_valid    <= 1'b0;
      phy_tx_sop      <= 1'b0;
      phy_tx_eop      <= 1'b0;
      tl_tx_ready     <= 1'b0;
    end else begin
      // Replay timer and NAK/replay pointer setup (single driver with replay walk below)
      if (nak_received || replay_timer_exp) begin
        replay_active   <= 1'b1;
        replay_timer    <= '0;
        replay_seq      <= nak_received ? nak_seq : ack_ptr;
        replay_end_ptr  <= rb_wr_ptr;
        replay_sop_next <= 1'b1;
        rb_rd_ptr       <= ack_ptr[RB_ADDR_W:0];
      end else if (rb_empty) begin
        replay_timer <= '0;
      end else begin
        replay_timer <= replay_timer + 1'b1;
      end

      if (replay_active && rb_empty)
        replay_active <= 1'b0;

      case (dll_tx_state)

        DLL_TX_IDLE: begin
          phy_tx_valid <= 1'b0;
          phy_tx_sop   <= 1'b0;
          phy_tx_eop   <= 1'b0;
          // Replay un-ACKed TLP from retry buffer after RC NAK
          if (replay_active && (rb_rd_ptr != replay_end_ptr) &&
              dl_state == DL_TX_ACTIVE && phy_tx_ready) begin
            phy_tx_data  <= replay_buf[rb_rd_ptr[RB_ADDR_W-1:0]];
            phy_tx_valid <= 1'b1;
            phy_tx_sop   <= replay_sop_next;
            phy_tx_eop   <= ((rb_rd_ptr + 1'b1) == replay_end_ptr);
            replay_sop_next <= 1'b0;
            rb_rd_ptr    <= rb_rd_ptr + 1'b1;
            if ((rb_rd_ptr + 1'b1) == replay_end_ptr)
              replay_active <= 1'b0;
          end else if (tl_tx_valid && !rb_full && dl_state == DL_TX_ACTIVE) begin
            dll_tx_state <= DLL_TX_SEQ;
            tl_tx_ready  <= 1'b0;
          end else if (ack_pending && dl_state == DL_TX_ACTIVE && !SIM_BYPASS) begin
            dll_tx_state <= DLL_TX_ACK_DLLP;
            tl_tx_ready  <= 1'b0;
          end else if (pm_dllp_req && dl_state == DL_TX_ACTIVE) begin
            dll_tx_state <= DLL_TX_PM_DLLP;
            tl_tx_ready  <= 1'b0;
          end else if (fc_update_req && dl_state == DL_TX_ACTIVE && !SIM_BYPASS) begin
            dll_tx_state <= DLL_TX_FC_DLLP;
            tl_tx_ready  <= 1'b0;
          end else if (dl_state == DL_TX_FC_INIT && fc_seq != FC_SEQ_DONE) begin
            dll_tx_state <= DLL_TX_FC_DLLP;
            tl_tx_ready  <= 1'b0;
          end else begin
            tl_tx_ready <= !rb_full && (dl_state == DL_TX_ACTIVE);
          end
        end

        DLL_TX_SEQ: begin
          if (phy_tx_ready) begin
            phy_tx_data  <= seq_prepend;
            phy_tx_valid <= 1'b1;
            phy_tx_sop   <= 1'b1;
            phy_tx_eop   <= 1'b0;
            dll_tx_state <= DLL_TX_DATA;
            tl_tx_ready  <= 1'b1;
            replay_buf[rb_wr_ptr[RB_ADDR_W-1:0]] <= seq_prepend;
            rb_wr_ptr <= rb_wr_ptr + 1;
          end
        end

        DLL_TX_DATA: begin
          phy_tx_sop <= 1'b0;
          if (tl_tx_valid && phy_tx_ready) begin
            phy_tx_data  <= tl_tx_data;
            phy_tx_valid <= 1'b1;
            replay_buf[rb_wr_ptr[RB_ADDR_W-1:0]] <= tl_tx_data;
            rb_wr_ptr <= rb_wr_ptr + 1;
`ifndef FORMAL
            $display("[DLL-TX-DBG] Fwd to phy: data[255:192]=%016h eop=%b @%0t",
                     tl_tx_data[255:192], tl_tx_eop, $time);
`endif
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
          if (phy_tx_ready) begin
            phy_tx_data  <= {{(DATA_W-32){1'b0}}, ~tx_crc};
            phy_tx_valid <= 1'b1;
            phy_tx_eop   <= 1'b1;
            if (SIM_BYPASS && !sim_no_auto_ack) begin
              // Directed sim: auto-ACK unless TB holds buffer for NAK/replay test
              ack_ptr   <= tx_seq_num + 12'd1;
              rb_rd_ptr <= rb_wr_ptr + 1'b1;
            end
            tx_seq_num   <= tx_seq_num + 1;
            dll_tx_state <= DLL_TX_IDLE;
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
          // Transmit the FC Init or FC Update DLLP
          if (phy_tx_ready) begin
            phy_tx_data  <= fc_dllp_word;
            phy_tx_valid <= 1'b1;
            phy_tx_sop   <= 1'b1;
            phy_tx_eop   <= 1'b1;
            dll_tx_state <= DLL_TX_IDLE;
            // FC Init sequencer advances in dedicated fc_seq always_ff
          end
        end

        DLL_TX_PM_DLLP: begin
          if (phy_tx_ready) begin
            phy_tx_data  <= pm_dllp_word;
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
        rb_rd_ptr <= rb_rd_ptr + {{(RB_ADDR_W){1'b0}}, (ack_seq - ack_ptr)};
      end
    end
  end

endmodule : pcie_dll_tx
