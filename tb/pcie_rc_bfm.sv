`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// Author  : Robert Kingsly Amalathas
// Email   : robertk@microprocessorlab.com
// LinkedIn: www.linkedin.com/in/robertkingslya
// -----------------------------------------------------------------------------
// =============================================================================
// PCIe 7.0 Controller - PCIe Root Complex BFM
// =============================================================================
// Description:
//   Root Complex (RC) side Bus Functional Model for testing the PCIe controller
//   in Endpoint mode. Simulates the remote link partner by:
//     - Providing a simulated PIPE interface (RX/TX data)
//     - Running the link training sequence (TS1/TS2 exchange with LTSSM)
//     - Detecting MRd TLPs from DUT TX and injecting CplD responses
//     - Reporting received TLPs for scoreboard checking
// =============================================================================

`ifndef PCIE_RC_BFM_SV
`define PCIE_RC_BFM_SV

`include "../rtl/pcie_pkg.sv"

module pcie_rc_bfm
  import pcie_pkg::*;
#(
  parameter int unsigned NUM_LANES = 16,
  parameter int unsigned PIPE_W    = 32,
  parameter int unsigned DATA_W    = 256
)(
  input  logic              clk,
  input  logic              rst_n,

  // -------------------------------------------------------------------------
  // PIPE Interface (from RC side, wired to DUT PIPE signals)
  // -------------------------------------------------------------------------
  // RC transmits → DUT RX
  output logic [NUM_LANES-1:0][PIPE_W-1:0]   rc_tx_data,
  output logic [NUM_LANES-1:0][PIPE_W/8-1:0] rc_tx_datak,
  output logic [NUM_LANES-1:0]               rc_tx_valid,
  output logic [NUM_LANES-1:0]               rc_tx_elec_idle,
  output logic [NUM_LANES-1:0][2:0]          rc_tx_status,
  output logic [NUM_LANES-1:0]               rc_tx_status_valid,

  // DUT transmits → RC RX
  input  logic [NUM_LANES-1:0][PIPE_W-1:0]   dut_tx_data,
  input  logic [NUM_LANES-1:0][PIPE_W/8-1:0] dut_tx_datak,
  input  logic [NUM_LANES-1:0]               dut_tx_elec_idle,

  // -------------------------------------------------------------------------
  // DUT LTSSM state (for reactive training sequence)
  // -------------------------------------------------------------------------
  input  ltssm_state_e      ltssm_state,

  // -------------------------------------------------------------------------
  // Status and scoreboard
  // -------------------------------------------------------------------------
  output logic              link_established,
  output logic [31:0]       tlp_rx_count,     // TLPs received from DUT
  output logic [31:0]       cpl_tx_count,     // Completions sent to DUT
  output logic              bfm_error,

  // -------------------------------------------------------------------------
  // Direct DMA tag input (bypasses PIPE decoding; driven from DUT hierarchy)
  // -------------------------------------------------------------------------
  input  logic [9:0]        dma_mrd_tag,      // DMA engine's current tag
  input  logic              dma_h2d_wait,     // High while DMA waits for host CplD

  // When pcie_pipe_if uses the TX gearbox (SIM_BYPASS_PIPE=0), only sample STP on
  // phase 0 so partial PIPE cycles do not flood false MRd/CplD injects.
  input  bit                gearbox_snoop_en,
  input  logic [2:0]        dut_tx_phase,
  input  logic [11:0]       dll_rx_next_seq    // Mirror DUT DLL RX next_expected_seq
);

  // When low, RC BFM does not auto-inject CplD for captured MRd (timeout tests)
  bit auto_cpld_en = 1'b1;

  // ---------------------------------------------------------------------------
  // PIPE K-code definitions
  // ---------------------------------------------------------------------------
  localparam logic [7:0] COM    = 8'hBC;
  localparam logic [7:0] STP    = 8'hFB;
  localparam logic [7:0] END_   = 8'hFD;
  localparam logic [7:0] IDL    = 8'h7C;
  localparam logic [7:0] TS1_ID = 8'h4A;
  localparam logic [7:0] TS2_ID = 8'h45;

  // ---------------------------------------------------------------------------
  // CplD Injection State Machine
  // ---------------------------------------------------------------------------
  // Gearbox ratio 2: accumulate {lane3..lane0} into 256-bit words (matches pipe_if).
  // TLP header layout (MSB-first): DW0[255:224], DW1[223:192], DW2[191:160]
  //   MRd tag = DW1[207:198]; MWr has fmt[2]=1 → DW0[255]=1
  // ---------------------------------------------------------------------------
  localparam int unsigned GB_RATIO    = DATA_W / (NUM_LANES * PIPE_W);
  localparam int unsigned TOTAL_PIPE_W = NUM_LANES * PIPE_W;

  typedef enum logic [1:0] {
    DLLP_INJ_IDLE,
    DLLP_INJ_P0,
    DLLP_INJ_P1
  } dllp_inj_sm_e;

  typedef enum logic [4:0] {
    CPL_IDLE,
    CPL_ACC,     // accumulate 256-bit words from PIPE
    CPL_HDR,     // parse TLP header word
    CPL_SKIP,    // skip LCRC/EOP PIPE cycles (lane-at-a-time inject)
    CPL_INJ_A1,
    CPL_INJ_A2,
    CPL_INJ_B1,
    CPL_INJ_B2,
    CPL_INJ_C1,
    CPL_INJ_C2,
    // Gearbox-aligned inject: STP + 3 DLL beats × GB_RATIO PIPE cycles
    CPL_GBX_STP,
    CPL_GBX_B0_P0,
    CPL_GBX_B0_P1,
    CPL_GBX_B1_P0,
    CPL_GBX_B1_P1,
    CPL_GBX_B2_P0,
    CPL_GBX_B2_P1
  } cpl_sm_e;

  cpl_sm_e              cpl_state;
  logic [2:0]           gb_half;
  logic                 acc_word1;    // 0=seq word, 1=TLP header word
  logic [DATA_W-1:0]    rx_accum;
  logic [DATA_W-1:0]    hdr_word;
  logic [9:0]           cap_tag;
  logic [7:0]           cpl_seq_num;
  logic [31:0]          cpld_dw0, cpld_dw1, cpld_dw2;
  logic [31:0]          cpld_dw3, cpld_dw4, cpld_dw5, cpld_dw6, cpld_dw7;
  logic [3:0]           skip_half;
  logic [15:0]          cpl_watchdog;
  logic [15:0]          cpl_reply_cooldown;
  logic                 dma_h2d_wait_prev;
  logic [DATA_W-1:0]    inj_beat0;
  logic [DATA_W-1:0]    inj_beat1;
  logic [DATA_W-1:0]    inj_beat2;
  logic                 inj_corrupt_lcrc;
  logic [31:0]          cpld_lcrc_dw0;
  dllp_inj_sm_e         dllp_inj_state;
  logic [DATA_W-1:0]    dllp_inj_word;

  function automatic logic [DATA_W/8-1:0] lane_byte0(input logic [PIPE_W-1:0] lane);
    lane_byte0 = lane[7:0];
  endfunction

  // LCRC over CplD payload beats (matches CPL_INJ_B1/B2 lane packing)
  function automatic logic [31:0] calc_cpld_body_crc(
    input logic [31:0] dw0, dw1, dw2
  );
    logic [31:0] crc;
    crc = LCRC_INIT;
    crc = calc_lcrc(crc, dw0);
    crc = calc_lcrc(crc, dw1);
    crc = calc_lcrc(crc, dw2);
    crc = calc_lcrc(crc, 32'hDEAD_C0DE);
    crc = calc_lcrc(crc, 32'h1234_5678);
    crc = calc_lcrc(crc, 32'hABCD_EF01);
    crc = calc_lcrc(crc, 32'hCAFE_BABE);
    crc = calc_lcrc(crc, 32'h0000_0001);
    calc_cpld_body_crc = crc;
  endfunction

  function automatic logic [31:0] eop_dw0_with_end(input logic [31:0] lcrc_upper);
    eop_dw0_with_end = {lcrc_upper[31:8], END_};
  endfunction

  // Solve LCRC upper 24 bits (byte0 reserved for END K-char on PIPE)
  function automatic logic [31:0] solve_eop_lcrc_dw0(
    input logic [31:0] crc_after_body,
    input logic [DATA_W-1:0] eop_beat
  );
    logic [31:0] accum, R;
    integer      iter, dw;
    R = 32'h0;
    for (iter = 0; iter < 4; iter = iter + 1) begin
      accum = crc_after_body;
      for (dw = 0; dw < DATA_W/32; dw = dw + 1) begin
        logic [31:0] d;
        d = (dw == 0) ? eop_dw0_with_end(R) : eop_beat[dw*32 +: 32];
        accum = calc_lcrc(accum, d);
      end
      R = ~accum;
    end
    solve_eop_lcrc_dw0 = R;
  endfunction

  function automatic logic [DATA_W-1:0] cpld_eop_beat_template;
    cpld_eop_beat_template = {
      32'h0, 32'h0, 32'h0, 32'h0,
      32'h0, 32'h0, 32'h0, 32'h0000_00FD
    };
  endfunction

  function automatic logic [DATA_W-1:0] build_nak_dllp_word(input logic [11:0] seq);
    logic [15:0] dcrc;
    dcrc = calc_dlcrc({8'(DLLP_NAK), 4'h0, seq, 8'h00});
    build_nak_dllp_word = {{(DATA_W-48){1'b0}},
                           8'(DLLP_NAK), 4'h0, seq, 8'h00, dcrc};
  endfunction

  function automatic logic [DATA_W-1:0] build_ack_dllp_word(input logic [11:0] seq);
    logic [15:0] dcrc;
    dcrc = calc_dlcrc({8'(DLLP_ACK), 4'h0, seq, 8'h00});
    build_ack_dllp_word = {{(DATA_W-48){1'b0}},
                           8'(DLLP_ACK), 4'h0, seq, 8'h00, dcrc};
  endfunction

  function automatic logic [DATA_W-1:0] build_pm_req_ack_dllp_word;
    logic [15:0] dcrc;
    dcrc = calc_dlcrc({8'(DLLP_PM_REQ_ACK), 20'h0});
    build_pm_req_ack_dllp_word = {{(DATA_W-48){1'b0}},
                                  8'(DLLP_PM_REQ_ACK), 20'h0, dcrc};
  endfunction

  wire pipe_has_stp_raw = (dut_tx_datak[0][0] && (lane_byte0(dut_tx_data[0]) == STP)) |
                          (NUM_LANES > 1 && dut_tx_datak[1][0] && (lane_byte0(dut_tx_data[1]) == STP)) |
                          (NUM_LANES > 2 && dut_tx_datak[2][0] && (lane_byte0(dut_tx_data[2]) == STP)) |
                          (NUM_LANES > 3 && dut_tx_datak[3][0] && (lane_byte0(dut_tx_data[3]) == STP));
  wire pipe_has_stp     = pipe_has_stp_raw &&
                          (!gearbox_snoop_en || (dut_tx_phase == 3'd0));

  wire cpl_gbx_injecting = gearbox_snoop_en &&
      (cpl_state == CPL_GBX_STP ||
       cpl_state == CPL_GBX_B0_P0 || cpl_state == CPL_GBX_B0_P1 ||
       cpl_state == CPL_GBX_B1_P0 || cpl_state == CPL_GBX_B1_P1 ||
       cpl_state == CPL_GBX_B2_P0 || cpl_state == CPL_GBX_B2_P1);
  wire cpl_lane_injecting = !gearbox_snoop_en &&
      (cpl_state == CPL_INJ_A1 || cpl_state == CPL_INJ_A2 ||
       cpl_state == CPL_INJ_B1 || cpl_state == CPL_INJ_B2 ||
       cpl_state == CPL_INJ_C1 || cpl_state == CPL_INJ_C2);
  wire cpl_injecting = cpl_gbx_injecting || cpl_lane_injecting;
  wire dllp_inj_active = (dllp_inj_state != DLLP_INJ_IDLE);

  function automatic logic [PIPE_W-1:0] inj_lane_word(
    input logic [DATA_W-1:0] beat,
    input int unsigned       phase,
    input int unsigned       lane
  );
    logic [TOTAL_PIPE_W-1:0] chunk;
    chunk = beat[DATA_W-1 - phase*TOTAL_PIPE_W -: TOTAL_PIPE_W];
    inj_lane_word = chunk[lane*PIPE_W +: PIPE_W];
  endfunction

  assign inj_beat0 = {{(DATA_W-32){1'b0}}, 4'h0, dll_rx_next_seq, 16'h0};
  assign inj_beat1 = {cpld_dw0, cpld_dw1, cpld_dw2, 32'hDEAD_C0DE};
  assign inj_beat2 = {192'h0, inj_corrupt_lcrc ? eop_dw0_with_end(32'hBADFFC05) :
                                         eop_dw0_with_end(cpld_lcrc_next)};

  logic [DATA_W-1:0] gbx_beat_mux;
  logic [2:0]        gbx_phase_mux;
  always_comb begin
    gbx_beat_mux  = inj_beat0;
    gbx_phase_mux = 3'd0;
    unique case (cpl_state)
      CPL_GBX_B0_P0: begin gbx_beat_mux = inj_beat0; gbx_phase_mux = 3'd0; end
      CPL_GBX_B0_P1: begin gbx_beat_mux = inj_beat0; gbx_phase_mux = 3'd1; end
      CPL_GBX_B1_P0: begin gbx_beat_mux = inj_beat1; gbx_phase_mux = 3'd0; end
      CPL_GBX_B1_P1: begin gbx_beat_mux = inj_beat1; gbx_phase_mux = 3'd1; end
      CPL_GBX_B2_P0: begin gbx_beat_mux = inj_beat2; gbx_phase_mux = 3'd0; end
      CPL_GBX_B2_P1: begin gbx_beat_mux = inj_beat2; gbx_phase_mux = 3'd1; end
      default:       ;
    endcase
  end

  logic [31:0] cpld_lcrc_next;
  always_comb begin
    if (inj_corrupt_lcrc)
      cpld_lcrc_next = 32'hBADFFC05;
    else
      cpld_lcrc_next = solve_eop_lcrc_dw0(
        calc_cpld_body_crc(cpld_dw0, cpld_dw1, cpld_dw2),
        cpld_eop_beat_template());
  end

  // ---------------------------------------------------------------------------
  // Combined always_ff: link training outputs + CplD injection
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      link_established <= 1'b0;
      tlp_rx_count     <= '0;
      cpl_tx_count     <= '0;
      bfm_error        <= 1'b0;
      cpl_state        <= CPL_IDLE;
      gb_half          <= '0;
      acc_word1        <= 1'b0;
      rx_accum         <= '0;
      hdr_word         <= '0;
      cap_tag          <= '0;
      skip_half        <= '0;
      cpl_seq_num      <= '0;
      cpld_dw0         <= '0;
      cpld_dw1         <= '0;
      cpld_dw2         <= '0;
      cpld_dw3         <= 32'hDEAD_C0DE;
      cpld_dw4         <= 32'h1234_5678;
      cpld_dw5         <= 32'hABCD_EF01;
      cpld_dw6         <= 32'hCAFE_BABE;
      cpld_dw7         <= 32'h0000_0001;
      cpl_watchdog       <= '0;
      cpl_reply_cooldown <= '0;
      dma_h2d_wait_prev <= 1'b0;
      inj_corrupt_lcrc  <= 1'b0;
      cpld_lcrc_dw0     <= '0;
      dllp_inj_state    <= DLLP_INJ_IDLE;
      dllp_inj_word     <= '0;
      for (int i = 0; i < NUM_LANES; i++) begin
        rc_tx_data[i]         <= '0;
        rc_tx_datak[i]        <= '0;
        rc_tx_valid[i]        <= 1'b0;
        rc_tx_elec_idle[i]    <= 1'b1;
        rc_tx_status[i]       <= 3'b000;
        rc_tx_status_valid[i] <= 1'b0;
      end
    end else begin
      // -----------------------------------------------------------------------
      // Link established indicator
      // -----------------------------------------------------------------------
      link_established <= (ltssm_state == L0);
      if (ltssm_state == L0 && !link_established)
        $display("[RC-BFM] Link established at time %0t", $time);

      // -----------------------------------------------------------------------
      // DMA H2D: inject CplD when engine enters wait (PIPE snoop may miss STP)
      // -----------------------------------------------------------------------
      dma_h2d_wait_prev <= dma_h2d_wait;
      if (dllp_inj_state == DLLP_INJ_P0)
        dllp_inj_state <= DLLP_INJ_P1;
      else if (dllp_inj_state == DLLP_INJ_P1)
        dllp_inj_state <= DLLP_INJ_IDLE;

      if (cpl_reply_cooldown != 16'd0)
        cpl_reply_cooldown <= cpl_reply_cooldown - 16'd1;

      if (dma_h2d_wait && !dma_h2d_wait_prev && ltssm_state == L0 &&
          cpl_state == CPL_IDLE && !cpl_injecting && !dllp_inj_active &&
          cpl_reply_cooldown == 16'd0 && auto_cpld_en) begin
        cap_tag   <= dma_mrd_tag;
        cpld_dw0  <= {3'b010, 5'b01011, 1'b0, 3'b000, 1'b0, 1'b0, 1'b0, 1'b0,
                       1'b0, 1'b0, 2'b00, 2'b00, 10'd8};
        cpld_dw1  <= {16'h0000, 3'b000, 1'b0, 12'd32};
        cpld_dw2  <= {16'h0001, dma_mrd_tag, 6'b0};
        cpl_state <= gearbox_snoop_en ? CPL_GBX_STP : CPL_INJ_A1;
        $display("[RC-BFM] DMA H2D CplD schedule tag=%0d @%0t", dma_mrd_tag, $time);
      end else if (ltssm_state == L0 && pipe_has_stp && !cpl_injecting && !dllp_inj_active &&
                   cpl_reply_cooldown == 16'd0 && auto_cpld_en) begin
        tlp_rx_count <= tlp_rx_count + 1;
        gb_half      <= '0;
        acc_word1    <= 1'b0;
        cpl_state    <= CPL_ACC;
        cpl_watchdog <= '0;
      end else if (cpl_state == CPL_IDLE) begin
        cpl_watchdog <= '0;
      end else begin
        if (cpl_watchdog >= 16'd512)
          cpl_state <= CPL_IDLE;
        else
          cpl_watchdog <= cpl_watchdog + 16'd1;

        case (cpl_state)
          CPL_ACC: begin
            rx_accum[DATA_W-1 - gb_half*NUM_LANES*PIPE_W -: NUM_LANES*PIPE_W] <=
              {dut_tx_data[3], dut_tx_data[2], dut_tx_data[1], dut_tx_data[0]};
            if (gb_half == GB_RATIO - 1) begin
              gb_half <= '0;
              if (!acc_word1) begin
                acc_word1 <= 1'b1;
              end else begin
                hdr_word  <= rx_accum;
                cpl_state <= CPL_HDR;
              end
            end else
              gb_half <= gb_half + 1'b1;
          end
          CPL_HDR: begin
            if (hdr_word[254] == 1'b0 &&
                (hdr_word[255:253] == 3'b000 || hdr_word[255:253] == 3'b001)) begin
              cap_tag <= (hdr_word[207:198] >= 10'd512) ? dma_mrd_tag
                                                       : hdr_word[207:198];
              $display("[RC-BFM] MRd captured tag=%0d @%0t", cap_tag, $time);
              if (!auto_cpld_en) begin
                acc_word1 <= 1'b0;
                cpl_state <= CPL_IDLE;
              end else begin
              cpld_dw0  <= {3'b010, 5'b01011, 1'b0, 3'b000, 1'b0, 1'b0, 1'b0, 1'b0,
                            1'b0, 1'b0, 2'b00, 2'b00, 10'd8};
              cpld_dw1  <= {16'h0000, 3'b000, 1'b0, 12'd32};
              cpld_dw2  <= {16'h0001, cap_tag, 6'b0};
              skip_half <= 4'd4;
              acc_word1 <= 1'b0;
              cpl_state <= gearbox_snoop_en ? CPL_GBX_STP : CPL_SKIP;
              end
            end else begin
              $display("[RC-BFM] Posted TLP — skip CplD @%0t", $time);
              acc_word1 <= 1'b0;
              cpl_state <= CPL_IDLE;
            end
          end
          CPL_SKIP: begin
            if (skip_half == 4'd0)
              cpl_state <= CPL_INJ_A1;
            else
              skip_half <= skip_half - 4'd1;
          end
          CPL_INJ_A1: cpl_state <= CPL_INJ_A2;
          CPL_INJ_A2: cpl_state <= CPL_INJ_B1;
          CPL_INJ_B1: cpl_state <= CPL_INJ_B2;
          CPL_INJ_B2: begin
            cpld_lcrc_dw0 <= cpld_lcrc_next;
            cpl_state     <= CPL_INJ_C1;
          end
          CPL_INJ_C1: cpl_state <= CPL_INJ_C2;
          CPL_INJ_C2: begin
            $display("[RC-BFM] CplD injected for tag=%0d seq=%0d @%0t",
                     cap_tag, dll_rx_next_seq, $time);
            cpl_seq_num          <= cpl_seq_num + 8'd1;
            cpl_tx_count         <= cpl_tx_count + 1;
            cpl_reply_cooldown   <= gearbox_snoop_en ? 16'd256 : 16'd32;
            hdr_word             <= '0;
            cpl_state            <= CPL_IDLE;
          end
          CPL_GBX_STP:   cpl_state <= CPL_GBX_B0_P0;
          CPL_GBX_B0_P0: cpl_state <= CPL_GBX_B0_P1;
          CPL_GBX_B0_P1: cpl_state <= CPL_GBX_B1_P0;
          CPL_GBX_B1_P0: cpl_state <= CPL_GBX_B1_P1;
          CPL_GBX_B1_P1: cpl_state <= CPL_GBX_B2_P0;
          CPL_GBX_B2_P0: cpl_state <= CPL_GBX_B2_P1;
          CPL_GBX_B2_P1: begin
            $display("[RC-BFM] CplD injected (gearbox) tag=%0d seq=%0d @%0t",
                     cap_tag, dll_rx_next_seq, $time);
            cpl_seq_num          <= cpl_seq_num + 8'd1;
            cpl_tx_count         <= cpl_tx_count + 1;
            cpl_reply_cooldown   <= gearbox_snoop_en ? 16'd256 : 16'd32;
            hdr_word             <= '0;
            cpl_state            <= CPL_IDLE;
          end
          default: cpl_state <= CPL_IDLE;
        endcase
      end

      // -----------------------------------------------------------------------
      // PIPE output: training data, or CplD injection override during INJ states
      // -----------------------------------------------------------------------
      for (int i = 0; i < NUM_LANES; i++) begin
        rc_tx_elec_idle[i]    <= 1'b0;
        rc_tx_status[i]       <= 3'b000;
        rc_tx_status_valid[i] <= 1'b0;

        if (dllp_inj_active) begin
          rc_tx_valid[i]  <= 1'b1;
          rc_tx_datak[i]  <= '0;
          rc_tx_data[i]   <= '0;
          if (dllp_inj_state == DLLP_INJ_P0)
            rc_tx_data[i] <= dllp_inj_word[i*PIPE_W +: PIPE_W];
          else
            rc_tx_data[i] <= dllp_inj_word[128 + i*PIPE_W +: PIPE_W];
        end else if (cpl_gbx_injecting) begin
          rc_tx_valid[i]     <= 1'b1;
          rc_tx_elec_idle[i] <= 1'b0;
          if (cpl_state == CPL_GBX_STP) begin
            rc_tx_data[i]  <= '0;
            rc_tx_datak[i] <= '0;
            if (i == 0) begin
              rc_tx_data[i]  <= {24'b0, STP};
              rc_tx_datak[i] <= 4'b0001;
            end
          end else begin
            rc_tx_data[i]  <= inj_lane_word(gbx_beat_mux, gbx_phase_mux, i);
            rc_tx_datak[i] <= '0;
            if (cpl_state == CPL_GBX_B2_P1 && i == 0)
              rc_tx_datak[i] <= 4'b0001;
          end
        end else if (cpl_lane_injecting) begin
          // ------------------------------------------------------------------
          // CplD injection: lane-at-a-time (SIM_BYPASS_PIPE flows)
          // ------------------------------------------------------------------
          rc_tx_valid[i] <= 1'b1;
          rc_tx_datak[i] <= '0;
          rc_tx_data[i]  <= '0;

          case (cpl_state)
            CPL_INJ_A1: begin  // STP cycle: lane0 byte0 = STP K-char
              if (i == 0) begin
                rc_tx_data[i]  <= {24'b0, STP};
                rc_tx_datak[i] <= 4'b0001;
              end
            end
            CPL_INJ_A2: begin  // seq-header: seq_num at bits[27:16] of lane0
              if (i == 0)
                rc_tx_data[i]  <= {4'b0, dll_rx_next_seq, 16'b0};
            end
            CPL_INJ_B1: begin  // TLP high-128: lane3=DW0,lane2=DW1,lane1=DW2,lane0=DW3
              if      (i == 3) rc_tx_data[i] <= cpld_dw0;
              else if (i == 2) rc_tx_data[i] <= cpld_dw1;
              else if (i == 1) rc_tx_data[i] <= cpld_dw2;
              else             rc_tx_data[i] <= cpld_dw3;
            end
            CPL_INJ_B2: begin  // TLP low-128: data DW4..DW7
              if      (i == 3) rc_tx_data[i] <= cpld_dw4;
              else if (i == 2) rc_tx_data[i] <= cpld_dw5;
              else if (i == 1) rc_tx_data[i] <= cpld_dw6;
              else             rc_tx_data[i] <= cpld_dw7;
            end
            CPL_INJ_C1: begin  // EOP beat upper half (zeros)
            end
            CPL_INJ_C2: begin  // EOP beat lower half: END_ in byte0; LCRC when checking enabled
              if (i == 0) begin
                if (inj_corrupt_lcrc)
                  rc_tx_data[i]  <= eop_dw0_with_end(32'hBADFFC05);
                else
                  rc_tx_data[i]  <= {24'b0, END_};
                rc_tx_datak[i] <= 4'b0001;
              end
            end
            default: ;
          endcase

        end else begin
          // ------------------------------------------------------------------
          // Normal link training outputs
          // ------------------------------------------------------------------
          case (ltssm_state)

            DETECT_QUIET: begin
              rc_tx_data[i]         <= '0;
              rc_tx_datak[i]        <= '0;
              rc_tx_valid[i]        <= 1'b0;
              rc_tx_status[i]       <= 3'b001;
              rc_tx_status_valid[i] <= 1'b1;
            end

            DETECT_ACTIVE: begin
              rc_tx_data[i]         <= {(PIPE_W/8){TS1_ID}};
              rc_tx_datak[i]        <= '0;
              rc_tx_valid[i]        <= 1'b1;
              rc_tx_status[i]       <= 3'b001;
              rc_tx_status_valid[i] <= 1'b1;
            end

            POLLING_ACTIVE,
            CONFIG_LWIDTH_START, CONFIG_LWIDTH_ACCEPT,
            CONFIG_LANENUM_WAIT,
            RECOVERY_RCVRLOCK: begin
              rc_tx_data[i]         <= {(PIPE_W/8){TS1_ID}};
              rc_tx_datak[i]        <= '0;
              rc_tx_valid[i]        <= 1'b1;
              rc_tx_status[i]       <= 3'b001;
              rc_tx_status_valid[i] <= 1'b1;
            end

            POLLING_CONFIGURATION, POLLING_SPEED,
            CONFIG_LANENUM_ACCEPT, CONFIG_COMPLETE,
            RECOVERY_RCVRCFG: begin
              rc_tx_data[i]         <= {(PIPE_W/8){TS2_ID}};
              rc_tx_datak[i]        <= '0;
              rc_tx_valid[i]        <= 1'b1;
              rc_tx_status[i]       <= 3'b010;
              rc_tx_status_valid[i] <= 1'b1;
            end

            CONFIG_IDLE, RECOVERY_IDLE: begin
              rc_tx_data[i]         <= {(PIPE_W/8){IDL}};
              rc_tx_datak[i]        <= '1;
              rc_tx_valid[i]        <= 1'b1;
              rc_tx_status[i]       <= 3'b000;
              rc_tx_status_valid[i] <= 1'b1;
            end

            default: begin  // L0 and PM sub-states: COM
              rc_tx_data[i]         <= {(PIPE_W/8){COM}};
              rc_tx_datak[i]        <= '1;
              rc_tx_valid[i]        <= 1'b1;
              rc_tx_status[i]       <= 3'b000;
              rc_tx_status_valid[i] <= 1'b0;
            end
          endcase
        end
      end // for lanes
    end
  end

  // ---------------------------------------------------------------------------
  // Task: inject_cpld_for_tag — directed-sim completion when snoop misses STP
  // ---------------------------------------------------------------------------
  task automatic inject_cpld_for_tag(input logic [9:0] tag, input bit corrupt_lcrc = 1'b0);
    inj_corrupt_lcrc = corrupt_lcrc;
    cap_tag   = tag;
    cpld_dw0  = {3'b010, 5'b01011, 1'b0, 3'b000, 1'b0, 1'b0, 1'b0, 1'b0,
                 1'b0, 1'b0, 2'b00, 2'b00, 10'd8};
    cpld_dw1  = {16'h0000, 3'b000, 1'b0, 12'd32};
    cpld_dw2  = {16'h0001, tag, 6'b0};
    cpl_state <= gearbox_snoop_en ? CPL_GBX_STP : CPL_INJ_A1;
    do @(posedge clk); while (cpl_state != CPL_IDLE);
    inj_corrupt_lcrc = 1'b0;
    $display("[RC-BFM] CplD task injected for tag=%0d @%0t", tag, $time);
  endtask

  // Directed negative test: RC sends CplD with corrupt LCRC → DUT DLL RX should NAK
  task automatic inject_nak_dllp(input logic [11:0] seq);
    dllp_inj_word  = build_nak_dllp_word(seq);
    dllp_inj_state <= DLLP_INJ_P0;
    do @(posedge clk); while (dllp_inj_state != DLLP_INJ_IDLE);
    $display("[RC-BFM] NAK DLLP injected seq=%0d @%0t", seq, $time);
  endtask

  task automatic inject_ack_dllp(input logic [11:0] seq);
    dllp_inj_word  = build_ack_dllp_word(seq);
    dllp_inj_state <= DLLP_INJ_P0;
    do @(posedge clk); while (dllp_inj_state != DLLP_INJ_IDLE);
    $display("[RC-BFM] ACK DLLP injected seq=%0d @%0t", seq, $time);
  endtask

  task automatic inject_pm_req_ack_dllp();
    dllp_inj_word  = build_pm_req_ack_dllp_word();
    dllp_inj_state <= DLLP_INJ_P0;
    do @(posedge clk); while (dllp_inj_state != DLLP_INJ_IDLE);
    $display("[RC-BFM] PM_Req_Ack DLLP injected @%0t", $time);
  endtask

  task automatic inject_bad_lcrc_tlp();
    inj_corrupt_lcrc = 1'b1;
    cap_tag   = 10'd0;
    cpld_dw0  = {3'b010, 5'b01011, 1'b0, 3'b000, 1'b0, 1'b0, 1'b0, 1'b0,
                 1'b0, 1'b0, 2'b00, 2'b00, 10'd8};
    cpld_dw1  = {16'h0000, 3'b000, 1'b0, 12'd32};
    cpld_dw2  = {16'h0001, 10'd0, 6'b0};
    cpl_state <= gearbox_snoop_en ? CPL_GBX_STP : CPL_INJ_A1;
    do @(posedge clk); while (cpl_state != CPL_IDLE);
    inj_corrupt_lcrc = 1'b0;
    $display("[RC-BFM] Bad-LCRC TLP injected @%0t", $time);
  endtask

  // RC → EP MWr with TD=1 and ECRC digest (directed ECRC RX test)
  task automatic inject_ecrc_mwr_to_ep(input bit corrupt_ecrc = 1'b0);
    logic [31:0] crc;
    logic [31:0] data_dw;
    logic [31:0] ecrc_dw;
    begin
      data_dw = 32'hCAFE_BABE;
      cpld_dw0 = {3'b010, 5'b00000, 1'b0, 3'b000, 1'b0, 1'b0, 1'b1, 1'b0,
                  1'b0, 1'b0, 2'b00, 2'b00, 10'd1};
      cpld_dw1 = 32'hE000_1000;
      cpld_dw2 = 32'h0000_0000;
      cpld_dw3 = data_dw;
      crc = ECRC_INIT;
      crc = ecrc_update_dw(crc, cpld_dw0);
      crc = ecrc_update_dw(crc, cpld_dw1);
      crc = ecrc_update_dw(crc, cpld_dw2);
      crc = ecrc_update_dw(crc, data_dw);
      ecrc_dw = ecrc_finalize(crc);
      if (corrupt_ecrc)
        ecrc_dw = ~ecrc_dw;
      cpld_dw4 = ecrc_dw;
      cpld_dw5 = 32'h0;
      cpld_dw6 = 32'h0;
      cpld_dw7 = 32'h0;
      cap_tag  = 10'd0;
      inj_corrupt_lcrc = 1'b0;
      cpl_state <= gearbox_snoop_en ? CPL_GBX_STP : CPL_INJ_A1;
      do @(posedge clk); while (cpl_state != CPL_IDLE);
      $display("[RC-BFM] ECRC MWr injected corrupt=%0b ecrc=%08h @%0t",
               corrupt_ecrc, ecrc_dw, $time);
    end
  endtask

  // ---------------------------------------------------------------------------
  // Task: send_mrd - Issue Memory Read Request to DUT EP
  // ---------------------------------------------------------------------------
  task automatic send_mrd(
    input logic [63:0]  addr,
    input logic [9:0]   len_dw,
    input logic [9:0]   tag
  );
    logic [DATA_W-1:0]  hdr;
    @(posedge clk);
    hdr = {
      3'b001, 5'b00001, 1'b0, 3'b000, 5'b0, 2'b0, 2'b0, len_dw,
      16'h0000,
      tag,
      4'hF, 4'hF,
      addr,
      {(DATA_W-128){1'b0}}
    };
    for (int i = 0; i < NUM_LANES; i++) begin
      rc_tx_data[i]  <= hdr[i*(PIPE_W) +: PIPE_W];
      rc_tx_datak[i] <= (i == 0) ? {{(PIPE_W/8-1){1'b0}}, 1'b1} : '0;
      rc_tx_valid[i] <= 1'b1;
    end
    @(posedge clk);
    for (int i = 0; i < NUM_LANES; i++) begin
      rc_tx_data[i]  <= {(PIPE_W){1'b0}};
      rc_tx_datak[i] <= (i == 0) ? {{(PIPE_W/8-1){1'b0}}, 1'b1} : '0;
    end
    @(posedge clk);
    for (int i = 0; i < NUM_LANES; i++) begin
      rc_tx_data[i]  <= {(PIPE_W/8){COM}};
      rc_tx_datak[i] <= '1;
    end
  endtask

  // ---------------------------------------------------------------------------
  // Task: send_mwr - Issue Memory Write to DUT EP
  // ---------------------------------------------------------------------------
  task automatic send_mwr(
    input logic [63:0]  addr,
    input logic [DATA_W-1:0] data
  );
    @(posedge clk);
    for (int i = 0; i < NUM_LANES; i++) begin
      rc_tx_data[i]  <= {(PIPE_W/8){8'hAA}};
      rc_tx_datak[i] <= (i == 0) ? {{(PIPE_W/8-1){1'b0}}, 1'b1} : '0;
      rc_tx_valid[i] <= 1'b1;
    end
    @(posedge clk);
    for (int i = 0; i < NUM_LANES; i++) begin
      rc_tx_data[i]  <= data[i*PIPE_W +: PIPE_W];
      rc_tx_datak[i] <= '0;
    end
    @(posedge clk);
    for (int i = 0; i < NUM_LANES; i++) begin
      rc_tx_data[i]  <= {(PIPE_W){1'b0}};
      rc_tx_datak[i] <= (i == 0) ? {{(PIPE_W/8-1){1'b0}}, 1'b1} : '0;
    end
    @(posedge clk);
    for (int i = 0; i < NUM_LANES; i++) begin
      rc_tx_data[i]  <= {(PIPE_W/8){COM}};
      rc_tx_datak[i] <= '1;
    end
  endtask

endmodule : pcie_rc_bfm

`endif // PCIE_RC_BFM_SV
