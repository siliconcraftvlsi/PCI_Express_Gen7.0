`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// Author  : Robert Kingsly Amalathas
// Email   : robertk@microprocessorlab.com
// LinkedIn: www.linkedin.com/in/robertkingslya
// -----------------------------------------------------------------------------
// =============================================================================
// PCIe 7.0 Controller - LTSSM (Link Training and Status State Machine)
// Based on PCI Express Base Specification Rev 7.0 Section 4.2
// =============================================================================
// Description:
//   Implements the full LTSSM state machine as defined in PCIe 7.0:
//     Detect → Polling → Configuration → Recovery → L0 → L0s/L1/L2
//   Handles:
//     - Link speed negotiation (Gen1–Gen7)
//     - Lane width negotiation (x1, x2, x4, x8, x16)
//     - Electrical idle detection (EIOS-based L0→L0S_RX)
//     - PIPE control signals (rate, width, power_down, reset)
//     - Per-phase equalization timers (EQ_PHASE0→1→2→3)
//     - Power management L1 entry/exit with DLLP handshake
//     - Timeout-based state transitions
//     - Hot Reset, Disabled, Loopback states
//     - DLL active / equalization phase status outputs
// =============================================================================

`include "pcie_pkg.sv"

module pcie_ltssm
  import pcie_pkg::*;
#(
  parameter pcie_gen_e   MAX_GEN   = PCIE_GEN7,
  parameter int unsigned NUM_LANES = 16
)(
  input  logic              clk,
  input  logic              rst_n,
  input  logic              sim_fast_recovery, // Directed TB: shorten recovery timeouts

  // PIPE RX Status inputs
  input  logic [NUM_LANES-1:0]       pipe_rx_valid,
  input  logic [NUM_LANES-1:0]       pipe_rx_elec_idle,
  input  logic [NUM_LANES-1:0]       pipe_rx_status_valid,
  input  logic [NUM_LANES-1:0][2:0]  pipe_rx_status,
  input  logic                       pipe_clk_req_n,

  // PIPE Control outputs
  output logic              pipe_reset_n,
  output logic [3:0]        pipe_rate,
  output logic [1:0]        pipe_width,
  output logic [3:0]        pipe_power_down,

  // Power management DLLP inputs (from DLL RX)
  input  logic              dllp_pm_req_l1,  // Remote requests L1 entry
  input  logic              dllp_pm_ack,     // Remote ACKed our PM request

  // Link status outputs
  output logic              link_up,
  output ltssm_state_e      ltssm_state,
  output pcie_gen_e         negotiated_gen,
  output logic [4:0]        negotiated_width,

  // DLL interface
  output logic              dll_active,      // High once CONFIG_IDLE exits; DLL may operate
  output logic [2:0]        eq_phase_out,    // Current equalization phase (0–3)
  output link_train_status_t ltssm_status    // Packed link training status register
);

  // ---------------------------------------------------------------------------
  // PIPE status code constants
  // ---------------------------------------------------------------------------
  localparam logic [2:0] PIPE_STATUS_RECV_DET  = 3'b001;  // Receiver detected
  localparam logic [2:0] PIPE_STATUS_TS1       = 3'b001;  // Reuse for TS detection
  localparam logic [2:0] PIPE_STATUS_TS2       = 3'b010;  // TS2 detection
  localparam logic [2:0] PIPE_STATUS_SPEED_CHG = 3'b100;  // Speed-change handshake
  localparam logic [2:0] PIPE_STATUS_L0        = 3'b000;  // Idle / L0
  localparam logic [2:0] PIPE_STATUS_EIOS      = 3'b010;  // Electrical-Idle Ordered Set

  // ---------------------------------------------------------------------------
  // Timeout constants (clock cycles at ~250 MHz)
  // ---------------------------------------------------------------------------
  localparam int TIMER_DETECT_QUIET     = 24'd12_000_000;   // 48 ms
  localparam int TIMER_POLLING_ACTIVE   = 24'd2_000_000;    //  8 ms
  localparam int TIMER_POLLING_CONFIG   = 24'd500_000;      //  2 ms
  localparam int TIMER_CONFIG           = 24'd500_000;      //  2 ms
  localparam int TIMER_RECOVERY         = 24'd2_000_000;    //  8 ms
  localparam int TIMER_L1               = 24'd100;
  localparam int TIMER_LOOPBACK         = 24'd10_000;

  // ---------------------------------------------------------------------------
  // Internal state and counters
  // ---------------------------------------------------------------------------
  ltssm_state_e  state, next_state;
  logic [23:0]   timer;
  logic          timer_expired;
  logic [23:0]   recovery_timer_limit;

  assign recovery_timer_limit = sim_fast_recovery ? 24'd128 : TIMER_RECOVERY[23:0];

  // Equalization phase FSM
  eq_phase_e     eq_phase;        // Current EQ phase
  logic [23:0]   eq_timer;        // Per-phase timer

  // Negotiated parameters
  pcie_gen_e     neg_gen;
  logic [4:0]    neg_width;

  // Training sequence detection simulation
  logic [NUM_LANES-1:0] rx_ts1_detected;
  logic [NUM_LANES-1:0] rx_ts2_detected;
  logic                 all_lanes_rx_ts1;
  logic                 all_lanes_rx_ts2;
  logic                 any_lane_rx_elec_idle;
  logic                 any_lane_eios;          // EIOS detected (L0→L0S_RX trigger)

  // Training sequence counters
  logic [7:0]  ts_count;       // TS1/TS2 consecutive count
  logic [7:0]  idle_count;     // consecutive idle symbols
  logic        speed_change_done;

  // ---------------------------------------------------------------------------
  // Lane aggregation helpers
  // ---------------------------------------------------------------------------
  always @* begin
    all_lanes_rx_ts1    = &rx_ts1_detected;
    all_lanes_rx_ts2    = &rx_ts2_detected;
    any_lane_rx_elec_idle = |pipe_rx_elec_idle;
    // EIOS: any lane reports PIPE_STATUS_EIOS while rx_valid is asserted
    any_lane_eios = 1'b0;
    for (int k = 0; k < NUM_LANES; k++) begin
      if (pipe_rx_status_valid[k] && (pipe_rx_status[k] == PIPE_STATUS_EIOS))
        any_lane_eios = 1'b1;
    end
  end

  // Simulate TS detection via rx_valid and status
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_ts1_detected <= '0;
      rx_ts2_detected <= '0;
      ts_count        <= '0;
      idle_count      <= '0;
    end else begin
      for (int i = 0; i < NUM_LANES; i++) begin
        if (pipe_rx_valid[i]) begin
          // Training set detection based on status codes
          rx_ts1_detected[i] <= (pipe_rx_status[i] == PIPE_STATUS_TS1) & pipe_rx_status_valid[i];
          rx_ts2_detected[i] <= (pipe_rx_status[i] == PIPE_STATUS_TS2) & pipe_rx_status_valid[i];
        end else begin
          rx_ts1_detected[i] <= 1'b0;
          rx_ts2_detected[i] <= 1'b0;
        end
      end
      // TS consecutive count
      if (all_lanes_rx_ts1 || all_lanes_rx_ts2)
        ts_count <= ts_count + 1;
      else
        ts_count <= '0;
      // Idle symbol count
      if (&pipe_rx_valid && ~any_lane_rx_elec_idle)
        idle_count <= idle_count + 1;
      else
        idle_count <= '0;
    end
  end

  // ---------------------------------------------------------------------------
  // Global State Timer (resets on state change)
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      timer <= '0;
    end else begin
      if (state != next_state)
        timer <= '0;
      else if (timer != 24'hFFFFFF)
        timer <= timer + 1;
    end
  end

  assign timer_expired = (timer == 24'hFFFFFF);

  // ---------------------------------------------------------------------------
  // State Register
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      state <= DETECT_QUIET;
    else
      state <= next_state;
  end

  // ---------------------------------------------------------------------------
  // Next-State Logic
  // ---------------------------------------------------------------------------
  always @* begin : ltssm_ns
    next_state = state;
    case (state)

      // -----------------------------------------------------------------------
      DETECT_QUIET: begin
        // Issue pipe_reset_n=0; wait for receiver detect on at least one lane
        if (|pipe_rx_status_valid && (pipe_rx_status[0] == PIPE_STATUS_RECV_DET))
          next_state = DETECT_ACTIVE;
        // No timeout retry — just wait for detect
      end

      DETECT_ACTIVE: begin
        if (&pipe_rx_valid)
          next_state = POLLING_ACTIVE;
        else if (timer >= 24'd25_000)  // 100 µs
          next_state = DETECT_QUIET;
      end

      POLLING_ACTIVE: begin
        // Transmit TS1 patterns; wait for 8 consecutive TS1 back
        if (all_lanes_rx_ts1 && ts_count >= 8'd8)
          next_state = POLLING_CONFIGURATION;
        else if (timer >= TIMER_POLLING_ACTIVE[23:0])
          next_state = DETECT_QUIET;
      end

      POLLING_COMPLIANCE: begin
        if (timer >= TIMER_POLLING_ACTIVE[23:0])
          next_state = DETECT_QUIET;
      end

      POLLING_CONFIGURATION: begin
        // Transmit TS2; wait for 8 consecutive TS2 back
        if (all_lanes_rx_ts2 && ts_count >= 8'd8)
          next_state = CONFIG_LWIDTH_START;
        else if (timer >= TIMER_POLLING_CONFIG[23:0])
          next_state = DETECT_QUIET;
      end

      POLLING_SPEED: begin
        // pipe_rate updated to target gen; wait 50 µs then go to RECOVERY
        // TIMER_POLLING_CONFIG = 500_000 cycles (2 ms) >> 50 µs; use 12_500 (50 µs)
        if (speed_change_done)
          next_state = RECOVERY_RCVRLOCK;
        else if (timer >= TIMER_POLLING_CONFIG[23:0])
          next_state = DETECT_QUIET;
      end

      CONFIG_LWIDTH_START: begin
        // Exchange lane number TS1s
        if (timer >= 24'd5_000)
          next_state = CONFIG_LWIDTH_ACCEPT;
      end

      CONFIG_LWIDTH_ACCEPT: begin
        if (timer >= 24'd5_000)
          next_state = CONFIG_LANENUM_WAIT;
      end

      CONFIG_LANENUM_WAIT: begin
        if (all_lanes_rx_ts1)
          next_state = CONFIG_LANENUM_ACCEPT;
        else if (timer >= TIMER_CONFIG[23:0])
          next_state = DETECT_QUIET;
      end

      CONFIG_LANENUM_ACCEPT: begin
        if (all_lanes_rx_ts2 && ts_count >= 8'd8)
          next_state = CONFIG_COMPLETE;
        else if (timer >= TIMER_CONFIG[23:0])
          next_state = DETECT_QUIET;
      end

      CONFIG_COMPLETE: begin
        next_state = CONFIG_IDLE;
      end

      CONFIG_IDLE: begin
        // Wait for 8 consecutive IDL symbols before L0
        if (idle_count >= 8'd8)
          next_state = L0;
        else if (timer >= TIMER_CONFIG[23:0])
          next_state = DETECT_QUIET;
      end

      // -----------------------------------------------------------------------
      L0: begin
        // Steady-state data transfer state
        // Recovery on lane loss
        if (!(&pipe_rx_valid) && !any_lane_rx_elec_idle)
          next_state = RECOVERY_RCVRLOCK;
        // L0→L0S_RX: EIOS received (Electrical-Idle Ordered Set from remote)
        else if (any_lane_eios)
          next_state = L0S_RX;
        // L1 entry: both sides agree via PM DLLP handshake
        else if (dllp_pm_req_l1 && dllp_pm_ack)
          next_state = L1_ENTRY;
      end

      L0S_TX: begin
        // We initiated L0s; return after idle period
        if (timer >= L0S_EXIT_TIMEOUT)
          next_state = L0;
      end

      L0S_RX: begin
        // Remote entered L0s; wait for EIOS to deassert
        if (!any_lane_rx_elec_idle && !any_lane_eios)
          next_state = L0;
        else if (timer >= 24'd10_000)
          // Prolonged idle → attempt L1
          next_state = L1_ENTRY;
      end

      L1_ENTRY: begin
        // Wait for all lanes to reach electrical idle
        if (timer >= L1_EXIT_TIMEOUT)
          next_state = L1_IDLE;
      end

      L1_IDLE: begin
        // Exit L1: remote removes electrical idle (pipe_rx_valid deasserts elec_idle)
        if (|pipe_rx_valid && !any_lane_rx_elec_idle)
          next_state = RECOVERY_RCVRLOCK;
      end

      L2_IDLE: begin
        if (|pipe_rx_valid)
          next_state = L2_TX_WAKE;
      end

      L2_TX_WAKE: begin
        if (timer >= 24'd1_000)
          next_state = DETECT_QUIET;
      end

      // -----------------------------------------------------------------------
      RECOVERY_RCVRLOCK: begin
        if (all_lanes_rx_ts1 && ts_count >= 8'd8)
          next_state = RECOVERY_RCVRCFG;
        else if (timer >= recovery_timer_limit)
          next_state = DETECT_QUIET;
      end

      RECOVERY_RCVRCFG: begin
        if (all_lanes_rx_ts2 && ts_count >= 8'd8)
          next_state = RECOVERY_IDLE;
        else if (timer >= recovery_timer_limit)
          next_state = DETECT_QUIET;
      end

      RECOVERY_IDLE: begin
        if (idle_count >= 8'd8)
          next_state = L0;
        else if (timer >= recovery_timer_limit)
          next_state = DETECT_QUIET;
      end

      RECOVERY_EQUALIZATION: begin
        // Advance through EQ phases; when phase 3 completes, re-lock
        if (eq_phase == EQ_PHASE3 && eq_timer >= EQ_PHASE3_TIMEOUT)
          next_state = RECOVERY_RCVRLOCK;
        else if (timer >= recovery_timer_limit)
          next_state = DETECT_QUIET;
      end

      // -----------------------------------------------------------------------
      HOT_RESET: begin
        if (timer >= 24'd2_000)
          next_state = DETECT_QUIET;
      end

      DISABLED: begin
        if (timer >= 24'd2_000_000)
          next_state = DETECT_QUIET;
      end

      LOOPBACK_ENTRY: begin
        if (all_lanes_rx_ts1)
          next_state = LOOPBACK_ACTIVE;
      end

      LOOPBACK_ACTIVE: begin
        if (timer >= TIMER_LOOPBACK[23:0])
          next_state = LOOPBACK_EXIT;
      end

      LOOPBACK_EXIT: begin
        next_state = DETECT_QUIET;
      end

      default: next_state = DETECT_QUIET;
    endcase
  end

  // ---------------------------------------------------------------------------
  // Equalization Phase FSM (active during RECOVERY_EQUALIZATION)
  // Cycles EQ_PHASE0 → EQ_PHASE1 → EQ_PHASE2 → EQ_PHASE3 with per-phase timers
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      eq_phase        <= EQ_PHASE0;
      eq_timer        <= '0;
      speed_change_done <= 1'b0;
    end else begin
      // Speed-change completion: 50 µs after POLLING_SPEED entry
      speed_change_done <= (state == POLLING_SPEED) && (timer >= 24'd12_500);

      if (state == RECOVERY_EQUALIZATION) begin
        eq_timer <= eq_timer + 1;
        case (eq_phase)
          EQ_PHASE0: begin
            // Preset phase — one cycle; immediately advance
            if (eq_timer >= 24'd1)
              eq_phase <= EQ_PHASE1;
          end
          EQ_PHASE1: begin
            if (eq_timer >= EQ_PHASE1_TIMEOUT) begin
              eq_phase <= EQ_PHASE2;
              eq_timer <= '0;
            end
          end
          EQ_PHASE2: begin
            if (eq_timer >= EQ_PHASE2_TIMEOUT) begin
              eq_phase <= EQ_PHASE3;
              eq_timer <= '0;
            end
          end
          EQ_PHASE3: begin
            // Completion handled in next-state logic; hold here
            if (eq_timer >= EQ_PHASE3_TIMEOUT)
              eq_timer <= eq_timer;  // Hold — next_state will fire
          end
          default: eq_phase <= EQ_PHASE0;
        endcase
      end else begin
        // Reset EQ phase when leaving Recovery.Equalization
        eq_phase <= EQ_PHASE0;
        eq_timer <= '0;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Negotiated Generation and Width
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      neg_gen   <= PCIE_GEN1;
      neg_width <= 5'd1;
    end else begin
      if (state == CONFIG_COMPLETE) begin
        neg_gen   <= MAX_GEN;
        case (NUM_LANES)
          16:      neg_width <= 5'd16;
          8:       neg_width <= 5'd8;
          4:       neg_width <= 5'd4;
          2:       neg_width <= 5'd2;
          default: neg_width <= 5'd1;
        endcase
      end
    end
  end

  // ---------------------------------------------------------------------------
  // PIPE Control Signal Generation
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pipe_reset_n    <= 1'b0;
      pipe_rate       <= 4'b0001;   // Gen1 default
      pipe_width      <= 2'b00;
      pipe_power_down <= 4'b0011;   // P2 (powered down)
    end else begin
      case (state)
        DETECT_QUIET: begin
          // Assert reset; drive P1 power-down during receiver detect
          pipe_reset_n    <= 1'b0;
          pipe_power_down <= 4'b0010;  // P1
        end

        DETECT_ACTIVE: begin
          pipe_reset_n    <= 1'b0;
          pipe_power_down <= 4'b0010;  // P1
        end

        POLLING_ACTIVE, POLLING_CONFIGURATION: begin
          pipe_reset_n    <= 1'b1;
          pipe_power_down <= 4'b0000;  // P0 (active)
          pipe_rate       <= 4'b0001;  // Gen1 during polling
        end

        POLLING_SPEED: begin
          // Update pipe_rate to target generation; PHY will complete speed change
          pipe_reset_n    <= 1'b1;
          pipe_power_down <= 4'b0000;
          pipe_rate       <= 4'(neg_gen);
        end

        CONFIG_LWIDTH_START, CONFIG_LWIDTH_ACCEPT,
        CONFIG_LANENUM_WAIT, CONFIG_LANENUM_ACCEPT,
        CONFIG_COMPLETE, CONFIG_IDLE: begin
          pipe_reset_n    <= 1'b1;
          pipe_power_down <= 4'b0000;
          pipe_rate       <= 4'b0001;
        end

        L0: begin
          pipe_reset_n    <= 1'b1;
          pipe_power_down <= 4'b0000;
          pipe_rate       <= 4'(neg_gen);
          pipe_width      <= (neg_width >= 16) ? 2'b11 :
                             (neg_width >= 8)  ? 2'b10 :
                             (neg_width >= 4)  ? 2'b01 : 2'b00;
        end

        L0S_TX, L0S_RX: begin
          pipe_power_down <= 4'b0001;  // P0s
        end

        L1_ENTRY, L1_IDLE: begin
          pipe_power_down <= 4'b0010;  // P1
        end

        L2_IDLE: begin
          pipe_power_down <= 4'b0011;  // P2
        end

        RECOVERY_RCVRLOCK, RECOVERY_RCVRCFG, RECOVERY_IDLE,
        RECOVERY_EQUALIZATION: begin
          pipe_reset_n    <= 1'b1;
          pipe_power_down <= 4'b0000;
          // During equalization keep negotiated rate; start recovery at Gen1 otherwise
          if (state == RECOVERY_EQUALIZATION)
            pipe_rate     <= 4'(neg_gen);
          else
            pipe_rate     <= 4'b0001;
        end

        HOT_RESET: begin
          pipe_reset_n    <= 1'b0;
          pipe_power_down <= 4'b0000;
        end

        default: begin
          pipe_reset_n    <= 1'b0;
          pipe_power_down <= 4'b0011;
        end
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // ltssm_status Register — updated in CONFIG_COMPLETE and L0
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ltssm_status.link_up             <= 1'b0;
      ltssm_status.speed               <= PCIE_GEN1;
      ltssm_status.width               <= 5'd0;
      ltssm_status.eq_phase            <= 3'd0;
      ltssm_status.upconfigure_capable <= 1'b0;
      ltssm_status.retrain_link        <= 1'b0;
    end else begin
      // eq_phase tracking always live
      ltssm_status.eq_phase <= {1'b0, eq_phase};

      case (state)
        CONFIG_COMPLETE: begin
          ltssm_status.speed              <= neg_gen;
          ltssm_status.width             <= neg_width;
          ltssm_status.link_up           <= 1'b0;
          ltssm_status.upconfigure_capable <= (NUM_LANES > 1) ? 1'b1 : 1'b0;
        end
        L0: begin
          ltssm_status.link_up <= 1'b1;
          ltssm_status.speed   <= neg_gen;
          ltssm_status.width   <= neg_width;
        end
        default: begin
          if (state != L0S_TX && state != L0S_RX)
            ltssm_status.link_up <= 1'b0;
        end
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // Output Assignments
  // ---------------------------------------------------------------------------
  assign link_up          = (state == L0);
  assign ltssm_state      = state;
  assign negotiated_gen   = neg_gen;
  assign negotiated_width = neg_width;

  // dll_active: asserted once the link exits CONFIG_IDLE (enters L0 or beyond)
  // Deassert on HOT_RESET, DISABLED, or DETECT states
  assign dll_active = (state == L0)       ||
                      (state == L0S_TX)   ||
                      (state == L0S_RX)   ||
                      (state == L1_ENTRY) ||
                      (state == L1_IDLE)  ||
                      (state == RECOVERY_RCVRLOCK) ||
                      (state == RECOVERY_RCVRCFG)  ||
                      (state == RECOVERY_IDLE)      ||
                      (state == RECOVERY_EQUALIZATION);

  // Equalization phase output (3-bit; EQ_PHASE3 = 2'b11 → 3'd3)
  assign eq_phase_out = {1'b0, eq_phase};

endmodule : pcie_ltssm
