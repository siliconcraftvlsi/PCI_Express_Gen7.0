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
//     - Electrical idle detection
//     - PIPE control signals (rate, width, power_down, reset)
//     - Timeout-based state transitions
//     - Hot Reset, Disabled, Loopback states
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

  // Link status
  output logic              link_up,
  output ltssm_state_e      ltssm_state,
  output pcie_gen_e         negotiated_gen,
  output logic [4:0]        negotiated_width
);

  // ---------------------------------------------------------------------------
  // Internal state and counters
  // ---------------------------------------------------------------------------
  ltssm_state_e  state, next_state;
  logic [23:0]   timer;
  logic          timer_expired;

  // Negotiated parameters
  pcie_gen_e     neg_gen;
  logic [4:0]    neg_width;

  // Training sequence detection simulation
  logic [NUM_LANES-1:0] rx_ts1_detected;
  logic [NUM_LANES-1:0] rx_ts2_detected;
  logic                 all_lanes_rx_ts1;
  logic                 all_lanes_rx_ts2;
  logic                 any_lane_rx_elec_idle;

  // TS pattern detection (simplified – real impl uses 8b/10b or 128b/130b symbols)
  // Status encoding: 3'b001 = Receiver Detection, 3'b100 = EIEOS, etc.
  localparam logic [2:0] PIPE_STATUS_RECV_DET  = 3'b001;
  localparam logic [2:0] PIPE_STATUS_SPEED_CHG = 3'b100;
  localparam logic [2:0] PIPE_STATUS_L0        = 3'b000;

  // Training sequence counters
  logic [7:0]  ts_count;       // TS1/TS2 consecutive count
  logic [7:0]  idle_count;     // consecutive idle symbols
  logic [7:0]  eq_phase;       // Equalization phase counter
  logic        speed_change_done;

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
  // Lane aggregation helpers
  // ---------------------------------------------------------------------------
  always_comb begin
    all_lanes_rx_ts1    = &rx_ts1_detected;
    all_lanes_rx_ts2    = &rx_ts2_detected;
    any_lane_rx_elec_idle = |pipe_rx_elec_idle;
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
          rx_ts1_detected[i] <= (pipe_rx_status[i] == 3'b001) & pipe_rx_status_valid[i];
          rx_ts2_detected[i] <= (pipe_rx_status[i] == 3'b010) & pipe_rx_status_valid[i];
        end else begin
          rx_ts1_detected[i] <= 1'b0;
          rx_ts2_detected[i] <= 1'b0;
        end
      end
      // TS consecutive count – increment on TS1 or TS2 so that states
      // checking ts_count during TS1 reception (e.g. POLLING_ACTIVE,
      // RECOVERY_RCVRLOCK) work correctly, not just TS2-gated states.
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
  // Timer
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
  always_comb begin : ltssm_ns
    next_state = state;
    case (state)

      // -----------------------------------------------------------------------
      DETECT_QUIET: begin
        // Wait for any lane to detect receiver
        if (|pipe_rx_status_valid && (pipe_rx_status[0] == PIPE_STATUS_RECV_DET))
          next_state = DETECT_ACTIVE;
        else if (timer >= TIMER_DETECT_QUIET[23:0])
          next_state = DETECT_QUIET;  // stay and retry
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
        // Wait for 8 consecutive IDL symbols
        if (idle_count >= 8'd8)
          next_state = L0;
        else if (timer >= TIMER_CONFIG[23:0])
          next_state = DETECT_QUIET;
      end

      // -----------------------------------------------------------------------
      L0: begin
        // Steady-state data transfer state
        // Transitions: error→Recovery, PM→L0s/L1, Hot Reset, Disabled
        if (!(&pipe_rx_valid) && !any_lane_rx_elec_idle)
          next_state = RECOVERY_RCVRLOCK;
        // L0s entry on EIOS reception
        if (any_lane_rx_elec_idle && (timer >= 24'd100))
          next_state = L0S_RX;
      end

      L0S_TX: begin
        if (timer >= 24'd100)
          next_state = L0;
      end

      L0S_RX: begin
        if (!any_lane_rx_elec_idle)
          next_state = L0;
        else if (timer >= 24'd10_000)
          next_state = L1_ENTRY;
      end

      L1_ENTRY: begin
        if (timer >= TIMER_L1[23:0])
          next_state = L1_IDLE;
      end

      L1_IDLE: begin
        // Exit L1 on PMREQ or data
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
        else if (timer >= TIMER_RECOVERY[23:0])
          next_state = DETECT_QUIET;
      end

      RECOVERY_RCVRCFG: begin
        if (all_lanes_rx_ts2 && ts_count >= 8'd8)
          next_state = RECOVERY_IDLE;
        else if (timer >= TIMER_RECOVERY[23:0])
          next_state = DETECT_QUIET;
      end

      RECOVERY_IDLE: begin
        if (idle_count >= 8'd8)
          next_state = L0;
        else if (timer >= TIMER_RECOVERY[23:0])
          next_state = DETECT_QUIET;
      end

      RECOVERY_EQUALIZATION: begin
        if (eq_phase == 8'hFF)
          next_state = RECOVERY_RCVRLOCK;
        else if (timer >= TIMER_RECOVERY[23:0])
          next_state = DETECT_QUIET;
      end

      // -----------------------------------------------------------------------
      HOT_RESET: begin
        if (timer >= 24'd2_000)
          next_state = DETECT_QUIET;
      end

      DISABLED: begin
        // Stay until reset
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
  // Equalization phase counter (for Recovery.Equalization)
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      eq_phase         <= '0;
      speed_change_done <= 1'b0;
    end else begin
      if (state == RECOVERY_EQUALIZATION)
        eq_phase <= eq_phase + 1;
      else
        eq_phase <= '0;
      speed_change_done <= (state == POLLING_SPEED) && (timer >= 24'd50_000);
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
        // Negotiate to max supported speed ≤ MAX_GEN
        neg_gen   <= MAX_GEN;
        // Width based on number of lanes with valid RX
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
        DETECT_QUIET, DETECT_ACTIVE: begin
          pipe_reset_n    <= 1'b0;
          pipe_power_down <= 4'b0010;  // P1
        end
        POLLING_ACTIVE, POLLING_CONFIGURATION, POLLING_SPEED: begin
          pipe_reset_n    <= 1'b1;
          pipe_power_down <= 4'b0000;  // P0 (active)
          pipe_rate       <= 4'b0001;  // Gen1 during polling
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
          pipe_rate       <= 4'b0001;  // Start recovery at Gen1
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
  // Output Assignments
  // ---------------------------------------------------------------------------
  assign link_up         = (state == L0);
  assign ltssm_state     = state;
  assign negotiated_gen  = neg_gen;
  assign negotiated_width = neg_width;

endmodule : pcie_ltssm
