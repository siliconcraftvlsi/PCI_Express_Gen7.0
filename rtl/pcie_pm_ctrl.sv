`timescale 1ns/1ps

// =============================================================================
// PCIe 7.0 Controller - Power Management Controller
// PCIe Base Spec Rev 7.0 Section 5 / PM Capability
// =============================================================================
// Implements L0s / L1 / L2 power state entry and exit handshaking:
//   L0s: autonomous low-latency power saving, no handshake required
//   L1:  DLLP handshake (PM_Enter_L1 / PM_Req_Ack)
//   L2:  software-initiated via PM registers
//
// Outputs DLLP request signals consumed by pcie_dll_tx and updates
// the LTSSM power-down request. Does NOT implement L0s timer internally;
// LTSSM handles that. This module tracks PM state and generates DLLP traffic.
// =============================================================================

`include "pcie_pkg.sv"

module pcie_pm_ctrl
  import pcie_pkg::*;
#(
  parameter int unsigned L0S_ENTRY_IDLE_CYCLES = 64,   // consecutive idle beats to enter L0s
  parameter int unsigned L1_ACK_TIMEOUT_CYCLES = 4096  // cycles to wait for PM_Req_Ack
)(
  input  logic          clk,
  input  logic          rst_n,
  input  logic          link_up,

  // From LTSSM — current power state
  input  ltssm_state_e  ltssm_state,

  // Software / config-space requests
  input  logic          sw_req_l1,      // software requests L1 (via PM register)
  input  logic          sw_req_l2,      // software requests L2 (via PM register)
  input  logic          sw_pme_en,      // PME enable (wake from L2)

  // TLP TX activity indicator (from TL TX arbiter)
  input  logic          tlp_tx_active,  // any TLP in flight or pending

  // DLLP feedback from DLL RX
  input  logic          dllp_pm_req_ack_rx,  // received PM_Req_Ack from partner
  input  logic          dllp_pm_pme_rx,      // received PME (wake request)

  // DLLP request outputs to DLL TX
  output logic          dllp_pm_enter_l1_req,   // request DLL TX to send PM_Enter_L1
  output logic          dllp_pm_enter_l23_req,  // request DLL TX to send PM_Enter_L23
  output logic          dllp_pm_act_req,         // request DLL TX to send PM_Active_State

  // Power-management status
  output logic          pm_state_l0s,    // currently in L0s
  output logic          pm_state_l1,     // currently in L1
  output logic          pm_state_l2,     // currently in L2
  output logic          pm_wakeup,       // wakeup event (PME or CLKREQ)

  // To LTSSM: request power-state transition
  output logic          ltssm_pm_req_l0s,
  output logic          ltssm_pm_req_l1,
  output logic          ltssm_pm_req_l2
);

  // ---------------------------------------------------------------------------
  // PM state machine
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    PM_L0,
    PM_L0S_ENTRY,
    PM_L0S,
    PM_L1_HANDSHAKE,
    PM_L1,
    PM_L2_ENTRY,
    PM_L2,
    PM_WAKEUP
  } pm_state_e;

  pm_state_e pm_state;

  logic [15:0] idle_count;
  logic [15:0] ack_timeout_count;

  // ---------------------------------------------------------------------------
  // Idle counter — track TLP TX inactivity for L0s auto-entry
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      idle_count <= '0;
    else if (tlp_tx_active || !link_up)
      idle_count <= '0;
    else if (idle_count < 16'(L0S_ENTRY_IDLE_CYCLES))
      idle_count <= idle_count + 1;
  end

  logic idle_threshold_met;
  assign idle_threshold_met = (idle_count >= 16'(L0S_ENTRY_IDLE_CYCLES));

  // ---------------------------------------------------------------------------
  // ACK timeout counter for L1 handshake
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      ack_timeout_count <= '0;
    else if (pm_state == PM_L1_HANDSHAKE)
      ack_timeout_count <= ack_timeout_count + 1;
    else
      ack_timeout_count <= '0;
  end

  logic l1_ack_timeout;
  assign l1_ack_timeout = (ack_timeout_count >= 16'(L1_ACK_TIMEOUT_CYCLES));

  // ---------------------------------------------------------------------------
  // PM state machine
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pm_state <= PM_L0;
    end else begin
      case (pm_state)

        PM_L0: begin
          if (!link_up)
            pm_state <= PM_L0;
          else if (sw_req_l2)
            pm_state <= PM_L2_ENTRY;
          else if (sw_req_l1 && !tlp_tx_active)
            pm_state <= PM_L1_HANDSHAKE;
          else if (idle_threshold_met)
            pm_state <= PM_L0S_ENTRY;
        end

        PM_L0S_ENTRY: begin
          // L0s entry: just signal LTSSM; no DLLP needed
          pm_state <= PM_L0S;
        end

        PM_L0S: begin
          if (tlp_tx_active || dllp_pm_pme_rx)
            pm_state <= PM_L0;
          else if (sw_req_l1)
            pm_state <= PM_L1_HANDSHAKE;
        end

        PM_L1_HANDSHAKE: begin
          if (dllp_pm_req_ack_rx)
            pm_state <= PM_L1;
          else if (l1_ack_timeout)
            pm_state <= PM_L0;  // timeout: abort L1 entry
        end

        PM_L1: begin
          if (dllp_pm_pme_rx || sw_pme_en)
            pm_state <= PM_WAKEUP;
        end

        PM_L2_ENTRY: begin
          pm_state <= PM_L2;
        end

        PM_L2: begin
          if (dllp_pm_pme_rx || sw_pme_en)
            pm_state <= PM_WAKEUP;
        end

        PM_WAKEUP: begin
          if (ltssm_state == L0)
            pm_state <= PM_L0;
        end

        default: pm_state <= PM_L0;
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // Output combinational logic
  // ---------------------------------------------------------------------------
  always @* begin
    dllp_pm_enter_l1_req  = 1'b0;
    dllp_pm_enter_l23_req = 1'b0;
    dllp_pm_act_req       = 1'b0;
    ltssm_pm_req_l0s      = 1'b0;
    ltssm_pm_req_l1       = 1'b0;
    ltssm_pm_req_l2       = 1'b0;
    pm_state_l0s          = 1'b0;
    pm_state_l1           = 1'b0;
    pm_state_l2           = 1'b0;
    pm_wakeup             = 1'b0;

    case (pm_state)
      PM_L0S_ENTRY: begin
        ltssm_pm_req_l0s = 1'b1;
      end
      PM_L0S: begin
        pm_state_l0s     = 1'b1;
        ltssm_pm_req_l0s = 1'b1;
      end
      PM_L1_HANDSHAKE: begin
        dllp_pm_enter_l1_req = 1'b1;
        ltssm_pm_req_l1      = 1'b1;
      end
      PM_L1: begin
        pm_state_l1     = 1'b1;
        ltssm_pm_req_l1 = 1'b1;
      end
      PM_L2_ENTRY: begin
        dllp_pm_enter_l23_req = 1'b1;
        ltssm_pm_req_l2       = 1'b1;
      end
      PM_L2: begin
        pm_state_l2     = 1'b1;
        ltssm_pm_req_l2 = 1'b1;
      end
      PM_WAKEUP: begin
        pm_wakeup       = 1'b1;
        dllp_pm_act_req = 1'b1;
      end
      default: ;
    endcase
  end

endmodule : pcie_pm_ctrl
