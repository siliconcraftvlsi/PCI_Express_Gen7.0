`timescale 1ns/1ps
// Minimal PIPE link partner for Verilator (no tasks, no @(posedge) in procedures).
// Mirrors tb/pcie_rc_bfm.sv link-training outputs keyed on DUT LTSSM state.

`include "pcie_pkg.sv"

module pcie_verilator_link #(
  parameter int NUM_LANES = 4,
  parameter int PIPE_W    = 32
)(
  input  logic              clk,
  input  logic              rst_n,
  input  ltssm_state_e      dut_ltssm_state,

  output logic [NUM_LANES-1:0][PIPE_W-1:0]     rc_tx_data,
  output logic [NUM_LANES-1:0][PIPE_W/8-1:0]   rc_tx_datak,
  output logic [NUM_LANES-1:0]                 rc_tx_valid,
  output logic [NUM_LANES-1:0]                 rc_tx_elec_idle,
  output logic [NUM_LANES-1:0][2:0]            rc_tx_status,
  output logic [NUM_LANES-1:0]                 rc_tx_status_valid,
  output logic                                 link_partner_ready
);

  import pcie_pkg::*;

  localparam logic [7:0] COM    = 8'hBC;
  localparam logic [7:0] IDL    = 8'h7C;
  localparam logic [7:0] TS1_ID = 8'h4A;
  localparam logic [7:0] TS2_ID = 8'h45;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      link_partner_ready <= 1'b0;
      for (int i = 0; i < NUM_LANES; i++) begin
        rc_tx_data[i]           <= '0;
        rc_tx_datak[i]          <= '0;
        rc_tx_valid[i]          <= 1'b0;
        rc_tx_elec_idle[i]      <= 1'b1;
        rc_tx_status[i]         <= 3'b000;
        rc_tx_status_valid[i]   <= 1'b0;
      end
    end else begin
      link_partner_ready <= (dut_ltssm_state == L0);

      for (int i = 0; i < NUM_LANES; i++) begin
        unique case (dut_ltssm_state)
          DETECT_QUIET: begin
            rc_tx_data[i]           <= '0;
            rc_tx_datak[i]          <= '0;
            rc_tx_valid[i]          <= 1'b0;
            rc_tx_elec_idle[i]      <= 1'b1;
            rc_tx_status[i]         <= 3'b001;
            rc_tx_status_valid[i]   <= 1'b1;
          end

          DETECT_ACTIVE: begin
            rc_tx_data[i]           <= {(PIPE_W/8){TS1_ID}};
            rc_tx_datak[i]          <= '0;
            rc_tx_valid[i]          <= 1'b1;
            rc_tx_elec_idle[i]      <= 1'b0;
            rc_tx_status[i]         <= 3'b001;
            rc_tx_status_valid[i]   <= 1'b1;
          end

          POLLING_ACTIVE,
          CONFIG_LWIDTH_START,
          CONFIG_LWIDTH_ACCEPT,
          CONFIG_LANENUM_WAIT,
          RECOVERY_RCVRLOCK: begin
            rc_tx_data[i]           <= {(PIPE_W/8){TS1_ID}};
            rc_tx_datak[i]          <= '0;
            rc_tx_valid[i]          <= 1'b1;
            rc_tx_elec_idle[i]      <= 1'b0;
            rc_tx_status[i]         <= 3'b001;
            rc_tx_status_valid[i]   <= 1'b1;
          end

          POLLING_CONFIGURATION,
          POLLING_SPEED,
          CONFIG_LANENUM_ACCEPT,
          CONFIG_COMPLETE,
          RECOVERY_RCVRCFG: begin
            rc_tx_data[i]           <= {(PIPE_W/8){TS2_ID}};
            rc_tx_datak[i]          <= '0;
            rc_tx_valid[i]          <= 1'b1;
            rc_tx_elec_idle[i]      <= 1'b0;
            rc_tx_status[i]         <= 3'b010;
            rc_tx_status_valid[i]   <= 1'b1;
          end

          CONFIG_IDLE,
          RECOVERY_IDLE: begin
            rc_tx_data[i]           <= {(PIPE_W/8){IDL}};
            rc_tx_datak[i]          <= '1;
            rc_tx_valid[i]          <= 1'b1;
            rc_tx_elec_idle[i]      <= 1'b0;
            rc_tx_status[i]         <= 3'b000;
            rc_tx_status_valid[i]   <= 1'b1;
          end

          default: begin
            rc_tx_data[i]           <= {(PIPE_W/8){COM}};
            rc_tx_datak[i]          <= '1;
            rc_tx_valid[i]          <= 1'b1;
            rc_tx_elec_idle[i]      <= 1'b0;
            rc_tx_status[i]         <= 3'b000;
            rc_tx_status_valid[i]   <= 1'b0;
          end
        endcase
      end
    end
  end

endmodule
