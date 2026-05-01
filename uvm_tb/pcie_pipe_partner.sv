// -----------------------------------------------------------------------------
// Author  : Robert Kingsly Amalathas
// Email   : robertk@microprocessorlab.com
// LinkedIn: www.linkedin.com/in/robertkingslya
// -----------------------------------------------------------------------------

module pcie_pipe_partner #(
  parameter int NUM_LANES = 4,
  parameter int PIPE_W = 32
)(
  input  logic                                 clk,
  input  logic                                 rst_n,

  output logic [NUM_LANES-1:0][PIPE_W-1:0]     rc_tx_data,
  output logic [NUM_LANES-1:0][PIPE_W/8-1:0]   rc_tx_datak,
  output logic [NUM_LANES-1:0]                 rc_tx_valid,
  output logic [NUM_LANES-1:0]                 rc_tx_elec_idle,
  output logic [NUM_LANES-1:0][2:0]            rc_tx_status,
  output logic [NUM_LANES-1:0]                 rc_tx_status_valid,

  input  logic [NUM_LANES-1:0][PIPE_W-1:0]     dut_tx_data,
  input  logic [NUM_LANES-1:0][PIPE_W/8-1:0]   dut_tx_datak,
  input  logic [NUM_LANES-1:0]                 dut_tx_elec_idle,

  output logic                                 link_partner_ready
);

  localparam logic [7:0] TS1_ID = 8'h4A;
  localparam logic [7:0] TS2_ID = 8'h45;
  localparam logic [7:0] COM    = 8'hBC;

  typedef enum logic [2:0] {
    P_DETECT,
    P_TS1,
    P_TS2,
    P_IDLE
  } state_e;

  state_e state;
  logic [15:0] cnt;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= P_DETECT;
      cnt <= '0;
      link_partner_ready <= 1'b0;
      for (int i = 0; i < NUM_LANES; i++) begin
        rc_tx_data[i] <= '0;
        rc_tx_datak[i] <= '0;
        rc_tx_valid[i] <= 1'b0;
        rc_tx_elec_idle[i] <= 1'b1;
        rc_tx_status[i] <= 3'b000;
        rc_tx_status_valid[i] <= 1'b0;
      end
    end else begin
      cnt <= cnt + 1;
      case (state)
        P_DETECT: begin
          for (int i = 0; i < NUM_LANES; i++) begin
            rc_tx_status[i] <= 3'b001;
            rc_tx_status_valid[i] <= 1'b1;
            rc_tx_valid[i] <= 1'b1;
            rc_tx_elec_idle[i] <= 1'b0;
          end
          if (cnt > 16'd50) begin
            state <= P_TS1;
            cnt <= '0;
          end
        end

        P_TS1: begin
          for (int i = 0; i < NUM_LANES; i++) begin
            rc_tx_data[i] <= {(PIPE_W/8){TS1_ID}};
            rc_tx_datak[i] <= '0;
            rc_tx_status[i] <= 3'b001;
            rc_tx_status_valid[i] <= 1'b1;
          end
          if (cnt > 16'd64) begin
            state <= P_TS2;
            cnt <= '0;
          end
        end

        P_TS2: begin
          for (int i = 0; i < NUM_LANES; i++) begin
            rc_tx_data[i] <= {(PIPE_W/8){TS2_ID}};
            rc_tx_datak[i] <= '0;
            rc_tx_status[i] <= 3'b010;
            rc_tx_status_valid[i] <= 1'b1;
          end
          if (cnt > 16'd64) begin
            state <= P_IDLE;
            cnt <= '0;
            link_partner_ready <= 1'b1;
          end
        end

        default: begin
          for (int i = 0; i < NUM_LANES; i++) begin
            rc_tx_data[i] <= {(PIPE_W/8){COM}};
            rc_tx_datak[i] <= '1;
            rc_tx_valid[i] <= 1'b1;
            rc_tx_status[i] <= 3'b000;
            rc_tx_status_valid[i] <= 1'b0;
          end
        end
      endcase
    end
  end

endmodule
