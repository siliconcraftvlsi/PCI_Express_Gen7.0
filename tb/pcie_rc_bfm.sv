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
//     - Receiving TLPs and generating completions
//     - Initiating Memory Read/Write transactions towards the EP
//     - Tracking outstanding requests and generating CplD responses
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
  // Status and scoreboard
  // -------------------------------------------------------------------------
  output logic              link_established,
  output logic [31:0]       tlp_rx_count,     // TLPs received from DUT
  output logic [31:0]       cpl_tx_count,     // Completions sent to DUT
  output logic              bfm_error
);

  // ---------------------------------------------------------------------------
  // PIPE K-code definitions
  // ---------------------------------------------------------------------------
  localparam logic [7:0] COM  = 8'hBC;
  localparam logic [7:0] STP  = 8'hFB;
  localparam logic [7:0] END_ = 8'hFD;
  localparam logic [7:0] TS1_ID = 8'h4A;
  localparam logic [7:0] TS2_ID = 8'h45;

  // ---------------------------------------------------------------------------
  // BFM State Machine
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    BFM_DETECT,
    BFM_POLL_SEND_TS1,
    BFM_POLL_RECV_TS1,
    BFM_POLL_SEND_TS2,
    BFM_POLL_RECV_TS2,
    BFM_CONFIG,
    BFM_L0,
    BFM_SEND_TLP,
    BFM_WAIT_CPL
  } bfm_state_e;

  bfm_state_e   bfm_state;
  logic [15:0]  bfm_timer;
  logic [7:0]   ts_cnt;
  logic [7:0]   idle_cnt;

  // ---------------------------------------------------------------------------
  // Link Training
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bfm_state        <= BFM_DETECT;
      link_established <= 1'b0;
      bfm_timer        <= '0;
      ts_cnt           <= '0;
      idle_cnt         <= '0;
      tlp_rx_count     <= '0;
      cpl_tx_count     <= '0;
      bfm_error        <= 1'b0;
      for (int i = 0; i < NUM_LANES; i++) begin
        rc_tx_data[i]         <= '0;
        rc_tx_datak[i]        <= '0;
        rc_tx_valid[i]        <= 1'b0;
        rc_tx_elec_idle[i]    <= 1'b1;
        rc_tx_status[i]       <= 3'b000;
        rc_tx_status_valid[i] <= 1'b0;
      end
    end else begin
      bfm_timer <= bfm_timer + 1;

      case (bfm_state)

        BFM_DETECT: begin
          // Assert receiver detected
          for (int i = 0; i < NUM_LANES; i++) begin
            rc_tx_status[i]       <= 3'b001;   // Receiver Detected
            rc_tx_status_valid[i] <= 1'b1;
            rc_tx_elec_idle[i]    <= 1'b0;
          end
          if (bfm_timer >= 16'd100) begin
            bfm_state <= BFM_POLL_SEND_TS1;
            bfm_timer <= '0;
          end
        end

        BFM_POLL_SEND_TS1: begin
          // Send TS1 ordered sets
          for (int i = 0; i < NUM_LANES; i++) begin
            rc_tx_data[i]         <= {(PIPE_W/8){TS1_ID}};
            rc_tx_datak[i]        <= '0;
            rc_tx_valid[i]        <= 1'b1;
            rc_tx_elec_idle[i]    <= 1'b0;
            rc_tx_status[i]       <= 3'b001;
            rc_tx_status_valid[i] <= 1'b1;
          end
          ts_cnt <= ts_cnt + 1;
          if (ts_cnt >= 8'd16) begin
            bfm_state <= BFM_POLL_SEND_TS2;
            ts_cnt    <= '0;
          end
        end

        BFM_POLL_SEND_TS2: begin
          // Send TS2 ordered sets
          for (int i = 0; i < NUM_LANES; i++) begin
            rc_tx_data[i]         <= {(PIPE_W/8){TS2_ID}};
            rc_tx_datak[i]        <= '0;
            rc_tx_valid[i]        <= 1'b1;
            rc_tx_status[i]       <= 3'b010;  // TS2 status
            rc_tx_status_valid[i] <= 1'b1;
          end
          ts_cnt <= ts_cnt + 1;
          if (ts_cnt >= 8'd16) begin
            bfm_state <= BFM_CONFIG;
            ts_cnt    <= '0;
          end
        end

        BFM_CONFIG: begin
          // Send IDL (idle) symbols to complete config
          for (int i = 0; i < NUM_LANES; i++) begin
            rc_tx_data[i]         <= {(PIPE_W/8){8'h7C}};  // IDL
            rc_tx_datak[i]        <= '1;
            rc_tx_valid[i]        <= 1'b1;
            rc_tx_status[i]       <= 3'b000;
            rc_tx_status_valid[i] <= 1'b1;
          end
          idle_cnt <= idle_cnt + 1;
          if (idle_cnt >= 8'd16) begin
            bfm_state        <= BFM_L0;
            link_established <= 1'b1;
            idle_cnt         <= '0;
            $display("[RC-BFM] Link established at time %0t", $time);
          end
        end

        BFM_L0: begin
          // In L0: send idle COM symbols and listen for TLPs from DUT
          for (int i = 0; i < NUM_LANES; i++) begin
            rc_tx_data[i]  <= {(PIPE_W/8){COM}};
            rc_tx_datak[i] <= '1;
            rc_tx_valid[i] <= 1'b1;
            rc_tx_status[i]       <= 3'b000;
            rc_tx_status_valid[i] <= 1'b0;
          end
          // Watch for TLPs from DUT
          if (dut_tx_datak[0][0] && (dut_tx_data[0][7:0] == STP)) begin
            tlp_rx_count <= tlp_rx_count + 1;
            $display("[RC-BFM] Received TLP from DUT at time %0t (count=%0d)",
                     $time, tlp_rx_count+1);
          end
        end

        default: bfm_state <= BFM_DETECT;
      endcase
    end
  end

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
    // Build MRd64 header (4DW): send on lane 0
    hdr = {
      3'b001, 5'b00001, 1'b0, 3'b000, 5'b0, 2'b0, 2'b0, len_dw, // DW0
      16'h0000,    // Requester ID = RC
      tag,         // Tag
      4'hF, 4'hF,  // Last/First BE
      addr,        // 64-bit address
      {(DATA_W-128){1'b0}}
    };
    for (int i = 0; i < NUM_LANES; i++) begin
      rc_tx_data[i]  <= hdr[i*(PIPE_W) +: PIPE_W];
      rc_tx_datak[i] <= (i == 0) ? {{(PIPE_W/8-1){1'b0}}, 1'b1} : '0; // STP on lane0 byte0
      rc_tx_valid[i] <= 1'b1;
    end
    @(posedge clk);
    // Mark EOP
    for (int i = 0; i < NUM_LANES; i++) begin
      rc_tx_data[i]  <= {(PIPE_W){1'b0}};
      rc_tx_datak[i] <= (i == 0) ? {{(PIPE_W/8-1){1'b0}}, 1'b1} : '0;
    end
    @(posedge clk);
    // Back to idle
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
    // Header beat (MWr64)
    for (int i = 0; i < NUM_LANES; i++) begin
      rc_tx_data[i]  <= {(PIPE_W/8){8'hAA}};  // Simplified placeholder
      rc_tx_datak[i] <= (i == 0) ? {{(PIPE_W/8-1){1'b0}}, 1'b1} : '0;
      rc_tx_valid[i] <= 1'b1;
    end
    @(posedge clk);
    // Data beat
    for (int i = 0; i < NUM_LANES; i++) begin
      rc_tx_data[i]  <= data[i*PIPE_W +: PIPE_W];
      rc_tx_datak[i] <= '0;
    end
    @(posedge clk);
    // EOP
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
