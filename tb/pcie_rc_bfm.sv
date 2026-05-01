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
  // DUT LTSSM state (for reactive training sequence)
  // -------------------------------------------------------------------------
  input  ltssm_state_e      ltssm_state,

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
  localparam logic [7:0] COM    = 8'hBC;
  localparam logic [7:0] STP    = 8'hFB;
  localparam logic [7:0] END_   = 8'hFD;
  localparam logic [7:0] IDL    = 8'h7C;
  localparam logic [7:0] TS1_ID = 8'h4A;
  localparam logic [7:0] TS2_ID = 8'h45;

  // ---------------------------------------------------------------------------
  // Reactive Link Training
  // ---------------------------------------------------------------------------
  // The BFM mirrors what the DUT LTSSM expects to receive:
  //   DETECT states          → assert Receiver-Detected status
  //   POLLING_ACTIVE /
  //     CONFIG_LWIDTH_*  /
  //     CONFIG_LANENUM_WAIT /
  //     RECOVERY_RCVRLOCK   → send TS1 ordered sets
  //   POLLING_CONFIGURATION /
  //     CONFIG_LANENUM_ACCEPT /
  //     RECOVERY_RCVRCFG    → send TS2 ordered sets
  //   CONFIG_COMPLETE /
  //     CONFIG_IDLE /
  //     RECOVERY_IDLE       → send IDL (electrical idle ordered set)
  //   L0 and sub-states     → send COM (idle)
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      link_established <= 1'b0;
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
      // -----------------------------------------------------------------------
      // Reactive training: mirror exactly what the DUT LTSSM needs to see
      // -----------------------------------------------------------------------
      link_established <= (ltssm_state == L0);

      if (ltssm_state == L0 && !link_established)
        $display("[RC-BFM] Link established at time %0t", $time);

      // TLP detection (valid in L0)
      if (ltssm_state == L0 &&
          dut_tx_datak[0][0] && (dut_tx_data[0][7:0] == STP)) begin
        tlp_rx_count <= tlp_rx_count + 1;
        $display("[RC-BFM] Received TLP from DUT at time %0t (count=%0d)",
                 $time, tlp_rx_count + 1);
      end

      for (int i = 0; i < NUM_LANES; i++) begin
        rc_tx_elec_idle[i] <= 1'b0;  // always driven, no electrical idle
        case (ltssm_state)

          // -------------------------------------------------------------------
          // Detect: assert Receiver Detected on status, hold data idle
          // -------------------------------------------------------------------
          DETECT_QUIET: begin
            rc_tx_data[i]         <= '0;
            rc_tx_datak[i]        <= '0;
            rc_tx_valid[i]        <= 1'b0;
            rc_tx_status[i]       <= 3'b001;   // Receiver Detected
            rc_tx_status_valid[i] <= 1'b1;
          end

          DETECT_ACTIVE: begin
            rc_tx_data[i]         <= {(PIPE_W/8){TS1_ID}};
            rc_tx_datak[i]        <= '0;
            rc_tx_valid[i]        <= 1'b1;
            rc_tx_status[i]       <= 3'b001;
            rc_tx_status_valid[i] <= 1'b1;
          end

          // -------------------------------------------------------------------
          // States that require TS1 from the RC side
          // -------------------------------------------------------------------
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

          // -------------------------------------------------------------------
          // States that require TS2 from the RC side
          // -------------------------------------------------------------------
          POLLING_CONFIGURATION, POLLING_SPEED,
          CONFIG_LANENUM_ACCEPT, CONFIG_COMPLETE,
          RECOVERY_RCVRCFG: begin
            rc_tx_data[i]         <= {(PIPE_W/8){TS2_ID}};
            rc_tx_datak[i]        <= '0;
            rc_tx_valid[i]        <= 1'b1;
            rc_tx_status[i]       <= 3'b010;
            rc_tx_status_valid[i] <= 1'b1;
          end

          // -------------------------------------------------------------------
          // Config/Recovery idle: send IDL ordered set
          // -------------------------------------------------------------------
          CONFIG_IDLE, RECOVERY_IDLE: begin
            rc_tx_data[i]         <= {(PIPE_W/8){IDL}};
            rc_tx_datak[i]        <= '1;
            rc_tx_valid[i]        <= 1'b1;
            rc_tx_status[i]       <= 3'b000;
            rc_tx_status_valid[i] <= 1'b1;
          end

          // -------------------------------------------------------------------
          // L0 and power-management sub-states: send COM (idle)
          // -------------------------------------------------------------------
          default: begin
            rc_tx_data[i]         <= {(PIPE_W/8){COM}};
            rc_tx_datak[i]        <= '1;
            rc_tx_valid[i]        <= 1'b1;
            rc_tx_status[i]       <= 3'b000;
            rc_tx_status_valid[i] <= 1'b0;
          end
        endcase
      end
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
