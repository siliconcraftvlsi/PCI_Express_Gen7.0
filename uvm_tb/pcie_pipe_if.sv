// =============================================================================
// PCIe 7.0 UVM Testbench - PIPE Interface (for UVM agent)
// =============================================================================
// Clocking-block-annotated interface wrapping the PIPE signals used by
// pcie_pipe_agent.  Separate from the RTL pcie_pipe_if.sv which lives
// in rtl/.
// =============================================================================

interface pcie_pipe_uvm_if #(
  parameter int unsigned NUM_LANES = 4,
  parameter int unsigned PIPE_W    = 32
)(
  input logic clk,
  input logic rst_n
);

  // TX (driven by DUT toward PHY)
  logic [NUM_LANES-1:0][PIPE_W-1:0]     tx_data;
  logic [NUM_LANES-1:0][PIPE_W/8-1:0]   tx_datak;
  logic [NUM_LANES-1:0]                  tx_elec_idle;
  logic [NUM_LANES-1:0]                  tx_compliance;
  logic [3:0]                            pipe_rate;
  logic [1:0]                            pipe_width;
  logic [3:0]                            pipe_power_down;
  logic                                  pipe_reset_n;

  // RX (driven by BFM/agent toward DUT)
  logic [NUM_LANES-1:0][PIPE_W-1:0]     rx_data;
  logic [NUM_LANES-1:0][PIPE_W/8-1:0]   rx_datak;
  logic [NUM_LANES-1:0]                  rx_valid;
  logic [NUM_LANES-1:0]                  rx_elec_idle;
  logic [NUM_LANES-1:0]                  rx_status_valid;
  logic [NUM_LANES-1:0][2:0]             rx_status;

  // Link status (DUT output, monitored by agent)
  logic                                  link_up;

  // ---------------------------------------------------------------------------
  // Driver clocking block (agent drives RX toward DUT)
  // ---------------------------------------------------------------------------
  clocking drv_cb @(posedge clk);
    default input #1ns output #1ns;
    output rx_data;
    output rx_datak;
    output rx_valid;
    output rx_elec_idle;
    output rx_status_valid;
    output rx_status;
  endclocking

  // ---------------------------------------------------------------------------
  // Monitor clocking block (agent monitors TX from DUT)
  // ---------------------------------------------------------------------------
  clocking mon_cb @(posedge clk);
    default input #1ns;
    input tx_data;
    input tx_datak;
    input tx_elec_idle;
    input tx_compliance;
    input pipe_rate;
    input pipe_width;
    input pipe_power_down;
    input pipe_reset_n;
    input link_up;
  endclocking

  modport drv_mp  (clocking drv_cb, input clk, rst_n);
  modport mon_mp  (clocking mon_cb, input clk, rst_n);

endinterface : pcie_pipe_uvm_if
