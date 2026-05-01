// -----------------------------------------------------------------------------
// Author  : Robert Kingsly Amalathas
// Email   : robertk@microprocessorlab.com
// LinkedIn: www.linkedin.com/in/robertkingslya
// -----------------------------------------------------------------------------
// =============================================================================
// PCIe 7.0 Controller - AXI4 Master Bus Functional Model (BFM)
// =============================================================================
// Description:
//   AXI4 Master BFM for driving the PCIe controller's Subordinate AXI
//   interface. Provides tasks for:
//     - Single AXI write (aw + w + b)
//     - Burst AXI write
//     - Single AXI read (ar + r)
//     - Burst AXI read
//     - Polling and checking read data
// =============================================================================

`ifndef AXI_MASTER_BFM_SV
`define AXI_MASTER_BFM_SV

module axi_master_bfm #(
  parameter int DATA_W   = 256,
  parameter int ADDR_W   = 64,
  parameter int AXI_ID_W = 8
)(
  input  logic                  clk,
  input  logic                  rst_n,

  // AXI4 Master outputs → DUT subordinate
  output logic [AXI_ID_W-1:0]   m_awid,
  output logic [ADDR_W-1:0]     m_awaddr,
  output logic [7:0]            m_awlen,
  output logic [2:0]            m_awsize,
  output logic [1:0]            m_awburst,
  output logic                  m_awvalid,
  input  logic                  m_awready,

  output logic [DATA_W-1:0]     m_wdata,
  output logic [DATA_W/8-1:0]   m_wstrb,
  output logic                  m_wlast,
  output logic                  m_wvalid,
  input  logic                  m_wready,

  input  logic [AXI_ID_W-1:0]   m_bid,
  input  logic [1:0]            m_bresp,
  input  logic                  m_bvalid,
  output logic                  m_bready,

  output logic [AXI_ID_W-1:0]   m_arid,
  output logic [ADDR_W-1:0]     m_araddr,
  output logic [7:0]            m_arlen,
  output logic [2:0]            m_arsize,
  output logic [1:0]            m_arburst,
  output logic                  m_arvalid,
  input  logic                  m_arready,

  input  logic [AXI_ID_W-1:0]   m_rid,
  input  logic [DATA_W-1:0]     m_rdata,
  input  logic [1:0]            m_rresp,
  input  logic                  m_rlast,
  input  logic                  m_rvalid,
  output logic                  m_rready
);

  // ---------------------------------------------------------------------------
  // Default idle state
  // ---------------------------------------------------------------------------
  initial begin
    m_awid    = '0;
    m_awaddr  = '0;
    m_awlen   = 8'd0;
    m_awsize  = 3'b101;  // 32 bytes
    m_awburst = 2'b01;
    m_awvalid = 1'b0;
    m_wdata   = '0;
    m_wstrb   = '1;
    m_wlast   = 1'b0;
    m_wvalid  = 1'b0;
    m_bready  = 1'b1;
    m_arid    = '0;
    m_araddr  = '0;
    m_arlen   = 8'd0;
    m_arsize  = 3'b101;
    m_arburst = 2'b01;
    m_arvalid = 1'b0;
    m_rready  = 1'b1;
  end

  // ---------------------------------------------------------------------------
  // Task: axi_write (single beat)
  // ---------------------------------------------------------------------------
  task automatic axi_write(
    input  logic [ADDR_W-1:0]   addr,
    input  logic [DATA_W-1:0]   data,
    input  logic [AXI_ID_W-1:0] id = '0
  );
    // AW channel
    @(posedge clk);
    m_awid    <= id;
    m_awaddr  <= addr;
    m_awlen   <= 8'd0;
    m_awsize  <= 3'b101;
    m_awburst <= 2'b01;
    m_awvalid <= 1'b1;
    @(posedge clk);
    while (!m_awready) @(posedge clk);
    m_awvalid <= 1'b0;

    // W channel
    m_wdata  <= data;
    m_wstrb  <= '1;
    m_wlast  <= 1'b1;
    m_wvalid <= 1'b1;
    @(posedge clk);
    while (!m_wready) @(posedge clk);
    m_wvalid <= 1'b0;
    m_wlast  <= 1'b0;

    // Wait for B channel response
    while (!m_bvalid) @(posedge clk);
    if (m_bresp != 2'b00)
      $display("[AXI-BFM] WARNING: Write response SLVERR/DECERR at addr=%0h", addr);
  endtask

  // ---------------------------------------------------------------------------
  // Task: axi_write_burst
  // ---------------------------------------------------------------------------
  task automatic axi_write_burst(
    input  logic [ADDR_W-1:0]   addr,
    input  logic [DATA_W-1:0]   data [],    // Dynamic array of data beats
    input  logic [AXI_ID_W-1:0] id = '0
  );
    int beats;
    beats = data.size();
    // AW channel
    @(posedge clk);
    m_awid    <= id;
    m_awaddr  <= addr;
    m_awlen   <= 8'(beats - 1);
    m_awsize  <= 3'b101;
    m_awburst <= 2'b01;
    m_awvalid <= 1'b1;
    @(posedge clk);
    while (!m_awready) @(posedge clk);
    m_awvalid <= 1'b0;

    // W channel: send all beats
    for (int i = 0; i < beats; i++) begin
      m_wdata  <= data[i];
      m_wstrb  <= '1;
      m_wlast  <= (i == beats - 1);
      m_wvalid <= 1'b1;
      @(posedge clk);
      while (!m_wready) @(posedge clk);
    end
    m_wvalid <= 1'b0;
    m_wlast  <= 1'b0;

    while (!m_bvalid) @(posedge clk);
    if (m_bresp != 2'b00)
      $display("[AXI-BFM] WARNING: Burst write response error at addr=%0h", addr);
  endtask

  // ---------------------------------------------------------------------------
  // Task: axi_read (single beat, returns data)
  // ---------------------------------------------------------------------------
  task automatic axi_read(
    input  logic [ADDR_W-1:0]   addr,
    output logic [DATA_W-1:0]   data,
    input  logic [AXI_ID_W-1:0] id = '0
  );
    @(posedge clk);
    m_arid    <= id;
    m_araddr  <= addr;
    m_arlen   <= 8'd0;
    m_arsize  <= 3'b101;
    m_arburst <= 2'b01;
    m_arvalid <= 1'b1;
    @(posedge clk);
    while (!m_arready) @(posedge clk);
    m_arvalid <= 1'b0;

    m_rready <= 1'b1;
    while (!m_rvalid) @(posedge clk);
    data = m_rdata;
    if (m_rresp != 2'b00)
      $display("[AXI-BFM] WARNING: Read response error at addr=%0h", addr);
    @(posedge clk);
  endtask

  // ---------------------------------------------------------------------------
  // Task: axi_read_burst (returns array)
  // ---------------------------------------------------------------------------
  task automatic axi_read_burst(
    input  logic [ADDR_W-1:0]   addr,
    input  int                  beats,
    output logic [DATA_W-1:0]   data [],
    input  logic [AXI_ID_W-1:0] id = '0
  );
    data = new[beats];
    @(posedge clk);
    m_arid    <= id;
    m_araddr  <= addr;
    m_arlen   <= 8'(beats - 1);
    m_arsize  <= 3'b101;
    m_arburst <= 2'b01;
    m_arvalid <= 1'b1;
    @(posedge clk);
    while (!m_arready) @(posedge clk);
    m_arvalid <= 1'b0;

    m_rready <= 1'b1;
    for (int i = 0; i < beats; i++) begin
      while (!m_rvalid) @(posedge clk);
      data[i] = m_rdata;
      @(posedge clk);
    end
  endtask

endmodule : axi_master_bfm

`endif // AXI_MASTER_BFM_SV
