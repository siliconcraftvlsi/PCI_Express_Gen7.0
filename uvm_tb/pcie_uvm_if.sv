// -----------------------------------------------------------------------------
// Author  : Robert Kingsly Amalathas
// Email   : robertk@microprocessorlab.com
// LinkedIn: www.linkedin.com/in/robertkingslya
// -----------------------------------------------------------------------------

`include "pcie_pkg.sv"

interface pcie_axi_if #(
  parameter int DATA_W = 256,
  parameter int ADDR_W = 64,
  parameter int AXI_ID_W = 8
)(
  input logic clk,
  input logic rst_n
);

  logic [AXI_ID_W-1:0] awid;
  logic [ADDR_W-1:0]   awaddr;
  logic [7:0]          awlen;
  logic [2:0]          awsize;
  logic [1:0]          awburst;
  logic                awvalid;
  logic                awready;

  logic [DATA_W-1:0]   wdata;
  logic [DATA_W/8-1:0] wstrb;
  logic                wlast;
  logic                wvalid;
  logic                wready;

  logic [AXI_ID_W-1:0] bid;
  logic [1:0]          bresp;
  logic                bvalid;
  logic                bready;

  logic [AXI_ID_W-1:0] arid;
  logic [ADDR_W-1:0]   araddr;
  logic [7:0]          arlen;
  logic [2:0]          arsize;
  logic [1:0]          arburst;
  logic                arvalid;
  logic                arready;

  logic [AXI_ID_W-1:0] rid;
  logic [DATA_W-1:0]   rdata;
  logic [1:0]          rresp;
  logic                rlast;
  logic                rvalid;
  logic                rready;

  task automatic init_master();
    awid    = '0;
    awaddr  = '0;
    awlen   = 8'd0;
    awsize  = 3'b101;
    awburst = 2'b01;
    awvalid = 1'b0;

    wdata   = '0;
    wstrb   = '0;
    wlast   = 1'b0;
    wvalid  = 1'b0;

    bready  = 1'b1;

    arid    = '0;
    araddr  = '0;
    arlen   = 8'd0;
    arsize  = 3'b101;
    arburst = 2'b01;
    arvalid = 1'b0;

    rready  = 1'b1;
  endtask

endinterface

interface pcie_ctrl_if #(
  parameter int ADDR_W = 64
)(
  input logic clk,
  input logic rst_n
);

  import pcie_pkg::*;

  logic              dma_start;
  logic [ADDR_W-1:0] dma_src_addr;
  logic [ADDR_W-1:0] dma_dst_addr;
  logic [31:0]       dma_length;
  logic              dma_dir;
  logic              dma_done;
  logic              dma_error;

  logic              intx_assert;
  logic              link_up;
  ltssm_state_e      ltssm_state;

  // Error-injection controls (UVM sequences drive; partner/BFM may observe)
  logic              inject_nak;
  logic              block_ack;
  logic              inject_malformed_tlp;
  logic              inject_poison;
  logic              block_cpl;
  bit                auto_cpld_en;

  // TB/UVM feature hooks (wired to DUT tb_* ports in uvm_top)
  logic              tb_sw_req_l1;
  logic              tb_pm_req_ack;
  logic              tb_sim_int_override;
  logic              tb_sim_msi_en;
  logic              tb_sim_msix_en;
  logic              pm_state_l0s;
  logic              pm_state_l1;
  logic              msix_irq_obs;
  logic              msi_irq_obs;
  logic              cfg_err_cor_obs;

  task automatic init_ctrl();
    dma_start    = 1'b0;
    dma_src_addr = '0;
    dma_dst_addr = '0;
    dma_length   = '0;
    dma_dir      = 1'b0;
    intx_assert  = 1'b0;
    inject_nak   = 1'b0;
    block_ack    = 1'b0;
    inject_malformed_tlp = 1'b0;
    inject_poison        = 1'b0;
    block_cpl    = 1'b0;
    auto_cpld_en = 1'b1;
    tb_sw_req_l1 = 1'b0;
    tb_pm_req_ack = 1'b0;
    tb_sim_int_override = 1'b0;
    tb_sim_msi_en = 1'b0;
    tb_sim_msix_en = 1'b0;
    ltssm_state  = DETECT_QUIET;
  endtask

  task automatic pulse_dma(
    input logic [ADDR_W-1:0] src_addr,
    input logic [ADDR_W-1:0] dst_addr,
    input logic [31:0]       length,
    input logic              dir
  );
    @(posedge clk);
    dma_src_addr <= src_addr;
    dma_dst_addr <= dst_addr;
    dma_length   <= length;
    dma_dir      <= dir;
    dma_start    <= 1'b1;
    @(posedge clk);
    dma_start    <= 1'b0;
  endtask

  task automatic pulse_intx();
    @(posedge clk);
    intx_assert <= 1'b1;
    @(posedge clk);
    intx_assert <= 1'b0;
  endtask

endinterface
