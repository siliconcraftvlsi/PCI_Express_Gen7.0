// -----------------------------------------------------------------------------
// Author  : Robert Kingsly Amalathas
// Email   : robertk@microprocessorlab.com
// LinkedIn: www.linkedin.com/in/robertkingslya
// -----------------------------------------------------------------------------

`timescale 1ns/1ps

module pcie_uvm_top;

  import uvm_pkg::*;
  import pcie_pkg::*;
  import pcie_uvm_pkg::*;

  localparam int NUM_LANES = 4;
  localparam int PIPE_W    = 32;
  localparam int DATA_W    = 256;
  localparam int ADDR_W    = 64;
  localparam int AXI_ID_W  = 8;

  logic core_clk;
  logic pipe_clk;
  logic aux_clk;
  logic rst_n;

  initial begin
    core_clk = 1'b0;
    forever #2 core_clk = ~core_clk;
  end

  initial begin
    pipe_clk = 1'b0;
    forever #2 pipe_clk = ~pipe_clk;
  end

  initial begin
    aux_clk = 1'b0;
    forever #50 aux_clk = ~aux_clk;
  end

  initial begin
    rst_n = 1'b0;
    repeat (20) @(posedge core_clk);
    rst_n = 1'b1;
  end

  pcie_axi_if #(
    .DATA_W(DATA_W),
    .ADDR_W(ADDR_W),
    .AXI_ID_W(AXI_ID_W)
  ) axi_if (
    .clk(core_clk),
    .rst_n(rst_n)
  );

  pcie_ctrl_if #(
    .ADDR_W(ADDR_W)
  ) ctrl_if (
    .clk(core_clk),
    .rst_n(rst_n)
  );

  logic [NUM_LANES-1:0][PIPE_W-1:0]   pipe_tx_data;
  logic [NUM_LANES-1:0][PIPE_W/8-1:0] pipe_tx_datak;
  logic [NUM_LANES-1:0]               pipe_tx_elec_idle;
  logic [NUM_LANES-1:0]               pipe_tx_compliance;
  logic [NUM_LANES-1:0]               pipe_tx_deemph;
  logic [NUM_LANES-1:0][2:0]          pipe_tx_margin;
  logic [NUM_LANES-1:0]               pipe_tx_swing;
  logic [NUM_LANES-1:0][1:0]          pipe_tx_eq_ctrl;

  logic [NUM_LANES-1:0][PIPE_W-1:0]   pipe_rx_data;
  logic [NUM_LANES-1:0][PIPE_W/8-1:0] pipe_rx_datak;
  logic [NUM_LANES-1:0]               pipe_rx_valid;
  logic [NUM_LANES-1:0]               pipe_rx_elec_idle;
  logic [NUM_LANES-1:0][2:0]          pipe_rx_status;
  logic [NUM_LANES-1:0]               pipe_rx_status_valid;

  logic [3:0] pipe_power_down;
  logic       pipe_reset_n;
  logic [3:0] pipe_rate;
  logic [1:0] pipe_width;
  logic       pipe_clk_req_n;

  logic [AXI_ID_W-1:0] m_axi_awid;
  logic [ADDR_W-1:0]   m_axi_awaddr;
  logic [7:0]          m_axi_awlen;
  logic [2:0]          m_axi_awsize;
  logic [1:0]          m_axi_awburst;
  logic                m_axi_awvalid;
  logic                m_axi_awready;
  logic [DATA_W-1:0]   m_axi_wdata;
  logic [DATA_W/8-1:0] m_axi_wstrb;
  logic                m_axi_wlast;
  logic                m_axi_wvalid;
  logic                m_axi_wready;
  logic [AXI_ID_W-1:0] m_axi_bid;
  logic [1:0]          m_axi_bresp;
  logic                m_axi_bvalid;
  logic                m_axi_bready;
  logic [AXI_ID_W-1:0] m_axi_arid;
  logic [ADDR_W-1:0]   m_axi_araddr;
  logic [7:0]          m_axi_arlen;
  logic [2:0]          m_axi_arsize;
  logic [1:0]          m_axi_arburst;
  logic                m_axi_arvalid;
  logic                m_axi_arready;
  logic [AXI_ID_W-1:0] m_axi_rid;
  logic [DATA_W-1:0]   m_axi_rdata;
  logic [1:0]          m_axi_rresp;
  logic                m_axi_rlast;
  logic                m_axi_rvalid;
  logic                m_axi_rready;

  logic              msi_irq;
  logic [4:0]        msi_vector;
  logic              msix_irq;
  logic [10:0]       msix_vector;
  ltssm_state_e      ltssm_state;
  pcie_gen_e         negotiated_gen;
  logic [4:0]        negotiated_width;
  logic              cfg_err_cor;
  logic              cfg_err_nonfatal;
  logic              cfg_err_fatal;
  logic [2:0]        max_payload_size;
  logic [2:0]        max_read_req_size;

  logic partner_ready;

  assign pipe_clk_req_n = 1'b0;

  initial begin
    axi_if.init_master();
    ctrl_if.init_ctrl();

    m_axi_awready = 1'b1;
    m_axi_wready  = 1'b1;
    m_axi_bid     = '0;
    m_axi_bresp   = 2'b00;
    m_axi_bvalid  = 1'b0;
    m_axi_arready = 1'b1;
    m_axi_rid     = '0;
    m_axi_rdata   = '0;
    m_axi_rresp   = 2'b00;
    m_axi_rlast   = 1'b0;
    m_axi_rvalid  = 1'b0;

    uvm_config_db#(virtual pcie_axi_if)::set(null, "uvm_test_top.*", "vif_axi", axi_if);
    uvm_config_db#(virtual pcie_ctrl_if)::set(null, "uvm_test_top.*", "vif_ctrl", ctrl_if);

    run_test("pcie_smoke_test");
  end

  assign ctrl_if.link_up = link_up;

  pcie_pipe_partner #(
    .NUM_LANES(NUM_LANES),
    .PIPE_W(PIPE_W)
  ) u_pipe_partner (
    .clk(core_clk),
    .rst_n(rst_n),
    .rc_tx_data(pipe_rx_data),
    .rc_tx_datak(pipe_rx_datak),
    .rc_tx_valid(pipe_rx_valid),
    .rc_tx_elec_idle(pipe_rx_elec_idle),
    .rc_tx_status(pipe_rx_status),
    .rc_tx_status_valid(pipe_rx_status_valid),
    .dut_tx_data(pipe_tx_data),
    .dut_tx_datak(pipe_tx_datak),
    .dut_tx_elec_idle(pipe_tx_elec_idle),
    .link_partner_ready(partner_ready)
  );

  logic link_up;

  pcie_controller_top #(
    .DEVICE_ROLE(ROLE_EP),
    .MAX_GEN(PCIE_GEN5),
    .NUM_LANES(NUM_LANES),
    .DATA_W(DATA_W),
    .ADDR_W(ADDR_W),
    .AXI_ID_W(AXI_ID_W),
    .PIPE_W(PIPE_W),
    .EN_DMA(1),
    .DMA_CHANNELS(4)
  ) dut (
    .core_clk(core_clk),
    .core_rst_n(rst_n),
    .pipe_clk(pipe_clk),
    .aux_clk(aux_clk),

    .pipe_tx_data(pipe_tx_data),
    .pipe_tx_datak(pipe_tx_datak),
    .pipe_tx_elec_idle(pipe_tx_elec_idle),
    .pipe_tx_compliance(pipe_tx_compliance),
    .pipe_tx_deemph(pipe_tx_deemph),
    .pipe_tx_margin(pipe_tx_margin),
    .pipe_tx_swing(pipe_tx_swing),
    .pipe_tx_eq_ctrl(pipe_tx_eq_ctrl),

    .pipe_rx_data(pipe_rx_data),
    .pipe_rx_datak(pipe_rx_datak),
    .pipe_rx_valid(pipe_rx_valid),
    .pipe_rx_elec_idle(pipe_rx_elec_idle),
    .pipe_rx_status_valid(pipe_rx_status_valid),
    .pipe_rx_status(pipe_rx_status),
    .pipe_clk_req_n(pipe_clk_req_n),

    .pipe_power_down(pipe_power_down),
    .pipe_reset_n(pipe_reset_n),
    .pipe_rate(pipe_rate),
    .pipe_width(pipe_width),

    .s_axi_awid(axi_if.awid),
    .s_axi_awaddr(axi_if.awaddr),
    .s_axi_awlen(axi_if.awlen),
    .s_axi_awsize(axi_if.awsize),
    .s_axi_awburst(axi_if.awburst),
    .s_axi_awvalid(axi_if.awvalid),
    .s_axi_awready(axi_if.awready),
    .s_axi_wdata(axi_if.wdata),
    .s_axi_wstrb(axi_if.wstrb),
    .s_axi_wlast(axi_if.wlast),
    .s_axi_wvalid(axi_if.wvalid),
    .s_axi_wready(axi_if.wready),
    .s_axi_bid(axi_if.bid),
    .s_axi_bresp(axi_if.bresp),
    .s_axi_bvalid(axi_if.bvalid),
    .s_axi_bready(axi_if.bready),
    .s_axi_arid(axi_if.arid),
    .s_axi_araddr(axi_if.araddr),
    .s_axi_arlen(axi_if.arlen),
    .s_axi_arsize(axi_if.arsize),
    .s_axi_arburst(axi_if.arburst),
    .s_axi_arvalid(axi_if.arvalid),
    .s_axi_arready(axi_if.arready),
    .s_axi_rid(axi_if.rid),
    .s_axi_rdata(axi_if.rdata),
    .s_axi_rresp(axi_if.rresp),
    .s_axi_rlast(axi_if.rlast),
    .s_axi_rvalid(axi_if.rvalid),
    .s_axi_rready(axi_if.rready),

    .m_axi_awid(m_axi_awid),
    .m_axi_awaddr(m_axi_awaddr),
    .m_axi_awlen(m_axi_awlen),
    .m_axi_awsize(m_axi_awsize),
    .m_axi_awburst(m_axi_awburst),
    .m_axi_awvalid(m_axi_awvalid),
    .m_axi_awready(m_axi_awready),
    .m_axi_wdata(m_axi_wdata),
    .m_axi_wstrb(m_axi_wstrb),
    .m_axi_wlast(m_axi_wlast),
    .m_axi_wvalid(m_axi_wvalid),
    .m_axi_wready(m_axi_wready),
    .m_axi_bid(m_axi_bid),
    .m_axi_bresp(m_axi_bresp),
    .m_axi_bvalid(m_axi_bvalid),
    .m_axi_bready(m_axi_bready),
    .m_axi_arid(m_axi_arid),
    .m_axi_araddr(m_axi_araddr),
    .m_axi_arlen(m_axi_arlen),
    .m_axi_arsize(m_axi_arsize),
    .m_axi_arburst(m_axi_arburst),
    .m_axi_arvalid(m_axi_arvalid),
    .m_axi_arready(m_axi_arready),
    .m_axi_rid(m_axi_rid),
    .m_axi_rdata(m_axi_rdata),
    .m_axi_rresp(m_axi_rresp),
    .m_axi_rlast(m_axi_rlast),
    .m_axi_rvalid(m_axi_rvalid),
    .m_axi_rready(m_axi_rready),

    .dma_start(ctrl_if.dma_start),
    .dma_src_addr(ctrl_if.dma_src_addr),
    .dma_dst_addr(ctrl_if.dma_dst_addr),
    .dma_length(ctrl_if.dma_length),
    .dma_dir(ctrl_if.dma_dir),
    .dma_done(ctrl_if.dma_done),
    .dma_error(ctrl_if.dma_error),

    .msi_irq(msi_irq),
    .msi_vector(msi_vector),
    .msix_irq(msix_irq),
    .msix_vector(msix_vector),
    .intx_assert(ctrl_if.intx_assert),

    .link_up(link_up),
    .ltssm_state(ltssm_state),
    .negotiated_gen(negotiated_gen),
    .negotiated_width(negotiated_width),
    .cfg_err_cor(cfg_err_cor),
    .cfg_err_nonfatal(cfg_err_nonfatal),
    .cfg_err_fatal(cfg_err_fatal),
    .max_payload_size(max_payload_size),
    .max_read_req_size(max_read_req_size)
  );

endmodule
