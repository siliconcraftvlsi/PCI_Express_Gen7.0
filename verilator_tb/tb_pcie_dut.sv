`timescale 1ns/1ps
// Minimal Verilator testbench: DUT + PIPE link partner only (no RC/AXI BFMs).
// Clocks are toggled from C++ (sim_main.cpp). Pass/fail via $finish.

`include "pcie_pkg.sv"

module tb_pcie_dut;

  import pcie_pkg::*;

  localparam int NUM_LANES = 4;
  localparam int PIPE_W    = 32;
  localparam int DATA_W    = 256;
  localparam int ADDR_W    = 64;
  localparam int AXI_ID_W  = 8;
  logic core_clk;
  logic pipe_clk;
  logic aux_clk;
  logic rst_n;

  logic [NUM_LANES-1:0][PIPE_W-1:0]     pipe_tx_data;
  logic [NUM_LANES-1:0][PIPE_W/8-1:0]   pipe_tx_datak;
  logic [NUM_LANES-1:0]                 pipe_tx_elec_idle;
  logic [NUM_LANES-1:0]                 pipe_tx_compliance;
  logic [NUM_LANES-1:0]                 pipe_tx_deemph;
  logic [NUM_LANES-1:0][2:0]            pipe_tx_margin;
  logic [NUM_LANES-1:0]                 pipe_tx_swing;
  logic [NUM_LANES-1:0][1:0]            pipe_tx_eq_ctrl;

  logic [NUM_LANES-1:0][PIPE_W-1:0]     pipe_rx_data;
  logic [NUM_LANES-1:0][PIPE_W/8-1:0]   pipe_rx_datak;
  logic [NUM_LANES-1:0]                 pipe_rx_valid;
  logic [NUM_LANES-1:0]                 pipe_rx_elec_idle;
  logic [NUM_LANES-1:0][2:0]            pipe_rx_status;
  logic [NUM_LANES-1:0]                 pipe_rx_status_valid;

  logic [3:0] pipe_power_down;
  logic       pipe_reset_n;
  logic [3:0] pipe_rate;
  logic [1:0] pipe_width;
  logic       pipe_clk_req_n;

  logic [AXI_ID_W-1:0] m_axi_awid, m_axi_arid, m_axi_bid, m_axi_rid;
  logic [ADDR_W-1:0]   m_axi_awaddr, m_axi_araddr;
  logic [7:0]          m_axi_awlen, m_axi_arlen;
  logic [2:0]          m_axi_awsize, m_axi_arsize;
  logic [1:0]          m_axi_awburst, m_axi_arburst;
  logic                m_axi_awvalid, m_axi_arvalid, m_axi_wvalid;
  logic                m_axi_awready, m_axi_wready, m_axi_arready;
  logic [DATA_W-1:0]   m_axi_wdata;
  logic [DATA_W/8-1:0] m_axi_wstrb;
  logic                m_axi_wlast;
  logic [1:0]          m_axi_bresp, m_axi_rresp;
  logic                m_axi_bvalid, m_axi_rvalid, m_axi_rlast;
  logic                m_axi_bready, m_axi_rready;
  logic [DATA_W-1:0]   m_axi_rdata;

  logic [AXI_ID_W-1:0] s_axi_awid, s_axi_arid;
  logic [ADDR_W-1:0]   s_axi_awaddr, s_axi_araddr;
  logic [7:0]          s_axi_awlen, s_axi_arlen;
  logic [2:0]          s_axi_awsize, s_axi_arsize;
  logic [1:0]          s_axi_awburst, s_axi_arburst;
  logic                s_axi_awvalid, s_axi_arvalid, s_axi_wvalid;
  logic                s_axi_awready, s_axi_wready, s_axi_arready;
  logic [DATA_W-1:0]   s_axi_wdata;
  logic [DATA_W/8-1:0] s_axi_wstrb;
  logic                s_axi_wlast;
  logic [AXI_ID_W-1:0] s_axi_bid;
  logic [1:0]          s_axi_bresp;
  logic                s_axi_bvalid, s_axi_bready;
  logic [AXI_ID_W-1:0] s_axi_rid;
  logic [DATA_W-1:0]   s_axi_rdata;
  logic [1:0]          s_axi_rresp;
  logic                s_axi_rlast, s_axi_rvalid, s_axi_rready;

  logic              dma_start, dma_dir, dma_done, dma_error;
  logic [ADDR_W-1:0] dma_src_addr, dma_dst_addr;
  logic [31:0]       dma_length;
  logic              msi_irq, msix_irq, intx_assert;
  logic [4:0]        msi_vector;
  logic [10:0]       msix_vector;
  logic              link_up;
  ltssm_state_e      ltssm_state;
  pcie_gen_e         negotiated_gen;
  logic [4:0]        negotiated_width;
  logic              cfg_err_cor, cfg_err_nonfatal, cfg_err_fatal;
  logic [2:0]        max_payload_size, max_read_req_size;
  logic [2:0]        eq_phase;
  logic              dll_link_active;

  logic link_partner_ready;

  assign pipe_clk       = core_clk;
  assign aux_clk        = core_clk;
  assign pipe_clk_req_n = 1'b0;

  // AXI subordinate idle; manager terminated
  assign s_axi_awready = 1'b1;
  assign s_axi_wready  = 1'b1;
  assign s_axi_bid     = '0;
  assign s_axi_bresp   = 2'b00;
  assign s_axi_bvalid  = 1'b0;
  assign s_axi_arready = 1'b1;
  assign s_axi_rid     = '0;
  assign s_axi_rdata   = '0;
  assign s_axi_rresp   = 2'b00;
  assign s_axi_rlast   = 1'b0;
  assign s_axi_rvalid  = 1'b0;
  assign s_axi_bready  = 1'b1;
  assign s_axi_rready  = 1'b1;

  assign m_axi_awready = 1'b1;
  assign m_axi_wready  = 1'b1;
  assign m_axi_arready = 1'b1;
  assign m_axi_bid     = '0;
  assign m_axi_bresp   = 2'b00;
  assign m_axi_bvalid  = 1'b0;
  assign m_axi_rid     = '0;
  assign m_axi_rdata   = '0;
  assign m_axi_rresp   = 2'b00;
  assign m_axi_rlast   = 1'b0;
  assign m_axi_rvalid  = 1'b0;
  assign m_axi_bready  = 1'b1;
  assign m_axi_rready  = 1'b1;

  assign dma_start    = 1'b0;
  assign dma_src_addr = '0;
  assign dma_dst_addr = '0;
  assign dma_length   = '0;
  assign dma_dir      = 1'b0;
  assign intx_assert  = 1'b0;

  pcie_verilator_link #(
    .NUM_LANES(NUM_LANES),
    .PIPE_W(PIPE_W)
  ) u_link (
    .clk(core_clk),
    .rst_n(rst_n),
    .dut_ltssm_state(ltssm_state),
    .rc_tx_data(pipe_rx_data),
    .rc_tx_datak(pipe_rx_datak),
    .rc_tx_valid(pipe_rx_valid),
    .rc_tx_elec_idle(pipe_rx_elec_idle),
    .rc_tx_status(pipe_rx_status),
    .rc_tx_status_valid(pipe_rx_status_valid),
    .link_partner_ready(link_partner_ready)
  );

  pcie_controller_top #(
    .DEVICE_ROLE(ROLE_EP),
    .MAX_GEN(PCIE_GEN5),
    .NUM_LANES(NUM_LANES),
    .DATA_W(DATA_W),
    .ADDR_W(ADDR_W),
    .AXI_ID_W(AXI_ID_W),
    .PIPE_W(PIPE_W),
    .EN_DMA(1),
    .DMA_CHANNELS(4),
    .SIM_BYPASS(1),
    .SIM_BYPASS_PIPE(1),
    .SIM_BYPASS_DLL_TX(1),
    .SIM_BYPASS_DLL_RX(1),
    .SIM_BYPASS_TLP_TX(1),
    .SIM_BYPASS_LCRC(1)
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

    .s_axi_awid(s_axi_awid),
    .s_axi_awaddr(s_axi_awaddr),
    .s_axi_awlen(s_axi_awlen),
    .s_axi_awsize(s_axi_awsize),
    .s_axi_awburst(s_axi_awburst),
    .s_axi_awvalid(s_axi_awvalid),
    .s_axi_awready(s_axi_awready),
    .s_axi_wdata(s_axi_wdata),
    .s_axi_wstrb(s_axi_wstrb),
    .s_axi_wlast(s_axi_wlast),
    .s_axi_wvalid(s_axi_wvalid),
    .s_axi_wready(s_axi_wready),
    .s_axi_bid(s_axi_bid),
    .s_axi_bresp(s_axi_bresp),
    .s_axi_bvalid(s_axi_bvalid),
    .s_axi_bready(s_axi_bready),
    .s_axi_arid(s_axi_arid),
    .s_axi_araddr(s_axi_araddr),
    .s_axi_arlen(s_axi_arlen),
    .s_axi_arsize(s_axi_arsize),
    .s_axi_arburst(s_axi_arburst),
    .s_axi_arvalid(s_axi_arvalid),
    .s_axi_arready(s_axi_arready),
    .s_axi_rid(s_axi_rid),
    .s_axi_rdata(s_axi_rdata),
    .s_axi_rresp(s_axi_rresp),
    .s_axi_rlast(s_axi_rlast),
    .s_axi_rvalid(s_axi_rvalid),
    .s_axi_rready(s_axi_rready),

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

    .dma_start(dma_start),
    .dma_src_addr(dma_src_addr),
    .dma_dst_addr(dma_dst_addr),
    .dma_length(dma_length),
    .dma_dir(dma_dir),
    .dma_done(dma_done),
    .dma_error(dma_error),

    .msi_irq(msi_irq),
    .msi_vector(msi_vector),
    .msix_irq(msix_irq),
    .msix_vector(msix_vector),
    .intx_assert(intx_assert),

    .link_up(link_up),
    .ltssm_state(ltssm_state),
    .negotiated_gen(negotiated_gen),
    .negotiated_width(negotiated_width),
    .cfg_err_cor(cfg_err_cor),
    .cfg_err_nonfatal(cfg_err_nonfatal),
    .cfg_err_fatal(cfg_err_fatal),
    .max_payload_size(max_payload_size),
    .max_read_req_size(max_read_req_size),
    .eq_phase(eq_phase),
    .dll_link_active(dll_link_active)
  );

endmodule
