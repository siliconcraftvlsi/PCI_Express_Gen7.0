// -----------------------------------------------------------------------------
// Author  : Robert Kingsly Amalathas
// Email   : robertk@microprocessorlab.com
// LinkedIn: www.linkedin.com/in/robertkingslya
// -----------------------------------------------------------------------------
// =============================================================================
// PCIe 7.0 Controller - Top Level Module
// Based on PCI Express 7.0 Specification 
// =============================================================================
// Description:
//   Top-level integration of the PCIe 7.0 Controller. Instantiates:
//     - LTSSM (Link Training and Status State Machine)
//     - PIPE Interface Adapter
//     - Transaction Layer (TLP TX / TLP RX)
//     - Data Link Layer (TX / RX with ACK/NAK and Retry Buffer)
//     - Flow Control Manager
//     - AXI Bridge (Manager + Subordinate)
//     - Configuration Space
//     - DMA Engine
//
//   Supports:
//     - PCIe 5.0 / 6.x / 7.0 (2.5 to 128 GT/s)
//     - x1 to x16 lane widths
//     - Endpoint (EP), Root Port (RP), Dual Mode (DM), Switch Port (SW)
//     - AMBA AXI3/4/4-Lite application interface (256-bit default)
//     - MSI / MSI-X interrupts
//     - AER, ECRC, ECC, SR-IOV
// =============================================================================

`include "pcie_pkg.sv"

module pcie_controller_top
  import pcie_pkg::*;
#(
  // -------------------------------------------------------------------------
  // Configuraton Parameters
  // -------------------------------------------------------------------------
  parameter  pcie_role_e   DEVICE_ROLE        = ROLE_EP,
  parameter  pcie_gen_e    MAX_GEN            = PCIE_GEN7,
  parameter  int unsigned  NUM_LANES          = 16,
  parameter  int unsigned  DATA_W             = 256,   // Internal / AXI data width
  parameter  int unsigned  ADDR_W             = 64,
  parameter  int unsigned  AXI_ID_W           = 8,
  parameter  int unsigned  PIPE_W             = 32,    // PIPE interface data width per lane
  // PCIe IDs
  parameter  logic [15:0]  VENDOR_ID          = 16'hCAFE,
  parameter  logic [15:0]  DEVICE_ID          = 16'h0001,
  parameter  logic [7:0]   REVISION_ID        = 8'h01,
  parameter  logic [23:0]  CLASS_CODE         = 24'h0C0300,  // Serial Bus / USB?  Override as needed
  parameter  logic [7:0]   BAR0_APERTURE      = 8'd24,       // 2^24 = 16 MB
  // Feature enables
  parameter  bit           EN_MSI             = 1,
  parameter  bit           EN_MSIX            = 1,
  parameter  bit           EN_SR_IOV          = 0,
  parameter  bit           EN_AER             = 1,
  parameter  bit           EN_ECRC            = 1,
  parameter  bit           EN_IDE             = 0,
  parameter  bit           EN_DMA             = 1,
  parameter  int unsigned  DMA_CHANNELS       = 4
)(
  // -------------------------------------------------------------------------
  // Clock & Reset
  // -------------------------------------------------------------------------
  input  logic                    core_clk,        // Core/application clock
  input  logic                    core_rst_n,      // Active-low async reset
  input  logic                    pipe_clk,        // PIPE reference clock (from PHY)
  input  logic                    aux_clk,         // Auxiliary clock for PM

  // -------------------------------------------------------------------------
  // PIPE Interface (to/from SerDes / PHY)
  // -------------------------------------------------------------------------
  // TX
  output logic [NUM_LANES-1:0][PIPE_W-1:0] pipe_tx_data,
  output logic [NUM_LANES-1:0][PIPE_W/8-1:0] pipe_tx_datak,
  output logic [NUM_LANES-1:0]             pipe_tx_elec_idle,
  output logic [NUM_LANES-1:0]             pipe_tx_compliance,
  output logic [NUM_LANES-1:0]             pipe_tx_deemph,
  output logic [NUM_LANES-1:0][2:0]        pipe_tx_margin,
  output logic [NUM_LANES-1:0]             pipe_tx_swing,
  output logic [NUM_LANES-1:0][1:0]        pipe_tx_eq_ctrl,
  // RX
  input  logic [NUM_LANES-1:0][PIPE_W-1:0] pipe_rx_data,
  input  logic [NUM_LANES-1:0][PIPE_W/8-1:0] pipe_rx_datak,
  input  logic [NUM_LANES-1:0]             pipe_rx_valid,
  input  logic [NUM_LANES-1:0]             pipe_rx_elec_idle,
  input  logic [NUM_LANES-1:0]             pipe_rx_status_valid,
  input  logic [NUM_LANES-1:0][2:0]        pipe_rx_status,
  // Link Status
  output logic [3:0]                        pipe_power_down,
  output logic                              pipe_reset_n,
  output logic [3:0]                        pipe_rate,       // Speed select
  output logic [1:0]                        pipe_width,      // Width select
  input  logic                              pipe_clk_req_n,

  // -------------------------------------------------------------------------
  // AXI4 Subordinate (Slave) Interface  –  Inbound AXI→PCIe
  // -------------------------------------------------------------------------
  input  logic [AXI_ID_W-1:0]    s_axi_awid,
  input  logic [ADDR_W-1:0]      s_axi_awaddr,
  input  logic [7:0]             s_axi_awlen,
  input  logic [2:0]             s_axi_awsize,
  input  logic [1:0]             s_axi_awburst,
  input  logic                   s_axi_awvalid,
  output logic                   s_axi_awready,
  input  logic [DATA_W-1:0]      s_axi_wdata,
  input  logic [DATA_W/8-1:0]    s_axi_wstrb,
  input  logic                   s_axi_wlast,
  input  logic                   s_axi_wvalid,
  output logic                   s_axi_wready,
  output logic [AXI_ID_W-1:0]    s_axi_bid,
  output logic [1:0]             s_axi_bresp,
  output logic                   s_axi_bvalid,
  input  logic                   s_axi_bready,
  input  logic [AXI_ID_W-1:0]    s_axi_arid,
  input  logic [ADDR_W-1:0]      s_axi_araddr,
  input  logic [7:0]             s_axi_arlen,
  input  logic [2:0]             s_axi_arsize,
  input  logic [1:0]             s_axi_arburst,
  input  logic                   s_axi_arvalid,
  output logic                   s_axi_arready,
  output logic [AXI_ID_W-1:0]    s_axi_rid,
  output logic [DATA_W-1:0]      s_axi_rdata,
  output logic [1:0]             s_axi_rresp,
  output logic                   s_axi_rlast,
  output logic                   s_axi_rvalid,
  input  logic                   s_axi_rready,

  // -------------------------------------------------------------------------
  // AXI4 Manager (Master) Interface  –  Outbound PCIe→AXI (completions in)
  // -------------------------------------------------------------------------
  output logic [AXI_ID_W-1:0]    m_axi_awid,
  output logic [ADDR_W-1:0]      m_axi_awaddr,
  output logic [7:0]             m_axi_awlen,
  output logic [2:0]             m_axi_awsize,
  output logic [1:0]             m_axi_awburst,
  output logic                   m_axi_awvalid,
  input  logic                   m_axi_awready,
  output logic [DATA_W-1:0]      m_axi_wdata,
  output logic [DATA_W/8-1:0]    m_axi_wstrb,
  output logic                   m_axi_wlast,
  output logic                   m_axi_wvalid,
  input  logic                   m_axi_wready,
  input  logic [AXI_ID_W-1:0]    m_axi_bid,
  input  logic [1:0]             m_axi_bresp,
  input  logic                   m_axi_bvalid,
  output logic                   m_axi_bready,
  output logic [AXI_ID_W-1:0]    m_axi_arid,
  output logic [ADDR_W-1:0]      m_axi_araddr,
  output logic [7:0]             m_axi_arlen,
  output logic [2:0]             m_axi_arsize,
  output logic [1:0]             m_axi_arburst,
  output logic                   m_axi_arvalid,
  input  logic                   m_axi_arready,
  input  logic [AXI_ID_W-1:0]    m_axi_rid,
  input  logic [DATA_W-1:0]      m_axi_rdata,
  input  logic [1:0]             m_axi_rresp,
  input  logic                   m_axi_rlast,
  input  logic                   m_axi_rvalid,
  output logic                   m_axi_rready,

  // -------------------------------------------------------------------------
  // DMA Interface (when EN_DMA=1)
  // -------------------------------------------------------------------------
  input  logic                   dma_start,
  input  logic [ADDR_W-1:0]      dma_src_addr,
  input  logic [ADDR_W-1:0]      dma_dst_addr,
  input  logic [31:0]            dma_length,
  input  logic                   dma_dir,         // 0=host→device, 1=device→host
  output logic                   dma_done,
  output logic                   dma_error,

  // -------------------------------------------------------------------------
  // Interrupt Interface
  // -------------------------------------------------------------------------
  output logic                   msi_irq,
  output logic [4:0]             msi_vector,
  output logic                   msix_irq,
  output logic [10:0]            msix_vector,
  input  logic                   intx_assert,     // Legacy INTx assertion

  // -------------------------------------------------------------------------
  // Status & Debug
  // -------------------------------------------------------------------------
  output logic                   link_up,
  output ltssm_state_e           ltssm_state,
  output pcie_gen_e              negotiated_gen,
  output logic [4:0]             negotiated_width,
  output logic                   cfg_err_cor,
  output logic                   cfg_err_nonfatal,
  output logic                   cfg_err_fatal,
  output logic [2:0]             max_payload_size,
  output logic [2:0]             max_read_req_size
);

  // ===========================================================================
  // Internal Wire Declarations
  // ===========================================================================

  // LTSSM → PIPE
  logic        ltssm_reset_n;
  logic [3:0]  ltssm_pipe_rate;
  logic [1:0]  ltssm_pipe_width;
  logic [3:0]  ltssm_power_down;
  logic        ltssm_link_up;
  ltssm_state_e ltssm_cur_state;
  pcie_gen_e   ltssm_neg_gen;
  logic [4:0]  ltssm_neg_width;

  // PIPE → DLL/TL
  logic [NUM_LANES-1:0][PIPE_W-1:0] phy_rx_data;
  logic [NUM_LANES-1:0][PIPE_W/8-1:0] phy_rx_datak;
  logic [NUM_LANES-1:0]             phy_rx_valid;
  logic [NUM_LANES-1:0][PIPE_W-1:0] phy_tx_data;
  logic [NUM_LANES-1:0][PIPE_W/8-1:0] phy_tx_datak;
  logic [NUM_LANES-1:0]             phy_tx_elec_idle;

  // DLL TX → Physical Scrambler / PIPE
  logic [DATA_W-1:0]  dll_tx_data;
  logic               dll_tx_valid;
  logic               dll_tx_ready;
  logic               dll_tx_start_of_tlp;
  logic               dll_tx_end_of_tlp;

  // DLL RX ← Physical Descrambler / PIPE
  logic [DATA_W-1:0]  dll_rx_data;
  logic               dll_rx_valid;
  logic               dll_rx_start_of_tlp;
  logic               dll_rx_end_of_tlp;
  logic               dll_rx_error;

  // TL TX → DLL TX
  logic [DATA_W-1:0]  tl_tx_data;
  logic               tl_tx_valid;
  logic               tl_tx_ready;
  logic               tl_tx_sop;
  logic               tl_tx_eop;
  logic [3:0]         tl_tx_be;

  // TL RX ← DLL RX
  logic [DATA_W-1:0]  tl_rx_data;
  logic               tl_rx_valid;
  logic               tl_rx_sop;
  logic               tl_rx_eop;
  logic               tl_rx_error;

  // Flow Control (TL ↔ FC Manager)
  fc_credits_t  fc_init_credits_p, fc_init_credits_np, fc_init_credits_cpl;
  fc_credits_t  fc_avail_p, fc_avail_np, fc_avail_cpl;
  logic         fc_update_tx;
  fc_credits_t  fc_consumed_p, fc_consumed_np, fc_consumed_cpl;

  // AXI Bridge ↔ TL
  // Posted (writes)
  logic [DATA_W-1:0]  axibr_tx_data;
  logic               axibr_tx_valid;
  logic               axibr_tx_ready;
  logic               axibr_tx_sop;
  logic               axibr_tx_eop;

  // Non-posted (reads)
  logic [DATA_W-1:0]  axibr_np_data;
  logic               axibr_np_valid;
  logic               axibr_np_ready;
  logic               axibr_np_sop;
  logic               axibr_np_eop;

  // Completion path to AXI bridge
  logic [DATA_W-1:0]  cpl_rx_data;
  logic               cpl_rx_valid;
  logic               cpl_rx_sop;
  logic               cpl_rx_eop;
  logic               cpl_rx_ready;

  // Config Space ↔ TL
  logic [DATA_W-1:0]  cfg_tx_data;
  logic               cfg_tx_valid;
  logic               cfg_tx_ready;
  logic               cfg_rx_valid;
  logic [31:0]        cfg_rx_data;
  logic [11:0]        cfg_rx_addr;
  logic               cfg_rx_wr;

  // DMA ↔ TL
  logic [DATA_W-1:0]  dma_tx_data;
  logic               dma_tx_valid;
  logic               dma_tx_ready;
  logic               dma_tx_sop;
  logic               dma_tx_eop;
  logic [DATA_W-1:0]  dma_rx_data;
  logic               dma_rx_valid;
  logic               dma_rx_sop;
  logic               dma_rx_eop;
  logic               dma_rx_ready;

  // Retry Buffer (DLL)
  logic               retry_nak_received;
  logic [11:0]        retry_ack_seq;
  logic [11:0]        retry_nak_seq;

  // Config space outputs
  logic [2:0]  cfg_mps;
  logic [2:0]  cfg_mrrs;
  logic        cfg_bus_master_en;
  logic        cfg_mem_space_en;

  // ===========================================================================
  // Module Instantiations
  // ===========================================================================

  // ---------------------------------------------------------------------------
  // LTSSM – Link Training and Status State Machine
  // ---------------------------------------------------------------------------
  pcie_ltssm #(
    .MAX_GEN   (MAX_GEN),
    .NUM_LANES (NUM_LANES)
  ) u_ltssm (
    .clk           (core_clk),
    .rst_n         (core_rst_n),
    .pipe_rx_valid       (pipe_rx_valid),
    .pipe_rx_elec_idle   (pipe_rx_elec_idle),
    .pipe_rx_status      (pipe_rx_status),
    .pipe_rx_status_valid(pipe_rx_status_valid),
    .pipe_clk_req_n      (pipe_clk_req_n),
    .link_up         (ltssm_link_up),
    .ltssm_state     (ltssm_cur_state),
    .negotiated_gen  (ltssm_neg_gen),
    .negotiated_width(ltssm_neg_width),
    .pipe_reset_n    (ltssm_reset_n),
    .pipe_rate       (ltssm_pipe_rate),
    .pipe_width      (ltssm_pipe_width),
    .pipe_power_down (ltssm_power_down)
  );

  assign link_up           = ltssm_link_up;
  assign ltssm_state       = ltssm_cur_state;
  assign negotiated_gen    = ltssm_neg_gen;
  assign negotiated_width  = ltssm_neg_width;
  assign pipe_reset_n      = ltssm_reset_n;
  assign pipe_rate         = ltssm_pipe_rate;
  assign pipe_width        = ltssm_pipe_width;
  assign pipe_power_down   = ltssm_power_down;

  // ---------------------------------------------------------------------------
  // PIPE Interface Adapter
  // ---------------------------------------------------------------------------
  pcie_pipe_if #(
    .NUM_LANES (NUM_LANES),
    .PIPE_W    (PIPE_W),
    .DATA_W    (DATA_W)
  ) u_pipe_if (
    .clk              (core_clk),
    .rst_n            (core_rst_n),
    .link_up          (ltssm_link_up),
    // To PHY
    .pipe_tx_data     (pipe_tx_data),
    .pipe_tx_datak    (pipe_tx_datak),
    .pipe_tx_elec_idle(pipe_tx_elec_idle),
    .pipe_tx_compliance(pipe_tx_compliance),
    .pipe_tx_deemph   (pipe_tx_deemph),
    .pipe_tx_margin   (pipe_tx_margin),
    .pipe_tx_swing    (pipe_tx_swing),
    .pipe_tx_eq_ctrl  (pipe_tx_eq_ctrl),
    // From PHY
    .pipe_rx_data     (pipe_rx_data),
    .pipe_rx_datak    (pipe_rx_datak),
    .pipe_rx_valid    (pipe_rx_valid),
    .pipe_rx_elec_idle(pipe_rx_elec_idle),
    // Internal TX (from DLL)
    .tx_data          (dll_tx_data),
    .tx_valid         (dll_tx_valid),
    .tx_ready         (dll_tx_ready),
    .tx_sop           (dll_tx_start_of_tlp),
    .tx_eop           (dll_tx_end_of_tlp),
    // Internal RX (to DLL)
    .rx_data          (dll_rx_data),
    .rx_valid         (dll_rx_valid),
    .rx_sop           (dll_rx_start_of_tlp),
    .rx_eop           (dll_rx_end_of_tlp),
    .rx_error         (dll_rx_error)
  );

  // ---------------------------------------------------------------------------
  // Data Link Layer TX  (TLP → DLLP wrapping, LCRC, sequence numbers)
  // ---------------------------------------------------------------------------
  pcie_dll_tx #(
    .DATA_W           (DATA_W),
    .RETRY_BUF_DEPTH  (RETRY_BUF_DEPTH)
  ) u_dll_tx (
    .clk              (core_clk),
    .rst_n            (core_rst_n),
    .link_up          (ltssm_link_up),
    // TLP from Transaction Layer
    .tl_tx_data       (tl_tx_data),
    .tl_tx_valid      (tl_tx_valid),
    .tl_tx_ready      (tl_tx_ready),
    .tl_tx_sop        (tl_tx_sop),
    .tl_tx_eop        (tl_tx_eop),
    // DLLP to PIPE
    .phy_tx_data      (dll_tx_data),
    .phy_tx_valid     (dll_tx_valid),
    .phy_tx_ready     (dll_tx_ready),
    .phy_tx_sop       (dll_tx_start_of_tlp),
    .phy_tx_eop       (dll_tx_end_of_tlp),
    // ACK/NAK from DLL RX
    .nak_received     (retry_nak_received),
    .ack_seq          (retry_ack_seq),
    .nak_seq          (retry_nak_seq)
  );

  // ---------------------------------------------------------------------------
  // Data Link Layer RX  (DLLP parsing, CRC check, ACK/NAK generation)
  // ---------------------------------------------------------------------------
  pcie_dll_rx #(
    .DATA_W (DATA_W)
  ) u_dll_rx (
    .clk              (core_clk),
    .rst_n            (core_rst_n),
    .link_up          (ltssm_link_up),
    // From PIPE
    .phy_rx_data      (dll_rx_data),
    .phy_rx_valid     (dll_rx_valid),
    .phy_rx_sop       (dll_rx_start_of_tlp),
    .phy_rx_eop       (dll_rx_end_of_tlp),
    .phy_rx_error     (dll_rx_error),
    // TLP to Transaction Layer
    .tl_rx_data       (tl_rx_data),
    .tl_rx_valid      (tl_rx_valid),
    .tl_rx_sop        (tl_rx_sop),
    .tl_rx_eop        (tl_rx_eop),
    .tl_rx_error      (tl_rx_error),
    // ACK/NAK feedback to DLL TX
    .nak_out          (retry_nak_received),
    .ack_seq_out      (retry_ack_seq),
    .nak_seq_out      (retry_nak_seq)
  );

  // ---------------------------------------------------------------------------
  // Flow Control Manager
  // ---------------------------------------------------------------------------
  pcie_flow_ctrl #(
    .DATA_W  (DATA_W),
    .NUM_VCS (NUM_VCS)
  ) u_flow_ctrl (
    .clk                (core_clk),
    .rst_n              (core_rst_n),
    .link_up            (ltssm_link_up),
    // Advertised credits (this side)
    .init_credits_p     (fc_init_credits_p),
    .init_credits_np    (fc_init_credits_np),
    .init_credits_cpl   (fc_init_credits_cpl),
    // Available credits toward remote
    .avail_p            (fc_avail_p),
    .avail_np           (fc_avail_np),
    .avail_cpl          (fc_avail_cpl),
    // Consumed credits
    .consumed_p         (fc_consumed_p),
    .consumed_np        (fc_consumed_np),
    .consumed_cpl       (fc_consumed_cpl),
    // FC DLLPs to/from DLL
    .fc_update_tx       (fc_update_tx)
  );

  // ---------------------------------------------------------------------------
  // Transaction Layer TX  (TLP assembly, mux from AXI-bridge, CFG, DMA)
  // ---------------------------------------------------------------------------
  pcie_tlp_tx #(
    .DATA_W    (DATA_W),
    .ADDR_W    (ADDR_W),
    .NUM_VCS   (NUM_VCS)
  ) u_tlp_tx (
    .clk              (core_clk),
    .rst_n            (core_rst_n),
    .link_up          (ltssm_link_up),
    // Flow credits
    .fc_avail_p       (fc_avail_p),
    .fc_avail_np      (fc_avail_np),
    .fc_avail_cpl     (fc_avail_cpl),
    .fc_consumed_p    (fc_consumed_p),
    .fc_consumed_np   (fc_consumed_np),
    .fc_consumed_cpl  (fc_consumed_cpl),
    // Inbound from AXI bridge (posted + non-posted)
    .axibr_tx_data    (axibr_tx_data),
    .axibr_tx_valid   (axibr_tx_valid),
    .axibr_tx_ready   (axibr_tx_ready),
    .axibr_tx_sop     (axibr_tx_sop),
    .axibr_tx_eop     (axibr_tx_eop),
    // Inbound from AXI bridge (non-posted reads)
    .axibr_np_data    (axibr_np_data),
    .axibr_np_valid   (axibr_np_valid),
    .axibr_np_ready   (axibr_np_ready),
    .axibr_np_sop     (axibr_np_sop),
    .axibr_np_eop     (axibr_np_eop),
    // Inbound from DMA
    .dma_tx_data      (dma_tx_data),
    .dma_tx_valid     (dma_tx_valid),
    .dma_tx_ready     (dma_tx_ready),
    .dma_tx_sop       (dma_tx_sop),
    .dma_tx_eop       (dma_tx_eop),
    // Config space completions
    .cfg_tx_data      (cfg_tx_data),
    .cfg_tx_valid     (cfg_tx_valid),
    .cfg_tx_ready     (cfg_tx_ready),
    // Config info
    .cfg_mps          (cfg_mps),
    .cfg_mrrs         (cfg_mrrs),
    // Out to DLL TX
    .tl_tx_data       (tl_tx_data),
    .tl_tx_valid      (tl_tx_valid),
    .tl_tx_ready      (tl_tx_ready),
    .tl_tx_sop        (tl_tx_sop),
    .tl_tx_eop        (tl_tx_eop)
  );

  // ---------------------------------------------------------------------------
  // Transaction Layer RX  (TLP parsing, routing to AXI bridge, CFG, DMA)
  // ---------------------------------------------------------------------------
  pcie_tlp_rx #(
    .DATA_W    (DATA_W),
    .ADDR_W    (ADDR_W),
    .DEVICE_ROLE (DEVICE_ROLE)
  ) u_tlp_rx (
    .clk              (core_clk),
    .rst_n            (core_rst_n),
    .link_up          (ltssm_link_up),
    // From DLL
    .tl_rx_data       (tl_rx_data),
    .tl_rx_valid      (tl_rx_valid),
    .tl_rx_sop        (tl_rx_sop),
    .tl_rx_eop        (tl_rx_eop),
    .tl_rx_error      (tl_rx_error),
    // Completions to AXI bridge
    .cpl_rx_data      (cpl_rx_data),
    .cpl_rx_valid     (cpl_rx_valid),
    .cpl_rx_sop       (cpl_rx_sop),
    .cpl_rx_eop       (cpl_rx_eop),
    .cpl_rx_ready     (cpl_rx_ready),
    // Received writes/reads to AXI bridge
    .axibr_rx_data    (axibr_tx_data),
    .axibr_rx_valid   (axibr_tx_valid),
    .axibr_rx_sop     (axibr_tx_sop),
    .axibr_rx_eop     (axibr_tx_eop),
    // Config space accesses
    .cfg_rx_valid     (cfg_rx_valid),
    .cfg_rx_data      (cfg_rx_data),
    .cfg_rx_addr      (cfg_rx_addr),
    .cfg_rx_wr        (cfg_rx_wr),
    // DMA completions
    .dma_rx_data      (dma_rx_data),
    .dma_rx_valid     (dma_rx_valid),
    .dma_rx_sop       (dma_rx_sop),
    .dma_rx_eop       (dma_rx_eop),
    .dma_rx_ready     (dma_rx_ready),
    // Error reporting
    .err_cor          (cfg_err_cor),
    .err_nonfatal     (cfg_err_nonfatal),
    .err_fatal        (cfg_err_fatal)
  );

  // ---------------------------------------------------------------------------
  // Configuration Space
  // ---------------------------------------------------------------------------
  pcie_cfg_space #(
    .DEVICE_ROLE  (DEVICE_ROLE),
    .VENDOR_ID    (VENDOR_ID),
    .DEVICE_ID    (DEVICE_ID),
    .REVISION_ID  (REVISION_ID),
    .CLASS_CODE   (CLASS_CODE),
    .BAR0_APERTURE(BAR0_APERTURE),
    .EN_MSI       (EN_MSI),
    .EN_MSIX      (EN_MSIX),
    .EN_AER       (EN_AER)
  ) u_cfg_space (
    .clk              (core_clk),
    .rst_n            (core_rst_n),
    // Write/Read access from TL RX
    .cfg_wr_valid     (cfg_rx_wr),
    .cfg_wr_addr      (cfg_rx_addr),
    .cfg_wr_data      (cfg_rx_data),
    .cfg_rd_valid     (cfg_rx_valid & ~cfg_rx_wr),
    .cfg_rd_addr      (cfg_rx_addr),
    // Completion data to TL TX
    .cfg_cpl_data     (cfg_tx_data),
    .cfg_cpl_valid    (cfg_tx_valid),
    .cfg_cpl_ready    (cfg_tx_ready),
    // Decoded config outputs
    .mps              (cfg_mps),
    .mrrs             (cfg_mrrs),
    .bus_master_en    (cfg_bus_master_en),
    .mem_space_en     (cfg_mem_space_en),
    // Interrupt control
    .msi_irq          (msi_irq),
    .msi_vector       (msi_vector),
    .msix_irq         (msix_irq),
    .msix_vector      (msix_vector),
    .intx_assert      (intx_assert),
    // Error signaling
    .err_cor          (cfg_err_cor),
    .err_nonfatal     (cfg_err_nonfatal),
    .err_fatal        (cfg_err_fatal)
  );

  assign max_payload_size  = cfg_mps;
  assign max_read_req_size = cfg_mrrs;

  // ---------------------------------------------------------------------------
  // AXI Bridge (Subordinate→TL and TL→Manager)
  // ---------------------------------------------------------------------------
  pcie_axi_bridge #(
    .DATA_W    (DATA_W),
    .ADDR_W    (ADDR_W),
    .AXI_ID_W  (AXI_ID_W)
  ) u_axi_bridge (
    .clk              (core_clk),
    .rst_n            (core_rst_n),
    .link_up          (ltssm_link_up),
    .cfg_mps          (cfg_mps),
    .cfg_mrrs         (cfg_mrrs),
    // AXI Subordinate (inbound)
    .s_axi_awid       (s_axi_awid),
    .s_axi_awaddr     (s_axi_awaddr),
    .s_axi_awlen      (s_axi_awlen),
    .s_axi_awsize     (s_axi_awsize),
    .s_axi_awburst    (s_axi_awburst),
    .s_axi_awvalid    (s_axi_awvalid),
    .s_axi_awready    (s_axi_awready),
    .s_axi_wdata      (s_axi_wdata),
    .s_axi_wstrb      (s_axi_wstrb),
    .s_axi_wlast      (s_axi_wlast),
    .s_axi_wvalid     (s_axi_wvalid),
    .s_axi_wready     (s_axi_wready),
    .s_axi_bid        (s_axi_bid),
    .s_axi_bresp      (s_axi_bresp),
    .s_axi_bvalid     (s_axi_bvalid),
    .s_axi_bready     (s_axi_bready),
    .s_axi_arid       (s_axi_arid),
    .s_axi_araddr     (s_axi_araddr),
    .s_axi_arlen      (s_axi_arlen),
    .s_axi_arsize     (s_axi_arsize),
    .s_axi_arburst    (s_axi_arburst),
    .s_axi_arvalid    (s_axi_arvalid),
    .s_axi_arready    (s_axi_arready),
    .s_axi_rid        (s_axi_rid),
    .s_axi_rdata      (s_axi_rdata),
    .s_axi_rresp      (s_axi_rresp),
    .s_axi_rlast      (s_axi_rlast),
    .s_axi_rvalid     (s_axi_rvalid),
    .s_axi_rready     (s_axi_rready),
    // AXI Manager (outbound)
    .m_axi_awid       (m_axi_awid),
    .m_axi_awaddr     (m_axi_awaddr),
    .m_axi_awlen      (m_axi_awlen),
    .m_axi_awsize     (m_axi_awsize),
    .m_axi_awburst    (m_axi_awburst),
    .m_axi_awvalid    (m_axi_awvalid),
    .m_axi_awready    (m_axi_awready),
    .m_axi_wdata      (m_axi_wdata),
    .m_axi_wstrb      (m_axi_wstrb),
    .m_axi_wlast      (m_axi_wlast),
    .m_axi_wvalid     (m_axi_wvalid),
    .m_axi_wready     (m_axi_wready),
    .m_axi_bid        (m_axi_bid),
    .m_axi_bresp      (m_axi_bresp),
    .m_axi_bvalid     (m_axi_bvalid),
    .m_axi_bready     (m_axi_bready),
    .m_axi_arid       (m_axi_arid),
    .m_axi_araddr     (m_axi_araddr),
    .m_axi_arlen      (m_axi_arlen),
    .m_axi_arsize     (m_axi_arsize),
    .m_axi_arburst    (m_axi_arburst),
    .m_axi_arvalid    (m_axi_arvalid),
    .m_axi_arready    (m_axi_arready),
    .m_axi_rid        (m_axi_rid),
    .m_axi_rdata      (m_axi_rdata),
    .m_axi_rresp      (m_axi_rresp),
    .m_axi_rlast      (m_axi_rlast),
    .m_axi_rvalid     (m_axi_rvalid),
    .m_axi_rready     (m_axi_rready),
    // TLP TX (posted writes + non-posted reads to PCIe)
    .tlp_tx_data      (axibr_np_data),
    .tlp_tx_valid     (axibr_np_valid),
    .tlp_tx_ready     (axibr_np_ready),
    .tlp_tx_sop       (axibr_np_sop),
    .tlp_tx_eop       (axibr_np_eop),
    // TLP RX completions from PCIe
    .cpl_rx_data      (cpl_rx_data),
    .cpl_rx_valid     (cpl_rx_valid),
    .cpl_rx_sop       (cpl_rx_sop),
    .cpl_rx_eop       (cpl_rx_eop),
    .cpl_rx_ready     (cpl_rx_ready)
  );

  // ---------------------------------------------------------------------------
  // DMA Engine (optional)
  // ---------------------------------------------------------------------------
  generate
    if (EN_DMA) begin : gen_dma
      pcie_dma #(
        .DATA_W       (DATA_W),
        .ADDR_W       (ADDR_W),
        .DMA_CHANNELS (DMA_CHANNELS)
      ) u_dma (
        .clk          (core_clk),
        .rst_n        (core_rst_n),
        .link_up      (ltssm_link_up),
        .cfg_mrrs     (cfg_mrrs),
        .cfg_mps      (cfg_mps),
        // Control
        .dma_start    (dma_start),
        .dma_src_addr (dma_src_addr),
        .dma_dst_addr (dma_dst_addr),
        .dma_length   (dma_length),
        .dma_dir      (dma_dir),
        .dma_done     (dma_done),
        .dma_error    (dma_error),
        // TLP TX
        .tlp_tx_data  (dma_tx_data),
        .tlp_tx_valid (dma_tx_valid),
        .tlp_tx_ready (dma_tx_ready),
        .tlp_tx_sop   (dma_tx_sop),
        .tlp_tx_eop   (dma_tx_eop),
        // TLP RX
        .tlp_rx_data  (dma_rx_data),
        .tlp_rx_valid (dma_rx_valid),
        .tlp_rx_sop   (dma_rx_sop),
        .tlp_rx_eop   (dma_rx_eop),
        .tlp_rx_ready (dma_rx_ready)
      );
    end else begin : gen_no_dma
      assign dma_done  = 1'b0;
      assign dma_error = 1'b0;
      assign dma_tx_data  = '0;
      assign dma_tx_valid = 1'b0;
      assign dma_tx_sop   = 1'b0;
      assign dma_tx_eop   = 1'b0;
      assign dma_rx_ready = 1'b1;
    end
  endgenerate

  // ---------------------------------------------------------------------------
  // FC Init credits — default generous values (can be overridden by parameterization)
  // ---------------------------------------------------------------------------
  assign fc_init_credits_p   = {12'd64,    20'd1024};
  assign fc_init_credits_np  = {12'd16,    20'd0};
  assign fc_init_credits_cpl = {12'hFFF,   20'hFFFFF};  // Infinite

endmodule : pcie_controller_top
