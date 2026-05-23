`timescale 1ns/1ps
// Bind PCIe SVA checkers to pcie_controller_top.
// With +define+VERILATOR only delivery assertions are bound (lint-sva target).
// Questa/VCS builds without VERILATOR get all five assertion modules.

`include "pcie_pkg.sv"

`ifndef VERILATOR

bind pcie_controller_top pcie_ltssm_assertions #(
  .NUM_LANES(NUM_LANES)
) u_ltssm_sva (
  .clk               (core_clk),
  .rst_n             (core_rst_n),
  .ltssm_state       (ltssm_cur_state),
  .link_up           (link_up),
  .pipe_rate         (pipe_rate),
  .pipe_width        (pipe_width),
  .pipe_power_down   (pipe_power_down),
  .pipe_reset_n      (pipe_reset_n),
  .pipe_rx_valid     (pipe_rx_valid),
  .pipe_rx_elec_idle (pipe_rx_elec_idle),
  .negotiated_gen    (negotiated_gen),
  .negotiated_width  (negotiated_width)
);

bind pcie_controller_top pcie_dll_assertions #(
  .DATA_W(DATA_W)
) u_dll_sva (
  .clk               (core_clk),
  .rst_n             (core_rst_n),
  .link_up           (link_up),
  .tl_tx_data        (tl_tx_data),
  .tl_tx_valid       (tl_tx_valid),
  .tl_tx_ready       (tl_tx_ready),
  .tl_tx_sop         (tl_tx_sop),
  .tl_tx_eop         (tl_tx_eop),
  .phy_tx_data       (dll_tx_data),
  .phy_tx_valid      (dll_tx_valid),
  .phy_tx_sop        (dll_tx_start_of_tlp),
  .phy_tx_eop        (dll_tx_end_of_tlp),
  .dll_rx_data       (dll_rx_data),
  .dll_rx_valid      (dll_rx_valid),
  .dll_rx_sop        (dll_rx_start_of_tlp),
  .dll_rx_eop        (dll_rx_end_of_tlp),
  .dll_rx_error      (dll_rx_error),
  .nak_received      (retry_nak_received),
  .ack_seq           (retry_ack_seq),
  .nak_seq           (retry_nak_seq),
  .ack_seq_out_valid (1'b0),
  .ack_seq_out       (u_dll_rx.ack_seq_out),
  .nak_seq_out_valid (u_dll_rx.nak_out),
  .nak_seq_out       (u_dll_rx.nak_seq_out),
  .dllp_type         (fc_dllp_valid ? 8'(fc_dllp_type) : 8'(pm_dllp_type)),
  .dllp_valid        (fc_dllp_valid | pm_dllp_valid),
  .dll_tx_active     (dll_tx_active_sig),
  .dll_error         (dll_error_sig),
  .dllp_err          (dllp_err_sig),
  .fc_dllp_valid     (fc_dllp_valid),
  .fc_dllp_type      (fc_dllp_type),
  .fc_dllp_hdr       (fc_dllp_hdr),
  .fc_dllp_data      (fc_dllp_data),
  .pm_dllp_valid     (pm_dllp_valid),
  .pm_dllp_type      (pm_dllp_type)
);

bind pcie_controller_top pcie_fc_assertions u_fc_sva (
  .clk               (core_clk),
  .rst_n             (core_rst_n),
  .link_up           (link_up),
  .avail_p           (fc_avail_p),
  .avail_np          (fc_avail_np),
  .avail_cpl         (fc_avail_cpl),
  .consumed_p        (fc_consumed_p),
  .consumed_np       (fc_consumed_np),
  .consumed_cpl      (fc_consumed_cpl),
  .init_credits_p    (fc_init_credits_p),
  .init_credits_np   (fc_init_credits_np),
  .init_credits_cpl  (fc_init_credits_cpl),
  .fc_update_tx      (fc_update_tx),
  .fc_rx_valid       (fc_dllp_valid),
  .fc_rx_type        (fc_dllp_type),
  .fc_rx_hdr         (fc_dllp_hdr),
  .fc_rx_data        (fc_dllp_data),
  .fc_upd_type       (fc_upd_type_sig)
);

bind pcie_controller_top pcie_tlp_assertions #(
  .DATA_W(DATA_W),
  .ADDR_W(ADDR_W)
) u_tlp_sva (
  .clk               (core_clk),
  .rst_n             (core_rst_n),
  .link_up           (link_up),
  .tlp_tx_data       (tl_tx_data),
  .tlp_tx_valid      (tl_tx_valid),
  .tlp_tx_ready      (tl_tx_ready),
  .tlp_tx_sop        (tl_tx_sop),
  .tlp_tx_eop        (tl_tx_eop),
  .tlp_rx_data       (tl_rx_data),
  .tlp_rx_valid      (tl_rx_valid),
  .tlp_rx_sop        (tl_rx_sop),
  .tlp_rx_eop        (tl_rx_eop),
  .tlp_rx_error      (tl_rx_error),
  .dma_done          (dma_done),
  .dma_error         (dma_error),
  .s_axi_awvalid     (s_axi_awvalid),
  .s_axi_awready     (s_axi_awready),
  .s_axi_awaddr      (s_axi_awaddr),
  .s_axi_wvalid      (s_axi_wvalid),
  .s_axi_wready      (s_axi_wready),
  .s_axi_wdata       (s_axi_wdata),
  .s_axi_bvalid      (s_axi_bvalid),
  .s_axi_bready      (s_axi_bready),
  .s_axi_bresp       (s_axi_bresp),
  .s_axi_arvalid     (s_axi_arvalid),
  .s_axi_arready     (s_axi_arready),
  .s_axi_rvalid      (s_axi_rvalid),
  .s_axi_rready      (s_axi_rready),
  .s_axi_rdata       (s_axi_rdata),
  .s_axi_rresp       (s_axi_rresp),
  .pending_tag       (axibr_sva_pending_tag),
  .tag_valid         (axibr_sva_tag_valid),
  .cpl_received      (axibr_sva_cpl_received),
  .cpl_tag           (axibr_sva_cpl_tag),
  .cfg_err_cor       (cfg_err_cor),
  .cfg_err_nonfatal  (cfg_err_nonfatal),
  .cfg_err_fatal     (cfg_err_fatal)
);

`endif

bind pcie_controller_top pcie_delivery_assertions #(
  .DATA_W(DATA_W),
  .ADDR_W(ADDR_W)
) u_delivery_sva (
  .clk               (core_clk),
  .rst_n             (core_rst_n),
  .link_up           (link_up),
  .tlp_tx_data       (tl_tx_data),
  .tlp_tx_valid      (tl_tx_valid),
  .tlp_tx_ready      (tl_tx_ready),
  .tlp_tx_sop        (tl_tx_sop),
  .tlp_tx_eop        (tl_tx_eop),
  .tlp_rx_data       (tl_rx_data),
  .tlp_rx_valid      (tl_rx_valid),
  .tlp_rx_sop        (tl_rx_sop),
  .tlp_rx_eop        (tl_rx_eop),
  .tlp_rx_error      (tl_rx_error),
  .s_axi_awvalid     (s_axi_awvalid),
  .s_axi_awready     (s_axi_awready),
  .s_axi_awaddr      (s_axi_awaddr),
  .s_axi_wvalid      (s_axi_wvalid),
  .s_axi_wready      (s_axi_wready),
  .s_axi_wlast       (s_axi_wlast),
  .s_axi_bvalid      (s_axi_bvalid),
  .s_axi_bready      (s_axi_bready),
  .s_axi_bresp       (s_axi_bresp),
  .s_axi_arvalid     (s_axi_arvalid),
  .s_axi_arready     (s_axi_arready),
  .s_axi_araddr      (s_axi_araddr),
  .s_axi_rvalid      (s_axi_rvalid),
  .s_axi_rready      (s_axi_rready),
  .s_axi_rlast       (s_axi_rlast),
  .s_axi_rdata       (s_axi_rdata),
  .s_axi_rresp       (s_axi_rresp),
  .dma_done          (dma_done),
  .dma_error         (dma_error),
  .dllp_valid        (fc_dllp_valid | pm_dllp_valid),
  .dll_tx_active     (dll_tx_active_sig),
  .dll_error         (dll_error_sig),
  .cfg_err_cor       (cfg_err_cor),
  .cfg_err_nonfatal  (cfg_err_nonfatal),
  .cfg_err_fatal     (cfg_err_fatal),
  .fc_update_tx      (fc_update_tx)
);
