// =============================================================================
// Directed tests 24–26 (AER RW1C, relaxed ordering, config capability walk)
// Included from tb_pcie_top.sv
// =============================================================================

  localparam int TB_DW_PCIE_DEV_CTL   = 18;
  localparam int TB_DW_MSI_CAP_HDR    = 36;
  localparam int TB_DW_AER_CAP_HDR    = 64;
  localparam int TB_DW_AER_UNCORR_STS = 65;
  localparam int TB_DW_AER_CORR_STS   = 68;

  task tb_pulse_cfg_err_cor;
    force dut.u_cpl_timeout.cfg_err_cor = 1'b1;
    @(posedge core_clk);
    release dut.u_cpl_timeout.cfg_err_cor;
  endtask

  task tb_pulse_cfg_err_nonfatal;
    force dut.u_cfg_space.err_nonfatal = 1'b1;
    @(posedge core_clk);
    release dut.u_cfg_space.err_nonfatal;
  endtask

  task tb_start_dma_h2d_np;
    begin
      dma_src_addr <= 64'hF000_0000_0000_0000;
      dma_dst_addr <= 64'h0000_0002_0000_0000;
      dma_length   <= 32'd8;
      dma_dir      <= 1'b0;
      @(posedge core_clk);
      dma_start    <= 1'b1;
      @(posedge core_clk);
      dma_start    <= 1'b0;
    end
  endtask

  task tb_finish_dma_h2d;
    int i;
    logic inj_done;
    begin
      inj_done = 1'b0;
      for (i = 0; i < 20000; i++) begin
        @(posedge core_clk);
        if (dma_h2d_wait_sig && !inj_done) begin
          u_rc_bfm.inject_cpld_for_tag(dut.gen_dma.u_dma.dma_tag);
          inj_done = 1'b1;
        end
        if (dma_done || dma_error)
          i = 20000;
      end
    end
  endtask

  task tb_wait_np_idle;
    int i;
    begin
      for (i = 0; i < 12000; i++) begin
        @(posedge core_clk);
        if (dut.tlp_np_outstanding == 0)
          i = 12000;
      end
    end
  endtask

  task tb_aer_cor_rw1c;
    force dut.u_cfg_space.cfg_wr_valid = 1'b1;
    force dut.u_cfg_space.cfg_wr_addr  = 12'd68;
    force dut.u_cfg_space.cfg_wr_data  = 32'h0000_0001;
    @(posedge core_clk);
    release dut.u_cfg_space.cfg_wr_valid;
    release dut.u_cfg_space.cfg_wr_addr;
    release dut.u_cfg_space.cfg_wr_data;
  endtask

  task tb_aer_unc_rw1c;
    force dut.u_cfg_space.cfg_wr_valid = 1'b1;
    force dut.u_cfg_space.cfg_wr_addr  = 12'd65;
    force dut.u_cfg_space.cfg_wr_data  = 32'h0000_2000;
    @(posedge core_clk);
    release dut.u_cfg_space.cfg_wr_valid;
    release dut.u_cfg_space.cfg_wr_addr;
    release dut.u_cfg_space.cfg_wr_data;
  endtask

  task tb_devctl_clear_relaxed;
    force dut.u_cfg_space.cfg_wr_valid = 1'b1;
    force dut.u_cfg_space.cfg_wr_addr  = 12'd18;
    force dut.u_cfg_space.cfg_wr_data  = 32'h0000_2800;
    @(posedge core_clk);
    release dut.u_cfg_space.cfg_wr_valid;
    release dut.u_cfg_space.cfg_wr_addr;
    release dut.u_cfg_space.cfg_wr_data;
  endtask

  task tb_devctl_set_relaxed;
    force dut.u_cfg_space.cfg_wr_valid = 1'b1;
    force dut.u_cfg_space.cfg_wr_addr  = 12'd18;
    force dut.u_cfg_space.cfg_wr_data  = 32'h0000_2810;
    @(posedge core_clk);
    release dut.u_cfg_space.cfg_wr_valid;
    release dut.u_cfg_space.cfg_wr_addr;
    release dut.u_cfg_space.cfg_wr_data;
  endtask

task automatic test_24_aer_rw1c;
  logic [31:0] aer_cor, aer_unc;
  begin
    $display("\n--- TEST 24: AER RW1C ---");
    wait_clocks(50);
    tb_pulse_cfg_err_cor();
    repeat (4) @(posedge core_clk);
    aer_cor = dut.u_cfg_space.cfg_space[TB_DW_AER_CORR_STS];
    check("AER correctable status bit set", aer_cor[0] === 1'b1);
    tb_aer_cor_rw1c();
    @(posedge core_clk);
    aer_cor = dut.u_cfg_space.cfg_space[TB_DW_AER_CORR_STS];
    check("AER correctable RW1C clears status", aer_cor[0] === 1'b0);
    tb_pulse_cfg_err_nonfatal();
    repeat (4) @(posedge core_clk);
    aer_unc = dut.u_cfg_space.cfg_space[TB_DW_AER_UNCORR_STS];
    check("AER uncorrectable status bit set", aer_unc[13] === 1'b1);
    tb_aer_unc_rw1c();
    repeat (4) @(posedge core_clk);
    aer_unc = dut.u_cfg_space.cfg_space[TB_DW_AER_UNCORR_STS];
    check("AER uncorrectable RW1C clears status", aer_unc[13] === 1'b0);
  end
endtask

task automatic test_25_relaxed_ordering;
  int tlp_mid, tlp_c;
  int i;
  logic np_wait;
  logic block_seen;
  logic send_seen;
  begin
    $display("\n--- TEST 25: Relaxed Ordering ---");
    wait_clocks(50);
    u_rc_bfm.auto_cpld_en = 1'b1;
    tb_wait_np_idle();
    u_rc_bfm.auto_cpld_en = 1'b0;
    tb_devctl_clear_relaxed();
    repeat (8) @(posedge core_clk);
    check("Device relaxed order disabled", dut.u_cfg_space.relaxed_order_en === 1'b0);
    tb_start_dma_h2d_np();
    np_wait = 1'b0;
    for (i = 0; i < 8000; i++) begin
      @(posedge core_clk);
      if (dma_h2d_wait_sig && dut.tlp_np_outstanding != 0)
        np_wait = 1'b1;
    end
    check("NP outstanding before ordering check", np_wait);
    tlp_mid = u_rc_bfm.tlp_rx_count;
    u_axi_bfm.cmd_addr = 64'h0000_0000_E000_2000;
    u_axi_bfm.cmd_id   = AXI_ID_W'(21);
    u_axi_bfm.axi_read_issue_only();
    block_seen = 1'b0;
    for (i = 0; i < 2000; i++) begin
      @(posedge core_clk);
      if (dut.u_tlp_tx.block_np_axi)
        block_seen = 1'b1;
    end
    check("Second NP blocked without relaxed order",
          block_seen || (u_rc_bfm.tlp_rx_count == tlp_mid));
    tb_devctl_set_relaxed();
    repeat (8) @(posedge core_clk);
    u_rc_bfm.auto_cpld_en = 1'b1;
    tb_finish_dma_h2d();
    tb_wait_np_idle();
    u_rc_bfm.auto_cpld_en = 1'b0;
    tb_devctl_set_relaxed();
    repeat (8) @(posedge core_clk);
    check("Device relaxed order enabled", dut.u_cfg_space.relaxed_order_en === 1'b1);
    tb_start_dma_h2d_np();
    np_wait = 1'b0;
    for (i = 0; i < 8000; i++) begin
      @(posedge core_clk);
      if (dma_h2d_wait_sig && dut.tlp_np_outstanding != 0)
        np_wait = 1'b1;
    end
    check("NP outstanding before relaxed-order pass", np_wait);
    tlp_mid = u_rc_bfm.tlp_rx_count;
    u_axi_bfm.cmd_addr = 64'h0000_0000_E000_4000;
    u_axi_bfm.cmd_id   = AXI_ID_W'(23);
    u_axi_bfm.axi_read_issue_only();
    send_seen = 1'b0;
    for (i = 0; i < 4000; i++) begin
      @(posedge core_clk);
      if (u_rc_bfm.tlp_rx_count > tlp_mid)
        send_seen = 1'b1;
      if (dut.u_axi_bridge.ard_state == 2 && dut.tlp_np_outstanding >= 2)
        send_seen = 1'b1;
    end
    tlp_c = u_rc_bfm.tlp_rx_count;
    u_rc_bfm.auto_cpld_en = 1'b1;
    tb_finish_dma_h2d();
    check("Second NP sent with device relaxed order", send_seen || (tlp_c > tlp_mid));
  end
endtask

task automatic test_26_config_capability_walk;
  logic [31:0] vid_did, aer_cap, msi_cap;
  begin
    $display("\n--- TEST 26: Config Capability Walk ---");
    wait_clocks(20);
    vid_did = dut.u_cfg_space.cfg_space[0];
    check("Vendor ID CAFE", vid_did[15:0] === 16'hCAFE);
    check("Device ID 0001", vid_did[31:16] === 16'h0001);
    aer_cap = dut.u_cfg_space.cfg_space[TB_DW_AER_CAP_HDR];
    check("AER extended capability present", aer_cap[15:0] === 16'h0001);
    msi_cap = dut.u_cfg_space.cfg_space[TB_DW_MSI_CAP_HDR];
    check("MSI capability ID 05", msi_cap[7:0] === 8'h05);
  end
endtask

task automatic run_advanced_tests_24_26;
  test_24_aer_rw1c();
  test_25_relaxed_ordering();
  test_26_config_capability_walk();
endtask
