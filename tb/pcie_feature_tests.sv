// =============================================================================
// Directed feature tests 16–18 (PM, completion timeout, MSI-X)
// Included from tb_pcie_top.sv — simulate with:
//   make sim-strict              (full 34-check regression)
//   make sim-features            (link-up + tests 16–18 only)
//   make sim-features-strict
// =============================================================================

task automatic test_16_power_management;
  int i;
  logic l0s_seen;
  logic l1_seen;
  logic l1_handshake;
  begin
    l0s_seen     = 1'b0;
    l1_seen      = 1'b0;
    l1_handshake = 1'b0;
    $display("\n--- TEST 16: Power Management ---");
    wait_clocks(20);
    for (i = 0; i < 256; i++) begin
      @(posedge core_clk);
      if (dut.u_pm_ctrl.pm_state_l0s)
        l0s_seen = 1'b1;
    end
    check("PM L0s entered after TX idle", l0s_seen);
    force dut.u_pm_ctrl.sw_req_l1 = 1'b1;
    for (i = 0; i < 512; i++) begin
      @(posedge core_clk);
      if (dut.u_pm_ctrl.dllp_pm_enter_l1_req && !l1_handshake) begin
        l1_handshake = 1'b1;
        u_rc_bfm.inject_pm_req_ack_dllp();
      end
      if (dut.u_pm_ctrl.pm_state_l1)
        l1_seen = 1'b1;
    end
    if (!l1_seen && l1_handshake) begin
      @(posedge core_clk);
      force dut.u_pm_ctrl.dllp_pm_req_ack_rx = 1'b1;
      @(posedge core_clk);
      release dut.u_pm_ctrl.dllp_pm_req_ack_rx;
      repeat (8) @(posedge core_clk);
      if (dut.u_pm_ctrl.pm_state_l1)
        l1_seen = 1'b1;
    end
    release dut.u_pm_ctrl.sw_req_l1;
    check("PM L1 entered after PM_Req_Ack", l1_seen);
  end
endtask

task automatic test_17_completion_timeout;
  int   i;
  logic cor_seen;
  begin
    cor_seen = 1'b0;
    $display("\n--- TEST 17: Completion Timeout ---");
    wait_clocks(20);
    u_rc_bfm.auto_cpld_en = 1'b0;
    u_axi_bfm.cmd_addr    = 64'h0000_0000_A000_0000;
    u_axi_bfm.cmd_id      = AXI_ID_W'(9);
    u_axi_bfm.axi_read_issue_only();
    for (i = 0; i < 8000; i++) begin
      @(posedge core_clk);
      if (cfg_err_cor || dut.u_cpl_timeout.cfg_err_cor)
        cor_seen = 1'b1;
    end
    u_rc_bfm.auto_cpld_en = 1'b1;
    check("Completion timeout raised correctable error", cor_seen);
  end
endtask

task automatic test_18_msix_interrupt;
  begin
    $display("\n--- TEST 18: MSI-X Interrupt ---");
    wait_clocks(20);
    force dut.u_cfg_space.sim_int_override = 1'b1;
    force dut.u_cfg_space.sim_msi_en       = 1'b0;
    force dut.u_cfg_space.sim_msix_en      = 1'b1;
    intx_assert <= 1'b1;
    @(posedge core_clk);
    @(posedge core_clk);
    check("MSI-X IRQ asserted", msix_irq === 1'b1);
    check("MSI not asserted when MSI-X enabled", msi_irq === 1'b0);
    intx_assert <= 1'b0;
    @(posedge core_clk);
    check("MSI-X IRQ deasserted", msix_irq === 1'b0);
    release dut.u_cfg_space.sim_int_override;
    release dut.u_cfg_space.sim_msi_en;
    release dut.u_cfg_space.sim_msix_en;
  end
endtask

task automatic run_feature_tests_16_18;
  test_16_power_management();
  test_17_completion_timeout();
  test_18_msix_interrupt();
endtask
