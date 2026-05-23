// =============================================================================
// Directed tests 27–29 (MSI-X table walk, LTSSM recovery, per-TLP relaxed order)
// Included from tb_pcie_top.sv
// =============================================================================

  localparam int TB_DW_MSIX_CAP_HDR   = 40;
  localparam int TB_DW_MSIX_TABLE_OFF = 41;
  localparam int TB_DW_MSIX_PBA_OFF   = 42;
  localparam int TB_MSIX_TB_BASE_DW   = 128;
  localparam int TB_MSIX_TB_PBA_DW    = 144;

  task tb_cfg_wr;
    input logic [11:0] addr;
    input logic [31:0] data;
    begin
      force dut.u_cfg_space.cfg_wr_valid = 1'b1;
      force dut.u_cfg_space.cfg_wr_addr  = addr;
      force dut.u_cfg_space.cfg_wr_data  = data;
      @(posedge core_clk);
      release dut.u_cfg_space.cfg_wr_valid;
      release dut.u_cfg_space.cfg_wr_addr;
      release dut.u_cfg_space.cfg_wr_data;
    end
  endtask

  task tb_msix_enable;
    logic [31:0] cap;
    begin
      cap = dut.u_cfg_space.cfg_space[TB_DW_MSIX_CAP_HDR];
      tb_cfg_wr(TB_DW_MSIX_CAP_HDR[11:0], cap | 32'h8000_0000);
    end
  endtask

  task tb_trigger_lane_loss;
    begin
      force u_rc_bfm.rc_tx_valid[0] = 1'b0;
      repeat (8) @(posedge core_clk);
      release u_rc_bfm.rc_tx_valid[0];
    end
  endtask

task automatic test_27_msix_table_walk;
  logic [31:0] tbl_off, pba_off, entry_data;
  begin
    $display("\n--- TEST 27: MSI-X Table Walk ---");
    wait_clocks(20);
    tbl_off = dut.u_cfg_space.cfg_space[TB_DW_MSIX_TABLE_OFF];
    pba_off = dut.u_cfg_space.cfg_space[TB_DW_MSIX_PBA_OFF];
    check("MSI-X table offset 0x4000", tbl_off[31:0] === 32'h0000_4000);
    check("MSI-X PBA offset 0x14000", pba_off[31:0] === 32'h0001_4000);
    tb_cfg_wr(12'(TB_MSIX_TB_BASE_DW + 0), 32'hFEE0_0000);
    tb_cfg_wr(12'(TB_MSIX_TB_BASE_DW + 1), 32'h0000_0000);
    tb_cfg_wr(12'(TB_MSIX_TB_BASE_DW + 2), 32'h0000_0042);
    tb_cfg_wr(12'(TB_MSIX_TB_BASE_DW + 3), 32'h0000_0000);
    tb_cfg_wr(12'(TB_MSIX_TB_BASE_DW + 4), 32'hFEE0_0000);
    tb_cfg_wr(12'(TB_MSIX_TB_BASE_DW + 5), 32'h0000_0000);
    tb_cfg_wr(12'(TB_MSIX_TB_BASE_DW + 6), 32'h0000_0043);
    tb_cfg_wr(12'(TB_MSIX_TB_BASE_DW + 7), 32'h0000_0000);
    entry_data = dut.u_cfg_space.msix_table[6];
    check("MSI-X table entry 1 data", entry_data === 32'h0000_0043);
    tb_cfg_wr(TB_DW_MSI_CAP_HDR[11:0], dut.u_cfg_space.cfg_space[TB_DW_MSI_CAP_HDR] & ~32'h0001_0000);
    tb_msix_enable();
    repeat (4) @(posedge core_clk);
    tb_cfg_wr(12'd144, 32'h0000_0002);
    intx_assert <= 1'b1;
    @(posedge core_clk);
    check("MSI-X vector 1 from PBA pending", msix_vector === 11'd1);
    check("MSI-X IRQ asserted", msix_irq === 1'b1);
    intx_assert <= 1'b0;
    tb_cfg_wr(12'd144, 32'h0000_0000);
    @(posedge core_clk);
    check("MSI-X IRQ deasserted", msix_irq === 1'b0);
  end
endtask

task automatic test_28_ltssm_recovery;
  int i;
  logic recovery_seen;
  begin
    $display("\n--- TEST 28: LTSSM Recovery ---");
    wait_clocks(50);
    check("Link in L0 before recovery", ltssm_state === L0 && link_up);
    tb_sim_fast_recovery_en = 1'b1;
    recovery_seen = 1'b0;
    tb_trigger_lane_loss();
    for (i = 0; i < 20000; i++) begin
      @(posedge core_clk);
      if (ltssm_state == RECOVERY_RCVRLOCK ||
          ltssm_state == RECOVERY_RCVRCFG  ||
          ltssm_state == RECOVERY_IDLE)
        recovery_seen = 1'b1;
      if (ltssm_state === L0 && link_up && recovery_seen)
        i = 20000;
    end
    check("Recovery state entered", recovery_seen);
    check("Link up after recovery", link_up);
    check("Returned to L0 after recovery", ltssm_state === L0);
    tb_sim_fast_recovery_en = 1'b0;
  end
endtask

task automatic test_29_tlp_relaxed_order_attr;
  int tlp_mid, i;
  logic np_wait;
  logic block_seen;
  logic send_seen;
  begin
    $display("\n--- TEST 29: Per-TLP Relaxed Ordering ---");
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
    check("NP outstanding for TLP-RO test", np_wait);
    tlp_mid = u_rc_bfm.tlp_rx_count;
    tb_np_relaxed_order_en = 1'b0;
    u_axi_bfm.cmd_addr = 64'h0000_0000_E000_5000;
    u_axi_bfm.cmd_id   = AXI_ID_W'(30);
    u_axi_bfm.axi_read_issue_only();
    block_seen = 1'b0;
    for (i = 0; i < 2000; i++) begin
      @(posedge core_clk);
      if (dut.u_tlp_tx.block_np_axi)
        block_seen = 1'b1;
    end
    check("NP blocked without TLP relaxed attr",
          block_seen || (u_rc_bfm.tlp_rx_count == tlp_mid));
    check("Bridge holding NP in ARD_SPLIT", dut.u_axi_bridge.ard_state == 1);
    tb_np_relaxed_order_en = 1'b1;
    repeat (4) @(posedge core_clk);
    send_seen = 1'b0;
    for (i = 0; i < 2000; i++) begin
      @(posedge core_clk);
      if (dut.u_axi_bridge.tlp_np_valid && !dut.u_tlp_tx.block_np_axi)
        send_seen = 1'b1;
      if (dut.u_axi_bridge.ard_state == 2)
        send_seen = 1'b1;
      if (u_rc_bfm.tlp_rx_count > tlp_mid)
        send_seen = 1'b1;
    end
    check("NP sent with TLP relaxed-order attr", send_seen);
    tb_np_relaxed_order_en = 1'b0;
    u_rc_bfm.auto_cpld_en = 1'b1;
    tb_finish_dma_h2d();
    tb_wait_np_idle();
  end
endtask

task automatic run_extended_tests_27_29;
  test_27_msix_table_walk();
  test_28_ltssm_recovery();
  test_29_tlp_relaxed_order_attr();
endtask
