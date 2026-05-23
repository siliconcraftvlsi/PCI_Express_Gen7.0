// =============================================================================
// Directed tests 19–22 (+ optional FLIT test 23 with +define+PCIE_FLIT_TEST)
// Included from tb_pcie_top.sv
// =============================================================================

task automatic test_19_tlp_ordering;
  int   tlp_before, tlp_mid, tlp_after;
  int   i;
  logic block_seen;
  logic np_seen;
  begin
    $display("\n--- TEST 19: TLP Ordering (posted blocked by NP) ---");
    wait_clocks(20);
    block_seen  = 1'b0;
    np_seen     = 1'b0;
    tlp_before  = u_rc_bfm.tlp_rx_count;
    u_rc_bfm.auto_cpld_en = 1'b0;
    u_axi_bfm.cmd_addr    = 64'h0000_0000_C000_1000;
    u_axi_bfm.cmd_id      = AXI_ID_W'(10);
    u_axi_bfm.axi_read_issue_only();
    for (i = 0; i < 4000; i++) begin
      @(posedge core_clk);
      if (dut.u_tlp_tx.block_posted && dut.tlp_np_outstanding != 0)
        block_seen = 1'b1;
      if (dut.tlp_np_outstanding != 0)
        np_seen = 1'b1;
    end
    check("NP outstanding after MRd issue", np_seen);
    tlp_mid = u_rc_bfm.tlp_rx_count;
    u_axi_bfm.cmd_addr  = 64'h0000_0000_C000_2000;
    u_axi_bfm.cmd_wdata = 256'h0BEE0000_DEADBEEF_C0DE1234_56789ABC_11112222_33334444_55556666_77778888;
    u_axi_bfm.cmd_id    = AXI_ID_W'(11);
    u_axi_bfm.axi_write_issue_only();
    for (i = 0; i < 2000; i++) begin
      @(posedge core_clk);
      if (dut.u_tlp_tx.block_posted)
        block_seen = 1'b1;
    end
    check("Posted blocked while NP outstanding",
          block_seen || (u_rc_bfm.tlp_rx_count == tlp_mid));
    u_rc_bfm.auto_cpld_en = 1'b1;
    for (i = 0; i < 8000; i++) begin
      @(posedge core_clk);
      if (u_rc_bfm.tlp_rx_count > tlp_mid)
        i = 8000;
    end
    tlp_after = u_rc_bfm.tlp_rx_count;
    check("Posted TLP sent after NP completion", tlp_after > tlp_mid);
  end
endtask

task automatic test_20_ecrc_check;
  int   i;
  logic prev_nf, nf_bad;
  begin
    $display("\n--- TEST 20: ECRC Check (RC → EP MWr TD=1) ---");
    wait_clocks(100);
    prev_nf = cfg_err_nonfatal;
    nf_bad  = 1'b0;
    u_rc_bfm.inject_ecrc_mwr_to_ep(1'b0);
    for (i = 0; i < 256; i++) begin
      @(posedge core_clk);
      prev_nf = cfg_err_nonfatal;
    end
    check("Good ECRC MWr accepted (link up)", link_up);
    prev_nf = cfg_err_nonfatal;
    u_rc_bfm.inject_ecrc_mwr_to_ep(1'b1);
    for (i = 0; i < 512; i++) begin
      @(posedge core_clk);
      if (cfg_err_nonfatal && !prev_nf)
        nf_bad = 1'b1;
      prev_nf = cfg_err_nonfatal;
    end
    check("Bad ECRC MWr raised nonfatal", nf_bad);
    check("Link up after ECRC tests", link_up);
  end
endtask

task automatic test_21_dma_stress;
  int k;
  begin
    $display("\n--- TEST 21: DMA Stress ---");
    wait_clocks(50);
    for (k = 0; k < 2; k++) begin
      dma_src_addr <= 64'hDEAD_BEEF_0000_0000 + (64'(k) << 20);
      dma_dst_addr <= 64'h0000_0001_0000_0000 + (64'(k) << 20);
`ifdef PCIE_STRICT_LAYERS
      dma_length   <= 32'd8;
`else
      dma_length   <= 32'd256;
`endif
      dma_dir      <= 1'b0;
      dma_start    <= 1'b1;
      @(posedge core_clk);
      dma_start    <= 1'b0;
      begin
        int cnt;
        logic dma_inj_done;
        cnt = 0;
        dma_inj_done = 1'b0;
        while (!dma_done && !dma_error && cnt < 200000) begin
`ifdef PCIE_STRICT_LAYERS
          if (dma_h2d_wait_sig && !dma_inj_done) begin
            u_rc_bfm.inject_cpld_for_tag(dut.gen_dma.u_dma.dma_tag);
            dma_inj_done = 1'b1;
          end
          if (!dma_h2d_wait_sig)
            dma_inj_done = 1'b0;
`endif
          @(posedge core_clk);
          cnt++;
        end
      end
      check($sformatf("DMA H2D stress pass %0d", k), dma_done && !dma_error);
      wait_clocks(200);
      dma_src_addr <= 64'h0000_0002_0000_0000 + (64'(k) << 20);
      dma_dst_addr <= 64'h0000_0003_0000_0000 + (64'(k) << 20);
      dma_length   <= 32'd256;
      dma_dir      <= 1'b1;
      dma_start    <= 1'b1;
      @(posedge core_clk);
      dma_start    <= 1'b0;
      begin
        int cnt2;
        cnt2 = 0;
        while (!dma_done && !dma_error && cnt2 < 200000) begin
          @(posedge core_clk);
          cnt2++;
        end
      end
      check($sformatf("DMA D2H stress pass %0d", k), dma_done && !dma_error);
      wait_clocks(200);
    end
  end
endtask

task automatic test_22_fc_credit_stress;
  int   i, k;
  logic [11:0] cons_before, cons_peak;
  begin
    $display("\n--- TEST 22: FC Credit Stress ---");
    wait_clocks(20);
    cons_before = dut.u_flow_ctrl.local_hdr_p_consumed;
    cons_peak   = cons_before;
    for (k = 0; k < 16; k++) begin
      u_axi_bfm.cmd_addr  = 64'h0000_0000_D000_0000 + (64'(k) << 8);
      u_axi_bfm.cmd_wdata = 256'hFC000000_00000001_00000002_00000003_00000004_00000005_00000006_00000007;
      u_axi_bfm.cmd_id    = AXI_ID_W'(k & 8'hFF);
      u_axi_bfm.axi_write_issue_only();
      if (dut.u_flow_ctrl.local_hdr_p_consumed > cons_peak)
        cons_peak = dut.u_flow_ctrl.local_hdr_p_consumed;
    end
    for (i = 0; i < 4000; i++)
      @(posedge core_clk);
    check("Posted FC consumption increased under burst", cons_peak > cons_before);
    check("Link up after FC burst", link_up);
  end
endtask

`ifdef PCIE_FLIT_TEST
task automatic test_23_flit_framing;
  int i;
  logic flit_seen;
  begin
    $display("\n--- TEST 23: FLIT Framing (Gen6+) ---");
    wait_clocks(20);
    flit_seen = 1'b0;
    u_axi_bfm.cmd_addr  = 64'h0000_0000_F000_0000;
    u_axi_bfm.cmd_wdata = 256'hF1170000_00000000_00000000_00000000_00000000_00000000_00000000_00000000;
    u_axi_bfm.cmd_id    = AXI_ID_W'(12);
    u_axi_bfm.axi_write();
    for (i = 0; i < 8000; i++) begin
      @(posedge core_clk);
      if (dut.flit_mode_active &&
          (dut.u_flit_if.tx_flit_seq != 8'd0 ||
           dut.pipe_tx_data[0][31:24] == 8'hF0))
        flit_seen = 1'b1;
    end
    check("FLIT mode active at Gen6+", dut.flit_mode_active);
    check("FLIT header or seq observed on TX path", flit_seen);
  end
endtask
`endif

task automatic run_stress_tests_19_22;
  test_19_tlp_ordering();
  test_20_ecrc_check();
  test_21_dma_stress();
  test_22_fc_credit_stress();
`ifdef PCIE_FLIT_TEST
  test_23_flit_framing();
`endif
endtask
