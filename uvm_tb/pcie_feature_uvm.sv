// =============================================================================
// UVM feature tests — PM, completion timeout, MSI-X (+ event coverage)
// Simulate later (Questa/VCS):
//   make -C uvm_tb feature_regression
//   make -C uvm_tb regress
// =============================================================================

package pcie_feature_uvm_pkg;

  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import pcie_uvm_pkg::*;

  // ---------------------------------------------------------------------------
  class pcie_pm_uvm_seq extends uvm_sequence #(pcie_axi_seq_item);
    `uvm_object_utils(pcie_pm_uvm_seq)
    virtual pcie_ctrl_if ctrl_vif;

    function new(string name = "pcie_pm_uvm_seq");
      super.new(name);
    endfunction

    task body();
      int i;
      bit l0s_seen, l1_seen;

      if (!uvm_config_db#(virtual pcie_ctrl_if)::get(null, "", "vif_ctrl", ctrl_vif))
        `uvm_fatal("NOVIF", "vif_ctrl not found for PM sequence")

      `uvm_info("FEAT", "PM: waiting for L0s after idle", UVM_LOW)
      for (i = 0; i < 256; i++) begin
        @(posedge ctrl_vif.clk);
        if (ctrl_vif.pm_state_l0s)
          l0s_seen = 1'b1;
      end
      if (!l0s_seen)
        `uvm_warning("FEAT", "PM L0s not observed — check idle threshold")

      ctrl_vif.tb_sw_req_l1 = 1'b1;
      for (i = 0; i < 512; i++) begin
        @(posedge ctrl_vif.clk);
        if (ctrl_vif.pm_state_l1)
          l1_seen = 1;
      end
      if (!l1_seen) begin
        ctrl_vif.tb_pm_req_ack = 1'b1;
        @(posedge ctrl_vif.clk);
        ctrl_vif.tb_pm_req_ack = 1'b0;
        repeat (8) @(posedge ctrl_vif.clk);
        if (ctrl_vif.pm_state_l1)
          l1_seen = 1;
      end
      ctrl_vif.tb_sw_req_l1 = 1'b0;
      `uvm_info("FEAT", $sformatf("PM sequence done l0s=%0b l1=%0b", l0s_seen, l1_seen), UVM_LOW)
    endtask
  endclass

  // ---------------------------------------------------------------------------
  class pcie_msix_uvm_seq extends uvm_sequence #(pcie_axi_seq_item);
    `uvm_object_utils(pcie_msix_uvm_seq)
    virtual pcie_ctrl_if ctrl_vif;

    function new(string name = "pcie_msix_uvm_seq");
      super.new(name);
    endfunction

    task body();
      if (!uvm_config_db#(virtual pcie_ctrl_if)::get(null, "", "vif_ctrl", ctrl_vif))
        `uvm_fatal("NOVIF", "vif_ctrl not found for MSI-X sequence")

      ctrl_vif.tb_sim_int_override = 1'b1;
      ctrl_vif.tb_sim_msi_en       = 1'b0;
      ctrl_vif.tb_sim_msix_en      = 1'b1;
      ctrl_vif.pulse_intx();
      repeat (4) @(posedge ctrl_vif.clk);
      ctrl_vif.tb_sim_int_override = 1'b0;
      ctrl_vif.tb_sim_msi_en       = 1'b0;
      ctrl_vif.tb_sim_msix_en      = 1'b0;
      `uvm_info("FEAT", $sformatf("MSI-X sequence done msix=%0b msi=%0b",
                ctrl_vif.msix_irq_obs, ctrl_vif.msi_irq_obs), UVM_LOW)
    endtask
  endclass

  // ---------------------------------------------------------------------------
  class pcie_cpl_timeout_feat_seq extends uvm_sequence #(pcie_axi_seq_item);
    `uvm_object_utils(pcie_cpl_timeout_feat_seq)
    virtual pcie_ctrl_if ctrl_vif;
    int unsigned poll_cycles = 8000;

    function new(string name = "pcie_cpl_timeout_feat_seq");
      super.new(name);
    endfunction

    task body();
      pcie_axi_seq_item req;
      bit cor_seen;
      int i;

      if (!uvm_config_db#(virtual pcie_ctrl_if)::get(null, "", "vif_ctrl", ctrl_vif))
        `uvm_fatal("NOVIF", "vif_ctrl not found for cpl-timeout feature seq")

      ctrl_vif.block_cpl = 1'b1;
      ctrl_vif.auto_cpld_en = 1'b0;
      req = pcie_axi_seq_item::type_id::create("mrd_no_cpl");
      start_item(req);
      req.is_write = 1'b0;
      req.addr     = 64'h0000_0000_B000_0000;
      req.id       = 8'h11;
      finish_item(req);

      for (i = 0; i < poll_cycles; i++) begin
        @(posedge ctrl_vif.clk);
        if (ctrl_vif.cfg_err_cor_obs)
          cor_seen = 1;
      end
      ctrl_vif.block_cpl = 1'b0;
      ctrl_vif.auto_cpld_en = 1'b1;
      if (!cor_seen)
        `uvm_warning("FEAT", "Completion timeout correctable error not seen — wire RC BFM auto_cpld_en or block_cpl on partner")
      else
        `uvm_info("FEAT", "Completion timeout correctable error observed", UVM_LOW)
    endtask
  endclass

  // ---------------------------------------------------------------------------
  class pcie_uvm_event_cov extends uvm_component;
    `uvm_component_utils(pcie_uvm_event_cov)

    virtual pcie_ctrl_if ctrl_vif;
    bit sample_pm_l0s, sample_pm_l1, sample_msix, sample_msi, sample_cpl_to;

    covergroup cg_events;
      option.per_instance = 1;
      cp_pm_l0s:   coverpoint sample_pm_l0s;
      cp_pm_l1:    coverpoint sample_pm_l1;
      cp_msix:     coverpoint sample_msix;
      cp_msi:      coverpoint sample_msi;
      cp_cpl_to:   coverpoint sample_cpl_to;
      cx_msix_msi: cross cp_msix, cp_msi;
    endgroup

    function new(string name = "pcie_uvm_event_cov", uvm_component parent = null);
      super.new(name, parent);
      cg_events = new();
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(virtual pcie_ctrl_if)::get(this, "", "vif_ctrl", ctrl_vif))
        `uvm_warning("NOVIF", "vif_ctrl not set — event coverage limited")
    endfunction

    task run_phase(uvm_phase phase);
      forever begin
        @(posedge ctrl_vif.clk);
        sample_pm_l0s = ctrl_vif.pm_state_l0s;
        sample_pm_l1  = ctrl_vif.pm_state_l1;
        sample_msix   = ctrl_vif.msix_irq_obs;
        sample_msi    = ctrl_vif.msi_irq_obs;
        sample_cpl_to = ctrl_vif.cfg_err_cor_obs;
        cg_events.sample();
      end
    endtask

    function void report_phase(uvm_phase phase);
      super.report_phase(phase);
      `uvm_info("COV", $sformatf("UVM event coverage: %.1f%%", cg_events.get_coverage()), UVM_LOW)
    endfunction
  endclass

  // ---------------------------------------------------------------------------
  class pcie_feature_regression_test extends pcie_base_test;
    `uvm_component_utils(pcie_feature_regression_test)

    pcie_uvm_event_cov event_cov;

    function new(string name = "pcie_feature_regression_test", uvm_component parent = null);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      event_cov = pcie_uvm_event_cov::type_id::create("event_cov", this);
    endfunction

    task run_phase(uvm_phase phase);
      pcie_pm_uvm_seq           pm_seq;
      pcie_cpl_timeout_feat_seq cpl_seq;
      pcie_msix_uvm_seq         msix_seq;

      phase.raise_objection(this);
      wait_for_link_up(200_000);

      `uvm_info("TEST", "=== Feature regression: PM / cpl-timeout / MSI-X ===", UVM_LOW)

      pm_seq = pcie_pm_uvm_seq::type_id::create("pm_seq");
      pm_seq.start(env.axi_agent.sqr);

      cpl_seq = pcie_cpl_timeout_feat_seq::type_id::create("cpl_seq");
      cpl_seq.start(env.axi_agent.sqr);

      msix_seq = pcie_msix_uvm_seq::type_id::create("msix_seq");
      msix_seq.start(env.axi_agent.sqr);

      repeat (500) @(posedge ctrl_vif.clk);
      phase.drop_objection(this);
    endtask
  endclass

endpackage : pcie_feature_uvm_pkg
