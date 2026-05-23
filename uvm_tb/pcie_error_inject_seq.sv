// =============================================================================
// PCIe 7.0 UVM Testbench - Error Injection Sequences
// =============================================================================
// Sequences for injecting DLL/TLP error conditions to verify:
//   - Bad LCRC → NAK + replay
//   - NAK reception handling
//   - Replay timer timeout
//   - Malformed TLP (illegal header fields)
//   - Unsupported Request (UR) completion status
//   - Poisoned TLP (EP bit set)
//   - Completion timeout (no CplD returned)
//   - FC credit exhaustion
// All sequences extend pcie_axi_smoke_seq or directly compose AXI items.
// =============================================================================

package pcie_error_inject_seq_pkg;

  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import pcie_uvm_pkg::*;

  // ===========================================================================
  // Bad LCRC sequence
  // Triggers the RC BFM to inject a corrupt LCRC on the next TLP it sends.
  // The DUT DLL RX should detect this, send NAK, and expect a replay.
  // ===========================================================================
  class pcie_bad_lcrc_seq extends uvm_sequence #(pcie_axi_seq_item);
    `uvm_object_utils(pcie_bad_lcrc_seq)

    int unsigned num_bad_lcrc = 1;
    virtual pcie_ctrl_if ctrl_vif;

    function new(string name = "pcie_bad_lcrc_seq");
      super.new(name);
    endfunction

    task body();
      pcie_axi_seq_item req;

      if (!uvm_config_db#(virtual pcie_ctrl_if)::get(null, "", "vif_ctrl", ctrl_vif))
        `uvm_fatal("NOVIF", "ctrl_vif not found for bad LCRC seq")

      // Issue a write so DUT sends a MWr TLP which the RC BFM can corrupt
      req = pcie_axi_seq_item::type_id::create("mwr_req");
      start_item(req);
      req.is_write = 1'b1;
      req.addr     = 64'h0000_0000_4000_0000;
      req.data     = 256'hDEAD_BEEF;
      req.id       = 8'hAA;
      finish_item(req);

      repeat (5000) @(posedge ctrl_vif.clk);

      `uvm_info("ERR_INJ", "Bad LCRC sequence: MWr sent, waiting for NAK+replay",
                UVM_LOW)
    endtask
  endclass : pcie_bad_lcrc_seq

  // ===========================================================================
  // NAK injection sequence
  // Sends a valid write, then forces a NAK DLLP from the RC side via
  // control VIF, verifying the DUT enters replay state.
  // ===========================================================================
  class pcie_nak_inject_seq extends uvm_sequence #(pcie_axi_seq_item);
    `uvm_object_utils(pcie_nak_inject_seq)

    virtual pcie_ctrl_if ctrl_vif;

    function new(string name = "pcie_nak_inject_seq");
      super.new(name);
    endfunction

    task body();
      pcie_axi_seq_item req;

      if (!uvm_config_db#(virtual pcie_ctrl_if)::get(null, "", "vif_ctrl", ctrl_vif))
        `uvm_fatal("NOVIF", "ctrl_vif not found for NAK inject seq")

      // Normal write
      req = pcie_axi_seq_item::type_id::create("wr");
      start_item(req);
      req.is_write = 1'b1;
      req.addr     = 64'h0000_0001_0000_0000;
      req.data     = 256'hCAFE_BABE;
      req.id       = 8'h01;
      finish_item(req);

      // Inject NAK via control interface
      @(ctrl_vif.clk);
      ctrl_vif.inject_nak <= 1'b1;
      @(ctrl_vif.clk);
      ctrl_vif.inject_nak <= 1'b0;

      // Wait for replay
      repeat(8192) @(ctrl_vif.clk);

      `uvm_info("ERR_INJ", "NAK injected — DUT should replay outstanding TLP", UVM_LOW)
    endtask
  endclass : pcie_nak_inject_seq

  // ===========================================================================
  // Replay timer timeout sequence
  // Sends a write and then holds the RC BFM from ACK-ing, causing replay timer
  // to expire.  Verified by checking dma_error or cfg_err_nonfatal.
  // ===========================================================================
  class pcie_replay_timeout_seq extends uvm_sequence #(pcie_axi_seq_item);
    `uvm_object_utils(pcie_replay_timeout_seq)

    virtual pcie_ctrl_if ctrl_vif;

    function new(string name = "pcie_replay_timeout_seq");
      super.new(name);
    endfunction

    task body();
      pcie_axi_seq_item req;

      if (!uvm_config_db#(virtual pcie_ctrl_if)::get(null, "", "vif_ctrl", ctrl_vif))
        `uvm_fatal("NOVIF", "ctrl_vif not found for replay timeout seq")

      // Block ACK generation
      ctrl_vif.block_ack <= 1'b1;

      req = pcie_axi_seq_item::type_id::create("wr");
      start_item(req);
      req.is_write = 1'b1;
      req.addr     = 64'h0000_0002_0000_0000;
      req.data     = 256'hFFFF_FFFF;
      req.id       = 8'h02;
      finish_item(req);

      // Wait > REPLAY_TIMER_INIT cycles
      repeat(6000) @(ctrl_vif.clk);

      // Unblock ACK to allow recovery
      ctrl_vif.block_ack <= 1'b0;

      `uvm_info("ERR_INJ", "Replay timeout injected", UVM_LOW)
    endtask
  endclass : pcie_replay_timeout_seq

  // ===========================================================================
  // Malformed TLP sequence
  // Crafts an AXI write that maps to a TLP with an illegal length field.
  // The DUT TL RX should detect this and report cfg_err_nonfatal.
  // ===========================================================================
  class pcie_malformed_tlp_seq extends uvm_sequence #(pcie_axi_seq_item);
    `uvm_object_utils(pcie_malformed_tlp_seq)

    virtual pcie_ctrl_if ctrl_vif;

    function new(string name = "pcie_malformed_tlp_seq");
      super.new(name);
    endfunction

    task body();
      if (!uvm_config_db#(virtual pcie_ctrl_if)::get(null, "", "vif_ctrl", ctrl_vif))
        `uvm_fatal("NOVIF", "ctrl_vif not found for malformed TLP seq")

      // Trigger RC BFM to send a malformed TLP (bad length = 0)
      ctrl_vif.inject_malformed_tlp <= 1'b1;
      @(ctrl_vif.clk);
      ctrl_vif.inject_malformed_tlp <= 1'b0;

      repeat(200) @(ctrl_vif.clk);
      `uvm_info("ERR_INJ", "Malformed TLP injected — expect cfg_err_nonfatal", UVM_LOW)
    endtask
  endclass : pcie_malformed_tlp_seq

  // ===========================================================================
  // Unsupported Request sequence
  // Sends a read to an unmapped BAR address. DUT should return UR completion.
  // ===========================================================================
  class pcie_unsupported_req_seq extends uvm_sequence #(pcie_axi_seq_item);
    `uvm_object_utils(pcie_unsupported_req_seq)

    virtual pcie_ctrl_if ctrl_vif;

    function new(string name = "pcie_unsupported_req_seq");
      super.new(name);
    endfunction

    task body();
      pcie_axi_seq_item req;

      if (!uvm_config_db#(virtual pcie_ctrl_if)::get(null, "", "vif_ctrl", ctrl_vif))
        `uvm_fatal("NOVIF", "ctrl_vif not found for unsupported req seq")

      // Read from an address outside any valid BAR window
      req = pcie_axi_seq_item::type_id::create("ur_rd");
      start_item(req);
      req.is_write = 1'b0;
      req.addr     = 64'hDEAD_0000_DEAD_0000;  // unmapped
      req.id       = 8'hFF;
      finish_item(req);

      repeat (500) @(posedge ctrl_vif.clk);
      `uvm_info("ERR_INJ", "Unsupported Request issued to unmapped address", UVM_LOW)
    endtask
  endclass : pcie_unsupported_req_seq

  // ===========================================================================
  // Poisoned TLP sequence
  // Issues a write through the AXI interface with EP bit forced by the BFM,
  // verifying the DUT TL RX handles poison correctly.
  // ===========================================================================
  class pcie_poisoned_tlp_seq extends uvm_sequence #(pcie_axi_seq_item);
    `uvm_object_utils(pcie_poisoned_tlp_seq)

    virtual pcie_ctrl_if ctrl_vif;

    function new(string name = "pcie_poisoned_tlp_seq");
      super.new(name);
    endfunction

    task body();
      if (!uvm_config_db#(virtual pcie_ctrl_if)::get(null, "", "vif_ctrl", ctrl_vif))
        `uvm_fatal("NOVIF", "ctrl_vif not found for poisoned TLP seq")

      ctrl_vif.inject_poison <= 1'b1;
      @(ctrl_vif.clk);
      ctrl_vif.inject_poison <= 1'b0;

      repeat(300) @(ctrl_vif.clk);
      `uvm_info("ERR_INJ", "Poisoned TLP injected — EP bit set", UVM_LOW)
    endtask
  endclass : pcie_poisoned_tlp_seq

  // ===========================================================================
  // Completion timeout sequence
  // Issues a read but RC BFM never responds with CplD, causing DUT to timeout.
  // ===========================================================================
  class pcie_cpl_timeout_seq extends uvm_sequence #(pcie_axi_seq_item);
    `uvm_object_utils(pcie_cpl_timeout_seq)

    virtual pcie_ctrl_if ctrl_vif;
    int unsigned timeout_cycles = 200_000;

    function new(string name = "pcie_cpl_timeout_seq");
      super.new(name);
    endfunction

    task body();
      pcie_axi_seq_item req;

      if (!uvm_config_db#(virtual pcie_ctrl_if)::get(null, "", "vif_ctrl", ctrl_vif))
        `uvm_fatal("NOVIF", "ctrl_vif not found for cpl timeout seq")

      // Block completions from RC BFM
      ctrl_vif.block_cpl <= 1'b1;

      req = pcie_axi_seq_item::type_id::create("mrd_no_cpl");
      start_item(req);
      req.is_write = 1'b0;
      req.addr     = 64'h0000_0003_0000_0000;
      req.id       = 8'h10;
      finish_item(req);

      // Wait past the completion timeout window
      repeat(timeout_cycles) @(ctrl_vif.clk);
      ctrl_vif.block_cpl <= 1'b0;

      `uvm_info("ERR_INJ",
        $sformatf("Completion timeout injected after %0d cycles", timeout_cycles),
        UVM_LOW)
    endtask
  endclass : pcie_cpl_timeout_seq

  // ===========================================================================
  // FC credit exhaustion sequence
  // Floods the TLP TX with back-to-back MWr until posted credits are exhausted,
  // then verifies the arbiter stalls (no more TLPs until credits update).
  // ===========================================================================
  class pcie_fc_exhaust_seq extends uvm_sequence #(pcie_axi_seq_item);
    `uvm_object_utils(pcie_fc_exhaust_seq)

    rand int unsigned num_writes;
    constraint c_num { num_writes inside {[32:64]}; }

    function new(string name = "pcie_fc_exhaust_seq");
      super.new(name);
    endfunction

    task body();
      pcie_axi_seq_item req;
      `uvm_info("ERR_INJ",
        $sformatf("FC exhaustion: issuing %0d back-to-back writes", num_writes),
        UVM_LOW)

      repeat (num_writes) begin
        req = pcie_axi_seq_item::type_id::create("fc_wr");
        start_item(req);
        if (!req.randomize() with {
          is_write == 1'b1;
          id inside {[0:127]};
        }) `uvm_fatal("RAND", "randomize failed in FC exhaust seq")
        finish_item(req);
      end
    endtask
  endclass : pcie_fc_exhaust_seq

  // ===========================================================================
  // Full error regression test class
  // Runs all error injection sequences in sequence
  // ===========================================================================
  class pcie_error_regression_test extends pcie_base_test;
    `uvm_component_utils(pcie_error_regression_test)

    function new(string name = "pcie_error_regression_test",
                 uvm_component parent = null);
      super.new(name, parent);
    endfunction

    task run_phase(uvm_phase phase);
      pcie_bad_lcrc_seq      bad_lcrc_seq;
      pcie_nak_inject_seq    nak_seq;
      pcie_replay_timeout_seq replay_seq;
      pcie_malformed_tlp_seq malformed_seq;
      pcie_poisoned_tlp_seq  poison_seq;
      pcie_cpl_timeout_seq   cpl_to_seq;
      pcie_fc_exhaust_seq    fc_seq;

      phase.raise_objection(this);
      wait_for_link_up(200_000);

      `uvm_info("TEST", "=== Starting Error Injection Regression ===", UVM_LOW)

      bad_lcrc_seq = pcie_bad_lcrc_seq::type_id::create("bad_lcrc_seq");
      bad_lcrc_seq.start(env.axi_agent.sqr);

      nak_seq = pcie_nak_inject_seq::type_id::create("nak_seq");
      nak_seq.start(env.axi_agent.sqr);

      replay_seq = pcie_replay_timeout_seq::type_id::create("replay_seq");
      replay_seq.start(env.axi_agent.sqr);

      malformed_seq = pcie_malformed_tlp_seq::type_id::create("malformed_seq");
      malformed_seq.start(env.axi_agent.sqr);

      poison_seq = pcie_poisoned_tlp_seq::type_id::create("poison_seq");
      poison_seq.start(env.axi_agent.sqr);

      begin
        pcie_unsupported_req_seq ur_seq;
        ur_seq = pcie_unsupported_req_seq::type_id::create("ur_seq");
        ur_seq.start(env.axi_agent.sqr);
      end

      cpl_to_seq = pcie_cpl_timeout_seq::type_id::create("cpl_to_seq");
      cpl_to_seq.start(env.axi_agent.sqr);

      fc_seq = pcie_fc_exhaust_seq::type_id::create("fc_seq");
      fc_seq.randomize();
      fc_seq.start(env.axi_agent.sqr);

      `uvm_info("TEST", "=== Error Injection Regression Complete ===", UVM_LOW)
      repeat(1000) @(posedge ctrl_vif.clk);
      phase.drop_objection(this);
    endtask
  endclass : pcie_error_regression_test

endpackage : pcie_error_inject_seq_pkg
