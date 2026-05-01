// -----------------------------------------------------------------------------
// Author  : Robert Kingsly Amalathas
// Email   : robertk@microprocessorlab.com
// LinkedIn: www.linkedin.com/in/robertkingslya
// -----------------------------------------------------------------------------

package pcie_uvm_pkg;

  import uvm_pkg::*;
  `include "uvm_macros.svh"

  class pcie_axi_seq_item extends uvm_sequence_item;
    rand bit                  is_write;
    rand bit [63:0]           addr;
    rand bit [255:0]          data;
    rand bit [7:0]            id;
    bit [255:0]               rdata;
    bit [1:0]                 resp;

    constraint c_addr_align { addr[4:0] == 5'b0; }

    `uvm_object_utils_begin(pcie_axi_seq_item)
      `uvm_field_int(is_write, UVM_DEFAULT)
      `uvm_field_int(addr, UVM_DEFAULT)
      `uvm_field_int(data, UVM_DEFAULT)
      `uvm_field_int(id, UVM_DEFAULT)
      `uvm_field_int(rdata, UVM_DEFAULT)
      `uvm_field_int(resp, UVM_DEFAULT)
    `uvm_object_utils_end

    function new(string name = "pcie_axi_seq_item");
      super.new(name);
    endfunction
  endclass

  class pcie_axi_sequencer extends uvm_sequencer#(pcie_axi_seq_item);
    `uvm_component_utils(pcie_axi_sequencer)
    function new(string name = "pcie_axi_sequencer", uvm_component parent = null);
      super.new(name, parent);
    endfunction
  endclass

  class pcie_axi_driver extends uvm_driver#(pcie_axi_seq_item);
    `uvm_component_utils(pcie_axi_driver)

    virtual pcie_axi_if vif;

    function new(string name = "pcie_axi_driver", uvm_component parent = null);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(virtual pcie_axi_if)::get(this, "", "vif_axi", vif)) begin
        `uvm_fatal("NOVIF", "pcie_axi_if not found in config db")
      end
    endfunction

    task run_phase(uvm_phase phase);
      pcie_axi_seq_item req;
      forever begin
        seq_item_port.get_next_item(req);
        if (req.is_write) begin
          drive_write(req);
        end else begin
          drive_read(req);
        end
        seq_item_port.item_done();
      end
    endtask

    task drive_write(pcie_axi_seq_item t);
      @(posedge vif.clk);
      vif.awid    <= t.id;
      vif.awaddr  <= t.addr;
      vif.awlen   <= 8'd0;
      vif.awsize  <= 3'b101;
      vif.awburst <= 2'b01;
      vif.awvalid <= 1'b1;
      do @(posedge vif.clk); while (!vif.awready);
      vif.awvalid <= 1'b0;

      vif.wdata   <= t.data;
      vif.wstrb   <= '1;
      vif.wlast   <= 1'b1;
      vif.wvalid  <= 1'b1;
      do @(posedge vif.clk); while (!vif.wready);
      vif.wvalid  <= 1'b0;
      vif.wlast   <= 1'b0;

      do @(posedge vif.clk); while (!vif.bvalid);
      t.resp = vif.bresp;
    endtask

    task drive_read(pcie_axi_seq_item t);
      @(posedge vif.clk);
      vif.arid    <= t.id;
      vif.araddr  <= t.addr;
      vif.arlen   <= 8'd0;
      vif.arsize  <= 3'b101;
      vif.arburst <= 2'b01;
      vif.arvalid <= 1'b1;
      do @(posedge vif.clk); while (!vif.arready);
      vif.arvalid <= 1'b0;

      do @(posedge vif.clk); while (!vif.rvalid);
      t.rdata = vif.rdata;
      t.resp  = vif.rresp;
    endtask
  endclass

  class pcie_axi_monitor extends uvm_component;
    `uvm_component_utils(pcie_axi_monitor)

    virtual pcie_axi_if vif;
    uvm_analysis_port#(pcie_axi_seq_item) ap;

    function new(string name = "pcie_axi_monitor", uvm_component parent = null);
      super.new(name, parent);
      ap = new("ap", this);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(virtual pcie_axi_if)::get(this, "", "vif_axi", vif)) begin
        `uvm_fatal("NOVIF", "pcie_axi_if not found in config db")
      end
    endfunction

    task run_phase(uvm_phase phase);
      pcie_axi_seq_item tr;
      forever begin
        @(posedge vif.clk);

        if (vif.awvalid && vif.awready) begin
          tr = pcie_axi_seq_item::type_id::create("wr_mon_tr");
          tr.is_write = 1'b1;
          tr.id = vif.awid;
          tr.addr = vif.awaddr;

          do @(posedge vif.clk); while (!(vif.wvalid && vif.wready));
          tr.data = vif.wdata;

          do @(posedge vif.clk); while (!vif.bvalid);
          tr.resp = vif.bresp;
          ap.write(tr);
        end

        if (vif.arvalid && vif.arready) begin
          tr = pcie_axi_seq_item::type_id::create("rd_mon_tr");
          tr.is_write = 1'b0;
          tr.id = vif.arid;
          tr.addr = vif.araddr;

          do @(posedge vif.clk); while (!vif.rvalid);
          tr.rdata = vif.rdata;
          tr.resp  = vif.rresp;
          ap.write(tr);
        end
      end
    endtask
  endclass

  class pcie_axi_agent extends uvm_component;
    `uvm_component_utils(pcie_axi_agent)

    uvm_active_passive_enum is_active = UVM_ACTIVE;
    pcie_axi_sequencer sqr;
    pcie_axi_driver    drv;
    pcie_axi_monitor   mon;

    function new(string name = "pcie_axi_agent", uvm_component parent = null);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(uvm_active_passive_enum)::get(this, "", "is_active", is_active)) begin
        is_active = UVM_ACTIVE;
      end
      mon = pcie_axi_monitor::type_id::create("mon", this);
      if (is_active == UVM_ACTIVE) begin
        sqr = pcie_axi_sequencer::type_id::create("sqr", this);
        drv = pcie_axi_driver::type_id::create("drv", this);
      end
    endfunction

    function void connect_phase(uvm_phase phase);
      super.connect_phase(phase);
      if (is_active == UVM_ACTIVE) begin
        drv.seq_item_port.connect(sqr.seq_item_export);
      end
    endfunction
  endclass

  class pcie_scoreboard extends uvm_component;
    `uvm_component_utils(pcie_scoreboard)

    uvm_analysis_imp#(pcie_axi_seq_item, pcie_scoreboard) imp;
    int unsigned write_count;
    int unsigned read_count;
    int unsigned err_count;

    function new(string name = "pcie_scoreboard", uvm_component parent = null);
      super.new(name, parent);
      imp = new("imp", this);
    endfunction

    function void write(pcie_axi_seq_item t);
      if (t.is_write) begin
        write_count++;
      end else begin
        read_count++;
      end
      if (t.resp != 2'b00) begin
        err_count++;
        `uvm_error("SB", $sformatf("AXI response error on addr=0x%0h resp=0x%0h", t.addr, t.resp))
      end
    endfunction

    function void report_phase(uvm_phase phase);
      super.report_phase(phase);
      `uvm_info("SB", $sformatf("Scoreboard summary: writes=%0d reads=%0d errors=%0d",
                                 write_count, read_count, err_count), UVM_LOW)
    endfunction
  endclass

  class pcie_coverage extends uvm_subscriber#(pcie_axi_seq_item);
    `uvm_component_utils(pcie_coverage)

    pcie_axi_seq_item tr;

    covergroup cg_axi;
      option.per_instance = 1;
      cp_dir: coverpoint tr.is_write;
      cp_resp: coverpoint tr.resp;
      cp_addr_hi: coverpoint tr.addr[11:8];
      cp_dir_x_resp: cross cp_dir, cp_resp;
    endgroup

    function new(string name = "pcie_coverage", uvm_component parent = null);
      super.new(name, parent);
      cg_axi = new();
    endfunction

    function void write(pcie_axi_seq_item t);
      tr = t;
      cg_axi.sample();
    endfunction
  endclass

  class pcie_env extends uvm_env;
    `uvm_component_utils(pcie_env)

    pcie_axi_agent  axi_agent;
    pcie_scoreboard sb;
    pcie_coverage   cov;

    function new(string name = "pcie_env", uvm_component parent = null);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      axi_agent = pcie_axi_agent::type_id::create("axi_agent", this);
      sb = pcie_scoreboard::type_id::create("sb", this);
      cov = pcie_coverage::type_id::create("cov", this);
    endfunction

    function void connect_phase(uvm_phase phase);
      super.connect_phase(phase);
      axi_agent.mon.ap.connect(sb.imp);
      axi_agent.mon.ap.connect(cov.analysis_export);
    endfunction
  endclass

  class pcie_axi_smoke_seq extends uvm_sequence#(pcie_axi_seq_item);
    `uvm_object_utils(pcie_axi_smoke_seq)

    function new(string name = "pcie_axi_smoke_seq");
      super.new(name);
    endfunction

    task body();
      pcie_axi_seq_item req;

      req = pcie_axi_seq_item::type_id::create("wr_req");
      start_item(req);
      req.is_write = 1'b1;
      req.addr = 64'h0000_0000_1000_0000;
      req.data = 256'hDEADBEEF_CAFEBABE_12345678_ABCDEF01_FEDCBA98_87654321_AABBCCDD_EEFF0011;
      req.id = 8'h11;
      finish_item(req);

      req = pcie_axi_seq_item::type_id::create("rd_req");
      start_item(req);
      req.is_write = 1'b0;
      req.addr = 64'h0000_0000_2000_0000;
      req.id = 8'h22;
      finish_item(req);
    endtask
  endclass

  class pcie_axi_rand_seq extends uvm_sequence#(pcie_axi_seq_item);
    `uvm_object_utils(pcie_axi_rand_seq)

    rand int unsigned n_ops;
    constraint c_n_ops { n_ops inside {[8:32]}; }

    function new(string name = "pcie_axi_rand_seq");
      super.new(name);
      n_ops = 16;
    endfunction

    task body();
      pcie_axi_seq_item req;
      repeat (n_ops) begin
        req = pcie_axi_seq_item::type_id::create("rand_req");
        start_item(req);
        if (!req.randomize() with {
          is_write dist {1 := 60, 0 := 40};
          id inside {[0:255]};
        }) begin
          `uvm_fatal("RAND", "Randomization failed for pcie_axi_seq_item")
        end
        finish_item(req);
      end
    endtask
  endclass

  class pcie_base_test extends uvm_test;
    `uvm_component_utils(pcie_base_test)

    pcie_env env;
    virtual pcie_ctrl_if ctrl_vif;

    function new(string name = "pcie_base_test", uvm_component parent = null);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      env = pcie_env::type_id::create("env", this);
      if (!uvm_config_db#(virtual pcie_ctrl_if)::get(this, "", "vif_ctrl", ctrl_vif)) begin
        `uvm_fatal("NOVIF", "pcie_ctrl_if not found in config db")
      end
    endfunction

    task wait_for_link_up(int unsigned timeout_cycles = 100000);
      int unsigned c;
      c = 0;
      while (!ctrl_vif.link_up && (c < timeout_cycles)) begin
        @(posedge ctrl_vif.clk);
        c++;
      end
      if (!ctrl_vif.link_up) begin
        `uvm_warning("LINK", $sformatf("link_up not asserted within %0d cycles", timeout_cycles))
      end else begin
        `uvm_info("LINK", $sformatf("link_up observed after %0d cycles", c), UVM_LOW)
      end
    endtask
  endclass

  class pcie_smoke_test extends pcie_base_test;
    `uvm_component_utils(pcie_smoke_test)

    function new(string name = "pcie_smoke_test", uvm_component parent = null);
      super.new(name, parent);
    endfunction

    task run_phase(uvm_phase phase);
      pcie_axi_smoke_seq smoke_seq;
      pcie_axi_rand_seq rand_seq;

      phase.raise_objection(this);

      wait_for_link_up(200000);

      smoke_seq = pcie_axi_smoke_seq::type_id::create("smoke_seq");
      smoke_seq.start(env.axi_agent.sqr);

      ctrl_vif.pulse_dma(64'hDEAD_BEEF_0000_0000,
                         64'h0000_0001_0000_0000,
                         32'd256,
                         1'b0);

      rand_seq = pcie_axi_rand_seq::type_id::create("rand_seq");
      rand_seq.randomize() with { n_ops == 12; };
      rand_seq.start(env.axi_agent.sqr);

      ctrl_vif.pulse_intx();

      repeat (1000) @(posedge ctrl_vif.clk);
      phase.drop_objection(this);
    endtask
  endclass

endpackage
