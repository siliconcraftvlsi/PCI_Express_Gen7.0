// =============================================================================
// PCIe 7.0 UVM Testbench - PIPE-Level UVM Agent Package
// =============================================================================
// Provides:
//   pcie_pipe_seq_item  — PIPE-level transaction (one beat per lane group)
//   pcie_pipe_sequencer
//   pcie_pipe_driver    — drives RX PIPE signals into DUT
//   pcie_pipe_monitor   — captures TX PIPE signals from DUT
//   pcie_pipe_agent     — bundles driver + monitor + sequencer
// =============================================================================

package pcie_pipe_agent_pkg;

  import uvm_pkg::*;
  `include "uvm_macros.svh"

  // ===========================================================================
  // Sequence Item
  // ===========================================================================
  class pcie_pipe_seq_item #(
    int unsigned NUM_LANES = 4,
    int unsigned PIPE_W    = 32
  ) extends uvm_sequence_item;

    typedef logic [PIPE_W-1:0]     lane_data_t;
    typedef logic [PIPE_W/8-1:0]   lane_datak_t;

    // Fields driven on RX path (toward DUT)
    rand lane_data_t   rx_data   [NUM_LANES];
    rand lane_datak_t  rx_datak  [NUM_LANES];
    rand logic         rx_valid  [NUM_LANES];
    rand logic         rx_elec_idle [NUM_LANES];
    rand logic         rx_status_valid [NUM_LANES];
    rand logic [2:0]   rx_status [NUM_LANES];

    // Observed TX fields (captured by monitor, not randomized)
    lane_data_t  tx_data  [NUM_LANES];
    lane_datak_t tx_datak [NUM_LANES];
    logic        tx_elec_idle [NUM_LANES];
    logic [3:0]  pipe_rate;
    logic [1:0]  pipe_width;
    logic        link_up;

    // Control: how many clocks to drive this item
    rand int unsigned drive_cycles;
    constraint c_drive_cycles { drive_cycles inside {[1:8]}; }

    `uvm_object_param_utils_begin(pcie_pipe_seq_item#(NUM_LANES, PIPE_W))
      `uvm_field_sarray_int(rx_data,          UVM_DEFAULT)
      `uvm_field_sarray_int(rx_datak,         UVM_DEFAULT)
      `uvm_field_sarray_int(rx_valid,         UVM_DEFAULT)
      `uvm_field_sarray_int(rx_elec_idle,     UVM_DEFAULT)
      `uvm_field_sarray_int(rx_status_valid,  UVM_DEFAULT)
      `uvm_field_sarray_int(rx_status,        UVM_DEFAULT)
      `uvm_field_int(drive_cycles,            UVM_DEFAULT)
    `uvm_object_utils_end

    function new(string name = "pcie_pipe_seq_item");
      super.new(name);
    endfunction

    // Helper: set all lanes to electrical idle (link-down state)
    function void set_elec_idle();
      foreach (rx_elec_idle[i]) begin
        rx_elec_idle[i]    = 1'b1;
        rx_valid[i]        = 1'b0;
        rx_status_valid[i] = 1'b0;
      end
    endfunction

    // Helper: set all lanes to active with given data pattern
    function void set_active(logic [31:0] data_pattern);
      foreach (rx_data[i]) begin
        rx_data[i]         = data_pattern;
        rx_datak[i]        = '0;
        rx_valid[i]        = 1'b1;
        rx_elec_idle[i]    = 1'b0;
        rx_status_valid[i] = 1'b0;
      end
    endfunction

    // Helper: inject receiver-detect status
    function void set_receiver_detect();
      foreach (rx_status[i]) begin
        rx_status[i]       = 3'b001;  // PIPE_STATUS_RECV_DET
        rx_status_valid[i] = 1'b1;
        rx_elec_idle[i]    = 1'b0;
        rx_valid[i]        = 1'b1;
      end
    endfunction

  endclass : pcie_pipe_seq_item

  // ===========================================================================
  // Sequencer
  // ===========================================================================
  class pcie_pipe_sequencer #(
    int unsigned NUM_LANES = 4,
    int unsigned PIPE_W    = 32
  ) extends uvm_sequencer #(pcie_pipe_seq_item#(NUM_LANES,PIPE_W));

    `uvm_component_param_utils(pcie_pipe_sequencer#(NUM_LANES,PIPE_W))
    function new(string name = "pcie_pipe_sequencer", uvm_component parent = null);
      super.new(name, parent);
    endfunction
  endclass

  // ===========================================================================
  // Driver
  // ===========================================================================
  class pcie_pipe_driver #(
    int unsigned NUM_LANES = 4,
    int unsigned PIPE_W    = 32
  ) extends uvm_driver #(pcie_pipe_seq_item#(NUM_LANES,PIPE_W));

    `uvm_component_param_utils(pcie_pipe_driver#(NUM_LANES,PIPE_W))

    typedef pcie_pipe_seq_item#(NUM_LANES,PIPE_W) item_t;
    virtual pcie_pipe_uvm_if#(NUM_LANES,PIPE_W).drv_mp vif;

    function new(string name = "pcie_pipe_driver", uvm_component parent = null);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(virtual pcie_pipe_uvm_if#(NUM_LANES,PIPE_W))
                         ::get(this, "", "vif_pipe", vif))
        `uvm_fatal("NOVIF", "pcie_pipe_uvm_if not found in config_db")
    endfunction

    task run_phase(uvm_phase phase);
      item_t req;
      // Default: all lanes in electrical idle
      drive_idle();
      forever begin
        seq_item_port.get_next_item(req);
        drive_item(req);
        seq_item_port.item_done();
      end
    endtask

    task drive_idle();
      @(vif.drv_cb);
      foreach (vif.drv_cb.rx_elec_idle[i])
        vif.drv_cb.rx_elec_idle[i] <= 1'b1;
      foreach (vif.drv_cb.rx_valid[i])
        vif.drv_cb.rx_valid[i] <= 1'b0;
      foreach (vif.drv_cb.rx_status_valid[i])
        vif.drv_cb.rx_status_valid[i] <= 1'b0;
    endtask

    task drive_item(item_t t);
      repeat (t.drive_cycles) begin
        @(vif.drv_cb);
        foreach (t.rx_data[i]) begin
          vif.drv_cb.rx_data[i]        <= t.rx_data[i];
          vif.drv_cb.rx_datak[i]       <= t.rx_datak[i];
          vif.drv_cb.rx_valid[i]       <= t.rx_valid[i];
          vif.drv_cb.rx_elec_idle[i]   <= t.rx_elec_idle[i];
          vif.drv_cb.rx_status_valid[i]<= t.rx_status_valid[i];
          vif.drv_cb.rx_status[i]      <= t.rx_status[i];
        end
      end
    endtask
  endclass : pcie_pipe_driver

  // ===========================================================================
  // Monitor
  // ===========================================================================
  class pcie_pipe_monitor #(
    int unsigned NUM_LANES = 4,
    int unsigned PIPE_W    = 32
  ) extends uvm_monitor;

    `uvm_component_param_utils(pcie_pipe_monitor#(NUM_LANES,PIPE_W))

    typedef pcie_pipe_seq_item#(NUM_LANES,PIPE_W) item_t;
    virtual pcie_pipe_uvm_if#(NUM_LANES,PIPE_W).mon_mp vif;
    uvm_analysis_port #(item_t) ap;

    function new(string name = "pcie_pipe_monitor", uvm_component parent = null);
      super.new(name, parent);
      ap = new("ap", this);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(virtual pcie_pipe_uvm_if#(NUM_LANES,PIPE_W))
                         ::get(this, "", "vif_pipe", vif))
        `uvm_fatal("NOVIF", "pcie_pipe_uvm_if not found in config_db")
    endfunction

    task run_phase(uvm_phase phase);
      item_t tr;
      forever begin
        @(vif.mon_cb);
        tr = item_t::type_id::create("pipe_mon_tr");
        foreach (tr.tx_data[i]) begin
          tr.tx_data[i]      = vif.mon_cb.tx_data[i];
          tr.tx_datak[i]     = vif.mon_cb.tx_datak[i];
          tr.tx_elec_idle[i] = vif.mon_cb.tx_elec_idle[i];
        end
        tr.pipe_rate  = vif.mon_cb.pipe_rate;
        tr.pipe_width = vif.mon_cb.pipe_width;
        tr.link_up    = vif.mon_cb.link_up;
        ap.write(tr);
      end
    endtask
  endclass : pcie_pipe_monitor

  // ===========================================================================
  // Agent
  // ===========================================================================
  class pcie_pipe_agent #(
    int unsigned NUM_LANES = 4,
    int unsigned PIPE_W    = 32
  ) extends uvm_agent;

    `uvm_component_param_utils(pcie_pipe_agent#(NUM_LANES,PIPE_W))

    pcie_pipe_sequencer #(NUM_LANES,PIPE_W) sqr;
    pcie_pipe_driver    #(NUM_LANES,PIPE_W) drv;
    pcie_pipe_monitor   #(NUM_LANES,PIPE_W) mon;

    function new(string name = "pcie_pipe_agent", uvm_component parent = null);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      mon = pcie_pipe_monitor#(NUM_LANES,PIPE_W)::type_id::create("mon", this);
      if (get_is_active() == UVM_ACTIVE) begin
        sqr = pcie_pipe_sequencer#(NUM_LANES,PIPE_W)::type_id::create("sqr", this);
        drv = pcie_pipe_driver#(NUM_LANES,PIPE_W)::type_id::create("drv", this);
      end
    endfunction

    function void connect_phase(uvm_phase phase);
      if (get_is_active() == UVM_ACTIVE)
        drv.seq_item_port.connect(sqr.seq_item_export);
    endfunction
  endclass : pcie_pipe_agent

  // ===========================================================================
  // Link-training sequence (drives receiver-detect + TS1/TS2 pattern)
  // ===========================================================================
  class pcie_pipe_link_train_seq #(
    int unsigned NUM_LANES = 4,
    int unsigned PIPE_W    = 32
  ) extends uvm_sequence #(pcie_pipe_seq_item#(NUM_LANES,PIPE_W));

    `uvm_object_param_utils(pcie_pipe_link_train_seq#(NUM_LANES,PIPE_W))
    typedef pcie_pipe_seq_item#(NUM_LANES,PIPE_W) item_t;

    int unsigned detect_cycles = 100;
    int unsigned ts1_cycles    = 500;
    int unsigned ts2_cycles    = 500;

    function new(string name = "pcie_pipe_link_train_seq");
      super.new(name);
    endfunction

    task body();
      item_t req;

      // Phase 1: electrical idle → receiver-detect
      req = item_t::type_id::create("detect_req");
      start_item(req);
      req.set_elec_idle();
      req.drive_cycles = detect_cycles;
      finish_item(req);

      // Phase 2: assert receiver-detect status
      req = item_t::type_id::create("rcvdet_req");
      start_item(req);
      req.set_receiver_detect();
      req.drive_cycles = 20;
      finish_item(req);

      // Phase 3: TS1 ordered sets (simplified: all-zeros data, K28.5 on datak)
      req = item_t::type_id::create("ts1_req");
      start_item(req);
      req.set_active(32'h00_00_00_BC);  // K28.5 comma character
      foreach (req.rx_datak[i]) req.rx_datak[i] = 4'b0001;
      req.drive_cycles = ts1_cycles;
      finish_item(req);

      // Phase 4: TS2 ordered sets
      req = item_t::type_id::create("ts2_req");
      start_item(req);
      req.set_active(32'h00_00_00_BC);
      foreach (req.rx_datak[i]) req.rx_datak[i] = 4'b0001;
      req.drive_cycles = ts2_cycles;
      finish_item(req);
    endtask
  endclass : pcie_pipe_link_train_seq

endpackage : pcie_pipe_agent_pkg
