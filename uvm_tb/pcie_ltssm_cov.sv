// =============================================================================
// PCIe 7.0 UVM Testbench - LTSSM Functional Coverage Collector
// =============================================================================
// Subscribes to PIPE monitor analysis port and samples:
//   - All LTSSM state visits
//   - All state-to-state transitions (legal and observed)
//   - Negotiated gen × width cross coverage
//   - Power state entry/exit
//   - Recovery entry cause (from L0 vs L0s vs config)
// =============================================================================

package pcie_ltssm_cov_pkg;

  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import pcie_pkg::*;
  import pcie_pipe_agent_pkg::*;

  class pcie_ltssm_cov #(
    int unsigned NUM_LANES = 4,
    int unsigned PIPE_W    = 32
  ) extends uvm_subscriber #(pcie_pipe_seq_item#(NUM_LANES,PIPE_W));

    `uvm_component_param_utils(pcie_ltssm_cov#(NUM_LANES,PIPE_W))

    typedef pcie_pipe_seq_item#(NUM_LANES,PIPE_W) item_t;

    // -----------------------------------------------------------------------
    // Sampled values (updated in write())
    // -----------------------------------------------------------------------
    ltssm_state_e cur_state;
    ltssm_state_e prev_state;
    logic [3:0]   cur_pipe_rate;
    logic [1:0]   cur_pipe_width;
    logic         cur_link_up;

    // -----------------------------------------------------------------------
    // Covergroup: state visitation
    // -----------------------------------------------------------------------
    covergroup cg_ltssm_states;
      option.per_instance = 1;
      option.comment      = "LTSSM state reachability";

      cp_state: coverpoint cur_state {
        bins detect_quiet  = {DETECT_QUIET};
        bins detect_active = {DETECT_ACTIVE};
        bins poll_active   = {POLLING_ACTIVE};
        bins poll_comply   = {POLLING_COMPLIANCE};
        bins poll_config   = {POLLING_CONFIGURATION};
        bins poll_speed    = {POLLING_SPEED};
        bins cfg_lw_start  = {ltssm_state_e'(6'h06)};
        bins cfg_lw_accept = {ltssm_state_e'(6'h07)};
        bins cfg_ln_wait   = {ltssm_state_e'(6'h08)};
        bins cfg_ln_accept = {ltssm_state_e'(6'h09)};
        bins cfg_complete  = {ltssm_state_e'(6'h0A)};
        bins cfg_idle      = {ltssm_state_e'(6'h0B)};
        bins rec_lock      = {RECOVERY_RCVRLOCK};
        bins rec_cfg       = {RECOVERY_RCVRCFG};
        bins rec_idle      = {RECOVERY_IDLE};
        bins rec_eq        = {RECOVERY_EQUALIZATION};
        bins l0            = {L0};
        bins l0s_tx        = {L0S_TX};
        bins l0s_rx        = {L0S_RX};
        bins l1_entry      = {L1_ENTRY};
        bins l1_idle       = {L1_IDLE};
        bins l2_idle       = {L2_IDLE};
        bins l2_tx_wake    = {L2_TX_WAKE};
        bins hot_reset     = {HOT_RESET};
        bins disabled      = {DISABLED};
        bins loopback_ent  = {LOOPBACK_ENTRY};
        bins loopback_act  = {LOOPBACK_ACTIVE};
        bins loopback_exit = {LOOPBACK_EXIT};
      }
    endgroup

    // -----------------------------------------------------------------------
    // Covergroup: state transitions (prev → cur)
    // -----------------------------------------------------------------------
    covergroup cg_ltssm_transitions;
      option.per_instance = 1;
      option.comment      = "Key LTSSM state transitions";

      cp_prev: coverpoint prev_state {
        bins detect  = {DETECT_QUIET, DETECT_ACTIVE};
        bins polling = {POLLING_ACTIVE, POLLING_CONFIGURATION};
        bins cfg_states = {ltssm_state_e'(6'h0A), ltssm_state_e'(6'h0B)};
        bins l0      = {L0};
        bins l0s     = {L0S_TX, L0S_RX};
        bins l1      = {L1_ENTRY, L1_IDLE};
        bins l2      = {L2_IDLE};
        bins recovery= {RECOVERY_RCVRLOCK, RECOVERY_RCVRCFG, RECOVERY_IDLE};
        bins hot_rst = {HOT_RESET};
      }

      cp_cur: coverpoint cur_state {
        bins polling = {POLLING_ACTIVE};
        bins cfg_start = {ltssm_state_e'(6'h06)};
        bins l0      = {L0};
        bins l0s     = {L0S_TX};
        bins l1      = {L1_ENTRY};
        bins l2      = {L2_IDLE};
        bins recovery= {RECOVERY_RCVRLOCK};
        bins detect  = {DETECT_QUIET};
        bins hot_rst = {HOT_RESET};
      }

      cx_transition: cross cp_prev, cp_cur;
    endgroup

    // -----------------------------------------------------------------------
    // Covergroup: negotiated gen × width
    // -----------------------------------------------------------------------
    covergroup cg_nego_params;
      option.per_instance = 1;
      option.comment      = "Negotiated Gen x Lane width combinations";

      cp_rate: coverpoint cur_pipe_rate {
        bins gen1 = {4'h1};
        bins gen2 = {4'h2};
        bins gen3 = {4'h3};
        bins gen4 = {4'h4};
        bins gen5 = {4'h5};
        bins gen6 = {4'h6};
        bins gen7 = {4'h7};
      }

      cp_width: coverpoint cur_pipe_width {
        bins x1  = {2'b00};
        bins x2  = {2'b01};
        bins x4  = {2'b10};
        bins x8_x16 = {2'b11};
      }

      cx_gen_width: cross cp_rate, cp_width;
    endgroup

    // -----------------------------------------------------------------------
    // Covergroup: link_up transitions
    // -----------------------------------------------------------------------
    covergroup cg_link_events;
      option.per_instance = 1;
      cp_link_up: coverpoint cur_link_up;
    endgroup

    // -----------------------------------------------------------------------
    function new(string name = "pcie_ltssm_cov", uvm_component parent = null);
      super.new(name, parent);
      cg_ltssm_states     = new();
      cg_ltssm_transitions = new();
      cg_nego_params      = new();
      cg_link_events      = new();
      cur_state           = DETECT_QUIET;
      prev_state          = DETECT_QUIET;
    endfunction

    // -----------------------------------------------------------------------
    virtual pcie_ctrl_if ctrl_vif;

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(virtual pcie_ctrl_if)::get(this, "", "vif_ctrl", ctrl_vif))
        `uvm_warning("NOVIF", "vif_ctrl not set — LTSSM state coverage may be stale")
    endfunction

    function void write(item_t t);
      prev_state     = cur_state;
      cur_pipe_rate  = t.pipe_rate;
      cur_pipe_width = t.pipe_width;
      cur_link_up    = t.link_up;
      cur_state = ctrl_vif.ltssm_state;

      cg_ltssm_states.sample();
      cg_ltssm_transitions.sample();
      cg_nego_params.sample();
      cg_link_events.sample();
    endfunction

    function void report_phase(uvm_phase phase);
      super.report_phase(phase);
      `uvm_info("COV",
        $sformatf("LTSSM State coverage:      %.1f%%",
                  cg_ltssm_states.get_coverage()),    UVM_LOW)
      `uvm_info("COV",
        $sformatf("LTSSM Transition coverage: %.1f%%",
                  cg_ltssm_transitions.get_coverage()), UVM_LOW)
      `uvm_info("COV",
        $sformatf("Nego Gen×Width coverage:   %.1f%%",
                  cg_nego_params.get_coverage()),     UVM_LOW)
    endfunction

  endclass : pcie_ltssm_cov

endpackage : pcie_ltssm_cov_pkg
