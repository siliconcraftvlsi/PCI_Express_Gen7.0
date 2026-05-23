// Yosys/SymbiYosys immediate assertions (subset of sva/pcie_ltssm_assertions.sv)
`include "pcie_pkg.sv"

module pcie_ltssm_formal_props #(
  parameter int unsigned NUM_LANES = 16
)(
  input  logic                   clk,
  input  logic                   rst_n,
  input  pcie_pkg::ltssm_state_e ltssm_state,
  input  logic                   link_up,
  input  logic                   pipe_reset_n,
  input  pcie_pkg::pcie_gen_e    negotiated_gen,
  input  logic [4:0]             negotiated_width
);

  function automatic logic state_is_legal(input pcie_pkg::ltssm_state_e st);
    state_is_legal =
      (st == pcie_pkg::DETECT_QUIET)          || (st == pcie_pkg::DETECT_ACTIVE)         ||
      (st == pcie_pkg::POLLING_ACTIVE)        || (st == pcie_pkg::POLLING_COMPLIANCE)    ||
      (st == pcie_pkg::POLLING_CONFIGURATION) || (st == pcie_pkg::POLLING_SPEED)         ||
      (st == pcie_pkg::CONFIG_LWIDTH_START)   || (st == pcie_pkg::CONFIG_LWIDTH_ACCEPT)  ||
      (st == pcie_pkg::CONFIG_LANENUM_WAIT)   || (st == pcie_pkg::CONFIG_LANENUM_ACCEPT)||
      (st == pcie_pkg::CONFIG_COMPLETE)       || (st == pcie_pkg::CONFIG_IDLE)           ||
      (st == pcie_pkg::RECOVERY_RCVRLOCK)     || (st == pcie_pkg::RECOVERY_RCVRCFG)      ||
      (st == pcie_pkg::RECOVERY_IDLE)         || (st == pcie_pkg::RECOVERY_EQUALIZATION) ||
      (st == pcie_pkg::L0)                    || (st == pcie_pkg::L0S_TX)                ||
      (st == pcie_pkg::L0S_RX)                || (st == pcie_pkg::L1_ENTRY)              ||
      (st == pcie_pkg::L1_IDLE)               || (st == pcie_pkg::L2_IDLE)               ||
      (st == pcie_pkg::L2_TX_WAKE)            || (st == pcie_pkg::HOT_RESET)             ||
      (st == pcie_pkg::DISABLED)              || (st == pcie_pkg::LOOPBACK_ENTRY)        ||
      (st == pcie_pkg::LOOPBACK_ACTIVE)       || (st == pcie_pkg::LOOPBACK_EXIT);
  endfunction

  always @(posedge clk) begin
    if (rst_n) begin
      assert(state_is_legal(ltssm_state));
      assert(!link_up || (ltssm_state == pcie_pkg::L0));
      assert((ltssm_state != pcie_pkg::L0) || link_up);
      assert((ltssm_state != pcie_pkg::DETECT_QUIET) || !pipe_reset_n);
      assert((ltssm_state != pcie_pkg::L0) || pipe_reset_n);
      assert(!link_up || (
        (negotiated_gen == pcie_pkg::PCIE_GEN1) || (negotiated_gen == pcie_pkg::PCIE_GEN2) ||
        (negotiated_gen == pcie_pkg::PCIE_GEN3) || (negotiated_gen == pcie_pkg::PCIE_GEN4) ||
        (negotiated_gen == pcie_pkg::PCIE_GEN5) || (negotiated_gen == pcie_pkg::PCIE_GEN6) ||
        (negotiated_gen == pcie_pkg::PCIE_GEN7)));
      assert(!link_up || (
        (negotiated_width == 5'd1) || (negotiated_width == 5'd2) ||
        (negotiated_width == 5'd4) || (negotiated_width == 5'd8) ||
        (negotiated_width == 5'd16)));
    end
  end

endmodule
