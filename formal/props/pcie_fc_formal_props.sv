// Yosys/SymbiYosys immediate assertions (subset of sva/pcie_fc_assertions.sv)
`include "pcie_pkg.sv"

module pcie_fc_formal_props (
  input logic        clk,
  input logic        rst_n,
  input logic        link_up,
  input logic [11:0] avail_p_hdr,
  input logic [19:0] avail_p_dat,
  input logic [11:0] avail_np_hdr,
  input logic [19:0] avail_np_dat,
  input logic [11:0] avail_cpl_hdr,
  input logic [19:0] avail_cpl_dat,
  input logic [11:0] consumed_p_hdr,
  input logic [19:0] consumed_p_dat,
  input logic [11:0] consumed_np_hdr,
  input logic [19:0] consumed_np_dat,
  input logic [11:0] consumed_cpl_hdr,
  input logic [19:0] consumed_cpl_dat,
  input logic        fc_update_tx,
  input logic        fc_rx_valid,
  input logic [7:0]  fc_rx_type,
  input logic [7:0]  fc_upd_type
);

  localparam logic [11:0] FC_HDR_INFINITE = 12'hFFF;
  localparam logic [19:0] FC_DAT_INFINITE = 20'hFFFFF;

  always @(posedge clk) begin
    if (rst_n) begin
      if (link_up && avail_p_hdr != FC_HDR_INFINITE)
        assert(consumed_p_hdr <= avail_p_hdr);
      if (link_up && avail_p_dat != FC_DAT_INFINITE)
        assert(consumed_p_dat <= avail_p_dat);
      if (link_up && avail_np_hdr != FC_HDR_INFINITE)
        assert(consumed_np_hdr <= avail_np_hdr);
      if (link_up && avail_np_dat != FC_DAT_INFINITE)
        assert(consumed_np_dat <= avail_np_dat);
      if (link_up && avail_cpl_hdr != FC_HDR_INFINITE)
        assert(consumed_cpl_hdr <= avail_cpl_hdr);
      if (link_up && avail_cpl_dat != FC_DAT_INFINITE)
        assert(consumed_cpl_dat <= avail_cpl_dat);
      if (fc_update_tx)
        assert((fc_upd_type == pcie_pkg::DLLP_FC_UPD_P) ||
               (fc_upd_type == pcie_pkg::DLLP_FC_UPD_NP) ||
               (fc_upd_type == pcie_pkg::DLLP_FC_UPD_CPL));
      if (fc_rx_valid)
        assert((fc_rx_type == pcie_pkg::DLLP_FC_INIT_P) ||
               (fc_rx_type == pcie_pkg::DLLP_FC_INIT_NP) ||
               (fc_rx_type == pcie_pkg::DLLP_FC_INIT_CPL) ||
               (fc_rx_type == pcie_pkg::DLLP_FC_UPD_P) ||
               (fc_rx_type == pcie_pkg::DLLP_FC_UPD_NP) ||
               (fc_rx_type == pcie_pkg::DLLP_FC_UPD_CPL));
    end
  end

endmodule
