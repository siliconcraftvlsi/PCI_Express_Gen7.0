// Yosys/SymbiYosys immediate assertions (subset of sva/pcie_dll_assertions.sv)
`include "pcie_pkg.sv"

module pcie_dll_formal_props #(
  parameter int unsigned DATA_W = 256
)(
  input  logic              clk,
  input  logic              rst_n,
  input  logic              link_up,
  input  logic [DATA_W-1:0] tl_tx_data,
  input  logic              tl_tx_valid,
  input  logic              tl_tx_ready,
  input  logic              tl_tx_sop,
  input  logic              tl_tx_eop,
  input  logic [DATA_W-1:0] phy_tx_data,
  input  logic              phy_tx_valid,
  input  logic              phy_tx_sop,
  input  logic              phy_tx_eop,
  input  logic              nak_received,
  input  logic [11:0]       ack_seq,
  input  logic [11:0]       nak_seq
);

  logic [DATA_W-1:0] tl_tx_data_q;
  logic              tl_tx_valid_q;
  logic              tl_tx_sop_q;
  logic              tl_tx_eop_q;

  always @(posedge clk) begin
    if (!rst_n) begin
      tl_tx_data_q <= '0;
      tl_tx_valid_q <= 1'b0;
      tl_tx_sop_q   <= 1'b0;
      tl_tx_eop_q   <= 1'b0;
    end else begin
      tl_tx_data_q <= tl_tx_data;
      tl_tx_valid_q <= tl_tx_valid;
      tl_tx_sop_q   <= tl_tx_sop;
      tl_tx_eop_q   <= tl_tx_eop;
    end
  end

  always @(posedge clk) begin
    if (rst_n) begin
      assert(!phy_tx_valid || link_up);
      assert(!tl_tx_valid || tl_tx_sop || !tl_tx_eop);
      assert(!tl_tx_valid_q || tl_tx_ready || (
        (tl_tx_data == tl_tx_data_q) &&
        (tl_tx_valid == tl_tx_valid_q) &&
        (tl_tx_sop == tl_tx_sop_q) &&
        (tl_tx_eop == tl_tx_eop_q)
      ));
      assert(!nak_received || link_up);
      assert(ack_seq <= 12'hFFF);
      assert(nak_seq <= 12'hFFF);
    end
  end

endmodule
