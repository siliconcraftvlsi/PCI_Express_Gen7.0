// Verilator C++ driver for tb_pcie_dut (minimal DUT + PIPE partner smoke).

#include <cstdio>
#include <cstdlib>

#include "Vtb_pcie_dut.h"
#include "Vtb_pcie_dut___024root.h"
#include "verilated.h"

#if VM_TRACE
#include "verilated_vcd_c.h"
#endif

static vluint64_t main_time = 0;

double sc_time_stamp() { return static_cast<double>(main_time); }

// pcie_pkg.sv state encodings
static constexpr uint8_t kLtssmL0          = 0x10u;
static constexpr uint8_t kLtssmConfigIdle  = 0x0Bu;

int main(int argc, char** argv) {
  Verilated::commandArgs(argc, argv);

  const int max_cycles = 500000;
  int exit_code      = 1;
  int link_cycles    = -1;

  Vtb_pcie_dut* top = new Vtb_pcie_dut;
  Vtb_pcie_dut___024root* const r = top->rootp;

#if VM_TRACE
  Verilated::traceEverOn(true);
  VerilatedVcdC* tfp = new VerilatedVcdC;
  top->trace(tfp, 99);
  tfp->open("../build/verilator/pcie_dut.vcd");
#endif

  r->tb_pcie_dut__DOT__rst_n    = 0;
  r->tb_pcie_dut__DOT__core_clk = 0;

  for (int cycle = 0; cycle < max_cycles; ++cycle) {
    if (cycle < 20) {
      r->tb_pcie_dut__DOT__rst_n = 0;
    } else {
      r->tb_pcie_dut__DOT__rst_n = 1;
    }

    r->tb_pcie_dut__DOT__core_clk = 1;
    top->eval();

#if VM_TRACE
    tfp->dump(static_cast<vluint64_t>(main_time));
#endif
    main_time++;

    r->tb_pcie_dut__DOT__core_clk = 0;
    top->eval();

#if VM_TRACE
    tfp->dump(static_cast<vluint64_t>(main_time));
#endif
    main_time++;

    const uint8_t st = r->tb_pcie_dut__DOT__dut__DOT__u_ltssm__DOT__state;
    const bool link_ok = r->tb_pcie_dut__DOT__rst_n &&
                         (st == kLtssmL0 || st == kLtssmConfigIdle);
    if (link_ok && link_cycles < 0) {
      link_cycles = cycle;
      exit_code   = 0;
    }
  }

#if VM_TRACE
  tfp->close();
  delete tfp;
#endif

  const uint8_t final_ltssm = r->tb_pcie_dut__DOT__dut__DOT__u_ltssm__DOT__state;

  delete top;

  if (exit_code == 0) {
    std::printf("[VERILATOR-TB] L0 @ cycle %d (ltssm=0x%02x)\n",
                link_cycles, static_cast<unsigned>(final_ltssm));
    std::printf("[VERILATOR] PASS\n");
  } else {
    std::printf("[VERILATOR-TB] FAIL: timeout (ltssm=0x%02x)\n",
                static_cast<unsigned>(final_ltssm));
    std::printf("[VERILATOR] FAIL\n");
  }

  return exit_code;
}
