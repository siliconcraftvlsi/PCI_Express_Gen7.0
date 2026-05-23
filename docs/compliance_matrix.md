# PCI Express Gen7 Controller Compliance Matrix

Generated: 2026-05-23 (updated)

This matrix maps each advertised feature to evidence and remaining gaps. It is an engineering traceability document, not an external standards certification.

| Feature or requirement | Current evidence | Gap / limitation |
| --- | --- | --- |
| LTSSM transition coverage | TEST 1–2 link-up; TEST 28 recovery to L0 (`sim_fast_recovery`) | Exhaustive error/recovery transition matrix |
| PIPE behavior | `pcie_rc_bfm.sv` PIPE-level RC; `make sim-pipe` (`SIM_BYPASS_PIPE=0`) | Not a PHY electrical compliance model |
| DLL ACK/NAK/replay | TEST 12–15: LCRC/NAK/replay timer/RC ACK DLLP | Production `REPLAY_TIMER_INIT`; per-layer bypass off matrix |
| Flow control credits | TEST 22 burst stress; RTL `pcie_flow_ctrl.sv`; SVA P23–P24 (`make lint-sva`) | Underflow/overflow corner matrix; iverilog cannot run bound SVA |
| TLP generation/decode | TEST 5–6 AXI MWr/MRd; RC BFM CplD for MRd/DMA | Broad TLP types and malformed TLP matrix |
| TLP ordering (§2.4) | TEST 19, 25, 29: DevCtl RO + TLP Attr[1] in `pcie_axi_bridge.sv` | Broad Attr/RO cross-product stress |
| ECRC | TEST 20: RC→EP MWr TD=1 good/bad digest; RX CRC-32 in `pcie_tlp_rx.sv` | TX-side ECRC append on outbound TLPs |
| Config/MSI/MSI-X/AER | TEST 3–4, 9–11, 18, 24–27; MSI-X table/PBA in `pcie_cfg_space.sv` | Full BAR MMIO table walk; RW/RW1C matrix |
| PM L0s/L1 | TEST 16; `pcie_pm_ctrl.sv` | L2, ASPM, PME handshake matrix |
| Completion timeout | TEST 17; `pcie_cpl_timeout.sv` | Per-tag timeout at production `CPL_TIMEOUT_CYCLES` |
| DMA H2D/D2H | TEST 7–8, 21 stress; RC `dma_h2d_wait` CplD inject | Random tag/alignment exhaustive stress |
| FLIT framing (Gen6+) | `pcie_flit_if.sv`; `make sim-flit` / TEST 23 path | Full Gen6/7 FLIT-mode protocol compliance |
| Static lint | `make lint` (RTL+TB), `make lint-sva` (bound assertions) | CDC/formal signoff not fully automated |
| Delivery assertions | `sva/pcie_delivery_assertions.sv` + `tb/pcie_sva_bind.sv` | Run on Verilator/commercial sim only (not iverilog) |
| UVM feature regression | `pcie_feature_regression_test`; `make -C uvm_tb feature_regression` | Needs Questa/VCS; RC BFM in UVM top for full cpl parity |
| Formal (SymbiYosys) | `make formal FORMAL_STRICT=1` — LTSSM/DLL/FC BMC smoke (z3, depth 5–10); `formal/props/` + `scripts/formal_yosys_prep.py`; CI via `.github/workflows/ci.yml` | Full concurrent SVA in `sva/` (sim-only); deeper prove/BMC depths |

## Release Rule

A row can move from partial to release-complete only when the evidence includes a passing test, reviewed logs, documented coverage, and any required assertion or lint/CDC review.
