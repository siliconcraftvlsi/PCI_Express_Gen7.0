# Changelog — PCIe 7.0 Controller IP

All notable changes to this project are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).
Versions follow [Semantic Versioning](https://semver.org/).

---

## [Unreleased]

### Added — RTL
- **TLP TX ordering**: posted/NP/completion gating per PCIe §2.4 using outstanding NP count and relaxed-ordering attribute
- **ECRC CRC-32**: replaced XOR placeholder in `pcie_tlp_rx.sv` with spec CRC-32 via `ecrc_update_beat()` in `pcie_pkg.sv`
- **Completion timeout integration**: `pcie_cpl_timeout` wired in `pcie_controller_top` to AXI bridge tag alloc/complete pulses

### Added — Verification
- **Directed tests 16–18**: PM L0s/L1, completion timeout (RC CplD disabled), MSI-X interrupt
- **`make check` includes `sim-strict`**
- **RC BFM**: `auto_cpld_en`, `inject_pm_req_ack_dllp()`

### Added — RTL (continued)
- **FLIT framing layer**: `rtl/pcie_flit_if.sv` — Gen6/7 FLIT header insert/strip (`EN_FLIT` on top, active when negotiated Gen ≥ 6)

### Planned
- Full UVM PIPE agent integration with RC BFM
- LTSSM error/recovery exhaustive transition tests
- DLL ACK/NAK stress with commercial simulator coverage

---

## [0.2.0] — 2026-05-21

### Added — Verification
- **SVA layer suites**: `sva/pcie_ltssm_assertions.sv`, `sva/pcie_dll_assertions.sv`,
  `sva/pcie_fc_assertions.sv`, `sva/pcie_tlp_assertions.sv` — 40+ properties
  covering state validity, backpressure, FC underflow, ordering, and AXI protocol
- **PIPE-level UVM agent**: `uvm_tb/pcie_pipe_agent_pkg.sv` with driver, monitor,
  sequencer, and link-training sequence
- **PIPE UVM interface**: `uvm_tb/pcie_pipe_if.sv` with clocking blocks
- **Error injection sequences**: `uvm_tb/pcie_error_inject_seq.sv` covering bad LCRC,
  NAK inject, replay timeout, malformed TLP, UR, poisoned TLP, completion timeout,
  and FC credit exhaustion
- **LTSSM functional coverage**: `uvm_tb/pcie_ltssm_cov.sv` with state, transition,
  and negotiated Gen×Width cross coverage
- **UVM Makefile**: `uvm_tb/Makefile` supporting Questa and VCS flows with
  per-test and regression targets

### Added — RTL
- **CDC synchronizers**: `rtl/pcie_cdc_sync.sv` — `pcie_sync2`, `pcie_sync_pulse`,
  `pcie_rst_sync` primitives covering all crossings in CDC checklist
- **Async FIFO**: `rtl/pcie_async_fifo.sv` — Gray-coded dual-clock FIFO for
  core_clk ↔ pipe_clk and core_clk ↔ aux_clk data crossings
- **Power management controller**: `rtl/pcie_pm_ctrl.sv` — L0s auto-entry, L1
  DLLP handshake (PM_Enter_L1 / PM_Req_Ack), L2 software entry, PME wake
- **Completion timeout engine**: `rtl/pcie_cpl_timeout.sv` — per-tag countdown
  timers per PCIe spec §2.8 Range B (50 ms default), correctable error reporting

### Added — Synthesis / Implementation
- **DC synthesis script**: `scripts/synth_dc.tcl` — compile_ultra flow with
  timing, area, power, and QoR reports
- **UPF power intent**: `rtl/pcie_controller_top.upf` — three power domains
  (PD_CORE, PD_PIPE, PD_AUX), isolation cells, retention, level shifters

### Added — Formal
- **SymbiYosys FPV setups**: `formal/pcie_ltssm_fpv.sby`, `formal/pcie_dll_fpv.sby`,
  `formal/pcie_fc_fpv.sby` — BMC and proof modes with bind-based SVA attachment

### Added — Packaging
- **IP-XACT**: `ip_xact/pcie_controller_top.xml` — IEEE 1685-2014 component
  description with bus interface mappings and complete fileset
- **Integration Guide**: `docs/integration_guide.md` — step-by-step SoC
  instantiation, clock planning, constraint application, and bring-up guide
- **Coverage merge script**: `scripts/coverage_merge.sh` — Questa and VCS support
- **Release package script**: `scripts/release_package.sh` — timestamped tarball
  with SHA-256 checksum and manifest
- **CDC waiver log**: `lint/cdc_waivers.md` — documented waivers for all
  crossings in CDC checklist
- **CHANGELOG**: this file

---

## [0.1.0] — 2026-05-17 (Initial commit)

### Added
- Full RTL stack: `pcie_pkg`, `pcie_controller_top`, `pcie_ltssm`, `pcie_pipe_if`,
  `pcie_dll_tx`, `pcie_dll_rx`, `pcie_flow_ctrl`, `pcie_tlp_tx`, `pcie_tlp_rx`,
  `pcie_cfg_space`, `pcie_axi_bridge`, `pcie_dma` (~4300 lines)
- iverilog directed testbench with 10 tests, AXI Master BFM, PCIe RC BFM
- UVM testbench skeleton: agent, driver, monitor, scoreboard, smoke + rand sequences
- 5 SVA delivery assertions (`sva/pcie_delivery_assertions.sv`)
- Documentation suite: specification, architecture, verification plan, compliance
  matrix, coverage plan, assertion plan, CDC/RDC checklist, register map,
  signoff checklist, known issues, release notes, synthesis/timing plan,
  software driver guide, block diagram
- SDC constraints, lint waivers, filelists (rtl.f, tb.f, uvm.f), regression script

### Fixed (commit dc1c690)
- Link training issue with PCIe RC BFM: PIPE rx_status_valid toggling corrected
  to allow LTSSM to proceed past DETECT_ACTIVE

---

[Unreleased]: https://github.com/siliconcraftvlsi/pcie_gen7/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/siliconcraftvlsi/pcie_gen7/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/siliconcraftvlsi/pcie_gen7/releases/tag/v0.1.0
