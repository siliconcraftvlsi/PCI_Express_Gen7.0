# PCI Express Gen7 Controller Assertion Plan

Generated: 2026-05-22

## Assertion Strategy

| Checker module | Bind location | Scope |
| --- | --- | --- |
| `pcie_delivery_assertions` | `tb/pcie_sva_bind.sv` | Directed iverilog (delivery only) |
| All five SVA modules | `uvm_tb/pcie_sva_bind.sv` | UVM / Questa / VCS (`pcie_controller_sva_wrapper`) |

| Flow | Command | Notes |
| --- | --- | --- |
| **Verilator + iverilog (default)** | `make check` | `lint` + `lint-sva` + `sim` |
| Directed sim (iverilog) | `cd tb && make sim` | Functional tests; no runtime SVA |
| Lint with SVA | `cd tb && make lint-sva` | Verilator: delivery bind only |
| Other SVA modules | Questa/VCS | `##` / `$past` not supported by Verilator lint |
| UVM + full SVA | `make uvm-smoke` | Optional; Questa/VCS only |

The UVM wrapper binds LTSSM, DLL, FC, TLP, and delivery checkers inside `pcie_controller_top` and uses internal hierarchy (`u_dll_rx`, `fc_avail_*`, `axibr_sva_*`, etc.). MRd tag and CplD match pulses are exported from `pcie_axi_bridge` as `sva_tag_valid`, `sva_pending_tag`, `sva_cpl_received`, and `sva_cpl_tag`.

Gradual bypass removal:

- **iverilog directed TB:** `make sim-pipe`, `sim-dll`, `sim-tlp`, `sim-lcrc`, or `make sim-strict` (`PCIE_STRICT_LAYERS`). Default `make check` runs bypass-on sim plus `sim-pipe`.
- **UVM:** `make uvm-strict` or `STRICT_LAYERS=1` (`+define+PCIE_STRICT_LAYERS`).

Integrations that use a different top should copy the bind connections or re-point the hierarchical signals.

## Required Assertion Categories

- Reset-known-state checks for externally visible outputs.
- No transfer when FIFO or credit state forbids it.
- No X/Z values on valid datapath transfers.
- Status flags and counters only change on documented events.
- Link or lane state machines do not transition through illegal states.
- DMA or packet completion cannot assert simultaneously with unrecoverable error unless the specification allows it.

## Integration Guidance

1. Bind checker modules at the top-level testbench or wrapper.
2. Connect only stable, clock-domain-local signals.
3. Disable assertions during reset and documented training/initialization windows.
4. Treat assertion failures as regression blockers unless a waiver is reviewed and recorded.
