# PCI Express Gen7 Controller Verification Plan

Generated: 2026-05-21

## Verification Goals

- Prove that the current RTL/model compiles and runs the documented directed tests.
- Exercise reset, bring-up, normal traffic, status reporting, and error/status paths.
- Track feature coverage through a compliance matrix and coverage plan.
- Avoid unsupported compliance claims until the corresponding tests and reviews exist.

## Existing Tests

| Test/collateral | Purpose | Classification |
| --- | --- | --- |
| tb_pcie_top.sv | 10 directed scenarios: link training, config, MPS/MRRS, AXI write/read, DMA H2D/D2H, MSI, no-spurious-AER | Directed regression item |
| axi_master_bfm.sv | AXI master BFM for host/application transactions | Directed regression item |
| pcie_rc_bfm.sv | Root complex BFM at PIPE-level model | Directed regression item |
| uvm_tb/pcie_uvm_pkg.sv | UVM sequence, scoreboard, and coverage scaffolding | Directed regression item |

## Required Regression Commands

- `make sim`
- `make compile`
- `make lint`

## Additional Test Work To Reach Industry Delivery

- Add injected bad LCRC, NAK, replay, timeout, malformed TLP, unsupported request, and poisoned TLP tests.
- Add LTSSM transition coverage and negative recovery tests.
- Add complete config-space capability-walk tests with RW/RW1C behavior.
- Add DMA random transfer size, alignment, and tag stress testing.

## Pass/Fail Policy

- A test passes only when the simulator exits with status 0 and prints the expected PASS or completion message.
- Warnings are reviewed; waived warnings must be listed in `lint/waivers.vlt`.
- Any unreviewed X propagation, timeout, fatal, assertion failure, or scoreboard mismatch blocks release.
- Compliance rows marked partial or planned cannot be advertised as complete features.
