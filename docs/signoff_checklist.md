# PCI Express Gen7 Controller Signoff Checklist

Generated: 2026-05-21

## Documentation

- [ ] Specification reviewed.
- [ ] Architecture reviewed.
- [ ] Register/configuration map reviewed.
- [ ] Verification plan reviewed.
- [ ] Compliance matrix reviewed.
- [ ] Known issues accepted by owner.
- [ ] Integration guide reviewed.
- [ ] CHANGELOG updated with release version.

## RTL and Verification

- [ ] RTL/testbench compiles from clean checkout (all 16 RTL files in rtl.f).
- [ ] Directed regressions pass (make sim → 10/10 tests).
- [ ] UVM regression complete: pcie_smoke_test, pcie_error_regression_test.
- [ ] Error injection sequences all exercised (bad LCRC, NAK, replay timeout, malformed TLP, poisoned TLP, cpl timeout, FC exhaust).
- [ ] Assertions enabled: all 5 SVA modules bound and reviewed.
- [ ] Functional coverage report reviewed (target: ≥80% line, ≥70% branch, 100% LTSSM states).
- [ ] LTSSM coverage: all 28 states visited, key transitions covered.
- [ ] Lint report reviewed and waivers in lint/waivers.vlt approved.
- [ ] CDC report reviewed: all 5 crossings mitigated; WAIVER-CDC-001 accepted.
- [ ] CDC waivers documented in lint/cdc_waivers.md and owner-signed.

## Implementation

- [ ] Filelists complete and ordered (filelists/rtl.f, tb.f, uvm.f).
- [ ] SDC constraints reviewed (constraints/pcie_controller_top.sdc).
- [ ] UPF power intent reviewed (rtl/pcie_controller_top.upf).
- [ ] Synthesis completes for intended top (scripts/synth_dc.tcl).
- [ ] Timing reports reviewed: no setup/hold violations at corner.
- [ ] Power report reviewed: within budget.
- [ ] IP-XACT component description validated (ip_xact/pcie_controller_top.xml).
- [ ] Release package created: scripts/release_package.sh run successfully.
- [ ] SHA-256 checksum recorded: ______________ (fill in).
- [ ] Formal FPV: pcie_ltssm_fpv, pcie_dll_fpv, pcie_fc_fpv — all PASS.

## Release Decision

Current status: v0.2.0 engineering collateral — significantly expanded.
All items above must show reviewed/checked status before promoting to production release.

Release version: ________ Date: ________ Approved by: ________
