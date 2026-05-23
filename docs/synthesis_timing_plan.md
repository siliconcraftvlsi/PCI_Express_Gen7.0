# PCI Express Gen7 Controller Synthesis and Timing Plan

Generated: 2026-05-21

## Collateral

- RTL filelist: `filelists/rtl.f`
- Testbench filelist: `filelists/tb.f`
- Starter constraints: `constraints/pcie_controller_top.sdc`
- Lint waiver placeholder: `lint/waivers.vlt`

## Flow Requirements

- Confirm which files are synthesizable RTL and which files are behavioral models or testbench-only collateral.
- Run lint before synthesis and review all warnings.
- Run synthesis with the target technology library and archive area/timing reports.
- Review unconstrained paths, generated clocks, false paths, async resets, and multi-cycle assumptions.
- Do not treat starter SDC files as signoff constraints without tool and technology review.

## Timing Signoff Entry Criteria

- Clean elaboration with intended top module `pcie_controller_top`.
- No unreviewed black boxes in implementation build.
- CDC/RDC checklist completed.
- Timing constraints reviewed by the implementation owner.
- All critical paths classified as real, false, multicycle, or waived with evidence.
