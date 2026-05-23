# Simulate Later — PM / Cpl-Timeout / MSI-X + UVM Coverage

Collateral is in the repo; run when your simulator is available.

## Directed tests (iverilog) — tests 16–18

| Test | Feature | Checks |
|------|---------|--------|
| 16 | PM L0s idle entry, L1 via PM_Req_Ack DLLP | 2 |
| 17 | Completion timeout (RC BFM blocks CplD) | 1 |
| 18 | MSI-X via cfg sim override ports | 3 |

**Files**

- `tb/pcie_feature_tests.sv` — tasks `test_16_*`, `test_17_*`, `test_18_*`
- `tb/tb_pcie_top.sv` — includes feature tasks; `+feature_tests_only` skips tests 1–15

**Commands**

```bash
# Full strict regression (18 tests, 34 checks)
make sim-strict

# Feature subset only (link-up + tests 16–18, 6 checks)
make sim-features
make sim-features-strict

# Wrapper script
STRICT=1 ./scripts/run_directed_features.sh
```

**Plusargs**

| Plusarg | Effect |
|---------|--------|
| `+feature_tests_only` | Link-up then tests 16–18 only |
| `+no_vcd` | Skip VCD dump |
| `+vcd=path` | Custom VCD path |

---

## UVM + coverage (Questa / VCS)

**Files**

| File | Purpose |
|------|---------|
| `uvm_tb/pcie_feature_uvm.sv` | `pcie_feature_regression_test`, PM/MSI-X/cpl-timeout sequences, `pcie_uvm_event_cov` |
| `uvm_tb/pcie_ltssm_cov.sv` | LTSSM state coverage (pipe monitor) |
| `uvm_tb/pcie_error_inject_seq.sv` | Error injection regression |

**Tests**

- `pcie_smoke_test` — AXI + DMA + INTx
- `pcie_error_regression_test` — LCRC, NAK, replay, poison, cpl-timeout (ctrl `block_cpl`)
- `pcie_feature_regression_test` — PM L0s/L1, MSI-X, cpl-timeout + event covergroup

**Commands**

```bash
make -C uvm_tb compile
make -C uvm_tb smoke
make -C uvm_tb error_inj
make -C uvm_tb feature_regression
make -C uvm_tb regress          # smoke + error + feature × NUM_SEEDS

./scripts/run_uvm_regression.sh
```

**Notes**

- Questa needs `+acc` for `uvm_hdl` paths used in PM/MSI-X sequences (enabled in `uvm_tb/Makefile`).
- UVM top uses `pcie_pipe_partner` (link training only). **Cpl-timeout in UVM** may warn until RC BFM auto-CplD blocking is wired (directed TB uses `pcie_rc_bfm.auto_cpld_en`). PM and MSI-X use hierarchical `uvm_hdl` on the DUT.
- `CPL_TIMEOUT_CYCLES(128)` is set on the UVM DUT for fast timeout when you run feature regression.

**Coverage artifacts**

- Questa: `build/uvm/*.ucdb`
- VCS: `coverage/*.vdb`
- Merge: `make -C uvm_tb cov_merge`

---

## Verilator-only flow

Verilator does not run these testbenches. Use:

```bash
make verilator-check    # lint + SVA lint + verilator_tb smoke
```

See `docs/VERILATOR_FLOW.md`.
