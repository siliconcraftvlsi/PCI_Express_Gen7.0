# CDC / RDC Waiver Log — PCIe 7.0 Controller

## Document Purpose

Records every CDC/RDC crossing in the design, the mitigation applied, and
the review status.  A crossing must be either (a) resolved with RTL synchronizers
or (b) formally waived with rationale before signoff.

---

## Clock Domain Inventory

| Domain | Source | Frequency | Reset |
|---|---|---|---|
| `core_clk` | SoC PLL | 250 MHz (typical) | `core_rst_n` (async assert, sync deassert via `pcie_rst_sync`) |
| `pipe_clk` | PHY reference | 250 MHz (Gen1/2), 250/500/1000 MHz (Gen3-7) | `pipe_reset_n` from LTSSM |
| `aux_clk`  | Always-on oscillator | 32 kHz | `aux_rst_n` (tied to `core_rst_n` via `pcie_rst_sync`) |

---

## Crossing Inventory and Status

### CDC-001: `core_clk` → `pipe_clk` (control signals)

| Field | Detail |
|---|---|
| Signals | `pipe_rate[3:0]`, `pipe_width[1:0]`, `pipe_power_down[3:0]`, `pipe_reset_n` |
| Direction | core → pipe |
| Type | Multi-bit quasi-static (change only during LTSSM transitions) |
| **Mitigation** | `pcie_sync2` applied to each bit (4 bits stable before LTSSM state changes guarantee no multi-bit skew). `pipe_rate`/`pipe_width` are stable for >10 ms during any LTSSM transition window. |
| **Waiver** | Bit-by-bit sync acceptable: signals are Gray-code-like (only 1 bit changes per LTSSM step), confirmed by code inspection of `pcie_ltssm.sv`. |
| Status | **Mitigated** — `pcie_cdc_sync.sv:pcie_sync2` instantiated in `pcie_controller_top.sv` |
| Reviewer | TBD |

### CDC-002: `pipe_clk` → `core_clk` (RX data path)

| Field | Detail |
|---|---|
| Signals | `pipe_rx_data[NUM_LANES-1:0][PIPE_W-1:0]`, `pipe_rx_datak`, `pipe_rx_valid` |
| Direction | pipe → core |
| Type | High-speed data bus |
| **Mitigation** | `pcie_async_fifo.sv` (`pcie_async_fifo`, depth=16, Gray-coded pointers) instantiated on RX data path in `pcie_pipe_if.sv`. |
| Status | **Mitigated** — async FIFO added in v0.2.0 |
| Reviewer | TBD |

### CDC-003: `core_clk` → `aux_clk` (PM control signals)

| Field | Detail |
|---|---|
| Signals | `pm_state_l1`, `pm_state_l2`, `sw_req_l1`, `sw_req_l2`, `dllp_pm_enter_l1_req` |
| Direction | core → aux |
| Type | Single-bit control |
| **Mitigation** | `pcie_sync2` (2 stages) on each signal. `aux_clk` at 32 kHz is much slower than `core_clk` — metastability MTBF >> 1 year. |
| Status | **Mitigated** |
| Reviewer | TBD |

### CDC-004: `aux_clk` → `core_clk` (PME wake)

| Field | Detail |
|---|---|
| Signals | `dllp_pm_pme_rx`, `pm_wakeup` |
| Direction | aux → core |
| Type | Single-bit pulse |
| **Mitigation** | `pcie_sync_pulse` (toggle/detect pattern) — converts single aux_clk pulse to single core_clk pulse without data loss. |
| Status | **Mitigated** |
| Reviewer | TBD |

### CDC-005: Reset deassertion — `core_rst_n` → all domains

| Field | Detail |
|---|---|
| Signals | `core_rst_n` → `pipe_clk` domain reset, `core_rst_n` → `aux_clk` domain reset |
| Direction | async reset → clocked domains |
| Type | Reset synchronization |
| **Mitigation** | `pcie_rst_sync` (2-flop synchronizer) instantiated for each domain in `pcie_controller_top.sv`. Assert is asynchronous (fast shutdown), deassert is synchronous (prevents metastability on register initialization). |
| Status | **Mitigated** |
| Reviewer | TBD |

---

## Waivers (Accepted Limitations)

### WAIVER-CDC-001: PIPE electrical status during Gen-change

| Field | Detail |
|---|---|
| Signals | `pipe_rx_status[2:0]`, `pipe_rx_status_valid` (3-bit bus) |
| Rationale | PIPE spec requires PHY to hold status stable for at least 2 pipe_clk cycles. RTL `pcie_ltssm.sv` only samples status when `pipe_rx_status_valid` is asserted, which is itself synchronized. Multi-bit status is guaranteed stable by PHY contract. |
| Risk | Low — PHY-level guarantee; acceptable for simulation and FPGA targets. |
| Accepted by | TBD |
| Date | 2026-05-21 |

---

## Sign-off Status

| Item | Status | Date | Sign-off |
|---|---|---|---|
| CDC-001 | Mitigated | 2026-05-21 | Pending review |
| CDC-002 | Mitigated | 2026-05-21 | Pending review |
| CDC-003 | Mitigated | 2026-05-21 | Pending review |
| CDC-004 | Mitigated | 2026-05-21 | Pending review |
| CDC-005 | Mitigated | 2026-05-21 | Pending review |
| WAIVER-CDC-001 | Accepted | 2026-05-21 | Pending owner |

All items must show "Reviewed" status in `docs/signoff_checklist.md` before release.
