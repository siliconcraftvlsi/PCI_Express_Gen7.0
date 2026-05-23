# PCIe 7.0 Controller — Hardware Integration Guide

Version: 0.2.0 | Date: 2026-05-21

---

## 1. Overview

This guide describes how to integrate `pcie_controller_top` into an SoC.
It covers RTL instantiation, clock and reset planning, constraint application,
power domain connections, testbench bring-up, and common pitfalls.

---

## 2. Prerequisites

| Item | Requirement |
|---|---|
| SystemVerilog | IEEE 1800-2017 |
| Simulator | Questa 2022.3+ or VCS 2021.09+ (iverilog for directed tests) |
| Synthesis | Design Compiler O-2018.06+ or Genus 21.1+ |
| CDC tool | Cadence Jasper CDC or Mentor CDC (Questa CDC) |
| PIPE PHY | PCI-SIG PIPE 6.x compliant SerDes macro |

---

## 3. File Inclusion Order

Include RTL in this order to satisfy package dependencies:

```
rtl/pcie_pkg.sv              ← must be first (package)
rtl/pcie_cdc_sync.sv         ← sync primitives
rtl/pcie_async_fifo.sv       ← depends on pcie_sync2
rtl/pcie_ltssm.sv
rtl/pcie_pipe_if.sv
rtl/pcie_dll_tx.sv
rtl/pcie_dll_rx.sv
rtl/pcie_flow_ctrl.sv
rtl/pcie_tlp_tx.sv
rtl/pcie_tlp_rx.sv
rtl/pcie_cfg_space.sv
rtl/pcie_axi_bridge.sv
rtl/pcie_dma.sv
rtl/pcie_pm_ctrl.sv
rtl/pcie_cpl_timeout.sv
rtl/pcie_controller_top.sv   ← top last
```

Use `filelists/rtl.f` directly with most tools:
```bash
vlog -sv -f filelists/rtl.f
```

---

## 4. Instantiation

Minimal instantiation example for a x4 Gen5 endpoint:

```systemverilog
pcie_controller_top #(
  .DEVICE_ROLE   (pcie_pkg::ROLE_EP),
  .MAX_GEN       (pcie_pkg::PCIE_GEN5),
  .NUM_LANES     (4),
  .DATA_W        (256),
  .ADDR_W        (64),
  .AXI_ID_W      (8),
  .VENDOR_ID     (16'hABCD),
  .DEVICE_ID     (16'h1234),
  .EN_DMA        (1),
  .DMA_CHANNELS  (4),
  .EN_MSI        (1),
  .EN_MSIX       (1),
  .EN_AER        (1)
) u_pcie_ctrl (
  // Clocks and resets
  .core_clk       (core_clk),
  .core_rst_n     (core_rst_n),
  .pipe_clk       (pipe_ref_clk),   // from PHY
  .aux_clk        (always_on_clk),  // 32 kHz PM clock

  // PIPE interface (connect to your SerDes macro)
  .pipe_tx_data       (pipe_tx_data),
  .pipe_tx_datak      (pipe_tx_datak),
  .pipe_tx_elec_idle  (pipe_tx_elec_idle),
  .pipe_tx_compliance (pipe_tx_compliance),
  .pipe_tx_deemph     (pipe_tx_deemph),
  .pipe_tx_margin     (pipe_tx_margin),
  .pipe_tx_swing      (pipe_tx_swing),
  .pipe_tx_eq_ctrl    (pipe_tx_eq_ctrl),
  .pipe_rx_data       (pipe_rx_data),
  .pipe_rx_datak      (pipe_rx_datak),
  .pipe_rx_valid      (pipe_rx_valid),
  .pipe_rx_elec_idle  (pipe_rx_elec_idle),
  .pipe_rx_status_valid(pipe_rx_status_valid),
  .pipe_rx_status     (pipe_rx_status),
  .pipe_power_down    (pipe_power_down),
  .pipe_reset_n       (pipe_reset_n),
  .pipe_rate          (pipe_rate),
  .pipe_width         (pipe_width),
  .pipe_clk_req_n     (pipe_clk_req_n),

  // AXI4 Subordinate (application → PCIe)
  .s_axi_awid     (s_axi_awid),
  // ... all AXI subordinate signals ...

  // AXI4 Manager (PCIe completions → application memory)
  .m_axi_awid     (m_axi_awid),
  // ... all AXI manager signals ...

  // DMA
  .dma_start      (dma_start),
  .dma_src_addr   (dma_src_addr),
  .dma_dst_addr   (dma_dst_addr),
  .dma_length     (dma_length),
  .dma_dir        (dma_dir),
  .dma_done       (dma_done),
  .dma_error      (dma_error),

  // Interrupts
  .msi_irq        (msi_irq),
  .msi_vector     (msi_vector),
  .msix_irq       (msix_irq),
  .msix_vector    (msix_vector),
  .intx_assert    (intx_assert),

  // Status
  .link_up        (pcie_link_up),
  .ltssm_state    (ltssm_state),
  .negotiated_gen (negotiated_gen),
  .negotiated_width(negotiated_width),
  .cfg_err_cor    (cfg_err_cor),
  .cfg_err_nonfatal(cfg_err_nonfatal),
  .cfg_err_fatal  (cfg_err_fatal),
  .max_payload_size(max_payload_size),
  .max_read_req_size(max_read_req_size)
);
```

---

## 5. Clock Planning

| Signal | Source | Notes |
|---|---|---|
| `core_clk` | SoC PLL | Must be stable before `core_rst_n` deasserts. Typical: 250 MHz. |
| `pipe_clk` | PHY | Provided by SerDes after PHY initialization. Frequency follows PIPE rate negotiation. |
| `aux_clk` | Always-on ring oscillator | Used for PM state retention during L1/L2. 32 kHz typical. |

**Key rule:** `core_rst_n` must deassert synchronously to `core_clk` via `pcie_rst_sync`. Do not connect an asynchronous reset directly into the core logic hierarchy.

---

## 6. Reset Sequencing

Recommended power-on sequence:

1. Assert `core_rst_n` = 0 (active-low reset)
2. Bring up `core_clk` and `aux_clk`
3. Initialize PHY; wait for PHY ready
4. When PHY asserts `pipe_clk` valid → bring up `pipe_clk`
5. Wait ≥ 100 ns with all clocks stable
6. Deassert `core_rst_n` (synchronize via `pcie_rst_sync`)
7. LTSSM will start automatically from `DETECT_QUIET`

**Hot reset:** Drive `core_rst_n` low for ≥ 10 clock cycles. LTSSM re-enters `DETECT_QUIET` automatically.

---

## 7. Timing Constraints

Apply the provided SDC:
```tcl
source constraints/pcie_controller_top.sdc
```

Key constraints:
- `core_clk` period: 4 ns (250 MHz)
- `pipe_clk` period: varies by negotiated rate; constrain to maximum (Gen7: 0.78 ns)
- `aux_clk`: 31.25 µs (32 kHz)
- All `core_clk ↔ pipe_clk` crossings: set as `set_false_path -through` the async FIFO synchronizer path or use `set_max_delay -datapath_only` with the FIFO pointer paths

---

## 8. Power Intent (UPF)

Load UPF after elaboration:
```tcl
read_upf rtl/pcie_controller_top.upf
```

Three power domains: `PD_CORE`, `PD_PIPE`, `PD_AUX`. See `rtl/pcie_controller_top.upf` for full isolation, retention, and level-shifter definitions. During L1, `PD_PIPE` may be powered down. During L2, `PD_CORE` may be powered down with `PD_AUX` retained.

---

## 9. AXI Interface Notes

- **Data width**: default 256-bit. Match `DATA_W` parameter to your interconnect.
- **ID width**: 8-bit. Sufficient for 256 outstanding transactions.
- **Address width**: 64-bit. Tie upper bits to 0 for 32-bit address spaces.
- **Burst**: only INCR (`AXBURST=01`) is used by the AXI bridge. WRAP and FIXED are not generated.
- **Outstanding reads**: up to 256 (10-bit tag pool in `pcie_axi_bridge`). Ensure your interconnect supports this depth.
- **Back-to-back writes**: fully pipelined; no bubble cycles required between AW and W channels.

---

## 10. DMA Usage

1. Set `dma_src_addr` (host or device address), `dma_dst_addr`, `dma_length` (bytes), `dma_dir` (0=H2D, 1=D2H)
2. Assert `dma_start` for one clock cycle
3. Poll `dma_done` (one-cycle pulse) or connect to interrupt fabric
4. Check `dma_error` on `dma_done`; re-issue after clearing if needed

Maximum single transfer length: 2^32 - 1 bytes. Internally chunked by MRRS (reads) or MPS (writes).

---

## 11. Interrupt Delivery

### MSI
- `msi_irq` pulses for one clock when an MSI is triggered
- `msi_vector[4:0]` holds the vector index (0–31)
- Configure MSI capability via MMIO writes to offset `0x90` in config space

### MSI-X
- `msix_irq` + `msix_vector[10:0]` (0–2047 vectors)
- Table base at BAR offset defined in MSI-X capability (offset `0xA0`)

### Legacy INTx
- Drive `intx_assert` high to generate INTx assertion message TLP
- Drive low to generate INTx de-assertion

---

## 12. Completion Timeout Configuration

Completion timeout range is set via config space Device Control register bits [3:0].
Default: Range B (50 ms). Timeout events appear on `cfg_err_cor` (first occurrence)
and `cfg_err_nonfatal` (if escalation is enabled).

---

## 13. Simulation Bring-Up Checklist

- [ ] RTL compiles without errors from `filelists/rtl.f`
- [ ] `make sim` (iverilog) passes all 10 directed tests
- [ ] LTSSM reaches `L0` within 200,000 clock cycles in simulation
- [ ] `link_up` asserts and remains stable in L0
- [ ] AXI write → MWr TLP observable on PIPE TX
- [ ] AXI read → MRd TLP + completion → AXI R channel completes
- [ ] DMA H2D + D2H completes without `dma_error`
- [ ] `msi_irq` pulses once after `intx_assert`
- [ ] No `cfg_err_*` asserted in steady-state L0

---

## 14. Common Integration Issues

| Symptom | Likely Cause | Fix |
|---|---|---|
| LTSSM stays in `DETECT_QUIET` | `pipe_clk` not connected or `pipe_reset_n` not toggling | Verify PHY initialization; check `pipe_clk_req_n` |
| No AXI response after write | `link_up` not asserted | Check LTSSM state; verify RC BFM sends TS1/TS2 |
| DMA never completes | RC BFM not sending CplD for MRd | Extend RC BFM to return real data |
| `msi_irq` never fires | MSI not enabled in config space | Write MSI Enable bit in `pcie_cfg_space` offset 0x90 |
| Spurious `cfg_err_cor` on startup | Completion timeout firing during reset | Ensure `core_rst_n` held long enough; link must be up before issuing reads |
| CDC lint warnings | `pipe_rate` crossed without synchronizer | Confirm `pcie_sync2` instances are in place and CDC tool recognizes them |
