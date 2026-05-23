<!--
  Author  : Robert Kingsly Amalathas
  Email   : robertk@microprocessorlab.com
  LinkedIn: www.linkedin.com/in/robertkingslya
-->
# PCIe 7.0 Controller  RTL and Simulation Environment

## Overview

This repository contains a complete **SystemVerilog RTL implementation** of a
**PCI Express 7.0 Controller** with a full transaction/data-link/physical layer
stack, AXI4 application interface, DMA engine, MSI/MSI-X interrupts, and AER
error reporting.

## Directory Structure

```
ip_cores/pcie/
 README.md                this file
 rtl/                     synthesisable SystemVerilog RTL
    pcie_pkg.sv             Types, enums, constants, utility functions
    pcie_controller_top.sv  Top-level integration module
    pcie_ltssm.sv           Link Training and Status State Machine
    pcie_pipe_if.sv         PIPE 6.x Interface Adapter (gearbox + K-sym)
    pcie_dll_tx.sv          Data Link Layer TX (seq, LCRC, retry buffer)
    pcie_dll_rx.sv          Data Link Layer RX (seq check, LCRC, ACK/NAK)
    pcie_flow_ctrl.sv       Credit-based Flow Control (P / NP / Cpl)
    pcie_tlp_tx.sv          Transaction Layer TX (arb, FC gate, TLP emit)
    pcie_tlp_rx.sv          Transaction Layer RX (header decode, routing)
    pcie_cfg_space.sv       PCIe Configuration Space (Type 0/1, AER, MSI)
    pcie_axi_bridge.sv      AXI4 Subordinate  PCIe TLP bridge
    pcie_dma.sv             Integrated DMA engine (H2D and D2H)
 tb/                      simulation testbench (iverilog)
     Makefile                Build and run targets
     tb_pcie_top.sv          Top-level testbench (29 scenarios, 75 checks)
     pcie_feature_tests.sv   Tests 16–18 (PM, cpl-timeout, MSI-X)
     pcie_stress_tests.sv    Tests 19–22 (ordering, ECRC, DMA/FC stress)
     pcie_advanced_tests.sv  Tests 24–26 (AER RW1C, relaxed order, cfg walk)

```

---

## Prerequisites (Verilator + iverilog)

| Tool        | Minimum Version | Purpose                        |
|-------------|-----------------|--------------------------------|
| `verilator` |  5.x           | RTL + SVA bind lint (**primary static check**) |
| `iverilog`  |  11.0          | Directed simulation (`make sim`) |
| `vvp`       | (with iverilog) | Simulation runtime             |
| `gtkwave`   | any             | Waveform viewing (optional)    |

Questa/VCS are **optional** (UVM only). See [docs/VERILATOR_FLOW.md](docs/VERILATOR_FLOW.md).

### Installing iverilog

**Ubuntu / Debian:**
```bash
sudo apt-get install iverilog gtkwave
```

**macOS (Homebrew):**
```bash
brew install icarus-verilog gtkwave
```

**From source (latest):**
```bash
git clone https://github.com/steveicarus/iverilog.git
cd iverilog && autoconf && ./configure && make && sudo make install
```

> **Note:** iverilog 11+ is required for `always_ff`, `always_comb`, packed
> struct members, and `import` statements. Version 10 and below may fail.

---

## Quick Start (Verilator + iverilog)

```bash
# From project root — recommended CI / local check
make check          # lint + SVA lint + default sim + sim-pipe (PIPE layer on)
make check-all      # check + Verilator DUT smoke
make sim-pipe       # directed sim with SIM_BYPASS_PIPE=0
make sim-strict     # all SIM_BYPASS layers off
make formal FORMAL_STRICT=1   # SymbiYosys BMC smoke (LTSSM + DLL + FC)
make verilator-run  # Minimal DUT C++ sim — LTSSM reaches L0 (see verilator_tb/)

# Or from tb/
cd tb
make lint           # Verilator RTL/TB lint
make lint-sva       # Verilator + delivery SVA bind
make sim            # iverilog directed tests
```

---

## Simulation Targets

| Target         | Description                                    |
|----------------|------------------------------------------------|
| `make` / `make sim` | Compile (if needed) and run simulation    |
| `make compile` | Only compile, do not simulate                  |
| `make waves`   | Run simulation then open GTKWave               |
| `make lint`    | Verilator static lint on RTL+TB (no SVA)       |
| `make lint-sva`| Verilator lint + delivery SVA bind (others: Questa only) |
| `make check`   | lint + lint-sva + sim + sim-pipe (recommended) |
| `make check-all` | check + `verilator-run`                    |
| `make sim-pipe` | Directed sim, real `pcie_pipe_if` path     |
| `make sim-dll` / `sim-tlp` / `sim-lcrc` | One layer group enabled |
| `make sim-strict` | All `SIM_BYPASS_*` off — **75/75 checks** (29 tests) |
| `make sim-features` | Link-up + tests 16–18 only (`+feature_tests_only`) |
| `make sim-flit` | FLIT framing test (`EN_FLIT=1`, Gen6+) |
| `make formal FORMAL_STRICT=1` | SymbiYosys BMC on LTSSM, DLL TX, flow control (~1 min; needs `sby`, `yosys`, `z3`) |
| `make verilator-run` | DUT-only Verilator sim (`verilator_tb/`) |
| `make clean`   | Remove `build/tb_pcie_top.vvp` and `pcie_sim.vcd` |
| `make help`    | Print usage summary                            |

### Optional: UVM (Questa / VCS only — not Verilator)

If you have Questa or VCS installed:

```bash
make uvm-smoke    # UVM smoke + full SVA (including TLP)
make uvm-strict   # smoke without SIM_BYPASS
```

Without Questa, `make uvm-smoke` prints a pointer to `docs/VERILATOR_FLOW.md`.

### Formal verification (SymbiYosys)

Requires [SymbiYosys](https://github.com/YosysHQ/sby) (build from source), Yosys, and z3. See [formal/README.md](formal/README.md).

```bash
make formal FORMAL_STRICT=1   # prep RTL + run pcie_ltssm/dll/fc BMC smoke
make formal-yosys-prep        # regenerate formal/yosys_rtl/ only
```

Full concurrent SVA in `sva/` runs in simulation (`make sim-strict`, `make lint-sva`). Open-source formal uses Yosys immediate assertions in `formal/props/`.

Custom simulation plusargs can be passed via:

```bash
make SIM_ARGS="+timeout=200000"
```

---

## Output Files

After `make sim`:

| File                        | Description                            |
|-----------------------------|----------------------------------------|
| `build/tb_pcie_top.vvp`     | Compiled iverilog simulation binary    |
| `build/pcie_sim.vcd`        | Value Change Dump for GTKWave          |

---

## Test Scenarios

The testbench `tb_pcie_top.sv` runs **29 scenarios** with **75 scoreboard checks**:

| # | Test Name                      | Checks | Description                                         |
|---|-------------------------------|--------|-----------------------------------------------------|
| 1 | **Link Training**             | 2      | LTSSM reaches L0; negotiated width &gt; 0            |
| 2 | **Negotiated Parameters**     | 2      | Valid Gen and lane width {1,2,4,8,16}               |
| 3 | **Config Space Init**         | 1      | Link remains up after config init                   |
| 4 | **MPS / MRRS Fields**         | 2      | Max Payload and Max Read Req Size in range          |
| 5 | **AXI Write → MWr TLP**       | 1      | AXI4 write completes without BFM errors             |
| 6 | **AXI Read → MRd TLP**        | 1      | AXI4 read completes without BFM errors              |
| 7 | **DMA H2D Transfer**          | 1      | Host-to-device via RC CplD (no TB fork inject)      |
| 8 | **DMA D2H Transfer**          | 1      | Device-to-host MWr path                             |
| 9 | **MSI Interrupt**             | 2      | MSI assert and deassert                             |
|10 | **AER Error Reporting**       | 2      | No spurious fatal/nonfatal in steady-state L0       |
|11 | **DLL RX Error Injection**    | 1      | Forced `tl_rx_error` raises `cfg_err_nonfatal`      |
|12 | **Bad LCRC → DLL NAK**        | 2      | RC corrupt LCRC with `sim_lcrc_check_en`; NAK, link up |
|13 | **RC NAK → DUT Replay**       | 3      | Posted MWr, RC `inject_nak_dllp(ack_ptr)`, replay buffer |
|14 | **Replay Timer Timeout**      | 3      | `sim_no_auto_ack` + `sim_fast_replay_timer`, no RC ACK |
|15 | **RC ACK DLLP Handshake**     | 3      | `inject_ack_dllp(tx_seq_num)` purges retry buffer |
|16 | **Power Management**          | 2      | L0s idle entry; L1 via PM_Req_Ack DLLP             |
|17 | **Completion Timeout**        | 1      | RC blocks CplD → `cfg_err_cor`                     |
|18 | **MSI-X Interrupt**           | 3      | MSI-X via cfg sim-override ports                   |
|19 | **TLP Ordering**              | 3      | Posted blocked while NP outstanding (§2.4)         |
|20 | **ECRC Check**                | 3      | Good/bad ECRC on RC→EP MWr with TD=1               |
|21 | **DMA Stress**                | 4      | Two H2D/D2H pairs with 128-byte transfers           |
|22 | **FC Credit Stress**            | 2      | Burst posted writes increase FC consumption         |
|23 | **FLIT Mode** (optional)        | 1      | `make sim-flit` — FLIT framing path smoke           |
|24 | **AER RW1C**                    | 4      | Correctable/uncorrectable sticky + clear-on-write   |
|25 | **Relaxed Ordering**            | 6      | DMA+AXI NP blocked vs allowed with DevCtl RO bit    |
|26 | **Config Capability Walk**      | 4      | Vendor/device ID, AER cap, MSI cap                  |
|27 | **MSI-X Table Walk**            | 6      | Table/PBA offsets, entry program, vector from PBA   |
|28 | **LTSSM Recovery**              | 4      | Lane loss → recovery → return to L0                  |
|29 | **Per-TLP Relaxed Order**       | 5      | `tb_np_relaxed_order` + TLP Attr[1] on MRd/MWr headers |

**Gradual `SIM_BYPASS`:** per-layer bypass on `pcie_controller_top`. Use Makefile targets (`make sim-pipe`, `sim-dll`, …) or override `BYPASS_PIPE=0` etc. on the `tb` make line; `make sim-strict` sets `PCIE_STRICT_LAYERS` (all off).

**SVA:** Bound in `tb/pcie_sva_bind.sv`. Use **`make lint-sva`** (Verilator) for delivery assertions. LTSSM/DLL/FC/TLP modules need Questa/VCS (Verilator cannot lint `##` / clocked `$past`). Iverilog does not run concurrent properties at sim time.

Pass / Fail results are printed to stdout at the end of simulation:

```
=============================================================
 Simulation Complete
 Tests PASSED: 75
 Tests FAILED: 0
=============================================================
 OVERALL RESULT: ** PASS **
```

---

## RTL Module Reference

### `pcie_pkg.sv`
Central package imported by all modules. Contains:
- `pcie_gen_e`: Gen1Gen7 enumeration
- `ltssm_state_e`: 28 LTSSM state enumeration  
- `tlp_type_e`: TLP type codes (MRd, MWr, CplD, Cfg, Msg, )
- `fc_credits_t`: Flow credit struct (12-bit header + 20-bit data)
- Timing constants: `REPLAY_TIMER_INIT`, `ACK_LATENCY_TIMER`, `FC_UPDATE_TIMER`
- `calc_lcrc()`: CRC-32 function for LCRC computation

### `pcie_controller_top.sv`
Top-level parameterized integration module.

Key parameters:

| Parameter      | Default      | Description                        |
|----------------|--------------|------------------------------------|
| `DEVICE_ROLE`  | `ROLE_EP`    | EP / RP / DM / SW                  |
| `MAX_GEN`      | `PCIE_GEN7`  | Maximum PCIe generation            |
| `NUM_LANES`    | 16           | 1, 2, 4, 8, or 16                  |
| `DATA_W`       | 256          | AXI and internal datapath width    |
| `VENDOR_ID`    | `16'hCAFE`   | PCIe Vendor ID                     |
| `DEVICE_ID`    | `16'h0001`   | PCIe Device ID                     |
| `EN_DMA`       | 1            | Enable integrated DMA engine       |
| `DMA_CHANNELS` | 4            | Number of concurrent DMA channels  |

### `pcie_ltssm.sv`
Full 28-state LTSSM per PCIe 7.0 Section 4.2. Controls PIPE `rate`, `width`,
`power_down`, and `reset_n`. Detects TS1/TS2 ordered sets via `pipe_rx_status`.

### `pcie_pipe_if.sv`
TX gearbox: expands 256-bit internal data to NUM_LANES  PIPE_W PIPE words.
Inserts STP/SDP/END_/EDB K-symbols. RX gearbox: collects PIPE words into a
256-bit internal beat, detecting SOP/EOP via K-symbol scan.

### `pcie_dll_tx.sv`
12-bit sequence numbers, LCRC append (CRC-32), 4096-entry replay buffer,
ACK/NAK handling, replay timer, ACK DLLP emission.

### `pcie_dll_rx.sv`
Sequence number validation, running CRC check, ACK/NAK generation,
DLLP type decode (ACK, NAK, FC_Init, FC_Update).

### `pcie_flow_ctrl.sv`
Credit-based FC for Posted / Non-Posted / Completion TLP categories.
FC Init handshake (INIT1  INIT2  ACTIVE), remote/local credit counters,
infinite credit detection, periodic FC Update timer.

### `pcie_tlp_tx.sv`
4-input round-robin arbiter (Posted, NP, DMA, Cfg). Flow-credit gate
prevents TLP emission when credits are insufficient. Tracks consumed credits.

### `pcie_tlp_rx.sv`
TLP header decode (3DW / 4DW), routing to AXI bridge / CFG / DMA /
completion sink. DMA tag range detection (tags 512767). Poisoned-TLP
and malformed-TLP error reporting.

### `pcie_cfg_space.sv`
4 KB type-0 / type-1 config space. Capabilities at fixed offsets:
PCIe Cap (0x40), PM Cap (0x80), MSI Cap (0x90), MSI-X Cap (0xA0, 2048
vectors), AER Extended Cap (0x100). RW1C AER status registers.

### `pcie_axi_bridge.sv`
AXI Subordinate write  PCIe MWr TLP. AXI Subordinate read  PCIe MRd TLP
with 256-entry tag pool. CompletionAXI-R data return.

### `pcie_dma.sv`
4-channel DMA. H2D: issues chunked MRd64 TLPs (MRRS-bounded), assembles
completions. D2H: reads local AXI, issues MWr64 TLPs (MPS-bounded).
Uses tag range 512767.

---

## Waveform Viewing

After `make waves`, GTKWave opens with `build/pcie_sim.vcd`. Suggested signal
groups to add:

1. `tb_pcie_top/dut/u_ltssm/state`  LTSSM state machine
2. `tb_pcie_top/dut/link_up`  Link status
3. `tb_pcie_top/s_axi_*`  AXI transactions from BFM
4. `tb_pcie_top/dut/pipe_tx_data[0]`  PIPE TX lane 0
5. `tb_pcie_top/rc_to_dut_data[0]`  PIPE RX lane 0 (from RC BFM)
6. `tb_pcie_top/dut/dma_done` / `dut/dma_error`  DMA status

---

## Customizing the Simulation

To change the device configuration, edit the `pcie_controller_top` parameter
override block in `tb/tb_pcie_top.sv`:

```systemverilog
pcie_controller_top #(
    .DEVICE_ROLE   (ROLE_EP),
    .MAX_GEN       (PCIE_GEN5),
    .NUM_LANES     (4),
    ...
) dut ( ... );
```

To reduce simulation time, lower `NUM_LANES` (4 is the default in the testbench)
or reduce `MAX_GEN`.

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `error: Unknown module type: pcie_pkg` | iverilog < 11 | Upgrade iverilog |
| `error: Unsupported: always_ff` | iverilog < 11 or missing `-g2012` | Check Makefile uses `-g2012` |
| Link stays in DETECT | RC BFM not asserting status | Check `rc_tx_status_valid` toggling |
| DMA never completes | No completions from RC BFM | RC BFM sends placeholder; extend `pcie_rc_bfm.sv` to return real CplD |
| VCD not generated | `$dumpfile` path issue | Ensure `build/` directory exists (`make compile` creates it) |

---

## License

This RTL is provided for educational and evaluation purposes. See repository
root for license terms.

## Industry Delivery Collateral

Additional industry-delivery collateral has been added under `docs/`, `filelists/`, `constraints/`, `lint/`, `sva/`, `coverage/`, and `scripts/`. Start with `docs/specification.md`, `docs/verification_plan.md`, and `docs/signoff_checklist.md` for review.

