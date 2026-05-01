<!--
  Author  : Robert Kingsly Amalathas
  Email   : robertk@microprocessorlab.com
  LinkedIn: www.linkedin.com/in/robertkingslya
-->
# PCIe 7.0 Controller – RTL and Simulation Environment

## Overview

This repository contains a complete **SystemVerilog RTL implementation** of a
**PCI Express 7.0 Controller** with a full transaction/data-link/physical layer
stack, AXI4 application interface, DMA engine, MSI/MSI-X interrupts, and AER
error reporting.

## Directory Structure

```
ip_cores/pcie/
├── README.md               ← this file
├── rtl/                    ← synthesisable SystemVerilog RTL
│   ├── pcie_pkg.sv             Types, enums, constants, utility functions
│   ├── pcie_controller_top.sv  Top-level integration module
│   ├── pcie_ltssm.sv           Link Training and Status State Machine
│   ├── pcie_pipe_if.sv         PIPE 6.x Interface Adapter (gearbox + K-sym)
│   ├── pcie_dll_tx.sv          Data Link Layer TX (seq, LCRC, retry buffer)
│   ├── pcie_dll_rx.sv          Data Link Layer RX (seq check, LCRC, ACK/NAK)
│   ├── pcie_flow_ctrl.sv       Credit-based Flow Control (P / NP / Cpl)
│   ├── pcie_tlp_tx.sv          Transaction Layer TX (arb, FC gate, TLP emit)
│   ├── pcie_tlp_rx.sv          Transaction Layer RX (header decode, routing)
│   ├── pcie_cfg_space.sv       PCIe Configuration Space (Type 0/1, AER, MSI)
│   ├── pcie_axi_bridge.sv      AXI4 Subordinate ↔ PCIe TLP bridge
│   └── pcie_dma.sv             Integrated DMA engine (H2D and D2H)
├── tb/                     ← simulation testbench (iverilog)
    ├── Makefile                Build and run targets
    ├── tb_pcie_top.sv          Top-level testbench (10 test scenarios)
    ├── axi_master_bfm.sv       AXI4 Master Bus Functional Model
    └── pcie_rc_bfm.sv          PCIe Root Complex BFM (PIPE-level)

```

---

## Prerequisites

| Tool        | Minimum Version | Purpose                        |
|-------------|-----------------|--------------------------------|
| `iverilog`  | ≥ 11.0          | SystemVerilog 2012 compilation |
| `vvp`       | (with iverilog) | Simulation runtime             |
| `gtkwave`   | any             | Waveform viewing (optional)    |
| `verilator` | ≥ 4.x           | Static lint check (optional)   |

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

## Quick Start

```bash
cd /tb

# Compile and run all tests
make

# Or step-by-step:
make compile        # Compile only (produces build/tb_pcie_top.vvp)
make sim            # Run simulation (produces build/pcie_sim.vcd)
make waves          # Open GTKWave with the generated VCD
```

---

## Simulation Targets

| Target         | Description                                    |
|----------------|------------------------------------------------|
| `make` / `make sim` | Compile (if needed) and run simulation    |
| `make compile` | Only compile, do not simulate                  |
| `make waves`   | Run simulation then open GTKWave               |
| `make lint`    | Verilator static lint (no simulation)          |
| `make clean`   | Remove `build/tb_pcie_top.vvp` and `pcie_sim.vcd` |
| `make help`    | Print usage summary                            |

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

The testbench `tb_pcie_top.sv` runs the following 10 tests automatically:

| # | Test Name                      | Description                                         |
|---|-------------------------------|-----------------------------------------------------|
| 1 | **Link Training**             | Verify LTSSM transitions from DETECT to L0         |
| 2 | **Negotiated Parameters**     | Check negotiated Gen and lane width are valid       |
| 3 | **Config Space Init**         | Confirm link is up after config-space initialization|
| 4 | **MPS / MRRS Fields**         | Validate Max Payload and Max Read Req Size fields   |
| 5 | **AXI Write → MWr TLP**       | AXI4 write generates a PCIe Memory Write TLP       |
| 6 | **AXI Read → MRd TLP**        | AXI4 read generates a PCIe Memory Read TLP         |
| 7 | **DMA H2D Transfer**          | DMA host-to-device (PCIe MRd → local memory write) |
| 8 | **DMA D2H Transfer**          | DMA device-to-host (local read → PCIe MWr TLP)     |
| 9 | **MSI Interrupt**             | INTx assertion triggers MSI IRQ output             |
|10 | **AER Error Reporting**       | No spurious errors in steady-state L0              |

Pass / Fail results are printed to stdout at the end of simulation:

```
=============================================================
 Simulation Complete
 Tests PASSED: 10
 Tests FAILED: 0
=============================================================
 OVERALL RESULT: ** PASS **
```

---

## RTL Module Reference

### `pcie_pkg.sv`
Central package imported by all modules. Contains:
- `pcie_gen_e`: Gen1–Gen7 enumeration
- `ltssm_state_e`: 28 LTSSM state enumeration  
- `tlp_type_e`: TLP type codes (MRd, MWr, CplD, Cfg, Msg, …)
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
TX gearbox: expands 256-bit internal data to NUM_LANES × PIPE_W PIPE words.
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
FC Init handshake (INIT1 → INIT2 → ACTIVE), remote/local credit counters,
infinite credit detection, periodic FC Update timer.

### `pcie_tlp_tx.sv`
4-input round-robin arbiter (Posted, NP, DMA, Cfg). Flow-credit gate
prevents TLP emission when credits are insufficient. Tracks consumed credits.

### `pcie_tlp_rx.sv`
TLP header decode (3DW / 4DW), routing to AXI bridge / CFG / DMA /
completion sink. DMA tag range detection (tags 512–767). Poisoned-TLP
and malformed-TLP error reporting.

### `pcie_cfg_space.sv`
4 KB type-0 / type-1 config space. Capabilities at fixed offsets:
PCIe Cap (0x40), PM Cap (0x80), MSI Cap (0x90), MSI-X Cap (0xA0, 2048
vectors), AER Extended Cap (0x100). RW1C AER status registers.

### `pcie_axi_bridge.sv`
AXI Subordinate write → PCIe MWr TLP. AXI Subordinate read → PCIe MRd TLP
with 256-entry tag pool. Completion→AXI-R data return.

### `pcie_dma.sv`
4-channel DMA. H2D: issues chunked MRd64 TLPs (MRRS-bounded), assembles
completions. D2H: reads local AXI, issues MWr64 TLPs (MPS-bounded).
Uses tag range 512–767.

---

## Waveform Viewing

After `make waves`, GTKWave opens with `build/pcie_sim.vcd`. Suggested signal
groups to add:

1. `tb_pcie_top/dut/u_ltssm/state` – LTSSM state machine
2. `tb_pcie_top/dut/link_up` – Link status
3. `tb_pcie_top/s_axi_*` – AXI transactions from BFM
4. `tb_pcie_top/dut/pipe_tx_data[0]` – PIPE TX lane 0
5. `tb_pcie_top/rc_to_dut_data[0]` – PIPE RX lane 0 (from RC BFM)
6. `tb_pcie_top/dut/dma_done` / `dut/dma_error` – DMA status

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
