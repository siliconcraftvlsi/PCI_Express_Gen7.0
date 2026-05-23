# Verilator-Only Workflow

Use this path when you have **Verilator only** (no Questa/VCS, and optionally no Icarus Verilog).

## Verilator-only commands

| Step | Command | What it proves |
|------|---------|----------------|
| RTL + TB lint | `make lint` | Syntax/structure on RTL + `tb_pcie_top` |
| SVA bind lint | `make lint-sva` | Delivery assertions bound (static check) |
| DUT link-up smoke | `make verilator-run` | LTSSM reaches **L0** via C++ sim |
| **All of the above** | **`make verilator-check`** | Recommended CI for Verilator-only setups |

```bash
# From project root — no iverilog or Questa needed
make verilator-check
```

Install Verilator only:

```bash
sudo apt-get install verilator   # Ubuntu/Debian
# or build from https://github.com/verilator/verilator
```

## What you cannot run with Verilator only

| Feature | Why | Alternative |
|---------|-----|-------------|
| **UVM** (`uvm_tb/`, item 4 from recent work) | UVM needs Questa or VCS | Use directed `tb_pcie_top` tests (needs iverilog) |
| **Runtime SVA** (LTSSM/DLL/FC/TLP `##` properties) | Verilator lint only; no sim-time properties | `make lint-sva` (delivery SVA static bind) |
| **34 directed tests** (PM, DMA, replay, …) | Full BFM TB targets **iverilog** | Install `iverilog` → `make sim-strict` |
| **`make check`** | Includes `make sim` (iverilog) | Use `make verilator-check` instead |

## Verilator + iverilog (full open-source stack)

If you can install **iverilog** as well (still no Questa), you get the full directed regression:

```bash
sudo apt-get install verilator iverilog gtkwave
make check          # lint + lint-sva + sim + sim-pipe
make sim-strict     # all SIM_BYPASS off, 34/34 checks
make check-all      # check + verilator-run
```

This project is set up for **open-source simulation + Verilator static/SVA lint**. UVM and runtime concurrent SVA require Questa or VCS, which are optional.

## Recommended toolchain

| Step | Tool | Command |
|------|------|---------|
| RTL + TB lint | Verilator | `make lint` |
| SVA bind lint | Verilator | `make lint-sva` |
| Directed sim | Icarus Verilog | `make sim` |
| Minimal DUT sim | Verilator C++ | `make verilator-run` |
| All of the above | Both | `make check` |
| Full open-source CI | Both + Verilator sim | `make check-all` |

Install:

```bash
sudo apt-get install verilator iverilog gtkwave
```

## What Verilator does here

- **`make lint`** — structural/syntax check on RTL + directed testbench (`tb_pcie_top`).
- **`make lint-sva`** — same, plus **delivery** assertions bound into the DUT. Other SVA modules (LTSSM, DLL, FC, TLP) use `##` / `$past` forms that Verilator does not support; they are disabled in the bind file when `+define+VERILATOR` is set.
- **Full directed TB** (`tb_pcie_top` + RC/AXI BFMs) targets iverilog only.
- **Minimal DUT smoke** (`verilator_tb/`) compiles the controller + LTSSM-aware PIPE partner; C++ toggles clock/reset and passes when LTSSM reaches **L0** or **CONFIG_IDLE**.

## What Verilator cannot do (in this repo)

| Feature | Reason |
|---------|--------|
| UVM tests (`uvm_tb/`) | UVM needs Questa/VCS |
| Runtime concurrent SVA (`##`, `\|->`, cover property) | Not in iverilog; Verilator lint only checks structure |
| TLP assertion bind at lint | `pcie_tlp_assertions.sv` — wrap skipped with `+define+VERILATOR` |
| `make uvm-smoke` | Requires `vlog`/`vsim` license |

## SVA coverage on Verilator

| Module | Verilator `lint-sva` |
|--------|----------------------|
| `pcie_delivery_assertions` | Yes |
| `pcie_ltssm_assertions` | Questa/VCS only |
| `pcie_dll_assertions` | Questa/VCS only |
| `pcie_fc_assertions` | Questa/VCS only |
| `pcie_tlp_assertions` | Questa/VCS only |

Bind file: `tb/pcie_sva_bind.sv` (shared with `uvm_tb/` via include).

## Quick start

```bash
cd tb
make check          # lint + lint-sva + sim (28 PASS)
```

From project root:

```bash
make check            # lint + SVA lint + iverilog sim + sim-pipe
make check-all        # check + verilator-run
make lint-sva
make sim-pipe         # directed sim with real PIPE layer
make verilator-run    # DUT + link partner (no UVM/BFMs)
make verilator-trace  # same with VCD under build/verilator/pcie_dut.vcd
```

`verilator_tb/` uses `pcie_verilator_link.sv`, which mirrors `pcie_rc_bfm.sv` training outputs keyed on DUT `ltssm_state` (required for CONFIG_LANENUM_* TS1/TS2 exchange).

## If you add Questa later

```bash
make uvm-compile
make uvm-smoke
```

Full TLP bind is enabled automatically (no `VERILATOR` define).
