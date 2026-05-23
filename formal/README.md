# Formal Verification — PCIe 7.0 Controller

## Tool: SymbiYosys (open-source)

Install (SymbiYosys is not on PyPI — build from source):
```bash
git clone https://github.com/YosysHQ/sby ~/.local/src/sby
cd ~/.local/src/sby && make install PREFIX=$HOME/.local
# ensure ~/.local/bin is on PATH
sby --version
```

Also required: Yosys (`yosys`, `yosys-smtbmc`), z3 (all BMC targets).

On Ubuntu: `sudo apt-get install yosys z3`

## Running

```bash
# Prep Yosys-compatible RTL + run all formal smoke targets
make formal FORMAL_STRICT=1

# Prep only (writes formal/yosys_rtl/)
make formal-yosys-prep
```

Individual targets (from `formal/`):
```bash
cd formal && sby -f pcie_ltssm_fpv.sby
cd formal && sby -f pcie_dll_fpv.sby
cd formal && sby -f pcie_fc_fpv.sby
```

## Architecture

Stock Yosys does not support `import pkg::*` or SVA `assert property`. The flow therefore:

1. **`scripts/formal_yosys_prep.py`** — strips `import`, qualifies `pcie_pkg` symbols, writes `formal/yosys_rtl/`.
2. **`formal/props/*_formal_props.sv`** — immediate `assert()` properties (Yosys-compatible subset).
3. **`sva/*.sv`** — full concurrent SVA for **simulation** (`make sim-strict`, `make lint-sva`).

For full SVA formal signoff, use Tabby/OSS CAD Suite (Verific frontend) or commercial tools.

## Files

| File | Module | Mode | Depth |
|---|---|---|---|
| `pcie_ltssm_fpv.sby` | `pcie_ltssm` | BMC (z3) | 10 |
| `pcie_dll_fpv.sby` | `pcie_dll_tx` | BMC (z3) | 5 |
| `pcie_fc_fpv.sby` | `pcie_flow_ctrl` | BMC (z3) | 10 |

**Default `make formal`:** all three smoke BMC targets above.

| Property source | Used by |
|---|---|
| `formal/props/pcie_*_formal_props.sv` | SymbiYosys |
| `sva/pcie_*_assertions.sv` | Simulation / Verilator lint |

## Notes

- Results appear in `formal/<name>/` directories with pass/fail logs and counter-examples (`.vcd`).
- Generated prep output: `formal/yosys_rtl/` (safe to delete; recreated by `make formal-yosys-prep`).
- Commercial alternatives: Cadence JasperGold, Synopsys VC Formal.
