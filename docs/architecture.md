# PCI Express Gen7 Controller Architecture

Generated: 2026-05-21

## Architectural Overview

`pcie_controller_top` is organized as a layered high-speed interface IP model. The implementation separates top-level integration, datapath processing, management/configuration logic, and verification-facing observability.

## Block Responsibilities

| Block | Role |
| --- | --- |
| pcie_controller_top.sv | Top-level controller integration |
| pcie_ltssm.sv | Link Training and Status State Machine |
| pcie_pipe_if.sv | PIPE gearbox and ordered-set interface model |
| pcie_dll_tx.sv, pcie_dll_rx.sv | Data Link Layer TX/RX, sequence, LCRC, ACK/NAK model |
| pcie_flow_ctrl.sv | Posted, non-posted, and completion credit model |
| pcie_tlp_tx.sv, pcie_tlp_rx.sv | Transaction Layer packet generation and decode |
| pcie_cfg_space.sv | 4 KB configuration space, PCIe/PM/MSI/MSI-X/AER capabilities |
| pcie_axi_bridge.sv, pcie_dma.sv | AXI bridge and DMA datapaths |

## Data Path

- Ingress stimulus enters through the primary interface listed in the specification.
- Datapath modules transform, encode, serialize, decode, align, or route traffic according to the project function.
- Status, error, and counter paths are exposed through the register/configuration model where implemented.
- Testbenches observe both functional outputs and key status outputs to classify pass/fail behavior.

## Control Path

- Reset initializes datapath state, counters, management registers, and status indicators.
- Configuration inputs or registers select feature enables and operating mode.
- Error and status signals are sticky or counted where the RTL/model implements counters.
- Bring-up sequences are documented in `docs/software_driver_guide.md` and project README files.

## Clocking and Reset

| Domain/reset | Description |
| --- | --- |
| Clocking | core_clk, pipe_clk, and aux_clk clock domains |
| Reset | core_rst_n active-low reset with PIPE reset outputs |
| CDC/RDC review | See docs/cdc_rdc_checklist.md |

## Integration Boundaries

- Synthesizable RTL and behavioral models must be separated before implementation signoff.
- Testbench BFMs, UVM components, SPICE netlists, and analog macromodels are verification collateral unless explicitly promoted to implementation collateral.
- The provided SDC files are starter constraints and must be reviewed for the target process, simulator, synthesis tool, and physical implementation flow.
