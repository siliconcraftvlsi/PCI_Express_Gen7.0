# PCI Express Gen7 Controller Specification

Generated: 2026-05-21  
Project directory: `PCI_Express_Gen7.0`  
Top module: `pcie_controller_top`

## Purpose

This specification defines the industry-delivery contract for `pcie_controller_top` as it exists in this repository. The design is treated as reusable IP collateral: RTL or models, verification collateral, register or configuration interface documentation, constraints scaffolding, and release/signoff documentation are all part of the deliverable.

## Scope

The scope includes the current SystemVerilog design/model files, available testbenches, software examples where present, generated documentation, and delivery automation. Compliance claims are limited to the evidence listed in the compliance matrix. This document is not a standards certification report.

## Top-Level Contract

| Item | Value |
| --- | --- |
| Top module | pcie_controller_top |
| Clocking | core_clk, pipe_clk, and aux_clk clock domains |
| Reset | core_rst_n active-low reset with PIPE reset outputs |
| Primary RTL/model directory | rtl |
| Primary testbench directory | tb |

## Interfaces

- PIPE interface
- AXI4 subordinate interface
- AXI4 manager interface
- configuration/status interface
- DMA control/status
- MSI/MSI-X/AER signals

## Feature Set

- Parameterized Gen1-Gen7, x1/x2/x4/x8/x16 controller model
- LTSSM, PIPE adapter, DLL TX/RX, replay/ACK/NAK concepts, and flow control
- TLP TX/RX, AXI bridge, configuration space, DMA engine, MSI/MSI-X, and AER hooks
- Root complex BFM, AXI master BFM, directed 10-test simulation, and UVM scaffolding

## Module Inventory

| Module or group | Responsibility |
| --- | --- |
| pcie_controller_top.sv | Top-level controller integration |
| pcie_ltssm.sv | Link Training and Status State Machine |
| pcie_pipe_if.sv | PIPE gearbox and ordered-set interface model |
| pcie_dll_tx.sv, pcie_dll_rx.sv | Data Link Layer TX/RX, sequence, LCRC, ACK/NAK model |
| pcie_flow_ctrl.sv | Posted, non-posted, and completion credit model |
| pcie_tlp_tx.sv, pcie_tlp_rx.sv | Transaction Layer packet generation and decode |
| pcie_cfg_space.sv | 4 KB configuration space, PCIe/PM/MSI/MSI-X/AER capabilities |
| pcie_axi_bridge.sv, pcie_dma.sv | AXI bridge and DMA datapaths |

## Register or Configuration Model

| Offset/address | Name | Access | Reset/current basis | Purpose |
| --- | --- | --- | --- | --- |
| 0x000 | Vendor/Device ID | RO/config | {DEVICE_ID,VENDOR_ID} | standard PCI config header word |
| 0x004 | Command/Status | RW/RO fields | capability-list present | PCI command and status fields |
| 0x010-0x024 | BAR0-BAR5 | RW/config | BAR0 64-bit prefetchable model | base address register model |
| 0x040-0x06c | PCIe Capability | RW/RO fields | Gen7/x16 capability model | device/link capability and control/status |
| 0x080-0x084 | PM Capability | RW/RO fields | D0 model | power-management capability |
| 0x090-0x09c | MSI Capability | RW/config | enabled by parameter | MSI address/data/control model |
| 0x0a0-0x0a8 | MSI-X Capability | RW/config | enabled by parameter | MSI-X table/PBA descriptors |
| 0x100-0x11c | AER Extended Capability | RW1C/RW fields | enabled by parameter | correctable and uncorrectable error reporting model |

## Acceptance Criteria

- The design compiles in the supported simulation flow.
- Directed tests listed in `docs/verification_plan.md` pass without fatal errors.
- Lint is either clean or all warnings are reviewed and documented in `lint/waivers.vlt`.
- All clock and reset crossings are reviewed in `docs/cdc_rdc_checklist.md`.
- Filelists and constraints are present and reviewed before synthesis or signoff use.
- Known limitations are documented before release.
