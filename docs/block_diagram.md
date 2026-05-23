# PCI Express Gen7 Controller Block Diagram

Generated: 2026-05-21

```mermaid
flowchart LR
  AXI[AXI4 Subordinate] --> BRIDGE[AXI Bridge]
  BRIDGE --> TLP_TX[TLP TX]
  DMA[DMA Engine] --> TLP_TX
  TLP_TX --> FC[Flow Control]
  FC --> DLLTX[DLL TX / Replay]
  DLLTX --> PIPE[PIPE Adapter]
  PIPE --> PHY[PIPE PHY / RC BFM]
  PHY --> DLLRX[DLL RX]
  DLLRX --> TLP_RX[TLP RX]
  TLP_RX --> BRIDGE
  CFG[Config Space / MSI / AER] --> TLP_TX
  LTSSM[LTSSM] --> PIPE
```


The diagram is a delivery-level block view, not a gate-level schematic. Use it to orient reviews and documentation; use the RTL files for exact connectivity.
