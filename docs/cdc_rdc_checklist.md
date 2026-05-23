# PCI Express Gen7 Controller CDC/RDC Checklist

Generated: 2026-05-21

| Crossing or reset item | Current implementation note | Required review |
| --- | --- | --- |
| core_clk to pipe_clk | Separate domains in top-level interface | Requires CDC review and synchronizers/FIFOs for production |
| core_clk to aux_clk | Power-management auxiliary domain | RDC/CDC review required |
| PIPE reset outputs | pipe_reset_n controlled by LTSSM/PIPE logic | Reset sequencing and deassertion synchronization must be checked |

## Checklist

- [ ] Every clock domain is listed with frequency, reset source, and generated-clock relationship.
- [ ] Every async input has a synchronizer, async FIFO, handshake, or documented assumption.
- [ ] Every reset deassertion is synchronous to the receiving domain or waived with rationale.
- [ ] Multi-bit crossings use stable handshake, FIFO, or encoded pointer method.
- [ ] CDC/RDC tool reports are reviewed and archived before release.
