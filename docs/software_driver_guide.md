# PCI Express Gen7 Controller Software Driver Guide

Generated: 2026-05-21

## Bring-Up Sequence

1. Hold reset active and keep traffic disabled.
2. Release reset and wait for the documented ready or link indication.
3. Program configuration/register fields required for the selected mode.
4. Enable datapath traffic or DMA only after status indicates ready.
5. Poll or interrupt on status/error counters during traffic.
6. On error, capture status, counters, and waveform/log evidence before clearing sticky state.

## Project-Specific Notes

A production-style driver package should include configuration-space access helpers, BAR discovery, DMA descriptor programming, MSI/MSI-X configuration, AER status dump, and link-state debug utilities.


## Driver Release Requirements

- C header generated or reviewed against RTL register definitions.
- Initialization example builds cleanly with warnings enabled.
- Negative tests cover invalid mode, timeout, and error-status readback.
- Public API documents side effects and reset values.
