# PCI Express Gen7 Controller Register and Configuration Map

Generated: 2026-05-21

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

## Access Policy

- RO fields must ignore writes or preserve documented behavior.
- RW fields must be included in reset-value and readback tests.
- Counter fields must document clear, saturate, wrap, or write behavior.
- Side effects such as read-pop, write-one-to-clear, start pulses, or self-clearing bits must be tested and documented before release.
