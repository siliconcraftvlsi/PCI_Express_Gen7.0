`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// Author  : Robert Kingsly Amalathas
// Email   : robertk@microprocessorlab.com
// LinkedIn: www.linkedin.com/in/robertkingslya
// -----------------------------------------------------------------------------
// =============================================================================
// PCIe 7.0 Controller - Configuration Space
// Based on PCI Express Base Specification Rev 7.0 Section 7
// =============================================================================
// Description:
//   Implements the PCIe Configuration Space (Type 0 header for EP,
//   Type 1 for Root Port/Bridge) including:
//     - Standard PCI Header (0x000–0x03F)
//     - PCI Express Capability Structure (0x040–0x07F)
//     - Power Management Capability (PCIe-required)
//     - MSI Capability (optional)
//     - MSI-X Capability (optional)
//     - Advanced Error Reporting (AER) Extended Capability (optional)
//
//   Configuration register writes arrive from TL RX (cfg_rx_wr).
//   Configuration read completions are queued to TL TX (cfg_cpl_data/valid).
// =============================================================================

`include "pcie_pkg.sv"

module pcie_cfg_space
  import pcie_pkg::*;
#(
  parameter  pcie_role_e  DEVICE_ROLE  = ROLE_EP,
  parameter  logic [15:0] VENDOR_ID    = 16'hCAFE,
  parameter  logic [15:0] DEVICE_ID    = 16'h0001,
  parameter  logic [7:0]  REVISION_ID  = 8'h01,
  parameter  logic [23:0] CLASS_CODE   = 24'h0C0300,
  parameter  logic [7:0]  BAR0_APERTURE = 8'd24,   // log2 of BAR0 size
  parameter  bit          EN_MSI        = 1,
  parameter  bit          EN_MSIX       = 1,
  parameter  bit          EN_AER        = 1
)(
  input  logic              clk,
  input  logic              rst_n,

  // -------------------------------------------------------------------------
  // Configuration Writes from TL RX
  // -------------------------------------------------------------------------
  input  logic              cfg_wr_valid,
  input  logic [11:0]       cfg_wr_addr,   // DW address within config space
  input  logic [31:0]       cfg_wr_data,

  // -------------------------------------------------------------------------
  // Configuration Reads from TL RX
  // -------------------------------------------------------------------------
  input  logic              cfg_rd_valid,
  input  logic [11:0]       cfg_rd_addr,

  // -------------------------------------------------------------------------
  // Completion data to TL TX
  // -------------------------------------------------------------------------
  output logic [255:0]      cfg_cpl_data,
  output logic              cfg_cpl_valid,
  input  logic              cfg_cpl_ready,

  // -------------------------------------------------------------------------
  // Decoded Configuration Outputs
  // -------------------------------------------------------------------------
  output logic [2:0]        mps,            // Max Payload Size
  output logic [2:0]        mrrs,           // Max Read Request Size
  output logic              relaxed_order_en,
  output logic              bus_master_en,
  output logic              mem_space_en,

  // -------------------------------------------------------------------------
  // Interrupt Interface
  // -------------------------------------------------------------------------
  output logic              msi_irq,
  output logic [4:0]        msi_vector,
  output logic              msix_irq,
  output logic [10:0]       msix_vector,
  input  logic              intx_assert,

  // Simulation-only interrupt mode override (tie off in production)
  input  logic              sim_int_override,
  input  logic              sim_msi_en,
  input  logic              sim_msix_en,

  // -------------------------------------------------------------------------
  // Error Inputs
  // -------------------------------------------------------------------------
  input  logic              err_cor,
  input  logic              err_nonfatal,
  input  logic              err_fatal
);

  // ---------------------------------------------------------------------------
  // Configuration Space Array (4 KB = 1024 DWs)
  // ---------------------------------------------------------------------------
  logic [31:0]  cfg_space [1024];

  // ---------------------------------------------------------------------------
  // Config Space Address Map
  // ---------------------------------------------------------------------------
  localparam int DW_VENDOR_DEVICE   = 0;   // 0x000: VendorID[15:0] | DeviceID[31:16]
  localparam int DW_CMD_STATUS      = 1;   // 0x004: Command | Status
  localparam int DW_CLASS_REV       = 2;   // 0x008: ClassCode | RevisionID
  localparam int DW_BIST_HDR        = 3;   // 0x00C: BIST, Header Type, Lat Timer, Cache Line
  localparam int DW_BAR0            = 4;   // 0x010: BAR0
  localparam int DW_BAR1            = 5;   // 0x014: BAR1
  localparam int DW_BAR2            = 6;   // 0x018: BAR2
  localparam int DW_BAR3            = 7;   // 0x01C: BAR3
  localparam int DW_BAR4            = 8;   // 0x020: BAR4
  localparam int DW_BAR5            = 9;   // 0x024: BAR5
  localparam int DW_CARDBUS_CIS     = 10;  // 0x028
  localparam int DW_SUBSYS          = 11;  // 0x02C: SubsysVendorID | SubsysID
  localparam int DW_EXP_ROM         = 12;  // 0x030: Expansion ROM BAR
  localparam int DW_CAP_PTR         = 13;  // 0x034: Capabilities Pointer
  localparam int DW_INT_LINE        = 15;  // 0x03C: Int Line, Pin, Min Gnt, Max Lat

  // PCIe Capability at offset 0x40 (DW 16)
  localparam int DW_PCIE_CAP_HDR    = 16;  // 0x040: CapID=0x10, NextPtr, PCIe CapReg
  localparam int DW_PCIE_DEV_CAP    = 17;  // 0x044: Device Capabilities
  localparam int DW_PCIE_DEV_CTL    = 18;  // 0x048: Device Control / Status
  localparam int DW_PCIE_LINK_CAP   = 19;  // 0x04C: Link Capabilities
  localparam int DW_PCIE_LINK_CTL   = 20;  // 0x050: Link Control / Status
  localparam int DW_PCIE_SLOT_CAP   = 21;  // 0x054: Slot Capabilities (RP only)
  localparam int DW_PCIE_SLOT_CTL   = 22;  // 0x058: Slot Control / Status
  localparam int DW_PCIE_ROOT_CTL   = 23;  // 0x05C: Root Control / Status (RP)
  localparam int DW_PCIE_DEV_CAP2   = 24;  // 0x060: Device Capabilities 2
  localparam int DW_PCIE_DEV_CTL2   = 25;  // 0x064: Device Control 2
  localparam int DW_PCIE_LINK_CAP2  = 26;  // 0x068: Link Capabilities 2
  localparam int DW_PCIE_LINK_CTL2  = 27;  // 0x06C: Link Control/Status 2

  // PM Capability at 0x80 (DW 32)
  localparam int DW_PM_CAP_HDR      = 32;  // 0x080: CapID=0x01
  localparam int DW_PM_CTL_STATUS   = 33;  // 0x084

  // MSI Capability at 0x90 (DW 36)
  localparam int DW_MSI_CAP_HDR     = 36;  // 0x090: CapID=0x05
  localparam int DW_MSI_ADDR_LO     = 37;  // 0x094
  localparam int DW_MSI_ADDR_HI     = 38;  // 0x098
  localparam int DW_MSI_DATA        = 39;  // 0x09C

  // MSI-X Capability at 0xA0 (DW 40)
  localparam int DW_MSIX_CAP_HDR   = 40;  // 0x0A0: CapID=0x11
  localparam int DW_MSIX_TABLE_OFF  = 41;  // 0x0A4
  localparam int DW_MSIX_PBA_OFF    = 42;  // 0x0A8
  // TB/backdoor MSI-X table (4 vectors × 4 DW) and PBA pending shadow
  localparam int MSIX_TB_BASE_DW    = 128;
  localparam int MSIX_TB_ENTRIES    = 4;
  localparam int MSIX_TB_DWNS       = MSIX_TB_ENTRIES * 4;
  localparam int MSIX_TB_PBA_DW     = 144;

  logic [31:0] msix_table [MSIX_TB_DWNS-1:0];
  logic [MSIX_TB_ENTRIES-1:0] msix_pba_pending;

  // AER Extended Capability at 0x100 (DW 64)
  localparam int DW_AER_CAP_HDR    = 64;  // 0x100: ExtCapID=0x0001
  localparam int DW_AER_UNCORR_STS = 65;  // 0x104
  localparam int DW_AER_UNCORR_MSK = 66;  // 0x108
  localparam int DW_AER_UNCORR_SEV = 67;  // 0x10C
  localparam int DW_AER_CORR_STS   = 68;  // 0x110
  localparam int DW_AER_CORR_MSK   = 69;  // 0x114
  localparam int DW_AER_ADVCAP_CTL = 70;  // 0x118
  localparam int DW_AER_HDR_LOG_0  = 71;  // 0x11C

  // ---------------------------------------------------------------------------
  // Reset / initialization of read-only fields
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      // Clear all
      for (int i = 0; i < 1024; i++)
        cfg_space[i] <= 32'h0;

      // Standard header
      cfg_space[DW_VENDOR_DEVICE] <= {DEVICE_ID, VENDOR_ID};
      cfg_space[DW_CMD_STATUS]    <= 32'h0010_0000;  // Capabilities list present
      cfg_space[DW_CLASS_REV]     <= {CLASS_CODE, REVISION_ID};
      cfg_space[DW_BIST_HDR]      <= (DEVICE_ROLE == ROLE_RP) ?
                                      32'h0000_0100 :  // Type1
                                      32'h0000_0000;   // Type0
      // BAR0: 64-bit memory BAR
      cfg_space[DW_BAR0]          <= 32'hFFFF_FFF4;   // 64-bit prefetchable
      cfg_space[DW_BAR1]          <= 32'hFFFF_FFFF;
      cfg_space[DW_BAR2]          <= 32'h0000_000C;   // 32-bit mem BAR for MSI-X table
      cfg_space[DW_CAP_PTR]       <= 32'h0000_0040;   // First capability at 0x40

      // PCIe Capability (CapID=0x10)
      cfg_space[DW_PCIE_CAP_HDR]  <= (DEVICE_ROLE == ROLE_EP) ?
                                      32'h0002_8010 :  // EP, version 2
                                      32'h0004_8010;   // RP, version 2
      cfg_space[DW_PCIE_DEV_CAP]  <= {19'd0, MPS_4KB, 10'd0};  // MaxPayload=4KB cap
      cfg_space[DW_PCIE_DEV_CTL]  <= 32'h0000_2810;   // MPS=256B, MRRS=512B
      cfg_space[DW_PCIE_LINK_CAP] <= {4'd15, 1'b0, 1'b0, 3'b001,  // x16 lanes, ASPM L0s+L1
                                        10'd16, 4'd7};  // Gen7, x16
      cfg_space[DW_PCIE_LINK_CTL] <= 32'h0000_0000;
      cfg_space[DW_PCIE_DEV_CAP2] <= 32'h000F_07E0;   // Various caps

      // PM Capability (CapID=0x01)
      cfg_space[DW_PM_CAP_HDR]    <= 32'h0000_8001;   // CapID=0x01, PM v1.2
      cfg_space[DW_PM_CTL_STATUS] <= 32'h0000_0008;   // D0 state

      // MSI Capability (CapID=0x05)
      if (EN_MSI) begin
        cfg_space[DW_MSI_CAP_HDR] <= 32'h0081_9005;   // CapID=0x05, 32-vector MSI, MSI Enable=1
        cfg_space[DW_MSI_ADDR_LO] <= 32'h0000_0000;
        cfg_space[DW_MSI_DATA]    <= 32'h0000_0000;
      end

      // MSI-X Capability (CapID=0x11)
      if (EN_MSIX) begin
        cfg_space[DW_MSIX_CAP_HDR]  <= 32'h07FF_A011;  // 2048 vectors, CapID=0x11
        cfg_space[DW_MSIX_TABLE_OFF] <= 32'h0000_4000;  // Table in BAR2
        cfg_space[DW_MSIX_PBA_OFF]   <= 32'h0001_4000;  // PBA after table
        for (int t = 0; t < MSIX_TB_DWNS; t++)
          msix_table[t] <= 32'h0;
        msix_pba_pending <= '0;
      end

      // AER Extended Capability (ExtCapID=0x0001)
      if (EN_AER) begin
        cfg_space[DW_AER_CAP_HDR]   <= 32'h1000_0001;  // ExtCapID=1, next=0x100
        cfg_space[DW_AER_UNCORR_MSK] <= 32'h0000_0000;
        cfg_space[DW_AER_CORR_MSK]  <= 32'h0000_0000;
        cfg_space[DW_AER_ADVCAP_CTL]<= 32'h0000_0000;
      end
    end else begin

      // -----------------------------------------------------------------------
      // Configuration Writes
      // -----------------------------------------------------------------------
      if (cfg_wr_valid) begin
        case (cfg_wr_addr)
          // Command register (writable bits: Bus Master, Memory Space, IO Space)
          DW_CMD_STATUS[11:0]: begin
            cfg_space[DW_CMD_STATUS][2:0] <= cfg_wr_data[2:0];  // IO, Mem, BusMaster
          end
          // BAR writes (for BAR sizing)
          DW_BAR0[11:0]: cfg_space[DW_BAR0] <= cfg_wr_data;
          DW_BAR1[11:0]: cfg_space[DW_BAR1] <= cfg_wr_data;
          DW_BAR2[11:0]: cfg_space[DW_BAR2] <= cfg_wr_data;
          DW_BAR3[11:0]: cfg_space[DW_BAR3] <= cfg_wr_data;
          DW_BAR4[11:0]: cfg_space[DW_BAR4] <= cfg_wr_data;
          DW_BAR5[11:0]: cfg_space[DW_BAR5] <= cfg_wr_data;
          // PCIe Device Control (MPS, MRRS, relax order, etc.)
          DW_PCIE_DEV_CTL[11:0]: begin
            cfg_space[DW_PCIE_DEV_CTL][14:12] <= cfg_wr_data[14:12]; // MRRS
            cfg_space[DW_PCIE_DEV_CTL][7:5]   <= cfg_wr_data[7:5];   // MPS
            cfg_space[DW_PCIE_DEV_CTL][4]      <= cfg_wr_data[4];     // Relaxed ordering
            cfg_space[DW_PCIE_DEV_CTL][11]     <= cfg_wr_data[11];    // No snoop
          end
          // PM Control
          DW_PM_CTL_STATUS[11:0]: begin
            cfg_space[DW_PM_CTL_STATUS][1:0] <= cfg_wr_data[1:0];
          end
          // MSI Control
          DW_MSI_CAP_HDR[11:0]: begin
            if (EN_MSI)
              cfg_space[DW_MSI_CAP_HDR][16] <= cfg_wr_data[16]; // MSI Enable
          end
          DW_MSI_ADDR_LO[11:0]: if (EN_MSI) cfg_space[DW_MSI_ADDR_LO] <= cfg_wr_data;
          DW_MSI_ADDR_HI[11:0]: if (EN_MSI) cfg_space[DW_MSI_ADDR_HI] <= cfg_wr_data;
          DW_MSI_DATA[11:0]:    if (EN_MSI) cfg_space[DW_MSI_DATA]    <= cfg_wr_data;
          // MSI-X Control
          DW_MSIX_CAP_HDR[11:0]: begin
            if (EN_MSIX) begin
              cfg_space[DW_MSIX_CAP_HDR][31] <= cfg_wr_data[31]; // MSI-X Enable
              cfg_space[DW_MSIX_CAP_HDR][30] <= cfg_wr_data[30]; // Function Mask
            end
          end
          // AER Status (RW1C)
          DW_AER_UNCORR_STS[11:0]: if (EN_AER) cfg_space[DW_AER_UNCORR_STS] <= cfg_space[DW_AER_UNCORR_STS] & ~cfg_wr_data;
          DW_AER_CORR_STS[11:0]:   if (EN_AER) cfg_space[DW_AER_CORR_STS]   <= cfg_space[DW_AER_CORR_STS]   & ~cfg_wr_data;
          default: begin
            if (EN_MSIX && cfg_wr_addr >= MSIX_TB_BASE_DW &&
                cfg_wr_addr < (MSIX_TB_BASE_DW + MSIX_TB_DWNS))
              msix_table[int'(cfg_wr_addr - MSIX_TB_BASE_DW)] <= cfg_wr_data;
            else if (EN_MSIX && cfg_wr_addr == MSIX_TB_PBA_DW)
              msix_pba_pending <= cfg_wr_data[MSIX_TB_ENTRIES-1:0];
          end
        endcase
      end

      // -----------------------------------------------------------------------
      // Error Status Update (AER) — defer sticky set when same DW is RW1C-cleared
      // -----------------------------------------------------------------------
      if (EN_AER) begin
        if (err_cor && !(cfg_wr_valid && (cfg_wr_addr == DW_AER_CORR_STS[11:0])))
          cfg_space[DW_AER_CORR_STS] <= cfg_space[DW_AER_CORR_STS] | 32'h0000_0001;
        if (err_nonfatal && !(cfg_wr_valid && (cfg_wr_addr == DW_AER_UNCORR_STS[11:0])))
          cfg_space[DW_AER_UNCORR_STS] <= cfg_space[DW_AER_UNCORR_STS] | 32'h0000_2000;
        if (err_fatal && !(cfg_wr_valid && (cfg_wr_addr == DW_AER_UNCORR_STS[11:0])))
          cfg_space[DW_AER_UNCORR_STS] <= cfg_space[DW_AER_UNCORR_STS] | 32'h0001_0000;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Configuration Read Response
  // ---------------------------------------------------------------------------
  logic         rd_pending;
  logic [11:0]  rd_addr_hold;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cfg_cpl_valid <= 1'b0;
      cfg_cpl_data  <= '0;
      rd_pending    <= 1'b0;
      rd_addr_hold  <= '0;
    end else begin
      if (cfg_rd_valid) begin
        rd_pending   <= 1'b1;
        rd_addr_hold <= cfg_rd_addr;
      end
      if (rd_pending && cfg_cpl_ready) begin
        // Build completion TLP header + data (simplified: 12DW = 3DW hdr + 1DW data)
        cfg_cpl_data  <= {cfg_space[rd_addr_hold], {(256-32){1'b0}}};
        cfg_cpl_valid <= 1'b1;
        rd_pending    <= 1'b0;
      end else if (cfg_cpl_valid && cfg_cpl_ready) begin
        cfg_cpl_valid <= 1'b0;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Decoded outputs
  // ---------------------------------------------------------------------------
  assign mps              = cfg_space[DW_PCIE_DEV_CTL][7:5];
  assign mrrs             = cfg_space[DW_PCIE_DEV_CTL][14:12];
  assign relaxed_order_en = cfg_space[DW_PCIE_DEV_CTL][4];
  assign mem_space_en     = cfg_space[DW_CMD_STATUS][1];
  assign bus_master_en    = cfg_space[DW_CMD_STATUS][2];

  // ---------------------------------------------------------------------------
  // Interrupt Generation
  // ---------------------------------------------------------------------------
  // MSI: fire when software writes to MSI pending register
  logic msi_enabled;
  assign msi_enabled = EN_MSI &&
                       (sim_int_override ? sim_msi_en : cfg_space[DW_MSI_CAP_HDR][16]);
  assign msi_irq     = msi_enabled && intx_assert;
  assign msi_vector  = 5'd0;  // Single-vector MSI (simplified)

  // MSI-X: table/PBA model with sim-override for legacy TEST 18
  logic msix_enabled;
  logic        msix_pending_hit;
  logic [10:0] msix_sel_vector;

  always_comb begin
    msix_pending_hit = 1'b0;
    msix_sel_vector  = 11'd0;
    if (EN_MSIX) begin
      for (int v = MSIX_TB_ENTRIES - 1; v >= 0; v--) begin
        if (msix_pba_pending[v] && !msix_table[v * 4 + 3][0]) begin
          msix_pending_hit = 1'b1;
          msix_sel_vector  = 11'(v);
        end
      end
    end
  end

  assign msix_enabled = EN_MSIX &&
                        (sim_int_override ? sim_msix_en : cfg_space[DW_MSIX_CAP_HDR][31]);
  assign msix_irq     = msix_enabled && intx_assert && !msi_enabled &&
                        (sim_int_override ? 1'b1 : msix_pending_hit);
  assign msix_vector  = sim_int_override ? 11'd0 : msix_sel_vector;

endmodule : pcie_cfg_space
