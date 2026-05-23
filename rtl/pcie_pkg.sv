// -----------------------------------------------------------------------------
// Author  : Robert Kingsly Amalathas
// Email   : robertk@microprocessorlab.com
// LinkedIn: www.linkedin.com/in/robertkingslya
// -----------------------------------------------------------------------------
// =============================================================================
// PCIe 7.0 Controller - Package: Parameters, Types, and Defines
// Based on PCI Express 7.0 Specification
// =============================================================================
// Description:
//   Central package containing all parameters, typedefs, enumerations, and
//   constant definitions used across the PCIe 7.0 Controller IP.
//   Covers Transaction Layer, Data Link Layer, and Physical Layer constants.
// =============================================================================

`ifndef PCIE_PKG_SV
`define PCIE_PKG_SV
`timescale 1ns/1ps


package pcie_pkg;

  // ---------------------------------------------------------------------------
  // Link Configuration Parameters
  // ---------------------------------------------------------------------------
  parameter int unsigned MAX_LANES        = 16;
  parameter int unsigned DATA_WIDTH       = 256;   // Internal datapath width (bits): 64/128/256/512/1024
  parameter int unsigned STRB_WIDTH       = DATA_WIDTH / 8;
  parameter int unsigned ADDR_WIDTH       = 64;    // AXI address width
  parameter int unsigned AXI_ID_WIDTH     = 8;
  parameter int unsigned AXI_USER_WIDTH   = 4;

  // ---------------------------------------------------------------------------
  // PCIe Speed / Generation Encoding
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    PCIE_GEN1   = 4'h1,  // 2.5  GT/s
    PCIE_GEN2   = 4'h2,  // 5.0  GT/s
    PCIE_GEN3   = 4'h3,  // 8.0  GT/s
    PCIE_GEN4   = 4'h4,  // 16.0 GT/s
    PCIE_GEN5   = 4'h5,  // 32.0 GT/s
    PCIE_GEN6   = 4'h6,  // 64.0 GT/s
    PCIE_GEN7   = 4'h7   // 128  GT/s
  } pcie_gen_e;

  // ---------------------------------------------------------------------------
  // TLP (Transaction Layer Packet) Type Encoding
  // PCIe Base Spec Table 2-3
  // ---------------------------------------------------------------------------
  typedef enum logic [4:0] {
    TLP_MRd32       = 5'b00000,  // Memory Read 32-bit address
    TLP_MRd64       = 5'b00001,  // Memory Read 64-bit address
    TLP_MRdLk32     = 5'b00010,  // Memory Read Locked 32-bit / IO Read (fmt distinguishes)
    TLP_MRdLk64     = 5'b00011,  // Memory Read Locked 64-bit
    TLP_MWr32       = 5'b10000,  // Memory Write 32-bit address / LPrfxT0 (fmt distinguishes)
    TLP_MWr64       = 5'b10001,  // Memory Write 64-bit address / LPrfxT1 (fmt distinguishes)
    TLP_IOWr        = 5'b10010,  // IO Write
    TLP_CfgRd0      = 5'b00100,  // Config Read Type 0
    TLP_CfgWr0      = 5'b10100,  // Config Write Type 0
    TLP_CfgRd1      = 5'b00101,  // Config Read Type 1
    TLP_CfgWr1      = 5'b10101,  // Config Write Type 1
    TLP_Msg         = 5'b10110,  // Message without data
    TLP_MsgD        = 5'b10111,  // Message with data
    TLP_Cpl         = 5'b01010,  // Completion without data
    TLP_CplD        = 5'b01011,  // Completion with data
    TLP_CplLk       = 5'b01100,  // Completion for Locked MR without data / FetchAdd
    TLP_CplDLk      = 5'b01101,  // Completion for Locked MR with data / Swap
    TLP_AtomicOpCpl = 5'b01110,  // AtomicOp Completion / CAS
    TLP_AtomicOpReq = 5'b11100   // AtomicOp Request
  } tlp_type_e;

  // ---------------------------------------------------------------------------
  // Completion Status Codes
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    CPL_SC   = 3'b000,  // Successful Completion
    CPL_UR   = 3'b001,  // Unsupported Request
    CPL_CRS  = 3'b010,  // Configuration Request Retry Status
    CPL_CA   = 3'b100   // Completer Abort
  } cpl_status_e;

  // ---------------------------------------------------------------------------
  // TLP Header Format (DW0 fields)
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic [2:0]  fmt;       // Format
    logic [4:0]  tlp_type;  // Type
    logic        T9;        // TLP Processing Hint bit 1
    logic [2:0]  tc;        // Traffic Class
    logic        T8;        // TLP Processing Hint bit 0
    logic        attr2;     // Attribute bit 2
    logic        ln;        // Lightweight Notification
    logic        th;        // TLP Hints Present
    logic        td;        // TLP Digest (ECRC)
    logic        ep;        // Poisoned Data
    logic [1:0]  attr;      // Attributes [Relaxed Ordering, No Snoop]
    logic [1:0]  at;        // Address Type
    logic [9:0]  length;    // Data length in DW
  } tlp_dw0_t;

  // ---------------------------------------------------------------------------
  // TLP Header (3DW non-posted request, 32-bit address)
  // ---------------------------------------------------------------------------
  typedef struct packed {
    tlp_dw0_t    dw0;
    logic [15:0] requester_id;
    logic [9:0]  tag;
    logic [3:0]  last_be;
    logic [3:0]  first_be;
    logic [31:0] addr;
  } tlp_hdr_3dw_t;

  // ---------------------------------------------------------------------------
  // TLP Header (4DW, 64-bit address)
  // ---------------------------------------------------------------------------
  typedef struct packed {
    tlp_dw0_t    dw0;
    logic [15:0] requester_id;
    logic [9:0]  tag;
    logic [3:0]  last_be;
    logic [3:0]  first_be;
    logic [63:0] addr;
  } tlp_hdr_4dw_t;

  // ---------------------------------------------------------------------------
  // Completion Header
  // ---------------------------------------------------------------------------
  typedef struct packed {
    tlp_dw0_t    dw0;
    logic [15:0] completer_id;
    cpl_status_e status;
    logic        bcm;        // Byte Count Modified
    logic [11:0] byte_count;
    logic [15:0] requester_id;
    logic [9:0]  tag;
    logic        rsvd;
    logic [6:0]  lower_addr;
  } tlp_cpl_hdr_t;

  // ---------------------------------------------------------------------------
  // DLLP (Data Link Layer Packet) Types
  // ---------------------------------------------------------------------------
  typedef enum logic [7:0] {
    DLLP_ACK      = 8'h00,
    DLLP_NAK      = 8'h10,
    DLLP_PM_ENTER_L1      = 8'h20,
    DLLP_PM_ENTER_L23     = 8'h21,
    DLLP_PM_ACT_STATE_REQ = 8'h22,
    DLLP_PM_REQ_ACK       = 8'h24,
    DLLP_VENDOR           = 8'h30,
    DLLP_FC_INIT_P        = 8'h40,  // Flow control Init: Posted
    DLLP_FC_INIT_NP       = 8'h50,  // Flow control Init: Non-Posted
    DLLP_FC_INIT_CPL      = 8'h60,  // Flow control Init: Completion
    DLLP_FC_UPD_P         = 8'hC0,  // Flow control Update: Posted
    DLLP_FC_UPD_NP        = 8'hD0,  // Flow control Update: Non-Posted
    DLLP_FC_UPD_CPL       = 8'hE0   // Flow control Update: Completion
  } dllp_type_e;

  // ---------------------------------------------------------------------------
  // LTSSM States
  // ---------------------------------------------------------------------------
  typedef enum logic [5:0] {
    DETECT_QUIET          = 6'h00,
    DETECT_ACTIVE         = 6'h01,
    POLLING_ACTIVE        = 6'h02,
    POLLING_COMPLIANCE    = 6'h03,
    POLLING_CONFIGURATION = 6'h04,
    POLLING_SPEED         = 6'h05,
    CONFIG_LWIDTH_START   = 6'h06,
    CONFIG_LWIDTH_ACCEPT  = 6'h07,
    CONFIG_LANENUM_WAIT   = 6'h08,
    CONFIG_LANENUM_ACCEPT = 6'h09,
    CONFIG_COMPLETE       = 6'h0A,
    CONFIG_IDLE           = 6'h0B,
    RECOVERY_RCVRLOCK     = 6'h0C,
    RECOVERY_RCVRCFG      = 6'h0D,
    RECOVERY_IDLE         = 6'h0E,
    RECOVERY_EQUALIZATION = 6'h0F,
    L0                    = 6'h10,
    L0S_TX                = 6'h11,
    L0S_RX                = 6'h12,
    L1_ENTRY              = 6'h13,
    L1_IDLE               = 6'h14,
    L2_IDLE               = 6'h15,
    L2_TX_WAKE            = 6'h16,
    HOT_RESET             = 6'h17,
    DISABLED              = 6'h18,
    LOOPBACK_ENTRY        = 6'h19,
    LOOPBACK_ACTIVE       = 6'h1A,
    LOOPBACK_EXIT         = 6'h1B
  } ltssm_state_e;

  // ---------------------------------------------------------------------------
  // Data Link Layer States
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    DL_INACTIVE = 2'b00,  // DLL not yet active; waiting for LTSSM CONFIG_IDLE exit
    DL_INIT     = 2'b01,  // FC initialization exchange in progress
    DL_ACTIVE   = 2'b10,  // DLL fully operational
    DL_ERROR    = 2'b11   // DLL error (e.g. excessive NAKs)
  } dll_state_e;

  // ---------------------------------------------------------------------------
  // Equalization Phase Encoding (Gen3+ link equalization)
  // PCIe Base Spec Section 4.2.6
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    EQ_PHASE0 = 2'b00,  // Preset: transmitter applies preset coefficients
    EQ_PHASE1 = 2'b01,  // Phase 1: DS port evaluates and requests coefficients
    EQ_PHASE2 = 2'b10,  // Phase 2: DS port requests final coefficients
    EQ_PHASE3 = 2'b11   // Phase 3: US port evaluates DS coefficients
  } eq_phase_e;

  // ---------------------------------------------------------------------------
  // Flow Control Credit Types
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    FC_POSTED     = 2'b00,
    FC_NONPOSTED  = 2'b01,
    FC_COMPLETION = 2'b10
  } fc_type_e;

  // ---------------------------------------------------------------------------
  // Flow Control Credit Structure
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic [11:0] header_credits;   // Header credits (infinite = 12'hFFF)
    logic [19:0] data_credits;     // Data credits in units of 4DW (infinite = 20'hFFFFF)
  } fc_credits_t;

  // ---------------------------------------------------------------------------
  // DLLP Packed Structure (48 bits total)
  //   [47:40] dllp_type  (8 bits)
  //   [39:16] reserved_or_seq (24 bits; for ACK/NAK bits [27:16] hold seq)
  //   [15:0]  dlcrc      (16 bits)
  // ---------------------------------------------------------------------------
  typedef struct packed {
    dllp_type_e  dllp_type;         // 8-bit DLLP type code
    logic [23:0] reserved_or_seq;   // Sequence/reserved field (type-dependent)
    logic [15:0] dlcrc;             // 16-bit DL CRC
  } dllp_t;

  // ---------------------------------------------------------------------------
  // FC DLLP Structure (48 bits total)
  //   [47:40] fc_type    (8 bits  — DLLP_FC_INIT_P/NP/CPL or UPD)
  //   [39:28] hdr_fc     (12 bits — header credit value)
  //   [27: 8] data_fc    (20 bits — data credit value)
  //   [ 7: 0] dlcrc_lo   (low 8 bits; upper 8 in separate field, stored as 16)
  // Practical layout: {fc_type[7:0], hdr_fc[11:0], data_fc[19:0], dlcrc[15:0]}
  // = 8+12+20+16 = 56 bits — pack as two words; keep in struct for clarity.
  // ---------------------------------------------------------------------------
  typedef struct packed {
    dllp_type_e  fc_type;    // FC DLLP type (init or update, P/NP/CPL)
    logic [11:0] hdr_fc;     // Header credit field (12-bit)
    logic [19:0] data_fc;    // Data credit field (20-bit)
    logic [15:0] dlcrc;      // 16-bit DL CRC
  } fc_dllp_t;

  // ---------------------------------------------------------------------------
  // BAR Configuration Structure
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic [63:0] base_addr;    // Base address (64-bit capable)
    logic [63:0] mask;         // Address mask (size - 1)
    logic        prefetchable; // BAR is prefetchable memory
    logic        mem64;        // BAR is 64-bit memory BAR
    logic        io_bar;       // BAR is an IO BAR (not memory)
  } bar_cfg_t;

  // ---------------------------------------------------------------------------
  // Link Training Status Register
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic        link_up;            // Link is in L0
    pcie_gen_e   speed;              // Negotiated PCIe generation
    logic [4:0]  width;              // Negotiated link width (1,2,4,8,16)
    logic [2:0]  eq_phase;           // Current equalization phase (0-3)
    logic        upconfigure_capable;// Link supports upconfigure (width change)
    logic        retrain_link;       // SW-writable: trigger recovery/retrain
  } link_train_status_t;

  // ---------------------------------------------------------------------------
  // AXI Transaction Types
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    AXI_BURST_FIXED  = 2'b00,
    AXI_BURST_INCR   = 2'b01,
    AXI_BURST_WRAP   = 2'b10,
    AXI_BURST_RSVD   = 2'b11
  } axi_burst_e;

  typedef enum logic [1:0] {
    AXI_RESP_OKAY    = 2'b00,
    AXI_RESP_EXOKAY  = 2'b01,
    AXI_RESP_SLVERR  = 2'b10,
    AXI_RESP_DECERR  = 2'b11
  } axi_resp_e;

  // AXI4 AxCACHE encoding constants (AXI4 Spec Table A4-5)
  localparam logic [3:0] AXI_CACHE_NON_CACHEABLE     = 4'b0000; // Non-bufferable, non-cacheable
  localparam logic [3:0] AXI_CACHE_BUFFERABLE        = 4'b0001; // Bufferable only
  localparam logic [3:0] AXI_CACHE_WT_NO_ALLOC       = 4'b0110; // Write-through, no allocate
  localparam logic [3:0] AXI_CACHE_WB_NO_ALLOC       = 4'b0111; // Write-back, no allocate
  localparam logic [3:0] AXI_CACHE_WT_RA             = 4'b1110; // Write-through, read-allocate
  localparam logic [3:0] AXI_CACHE_WB_RA             = 4'b1111; // Write-back, read-allocate (no WA)
  localparam logic [3:0] AXI_CACHE_WB_RA_WA          = 4'b1111; // Write-back, read+write allocate (full cache)

  // ---------------------------------------------------------------------------
  // PIPE Interface Width Encoding
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    PIPE_8BIT   = 3'b000,
    PIPE_16BIT  = 3'b001,
    PIPE_32BIT  = 3'b010,
    PIPE_64BIT  = 3'b011,
    PIPE_128BIT = 3'b100
  } pipe_width_e;

  // ---------------------------------------------------------------------------
  // PCIe Configuration Space Header Type
  // ---------------------------------------------------------------------------
  typedef enum logic [6:0] {
    CFG_TYPE0 = 7'h00,  // Endpoint
    CFG_TYPE1 = 7'h01   // Bridge / Root Port
  } cfg_hdr_type_e;

  // ---------------------------------------------------------------------------
  // Device/Function Role
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    ROLE_EP     = 2'b00,  // Endpoint
    ROLE_RP     = 2'b01,  // Root Port
    ROLE_DM     = 2'b10,  // Dual Mode
    ROLE_SW     = 2'b11   // Switch Port
  } pcie_role_e;

  // ---------------------------------------------------------------------------
  // Error Reporting Types
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    ERR_COR      = 3'b000,  // Correctable Error
    ERR_NONFATAL = 3'b001,  // Non-fatal Uncorrectable
    ERR_FATAL    = 3'b010,  // Fatal Uncorrectable
    ERR_NONE     = 3'b111
  } pcie_err_type_e;

  // ---------------------------------------------------------------------------
  // Message Codes (PCIe Spec Table 2-19)
  // ---------------------------------------------------------------------------
  localparam logic [7:0] MSG_ASSERT_INTA   = 8'h20;
  localparam logic [7:0] MSG_ASSERT_INTB   = 8'h21;
  localparam logic [7:0] MSG_ASSERT_INTC   = 8'h22;
  localparam logic [7:0] MSG_ASSERT_INTD   = 8'h23;
  localparam logic [7:0] MSG_DEASSERT_INTA = 8'h24;
  localparam logic [7:0] MSG_DEASSERT_INTB = 8'h25;
  localparam logic [7:0] MSG_DEASSERT_INTC = 8'h26;
  localparam logic [7:0] MSG_DEASSERT_INTD = 8'h27;
  localparam logic [7:0] MSG_PM_ACT_STATE  = 8'h18;
  localparam logic [7:0] MSG_PM_PME        = 8'h18;
  localparam logic [7:0] MSG_ERR_COR       = 8'h30;
  localparam logic [7:0] MSG_ERR_NONFATAL  = 8'h31;
  localparam logic [7:0] MSG_ERR_FATAL     = 8'h33;
  localparam logic [7:0] MSG_UNLOCK        = 8'h00;
  localparam logic [7:0] MSG_SET_SLOT_PWR  = 8'h50;

  // ---------------------------------------------------------------------------
  // Timing Constants (in clock cycles at 250 MHz unless noted)
  // ---------------------------------------------------------------------------
  localparam int unsigned REPLAY_TIMER_INIT  = 4096;
  localparam int unsigned ACK_LATENCY_TIMER  = 256;
  localparam int unsigned FC_UPDATE_TIMER    = 200000;  // ~0.8 ms
  localparam int unsigned LINK_UP_TIMEOUT    = 24'hFFFFFF;

  // Equalization per-phase timeouts (250 MHz clock cycles)
  // EQ_PHASE1_TIMEOUT = 500,000 cycles = 2 ms  (PCIe spec: 2 ms max)
  // EQ_PHASE2_TIMEOUT = 500,000 cycles = 2 ms
  // EQ_PHASE3_TIMEOUT =  12,500 cycles = 50 µs
  localparam logic [23:0] EQ_PHASE1_TIMEOUT = 24'd500_000;
  localparam logic [23:0] EQ_PHASE2_TIMEOUT = 24'd500_000;
  localparam logic [23:0] EQ_PHASE3_TIMEOUT = 24'd12_500;

  // Power management exit latency timeouts
  localparam logic [23:0] L0S_EXIT_TIMEOUT  = 24'd250;      // ~1 µs
  localparam logic [23:0] L1_EXIT_TIMEOUT   = 24'd4_000;    // ~16 µs
  localparam logic [23:0] DLL_INIT_TIMEOUT  = 24'd2_000_000;// 8 ms

  // ---------------------------------------------------------------------------
  // Maximum Payload Sizes (encoded as log2(bytes)-7)
  // 000=128B, 001=256B, 010=512B, 011=1024B, 100=2048B, 101=4096B
  // ---------------------------------------------------------------------------
  localparam logic [2:0] MPS_128B  = 3'b000;
  localparam logic [2:0] MPS_256B  = 3'b001;
  localparam logic [2:0] MPS_512B  = 3'b010;
  localparam logic [2:0] MPS_1KB   = 3'b011;
  localparam logic [2:0] MPS_2KB   = 3'b100;
  localparam logic [2:0] MPS_4KB   = 3'b101;

  // ---------------------------------------------------------------------------
  // CRC Polynomial Constants
  // ---------------------------------------------------------------------------
  localparam logic [31:0] LCRC_INIT  = 32'hFFFFFFFF;
  localparam logic [15:0] DLCRC_INIT = 16'hFFFF;

  // ---------------------------------------------------------------------------
  // Retry Buffer Depth
  // ---------------------------------------------------------------------------
  parameter int unsigned RETRY_BUF_DEPTH = 2048;

  // ---------------------------------------------------------------------------
  // Tag Space
  // ---------------------------------------------------------------------------
  parameter int unsigned MAX_TAGS = 1024;  // 10-bit tags

  // DMA tag allocation window (tags 512-767 reserved for DMA engine)
  localparam logic [9:0] DMA_TAG_BASE  = 10'd512;
  localparam logic [9:0] DMA_TAG_LIMIT = 10'd767;
  // AXI (host-initiated) tag allocation window (tags 0-511)
  localparam logic [9:0] AXI_TAG_BASE  = 10'd0;
  localparam logic [9:0] AXI_TAG_LIMIT = 10'd511;

  // ---------------------------------------------------------------------------
  // Virtual Channels
  // ---------------------------------------------------------------------------
  parameter int unsigned NUM_VCS = 8;

  // ---------------------------------------------------------------------------
  // Traffic Classes
  // ---------------------------------------------------------------------------
  parameter int unsigned NUM_TCS = 8;

  // ---------------------------------------------------------------------------
  // MSI/MSI-X
  // ---------------------------------------------------------------------------
  parameter int unsigned MAX_MSI_VECTORS  = 32;
  parameter int unsigned MAX_MSIX_VECTORS = 2048;

  // ---------------------------------------------------------------------------
  // Replay / NAK Limit
  // 3 consecutive NAKs (or replay-timer expirations) → DL_ERROR
  // ---------------------------------------------------------------------------
  localparam logic [3:0] REPLAY_COUNT_MAX = 4'd3;

  // ---------------------------------------------------------------------------
  // Utility Functions
  // ---------------------------------------------------------------------------

  // Calculate number of DWs from byte count
  function automatic logic [9:0] bytes_to_dw;
    input logic [11:0] bytes;
    bytes_to_dw = (bytes + 3) >> 2;
  endfunction

  // Calculate LCRC-32 (CRC-32 used for TLP LCRC)
  function automatic logic [31:0] calc_lcrc;
    input logic [31:0] crc_in;
    input logic [31:0] data;
    logic [31:0] crc;
    integer i;
    begin
      crc = crc_in;
      for (i = 0; i < 32; i++) begin
        if ((crc[31] ^ data[31-i]) == 1'b1)
          crc = (crc << 1) ^ 32'h04C11DB7;
        else
          crc = crc << 1;
      end
      calc_lcrc = crc;
    end
  endfunction

  // ECRC uses the same CRC-32 polynomial as LCRC (PCIe Base Spec §2.7.1)
  localparam logic [31:0] ECRC_INIT = LCRC_INIT;

  function automatic logic [31:0] ecrc_update_dw;
    input logic [31:0] crc_in;
    input logic [31:0] dw;
    ecrc_update_dw = calc_lcrc(crc_in, dw);
  endfunction

  function automatic logic [31:0] ecrc_finalize;
    input logic [31:0] crc_in;
    ecrc_finalize = ~crc_in;
  endfunction

  // Accumulate ECRC over n_dw dwords from the MSB of a DATA_W beat (n_dw in 1..8)
  function automatic logic [31:0] ecrc_update_beat;
    input logic [31:0] crc_in;
    input logic [255:0] beat;
    input int unsigned n_dw;
    logic [31:0] crc;
    int unsigned i;
    begin
      crc = crc_in;
      for (i = 0; i < n_dw; i = i + 1)
        crc = calc_lcrc(crc, beat[255 - 32*i -: 32]);
      ecrc_update_beat = crc;
    end
  endfunction

  function automatic logic tlp_is_posted_dw0;
    input tlp_dw0_t dw0;
    tlp_is_posted_dw0 = dw0.fmt[2];
  endfunction

  function automatic logic tlp_relaxed_ordering_dw0;
    input tlp_dw0_t dw0;
    tlp_relaxed_ordering_dw0 = dw0.attr[1];
  endfunction

  // ---------------------------------------------------------------------------
  // calc_dlcrc — 16-bit CRC for DLLP validation
  //   Polynomial: CRC-16/CMS  x^16 + x^15 + x^2 + 1  (0x8005)
  //   Initial value: 16'hFFFF (DLCRC_INIT)
  //   Input: first 32 bits of DLLP (type + seq/reserved fields)
  //   Returns: 16-bit CRC result (no final inversion per PCIe DLL CRC rules)
  // ---------------------------------------------------------------------------
  function automatic logic [15:0] calc_dlcrc;
    input logic [31:0] data;
    logic [15:0] crc;
    integer i;
    begin
      crc = DLCRC_INIT;
      for (i = 31; i >= 0; i--) begin
        if (crc[15] ^ data[i])
          crc = (crc << 1) ^ 16'h8005;
        else
          crc = crc << 1;
      end
      calc_dlcrc = crc;
    end
  endfunction

  // Encode Maximum Read Request Size
  function automatic logic [2:0] encode_mrrs;
    input logic [12:0] size;
    case (size)
      13'd128:  encode_mrrs = 3'b000;
      13'd256:  encode_mrrs = 3'b001;
      13'd512:  encode_mrrs = 3'b010;
      13'd1024: encode_mrrs = 3'b011;
      13'd2048: encode_mrrs = 3'b100;
      13'd4096: encode_mrrs = 3'b101;
      default:  encode_mrrs = 3'b010;  // Default 512B
    endcase
  endfunction

  // ---------------------------------------------------------------------------
  // mps_to_bytes — decode MPS field (3-bit encoding) to byte count (13-bit)
  //   Same table as encode_mrrs but inverted direction.
  // ---------------------------------------------------------------------------
  function automatic logic [12:0] mps_to_bytes;
    input logic [2:0] mps;
    case (mps)
      3'b000:  mps_to_bytes = 13'd128;
      3'b001:  mps_to_bytes = 13'd256;
      3'b010:  mps_to_bytes = 13'd512;
      3'b011:  mps_to_bytes = 13'd1024;
      3'b100:  mps_to_bytes = 13'd2048;
      3'b101:  mps_to_bytes = 13'd4096;
      default: mps_to_bytes = 13'd128;  // Reserved → minimum
    endcase
  endfunction

  // ---------------------------------------------------------------------------
  // is_infinite_hdr_credit — returns 1 when the header credit field encodes
  //   infinite credits (all-ones, per PCIe Base Spec Section 2.11.1)
  // ---------------------------------------------------------------------------
  function automatic logic is_infinite_hdr_credit;
    input logic [11:0] hdr;
    is_infinite_hdr_credit = (hdr == 12'hFFF);
  endfunction

  // ---------------------------------------------------------------------------
  // is_infinite_data_credit — returns 1 when the data credit field encodes
  //   infinite credits (all-ones, per PCIe Base Spec Section 2.11.1)
  // ---------------------------------------------------------------------------
  function automatic logic is_infinite_data_credit;
    input logic [19:0] dat;
    is_infinite_data_credit = (dat == 20'hFFFFF);
  endfunction

endpackage : pcie_pkg

`endif // PCIE_PKG_SV
