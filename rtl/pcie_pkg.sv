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
    ERR_COR  = 3'b000,  // Correctable Error
    ERR_NONFATAL = 3'b001, // Non-fatal Uncorrectable
    ERR_FATAL    = 3'b010, // Fatal Uncorrectable
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
  // Utility Functions
  // ---------------------------------------------------------------------------

  // Calculate number of DWs from byte count
  function automatic logic [9:0] bytes_to_dw;
    input logic [11:0] bytes;
    return (bytes + 3) >> 2;
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

endpackage : pcie_pkg

`endif // PCIE_PKG_SV
