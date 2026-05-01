// -----------------------------------------------------------------------------
// Author  : Robert Kingsly Amalathas
// Email   : robertk@microprocessorlab.com
// LinkedIn: www.linkedin.com/in/robertkingslya
// -----------------------------------------------------------------------------
// =============================================================================
// PCIe 7.0 Controller - Flow Control Manager
// Based on PCI Express Base Specification Rev 7.0 Section 2.11
// =============================================================================
// Description:
//   Manages PCIe credit-based flow control for all three TLP categories:
//     - Posted (P): Memory Writes, Messages
//     - Non-Posted (NP): Memory Reads, IO Rd/Wr, Config Rd/Wr, AtomicOps
//     - Completions (CPL)
//
//   Functions:
//     - Flow Control Initialization (FC_INIT1/FC_INIT2 exchange via DLLPs)
//     - Credit accumulation and consumption tracking
//     - FC Update DLLP generation (periodic and on threshold)
//     - Header credit and Data credit management (12-bit and 19-bit fields)
//     - Support for infinite credits (all-ones encoding)
//     - Multiple Virtual Channel support (NUM_VCS channels)
// =============================================================================

`include "pcie_pkg.sv"

module pcie_flow_ctrl
  import pcie_pkg::*;
#(
  parameter int unsigned DATA_W  = 256,
  parameter int unsigned NUM_VCS = 8
)(
  input  logic              clk,
  input  logic              rst_n,
  input  logic              link_up,

  // -------------------------------------------------------------------------
  // Advertised credits (what we offer to the remote side)
  // -------------------------------------------------------------------------
  input  fc_credits_t       init_credits_p,
  input  fc_credits_t       init_credits_np,
  input  fc_credits_t       init_credits_cpl,

  // -------------------------------------------------------------------------
  // Available credits (what the remote side offers us; used to gate TX)
  // -------------------------------------------------------------------------
  output fc_credits_t       avail_p,
  output fc_credits_t       avail_np,
  output fc_credits_t       avail_cpl,

  // -------------------------------------------------------------------------
  // Consumed credits (decremented by TL TX when a TLP is sent)
  // -------------------------------------------------------------------------
  input  fc_credits_t       consumed_p,
  input  fc_credits_t       consumed_np,
  input  fc_credits_t       consumed_cpl,

  // -------------------------------------------------------------------------
  // FC Update DLLP request to DLL TX
  // -------------------------------------------------------------------------
  output logic              fc_update_tx
);

  // ---------------------------------------------------------------------------
  // FC Initialization State
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    FC_INIT_IDLE,
    FC_INIT1_P,    // Send FC_INIT1 for Posted
    FC_INIT1_NP,   // Send FC_INIT1 for Non-Posted
    FC_INIT1_CPL,  // Send FC_INIT1 for Completion
    FC_INIT2_P,    // Send FC_INIT2 for Posted
    FC_INIT2_NP,   // Send FC_INIT2 for Non-Posted
    FC_INIT2_CPL,  // Send FC_INIT2 for Completion
    FC_ACTIVE      // Normal operation
  } fc_init_state_e;

  fc_init_state_e  fc_state;

  // ---------------------------------------------------------------------------
  // Credit Counters (per VC, simplified to VC0)
  // RemoteHdrFC[P/NP/CPL] and RemoteDataFC[P/NP/CPL] track remote credits
  // LocalHdrFC[P/NP/CPL] and LocalDataFC[P/NP/CPL] track consumed credits
  // ---------------------------------------------------------------------------
  // Remote (offered by peer, what we can send)
  logic [11:0]  remote_hdr_p,  remote_hdr_np,  remote_hdr_cpl;
  logic [19:0]  remote_data_p, remote_data_np, remote_data_cpl;

  // Our consumed credits (for UpdateFC computation)
  logic [11:0]  local_hdr_p_consumed,  local_hdr_np_consumed,  local_hdr_cpl_consumed;
  logic [19:0]  local_data_p_consumed, local_data_np_consumed, local_data_cpl_consumed;

  // Infinite credit flag
  logic  inf_p, inf_np, inf_cpl;

  // ---------------------------------------------------------------------------
  // FC Update Timer
  // ---------------------------------------------------------------------------
  logic [23:0]  fc_upd_timer;
  logic         fc_upd_timer_exp;

  assign fc_upd_timer_exp = (fc_upd_timer == FC_UPDATE_TIMER[23:0]);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      fc_upd_timer <= '0;
      fc_update_tx <= 1'b0;
    end else if (!link_up) begin
      fc_upd_timer <= '0;
      fc_update_tx <= 1'b0;
    end else begin
      if (fc_upd_timer_exp) begin
        fc_update_tx <= 1'b1;
        fc_upd_timer <= '0;
      end else begin
        fc_update_tx <= 1'b0;
        fc_upd_timer <= fc_upd_timer + 1;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Initialization FSM
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      fc_state        <= FC_INIT_IDLE;
      remote_hdr_p    <= '0;
      remote_hdr_np   <= '0;
      remote_hdr_cpl  <= '0;
      remote_data_p   <= '0;
      remote_data_np  <= '0;
      remote_data_cpl <= '0;
      inf_p   <= 1'b0;
      inf_np  <= 1'b0;
      inf_cpl <= 1'b0;
    end else begin
      case (fc_state)
        FC_INIT_IDLE: begin
          if (link_up)
            fc_state <= FC_INIT1_P;
        end

        FC_INIT1_P: begin
          // Advertise our Posted credits to remote
          // (Real impl would send DLLP; here we model instant exchange)
          remote_hdr_p  <= init_credits_p.header_credits;
          remote_data_p <= init_credits_p.data_credits;
          inf_p         <= (init_credits_p.header_credits == 12'hFFF);
          fc_state      <= FC_INIT1_NP;
        end

        FC_INIT1_NP: begin
          remote_hdr_np  <= init_credits_np.header_credits;
          remote_data_np <= init_credits_np.data_credits;
          inf_np         <= (init_credits_np.header_credits == 12'hFFF);
          fc_state       <= FC_INIT1_CPL;
        end

        FC_INIT1_CPL: begin
          remote_hdr_cpl  <= init_credits_cpl.header_credits;
          remote_data_cpl <= init_credits_cpl.data_credits;
          inf_cpl         <= (init_credits_cpl.header_credits == 12'hFFF);
          fc_state        <= FC_INIT2_P;
        end

        FC_INIT2_P, FC_INIT2_NP, FC_INIT2_CPL: begin
          // Second FC_INIT exchange: same values (simplified)
          fc_state <= (fc_state == FC_INIT2_CPL) ? FC_ACTIVE : fc_init_state_e'(fc_state + 1);
        end

        FC_ACTIVE: begin
          // Credit consumption from TL TX
          if (!inf_p) begin
            remote_hdr_p  <= remote_hdr_p  - consumed_p.header_credits;
            remote_data_p <= remote_data_p - consumed_p.data_credits;
          end
          if (!inf_np) begin
            remote_hdr_np  <= remote_hdr_np  - consumed_np.header_credits;
            remote_data_np <= remote_data_np - consumed_np.data_credits;
          end
          if (!inf_cpl) begin
            remote_hdr_cpl  <= remote_hdr_cpl  - consumed_cpl.header_credits;
            remote_data_cpl <= remote_data_cpl - consumed_cpl.data_credits;
          end
          // Credit return: FC Update DLLPs received would add credits back
          // (simplified: restore on FC update trigger)
          if (fc_upd_timer_exp) begin
            remote_hdr_p  <= remote_hdr_p  + 12'd4;
            remote_hdr_np <= remote_hdr_np + 12'd4;
            remote_data_p <= remote_data_p + 20'd16;
          end
        end

        default: fc_state <= FC_INIT_IDLE;
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // Credit Tracking for Consumed Side
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      local_hdr_p_consumed   <= '0;
      local_data_p_consumed  <= '0;
      local_hdr_np_consumed  <= '0;
      local_data_np_consumed <= '0;
      local_hdr_cpl_consumed  <= '0;
      local_data_cpl_consumed <= '0;
    end else if (fc_state == FC_ACTIVE) begin
      local_hdr_p_consumed   <= local_hdr_p_consumed   + consumed_p.header_credits;
      local_data_p_consumed  <= local_data_p_consumed  + consumed_p.data_credits;
      local_hdr_np_consumed  <= local_hdr_np_consumed  + consumed_np.header_credits;
      local_data_np_consumed <= local_data_np_consumed + consumed_np.data_credits;
      local_hdr_cpl_consumed  <= local_hdr_cpl_consumed  + consumed_cpl.header_credits;
      local_data_cpl_consumed <= local_data_cpl_consumed + consumed_cpl.data_credits;
    end
  end

  // ---------------------------------------------------------------------------
  // Output: Available credits (remote side remaining credits we can use)
  // ---------------------------------------------------------------------------
  always_comb begin
    avail_p.header_credits  = inf_p   ? 12'hFFF : remote_hdr_p;
    avail_p.data_credits    = inf_p   ? 20'hFFFFF : remote_data_p;
    avail_np.header_credits = inf_np  ? 12'hFFF : remote_hdr_np;
    avail_np.data_credits   = inf_np  ? 20'hFFFFF : remote_data_np;
    avail_cpl.header_credits = inf_cpl ? 12'hFFF : remote_hdr_cpl;
    avail_cpl.data_credits   = inf_cpl ? 20'hFFFFF : remote_data_cpl;
  end

endmodule : pcie_flow_ctrl
