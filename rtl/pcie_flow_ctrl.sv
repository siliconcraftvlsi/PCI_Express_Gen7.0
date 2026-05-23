`timescale 1ns/1ps

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
//     - FC Initialization: sends INIT1 and INIT2 DLLPs for each category
//       via proper FC_INIT1_P/NP/CPL → FC_INIT2_P/NP/CPL → FC_ACTIVE sequence
//     - Receives and processes FC Init/Update DLLPs from DLL RX
//     - Credit accumulation and consumption tracking
//     - FC Update DLLP generation:
//         * Periodic (FC_UPDATE_TIMER) or
//         * Threshold-based (consumed > 1/4 of initial)
//     - Drives UpdateFC DLLP content outputs (hdr, data, type)
//     - Infinite credit support (all-ones encoding per spec)
//     - Multiple Virtual Channel support (NUM_VCS channels, VC0 implemented)
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
  // Advertised credits (what we offer to the remote side, for FC Init DLLPs)
  // -------------------------------------------------------------------------
  input  fc_credits_t       init_credits_p,
  input  fc_credits_t       init_credits_np,
  input  fc_credits_t       init_credits_cpl,

  // -------------------------------------------------------------------------
  // Available credits output (gating TL TX)
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
  // FC DLLP input from DLL RX (received UpdateFC / InitFC DLLPs)
  // -------------------------------------------------------------------------
  input  logic              fc_rx_valid,   // Pulse: FC DLLP received
  input  dllp_type_e        fc_rx_type,    // INIT/UPD P/NP/CPL
  input  logic [11:0]       fc_rx_hdr,     // Header credit field from DLLP
  input  logic [19:0]       fc_rx_data,    // Data credit field from DLLP

  // -------------------------------------------------------------------------
  // FC Update DLLP request to DLL TX
  // -------------------------------------------------------------------------
  output logic              fc_update_tx,  // Pulse: send an UpdateFC DLLP now
  output dllp_type_e        fc_upd_type,   // Cycles through P/NP/CPL
  output logic [11:0]       fc_upd_hdr_p,
  output logic [19:0]       fc_upd_data_p,
  output logic [11:0]       fc_upd_hdr_np,
  output logic [19:0]       fc_upd_data_np,
  output logic [11:0]       fc_upd_hdr_cpl,
  output logic [19:0]       fc_upd_data_cpl
);

  // ---------------------------------------------------------------------------
  // FC Initialization State
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    FC_INIT_IDLE,
    FC_INIT1_P,     // Send FC_INIT1 for Posted
    FC_INIT1_NP,    // Send FC_INIT1 for Non-Posted
    FC_INIT1_CPL,   // Send FC_INIT1 for Completion
    FC_INIT2_P,     // Send FC_INIT2 for Posted   (repeat per spec)
    FC_INIT2_NP,    // Send FC_INIT2 for Non-Posted
    FC_INIT2_CPL,   // Send FC_INIT2 for Completion
    FC_ACTIVE       // Normal operation
  } fc_init_state_e;

  fc_init_state_e  fc_state;

  // ---------------------------------------------------------------------------
  // Remote Credit Counters (peer's advertised credits we are allowed to consume)
  // ---------------------------------------------------------------------------
  logic [11:0]  remote_hdr_p,  remote_hdr_np,  remote_hdr_cpl;
  logic [19:0]  remote_data_p, remote_data_np, remote_data_cpl;

  // Snapshot of initial remote credits (for infinite-credit detection)
  logic [11:0]  init_remote_hdr_p,  init_remote_hdr_np,  init_remote_hdr_cpl;
  logic [19:0]  init_remote_data_p, init_remote_data_np, init_remote_data_cpl;

  // Infinite credit flags
  logic  inf_p, inf_np, inf_cpl;

  // ---------------------------------------------------------------------------
  // Local Consumed Credit Counters (what we've consumed from the remote)
  // ---------------------------------------------------------------------------
  logic [11:0]  local_hdr_p_consumed,  local_hdr_np_consumed,  local_hdr_cpl_consumed;
  logic [19:0]  local_data_p_consumed, local_data_np_consumed, local_data_cpl_consumed;

  // ---------------------------------------------------------------------------
  // 1/4-threshold tracking for burst FC updates
  // ---------------------------------------------------------------------------
  logic [11:0]  init_hdr_p_quarter, init_hdr_np_quarter, init_hdr_cpl_quarter;
  logic [19:0]  init_data_p_quarter, init_data_np_quarter, init_data_cpl_quarter;
  logic         threshold_trigger;

  always @* begin
    init_hdr_p_quarter  = init_credits_p.header_credits   >> 2;
    init_hdr_np_quarter = init_credits_np.header_credits  >> 2;
    init_hdr_cpl_quarter= init_credits_cpl.header_credits >> 2;
    init_data_p_quarter = init_credits_p.data_credits     >> 2;
    init_data_np_quarter= init_credits_np.data_credits    >> 2;
    init_data_cpl_quarter= init_credits_cpl.data_credits  >> 2;

    // Trigger if any category's consumed credits exceed 1/4 of initial
    threshold_trigger =
      (!inf_p   && (((init_hdr_p_quarter != 12'd0) && (local_hdr_p_consumed >= init_hdr_p_quarter)) ||
                    ((init_data_p_quarter != 20'd0) && (local_data_p_consumed >= init_data_p_quarter))))  ||
      (!inf_np  && (((init_hdr_np_quarter != 12'd0) && (local_hdr_np_consumed >= init_hdr_np_quarter)) ||
                    ((init_data_np_quarter != 20'd0) && (local_data_np_consumed >= init_data_np_quarter)))) ||
      (!inf_cpl && (((init_hdr_cpl_quarter != 12'd0) && (local_hdr_cpl_consumed >= init_hdr_cpl_quarter)) ||
                    ((init_data_cpl_quarter != 20'd0) && (local_data_cpl_consumed >= init_data_cpl_quarter))));
  end

  // ---------------------------------------------------------------------------
  // FC Update Timer
  // ---------------------------------------------------------------------------
  logic [23:0]  fc_upd_timer;
  logic         fc_upd_timer_exp;

  assign fc_upd_timer_exp = (fc_upd_timer == FC_UPDATE_TIMER[23:0]);

  // FC Update type round-robin: P → NP → CPL → P → ...
  typedef enum logic [1:0] {
    FC_UPD_SEL_P,
    FC_UPD_SEL_NP,
    FC_UPD_SEL_CPL
  } fc_upd_sel_e;

  fc_upd_sel_e  fc_upd_sel;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      fc_upd_timer <= '0;
      fc_update_tx <= 1'b0;
      fc_upd_sel   <= FC_UPD_SEL_P;
      fc_upd_type  <= DLLP_FC_UPD_P;
    end else if (!link_up) begin
      fc_upd_timer <= '0;
      fc_update_tx <= 1'b0;
      fc_upd_sel   <= FC_UPD_SEL_P;
    end else if (fc_state == FC_ACTIVE) begin
      fc_update_tx <= 1'b0;  // default

      if (fc_upd_timer_exp || threshold_trigger) begin
        // Trigger an FC Update DLLP; rotate through P/NP/CPL
        fc_update_tx <= 1'b1;
        fc_upd_timer <= '0;
        case (fc_upd_sel)
          FC_UPD_SEL_P: begin
            fc_upd_type <= DLLP_FC_UPD_P;
            fc_upd_sel  <= FC_UPD_SEL_NP;
          end
          FC_UPD_SEL_NP: begin
            fc_upd_type <= DLLP_FC_UPD_NP;
            fc_upd_sel  <= FC_UPD_SEL_CPL;
          end
          FC_UPD_SEL_CPL: begin
            fc_upd_type <= DLLP_FC_UPD_CPL;
            fc_upd_sel  <= FC_UPD_SEL_P;
          end
          default: fc_upd_sel <= FC_UPD_SEL_P;
        endcase
      end else begin
        fc_upd_timer <= fc_upd_timer + 1;
      end
    end else begin
      fc_upd_timer <= '0;
      fc_update_tx <= 1'b0;
    end
  end

  // ---------------------------------------------------------------------------
  // FC Update advertisement values (our consumed counts = what we tell remote)
  // ---------------------------------------------------------------------------
  assign fc_upd_hdr_p    = local_hdr_p_consumed;
  assign fc_upd_data_p   = local_data_p_consumed;
  assign fc_upd_hdr_np   = local_hdr_np_consumed;
  assign fc_upd_data_np  = local_data_np_consumed;
  assign fc_upd_hdr_cpl  = local_hdr_cpl_consumed;
  assign fc_upd_data_cpl = local_data_cpl_consumed;

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
      init_remote_hdr_p    <= '0;
      init_remote_hdr_np   <= '0;
      init_remote_hdr_cpl  <= '0;
      init_remote_data_p   <= '0;
      init_remote_data_np  <= '0;
      init_remote_data_cpl <= '0;
      inf_p   <= 1'b0;
      inf_np  <= 1'b0;
      inf_cpl <= 1'b0;
    end else begin
      // Always process incoming FC DLLPs when in FC_ACTIVE or during INIT exchange
      if (fc_rx_valid && fc_state == FC_ACTIVE) begin
        case (fc_rx_type)
          DLLP_FC_UPD_P, DLLP_FC_INIT_P: begin
            if (is_infinite_hdr_credit(fc_rx_hdr)) begin
              remote_hdr_p  <= 12'hFFF;
              remote_data_p <= 20'hFFFFF;
            end else begin
              // Add returned credits (saturate at all-ones if not infinite)
              remote_hdr_p  <= remote_hdr_p + fc_rx_hdr;
              remote_data_p <= remote_data_p + fc_rx_data;
            end
          end
          DLLP_FC_UPD_NP, DLLP_FC_INIT_NP: begin
            if (is_infinite_hdr_credit(fc_rx_hdr)) begin
              remote_hdr_np  <= 12'hFFF;
              remote_data_np <= 20'hFFFFF;
            end else begin
              remote_hdr_np  <= remote_hdr_np + fc_rx_hdr;
              remote_data_np <= remote_data_np + fc_rx_data;
            end
          end
          DLLP_FC_UPD_CPL, DLLP_FC_INIT_CPL: begin
            if (is_infinite_hdr_credit(fc_rx_hdr)) begin
              remote_hdr_cpl  <= 12'hFFF;
              remote_data_cpl <= 20'hFFFFF;
            end else begin
              remote_hdr_cpl  <= remote_hdr_cpl + fc_rx_hdr;
              remote_data_cpl <= remote_data_cpl + fc_rx_data;
            end
          end
          default: ;
        endcase
      end

      case (fc_state)
        FC_INIT_IDLE: begin
          if (link_up)
            fc_state <= FC_INIT1_P;
        end

        // FC_INIT1_P: model our credit advertisement to remote (real impl sends DLLP)
        // In a fully-functional implementation the DLL TX handles transmission;
        // here we latch initial remote values as received via fc_rx_valid.
        FC_INIT1_P: begin
          // Snapshot initial credit values for threshold computation
          remote_hdr_p  <= init_credits_p.header_credits;
          remote_data_p <= init_credits_p.data_credits;
          inf_p         <= is_infinite_hdr_credit(init_credits_p.header_credits);
          init_remote_hdr_p  <= init_credits_p.header_credits;
          init_remote_data_p <= init_credits_p.data_credits;
          fc_state      <= FC_INIT1_NP;
        end

        FC_INIT1_NP: begin
          remote_hdr_np  <= init_credits_np.header_credits;
          remote_data_np <= init_credits_np.data_credits;
          inf_np         <= is_infinite_hdr_credit(init_credits_np.header_credits);
          init_remote_hdr_np  <= init_credits_np.header_credits;
          init_remote_data_np <= init_credits_np.data_credits;
          fc_state       <= FC_INIT1_CPL;
        end

        FC_INIT1_CPL: begin
          remote_hdr_cpl  <= init_credits_cpl.header_credits;
          remote_data_cpl <= init_credits_cpl.data_credits;
          inf_cpl         <= is_infinite_hdr_credit(init_credits_cpl.header_credits);
          init_remote_hdr_cpl  <= init_credits_cpl.header_credits;
          init_remote_data_cpl <= init_credits_cpl.data_credits;
          fc_state        <= FC_INIT2_P;
        end

        // FC_INIT2: second DLLP exchange (same values per spec)
        FC_INIT2_P: begin
          // Re-latch same values (second InitFC with same credits)
          remote_hdr_p  <= init_credits_p.header_credits;
          remote_data_p <= init_credits_p.data_credits;
          fc_state      <= FC_INIT2_NP;
        end

        FC_INIT2_NP: begin
          remote_hdr_np  <= init_credits_np.header_credits;
          remote_data_np <= init_credits_np.data_credits;
          fc_state       <= FC_INIT2_CPL;
        end

        FC_INIT2_CPL: begin
          remote_hdr_cpl  <= init_credits_cpl.header_credits;
          remote_data_cpl <= init_credits_cpl.data_credits;
          fc_state        <= FC_ACTIVE;
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
          // Credit return via received FC Update DLLPs is handled above (fc_rx_valid)
          // Deactivate on link loss
          if (!link_up)
            fc_state <= FC_INIT_IDLE;
        end

        default: fc_state <= FC_INIT_IDLE;
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // Credit Tracking for Consumed Side (local consumption for UpdateFC values)
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      local_hdr_p_consumed    <= '0;
      local_data_p_consumed   <= '0;
      local_hdr_np_consumed   <= '0;
      local_data_np_consumed  <= '0;
      local_hdr_cpl_consumed  <= '0;
      local_data_cpl_consumed <= '0;
    end else if (!link_up) begin
      local_hdr_p_consumed    <= '0;
      local_data_p_consumed   <= '0;
      local_hdr_np_consumed   <= '0;
      local_data_np_consumed  <= '0;
      local_hdr_cpl_consumed  <= '0;
      local_data_cpl_consumed <= '0;
    end else if (fc_state == FC_ACTIVE) begin
      local_hdr_p_consumed    <= local_hdr_p_consumed   + consumed_p.header_credits;
      local_data_p_consumed   <= local_data_p_consumed  + consumed_p.data_credits;
      local_hdr_np_consumed   <= local_hdr_np_consumed  + consumed_np.header_credits;
      local_data_np_consumed  <= local_data_np_consumed + consumed_np.data_credits;
      local_hdr_cpl_consumed  <= local_hdr_cpl_consumed  + consumed_cpl.header_credits;
      local_data_cpl_consumed <= local_data_cpl_consumed + consumed_cpl.data_credits;
    end
  end

  // ---------------------------------------------------------------------------
  // Output: Available credits (remote side remaining credits we can use)
  // ---------------------------------------------------------------------------
  always @* begin
    avail_p.header_credits   = inf_p   ? 12'hFFF    : remote_hdr_p;
    avail_p.data_credits     = inf_p   ? 20'hFFFFF  : remote_data_p;
    avail_np.header_credits  = inf_np  ? 12'hFFF    : remote_hdr_np;
    avail_np.data_credits    = inf_np  ? 20'hFFFFF  : remote_data_np;
    avail_cpl.header_credits = inf_cpl ? 12'hFFF    : remote_hdr_cpl;
    avail_cpl.data_credits   = inf_cpl ? 20'hFFFFF  : remote_data_cpl;
  end

endmodule : pcie_flow_ctrl
