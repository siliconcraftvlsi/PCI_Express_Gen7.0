`timescale 1ns/1ps

// =============================================================================
// PCIe 7.0 Controller - Completion Timeout Engine
// PCIe Base Spec Rev 7.0 Section 2.8
// =============================================================================
// Tracks outstanding Non-Posted requests (Memory Read, IO, Config) by tag.
// Each tag has an independent countdown timer.  On expiry, the engine signals
// a completion timeout error and optionally cancels the tag.
//
// Timer ranges per spec (Table 2-5):
//   Range A:  50 µs – 10  ms
//   Range B:  10 ms – 250 ms
//   Range C: 250 ms – 4   s
//   Range D:   4  s – 64  s
// This implementation uses Range B default (50 ms at 250 MHz = 12.5M cycles).
// =============================================================================

`include "pcie_pkg.sv"

module pcie_cpl_timeout
  import pcie_pkg::*;
#(
  parameter int unsigned NUM_TAGS        = 256,
  parameter int unsigned TIMER_WIDTH     = 24,
  // Default timeout: 50 ms at 250 MHz
  parameter int unsigned DEFAULT_TIMEOUT = 24'd12_500_000
)(
  input  logic                      clk,
  input  logic                      rst_n,
  input  logic                      link_up,

  // Timeout range select (from config space Device Control register [3:0])
  input  logic [3:0]                cpl_timeout_range,

  // Tag allocation: assert alloc_en for one cycle with the new tag
  input  logic                      alloc_en,
  input  logic [$clog2(NUM_TAGS)-1:0] alloc_tag,

  // Tag completion: assert cpl_en when a completion arrives for a tag
  input  logic                      cpl_en,
  input  logic [$clog2(NUM_TAGS)-1:0] cpl_tag,

  // Timeout outputs
  output logic                      timeout_en,       // pulse: one tag timed out
  output logic [$clog2(NUM_TAGS)-1:0] timeout_tag,   // which tag timed out
  output logic                      cfg_err_cor,      // correctable error (timeout)
  output logic                      cfg_err_nonfatal  // non-fatal error (repeated)
);

  localparam int unsigned TAG_W = $clog2(NUM_TAGS);

  // ---------------------------------------------------------------------------
  // Timer value from cpl_timeout_range encoding (spec Table 2-5)
  // ---------------------------------------------------------------------------
  logic [TIMER_WIDTH-1:0] timeout_val;

  always @* begin
    case (cpl_timeout_range)
      4'b0001: timeout_val = TIMER_WIDTH'(50_000);        // Range A: 50 µs
      4'b0010: timeout_val = TIMER_WIDTH'(DEFAULT_TIMEOUT); // Range B (default)
      4'b0101: timeout_val = TIMER_WIDTH'(24'hFFFFFF);    // Range C (max representable)
      4'b0110: timeout_val = TIMER_WIDTH'(24'hFFFFFF);    // Range D (saturate)
      default: timeout_val = TIMER_WIDTH'(DEFAULT_TIMEOUT);
    endcase
  end

  // ---------------------------------------------------------------------------
  // Per-tag state: active flag + countdown timer
  // ---------------------------------------------------------------------------
  logic                  tag_active [0:NUM_TAGS-1];
  logic [TIMER_WIDTH-1:0] tag_timer  [0:NUM_TAGS-1];

  // ---------------------------------------------------------------------------
  // Round-robin scan pointer for checking expired timers
  // ---------------------------------------------------------------------------
  logic [TAG_W-1:0] scan_ptr;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      scan_ptr <= '0;
    else
      scan_ptr <= scan_ptr + 1;
  end

  // ---------------------------------------------------------------------------
  // Allocation / Completion / Timer update
  // ---------------------------------------------------------------------------
  genvar i;
  generate
    for (i = 0; i < NUM_TAGS; i++) begin : gen_tag
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          tag_active[i] <= 1'b0;
          tag_timer[i]  <= '0;
        end else begin
          if (alloc_en && (alloc_tag == TAG_W'(i))) begin
            tag_active[i] <= 1'b1;
            tag_timer[i]  <= timeout_val;
          end else if (cpl_en && (cpl_tag == TAG_W'(i))) begin
            tag_active[i] <= 1'b0;
            tag_timer[i]  <= '0;
          end else if (tag_active[i] && link_up) begin
            if (tag_timer[i] != '0)
              tag_timer[i] <= tag_timer[i] - 1;
          end
        end
      end
    end
  endgenerate

  // ---------------------------------------------------------------------------
  // Scan for expired timers (one per clock via scan_ptr)
  // ---------------------------------------------------------------------------
  logic expired;
  assign expired = tag_active[scan_ptr] && (tag_timer[scan_ptr] == '0) && link_up;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      timeout_en      <= 1'b0;
      timeout_tag     <= '0;
      cfg_err_cor     <= 1'b0;
      cfg_err_nonfatal<= 1'b0;
    end else begin
      timeout_en       <= expired;
      timeout_tag      <= expired ? scan_ptr : '0;
      cfg_err_cor      <= expired;      // first timeout: correctable
      cfg_err_nonfatal <= 1'b0;         // escalation logic can be added here
    end
  end

endmodule : pcie_cpl_timeout
