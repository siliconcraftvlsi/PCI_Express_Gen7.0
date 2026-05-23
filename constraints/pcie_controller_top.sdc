# =============================================================================
# PCIe 7.0 Controller — Synthesis Timing Constraints
# =============================================================================
# Target configuration: x4 Gen7 Endpoint, 250 MHz core clock
# Review and adjust all constants for your target technology and operating corner.
# These are starter constraints, NOT signoff constraints.
# =============================================================================

# ---------------------------------------------------------------------------
# Primary clocks
# ---------------------------------------------------------------------------
# core_clk: application/logic clock (250 MHz)
create_clock -name core_clk -period 4.000 -waveform {0 2.0} [get_ports core_clk]

# pipe_clk: PIPE reference clock from PHY
#   Gen1/2: 250 MHz (4 ns), Gen3: 250 MHz, Gen4: 250 MHz,
#   Gen5: 500 MHz (2 ns), Gen6/7: 500–1000 MHz
#   Constrain to maximum supported (Gen7 PIPE at 128b/130b, 32-bit width ≈ 500 MHz)
create_clock -name pipe_clk -period 2.000 -waveform {0 1.0} [get_ports pipe_clk]

# aux_clk: always-on auxiliary clock for power management (100 MHz for synthesis;
#   real SoC integration may use 32 kHz — adjust period accordingly)
create_clock -name aux_clk  -period 10.000 -waveform {0 5.0} [get_ports aux_clk]

# ---------------------------------------------------------------------------
# Generated clocks (if any internal clock dividers are added)
# ---------------------------------------------------------------------------
# Example: derive_pll_clocks -create_generated_clocks

# ---------------------------------------------------------------------------
# Clock groups: all three domains are asynchronous to each other
# ---------------------------------------------------------------------------
set_clock_groups -asynchronous \
  -group [get_clocks core_clk] \
  -group [get_clocks pipe_clk] \
  -group [get_clocks aux_clk]

# ---------------------------------------------------------------------------
# Clock uncertainty (jitter + skew estimates)
# ---------------------------------------------------------------------------
set_clock_uncertainty -setup 0.100 [get_clocks core_clk]
set_clock_uncertainty -hold  0.050 [get_clocks core_clk]
set_clock_uncertainty -setup 0.080 [get_clocks pipe_clk]
set_clock_uncertainty -hold  0.040 [get_clocks pipe_clk]
set_clock_uncertainty -setup 0.200 [get_clocks aux_clk]
set_clock_uncertainty -hold  0.100 [get_clocks aux_clk]

# ---------------------------------------------------------------------------
# Input / output delays
# ---------------------------------------------------------------------------
# PIPE RX inputs arrive from PHY — model as half a pipe_clk period
set_input_delay  -clock pipe_clk -max 0.500 [get_ports pipe_rx_*]
set_input_delay  -clock pipe_clk -min 0.100 [get_ports pipe_rx_*]
set_input_delay  -clock pipe_clk -max 0.200 [get_ports pipe_clk_req_n]

# AXI inputs arrive from SoC interconnect — model relative to core_clk
set_input_delay  -clock core_clk -max 1.000 [get_ports s_axi_*]
set_input_delay  -clock core_clk -min 0.200 [get_ports s_axi_*]
set_input_delay  -clock core_clk -max 1.000 [get_ports m_axi_*]
set_input_delay  -clock core_clk -min 0.200 [get_ports m_axi_*]

# DMA / interrupt / status inputs
set_input_delay  -clock core_clk -max 0.800 [get_ports {dma_start dma_src_addr dma_dst_addr dma_length dma_dir}]
set_input_delay  -clock core_clk -max 0.800 [get_ports intx_assert]

# PIPE TX outputs to PHY
set_output_delay -clock pipe_clk -max 0.500 [get_ports pipe_tx_*]
set_output_delay -clock pipe_clk -min 0.100 [get_ports pipe_tx_*]
set_output_delay -clock pipe_clk -max 0.500 [get_ports {pipe_power_down pipe_reset_n pipe_rate pipe_width}]

# AXI outputs
set_output_delay -clock core_clk -max 1.000 [get_ports {s_axi_awready s_axi_wready s_axi_bid s_axi_bresp s_axi_bvalid}]
set_output_delay -clock core_clk -max 1.000 [get_ports {s_axi_arready s_axi_rid s_axi_rdata s_axi_rresp s_axi_rlast s_axi_rvalid}]
set_output_delay -clock core_clk -max 1.000 [get_ports m_axi_*]

# Interrupt and status outputs
set_output_delay -clock core_clk -max 1.000 \
  [get_ports {msi_irq msi_vector msix_irq msix_vector link_up dma_done dma_error}]
set_output_delay -clock core_clk -max 1.000 \
  [get_ports {cfg_err_cor cfg_err_nonfatal cfg_err_fatal negotiated_gen negotiated_width}]

# ---------------------------------------------------------------------------
# False paths
# ---------------------------------------------------------------------------
# Async reset input: assert is asynchronous by design
set_false_path -from [get_ports core_rst_n]

# LTSSM state output is purely for debug/status; relax timing
set_false_path -to [get_ports ltssm_state]
set_false_path -to [get_ports max_payload_size]
set_false_path -to [get_ports max_read_req_size]

# CDC crossing paths (handled by synchronizers — set as false paths through sync cells)
# Replace pcie_sync2 with your foundry sync cell name if different
set_false_path -through [get_cells -hierarchical -filter {REF_NAME =~ pcie_sync2*}]
set_false_path -through [get_cells -hierarchical -filter {REF_NAME =~ pcie_rst_sync*}]

# Async FIFO pointer crossings — constrained by Gray-code guarantee
set_max_delay -datapath_only 2.0 \
  -from [get_cells -hierarchical -filter {NAME =~ *pcie_async_fifo*wr_ptr_gray*}] \
  -to   [get_cells -hierarchical -filter {NAME =~ *pcie_async_fifo*sync_ff*}]
set_max_delay -datapath_only 2.0 \
  -from [get_cells -hierarchical -filter {NAME =~ *pcie_async_fifo*rd_ptr_gray*}] \
  -to   [get_cells -hierarchical -filter {NAME =~ *pcie_async_fifo*sync_ff*}]

# ---------------------------------------------------------------------------
# Operating conditions and drive/load models
# ---------------------------------------------------------------------------
# set_operating_conditions -max WCCOM -library <your_lib>
set_drive        0 [all_inputs]
set_load         0.050 [all_outputs]   ;# 50 fF placeholder; adjust for target process

# ---------------------------------------------------------------------------
# Disable timing on unused configurations (e.g. unused lanes when x4)
# ---------------------------------------------------------------------------
# set_false_path -from [get_ports pipe_rx_data[4]*]  ;# lanes 4-15 when NUM_LANES=4
