// UVM Testbench filelist — PCIe 7.0 Controller
// Include after rtl.f and before top-level simulation command
// Order: interfaces → packages → agents → coverage → top

// Interfaces (must precede packages that reference them)
uvm_tb/pcie_uvm_if.sv
uvm_tb/pcie_pipe_if.sv

// UVM packages
uvm_tb/pcie_uvm_pkg.sv
uvm_tb/pcie_pipe_agent_pkg.sv
uvm_tb/pcie_ltssm_cov.sv
uvm_tb/pcie_error_inject_seq.sv

// PIPE partner (RC-side behavioral model)
uvm_tb/pcie_pipe_partner.sv

// SVA assertion modules + bind wrapper (pcie_controller_sva_wrapper)
sva/pcie_ltssm_assertions.sv
sva/pcie_dll_assertions.sv
sva/pcie_fc_assertions.sv
sva/pcie_tlp_assertions.sv
sva/pcie_delivery_assertions.sv
uvm_tb/pcie_sva_bind.sv

// Simulation top
uvm_tb/pcie_uvm_top.sv
