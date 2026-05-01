+incdir+./rtl
+incdir+./uvm_tb
+incdir+$UVM_HOME/src

$UVM_HOME/src/uvm_pkg.sv

./rtl/pcie_pkg.sv
./rtl/pcie_ltssm.sv
./rtl/pcie_pipe_if.sv
./rtl/pcie_dll_tx.sv
./rtl/pcie_dll_rx.sv
./rtl/pcie_flow_ctrl.sv
./rtl/pcie_tlp_tx.sv
./rtl/pcie_tlp_rx.sv
./rtl/pcie_cfg_space.sv
./rtl/pcie_axi_bridge.sv
./rtl/pcie_dma.sv
./rtl/pcie_controller_top.sv

./uvm_tb/pcie_uvm_if.sv
./uvm_tb/pcie_pipe_partner.sv
./uvm_tb/pcie_uvm_pkg.sv
./uvm_tb/pcie_uvm_top.sv
