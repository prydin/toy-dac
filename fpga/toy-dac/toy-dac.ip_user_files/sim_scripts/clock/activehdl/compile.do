transcript off
onbreak {quit -force}
onerror {quit -force}
transcript on

vlib work
vlib activehdl/xpm
vlib activehdl/xil_defaultlib

vmap xpm activehdl/xpm
vmap xil_defaultlib activehdl/xil_defaultlib

vlog -work xpm  -sv2k12 "+incdir+../../../ipstatic" "+incdir+../../../../../toy-dac.gen/sources_1/ip/clock" -l xpm -l xil_defaultlib \
"C:/Xilinx/Vivado/2024.2/data/ip/xpm/xpm_cdc/hdl/xpm_cdc.sv" \

vcom -work xpm -93  \
"C:/Xilinx/Vivado/2024.2/data/ip/xpm/xpm_VCOMP.vhd" \

vlog -work xil_defaultlib  -v2k5 "+incdir+../../../ipstatic" "+incdir+../../../../../toy-dac.gen/sources_1/ip/clock" -l xpm -l xil_defaultlib \
"../../../../../toy-dac.gen/sources_1/ip/clock/clock_clk_wiz.v" \
"../../../../../toy-dac.gen/sources_1/ip/clock/clock.v" \

vlog -work xil_defaultlib \
"glbl.v"

