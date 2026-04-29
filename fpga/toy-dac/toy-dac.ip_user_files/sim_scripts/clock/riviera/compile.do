transcript off
onbreak {quit -force}
onerror {quit -force}
transcript on

vlib work
vlib riviera/xpm
vlib riviera/xil_defaultlib

vmap xpm riviera/xpm
vmap xil_defaultlib riviera/xil_defaultlib

vlog -work xpm  -incr "+incdir+../../../ipstatic" "+incdir+../../../../../toy-dac.gen/sources_1/ip/clock" -l xpm -l xil_defaultlib \
"C:/Xilinx/Vivado/2024.2/data/ip/xpm/xpm_cdc/hdl/xpm_cdc.sv" \

vcom -work xpm -93  -incr \
"C:/Xilinx/Vivado/2024.2/data/ip/xpm/xpm_VCOMP.vhd" \

vlog -work xil_defaultlib  -incr -v2k5 "+incdir+../../../ipstatic" "+incdir+../../../../../toy-dac.gen/sources_1/ip/clock" -l xpm -l xil_defaultlib \
"../../../../../toy-dac.gen/sources_1/ip/clock/clock_clk_wiz.v" \
"../../../../../toy-dac.gen/sources_1/ip/clock/clock.v" \

vlog -work xil_defaultlib \
"glbl.v"

