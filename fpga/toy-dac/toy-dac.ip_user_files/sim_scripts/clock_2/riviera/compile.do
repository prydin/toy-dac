transcript off
onbreak {quit -force}
onerror {quit -force}
transcript on

vlib work
vlib riviera/xpm
vlib riviera/xil_defaultlib

vmap xpm riviera/xpm
vmap xil_defaultlib riviera/xil_defaultlib

vlog -work xpm  -incr "+incdir+c:/proj/toy-dac/fpga/toy-dac/toy-dac.gen/sources_1/ip/clock_2" "+incdir+../../../../toy-dac.gen/sources_1/ip/clock_2" -l xpm -l xil_defaultlib \
"C:/Xilinx/Vivado/2024.2/data/ip/xpm/xpm_cdc/hdl/xpm_cdc.sv" \

vcom -work xpm -93  -incr \
"C:/Xilinx/Vivado/2024.2/data/ip/xpm/xpm_VCOMP.vhd" \

vlog -work xil_defaultlib  -incr -v2k5 "+incdir+c:/proj/toy-dac/fpga/toy-dac/toy-dac.gen/sources_1/ip/clock_2" "+incdir+../../../../toy-dac.gen/sources_1/ip/clock_2" -l xpm -l xil_defaultlib \
"c:/proj/toy-dac/fpga/toy-dac/toy-dac.gen/sources_1/ip/clock_2/clock_clk_wiz.v" \
"c:/proj/toy-dac/fpga/toy-dac/toy-dac.gen/sources_1/ip/clock_2/clock.v" \

vlog -work xil_defaultlib \
"glbl.v"

