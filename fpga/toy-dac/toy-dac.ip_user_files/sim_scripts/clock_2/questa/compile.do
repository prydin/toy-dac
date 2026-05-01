vlib questa_lib/work
vlib questa_lib/msim

vlib questa_lib/msim/xpm
vlib questa_lib/msim/xil_defaultlib

vmap xpm questa_lib/msim/xpm
vmap xil_defaultlib questa_lib/msim/xil_defaultlib

vlog -work xpm  -incr -mfcu  -sv "+incdir+c:/proj/toy-dac/fpga/toy-dac/toy-dac.gen/sources_1/ip/clock_2" "+incdir+../../../../toy-dac.gen/sources_1/ip/clock_2" \
"C:/Xilinx/Vivado/2024.2/data/ip/xpm/xpm_cdc/hdl/xpm_cdc.sv" \

vcom -work xpm  -93  \
"C:/Xilinx/Vivado/2024.2/data/ip/xpm/xpm_VCOMP.vhd" \

vlog -work xil_defaultlib  -incr -mfcu  "+incdir+c:/proj/toy-dac/fpga/toy-dac/toy-dac.gen/sources_1/ip/clock_2" "+incdir+../../../../toy-dac.gen/sources_1/ip/clock_2" \
"c:/proj/toy-dac/fpga/toy-dac/toy-dac.gen/sources_1/ip/clock_2/clock_clk_wiz.v" \
"c:/proj/toy-dac/fpga/toy-dac/toy-dac.gen/sources_1/ip/clock_2/clock.v" \

vlog -work xil_defaultlib \
"glbl.v"

