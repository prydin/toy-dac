onbreak {quit -f}
onerror {quit -f}

vsim -voptargs="+acc"  -L xil_defaultlib -L xpm -L xbip_utils_v3_0_14 -L axi_utils_v2_0_10 -L mult_gen_v12_0_22 -L xbip_dsp48_wrapper_v3_0_6 -L xbip_pipe_v3_0_10 -L floating_point_v7_1_19 -L dds_compiler_v6_0_26 -L unisims_ver -L unimacro_ver -L secureip -lib xil_defaultlib xil_defaultlib.dds_compiler_0 xil_defaultlib.glbl

set NumericStdNoWarnings 1
set StdArithNoWarnings 1

do {wave.do}

view wave
view structure
view signals

do {dds_compiler_0.udo}

run 1000ns

quit -force
