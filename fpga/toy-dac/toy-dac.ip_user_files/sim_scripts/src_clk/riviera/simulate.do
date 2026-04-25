transcript off
onbreak {quit -force}
onerror {quit -force}
transcript on

asim +access +r +m+src_clk  -L xil_defaultlib -L xpm -L unisims_ver -L unimacro_ver -L secureip -O5 xil_defaultlib.src_clk xil_defaultlib.glbl

do {src_clk.udo}

run 1000ns

endsim

quit -force
