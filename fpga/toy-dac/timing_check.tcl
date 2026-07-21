read_verilog -sv [glob src/rtl/*.v]
read_xdc src/constr/root.xdc
synth_design -top top -part xc7s25csga225-1
report_timing_summary -file timing_check_summary.rpt
report_exceptions -file timing_check_exceptions.rpt