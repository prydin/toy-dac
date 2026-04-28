// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2024.2 (win64) Build 5239630 Fri Nov 08 22:35:27 MST 2024
// Date        : Sat Apr 25 08:09:11 2026
// Host        : da_beast running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode synth_stub -rename_top clock -prefix
//               clock_ clock_stub.v
// Design      : clock
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7s25csga225-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* CORE_GENERATION_INFO = "clock,clk_wiz_v6_0_15_0_0,{component_name=clock,use_phase_alignment=true,use_min_o_jitter=false,use_max_i_jitter=false,use_dyn_phase_shift=true,use_inclk_switchover=false,use_dyn_reconfig=false,enable_axi=0,feedback_source=FDBK_AUTO,PRIMITIVE=MMCM,num_out_clk=1,clkin1_period=83.333,clkin2_period=10.0,use_power_down=false,use_reset=true,use_locked=true,use_inclk_stopped=false,feedback_type=SINGLE,CLOCK_MGR_TYPE=NA,manual_override=false}" *) 
module clock(mclk, psclk, psen, psincdec, psdone, reset, locked, 
  clk_in1)
/* synthesis syn_black_box black_box_pad_pin="psen,psincdec,psdone,reset,locked" */
/* synthesis syn_force_seq_prim="mclk" */
/* synthesis syn_force_seq_prim="psclk" */
/* synthesis syn_force_seq_prim="clk_in1" */;
  output mclk /* synthesis syn_isclock = 1 */;
  input psclk /* synthesis syn_isclock = 1 */;
  input psen;
  input psincdec;
  output psdone;
  input reset;
  output locked;
  input clk_in1 /* synthesis syn_isclock = 1 */;
endmodule
