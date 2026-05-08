//lpm_mult CBX_DECLARE_ALL_CONNECTED_PORTS="OFF" DEDICATED_MULTIPLIER_CIRCUITRY="YES" DEVICE_FAMILY="Cyclone V" DSP_BLOCK_BALANCING="Auto" LPM_REPRESENTATION="SIGNED" LPM_WIDTHA=8 LPM_WIDTHB=8 LPM_WIDTHP=16 MAXIMIZE_SPEED=9 dataa datab result CARRY_CHAIN="MANUAL" CARRY_CHAIN_LENGTH=48
//VERSION_BEGIN 25.1 cbx_cycloneii 2025:10:22:10:31:18:SC cbx_lpm_add_sub 2025:10:22:10:31:18:SC cbx_lpm_mult 2025:10:22:10:31:18:SC cbx_mgl 2025:10:22:10:38:57:SC cbx_nadder 2025:10:22:10:31:18:SC cbx_padd 2025:10:22:10:31:18:SC cbx_stratix 2025:10:22:10:31:18:SC cbx_stratixii 2025:10:22:10:31:18:SC cbx_util_mgl 2025:10:22:10:31:18:SC  VERSION_END
// synthesis VERILOG_INPUT_VERSION VERILOG_2001
// altera message_off 10463



// Copyright (C) 2025  Altera Corporation. All rights reserved.
//  Your use of Altera Corporation's design tools, logic functions 
//  and other software and tools, and any partner logic 
//  functions, and any output files from any of the foregoing 
//  (including device programming or simulation files), and any 
//  associated documentation or information are expressly subject 
//  to the terms and conditions of the Altera Program License 
//  Subscription Agreement, the Altera Quartus Prime License Agreement,
//  the Altera IP License Agreement, or other applicable license
//  agreement, including, without limitation, that your use is for
//  the sole purpose of programming logic devices manufactured by
//  Altera and sold by Altera or its authorized distributors.  Please
//  refer to the Altera Software License Subscription Agreements 
//  on the Quartus Prime software download page.



//synthesis_resources = 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  mult_rhq
	( 
	dataa,
	datab,
	result) /* synthesis synthesis_clearbox=1 */;
	input   [7:0]  dataa;
	input   [7:0]  datab;
	output   [15:0]  result;

	wire signed	[7:0]    dataa_wire;
	wire signed	[7:0]    datab_wire;
	wire signed	[15:0]    result_wire;



	assign dataa_wire = dataa;
	assign datab_wire = datab;
	assign result_wire = dataa_wire * datab_wire;
	assign result = ({result_wire[15:0]});

endmodule //mult_rhq
//VALID FILE
