//----------------------------------------------------------------------
// overflow_gen.v:
//   code NCO overflow signal generation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module overflow_gen
(
// system signals				
input	wire		clk,		//system clock, 80Mhz
input	wire		rst_b,	//reset signal, low active
// internal signals
input data_down_en,
input [31:0] code_freq,
input code_phase_en,
input [31:0] code_phase_i,
output reg [31:0] code_phase_o,
input fill_finished,
input jump_count_en,
input	[7:0]	jump_count_i,	//input jump count
output reg [7:0] jump_count_o,	//output jump count
// code NCO overflow signal
output overflow
);
//code nco
wire [32:0] code_nco;
wire        code_overflow;
assign      code_nco = {1'b0,code_phase_o} + {1'b0,code_freq};
assign      code_overflow = code_nco[32];

always @ (posedge clk or negedge rst_b)
	if (~rst_b)
		code_phase_o <= 32'h0;
	else if (code_phase_en)
		code_phase_o <= code_phase_i;
	else if (data_down_en)
		code_phase_o <= code_nco[31:0];

//jump count processing
reg [7:0] jump_count_r;
wire jump_cnt_pos;
wire jump_cnt_neg;

assign jump_cnt_pos = (~jump_count_o[7])&(|jump_count_o[6:0]);
assign jump_cnt_neg = jump_count_o[7];

always @ (posedge clk or negedge rst_b)
	if (~rst_b)
		jump_count_r <= 8'h0;
	else if (jump_count_en)
		jump_count_r <= jump_count_i;
	    
always @ (posedge clk or negedge rst_b)
	if (~rst_b)
		jump_count_o <= 8'h0;
	else if (fill_finished)
		jump_count_o <= jump_count_r;
	else if (jump_cnt_pos)
		jump_count_o <= jump_count_o - 1'b1;
	else if (jump_cnt_neg & code_overflow & data_down_en)
		jump_count_o <= jump_count_o + 1'b1;
    	
assign overflow = jump_cnt_pos | (code_overflow & ~jump_cnt_neg & data_down_en);

endmodule
