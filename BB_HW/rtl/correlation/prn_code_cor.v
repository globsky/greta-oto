//----------------------------------------------------------------------
// prn_code_cor.v:
//   Generate PRN code bits used for correlator
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module prn_code_cor
(
// system signals
input	clk,
input	rst_b,
// control signal
input enable_boc,
input enable_2nd_prn,
input [1:0] narrow_factor,
input code_sub_phase,
input [1:0] code_phase,
input overflow,
// PRN input from PRN generator and NH generator
input prn_code1,
input prn_code2,
input nh_code1,
input nh_code2,
// variables
input prn_code_load_en,  // load enable
input [7:0]  prn_code_i, 		// prn code load value
output [7:0]  prn_code_o, 	// prn code output
input corr_state_load_en, // load enable
input [3:0]  prn_code2_i, 		// prn code2 load value
output [3:0] prn_code2_o,				// prn code2 output
// output PRN bits
output [7:0] prn_bits
);

wire prn_code1_in, prn_code2_in;
assign prn_code1_in = prn_code1 ^ (enable_boc & code_sub_phase) ^ (nh_code1);
assign prn_code2_in = prn_code2 ^ (enable_boc & code_sub_phase) ^ (nh_code2);

reg [6:0] prn_code_r;
reg [3:0] prn_code2_r;

always @ (posedge clk or negedge rst_b)
	if (~rst_b)
		prn_code_r <= 7'h0;
	else if (prn_code_load_en)
		prn_code_r <= prn_code_i[7:1];
	else if (overflow)
		prn_code_r <= {prn_code_r[5:0], prn_code1_in};

always @ (posedge clk or negedge rst_b)
	if (~rst_b)
		prn_code2_r <= 4'h0;
	else if (corr_state_load_en)
		prn_code2_r <= prn_code2_i;
	else if (overflow & enable_2nd_prn)
		prn_code2_r <= {prn_code2_r[2:0], prn_code2_in};

wire advance4, lag4;
wire advance8, lag8;
assign advance4 = code_phase[1];
assign lag4 = ~code_phase[1];
assign advance8 = (code_phase == 2'b11) ? 1 : 0;
assign lag8 = (code_phase == 2'b00) ? 1 : 0;

wire advance_bit, lag_bit, prompt_bit;
assign advance_bit = prn_code_r[2];
assign lag_bit = prn_code_r[4];
assign prompt_bit = prn_code_r[3];

wire prn_code_cor0;
reg prn_code_cor2, prn_code_cor3, prn_code_cor5, prn_code_cor6;

// select PRN bit for cor0
assign prn_code_cor0 = enable_2nd_prn ? prn_code2_r[3] : prn_code1_in;

// select PRN bit for cor2
always @(*)
	case (narrow_factor)
		2'b00: prn_code_cor2 = prn_code_r[1];
		2'b01: prn_code_cor2 = advance_bit;
		2'b10: prn_code_cor2 = advance4 ? advance_bit : prompt_bit;
		default: prn_code_cor2 = prn_code_r[1];
	endcase

// select PRN bit for cor3
always @(*)
	case (narrow_factor)
		2'b00: prn_code_cor3 = prn_code_r[2];
		2'b01: prn_code_cor3 = advance4 ? advance_bit : prompt_bit;
		2'b10: prn_code_cor3 = advance8 ? advance_bit : prompt_bit;
		default: prn_code_cor3 = prn_code_r[1];
	endcase

// select PRN bit for cor5
always @(*)
	case (narrow_factor)
		2'b00: prn_code_cor5 = prn_code_r[4];
		2'b01: prn_code_cor5 = lag4 ? lag_bit : prompt_bit;
		2'b10: prn_code_cor5 = lag8 ? lag_bit : prompt_bit;
		default: prn_code_cor5 = prn_code_r[4];
	endcase

// select PRN bit for cor6
always @(*)
	case (narrow_factor)
		2'b00: prn_code_cor6 = prn_code_r[5];
		2'b01: prn_code_cor6 = lag_bit;
		2'b10: prn_code_cor6 = lag4 ? lag_bit : prompt_bit;
		default: prn_code_cor6 = prn_code_r[5];
	endcase

assign prn_code_o = {prn_code_r, prn_code1_in};
assign prn_code2_o = prn_code2_r;
assign prn_bits = {prn_code_r[6], prn_code_cor6, prn_code_cor5, prn_code_r[3], prn_code_cor3, prn_code_cor2, prn_code_r[0], prn_code_cor0};

endmodule
