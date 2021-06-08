//----------------------------------------------------------------------
// peak_sorter.v:
//   peak sorter to get 3 maximum peaks
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module peak_sorter
(
// system signals				
input clk,	//system clock
input rst_b,	//reset signal, low active

// control interface
input clear,

// non-coherent result input
input [7:0] input_amp,
input [3:0] input_exp,
input [14:0] code_pos,
input [8:0] freq_pos,
input	peak_valid,

// output of the peak sorter
output reg [7:0] peak1_amp,
output reg [7:0] peak2_amp,
output reg [7:0] peak3_amp,
output reg [8:0] peak1_freq,
output reg [8:0] peak2_freq,
output reg [8:0] peak3_freq,
output reg [14:0] peak1_cor,
output reg [14:0] peak2_cor,
output reg [14:0] peak3_cor,
output reg [3:0] peak_exp
);

//----------------------------------------------------------
// compare input exp with peak exp and adjust input amplitude
//----------------------------------------------------------
wire input_exp_larger;
wire [3:0] shift_bits;
reg [7:0] amp_shift;
assign input_exp_larger = (peak_exp >= input_exp) ? 1'b0 : 1'b1;
assign shift_bits = input_exp_larger ? 4'd0 : (peak_exp - input_exp);

always@(*)
	case(shift_bits)
		4'd0: amp_shift = input_amp;
		4'd1: amp_shift = {1'b0,input_amp[7:1]};
		4'd2: amp_shift = {2'b0,input_amp[7:2]};
		4'd3: amp_shift = {3'b0,input_amp[7:3]};
		4'd4: amp_shift = {4'b0,input_amp[7:4]};
		4'd5: amp_shift = {5'b0,input_amp[7:5]};
		4'd6: amp_shift = {6'b0,input_amp[7:6]};
		4'd7: amp_shift = {7'b0,input_amp[7]};
		default: amp_shift = 8'b0;
	endcase

// latch shifted amplitude and cor/freq position for update use
reg [7:0] amp_shift_r;
reg [14:0] code_pos_r;
reg [8:0] freq_pos_r;
reg index_valid;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		amp_shift_r <= 'd0;
	else if (peak_valid)
		amp_shift_r <= amp_shift;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		code_pos_r <= 'd0;
	else if (peak_valid)
		code_pos_r <= code_pos;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		freq_pos_r <= 'd0;
	else if (peak_valid)
		freq_pos_r <= freq_pos;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		index_valid <= 1'b0;
	else
		index_valid <= peak_valid;

// latch new exponential
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		peak_exp <= 'd0;
	else if (clear)
		peak_exp <= 'd0;
	else if (peak_valid && input_exp_larger)
		peak_exp <= input_exp;

//----------------------------------------------------------
// compare input amplitude with peaks and do match check
//----------------------------------------------------------
// assume input exp will only larger than peak exp by 1 except for first data
// so if input exp larger than peak exp, only shift peak amp by 1 bit
wire [7:0] peak1_compare, peak2_compare, peak3_compare;
reg [7:0] peak1_next, peak2_next, peak3_next;
assign peak1_compare = input_exp_larger ? {1'b0, peak1_amp[7:1]} : peak1_amp;
assign peak2_compare = input_exp_larger ? {1'b0, peak2_amp[7:1]} : peak2_amp;
assign peak3_compare = input_exp_larger ? {1'b0, peak3_amp[7:1]} : peak3_amp;

always @(posedge clk or negedge rst_b)
	if (!rst_b) begin
		peak1_next <= 'd0;
		peak2_next <= 'd0;
		peak3_next <= 'd0;
	end
	else if (peak_valid) begin
		peak1_next <= peak1_compare;
		peak2_next <= peak2_compare;
		peak3_next <= peak3_compare;
	end

// value compare result
wire [2:0] peak_larger;
reg [1:0] value_index;
assign peak_larger[0] = (amp_shift > peak1_compare) ? 1'b1 : 1'b0;
assign peak_larger[1] = (amp_shift > peak2_compare) ? 1'b1 : 1'b0;
assign peak_larger[2] = (amp_shift > peak3_compare) ? 1'b1 : 1'b0;
// map 000 100 110 111 to 00 01 10 11
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		value_index <= 2'b00;
	else if (peak_valid)
		value_index <= {peak_larger[1], peak_larger[0] ^ peak_larger[1] ^ peak_larger[2]};

// match check on code position and frequency
wire [8:0] freq_diff1, freq_diff2, freq_diff3;
wire [2:0] match_freq;
assign freq_diff1 = freq_pos - peak1_freq;
assign freq_diff2 = freq_pos - peak2_freq;
assign freq_diff3 = freq_pos - peak3_freq;
assign match_freq[0] = (&freq_diff1 || ~(|freq_diff1[8:1]));	// == -1/0/1
assign match_freq[1] = (&freq_diff2 || ~(|freq_diff2[8:1]));	// == -1/0/1
assign match_freq[2] = (&freq_diff3 || ~(|freq_diff3[8:1]));	// == -1/0/1

wire [14:0] cor_diff1, cor_diff2, cor_diff3;
wire [2:0] match_cor;
assign cor_diff1 = code_pos - peak1_cor;
assign cor_diff2 = code_pos - peak2_cor;
assign cor_diff3 = code_pos - peak3_cor;
assign match_cor[0] = (&cor_diff1 || ~(|cor_diff1[14:1]));	// == -1/0/1
assign match_cor[1] = (&cor_diff2 || ~(|cor_diff2[14:1]));	// == -1/0/1
assign match_cor[2] = (&cor_diff3 || ~(|cor_diff3[14:1]));	// == -1/0/1

reg [2:0] match_freq_r, match_cor_r;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		match_freq_r <= 3'b000;
	else if (peak_valid)
		match_freq_r <= match_freq;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		match_cor_r <= 3'b000;
	else if (peak_valid)
		match_cor_r <= match_cor;

wire [2:0] match_freq_cor;
wire [1:0] match_index;
assign match_freq_cor = match_freq_r & match_cor_r;
// map 000 XX1 X10 100 to 00 01 11 10
assign match_index[1] = (match_freq_cor[1:0] == 2'b10) || ((match_freq_cor[2] == 1) && (match_freq_cor[0] == 0));
assign match_index[0] = match_freq_cor[1] || match_freq_cor[0];

//----------------------------------------------------------
// determine action bit for peaks and update peaks
//----------------------------------------------------------
wire update_peak1, update_peak2, update_peak3;
wire replace_new2, replace_new3;

assign update_peak1 = index_valid && (value_index == 2'b11);
assign update_peak2 = index_valid && ((value_index[1] && ~match_index[0]) || (value_index[1] && match_index[1]));
assign update_peak3 = index_valid && (value_index != 2'b00) && (~match_index[0]);
assign replace_new2 = (value_index == 2'b10);
assign replace_new3 = ~value_index[1];

// peak1 update
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		peak1_amp <= 'd0;
	else if (clear)
		peak1_amp <= 'd0;
	else if (update_peak1)
		peak1_amp <= amp_shift_r;
	else if (index_valid)
		peak1_amp <= peak1_next;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		peak1_cor <= 'd0;
	else if (clear)
		peak1_cor <= 'd0;
	else if (update_peak1)
		peak1_cor <= code_pos_r;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		peak1_freq <= 'd0;
	else if (clear)
		peak1_freq <= 'd0;
	else if (update_peak1)
		peak1_freq <= freq_pos_r;

// peak2 update
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		peak2_amp <= 'd0;
	else if (clear)
		peak2_amp <= 'd0;
	else if (update_peak2)
		peak2_amp <= replace_new2 ? amp_shift_r : peak1_next;
	else if (index_valid)
		peak2_amp <= peak2_next;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		peak2_cor <= 'd0;
	else if (clear)
		peak2_cor <= 'd0;
	else if (update_peak2)
		peak2_cor <= replace_new2 ? code_pos_r : peak1_cor;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		peak2_freq <= 'd0;
	else if (clear)
		peak2_freq <= 'd0;
	else if (update_peak2)
		peak2_freq <= replace_new2 ? freq_pos_r : peak1_freq;

// peak3 update
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		peak3_amp <= 'd0;
	else if (clear)
		peak3_amp <= 'd0;
	else if (update_peak3)
		peak3_amp <= replace_new3 ? amp_shift_r : peak2_next;
	else if (index_valid)
		peak3_amp <= peak3_next;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		peak3_cor <= 'd0;
	else if (clear)
		peak3_cor <= 'd0;
	else if (update_peak3)
		peak3_cor <= replace_new3 ? code_pos_r : peak2_cor;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		peak3_freq <= 'd0;
	else if (clear)
		peak3_freq <= 'd0;
	else if (update_peak3)
		peak3_freq <= replace_new3 ? freq_pos_r : peak2_freq;

endmodule
