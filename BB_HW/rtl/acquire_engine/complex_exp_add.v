//----------------------------------------------------------------------
// complex_exp_add.v:
//   add two exp10 format complex number
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module complex_exp_add
(
input [9:0] in1_i,
input [9:0] in1_q,
input [3:0] in1_exp,
input [9:0] in2_i,
input [9:0] in2_q,
input [3:0] in2_exp,
output [9:0] out_i,
output [9:0] out_q,
output [3:0] out_exp
);

//--------------------------------------------
// find the maximum exp and calculate difference
//--------------------------------------------
wire [4:0] exp_diff;
assign exp_diff = {1'b0, in1_exp} - {1'b0, in2_exp};

wire [3:0] max_exp;
assign max_exp = exp_diff[4] ? in2_exp : in1_exp;

//--------------------------------------------
// shift input to the same exponential value
//--------------------------------------------
reg [9:0] in1_i_shift, in1_q_shift;
reg [9:0] in2_i_shift, in2_q_shift;

// shift in1_i according to exp_diff
always @(*)
	casex(exp_diff)
	5'b0????: in1_i_shift = in1_i;	// do not shift
	5'b11111: in1_i_shift = {in1_i[9], in1_i[9:1]};
	5'b11110: in1_i_shift = {{2{in1_i[9]}}, in1_i[9:2]};
	5'b11101: in1_i_shift = {{3{in1_i[9]}}, in1_i[9:3]};
	5'b11100: in1_i_shift = {{4{in1_i[9]}}, in1_i[9:4]};
	5'b11011: in1_i_shift = {{5{in1_i[9]}}, in1_i[9:5]};
	5'b11010: in1_i_shift = {{6{in1_i[9]}}, in1_i[9:6]};
	5'b11001: in1_i_shift = {{7{in1_i[9]}}, in1_i[9:7]};
	5'b11000: in1_i_shift = {{8{in1_i[9]}}, in1_i[9:8]};
	5'b10111: in1_i_shift = {10{in1_i[9]}};
	default: in1_i_shift = in1_i;
	endcase

// shift in1_q according to exp_diff
always @(*)
	casex(exp_diff)
	5'b0????: in1_q_shift = in1_q;
	5'b11111: in1_q_shift = {in1_q[9], in1_q[9:1]};
	5'b11110: in1_q_shift = {{2{in1_q[9]}}, in1_q[9:2]};
	5'b11101: in1_q_shift = {{3{in1_q[9]}}, in1_q[9:3]};
	5'b11100: in1_q_shift = {{4{in1_q[9]}}, in1_q[9:4]};
	5'b11011: in1_q_shift = {{5{in1_q[9]}}, in1_q[9:5]};
	5'b11010: in1_q_shift = {{6{in1_q[9]}}, in1_q[9:6]};
	5'b11001: in1_q_shift = {{7{in1_q[9]}}, in1_q[9:7]};
	5'b11000: in1_q_shift = {{8{in1_q[9]}}, in1_q[9:8]};
	5'b10111: in1_q_shift = {10{in1_q[9]}};
	default: in1_q_shift = in1_q;
	endcase

// shift in2_i according to exp_diff
always @(*)
	casex(exp_diff)
	5'b1????: in2_i_shift = in2_i;
	5'b00001: in2_i_shift = {in2_i[9], in2_i[9:1]};
	5'b00010: in2_i_shift = {{2{in2_i[9]}}, in2_i[9:2]};
	5'b00011: in2_i_shift = {{3{in2_i[9]}}, in2_i[9:3]};
	5'b00100: in2_i_shift = {{4{in2_i[9]}}, in2_i[9:4]};
	5'b00101: in2_i_shift = {{5{in2_i[9]}}, in2_i[9:5]};
	5'b00110: in2_i_shift = {{6{in2_i[9]}}, in2_i[9:6]};
	5'b00111: in2_i_shift = {{7{in2_i[9]}}, in2_i[9:7]};
	5'b01000: in2_i_shift = {{8{in2_i[9]}}, in2_i[9:8]};
	5'b01001: in2_i_shift = {10{in2_i[9]}};
	default: in2_i_shift = in2_i;
	endcase

// shift in2_q according to exp_diff
always @(*)
	casex(exp_diff)
	5'b1????: in2_q_shift = in2_q;
	5'b00001: in2_q_shift = {in2_q[9], in2_q[9:1]};
	5'b00010: in2_q_shift = {{2{in2_q[9]}}, in2_q[9:2]};
	5'b00011: in2_q_shift = {{3{in2_q[9]}}, in2_q[9:3]};
	5'b00100: in2_q_shift = {{4{in2_q[9]}}, in2_q[9:4]};
	5'b00101: in2_q_shift = {{5{in2_q[9]}}, in2_q[9:5]};
	5'b00110: in2_q_shift = {{6{in2_q[9]}}, in2_q[9:6]};
	5'b00111: in2_q_shift = {{7{in2_q[9]}}, in2_q[9:7]};
	5'b01000: in2_q_shift = {{8{in2_q[9]}}, in2_q[9:8]};
	5'b01001: in2_q_shift = {10{in2_q[9]}};
	default: in2_q_shift = in2_q;
	endcase

//--------------------------------------------
// add shifted result together
//--------------------------------------------
wire [10:0] sum_i, sum_q;
assign sum_i = {in1_i_shift[9], in1_i_shift} + {in2_i_shift[9], in2_i_shift};
assign sum_q = {in1_q_shift[9], in1_q_shift} + {in2_q_shift[9], in2_q_shift};

wire overflow;
assign overflow = (sum_i[10] ^ sum_i[9]) | (sum_q[10] ^ sum_q[9]);

assign out_exp = overflow ? (max_exp + 1) : max_exp;
assign out_i = overflow ? sum_i[10:1] : sum_i[9:0];
assign out_q = overflow ? sum_q[10:1] : sum_q[9:0];

endmodule
