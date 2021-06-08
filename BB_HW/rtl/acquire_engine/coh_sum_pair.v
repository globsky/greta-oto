//----------------------------------------------------------------------
// coh_sum_pair.v:
//   do one frequency pair of coherent summation of acc value and correlation value 
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module coh_sum_pair
(
// system signals				
input clk,	//system clock
input rst_b,	//reset signal, low active

// input correlation result
input signed [15:0] cor_input_i,
input signed [15:0] cor_input_q,
input cor_input_valid,

// twiddle factors, align with input correlation result
input sign_cos,
input [8:0] mag_cos,
input sign_sin,
input [8:0] mag_sin,

// coherent acc input and output
input [47:0] coh_data_in,
output [47:0] coh_data_out,

// interface to control the behavior of coherent sum
input first_segment,	// do not add previous coherent result, equivalent to SegmentCount == 0
input first_acc			// first time accumulation, do not multiply with twiddle factor, equivalent to CohCount == 0
);

// calculate one frequency pair of coherent accumulation with following stages:
// stage 1: multiply the input correlation data with twiddle factor
// stage 2: add shifted multiply result together to get result of complex multiply
// stage 3: convert complex result to exp10 format, this stage will also latch coherent RAM read data
// stage 4: output added data to coh_data_out
// upper level module need to latch coh_data_out 4 clock cycle delay after asserting cor_input_valid
// for first_acc asserted, no twiddle factor applied, positive and negative results are the same
// so upper level module will clear twiddle factor to prevent multiply logic toggles
// input of conversion to exp10 and adding for negative frequency will all clear to 0

//----------------------------------------------------------
// generate signal for each stage of calculation
//----------------------------------------------------------
wire do_multiply;
reg do_shift, do_convert;
assign do_multiply = cor_input_valid;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		do_shift <= 1'b0;
	else
		do_shift <= do_multiply;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		do_convert <= 1'b0;
	else
		do_convert <= do_shift;

//----------------------------------------------------------
// calulate result of cor result with twiddle factor
//----------------------------------------------------------
reg signed [24:0] cos_i, cos_q, sin_i, sin_q;

wire signed [9:0] mag_cos_ext, mag_sin_ext;
assign mag_cos_ext = {1'b0, mag_cos};
assign mag_sin_ext = {1'b0, mag_sin};

// calculate multiply of (sign extened) sin/cos with I/Q
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cos_i <= 'd0;
	else if (do_multiply)
		cos_i <= cor_input_i * mag_cos_ext;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cos_q <= 'd0;
	else if (do_multiply)
		cos_q <= cor_input_q * mag_cos_ext;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		sin_i <= 'd0;
	else if (do_multiply)
		sin_i <= cor_input_i * mag_sin_ext;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		sin_q <= 'd0;
	else if (do_multiply)
		sin_q <= cor_input_q * mag_sin_ext;

// round shift
wire signed [15:0] round_ci, round_cq, round_si, round_sq;	// result of round shift

round_shift #(.DATA_WIDTH(22), .SHIFT_BITS(6)) round_shift_u1
(
	.data_input  (cos_i[24:3]),
	.data_output (round_ci   )
);

round_shift #(.DATA_WIDTH(22), .SHIFT_BITS(6)) round_shift_u2
(
	.data_input  (cos_q[24:3]),
	.data_output (round_cq   )
);

round_shift #(.DATA_WIDTH(22), .SHIFT_BITS(6)) round_shift_u3
(
	.data_input  (sin_i[24:3]),
	.data_output (round_si   )
);

round_shift #(.DATA_WIDTH(22), .SHIFT_BITS(6)) round_shift_u4
(
	.data_input  (sin_q[24:3]),
	.data_output (round_sq   )
);

wire [15:0] neg_ci, neg_cq, neg_si, neg_sq;
assign neg_ci = -round_ci;
assign neg_cq = -round_cq;
assign neg_si = -round_si;
assign neg_sq = -round_sq;

// one clock cycle delay to input I/Q to skip multiply when CohCount == 0
reg signed [15:0] cor_input_i_d;
reg signed [15:0] cor_input_q_d;
reg [1:0] first_segment_d, first_acc_d;
reg sign_cos_d, sign_sin_d;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cor_input_i_d <= 'd0;
	else
		cor_input_i_d <= cor_input_i;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cor_input_q_d <= 'd0;
	else
		cor_input_q_d <= cor_input_q;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		first_segment_d <= 2'b00;
	else
		first_segment_d <= {first_segment_d[0], first_segment};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		first_acc_d <= 2'b00;
	else
		first_acc_d <= {first_acc_d[0], first_acc};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		sign_cos_d <= 1'b0;
	else
		sign_cos_d <= sign_cos;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		sign_sin_d <= 1'b0;
	else
		sign_sin_d <= sign_sin;

// assign multiply result
reg signed [15:0] pos_i, pos_q, neg_i, neg_q;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		pos_i <= 'd0;
	else if (do_shift)
		pos_i <= first_acc ? cor_input_i_d : ((sign_cos_d ? neg_ci : round_ci) + (sign_sin_d ? round_sq : neg_sq));

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		pos_q <= 'd0;
	else if (do_shift)
		pos_q <= first_acc ? cor_input_q_d : ((sign_cos_d ? neg_cq : round_cq) + (sign_sin_d ? neg_si : round_si));

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		neg_i <= 'd0;
	else if (do_shift)
		neg_i <= first_acc ? 16'd0 : ((sign_cos_d ? neg_ci : round_ci) + (sign_sin_d ? neg_sq : round_sq));		// do not calculate negative frequency result on first acc

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		neg_q <= 'd0;
	else if (do_shift)
		neg_q <= first_acc ? 16'd0 : ((sign_cos_d ? neg_cq : round_cq) + (sign_sin_d ? round_si : neg_si));		// do not calculate negative frequency result on first acc

//--------------------------------------------
// convert to exp10 format
//--------------------------------------------
wire [9:0] pos_i10, pos_q10, neg_i10, neg_q10;	// result positive and negative multiply result
wire [3:0] pos_exp, neg_exp;
reg [23:0] pos_freq_result, neg_freq_result;

complex2exp10 u1_complex2exp10
(
	.input_i    (pos_i  ),
	.input_q    (pos_q  ),
	.output_i   (pos_i10),
	.output_q   (pos_q10),
	.output_exp (pos_exp)
);

complex2exp10 u2_complex2exp10
(
	.input_i    (neg_i  ),
	.input_q    (neg_q  ),
	.output_i   (neg_i10),
	.output_q   (neg_q10),
	.output_exp (neg_exp)
);

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		pos_freq_result <= 'd0;
	else if (do_convert)
		pos_freq_result <= {pos_i10, pos_q10, pos_exp};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		neg_freq_result <= 'd0;
	else if (do_convert)
		neg_freq_result <= {neg_i10, neg_q10, neg_exp};

//--------------------------------------------
// do coherent accumulation on two exp10 data
//--------------------------------------------
reg [23:0] coh_pos_input, coh_neg_input;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coh_pos_input <= 'd0;
	else if (do_convert)
		coh_pos_input <= coh_data_in[47:24];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coh_neg_input <= 'd0;
	else if (do_convert)
		coh_neg_input <= first_acc_d[0] ? 24'd0 : coh_data_in[23:0];		// do not calculate negative frequency result on first acc

// do coherent sum and latch into corresponding data out register
wire [9:0] acc_pos_i, acc_pos_q, acc_neg_i, acc_neg_q;
wire [3:0] acc_pos_exp, acc_neg_exp;

complex_exp_add u1_complex_exp_add
(
	.in1_i   (coh_pos_input[23:14]  ),
	.in1_q   (coh_pos_input[13:4]   ),
	.in1_exp (coh_pos_input[3:0]    ),
	.in2_i   (pos_freq_result[23:14]),
	.in2_q   (pos_freq_result[13:4] ),
	.in2_exp (pos_freq_result[3:0]  ),
	.out_i   (acc_pos_i             ),
	.out_q   (acc_pos_q             ),
	.out_exp (acc_pos_exp           )
);

complex_exp_add u2_complex_exp_add
(
	.in1_i   (coh_neg_input[23:14]  ),
	.in1_q   (coh_neg_input[13:4]   ),
	.in1_exp (coh_neg_input[3:0]    ),
	.in2_i   (neg_freq_result[23:14]),
	.in2_q   (neg_freq_result[13:4] ),
	.in2_exp (neg_freq_result[3:0]  ),
	.out_i   (acc_neg_i             ),
	.out_q   (acc_neg_q             ),
	.out_exp (acc_neg_exp           )
);

wire [23:0] acc_pos, acc_neg;
assign acc_pos = {acc_pos_i, acc_pos_q, acc_pos_exp};
assign acc_neg = {acc_neg_i, acc_neg_q, acc_neg_exp};

assign coh_data_out = (first_segment_d[1] && first_acc_d[1]) ? {pos_freq_result, pos_freq_result} : {acc_pos, (first_acc_d[1] ? acc_pos : acc_neg)};

endmodule
