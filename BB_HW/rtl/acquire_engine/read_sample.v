//----------------------------------------------------------------------
// read_sample.v:
//   read sample from AE buffer, do down conversion and add two together
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module read_sample
(
// system signals				
input clk,	//system clock
input rst_b,	//reset signal, low active

// interface of control signal
input clear,	// clear NCO and previous input valid flag
input [31:0] carrier_freq,	// carrier frequency
//input sync_in,				// signal to align input and output delay
//output reg sync_out,	// signal to align input and output delay

// interface to sample input from AE buffer
input [3:0] sample_in,
input sample_in_valid,

// interface of output sample
output [5:0] sample_out_i,
output [5:0] sample_out_q,
output reg sample_out_valid
);

reg sample_in_valid_d;
reg prev_input_valid;
reg [31:0] carrier_nco;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		sample_in_valid_d <= 1'b0;
	else if (clear)
		sample_in_valid_d <= 1'b0;
	else
		sample_in_valid_d <= sample_in_valid;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		prev_input_valid <= 1'b0;
	else if (clear)
		prev_input_valid <= 1'b0;
	else if (sample_in_valid)
		prev_input_valid <= 1'b1;

//--------------------------------------------
// sample_in latch
//--------------------------------------------
reg [3:0] sample_in_r;
reg [3:0] sample_in_r2;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		sample_in_r <= 'd0;
	else if (clear)
		sample_in_r <= 'd0;
	else if (sample_in_valid)
		sample_in_r <= sample_in;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		sample_in_r2 <= 'd0;
	else if (clear)
		sample_in_r2 <= 'd0;
	else
		sample_in_r2 <= sample_in_r;

//--------------------------------------------
// NCO accumulation and phase calculation
//--------------------------------------------
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		carrier_nco <= 'd0;
	else if (clear)
		carrier_nco <= 'd0;
	else if (sample_in_valid)
		carrier_nco <= carrier_nco + carrier_freq;

wire [5:0] phase;

assign phase = carrier_nco[31:26];

//--------------------------------------------
// sine/cosine LUT
//--------------------------------------------
wire [2:0] cos_mag;
wire [2:0] sin_mag;
wire cos_sign;
wire sin_sign;

sincos_lut_64x4 u_sincos_lut_64x4
(
	.phase    (phase    ),
	.cos_mag  (cos_mag  ),
	.sin_mag  (sin_mag  ),
	.cos_sign (cos_sign ),
	.sin_sign (sin_sign )
);

//--------------------------------------------
// sine/cosine value latch
//--------------------------------------------
reg [2:0] cos_mag_r;
reg [2:0] sin_mag_r;
reg cos_sign_r;
reg sin_sign_r;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cos_mag_r <= 'd0;
	else if (sample_in_valid)
		cos_mag_r <= cos_mag;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		sin_mag_r <= 'd0;
	else if (sample_in_valid)
		sin_mag_r <= sin_mag;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cos_sign_r <= 1'b0;
	else if (sample_in_valid)
		cos_sign_r <= cos_sign;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		sin_sign_r <= 1'b0;
	else if (sample_in_valid)
		sin_sign_r <= sin_sign;

//--------------------------------------------
// calculate sample_in * twiddle = 
// (sr + j*si)(cos - j*sin) = (sr*cos + si*sin) + j*(si*cos - sr*sin)
//--------------------------------------------
wire [5:0] real_out, imag_out;
reg [5:0] last_input_i, last_input_q;

complex_mul_sm u_complex_mul_sm
(
	.sample_sm (sample_in_r),
	.cos_mag   (cos_mag_r  ),
	.sin_mag   (sin_mag_r  ),
	.cos_sign  (cos_sign_r ),
	.sin_sign  (sin_sign_r ),
	.result_i  (real_out   ),
	.result_q  (imag_out   )
);

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		last_input_i <= 'd0;
	else if (clear)
		last_input_i <= 'd0;
	else if (sample_in_valid_d)
		last_input_i <= real_out;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		last_input_q <= 'd0;
	else if (clear)
		last_input_q <= 'd0;
	else if (sample_in_valid_d)
		last_input_q <= imag_out;

wire [6:0] add_result_i, add_result_q;
assign add_result_i = {real_out[5], real_out[5:0]} + {last_input_i[5], last_input_i[5:0]};
assign add_result_q = {imag_out[5], imag_out[5:0]} + {last_input_q[5], last_input_q[5:0]};
/*
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		sync_out <= 1'b0;
	else
		sync_out <= (sync_in && sample_in_valid && prev_input_valid);
*/
//--------------------------------------------
// assign output
//--------------------------------------------
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		sample_out_valid <= 1'b0;
	else
		sample_out_valid <= sample_in_valid & prev_input_valid;

assign sample_out_i = add_result_i[6:1];
assign sample_out_q = add_result_q[6:1];

endmodule
