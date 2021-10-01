//----------------------------------------------------------------------
// noncoh_sum.v:
//   do summation of coherent result and non-coherent amplitude
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module noncoh_sum
(
// system signals				
input clk,	//system clock
input rst_b,	//reset signal, low active

// input coherent data and noncoherent data
input	coh_valid,		// coherent data valid
input	[23:0] coh_data,
input [3:0] coh_shift,
// noncoherent data and shift has two clock cycle delay to coherent data
input [7:0] noncoh_data,
input [3:0] noncoh_shift,

input extra_shift,
output exceed,	// output exceed bit one cycle before latched result of noncoh_out

// output sum data
output reg [8:0] noncoh_out
);

reg [2:0] coh_valid_d;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coh_valid_d <= 3'b000;
	else
		coh_valid_d <= {coh_valid_d[1:0], coh_valid};

wire [9:0] data_amp;
reg [9:0] coh_amplitude;

amplitude #(.DATA_WIDTH(10)) u_amplitude
(
	.clk       (clk            ),
	.rst_b     (rst_b          ),
	.data_real (coh_data[23:14]),
	.data_imag (coh_data[13:4] ),
	.data_amp  (data_amp       )
);

// latch amplitude
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coh_amplitude <= 'd0;
	else if (coh_valid_d[0])
		coh_amplitude <= data_amp;

// delay coh_shift 2 clock cycles to match time slot of amplitude
reg [3:0] coh_shift_d, coh_shift_d2;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coh_shift_d <= 'd0;
	else if (coh_valid)
		coh_shift_d <= coh_shift - coh_data[3:0];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coh_shift_d2 <= 'd0;
	else if (coh_valid_d[0])
		coh_shift_d2 <= coh_shift_d;

// shift coherent data
wire [9:0] coh_shift_amp;

round_shift_10_4 u_shift_coh
(
	.data_input   (coh_amplitude),
	.shift_bit    (coh_shift_d2 ),
	.data_output  (coh_shift_amp)
);

// shift noncoherent data
wire [9:0] noncoh_shift_amp;

round_shift_10_4 u_shift_noncoh
(
	.data_input   ({2'b00, noncoh_data}),
	.shift_bit    (noncoh_shift        ),
	.data_output  (noncoh_shift_amp    )
);

// calculate sum of coherent and noncoherent result
wire [9:0] noncoh_sum, noncoh_sum_adjust;
wire [8:0] noncoh_sum_clip;
assign noncoh_sum = coh_shift_amp + noncoh_shift_amp;
assign noncoh_sum_adjust = extra_shift ? ({1'b0, noncoh_sum[9:1]} + noncoh_sum[0]) : noncoh_sum;
assign noncoh_sum_clip = noncoh_sum[9] ? 9'd510 : noncoh_sum_adjust[8:0];

// latch output value
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		noncoh_out <= 'd0;
	else if (coh_valid_d[1])
		noncoh_out <= noncoh_sum_clip;

assign exceed = noncoh_sum_clip[8];

endmodule
