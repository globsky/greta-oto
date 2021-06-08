//----------------------------------------------------------------------
// amplitude.v:
//   complex number amplitude calculation using JPL algorithm
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module amplitude
(
input clk,
input rst_b,
input [9:0] data_real,
input [9:0] data_imag,
output [9:0] data_amp
);

wire [8:0] real_abs, imag_abs;

assign real_abs = {9{data_real[9]}} ^ data_real[8:0];	// NOT instead negative
assign imag_abs = {9{data_imag[9]}} ^ data_imag[8:0];	// NOT instead negative

// latch maximum and minimum value first
reg [8:0] max_abs, min_abs;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		max_abs <= 9'd0;
	else
		max_abs <= (real_abs > imag_abs) ? real_abs : imag_abs;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		min_abs <= 9'd0;
	else
		min_abs <= (real_abs > imag_abs) ? imag_abs : real_abs;

// output amplitude according to max and min
wire [9:0] amp1, amp2;
assign amp1 = {1'b0, max_abs} + {4'h0, min_abs[8:3]};
assign amp2 = {1'b0, max_abs} - {4'h0, max_abs[8:3]} + {2'b00, min_abs[8:1]};

wire [10:0] min_abs3;
assign min_abs3 = {2'b00, min_abs} + {1'b0, min_abs, 1'b0};

wire select_amp;
assign select_amp = ({2'b00, max_abs} > min_abs3);

assign data_amp = select_amp ? amp1 : amp2;

endmodule
