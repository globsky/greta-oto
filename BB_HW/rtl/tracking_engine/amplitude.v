//----------------------------------------------------------------------
// amplitude.v:
//   complex number amplitude calculation using JPL algorithm
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module amplitude #(parameter DATA_WIDTH = 16)
(
input clk,
input rst_b,
input [DATA_WIDTH-1:0] data_real,
input [DATA_WIDTH-1:0] data_imag,
output [DATA_WIDTH-1:0] data_amp
);

wire [DATA_WIDTH-2:0] real_abs, imag_abs;

assign real_abs = {(DATA_WIDTH-1){data_real[DATA_WIDTH-1]}} ^ data_real[DATA_WIDTH-2:0];	// NOT instead negative
assign imag_abs = {(DATA_WIDTH-1){data_imag[DATA_WIDTH-1]}} ^ data_imag[DATA_WIDTH-2:0];	// NOT instead negative

// latch maximum and minimum value first
reg [DATA_WIDTH-2:0] max_abs, min_abs;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		max_abs <= 'd0;
	else
		max_abs <= (real_abs > imag_abs) ? real_abs : imag_abs;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		min_abs <= 'd0;
	else
		min_abs <= (real_abs > imag_abs) ? imag_abs : real_abs;

// output amplitude according to max and min
wire [DATA_WIDTH-1:0] amp1, amp2;
assign amp1 = {1'b0, max_abs} + {4'h0, min_abs[DATA_WIDTH-2:3]};
assign amp2 = {1'b0, max_abs} - {4'h0, max_abs[DATA_WIDTH-2:3]} + {2'b00, min_abs[DATA_WIDTH-2:1]};

wire [DATA_WIDTH:0] min_abs3;
assign min_abs3 = {2'b00, min_abs} + {1'b0, min_abs, 1'b0};

wire select_amp;
assign select_amp = ({2'b00, max_abs} > min_abs3);

assign data_amp = select_amp ? amp1 : amp2;

endmodule
