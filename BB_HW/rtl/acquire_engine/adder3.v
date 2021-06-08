//----------------------------------------------------------------------
// adder3.v:
//   3 input 1bit adder (1bit full adder)
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module adder3
(
input [2:0] in,
output reg [1:0] out
);

always @(*)
	case(in)
		3'b000: out = 2'b00;
		3'b001: out = 2'b01;
		3'b010: out = 2'b01;
		3'b011: out = 2'b10;
		3'b100: out = 2'b01;
		3'b101: out = 2'b10;
		3'b110: out = 2'b10;
		3'b111: out = 2'b11;
		default: out = 2'b00;
	endcase

endmodule
