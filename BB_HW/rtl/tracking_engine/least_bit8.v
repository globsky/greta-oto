//----------------------------------------------------------------------
// least_bit8.v:
//   Find first 1 position from LSB
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module least_bit8
(
input [7:0] bits,	// input 8 bit
output [2:0] position,	// bit position of l
output active		// any bit active in input
);

wire [3:0] bits4;
wire [1:0] bits2;

assign position[2] = ~(|bits[3:0]);		// any bits active in [3:0]
assign bits4 = position[2] ? bits[7:4] : bits[3:0];	// select first/second half
assign position[1] = ~(|bits4[1:0]);		// any bits active in [1:0] of half bits
assign bits2 = position[1] ? bits4[3:2] : bits4[1:0];	//select first/second half
assign position[0] = ~bits2[0];
assign active = (|bits);

endmodule
