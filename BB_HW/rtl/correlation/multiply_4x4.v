//----------------------------------------------------------------------
// multiply_4x4.v:
//   4bit x 4bit unsigned multiply module
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module m_multiply_4x4
(
input [3:0] value1_i,
input [3:0] value2_i,
input [7:0] value_o
);

wire [7:0] shift1, shift2, shift3, shift4;
assign shift1 = value2_i[0] ? {4'b0000, value1_i} : 0;
assign shift2 = value2_i[1] ? {3'b000, value1_i, 1'b0} : 0;
assign shift3 = value2_i[2] ? {2'b00, value1_i, 2'b00} : 0;
assign shift4 = value2_i[3] ? {1'b0, value1_i, 3'b000} : 0;

assign value_o = shift1 + shift2 + shift3 + shift4;

endmodule
