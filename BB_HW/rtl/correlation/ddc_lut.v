//----------------------------------------------------------------------
// ddc_lut.v:
//   sin/cos LUT table for down converter
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module m_ddc_lut
(
input [5:0]	phase_i,
output [3:0] sin_value_o,
output [3:0] cos_value_o,
output sin_sign_o,
output cos_sign_o
);

reg  [3:0]	sin_value;
reg  [3:0]	cos_value;

wire [3:0] address;
assign address = phase_i[4] ? (~phase_i[3:0]) : phase_i[3:0];

always @(*)
	case(address)
	4'd0:  sin_value = 'd1;
	4'd1:  sin_value = 'd2; 
	4'd2:  sin_value = 'd3; 
	4'd3:  sin_value = 'd4; 
	4'd4:  sin_value = 'd5; 
	4'd5:  sin_value = 'd6; 
	4'd6:  sin_value = 'd7; 
	4'd7:  sin_value = 'd8; 
	4'd8:  sin_value = 'd9; 
	4'd9:  sin_value = 'd10; 
	4'd10: sin_value = 'd10;
	4'd11: sin_value = 'd11;
	4'd12: sin_value = 'd11;
	4'd13: sin_value = 'd12;
	4'd14: sin_value = 'd12;
	4'd15: sin_value = 'd12;
	default: sin_value = 'd0;
	endcase

always @(*)
	case(address)
	4'd0:  cos_value = 'd12;
	4'd1:  cos_value = 'd12; 
	4'd2:  cos_value = 'd12; 
	4'd3:  cos_value = 'd11; 
	4'd4:  cos_value = 'd11; 
	4'd5:  cos_value = 'd10; 
	4'd6:  cos_value = 'd10; 
	4'd7:  cos_value = 'd9; 
	4'd8:  cos_value = 'd8; 
	4'd9:  cos_value = 'd7; 
	4'd10: cos_value = 'd6;
	4'd11: cos_value = 'd5;
	4'd12: cos_value = 'd4;
	4'd13: cos_value = 'd3;
	4'd14: cos_value = 'd2;
	4'd15: cos_value = 'd1;
	default: cos_value = 'd0;
	endcase

assign sin_value_o = sin_value;
assign cos_value_o = cos_value;

assign sin_sign_o = ~phase_i[5];
assign cos_sign_o = phase_i[4] ^ phase_i[5];

endmodule
