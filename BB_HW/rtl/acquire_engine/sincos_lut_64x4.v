//----------------------------------------------------------------------
// sincos_lut_64x4.v:
//   64 entry 4bit sin/cos LUT
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module sincos_lut_64x4
(
input [5:0] phase,
output [2:0] cos_mag,
output [2:0] sin_mag,
output cos_sign,
output sin_sign
);

reg [5:0] lut_value;

always @(*)
	case(phase[3:0])
	4'd0:
		lut_value = {3'd7, 3'd0};
	4'd1:
		lut_value = {3'd7, 3'd1};
	4'd2:
		lut_value = {3'd7, 3'd2};
	4'd3:
		lut_value = {3'd7, 3'd2};
	4'd4:
		lut_value = {3'd6, 3'd3};
	4'd5:
		lut_value = {3'd6, 3'd4};
	4'd6:
		lut_value = {3'd6, 3'd4};
	4'd7:
		lut_value = {3'd5, 3'd5};
	4'd8:
		lut_value = {3'd5, 3'd5};
	4'd9:
		lut_value = {3'd4, 3'd6};
	4'd10:
		lut_value = {3'd4, 3'd6};
	4'd11:
		lut_value = {3'd3, 3'd6};
	4'd12:
		lut_value = {3'd2, 3'd7};
	4'd13:
		lut_value = {3'd2, 3'd7};
	4'd14:
		lut_value = {3'd1, 3'd7};
	4'd15:
		lut_value = {3'd0, 3'd7};
	default:
		lut_value = 6'd0;
	endcase

assign cos_mag = phase[4] ? lut_value[2:0] : lut_value[5:3];
assign sin_mag = phase[4] ? lut_value[5:3] : lut_value[2:0];
assign cos_sign = phase[5] ^ phase[4];
assign sin_sign = phase[5];

endmodule
