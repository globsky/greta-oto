//----------------------------------------------------------------------
// sincos_lut_256x10.v:
//   256 entry 10bit sin/cos LUT
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module sincos_lut_256x10
(
input [7:0] phase,
output [8:0] cos_mag,
output [8:0] sin_mag,
output cos_sign,
output sin_sign
);

reg [17:0] lut_value;

always @(*)
	case(phase[5:0])
	 6'd0: lut_value = { 9'd511,   9'd0 };
	 6'd1: lut_value = { 9'd511,  9'd13 };
	 6'd2: lut_value = { 9'd510,  9'd25 };
	 6'd3: lut_value = { 9'd510,  9'd38 };
	 6'd4: lut_value = { 9'd509,  9'd50 };
	 6'd5: lut_value = { 9'd507,  9'd63 };
	 6'd6: lut_value = { 9'd505,  9'd75 };
	 6'd7: lut_value = { 9'd503,  9'd87 };
	 6'd8: lut_value = { 9'd501, 9'd100 };
	 6'd9: lut_value = { 9'd499, 9'd112 };
	6'd10: lut_value = { 9'd496, 9'd124 };
	6'd11: lut_value = { 9'd492, 9'd136 };
	6'd12: lut_value = { 9'd489, 9'd148 };
	6'd13: lut_value = { 9'd485, 9'd160 };
	6'd14: lut_value = { 9'd481, 9'd172 };
	6'd15: lut_value = { 9'd477, 9'd184 };
	6'd16: lut_value = { 9'd472, 9'd196 };
	6'd17: lut_value = { 9'd467, 9'd207 };
	6'd18: lut_value = { 9'd462, 9'd218 };
	6'd19: lut_value = { 9'd456, 9'd230 };
	6'd20: lut_value = { 9'd451, 9'd241 };
	6'd21: lut_value = { 9'd445, 9'd252 };
	6'd22: lut_value = { 9'd438, 9'd263 };
	6'd23: lut_value = { 9'd432, 9'd273 };
	6'd24: lut_value = { 9'd425, 9'd284 };
	6'd25: lut_value = { 9'd418, 9'd294 };
	6'd26: lut_value = { 9'd410, 9'd304 };
	6'd27: lut_value = { 9'd403, 9'd314 };
	6'd28: lut_value = { 9'd395, 9'd324 };
	6'd29: lut_value = { 9'd387, 9'd334 };
	6'd30: lut_value = { 9'd379, 9'd343 };
	6'd31: lut_value = { 9'd370, 9'd352 };
	6'd32: lut_value = { 9'd361, 9'd361 };
	6'd33: lut_value = { 9'd352, 9'd370 };
	6'd34: lut_value = { 9'd343, 9'd379 };
	6'd35: lut_value = { 9'd334, 9'd387 };
	6'd36: lut_value = { 9'd324, 9'd395 };
	6'd37: lut_value = { 9'd314, 9'd403 };
	6'd38: lut_value = { 9'd304, 9'd410 };
	6'd39: lut_value = { 9'd294, 9'd418 };
	6'd40: lut_value = { 9'd284, 9'd425 };
	6'd41: lut_value = { 9'd273, 9'd432 };
	6'd42: lut_value = { 9'd263, 9'd438 };
	6'd43: lut_value = { 9'd252, 9'd445 };
	6'd44: lut_value = { 9'd241, 9'd451 };
	6'd45: lut_value = { 9'd230, 9'd456 };
	6'd46: lut_value = { 9'd218, 9'd462 };
	6'd47: lut_value = { 9'd207, 9'd467 };
	6'd48: lut_value = { 9'd196, 9'd472 };
	6'd49: lut_value = { 9'd184, 9'd477 };
	6'd50: lut_value = { 9'd172, 9'd481 };
	6'd51: lut_value = { 9'd160, 9'd485 };
	6'd52: lut_value = { 9'd148, 9'd489 };
	6'd53: lut_value = { 9'd136, 9'd492 };
	6'd54: lut_value = { 9'd124, 9'd496 };
	6'd55: lut_value = { 9'd112, 9'd499 };
	6'd56: lut_value = { 9'd100, 9'd501 };
	6'd57: lut_value = {  9'd87, 9'd503 };
	6'd58: lut_value = {  9'd75, 9'd505 };
	6'd59: lut_value = {  9'd63, 9'd507 };
	6'd60: lut_value = {  9'd50, 9'd509 };
	6'd61: lut_value = {  9'd38, 9'd510 };
	6'd62: lut_value = {  9'd25, 9'd510 };
	6'd63: lut_value = {  9'd13, 9'd511 };
	default:
		lut_value = 18'd0;
	endcase

assign cos_mag = phase[6] ? lut_value[8:0] : lut_value[17:9];
assign sin_mag = phase[6] ? lut_value[17:9] : lut_value[8:0];
assign cos_sign = phase[7] ^ phase[6];
assign sin_sign = phase[7];

endmodule
