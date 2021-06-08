//----------------------------------------------------------------------
// complex_mul_sm.v:
//   4bit sign/mag format complex multiply 2bit sign/mag complex sample
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module complex_mul_sm
(
input [3:0] sample_sm,
input [2:0] cos_mag,
input [2:0] sin_mag,
input cos_sign,
input sin_sign,
output [5:0] result_i,
output [5:0] result_q
);

wire [5:0] real_cos, real_sin, imag_cos, imag_sin;
wire sign_rc, sign_rs, sign_ic, sign_is;

assign real_cos = {3'b00, cos_mag} + (sample_sm[2] ? {2'b00, cos_mag, 1'b0} : 5'd0);
assign real_sin = {3'b00, sin_mag} + (sample_sm[2] ? {2'b00, sin_mag, 1'b0} : 5'd0);
assign imag_cos = {3'b00, cos_mag} + (sample_sm[0] ? {2'b00, cos_mag, 1'b0} : 5'd0);
assign imag_sin = {3'b00, sin_mag} + (sample_sm[0] ? {2'b00, sin_mag, 1'b0} : 5'd0);
assign sign_rc = sample_sm[3] ^ cos_sign;
assign sign_rs = sample_sm[3] ^ sin_sign;
assign sign_ic = sample_sm[1] ^ cos_sign;
assign sign_is = sample_sm[1] ^ sin_sign;

assign result_i = (sign_rc ? (~real_cos + 1) : real_cos) + (sign_is ? (~imag_sin + 1) : imag_sin);
assign result_q = (sign_ic ? (~imag_cos + 1) : imag_cos) + (sign_rs ? real_sin : (~real_sin + 1));

endmodule
