//----------------------------------------------------------------------
// complex_mul.v:
//   complex value multiply module
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module m_complex_mul
(
input [7:0] sample_data_i,
input [3:0] cos_value_i,
input [3:0] sin_value_i,
input cos_sign_i,
input sin_sign_i,
output [8:0] i_data_o,
output [8:0] q_data_o
);

wire [3:0] data_i_amp;
wire [3:0] data_q_amp;
assign data_i_amp = {sample_data_i[6:4], 1'b1};
assign data_q_amp = {sample_data_i[2:0], 1'b1};

wire [7:0] i_cos, i_sin, q_cos, q_sin;

m_multiply_4x4 multiply_4x4_1
(
	.value1_i (data_i_amp),
	.value2_i (cos_value_i),
	.value_o (i_cos)
);

m_multiply_4x4 multiply_4x4_2
(
	.value1_i (data_i_amp),
	.value2_i (sin_value_i),
	.value_o (i_sin)
);

m_multiply_4x4 multiply_4x4_3
(
	.value1_i (data_q_amp),
	.value2_i (cos_value_i),
	.value_o (q_cos)
);

m_multiply_4x4 multiply_4x4_4
(
	.value1_i (data_q_amp),
	.value2_i (sin_value_i),
	.value_o (q_sin)
);

wire i_cos_sign, i_sin_sign, q_cos_sign, q_sin_sign;
assign i_cos_sign = sample_data_i[7] ^ cos_sign_i;
assign i_sin_sign = sample_data_i[7] ^ sin_sign_i;
assign q_cos_sign = sample_data_i[3] ^ cos_sign_i;
assign q_sin_sign = sample_data_i[3] ^ sin_sign_i;

assign i_data_o = {i_cos_sign, i_cos_sign ? -i_cos : i_cos} + {~q_sin_sign, q_sin_sign ? q_sin : -q_sin};
assign q_data_o = {i_sin_sign, i_sin_sign ? -i_sin : i_sin} + {q_cos_sign, q_cos_sign ? -q_cos : q_cos};

endmodule
