//----------------------------------------------------------------------
// complex2exp10.v:
//   16bit complex value to 10bit+exp complex conversion
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module complex2exp10
(
input	[15:0] input_i,
input	[15:0] input_q,
output	[9:0] output_i,
output	[9:0] output_q,
output	[3:0] output_exp
);

wire [5:0] sign_i, sign_q;
// determine sign bit toggle position
assign sign_i = input_i[15:10] ^ input_i[14:9];
assign sign_q = input_q[15:10] ^ input_q[14:9];

reg [2:0] exp_i, exp_q;

always @(*)
	casex(sign_i)
	6'b000000: exp_i = 3'b000;
	6'b000001: exp_i = 3'b001;
	6'b00001?: exp_i = 3'b010;
	6'b0001??: exp_i = 3'b011;
	6'b001???: exp_i = 3'b100;
	6'b01????: exp_i = 3'b101;
	6'b1?????: exp_i = 3'b110;
	default: exp_i = 3'b000;
	endcase

always @(*)
	casex(sign_q)
	6'b000000: exp_q = 3'b000;
	6'b000001: exp_q = 3'b001;
	6'b00001?: exp_q = 3'b010;
	6'b0001??: exp_q = 3'b011;
	6'b001???: exp_q = 3'b100;
	6'b01????: exp_q = 3'b101;
	6'b1?????: exp_q = 3'b110;
	default: exp_q = 3'b000;
	endcase

wire [2:0] max_exp;
assign max_exp = (exp_i > exp_q) ? exp_i : exp_q;

// shifted I/Q according to max_exp
reg [9:0] bits_i, bits_q;

always @(*)
	case(max_exp)
	3'b000: bits_i = input_i[9:0];
	3'b001: bits_i = input_i[10:1];
	3'b010: bits_i = input_i[11:2];
	3'b011: bits_i = input_i[12:3];
	3'b100: bits_i = input_i[13:4];
	3'b101: bits_i = input_i[14:5];
	3'b110: bits_i = input_i[15:6];
	default: bits_i = input_i[9:0];
	endcase

always @(*)
	case(max_exp)
	3'b000: bits_q = input_q[9:0];
	3'b001: bits_q = input_q[10:1];
	3'b010: bits_q = input_q[11:2];
	3'b011: bits_q = input_q[12:3];
	3'b100: bits_q = input_q[13:4];
	3'b101: bits_q = input_q[14:5];
	3'b110: bits_q = input_q[15:6];
	default: bits_q = input_q[9:0];
	endcase

assign output_i = bits_i;
assign output_q = bits_q;
assign output_exp = {1'b0, max_exp};

endmodule
