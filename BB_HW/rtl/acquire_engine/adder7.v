//----------------------------------------------------------------------
// adder7.v:
//   7 input 1bit adder
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module adder7
(
input [6:0] in,
output [2:0] out
);

wire [1:0] sum1, sum2, sum3, sum4;

adder3 adder3_1
(
	.in  (in[2:0]),
	.out (sum1   )
);

adder3 adder3_2
(
	.in  (in[5:3]),
	.out (sum2   )
);

adder3 adder3_3
(
	.in  ({sum1[0], sum2[0], in[6]}),
	.out (sum3)
);

adder3 adder3_4
(
	.in  ({sum1[1], sum2[1], sum3[1]}),
	.out (sum4)
);

assign out = {sum4, sum3[0]};

endmodule
