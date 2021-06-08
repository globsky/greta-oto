//----------------------------------------------------------------------
// adder31.v:
//   31 input 1bit adder
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module adder31
(
input [30:0] in,
output [4:0] out
);

wire [2:0] sum1, sum2, sum3, sum4;
wire [1:0] sum5;

adder7 adder7_1
(
	.in  (in[6:0]  ),
	.out (sum1     )
);

adder7 adder7_2
(
	.in  (in[13:7] ),
	.out (sum2     )
);

adder7 adder7_3
(
	.in  (in[20:14]),
	.out (sum3     )
);

adder7 adder7_4
(
	.in  (in[27:21]),
	.out (sum4     )
);

adder3 u_adder3
(
	.in  (in[30:28]),
	.out (sum5     )
);

assign out = {2'b00, sum1} + {2'b00, sum2} + {2'b00, sum3} + {2'b00, sum4} + {3'b000, sum5};

endmodule
