//----------------------------------------------------------------------
// adder_tree.v:
//   1023 input 1bit adder tree
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module adder_tree
(
// system signals				
input clk,	//system clock
input rst_b,	//reset signal, low active
// 341 input add to 9bit result
input	[340:0] in,
output reg [8:0] out
);

genvar i_gen;

//----------------------------------------------------------
// split into 11 instances of 31bit adder and latch
//----------------------------------------------------------
wire [4:0] adder31_out[10:0];

generate
for (i_gen = 0; i_gen < 11; i_gen = i_gen + 1) begin: add_tree_gen
	adder31 adder31_inst
	(
			.in  (in[i_gen*31+30:i_gen*31]),
			.out (adder31_out[i_gen])
	);
end
endgenerate

reg [4:0] adder31_out_r[10:0];

generate
for (i_gen = 0; i_gen < 11; i_gen = i_gen + 1) begin: add_tree_latch
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		adder31_out_r[i_gen] <= 'd0;
	else
		adder31_out_r[i_gen] <= adder31_out[i_gen];
end
endgenerate

//----------------------------------------------------------
// second stage add 11 result together
//----------------------------------------------------------
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		out <= 'd0;
	else
		out <=   {4'b0000, adder31_out_r[0]} + {4'b000, adder31_out_r[1]} + {4'b000, adder31_out_r[2]}
		       + {4'b0000, adder31_out_r[3]} + {4'b000, adder31_out_r[4]} + {4'b000, adder31_out_r[5]}
		       + {4'b0000, adder31_out_r[6]} + {4'b000, adder31_out_r[7]} + {4'b000, adder31_out_r[8]}
		       + {4'b0000, adder31_out_r[9]} + {4'b000, adder31_out_r[10]};

endmodule
