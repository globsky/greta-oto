//----------------------------------------------------------------------
// multiplex_4_1.v:
//   4 to 1 multiplex
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module multiplex_4_1 #(parameter DATA_WIDTH = 32)
(
input [DATA_WIDTH-1:0] data_in_0,
input [DATA_WIDTH-1:0] data_in_1,
input [DATA_WIDTH-1:0] data_in_2,
input [DATA_WIDTH-1:0] data_in_3,
input [1:0] data_sel,
output reg [DATA_WIDTH-1:0] data_out
);
	
always @ (*)
	case(data_sel)
		2'b00  : data_out = data_in_0;
		2'b01  : data_out = data_in_1;
		2'b10  : data_out = data_in_2;
		2'b11  : data_out = data_in_3;
		default: data_out = 'd0;
	endcase

endmodule
