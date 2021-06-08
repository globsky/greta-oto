//----------------------------------------------------------------------
// round_shift.v:
//   unbiased round shift module
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module round_shift #(parameter DATA_WIDTH = 9, SHIFT_BITS = 2)
(
input	[DATA_WIDTH-1:0] data_input,
output	[DATA_WIDTH-SHIFT_BITS-1:0] data_output
);

wire round_valid;
assign round_valid = data_input[SHIFT_BITS] | (~data_input[SHIFT_BITS-1]) | (|data_input[SHIFT_BITS-2:0]);

wire round_bit;
assign round_bit = data_input[SHIFT_BITS-1] & round_valid;

assign data_output = data_input[DATA_WIDTH-1:SHIFT_BITS] + round_bit;

endmodule
