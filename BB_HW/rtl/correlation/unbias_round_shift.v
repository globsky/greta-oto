//----------------------------------------------------------------------
// unbias_round_shift.v:
//   unbiased round shift 3~5bit with saturation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module unbias_round_shift
(
input [8:0] data_i,
input [1:0] shift_bits,
output [5:0] data_o
);

reg [5:0] shift_data;
always @ (*)
	case(shift_bits)
		2'b00: shift_data = {data_i[8:3]};
		2'b01: shift_data = {data_i[8], data_i[8:4]};
		2'b10: shift_data = {data_i[8], data_i[8], data_i[8:5]};
		default: shift_data = data_i[8:3];
	endcase
	
reg carry_bit;
always @ (*) 
	case(shift_bits)
		2'b00: carry_bit = data_i[2] & (data_i[3] | (|data_i[1:0]));
		2'b01: carry_bit = data_i[3] & (data_i[4] | (|data_i[2:0]));
		2'b10: carry_bit = data_i[4] & (data_i[5] | (|data_i[3:0]));
		default: carry_bit = 1'b0;
	endcase	

wire [6:0] round_value;
assign round_value = {shift_data[5], shift_data} + carry_bit;

assign data_o = (round_value[5:0] == 6'b100000) ? (round_value[6] ? 6'd33 : 6'd31) : round_value[5:0];	// saturate +-32 to +-31 to protect overflow

endmodule
