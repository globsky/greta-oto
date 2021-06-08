//----------------------------------------------------------------------
// round_shift_10_4.v:
//   biased 10bit round shift with 4 shift bits control
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module round_shift_10_4
(
input [9:0] data_input,
input [3:0] shift_bit,
output reg [9:0] data_output
);

always @(*)
	case(shift_bit)
	4'd0: data_output = data_input;
	4'd1: data_output = {1'd0, data_input[9:1]} + data_input[0];
	4'd2: data_output = {2'd0, data_input[9:2]} + data_input[1];
	4'd3: data_output = {3'd0, data_input[9:3]} + data_input[2];
	4'd4: data_output = {4'd0, data_input[9:4]} + data_input[3];
	4'd5: data_output = {5'd0, data_input[9:5]} + data_input[4];
	4'd6: data_output = {6'd0, data_input[9:6]} + data_input[5];
	4'd7: data_output = {7'd0, data_input[9:7]} + data_input[6];
	4'd8: data_output = {8'd0, data_input[9:8]} + data_input[7];
	4'd9: data_output = {9'd0, data_input[9]} + data_input[8];
	4'd10: data_output = {9'd0, data_input[9]};
	default: data_output = 10'd0;
	endcase

endmodule
