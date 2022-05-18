//----------------------------------------------------------------------
// nh_code_gen.v:
//   NH code generation module
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module nh_code_gen
(
// system signals				
input clk,
input rst_b,
// internal signals
input [24:0] nh_code1,	// NH code 1
input [19:0] nh_code2,	// NH code 2
input nh_increase,
input [4:0] nh_length,
input nh_count_en,
input [4:0] nh_count_i,
output reg [4:0] nh_count_o,
output cur_nh_code1,
output cur_nh_code2
);

wire nh_enable;
assign nh_enable = (|nh_length);

wire [5:0] nh_count_next;
wire       nh_count_reset;
assign     nh_count_next = nh_count_o + 1'b1;
assign     nh_count_reset = (nh_count_next == nh_length) & nh_increase;

// nh counter
always @ (posedge clk or negedge rst_b)
	if (~rst_b)
		nh_count_o <= 5'h0;
	else if (nh_count_en)
		nh_count_o <= nh_count_i;
	else if (nh_count_reset)
		nh_count_o <= 5'h0;
	else if (nh_increase & nh_enable)
		nh_count_o <= nh_count_next;
		
//generate nh code
reg nh_code1_select, nh_code2_select;

always @ (*)
	case (nh_count_o)
	    5'd0:  nh_code1_select = nh_code1[0];
	    5'd1:  nh_code1_select = nh_code1[1];
	    5'd2:  nh_code1_select = nh_code1[2];
	    5'd3:  nh_code1_select = nh_code1[3];
	    5'd4:  nh_code1_select = nh_code1[4];
	    5'd5:  nh_code1_select = nh_code1[5];
	    5'd6:  nh_code1_select = nh_code1[6];
	    5'd7:  nh_code1_select = nh_code1[7];
	    5'd8:  nh_code1_select = nh_code1[8];
	    5'd9:  nh_code1_select = nh_code1[9];
	    5'd10: nh_code1_select = nh_code1[10];
	    5'd11: nh_code1_select = nh_code1[11];
	    5'd12: nh_code1_select = nh_code1[12];
	    5'd13: nh_code1_select = nh_code1[13];
	    5'd14: nh_code1_select = nh_code1[14];
	    5'd15: nh_code1_select = nh_code1[15];
	    5'd16: nh_code1_select = nh_code1[16];
	    5'd17: nh_code1_select = nh_code1[17];
	    5'd18: nh_code1_select = nh_code1[18];
	    5'd19: nh_code1_select = nh_code1[19];
	    5'd20: nh_code1_select = nh_code1[20];
	    5'd21: nh_code1_select = nh_code1[21];
	    5'd22: nh_code1_select = nh_code1[22];
	    5'd23: nh_code1_select = nh_code1[23];
	    5'd24: nh_code1_select = nh_code1[24];
	    default: nh_code1_select = 1'b0;
	endcase

always @ (*)
	case (nh_count_o)
	    5'd0:  nh_code2_select = nh_code2[0];
	    5'd1:  nh_code2_select = nh_code2[1];
	    5'd2:  nh_code2_select = nh_code2[2];
	    5'd3:  nh_code2_select = nh_code2[3];
	    5'd4:  nh_code2_select = nh_code2[4];
	    5'd5:  nh_code2_select = nh_code2[5];
	    5'd6:  nh_code2_select = nh_code2[6];
	    5'd7:  nh_code2_select = nh_code2[7];
	    5'd8:  nh_code2_select = nh_code2[8];
	    5'd9:  nh_code2_select = nh_code2[9];
	    5'd10: nh_code2_select = nh_code2[10];
	    5'd11: nh_code2_select = nh_code2[11];
	    5'd12: nh_code2_select = nh_code2[12];
	    5'd13: nh_code2_select = nh_code2[13];
	    5'd14: nh_code2_select = nh_code2[14];
	    5'd15: nh_code2_select = nh_code2[15];
	    5'd16: nh_code2_select = nh_code2[16];
	    5'd17: nh_code2_select = nh_code2[17];
	    5'd18: nh_code2_select = nh_code2[18];
	    5'd19: nh_code2_select = nh_code2[19];
	    5'd20: nh_code2_select = nh_code2[0];
	    5'd21: nh_code2_select = nh_code2[1];
	    5'd22: nh_code2_select = nh_code2[2];
	    5'd23: nh_code2_select = nh_code2[3];
	    5'd24: nh_code2_select = nh_code2[4];
	    default: nh_code2_select = 1'b0;
	endcase

assign cur_nh_code1 = nh_enable ? nh_code1_select : 1'b0;
assign cur_nh_code2 = nh_enable ? nh_code2_select : 1'b0;

endmodule
