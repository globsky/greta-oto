//----------------------------------------------------------------------
// data_acc.v:
//   Correlator data accumulator
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module data_acc #(parameter ACC_DATA_WIDTH = 16)
(
// system signals
input	clk,
input	rst_b,
// data load
input acc_in_en,
input [ACC_DATA_WIDTH-1:0] i_acc_i,
input [ACC_DATA_WIDTH-1:0] q_acc_i,
// input data and PRN
input acc_clear,
input [5:0] i_data_pos,
input [5:0] q_data_pos,
input [5:0] i_data_neg,
input [5:0] q_data_neg,
input prn_code,
output reg [ACC_DATA_WIDTH-1:0] i_acc_o,
output reg [ACC_DATA_WIDTH-1:0] q_acc_o
);

localparam EXPAND_WIDTH = ACC_DATA_WIDTH - 6;

wire [5:0] i_acc;
wire [5:0] q_acc;
assign i_acc = prn_code ? i_data_neg : i_data_pos;
assign q_acc = prn_code ? q_data_neg : q_data_pos;

wire [ACC_DATA_WIDTH-1:0] i_acc_feedback;
wire [ACC_DATA_WIDTH-1:0] q_acc_feedback;
assign i_acc_feedback = acc_clear? 'd0 : i_acc_o;
assign q_acc_feedback = acc_clear? 'd0 : q_acc_o;

always @ (posedge clk or negedge rst_b)
	if (~rst_b)
		i_acc_o <= 'd0;
	else if (acc_in_en)
		i_acc_o <= i_acc_i;
	else
		i_acc_o <= i_acc_feedback + {{EXPAND_WIDTH{i_acc[5]}}, i_acc};

always @ (posedge clk or negedge rst_b)
	if (~rst_b)
		q_acc_o <= 'd0;
	else if (acc_in_en)
		q_acc_o <= q_acc_i;
	else
		q_acc_o <= q_acc_feedback + {{EXPAND_WIDTH{q_acc[5]}}, q_acc};

endmodule
