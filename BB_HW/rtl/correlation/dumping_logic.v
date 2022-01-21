//----------------------------------------------------------------------
// dumping_logic.v:
//   Dumping signal generation and dump data and address output
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module dumping_logic
(
// system signals
input	clk,
input	rst_b,
// control signals
input overflow,
input shift_code,
input [4:0] bit_length,
input [5:0] coherent_number,
input enable_2nd_prn,
// variable
input [15:0] dump_length,
input dump_count_en,
input [15:0] dump_count_i,
output reg [15:0] dump_count_o,
input dumping_en,
input dumping_i,
output reg dumping_o,
input current_cor_en,
input [2:0] current_cor_i,
output reg [2:0] current_cor_o,
input coherent_count_en,
input [5:0] coherent_count_i,
output reg [5:0] coherent_count_o,
input bit_count_en,
input [4:0] bit_count_i,
output reg [4:0] bit_count_o,
// acc data input and dumping data output
input [15:0] i_acc_shift,
input [15:0] q_acc_shift,
output reg coherent_sum_valid,
output reg [4:0] cor_index,  // bit4~2: correlator index, bit1: overwrite protect flag, bit0: new coherent sum
output reg [15:0] i_coherent_sum, // I channel data for coherent sum
output reg [15:0] q_coherent_sum, // Q channel data for coherent sum
// state signals
output dumping_valid,
output reg coherent_done_o,    //coherent data is ready
output reg overwrite_protect,
output reg data_decode_valid
);

//overflow delay
reg overflow_d;
always @ (posedge clk or negedge rst_b)
	if (~rst_b)
		overflow_d <= 1'b0;
	else 
		overflow_d <= overflow;

wire dumping_clear;
assign dumping_valid = dumping_o & overflow_d;
assign dumping_clear = (dumping_valid) & (current_cor_o == 3'd7);

//--------------------------------------------------------------
// variable counter load and counting
//--------------------------------------------------------------
wire [15:0] dump_count_next;
wire dump_count_reset;
assign dump_count_next = dump_count_o + 1'b1;
assign dump_count_reset = (dump_count_next == dump_length);

always @ (posedge clk or negedge rst_b)
	if (~rst_b)
		dump_count_o <= 16'h0;
	else if (dump_count_en)
		dump_count_o <= dump_count_i;
	else if (shift_code)
		dump_count_o <= dump_count_reset ? 16'd0 : dump_count_next;

always @ (posedge clk or negedge rst_b)
	if (~rst_b)
		dumping_o <= 1'b0;
	else if (dumping_en)
		dumping_o <= dumping_i;
	else if (dumping_clear)
		dumping_o <= 1'b0;
	else if (shift_code & dump_count_reset)
		dumping_o <= 1'b1;

always @ (posedge clk or negedge rst_b)
	if (~rst_b)
		current_cor_o <= 3'd0;
	else if (current_cor_en)
		current_cor_o <= current_cor_i;
	else if (dumping_clear)		
		current_cor_o <= 3'd0;
	else if (dumping_valid)
		current_cor_o <= current_cor_o + 1'b1;

wire [5:0] coherent_count_next;
wire coherent_count_clear;
assign coherent_count_next = coherent_count_o + 1'b1;
assign coherent_count_clear = (coherent_count_next == coherent_number) & dumping_clear;

always @ (posedge clk or negedge rst_b)
	if (~rst_b)
		coherent_count_o <= 6'd0;
	else if (coherent_count_en)
		coherent_count_o <= coherent_count_i;
	else if (coherent_count_clear)
		coherent_count_o <= 6'd0;
	else if (dumping_clear)
		coherent_count_o <= coherent_count_next;

//--------------------------------------------------------------
// latch coherent sum data to dump
//--------------------------------------------------------------
always @ (posedge clk or negedge rst_b)
	if (~rst_b)
  begin
		i_coherent_sum <= 16'h0;
		q_coherent_sum <= 16'h0;
  end
  else if (dumping_valid)
  begin
		i_coherent_sum <= i_acc_shift;
		q_coherent_sum <= q_acc_shift;
  end

//--------------------------------------------------------------
// bit count increment and state determine
//--------------------------------------------------------------
wire [4:0] bit_count_next;
wire is_data_cor;
assign bit_count_next = bit_count_o + 1'b1;
assign is_data_cor = enable_2nd_prn && (current_cor_o == 3'b000) && (|bit_length);

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		bit_count_o <= 5'd0;
	else if (bit_count_en)
		bit_count_o <= bit_count_i;
	else if (dumping_valid && is_data_cor)
		bit_count_o <= (bit_count_next == bit_length) ? 5'd0 : bit_count_next;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		data_decode_valid <= 1'b0;
	else if (bit_count_en)
		data_decode_valid <= 1'b0;	// clear on fill
	else if (dumping_valid && is_data_cor && (bit_count_next == bit_length))
		data_decode_valid <= 1'b1;

//--------------------------------------------------------------
// overwrite_protect
//--------------------------------------------------------------
reg first_cor_index_valid;
reg [2:0] first_cor_index;
wire overwrite_flag;

assign overwrite_flag = first_cor_index_valid && (first_cor_index == current_cor_o) && (coherent_count_o == 0) && dumping_valid;

always @ (posedge clk or negedge rst_b)
	if (~rst_b)
		overwrite_protect <= 0;
	else if (current_cor_en)	// clear on every fill state
		overwrite_protect <= 0;
	else if (overwrite_flag)
		overwrite_protect <= 1;

always @ (posedge clk or negedge rst_b)
	if (~rst_b)
		first_cor_index_valid <= 0;
	else if (current_cor_en)	// clear on every fill state
		first_cor_index_valid <= 0;
	else if (dumping_valid)
		first_cor_index_valid <= 1;

always @ (posedge clk or negedge rst_b)
	if (~rst_b)
		first_cor_index <= 0;
	else if (current_cor_en)	// clear on every fill state
		first_cor_index <= 0;
	else if (dumping_valid && !first_cor_index_valid)
		first_cor_index <= current_cor_o;

//--------------------------------------------------------------
// cor index generation
//--------------------------------------------------------------
always @ (posedge clk or negedge rst_b)
	if (~rst_b)
		coherent_sum_valid <= 1'b0;
	else
		coherent_sum_valid <= dumping_valid;

always @ (posedge clk or negedge rst_b)
	if (~rst_b)
		cor_index <= 5'h0;
	else if (dumping_valid)
		cor_index <= {current_cor_o, overwrite_flag | overwrite_protect, is_data_cor ? (~(|bit_count_o)) : (~(|coherent_count_o))};

always @ (posedge clk or negedge rst_b)
	if (~rst_b)
		coherent_done_o <= 1'b0;
	else if (coherent_count_en)
		coherent_done_o <= 1'b0;
	else if ((coherent_count_next == coherent_number) & dumping_valid)
		coherent_done_o <= 1'b1;

endmodule
