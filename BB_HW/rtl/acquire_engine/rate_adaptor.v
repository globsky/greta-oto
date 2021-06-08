//----------------------------------------------------------------------
// rate_adaptor.v:
//   Code rate adaptor and requantization for AE
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module rate_adaptor
(
// system signals
input clk,   // system clock
input rst_b, // reset signal, low active

// input sample data and output requantized data
input sample_valid,				// sample data valid signal
input [7:0]	sample_data,	// sample data
output data_valid,				// requantized data valid signal
output [3:0] data_quant,	// requantized data

// interface of control
input init_nco,
input [23:0] code_rate_ratio,	// code rate decimation ratio
input [31:0] carrier_freq,	// carrier frequency
input [7:0] threshold
);

//----------------------------------------------------------
// accumulation of code rate NCO and carrier NCO
//----------------------------------------------------------
reg [24:0] code_rate_nco;
reg [31:0] carrier_nco;
reg nco_msb;
assign code_nco = code_rate_nco;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		code_rate_nco <= 25'd0;
	else if (init_nco)
		code_rate_nco <= 25'd0;
	else if (sample_valid)
		code_rate_nco <= code_rate_nco + {1'b0, code_rate_ratio};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		carrier_nco <= 32'd0;
	else if (init_nco)
		carrier_nco <= 32'd0;
	else if (sample_valid)
		carrier_nco <= carrier_nco + carrier_freq;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		nco_msb <= 1'b0;
	else
		nco_msb <= code_rate_nco[24];

//----------------------------------------------------------
// down convert input data
//----------------------------------------------------------
wire [3:0] sin_value;
wire [3:0] cos_value;
wire sin_sign;
wire cos_sign;

m_ddc_lut ddc_lut
(
  .phase_i     (carrier_nco[31:26]),
  .sin_value_o (sin_value         ),
  .cos_value_o (cos_value         ),
  .sin_sign_o  (sin_sign          ),
  .cos_sign_o  (cos_sign          )
);

wire [8:0] i_data;
wire [8:0] q_data;
m_complex_mul complex_mul
(
  .sample_data_i (sample_data ),
  .cos_value_i   (cos_value   ),
  .sin_value_i   (sin_value   ),
  .sin_sign_i    (sin_sign    ),
  .cos_sign_i    (cos_sign    ),
  .i_data_o      (i_data      ),
  .q_data_o      (q_data      )
);

//----------------------------------------------------------
// latch multiply result into filter buffer
//----------------------------------------------------------
reg [8:0] filter_buffer_i [5:0];
reg [8:0] filter_buffer_q [5:0];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		filter_buffer_i[0] <= 6'd0;
	else if (sample_valid)
		filter_buffer_i[0] <= i_data;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		filter_buffer_q[0] <= 6'd0;
	else if (sample_valid)
		filter_buffer_q[0] <= q_data;

genvar i_gen;

generate
  for (i_gen = 1; i_gen < 6; i_gen = i_gen + 1) begin: buffer_gen_i
		always @(posedge clk or negedge rst_b)
			if (!rst_b)
				filter_buffer_i[i_gen] <= 6'd0;
			else if (sample_valid)
				filter_buffer_i[i_gen] <= filter_buffer_i[i_gen-1];
	end
endgenerate

generate
  for (i_gen = 1; i_gen < 6; i_gen = i_gen + 1) begin: buffer_gen_q
		always @(posedge clk or negedge rst_b)
			if (!rst_b)
				filter_buffer_q[i_gen] <= 6'd0;
			else if (sample_valid)
				filter_buffer_q[i_gen] <= filter_buffer_q[i_gen-1];
	end
endgenerate

//----------------------------------------------------------
// do filter
//----------------------------------------------------------
wire do_filter;
reg [3:0] do_filter_d;
assign do_filter = code_rate_nco[24] ^ nco_msb;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		do_filter_d <= 4'b000;
	else
		do_filter_d <= {do_filter_d[2:0], do_filter};

wire signed [9:0] buffer_sum_i [2:0];
wire signed [9:0] buffer_sum_q [2:0];
wire signed [9:0] buffer_sum [2:0];

// add two samples in buffer together
assign buffer_sum_i[0] = {filter_buffer_i[0][8], filter_buffer_i[0]} + {filter_buffer_i[5][8], filter_buffer_i[5]};
assign buffer_sum_i[1] = {filter_buffer_i[1][8], filter_buffer_i[1]} + {filter_buffer_i[4][8], filter_buffer_i[4]};
assign buffer_sum_i[2] = {filter_buffer_i[2][8], filter_buffer_i[2]} + {filter_buffer_i[3][8], filter_buffer_i[3]};
assign buffer_sum_q[0] = {filter_buffer_q[0][8], filter_buffer_q[0]} + {filter_buffer_q[5][8], filter_buffer_q[5]};
assign buffer_sum_q[1] = {filter_buffer_q[1][8], filter_buffer_q[1]} + {filter_buffer_q[4][8], filter_buffer_q[4]};
assign buffer_sum_q[2] = {filter_buffer_q[2][8], filter_buffer_q[2]} + {filter_buffer_q[3][8], filter_buffer_q[3]};
assign buffer_sum[0] = do_filter_d[0] ? buffer_sum_q[0] : buffer_sum_i[0];
assign buffer_sum[1] = do_filter_d[0] ? buffer_sum_q[1] : buffer_sum_i[1];
assign buffer_sum[2] = do_filter_d[0] ? buffer_sum_q[2] : buffer_sum_i[2];

reg signed [14:0] mul_data[2:0];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		mul_data[0] <= 15'd0;
	else if (do_filter || do_filter_d[0])
		mul_data[0] <= {{3{buffer_sum[0][9]}} , buffer_sum[0], 2'b00} + {{5{buffer_sum[0][9]}} , buffer_sum[0]};	// x5

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		mul_data[1] <= 15'd0;
	else if (do_filter || do_filter_d[0])
		mul_data[1] <= {{2{buffer_sum[1][9]}}, buffer_sum[1], 3'b000};	// x8

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		mul_data[2] <= 14'd0;
	else if (do_filter || do_filter_d[0])
		mul_data[2] <= {buffer_sum[2][9], buffer_sum[2], 4'b0000} + {{3{buffer_sum[2][9]}} , buffer_sum[2], 2'b00};	// x20

wire signed [14:0] filter_data;
wire signed [8:0] round_data;
assign filter_data = mul_data[2] + mul_data[1] - mul_data[0];
assign round_data = filter_data[14:6] + filter_data[5];

wire [8:0] pos_cmp, neg_cmp;
assign pos_cmp = round_data + {1'b1, ~threshold};
assign neg_cmp = round_data + {1'b0, threshold};

reg [1:0] quant_i, quant_q;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		quant_i <= 2'b00;
	else if (do_filter_d[0])
		quant_i <= {round_data[8], (round_data[8] ? neg_cmp[8] : (~pos_cmp[8]))};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		quant_q <= 2'b00;
	else if (do_filter_d[1])
		quant_q <= {round_data[8], (round_data[8] ? neg_cmp[8] : (~pos_cmp[8]))};

assign data_valid = do_filter_d[2];
assign data_quant = {quant_i, quant_q};

endmodule
