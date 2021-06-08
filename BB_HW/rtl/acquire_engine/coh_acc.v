//----------------------------------------------------------------------
// coh_acc.v:
//   coherent accumulation module
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module coh_acc #(parameter COH_DATA_NUMBER = 682)
(
// system signals				
input clk,	//system clock
input rst_b,	//reset signal, low active

// input correlation result
input signed [15:0] cor_result_i,
input signed [15:0] cor_result_q,
input cor_result_valid,

// twiddle factors
input sign_cos1,
input [8:0] mag_cos1,
input sign_sin1,
input [8:0] mag_sin1,
input sign_cos3,
input [8:0] mag_cos3,
input sign_sin3,
input [8:0] mag_sin3,
input sign_cos5,
input [8:0] mag_cos5,
input sign_sin5,
input [8:0] mag_sin5,
input sign_cos7,
input [8:0] mag_cos7,
input sign_sin7,
input [8:0] mag_sin7,

// interface to control the behavior of coherent sum
input first_result,		// activate with the first input match filter result of each segment
input first_segment,	// do not add previous coherent result, equivalent to SegmentCount == 0
input first_acc,			// first time accumulation, do not multiply with twiddle factor, equivalent to CohCount == 0
output read_finish,
output write_finish,

// maximum exponential, value valid at the same cycle last data written into coherent RAM
output reg [3:0] max_exp,

// interface to coherent memory
output rd,
output we,
output [9:0] addr,	// 682 depth
output [191:0] d4wt,	// 24x8 width
input [191:0] d4rd
);

//----------------------------------------------------------
// stages control and counter
//----------------------------------------------------------
reg [5:0] cor_valid_delay;

// cor_valid_delay[0] as read signal
// cor_valid_delay[5] as write signal
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cor_valid_delay <= 9'h00;
	else
		cor_valid_delay <= {cor_valid_delay[4:0], cor_result_valid};

reg [2:0] first_result_counter;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		first_result_counter <= 3'd0;
	else if (cor_result_valid && first_result)
		first_result_counter <= first_result_counter + 1;
	else if (first_result_counter != 0)
		first_result_counter <= first_result_counter + 1;

//----------------------------------------------------------
// read and write coherent RAM
//----------------------------------------------------------
// latch of read data and write data of coherent ram
reg [47:0] freq1_data_in, freq3_data_in, freq5_data_in, freq7_data_in;
reg [47:0] freq1_data_out, freq3_data_out, freq5_data_out, freq7_data_out;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		{freq1_data_in, freq3_data_in, freq5_data_in, freq7_data_in} <= 'd0;
	else if (cor_valid_delay[1])
		{freq1_data_in, freq3_data_in, freq5_data_in, freq7_data_in} <= {d4rd[119:72], d4rd[143:120], d4rd[71:48], d4rd[167:144], d4rd[47:24], d4rd[191:168], d4rd[23:0]};

assign d4wt = {freq7_data_out[47:24], freq5_data_out[47:24], freq3_data_out[47:24], freq1_data_out, freq3_data_out[23:0], freq5_data_out[23:0], freq7_data_out[23:0]};

assign rd = cor_valid_delay[0];
assign we = cor_valid_delay[5];

reg [10:0] addr_read, addr_write;

// read and write address
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		addr_read <= 'd0;
	else if (cor_result_valid && first_result)
		addr_read <= 'd0;
	else if (cor_valid_delay[0])
		addr_read = addr_read + 1;
	else if (addr_read == COH_DATA_NUMBER)
		addr_read = 'd0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		addr_write <= 'd0;
	else if (cor_valid_delay[4] && (first_result_counter == 'd5))
		addr_write <= 'd0;
	else if (cor_valid_delay[5])
		addr_write = addr_write + 1;
	else if (addr_write == COH_DATA_NUMBER)
		addr_write = 'd0;

assign addr = cor_valid_delay[5] ? addr_write : addr_read;
assign read_finish = (addr_read == COH_DATA_NUMBER);
assign write_finish = (addr_write == COH_DATA_NUMBER);

//----------------------------------------------------------
// latch correlation result
//----------------------------------------------------------
reg signed [15:0] cor_input_i;
reg signed [15:0] cor_input_q;
reg cor_input_valid;
reg twiddle_select;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cor_input_i <= 'd0;
	else if (cor_result_valid)
		cor_input_i <= cor_result_i;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cor_input_q <= 'd0;
	else if (cor_result_valid)
		cor_input_q <= cor_result_q;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cor_input_valid <= 'd0;
	else if (cor_result_valid)
		cor_input_valid <= 1'b1;
	else if (twiddle_select)
		cor_input_valid <= 'd0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		twiddle_select <= 'd0;
	else if (cor_input_valid)
		twiddle_select <= ~twiddle_select;

wire [47:0] coh_data_out1, coh_data_out2;

reg [1:0] first_segment_d;
reg [4:0] first_acc_d;
reg [1:0] twiddle_select_d;

// delay 2 clock cycles to match delay of correlation I/Q latch data for first and second frequency pair
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		first_segment_d <= 2'b00;
	else
		first_segment_d <= {first_segment_d[0], first_segment};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		first_acc_d <= 5'h0;
	else
		first_acc_d <= {first_acc_d[3:0], first_acc};

// to match coh_data_in 2 clock cycles delay to correlation data and twiddle factor
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		twiddle_select_d <= 2'b00;
	else
		twiddle_select_d <= {twiddle_select_d[0], twiddle_select};

// do time multiplex coherent acc for frequency pair 1/5
coh_sum_pair u1_coh_sum_pair
(
	.clk              (clk               ),
	.rst_b            (rst_b             ),

	.cor_input_i      (cor_input_i       ),
	.cor_input_q      (cor_input_q       ),
	.cor_input_valid  (cor_input_valid   ),

	.sign_cos         (twiddle_select ? sign_cos5 : sign_cos1),
	.mag_cos          (twiddle_select ? mag_cos5 : mag_cos1  ),
	.sign_sin         (twiddle_select ? sign_sin5 : sign_sin1),
	.mag_sin          (twiddle_select ? mag_sin5 : mag_sin1  ),

	.coh_data_in      (twiddle_select_d[1] ? freq5_data_in : freq1_data_in),
	.coh_data_out     (coh_data_out1     ),

	.first_segment    (first_segment_d[1]),
	.first_acc        (first_acc_d[1]    )
);

// do time multiplex coherent acc for frequency pair 3/7
coh_sum_pair u2_coh_sum_pair
(
	.clk              (clk               ),
	.rst_b            (rst_b             ),

	.cor_input_i      (cor_input_i       ),
	.cor_input_q      (cor_input_q       ),
	.cor_input_valid  (cor_input_valid & ~first_acc),		// mute input for CohCount == 0

	.sign_cos         (twiddle_select ? sign_cos7 : sign_cos3),
	.mag_cos          (twiddle_select ? mag_cos7 : mag_cos3  ),
	.sign_sin         (twiddle_select ? sign_sin7 : sign_sin3),
	.mag_sin          (twiddle_select ? mag_sin7 : mag_sin3  ),

	.coh_data_in      (twiddle_select_d[1] ? freq7_data_in : freq3_data_in),
	.coh_data_out     (coh_data_out2     ),

	.first_segment    (first_segment_d[1]),
	.first_acc        (first_acc_d[1]    )
);

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		freq1_data_out <= 'd0;
	else if (cor_valid_delay[3])
		freq1_data_out <= coh_data_out1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		freq3_data_out <= 'd0;
	else if (cor_valid_delay[3])
		freq3_data_out <= first_acc_d[3] ? coh_data_out1 : coh_data_out2;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		freq5_data_out <= 'd0;
	else if (cor_valid_delay[4])
		freq5_data_out <= coh_data_out1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		freq7_data_out <= 'd0;
	else if (cor_valid_delay[4])
		freq7_data_out <= first_acc_d[4] ? coh_data_out1 : coh_data_out2;

//--------------------------------------------
// calculate max_exp
//--------------------------------------------
wire [3:0] max_exp_pos1, max_exp_neg1, max_exp_pos2, max_exp_neg2;
assign max_exp_pos1 = cor_valid_delay[4] ? freq1_data_out[27:24] : freq5_data_out[27:24];
assign max_exp_neg1 = cor_valid_delay[4] ? freq1_data_out[ 3: 0] : freq5_data_out[ 3: 0];
assign max_exp_pos2 = cor_valid_delay[4] ? freq3_data_out[27:24] : freq7_data_out[27:24];
assign max_exp_neg2 = cor_valid_delay[4] ? freq3_data_out[ 3: 0] : freq7_data_out[ 3: 0];

wire [3:0] max_exp_pos, max_exp_neg, max_exp_local;
assign max_exp_pos = (max_exp_pos1 > max_exp_pos2) ? max_exp_pos1 : max_exp_pos2;
assign max_exp_neg = (max_exp_neg1 > max_exp_neg2) ? max_exp_neg1 : max_exp_neg2;
assign max_exp_local = (max_exp_pos > max_exp_neg) ? max_exp_pos : max_exp_neg;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		max_exp <= 'd0;
	else if (first_acc_d[3] && ~first_acc_d[4])		// clear max_exp on rising edge of first_acc (first coherent round)
		max_exp <= 'd0;
	else if (cor_valid_delay[4] || cor_valid_delay[5])
		max_exp <= (max_exp > max_exp_local) ? max_exp : max_exp_local;

endmodule
