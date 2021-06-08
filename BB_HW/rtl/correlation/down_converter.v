//----------------------------------------------------------------------
// down_converter.v:
//   signal down converter module
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module m_down_converter
(
input			clk,
input			rst_b,
// control signal
input [31:0]	carrier_freq,
input [1:0]   pre_shift_bits,
// load register signal
input	carrier_phase_en,
input [31:0] carrier_phase_i,
output [31:0]	carrier_phase_o,
input	carrier_count_en,
input [31:0] carrier_count_i,
output [31:0]	carrier_count_o,
// input sample and enable flag
input [7:0]	input_sample_i,
input			  sample_en_i,
// output result
output reg [5:0]	i_data_o,
output reg [5:0]	q_data_o,
output reg 		    data_en_o
);

// registers
reg [31:0]	carrier_phase_r;
reg [31:0]	carrier_count_r;

wire [32:0]		carrier_phase_next;
wire			pos_overflow_flag;
wire			neg_overflow_flag;
//--------------------------------------------
// calculate next phase and overflow flag
//--------------------------------------------
assign carrier_phase_next = {1'd0, carrier_phase_r} + {1'd0, carrier_freq};
assign pos_overflow_flag = ~carrier_freq[31] & carrier_phase_next[32];
assign neg_overflow_flag = carrier_freq[31] & ~carrier_phase_next[32];

// assign next carrier phase
always @(posedge clk or negedge rst_b)
	if(!rst_b)
		carrier_phase_r <= 'b0;
	else if (carrier_phase_en)
		carrier_phase_r <= carrier_phase_i;
	else if (sample_en_i)
		carrier_phase_r <= carrier_phase_next[31:0];

// accumulate carrier overflow count
always @(posedge clk or negedge rst_b)
	if(!rst_b)
		carrier_count_r <= 'b0;
	else if (carrier_count_en)
		carrier_count_r <= carrier_count_i;
	else if (sample_en_i && pos_overflow_flag)
		carrier_count_r <= carrier_count_r + 1'b1;
	else if  (sample_en_i && neg_overflow_flag)
	    carrier_count_r <= carrier_count_r - 1'b1;

// 4 set of down converter DDC multiply
wire [3:0] sin_value;
wire [3:0] cos_value;
wire sin_sign;
wire cos_sign;

m_ddc_lut ddc_lut
(
  .phase_i (carrier_phase_r[31:26]),
  .sin_value_o (sin_value),
  .cos_value_o (cos_value),
  .sin_sign_o (sin_sign),
  .cos_sign_o (cos_sign)
);

wire [8:0] i_data;
wire [8:0] q_data;
m_complex_mul complex_mul
(
  .sample_data_i (input_sample_i),
  .cos_value_i (cos_value),
  .sin_value_i (sin_value),
  .sin_sign_i (sin_sign),
  .cos_sign_i (cos_sign),
  .i_data_o (i_data),
  .q_data_o (q_data)
);

// latch multiply result
reg [8:0] 	i_data_r;
reg [8:0] 	q_data_r;
reg         data_en_d;
always @(posedge clk or negedge rst_b)			
	if (~rst_b) begin
		i_data_r    <= 'd0;
		q_data_r    <= 'd0;	
		data_en_d   <= 'd0;
	end
	else begin
		i_data_r    <= i_data;
		q_data_r    <= q_data;	
		data_en_d   <= sample_en_i;		
	end

wire [5:0] i_data_shift;
wire [5:0] q_data_shift;

unbias_round_shift i_sat_round
(
	.data_i (i_data_r),
	.shift_bits (pre_shift_bits),
	.data_o (i_data_shift)
);

unbias_round_shift q_sat_round
(
	.data_i (q_data_r),
	.shift_bits (pre_shift_bits),
	.data_o (q_data_shift)
);

always @(posedge clk or negedge rst_b)
	if (!rst_b) 
		data_en_o <= 1'b0;
	else 
		data_en_o <= data_en_d;	
	
always @(posedge clk or negedge rst_b)
	if (!rst_b) begin	
		i_data_o  <= 6'h0;
		q_data_o  <= 6'h0;
	end
	else begin
		i_data_o  <= data_en_d ? i_data_shift : 6'd0;
		q_data_o  <= data_en_d ? q_data_shift : 6'd0 ;
	end 		
	
assign carrier_phase_o = carrier_phase_r;
assign carrier_count_o = carrier_count_r;	

endmodule
