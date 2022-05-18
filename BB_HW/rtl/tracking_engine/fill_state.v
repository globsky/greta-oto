//----------------------------------------------------------------------
// fill_state.v:
//   Fill state of one physical channel, latch unchanged register
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module fill_state
(
// system signals
input clk,   // system clock
input rst_b, // reset signal, low active

// control signal
input fill_enable,
input state_rd,
input [4:0] state_addr,
input [31:0] state_d4rd,

// channel control parameters
output reg [31:0] carrier_freq,	// carrier frequence
output reg [31:0] code_freq,		// code frequence
output reg [1:0]  pre_shift_bits,		// pre shift bits
output reg [1:0]  post_shift_bits,  // post shift bits
output reg enable_boc,			// enable BOC
output reg data_in_q,			// data in Q branch
output reg enable_2nd_prn,	// enable second prn
output reg [1:0]  decode_bit,	// indicator of decoded data bit number
output reg [1:0]  narrow_factor,	// narrow correlator factor
output reg [4:0]  bit_length,		// bit length in ms
output reg [5:0]	coherent_number,  // coherent number
output reg [24:0] nh_code,  // NH code
output reg [4:0] 	nh_length,  // NH data lenth
output reg [19:0] dump_length,  	// dump data length (16LSB for DumpLength, or 20bit NH Code2)
output reg [31:0] prn_config,			// PRN config
output reg [31:0] prn2_config,		// PRN2 config

// channel variables load enable
output reg prn_state_en,  // PRN state load enable
output reg prn_count_en,	// PRN count load enable
output reg carrier_phase_en, // carrier phase load enable
output reg carrier_count_en, // carrier count load enable
output reg code_phase_en,  // code phase load enable
output reg prn_code_load_en,  // load enable
output reg corr_state_load_en, // load enable
output reg decode_data_en,		// load enable
output reg prn2_state_en,  // PRN2 state load enable
output reg acc_en // acc result load enable
);

//----------------------------------------------------------
// load enable signal generation
//----------------------------------------------------------
reg carrier_freq_en;
reg code_freq_en;
reg cor_config_en;
reg nh_config_en;
reg dump_length_en;
reg prn_config_en;
reg prn_config2_en;


always @(posedge clk or negedge rst_b)
	if (~rst_b) begin
		carrier_freq_en    <= 1'b0;
		code_freq_en       <= 1'b0;
		cor_config_en      <= 1'b0;
		nh_config_en       <= 1'b0;
		dump_length_en     <= 1'b0;
		prn_config_en      <= 1'b0;
		prn_state_en       <= 1'b0;
		prn_count_en       <= 1'b0;
		carrier_phase_en   <= 1'b0;
		carrier_count_en   <= 1'b0;
		code_phase_en      <= 1'b0;
		prn_code_load_en   <= 1'b0;
		corr_state_load_en <= 1'b0;
		decode_data_en     <= 1'b0;
		prn_config2_en     <= 1'b0;
		prn2_state_en      <= 1'b0;
	end
	else if (fill_enable & state_rd) begin
		carrier_freq_en    <= (state_addr ==  'd0) ? 1'b1 : 1'b0;
		code_freq_en       <= (state_addr ==  'd1) ? 1'b1 : 1'b0;
		cor_config_en      <= (state_addr ==  'd2) ? 1'b1 : 1'b0;
		nh_config_en       <= (state_addr ==  'd3) ? 1'b1 : 1'b0;
		dump_length_en     <= (state_addr ==  'd4) ? 1'b1 : 1'b0;
		prn_config_en      <= (state_addr ==  'd5) ? 1'b1 : 1'b0;
		prn_state_en       <= (state_addr ==  'd6) ? 1'b1 : 1'b0;
		prn_count_en       <= (state_addr ==  'd7) ? 1'b1 : 1'b0;
		carrier_phase_en   <= (state_addr ==  'd8) ? 1'b1 : 1'b0;
		carrier_count_en   <= (state_addr ==  'd9) ? 1'b1 : 1'b0;
		code_phase_en      <= (state_addr == 'd10) ? 1'b1 : 1'b0;
		prn_code_load_en   <= (state_addr == 'd11) ? 1'b1 : 1'b0;
		corr_state_load_en <= (state_addr == 'd12) ? 1'b1 : 1'b0;
		decode_data_en     <= (state_addr == 'd13) ? 1'b1 : 1'b0;
		prn_config2_en     <= (state_addr == 'd14) ? 1'b1 : 1'b0;
		prn2_state_en      <= (state_addr == 'd15) ? 1'b1 : 1'b0;
	end

always @(posedge clk or negedge rst_b)
	if (~rst_b)
		acc_en <= 1'b0;
	else if (fill_enable & state_rd) begin
		if (state_addr == 'd15)
			acc_en <= 1'b1;
		else
			acc_en <= 1'b0;
	end

//----------------------------------------------------------
// latch control parameters
//----------------------------------------------------------
always @(posedge clk or negedge rst_b)
	if (~rst_b) begin
		carrier_freq    <= 'd0;
		code_freq       <= 'd0;
		pre_shift_bits  <= 'd0;
		post_shift_bits <= 'd0;
		data_in_q       <= 'd0;
		enable_2nd_prn  <= 'd0;
		enable_boc      <= 'd0;
		decode_bit      <= 'd0;
		narrow_factor   <= 'd0;
		bit_length      <= 'd0;
		coherent_number <= 'd0;
		nh_code         <= 'd0;
		nh_length       <= 'd0;
		dump_length     <= 'd0;
		prn_config      <= 'd0;
		prn2_config     <= 'd0;
	end
	else begin
		case (1'b1)
			carrier_freq_en: carrier_freq <= state_d4rd;
			code_freq_en:    code_freq <= state_d4rd;
			cor_config_en: begin
				pre_shift_bits  <= state_d4rd[1:0];
				post_shift_bits <= state_d4rd[3:2];
				data_in_q       <= state_d4rd[5];
				enable_2nd_prn  <= state_d4rd[6];
				enable_boc      <= state_d4rd[7];
				decode_bit      <= state_d4rd[9:8];
	      narrow_factor   <= state_d4rd[11:10];
				bit_length      <= state_d4rd[20:16];
				coherent_number <= state_d4rd[26:21];
			end
			nh_config_en: begin
				nh_code         <= state_d4rd[24:0];
				nh_length       <= state_d4rd[31:27];
			end
			dump_length_en:	dump_length <= state_d4rd[19:0];
			prn_config_en:  prn_config  <= state_d4rd;
			prn_config2_en: prn2_config <= state_d4rd;
		endcase
	end

endmodule
