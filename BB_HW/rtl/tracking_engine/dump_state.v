//----------------------------------------------------------------------
// dump_state.v:
//   Dump state of all physical channels
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module dump_state
(
// system signals
input clk,   // system clock
input rst_b, // reset signal, low active

// dump signal
input [1:0] physical_channel_index,
input [4:0] state_addr,
output reg [31:0] state_d4wt,

// variables from correlator 0
input [31:0] prn_state_0, // PRN state output
input [31:0] prn_count_0,	// PRN counter output
input [31:0]  carrier_phase_0,  // carrier phase output
input [31:0]  carrier_count_0,  // carrier count output
input [31:0]  code_phase_0,		 // code phase output
input [15:0] dump_count_0, // dump count output
input [7:0]  jump_count_0,	// jump count output
input [7:0]  prn_code_0, 	// prn code output
input [4:0] nh_count_0, // NH code count output
input [5:0] coherent_count_0,	// coherent count output
input [4:0] bit_count_0,		// decode data count output
input [3:0] prn_code2_0,				// prn code2 output
input [2:0] current_cor_0,			// current correlator output
input code_sub_phase_0,  // code sub phase output
input dumping_0,  // dumping flag output
input overwrite_protect_0, // overwrite protect flag
input coherent_done_0,    //coherent data is ready
input [31:0] decode_data_0,	// decode data output
input [31:0] prn2_state_0, // PRN2 state output
input [15:0] i_acc_0,  // acc I channel data output
input [15:0] q_acc_0,  // acc Q channel data output

// variables from correlator 1
input [31:0] prn_state_1, // PRN state output
input [31:0] prn_count_1,	// PRN counter output
input [31:0]  carrier_phase_1,  // carrier phase output
input [31:0]  carrier_count_1,  // carrier count output
input [31:0]  code_phase_1,		 // code phase output
input [15:0] dump_count_1, // dump count output
input [7:0]  jump_count_1,	// jump count output
input [7:0]  prn_code_1, 	// prn code output
input [4:0] nh_count_1, // NH code count output
input [5:0] coherent_count_1,	// coherent count output
input [4:0] bit_count_1,		// decode data count output
input [3:0] prn_code2_1,				// prn code2 output
input [2:0] current_cor_1,			// current correlator output
input code_sub_phase_1,  // code sub phase output
input dumping_1,  // dumping flag output
input overwrite_protect_1, // overwrite protect flag
input coherent_done_1,    //coherent data is ready
input [31:0] decode_data_1,	// decode data output
input [31:0] prn2_state_1, // PRN2 state output
input [15:0] i_acc_1,  // acc I channel data output
input [15:0] q_acc_1,  // acc Q channel data output

// variables from correlator 2
input [31:0] prn_state_2, // PRN state output
input [31:0] prn_count_2,	// PRN counter output
input [31:0]  carrier_phase_2,  // carrier phase output
input [31:0]  carrier_count_2,  // carrier count output
input [31:0]  code_phase_2,		 // code phase output
input [15:0] dump_count_2, // dump count output
input [7:0]  jump_count_2,	// jump count output
input [7:0]  prn_code_2, 	// prn code output
input [4:0] nh_count_2, // NH code count output
input [5:0] coherent_count_2,	// coherent count output
input [4:0] bit_count_2,		// decode data count output
input [3:0] prn_code2_2,				// prn code2 output
input [2:0] current_cor_2,			// current correlator output
input code_sub_phase_2,  // code sub phase output
input dumping_2,  // dumping flag output
input overwrite_protect_2, // overwrite protect flag
input coherent_done_2,    //coherent data is ready
input [31:0] decode_data_2,	// decode data output
input [31:0] prn2_state_2, // PRN2 state output
input [15:0] i_acc_2,  // acc I channel data output
input [15:0] q_acc_2,  // acc Q channel data output

// variables from correlator 3
input [31:0] prn_state_3, // PRN state output
input [31:0] prn_count_3,	// PRN counter output
input [31:0]  carrier_phase_3,  // carrier phase output
input [31:0]  carrier_count_3,  // carrier count output
input [31:0]  code_phase_3,		 // code phase output
input [15:0] dump_count_3, // dump count output
input [7:0]  jump_count_3,	// jump count output
input [7:0]  prn_code_3, 	// prn code output
input [4:0] nh_count_3, // NH code count output
input [5:0] coherent_count_3,	// coherent count output
input [4:0] bit_count_3,		// decode data count output
input [3:0] prn_code2_3,				// prn code2 output
input [2:0] current_cor_3,			// current correlator output
input code_sub_phase_3,  // code sub phase output
input dumping_3,  // dumping flag output
input overwrite_protect_3, // overwrite protect flag
input coherent_done_3,    //coherent data is ready
input [31:0] decode_data_3,	// decode data output
input [31:0] prn2_state_3, // PRN2 state output
input [15:0] i_acc_3,  // acc I channel data output
input [15:0] q_acc_3   // acc Q channel data output
);

//--------------------------------------------------------------
// multiplex of input variables
//--------------------------------------------------------------
wire [31:0] prn_state;
multiplex_4_1 #(32) multiplex_prn_state
(
	.data_in_0  (prn_state_0),
	.data_in_1  (prn_state_1),
	.data_in_2  (prn_state_2),
	.data_in_3  (prn_state_3),
	.data_sel   (physical_channel_index),
	.data_out   (prn_state)
);

wire [31:0] prn_count;
multiplex_4_1 #(32) multiplex_prn_count
(
	.data_in_0  (prn_count_0),
	.data_in_1  (prn_count_1),
	.data_in_2  (prn_count_2),
	.data_in_3  (prn_count_3),
	.data_sel   (physical_channel_index),
	.data_out   (prn_count)
);

wire [31:0] carrier_phase;
multiplex_4_1 #(32) multiplex_carrier_phase
(
	.data_in_0  (carrier_phase_0),
	.data_in_1  (carrier_phase_1),
	.data_in_2  (carrier_phase_2),
	.data_in_3  (carrier_phase_3),
	.data_sel   (physical_channel_index),
	.data_out   (carrier_phase)
);

wire [31:0] carrier_count;
multiplex_4_1 #(32) multiplex_carrier_count
(
	.data_in_0  (carrier_count_0),
	.data_in_1  (carrier_count_1),
	.data_in_2  (carrier_count_2),
	.data_in_3  (carrier_count_3),
	.data_sel   (physical_channel_index),
	.data_out   (carrier_count)
);

wire [31:0] code_phase;
multiplex_4_1 #(32) multiplex_code_phase
(
	.data_in_0  (code_phase_0),
	.data_in_1  (code_phase_1),
	.data_in_2  (code_phase_2),
	.data_in_3  (code_phase_3),
	.data_sel   (physical_channel_index),
	.data_out   (code_phase)
);

wire [31:0] prn_code;
multiplex_4_1 #(32) multiplex_prn_code
(
	.data_in_0  ({dump_count_0, jump_count_0, prn_code_0}),
	.data_in_1  ({dump_count_1, jump_count_1, prn_code_1}),
	.data_in_2  ({dump_count_2, jump_count_2, prn_code_2}),
	.data_in_3  ({dump_count_3, jump_count_3, prn_code_3}),
	.data_sel   (physical_channel_index),
	.data_out   (prn_code)
);

wire [31:0] cor_state;
multiplex_4_1 #(32) multiplex_cor_state
(
	.data_in_0  ({nh_count_0, coherent_count_0, bit_count_0, prn_code2_0, 3'b000, code_sub_phase_0, dumping_0, current_cor_0, 2'b00, overwrite_protect_0, coherent_done_0}),
	.data_in_1  ({nh_count_1, coherent_count_1, bit_count_1, prn_code2_1, 3'b000, code_sub_phase_1, dumping_1, current_cor_1, 2'b00, overwrite_protect_1, coherent_done_1}),
	.data_in_2  ({nh_count_2, coherent_count_2, bit_count_2, prn_code2_2, 3'b000, code_sub_phase_2, dumping_2, current_cor_2, 2'b00, overwrite_protect_2, coherent_done_2}),
	.data_in_3  ({nh_count_3, coherent_count_3, bit_count_3, prn_code2_3, 3'b000, code_sub_phase_3, dumping_3, current_cor_3, 2'b00, overwrite_protect_3, coherent_done_3}),
	.data_sel   (physical_channel_index),
	.data_out   (cor_state)
);

wire [31:0] decode_data;
multiplex_4_1 #(32) multiplex_decode_data
(
	.data_in_0  (decode_data_0),
	.data_in_1  (decode_data_1),
	.data_in_2  (decode_data_2),
	.data_in_3  (decode_data_3),
	.data_sel   (physical_channel_index),
	.data_out   (decode_data)
);

wire [31:0] prn2_state;
multiplex_4_1 #(32) multiplex_prn2_state
(
	.data_in_0  (prn2_state_0),
	.data_in_1  (prn2_state_1),
	.data_in_2  (prn2_state_2),
	.data_in_3  (prn2_state_3),
	.data_sel   (physical_channel_index),
	.data_out   (prn2_state)
);

wire [31:0] iq_acc;
multiplex_4_1 #(32) multiplex_acc
(
	.data_in_0  ({i_acc_0,q_acc_0}),
	.data_in_1  ({i_acc_1,q_acc_1}),
	.data_in_2  ({i_acc_2,q_acc_2}),
	.data_in_3  ({i_acc_3,q_acc_3}),
	.data_sel   (physical_channel_index),
	.data_out   (iq_acc)
);

//--------------------------------------------------------------
// select dump state data
//--------------------------------------------------------------
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		state_d4wt <= 32'h0;
	else if (state_addr[4])
		state_d4wt <= iq_acc;	
	else begin
		case (state_addr[3:0])
			4'd6 :   state_d4wt <= prn_state;
			4'd7 :   state_d4wt <= prn_count;
			4'd8 :   state_d4wt <= carrier_phase;
			4'd9 :   state_d4wt <= carrier_count;
			4'd10:   state_d4wt <= code_phase;
			4'd11:   state_d4wt <= prn_code;
			4'd12:   state_d4wt <= cor_state;
			4'd13:   state_d4wt <= decode_data;		
			4'd15:   state_d4wt <= prn2_state;
			default: state_d4wt <= 32'h0;
		endcase
	end

endmodule