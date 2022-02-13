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
input [4:0] state_addr,
output reg [31:0] state_d4wt,

// variables from correlator
input [31:0] prn_state, // PRN state output
input [31:0] prn_count,	// PRN counter output
input [31:0]  carrier_phase,  // carrier phase output
input [31:0]  carrier_count,  // carrier count output
input [31:0]  code_phase,		 // code phase output
input [15:0] dump_count, // dump count output
input [7:0]  jump_count,	// jump count output
input [7:0]  prn_code, 	// prn code output
input [4:0] nh_count, // NH code count output
input [5:0] coherent_count,	// coherent count output
input [4:0] bit_count,		// decode data count output
input [3:0] prn_code2,				// prn code2 output
input [2:0] current_cor,			// current correlator output
input code_sub_phase,  // code sub phase output
input dumping,  // dumping flag output
input overwrite_protect, // overwrite protect flag
input coherent_done,    //coherent data is ready
input [31:0] decode_data,	// decode data output
input [31:0] prn2_state, // PRN2 state output
input [15:0] i_acc,  // acc I channel data output
input [15:0] q_acc  // acc Q channel data output
);

//--------------------------------------------------------------
// select dump state data
//--------------------------------------------------------------
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		state_d4wt <= 32'h0;
	else if (state_addr[4])
		state_d4wt <= {i_acc, q_acc};	
	else begin
		case (state_addr[3:0])
			4'd6 :   state_d4wt <= prn_state;
			4'd7 :   state_d4wt <= prn_count;
			4'd8 :   state_d4wt <= carrier_phase;
			4'd9 :   state_d4wt <= carrier_count;
			4'd10:   state_d4wt <= code_phase;
			4'd11:   state_d4wt <= {dump_count, jump_count, prn_code};
			4'd12:   state_d4wt <= {nh_count, coherent_count, bit_count, prn_code2, 3'b000, code_sub_phase, dumping, current_cor, 2'b00, overwrite_protect, coherent_done};
			4'd13:   state_d4wt <= decode_data;		
			4'd15:   state_d4wt <= prn2_state;
			default: state_d4wt <= 32'h0;
		endcase
	end

endmodule