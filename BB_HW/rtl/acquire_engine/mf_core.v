//----------------------------------------------------------------------
// mf_core.v:
//   match filter core
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------
//`define SKIP_MF_CALCULATE		// activate this definition to prevent MF core to do calculation, for debug use only to save time

`define MF_DEPTH 682
`define ADDER_TREE_WIDTH (`MF_DEPTH/2)
`define MF_DELAY 5

module mf_core #(parameter SYNC_SIGNAL_NUMBER = 3)
(
// system signals				
input clk,	//system clock
input rst_b,	//reset signal, low active

// synchronization signal same as MF delay
input [SYNC_SIGNAL_NUMBER-1:0] sync_in,
output [SYNC_SIGNAL_NUMBER-1:0] sync_out,

// compensate value is 192 or 160
input value_select,	// 1: 192, 0: 160

// interface of sample input
input [5:0] sample_i,
input [5:0] sample_q,
input sample_valid,
input preload,	// preload match filter sample buffer, no output
input [`ADDER_TREE_WIDTH-1:0] code,	// code to do match filter

// interface of match filter output
output [15:0] out_i,
output [15:0] out_q,
output reg out_valid
);

reg [11:0] sample_buffer[`MF_DEPTH-2:0];	// sample_buffer[11:6] for I and sample_buffer[5:0] for Q

genvar i_gen;

//----------------------------------------------------------
// synchronization signal
//----------------------------------------------------------
reg [`MF_DELAY-1:0] sync_delay[SYNC_SIGNAL_NUMBER-1:0];

generate
for (i_gen = 0; i_gen < SYNC_SIGNAL_NUMBER; i_gen = i_gen + 1) begin: sync_gen
always @(posedge clk)	// no asynchronize reset needed
	sync_delay[i_gen] <= {sync_delay[i_gen][`MF_DELAY-2:0], sync_in[i_gen]};

assign sync_out[i_gen] = sync_delay[i_gen][`MF_DELAY-1];
end
endgenerate

//----------------------------------------------------------
// move sample input into sample_buffer
//----------------------------------------------------------
wire shift_in_sample;
`ifdef SKIP_MF_CALCULATE
assign shift_in_sample = 1'b0;
`else
assign shift_in_sample = sample_valid;
`endif

always @(posedge clk)	// no asynchronize reset needed
	if (shift_in_sample)
		sample_buffer[0] <= {sample_i, sample_q};	// 6MSB has I and 6LSB has Q

generate
for (i_gen = 1; i_gen < `MF_DEPTH-1; i_gen = i_gen + 1) begin: shift_reg_gen
	always @(posedge clk)	// no asynchronize reset needed
		if (shift_in_sample)
			sample_buffer[i_gen] <= sample_buffer[i_gen-1];
end
endgenerate

//----------------------------------------------------------
// generate calculate signal and iq_select signal
//----------------------------------------------------------
// for each sample_valid, calculate signal will active for 
// 2 cycles if not preload, first cycle for I and secodn for Q
// module to generate sample_valid signal should make sure 
// at least 1 clock cycles interval between sample_valid
reg calculate;
reg iq_select;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		calculate <= 1'b0;
	else if (sample_valid)
		calculate <= ~preload;
	else if (iq_select)
		calculate <= 1'b0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		iq_select <= 1'b0;
	else if (calculate)
		iq_select <= ~iq_select;

//----------------------------------------------------------
// selection for adder tree input and adder tree instances
//----------------------------------------------------------
wire [`ADDER_TREE_WIDTH-1:0] input_select [5:0];

generate
for (i_gen = 0; i_gen < `ADDER_TREE_WIDTH; i_gen = i_gen + 1) begin: sample_select_gen
	assign input_select[5][i_gen] = calculate ? ((iq_select ? sample_buffer[i_gen*2][5] : sample_buffer[i_gen*2][11]) ^ code[i_gen]) : 1'b0;
	assign input_select[4][i_gen] = calculate ? ((iq_select ? sample_buffer[i_gen*2][4] : sample_buffer[i_gen*2][10]) ^ code[i_gen]) : 1'b0;
	assign input_select[3][i_gen] = calculate ? ((iq_select ? sample_buffer[i_gen*2][3] : sample_buffer[i_gen*2][ 9]) ^ code[i_gen]) : 1'b0;
	assign input_select[2][i_gen] = calculate ? ((iq_select ? sample_buffer[i_gen*2][2] : sample_buffer[i_gen*2][ 8]) ^ code[i_gen]) : 1'b0;
	assign input_select[1][i_gen] = calculate ? ((iq_select ? sample_buffer[i_gen*2][1] : sample_buffer[i_gen*2][ 7]) ^ code[i_gen]) : 1'b0;
	assign input_select[0][i_gen] = calculate ? ((iq_select ? sample_buffer[i_gen*2][0] : sample_buffer[i_gen*2][ 6]) ^ code[i_gen]) : 1'b0;
end
endgenerate

wire [8:0] adder_tree_out [5:0];

generate
for (i_gen = 0; i_gen < 6; i_gen = i_gen + 1) begin: adder_tree_gen
	adder_tree u_adder_tree
	(
			.clk   (clk                  ),
			.rst_b (rst_b                ),
			.in    (input_select[i_gen]  ),
			.out   (adder_tree_out[i_gen])
	);
end
endgenerate

//--------------------------------------------
// accumulate the result of adder tree
//--------------------------------------------
reg [1:0] calculate_i, calculate_q;

always @(posedge clk or negedge rst_b)
	if (!rst_b) begin
		calculate_i <= 2'b00;
		calculate_q <= 2'b00;
	end
	else begin
		calculate_i <= {calculate_i[0], calculate & ~iq_select};
		calculate_q <= {calculate_q[0], calculate & iq_select};
	end
	
reg [15:0] result_i, result_q;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		result_i <= 'd0;
	else if (calculate_i[1])
		result_i <= {7'h0, adder_tree_out[0]} + {6'h0, adder_tree_out[1], 1'b0} + {5'h0, adder_tree_out[2], 2'b00}
								+ {4'h0, adder_tree_out[3], 3'h0} + {3'h0, adder_tree_out[4], 4'h0} + {2'b11, ~adder_tree_out[5], 5'h0};


always @(posedge clk or negedge rst_b)
	if (!rst_b)
		result_q <= 'd0;
	else if (calculate_q[1])
		result_q <= {7'h0, adder_tree_out[0]} + {6'h0, adder_tree_out[1], 1'b0} + {5'h0, adder_tree_out[2], 2'b00}
								+ {4'h0, adder_tree_out[3], 3'h0} + {3'h0, adder_tree_out[4], 4'h0} + {2'b11, ~adder_tree_out[5], 5'h0};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		out_valid <= 1'b0;
	else if (calculate_q[1])
		out_valid <= 1'b1;
	else
		out_valid <= 1'b0;

wire [10:0] compensate_i, compensate_q;
assign compensate_i = result_i[15:5] + {10'h3, value_select};	// 160,192+32=192,224
assign compensate_q = result_q[15:5] + {10'h3, value_select};	// 160,192+32=192,224

assign out_i = {compensate_i, result_i[4:0]};
assign out_q = {compensate_q, result_q[4:0]};

endmodule
