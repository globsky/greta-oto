//----------------------------------------------------------------------
// prn_general.v:
//   General PRN code generation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module m_prn_general #(parameter G12_LENGTH = 14, parameter GLOBAL_LENGTH = 18)
(
// system signals
input	clk,
input	rst_b,
// polynimial and length
input	serial_parallel,
input [G12_LENGTH-1:0]		g1_poly,
input [G12_LENGTH-1:0]		g2_poly,
input [31:0]              gx_length,
// initial state
input [G12_LENGTH-1:0]		g1_init_state,
input [G12_LENGTH-1:0]		g2_init_state,
// load register signal
input [G12_LENGTH-1:0]		g1_state_i,
input [G12_LENGTH-1:0]		g2_state_i,
input [31:0]              gx_count_i,
input	state_load,
input phase_load,
input phase_init,
// shift control
input						shift_code,
// output state and count
output [G12_LENGTH-1:0]		g1_state_o,
output [G12_LENGTH-1:0]		g2_state_o,
output [31:0]             gx_count_o,
output prn_reset,
// PRN code output
output prn_code
);

// registers
reg [G12_LENGTH-1:0]		g1_state_r;
reg [G12_LENGTH-1:0]		g2_state_r;
reg [G12_LENGTH-1:0]		g1_count_r;
reg [GLOBAL_LENGTH-1:0]		global_count_r;

wire [G12_LENGTH-1:0] g1_length;
wire [GLOBAL_LENGTH-1:0] global_length;

assign g1_length = gx_length[G12_LENGTH-1:0];
assign global_length = serial_parallel ? gx_length[GLOBAL_LENGTH-1:0] : gx_length[31:G12_LENGTH];

wire	g1_feedback;
wire	g2_feedback;
wire	g1_shift_in;
wire	g2_shift_in;
wire	[G12_LENGTH:0]	g1_count_next;
wire	[GLOBAL_LENGTH-1:0]	global_count_next;
wire	g1_identical;
wire	global_identical;
wire	reset_g1;
wire	reset_global;

assign g1_feedback = ^(g1_state_r & g1_poly);
assign g2_feedback = ^(g2_state_r & g2_poly);
assign g1_shift_in = serial_parallel ? (g1_feedback ^ g2_feedback) : g1_feedback;
assign g2_shift_in = serial_parallel ? g1_state_r[G12_LENGTH-1] : g2_feedback;
assign g1_count_next = {1'b0, g1_count_r} + 1'b1;
assign global_count_next = global_count_r + 1'b1;
assign g1_identical = (g1_count_next == g1_length);
assign global_identical = (global_count_next == global_length) ;
assign reset_g1 = serial_parallel ? 1'b0 : g1_identical;
assign reset_global = global_identical ;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		g1_state_r <= 0;
	else if (phase_init)
		g1_state_r <= g1_init_state;
	else if (state_load)
		g1_state_r <= g1_state_i;
	else if (shift_code)
	begin
		if (reset_global || reset_g1)
			g1_state_r <= g1_init_state;
		else
			g1_state_r <= {g1_state_r[G12_LENGTH-2:0], g1_shift_in};
	end

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		g2_state_r <= 0;
	else if (phase_init)
		g2_state_r <= g2_init_state;
	else if (state_load)
		g2_state_r <= g2_state_i;
	else if (shift_code)
	begin
		if (reset_global)
			g2_state_r <= g2_init_state;
		else
			g2_state_r <= {g2_state_r[G12_LENGTH-2:0], g2_shift_in};
	end

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		g1_count_r <= 0;
	else if (phase_init)
		g1_count_r <= 0;
	else if (phase_load)
		g1_count_r <= gx_count_i[G12_LENGTH-1:0];
	else if (shift_code)
	begin
		if (reset_global || reset_g1)
			g1_count_r <= 0;
		else
			g1_count_r <= g1_count_next[G12_LENGTH-1:0];
	end

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		global_count_r <= 0;
	else if (phase_init)
		global_count_r <= 0;
	else if (phase_load)
		global_count_r <= serial_parallel ? gx_count_i[GLOBAL_LENGTH-1:0] : gx_count_i[31:G12_LENGTH];
	else if (shift_code)
	begin
		if (reset_global)
			global_count_r <= 0;
		else
			global_count_r <= global_count_next[GLOBAL_LENGTH-1:0];
	end

assign g1_state_o = g1_state_r;
assign g2_state_o = g2_state_r;
assign g1_count_o = g1_count_r;
assign gx_count_o = serial_parallel ? global_count_r : {global_count_r[31-G12_LENGTH:0], g1_count_r};
assign prn_code = serial_parallel ? g2_state_r[G12_LENGTH-1] : (g1_state_r[G12_LENGTH-1] ^ g2_state_r[G12_LENGTH-1]);
assign prn_reset = reset_global;

endmodule
