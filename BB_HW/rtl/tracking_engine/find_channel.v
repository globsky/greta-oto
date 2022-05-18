//----------------------------------------------------------------------
// find_channel.v:
//   Find next 4 channel to do correlation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module find_channel
(
// system signals
input clk,   // system clock
input rst_b, // reset signal, low active

// control signal interface to TE
input latch_enable_channel,
input start_find,
input te_over,
output find_channel_done,
input [31:0] te_channel_enable,
output reg [31:0] channel_remain,

// result of find channel
output reg [3:0] physical_channel_en,
output reg [4:0] logic_channel_index0,
output reg [4:0] logic_channel_index1,
output reg [4:0] logic_channel_index2,
output reg [4:0] logic_channel_index3,
output reg [31:0] logic_channel_mask0,
output reg [31:0] logic_channel_mask1,
output reg [31:0] logic_channel_mask2,
output reg [31:0] logic_channel_mask3
);

wire [4:0] channel_position;
wire channel_active;

least_bit32 u_least_bit32
(
	.bits (channel_remain),
	.position (channel_position),
	.active (channel_active)
);

//----------------------------------------------------------
// state machine
//----------------------------------------------------------
reg [1:0] physical_channel_index;
reg [2:0] cur_state;
reg [2:0] next_state;

// states
localparam
		IDLE         = 3'd0,
		FIND_CHANNEL = 3'd1,
		SET_POSITION = 3'd2,
		CLEAR_BIT    = 3'd3;

wire last_channel;
wire more_channel;
assign last_channel = physical_channel_en[3];
assign more_channel = (channel_active && ~last_channel);

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cur_state <= IDLE;
	else
		cur_state <= next_state;

always @ (*) begin
	case(cur_state)
		IDLE           : next_state = start_find ? FIND_CHANNEL : IDLE;
		FIND_CHANNEL   : next_state = more_channel ? SET_POSITION : IDLE;
		SET_POSITION   : next_state = CLEAR_BIT;
		CLEAR_BIT      : next_state = FIND_CHANNEL;
		default        : next_state = IDLE;
	endcase
end

assign find_channel_done = (cur_state == FIND_CHANNEL) && (~more_channel);

//----------------------------------------------------------
// set bit position and clear remain channel bit
//----------------------------------------------------------
reg [4:0] first_position;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		first_position <= 5'h0;
	else if (cur_state == FIND_CHANNEL)
		first_position <= channel_position;

wire [31:0] position_mask;
assign position_mask = (32'd1 << first_position);

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		channel_remain <= 32'h0;
	else if (latch_enable_channel)
		channel_remain <= te_channel_enable;
	else if (cur_state == CLEAR_BIT)
		channel_remain <= channel_remain & (~position_mask);

//----------------------------------------------------------
// set index and mask
//----------------------------------------------------------
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		physical_channel_index <= 2'b00;
	else if (start_find)
		physical_channel_index <= 2'b00;
	else if (cur_state == CLEAR_BIT)
		physical_channel_index <= physical_channel_index + 1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		physical_channel_en <= 4'h0;
	else if (start_find || te_over)
		physical_channel_en <= 4'h0;
	else if (cur_state == SET_POSITION)
		case (physical_channel_index)
			2'b00: physical_channel_en[0] <= 1'b1;
			2'b01: physical_channel_en[1] <= 1'b1;
			2'b10: physical_channel_en[2] <= 1'b1;
			2'b11: physical_channel_en[3] <= 1'b1;
		endcase

always @(posedge clk or negedge rst_b)
	if (!rst_b) begin
		logic_channel_index0 <= 5'h0;
		logic_channel_index1 <= 5'h0;
		logic_channel_index2 <= 5'h0;
		logic_channel_index3 <= 5'h0;
	end
	else if (start_find) begin
		logic_channel_index0 <= 5'h0;
		logic_channel_index1 <= 5'h0;
		logic_channel_index2 <= 5'h0;
		logic_channel_index3 <= 5'h0;
	end
	else if (cur_state == SET_POSITION)
		case (physical_channel_index)
			2'b00: logic_channel_index0 <= first_position;
			2'b01: logic_channel_index1 <= first_position;
			2'b10: logic_channel_index2 <= first_position;
			2'b11: logic_channel_index3 <= first_position;
		endcase

always @(posedge clk or negedge rst_b)
	if (!rst_b) begin
		logic_channel_mask0 <= 32'h0;
		logic_channel_mask1 <= 32'h0;
		logic_channel_mask2 <= 32'h0;
		logic_channel_mask3 <= 32'h0;
	end
	else if (start_find) begin
		logic_channel_mask0 <= 32'h0;
		logic_channel_mask1 <= 32'h0;
		logic_channel_mask2 <= 32'h0;
		logic_channel_mask3 <= 32'h0;
	end
	else if (cur_state == CLEAR_BIT)
		case (physical_channel_index)
			2'b00: logic_channel_mask0 <= position_mask;
			2'b01: logic_channel_mask1 <= position_mask;
			2'b10: logic_channel_mask2 <= position_mask;
			2'b11: logic_channel_mask3 <= position_mask;
		endcase

endmodule
