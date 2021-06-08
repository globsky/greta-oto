//----------------------------------------------------------------------
// fill_dump_ctrl.v:
//   Fill and dump state control logic
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module fill_dump_ctrl
(
// system signals
input clk,   // system clock
input rst_b, // reset signal, low active

// result of find channel
input [3:0] physical_channel_en,
input [4:0] logic_channel_index0,
input [4:0] logic_channel_index1,
input [4:0] logic_channel_index2,
input [4:0] logic_channel_index3,

// input control signal
input fill_start,
input dump_start,

// output control signal
output [1:0] physical_channel_index,
output fill_state_done,
output dump_state_done,
output state_rd,	// state buffer read signal
output state_wr,	// state buffer write signal
output [9:0] state_addr
);

//----------------------------------------------------------
// state machine
//----------------------------------------------------------
reg [2:0] cur_channel_index;
reg [2:0] cur_state;
reg [2:0] next_state;
reg [4:0] fill_dump_addr;
reg channel_enable;

// states
localparam
		IDLE           = 2'd0,
		SELECT_CHANNEL = 2'd1,
		FILL_DUMP      = 2'd2;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cur_state <= IDLE;
	else
		cur_state <= next_state;

always @ (*) begin
	case(cur_state)
		IDLE           : next_state = (fill_start | dump_start) ? SELECT_CHANNEL : IDLE;
		SELECT_CHANNEL : next_state = channel_enable ? FILL_DUMP : IDLE;
		FILL_DUMP      : next_state = (fill_dump_addr == 5'd23) ? SELECT_CHANNEL : FILL_DUMP;
		default        : next_state = IDLE;
	endcase
end

//----------------------------------------------------------
// address control
//----------------------------------------------------------
reg [4:0] logic_channel_index;
reg fill_dump_sel;

always @(*)
	case (cur_channel_index)
		3'b000 : channel_enable = physical_channel_en[0];
		3'b001 : channel_enable = physical_channel_en[1];
		3'b010 : channel_enable = physical_channel_en[2];
		3'b011 : channel_enable = physical_channel_en[3];
		default: channel_enable = 0;
	endcase

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		logic_channel_index <= 5'h0;
	else if (cur_state == SELECT_CHANNEL)
	begin
		if (cur_channel_index == 3'b000)
			logic_channel_index <= logic_channel_index0;
		else if (cur_channel_index == 3'b001)
			logic_channel_index <= logic_channel_index1;
		else if (cur_channel_index == 3'b010)
			logic_channel_index <= logic_channel_index2;
		else if (cur_channel_index == 3'b011)
			logic_channel_index <= logic_channel_index3;
	end

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cur_channel_index <= 3'd0;
	else if (fill_start | dump_start)
		cur_channel_index <= 3'd0;
	else if (fill_dump_addr == 5'd23)
		cur_channel_index <= cur_channel_index + 1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		fill_dump_sel <= 1'b0;
	else if (fill_start)
		fill_dump_sel <= 1'b0;
	else if (dump_start)	
		fill_dump_sel <= 1'b1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		fill_dump_addr <= 5'd0;
	else if (cur_state == SELECT_CHANNEL)
		fill_dump_addr <= fill_dump_sel ? 5'd6 : 5'd0;
	else if (cur_state == FILL_DUMP)
		fill_dump_addr <= fill_dump_addr + 1;

//----------------------------------------------------------
// output signal
//----------------------------------------------------------
assign physical_channel_index = cur_channel_index[1:0];
assign fill_state_done = (cur_state == SELECT_CHANNEL) && (~channel_enable) && (~fill_dump_sel);
assign dump_state_done = (cur_state == SELECT_CHANNEL) && (~channel_enable) && fill_dump_sel;
assign state_rd = (cur_state == FILL_DUMP) && (~fill_dump_sel);
assign state_wr = (cur_state == FILL_DUMP) && fill_dump_sel && (~(fill_dump_addr == 5'd14));
assign state_addr = {logic_channel_index, fill_dump_addr};

endmodule
