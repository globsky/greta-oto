//----------------------------------------------------------------------
// ae_buffer_rw.v:
//   AE buffer read/write interface
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module ae_buffer_rw #(parameter RAM_SIZE = 128*256, ADDR_WIDTH = 15)
(
// system signals				
input	wire		clk,	//system clock
input	wire		rst_b,	//reset signal, low active

// interface to sample write
input	[3:0] sample_in,
input	sample_valid,

// interface to control and status
input	refill,
output reg write_full,
input	[ADDR_WIDTH-9:0] threshold,
output reg reach_threshold,
// interface to sample read
input	address_set,
input	[ADDR_WIDTH+2:0] read_address,	// each DWORD has 8 samples
input	read_next,
output reg [3:0] sample_out,
output sample_ready,
// interface to AE Buffer SRAM
output en,
output we,
output [ADDR_WIDTH-1:0] addr,
output [31:0] d4wt,
input [31:0] d4rd
);

//--------------------------------------------
// sample write
//--------------------------------------------
reg [31:0] sample_shift_in;
reg [ADDR_WIDTH-1:0] write_dword_addr;
reg [2:0] write_sample_addr;
reg sample_write_en;
wire sample_en;
wire [ADDR_WIDTH-1:0] write_dword_addr_next;
wire [2:0] write_sample_addr_next;

assign write_dword_addr_next = write_dword_addr + 1;
assign write_sample_addr_next = write_sample_addr + 1;
assign sample_en = sample_valid && !write_full;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		write_full <= 1'b0;
	else if (refill)
		write_full <= 1'b0;
	else if (sample_write_en && write_dword_addr == RAM_SIZE - 1)
		write_full <= 1'b1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		reach_threshold <= 1'b0;
	else if (refill)
		reach_threshold <= 1'b0;
	else if (write_dword_addr[ADDR_WIDTH-1:8] == threshold)
		reach_threshold <= 1'b1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		sample_shift_in <= 'd0;
	else if (sample_en)
		sample_shift_in <= {sample_shift_in[27:0], sample_in};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		write_sample_addr <= 'd0;
	else if (refill)
		write_sample_addr <= 'd0;
	else if (sample_en)
		write_sample_addr <= write_sample_addr_next;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		write_dword_addr <= 'd0;
	else if (refill)
		write_dword_addr <= 'd0;
	else if (sample_write_en)
		write_dword_addr <= write_dword_addr_next;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		sample_write_en <= 1'b0;
	else if (sample_en && (write_sample_addr == 3'd7))
		sample_write_en <= 1'b1;
	else
		sample_write_en <= 1'b0;

//--------------------------------------------
// read address
//--------------------------------------------
reg [ADDR_WIDTH-1:0] read_dword_addr;
reg [2:0] read_sample_addr;
reg load_next;
reg preload_empty;
reg output_empty;
reg sample_read_en;
wire [2:0] read_sample_addr_next;
wire read_next_dword;

assign read_sample_addr_next = read_sample_addr + 1;
assign read_next_dword = read_next && (read_sample_addr == 3'd7);

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		read_sample_addr <= 'd0;
	else if (address_set)
		read_sample_addr <= read_address[2:0];
	else if (read_next)
		read_sample_addr <= read_sample_addr_next;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		read_dword_addr <= 'd0;
	else if (address_set)
		read_dword_addr <= read_address[ADDR_WIDTH+2:3];
	else if (load_next && !sample_write_en)
		read_dword_addr <= read_dword_addr + 1;

//--------------------------------------------
// sample load
//--------------------------------------------
reg [31:0] preload_sample;
reg [31:0] output_sample;

// assert one cycle when preload empty is high
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		load_next <= 'd0;
	else if (load_next && !sample_write_en)
		load_next <= 'd0;
	else if ((address_set || preload_empty || output_empty) && !sample_read_en)
		load_next <= 'd1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		preload_empty <= 'd1;
	else if (address_set)
		preload_empty <= 'd1;
	else if (sample_read_en)
		preload_empty <= 'd0;
	else if (output_empty || read_next_dword)
		preload_empty <= 'd1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		output_empty <= 'd1;
	else if (address_set)
		output_empty <= 'd1;
	else if (output_empty || read_next_dword)
		output_empty <= preload_empty;
	else
		output_empty <= 'd0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		preload_sample <= 'd0;
	else if (sample_read_en)
		preload_sample <= d4rd;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		output_sample <= 'd0;
	else if (output_empty || read_next_dword)
		output_sample <= preload_sample;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		sample_read_en <= 1'b0;
	else
		sample_read_en <= load_next;
	
// sample output
always @(*)
	case(read_sample_addr)
	3'd0:
		sample_out = output_sample[31:28];
	3'd1:
		sample_out = output_sample[27:24];
	3'd2:
		sample_out = output_sample[23:20];
	3'd3:
		sample_out = output_sample[19:16];
	3'd4:
		sample_out = output_sample[15:12];
	3'd5:
		sample_out = output_sample[11:8];
	3'd6:
		sample_out = output_sample[7:4];
	3'd7:
		sample_out = output_sample[3:0];
	default:
		sample_out = 4'd0;
	endcase

assign sample_ready = ~output_empty && (!preload_empty || ~read_sample_addr[2]);	// output_sample not empty and either preload is full or output_sample has at least 4 output samples

assign en = load_next | sample_write_en;
//assign en = address_set | preload_empty | sample_write_en;
assign we = sample_write_en;
assign addr = sample_write_en ? write_dword_addr : read_dword_addr;
//assign addr = sample_write_en ? write_dword_addr : address_set ? read_address[ADDR_WIDTH+2:3] : read_dword_addr;
assign d4wt = sample_shift_in;

endmodule
