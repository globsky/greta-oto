//----------------------------------------------------------------------
// dpram_full.v:
//   Full dual port SRAM with two read/write port
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module dpram_full #(parameter RAM_SIZE = 1024, parameter ADDR_WIDTH = 10, parameter DATA_WIDTH = 32)
(
input clk_a,
input [ADDR_WIDTH-1:0] addr_a,
input rd_a,
input wr_a,
input [DATA_WIDTH-1:0] wdata_a,
output reg [DATA_WIDTH-1:0] rdata_a,

input clk_b,
input [ADDR_WIDTH-1:0] addr_b,
input rd_b,
input wr_b,
input [DATA_WIDTH-1:0] wdata_b,
output reg [DATA_WIDTH-1:0] rdata_b
);

reg [DATA_WIDTH-1:0] mem [0:RAM_SIZE-1]; // RAM content

// port a
always @(posedge clk_a)
	if(rd_a)
		rdata_a <= mem[addr_a];

always @(posedge clk_a)
	if (wr_a)
		mem[addr_a] <= wdata_a;

// port b
always @(posedge clk_b)
	if(rd_b)
		rdata_b <= mem[addr_b];

always @(posedge clk_b)
	if (wr_b)
		mem[addr_b] <= wdata_b;

endmodule
