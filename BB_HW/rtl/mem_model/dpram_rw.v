//----------------------------------------------------------------------
// dpram_rw.v:
//   Dual port SRAM with one read port and one write port
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module dpram_rw	#(parameter RAM_SIZE = 1024, parameter ADDR_WIDTH = 10, parameter DATA_WIDTH = 32)
(
input clk,
// write port
input we,
input [ADDR_WIDTH-1:0] write_addr,
input [DATA_WIDTH-1:0] data_in,
// read port
input rd,
input [ADDR_WIDTH-1:0] read_addr,
output reg [DATA_WIDTH-1:0] data_out
);

reg [DATA_WIDTH-1:0] mem[0:RAM_SIZE-1];

always @(posedge clk)
	if(we)
		mem[write_addr] <= data_in;

always @(posedge clk)
	if(rd)
		data_out <= mem[read_addr];

endmodule