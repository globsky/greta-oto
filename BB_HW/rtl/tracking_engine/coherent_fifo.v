//----------------------------------------------------------------------
// coherent_fifo.v:
//   Coherent FIFO, pipe to pass correlation result to coherent sum module
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module coherent_fifo #(parameter DATA_WIDTH = 44)
(
// system signals
input clk,   // system clock
input rst_b, // reset signal, low active
// input port
input wr_req,	// write request
input [DATA_WIDTH-1:0] data_in,
// output port
input rd_req,	// read request
output reg [DATA_WIDTH-1:0] data_out,
output empty	// FIFO empty indicator
);

reg [DATA_WIDTH-1:0] mem[7:0];	// depth 8

reg [2:0] rd_addr;
reg [2:0] wr_addr;
reg [3:0] data_cnt;	// range 0~8

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		rd_addr <= 3'd0;
	else if (rd_req)
		rd_addr <= rd_addr + 1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		wr_addr <= 3'd0;
	else if (wr_req)
		wr_addr <= wr_addr + 1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		data_cnt <= 5'b0;
	else if (rd_req & !wr_req)	// only read, data count decrease
		data_cnt <= data_cnt - 1;
	else if (wr_req & !rd_req)	// only write, data count increase
		data_cnt <= data_cnt + 1;

always @(posedge clk)
	if(wr_req)
		mem[wr_addr] <= data_in;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		data_out <= 'h0;
	else if(rd_req)
		data_out <= mem[rd_addr];
 
assign 	empty = (|data_cnt) ? 1'b0 : 1'b1;

endmodule
