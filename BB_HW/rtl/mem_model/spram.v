//----------------------------------------------------------------------
// spram.v:
//   single port synchronize SRAM model
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

`timescale 1ns / 1ns

module spram #(parameter RAM_SIZE = 1024, parameter ADDR_WIDTH = 10, parameter DATA_WIDTH = 32) (
	input clk,
	input en,
	input we,
	input       [ADDR_WIDTH-1:0]      addr,
	input       [DATA_WIDTH-1:0]      wdata,
	output  reg [DATA_WIDTH-1:0]      rdata
);

wire rd = en & (~we);

//
// Generic RAM's registers and wires
//
reg [DATA_WIDTH-1:0]    mem [0:RAM_SIZE-1]; // RAM content

// write operation
always @(posedge clk)
  if (en && we)
    mem[addr] <= #1 wdata;

always @ (posedge clk) begin
    if(rd)
        rdata <= mem[addr];
end

endmodule


