//----------------------------------------------------------------------
// sprom.v:
//   single port synchronize ROM model
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module sprom #(parameter ROM_SIZE = 1024, parameter ADDR_WIDTH = 10, parameter DATA_WIDTH = 32) (
	input clk,
	input rd,
	input       [ADDR_WIDTH-1:0]      addr,
	output  reg [DATA_WIDTH-1:0]      rdata
);

// content to be initialized in other module
reg [DATA_WIDTH-1:0]    mem [0:ROM_SIZE-1]; // ROM content

always @ (posedge clk) begin
	if(rd)
		rdata <= mem[addr];
end

endmodule


