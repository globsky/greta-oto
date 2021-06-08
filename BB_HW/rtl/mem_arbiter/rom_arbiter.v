module m_rom_arbiter #(parameter ADDR_WIDTH = 10, parameter DATA_WIDTH = 32)
(
// connection on port 0
input	mem_rd0_i,
input [ADDR_WIDTH-1:0]	mem_addr0_i,
output mem_accept0_o,
// connection on port 1
input	mem_rd1_i,
input [ADDR_WIDTH-1:0]	mem_addr1_i,
output mem_accept1_o,
// connection on port 2
input	mem_rd2_i,
input [ADDR_WIDTH-1:0]	mem_addr2_i,
output mem_accept2_o,
// connection on port 3
input	mem_rd3_i,
input [ADDR_WIDTH-1:0]	mem_addr3_i,
output mem_accept3_o,
// read data out
output [DATA_WIDTH-1:0]	mem_d4rd_o,
// connection on memory
output mem_rd_o,
output [ADDR_WIDTH-1:0]	mem_addr_o,
input mem_accept_i,
input [DATA_WIDTH-1:0]	mem_d4rd_i
);

reg [ADDR_WIDTH-1:0]	mem_addr_r;
always @(*)
	if (mem_rd0_i == 1)
		mem_addr_r = mem_addr0_i;
	else if (mem_rd1_i == 1)
		mem_addr_r = mem_addr1_i;
	else if (mem_rd2_i == 1)
		mem_addr_r = mem_addr2_i;
	else if (mem_rd3_i == 1)
		mem_addr_r = mem_addr3_i;
	else
		mem_addr_r = 0;

assign mem_addr_o = mem_addr_r;

reg [3:0] mem_accept_r;
always @(*)
	if (mem_accept_i == 0)
		mem_accept_r = 4'b0000;
	else if (mem_rd0_i == 1)
		mem_accept_r = 4'b0001;
	else if (mem_rd1_i == 1)
		mem_accept_r = 4'b0010;
	else if (mem_rd2_i == 1)
		mem_accept_r = 4'b0100;
	else if (mem_rd3_i == 1)
		mem_accept_r = 4'b1000;
	else
		mem_accept_r = 4'b0000;

assign {mem_accept3_o, mem_accept2_o, mem_accept1_o, mem_accept0_o} = mem_accept_r;

assign mem_rd_o = mem_rd0_i | mem_rd1_i | mem_rd2_i | mem_rd3_i;

assign mem_d4rd_o = mem_d4rd_i;

endmodule                 
