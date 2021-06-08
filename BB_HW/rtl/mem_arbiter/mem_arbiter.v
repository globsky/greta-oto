module m_mem_arbiter #(parameter ADDR_WIDTH = 10, parameter DATA_WIDTH = 32)
(
// connection on port 0
input					mem_rd0_i,
input					mem_wr0_i,
input [ADDR_WIDTH-1:0]	mem_addr0_i,
input [DATA_WIDTH-1:0]	mem_d4wt0_i,
output					mem_accept0_o,
output [DATA_WIDTH-1:0]	mem_d4rd0_o,
// connection on port 1
input					mem_rd1_i,
input					mem_wr1_i,
input [ADDR_WIDTH-1:0]	mem_addr1_i,
input [DATA_WIDTH-1:0]	mem_d4wt1_i,
output					mem_accept1_o,
output [DATA_WIDTH-1:0]	mem_d4rd1_o,
// connection on port 2
input					mem_rd2_i,
input					mem_wr2_i,
input [ADDR_WIDTH-1:0]	mem_addr2_i,
input [DATA_WIDTH-1:0]	mem_d4wt2_i,
output					mem_accept2_o,
output [DATA_WIDTH-1:0]	mem_d4rd2_o,
// connection on port 3
input					mem_rd3_i,
input					mem_wr3_i,
input [ADDR_WIDTH-1:0]	mem_addr3_i,
input [DATA_WIDTH-1:0]	mem_d4wt3_i,
output					mem_accept3_o,
output [DATA_WIDTH-1:0]	mem_d4rd3_o,
// connection on memory
output					mem_rd_o,
output					mem_wr_o,
output [ADDR_WIDTH-1:0]	mem_addr_o,
output [DATA_WIDTH-1:0]	mem_d4wt_o,
input [DATA_WIDTH-1:0]	mem_d4rd_i
);

wire [3:0] port_read;
wire [3:0] port_write;

assign port_read = {mem_rd3_i, mem_rd2_i, mem_rd1_i, mem_rd0_i};
assign port_write = {mem_wr3_i, mem_wr2_i, mem_wr1_i, mem_wr0_i};

reg [ADDR_WIDTH-1:0]	mem_addr_r;
always @(*)
	if ((port_read[0] == 1) || (port_write[0] == 1))
		mem_addr_r = mem_addr0_i;
	else if ((port_read[1] == 1) || (port_write[1] == 1))
		mem_addr_r = mem_addr1_i;
	else if ((port_read[2] == 1) || (port_write[2] == 1))
		mem_addr_r = mem_addr2_i;
	else if ((port_read[3] == 1) || (port_write[3] == 1))
		mem_addr_r = mem_addr3_i;
	else
		mem_addr_r = 0;
assign mem_addr_o = mem_addr_r;

reg [3:0] mem_accept_r;
always @(*)
	if ((port_read[0] == 1) || (port_write[0] == 1))
		mem_accept_r = 4'b0001;
	else if ((port_read[1] == 1) || (port_write[1] == 1))
		mem_accept_r = 4'b0010;
	else if ((port_read[2] == 1) || (port_write[2] == 1))
		mem_accept_r = 4'b0100;
	else if ((port_read[3] == 1) || (port_write[3] == 1))
		mem_accept_r = 4'b1000;
	else
		mem_accept_r = 4'b0000;
assign {mem_accept3_o, mem_accept2_o, mem_accept1_o, mem_accept0_o} = mem_accept_r;

reg [DATA_WIDTH-1:0]	mem_d4wt_r;
always @(*)
	casez ({port_read, port_write})
	8'b???????1: mem_d4wt_r = mem_d4wt0_i;
	8'b???0??10: mem_d4wt_r = mem_d4wt1_i;
	8'b??00?100: mem_d4wt_r = mem_d4wt2_i;
	8'b?0001000: mem_d4wt_r = mem_d4wt3_i;
	default: mem_d4wt_r = 0;
	endcase
assign mem_d4wt_o = mem_d4wt_r;

reg mem_wr_r;
always @(*)
	casez ({port_read, port_write})
	8'b???????1: mem_wr_r = 1'b1;
	8'b???0??10: mem_wr_r = 1'b1;
	8'b??00?100: mem_wr_r = 1'b1;
	8'b?0001000: mem_wr_r = 1'b1;
	default: mem_wr_r = 1'b0;
	endcase
assign mem_wr_o = mem_wr_r;

reg mem_rd_r;
always @(*)
	casez ({port_read, port_write})
	8'b???1????: mem_rd_r = 1'b1;
	8'b??10???0: mem_rd_r = 1'b1;
	8'b?100??00: mem_rd_r = 1'b1;
	8'b1000?000: mem_rd_r = 1'b1;
	default: mem_rd_r = 1'b0;
	endcase
assign mem_rd_o = mem_rd_r;

assign mem_d4rd0_o = mem_d4rd_i;
assign mem_d4rd1_o = mem_d4rd_i;
assign mem_d4rd2_o = mem_d4rd_i;
assign mem_d4rd3_o = mem_d4rd_i;

endmodule                 
