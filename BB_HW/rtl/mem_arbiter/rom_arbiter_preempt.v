module m_rom_arbiter_preempt #(parameter ADDR_WIDTH = 10, parameter DATA_WIDTH = 32)
(
// connection on port 0
input	rd0,
input preempt0,
input [ADDR_WIDTH-1:0]	addr0,
output accept0,
// connection on port 1
input	rd1,
input preempt1,
input [ADDR_WIDTH-1:0]	addr1,
output accept1,
// connection on port 2
input	rd2,
input preempt2,
input [ADDR_WIDTH-1:0]	addr2,
output accept2,
// connection on port 3
input	rd3,
input preempt3,
input [ADDR_WIDTH-1:0]	addr3,
output accept3,
// read data out
output [DATA_WIDTH-1:0]	data,
// connection on memory
output mem_rd,
output reg [ADDR_WIDTH-1:0]	mem_addr,
input mem_accept,
input [DATA_WIDTH-1:0]	mem_d4rd
);

always @(*)
	if (preempt3 == 1)	// port 3 has priority
	begin
		casez ({rd2, rd1, rd0, rd3})
		4'b???1: mem_addr = addr3;
		4'b??10: mem_addr = addr0;
		4'b?100: mem_addr = addr1;
		4'b1000: mem_addr = addr2;
		default: mem_addr = 0;
		endcase
	end
	else if (preempt3 == 2)	// port 2 has priority
	begin
		casez ({rd1, rd0, rd3, rd2})
		4'b???1: mem_addr = addr2;
		4'b??10: mem_addr = addr3;
		4'b?100: mem_addr = addr0;
		4'b1000: mem_addr = addr1;
		default: mem_addr = 0;
		endcase
	end
	else if (preempt3 == 2)	// port 1 has priority
	begin
		casez ({rd0, rd3, rd2, rd1})
		4'b???1: mem_addr = addr1;
		4'b??10: mem_addr = addr2;
		4'b?100: mem_addr = addr3;
		4'b1000: mem_addr = addr0;
		default: mem_addr = 0;
		endcase
	end
	else	// port 0 or neither has priotity
	begin
		casez ({rd3, rd2, rd1, rd0})
		4'b???1: mem_addr = addr0;
		4'b??10: mem_addr = addr1;
		4'b?100: mem_addr = addr2;
		4'b1000: mem_addr = addr3;
		default: mem_addr = 0;
		endcase
	end
	
reg [3:0] accept_r;
always @(*)
	if (mem_accept == 0)
		accept_r = 4'b0000;
	else if (preempt3 == 1)	// port 3 has priority
	begin
		casez ({rd2, rd1, rd0, rd3})
		4'b???1: accept_r = 4'b1000;
		4'b??10: accept_r = 4'b0001;
		4'b?100: accept_r = 4'b0010;
		4'b1000: accept_r = 4'b0100;
		default: accept_r = 4'b0000;
		endcase
	end
	else if (preempt3 == 2)	// port 2 has priority
	begin
		casez ({rd1, rd0, rd3, rd2})
		4'b???1: accept_r = 4'b0100;
		4'b??10: accept_r = 4'b1000;
		4'b?100: accept_r = 4'b0001;
		4'b1000: accept_r = 4'b0010;
		default: accept_r = 4'b0000;
		endcase
	end
	else if (preempt3 == 2)	// port 1 has priority
	begin
		casez ({rd0, rd3, rd2, rd1})
		4'b???1: accept_r = 4'b0010;
		4'b??10: accept_r = 4'b0100;
		4'b?100: accept_r = 4'b1000;
		4'b1000: accept_r = 4'b0001;
		default: accept_r = 4'b0000;
		endcase
	end
	else	// port 0 or neither has priotity
	begin
		casez ({rd3, rd2, rd1, rd0})
		4'b???1: accept_r = 4'b0001;
		4'b??10: accept_r = 4'b0010;
		4'b?100: accept_r = 4'b0100;
		4'b1000: accept_r = 4'b1000;
		default: accept_r = 4'b0000;
		endcase
	end

assign {accept3, accept2, accept1, accept0} = accept_r;

assign mem_rd = rd0 | rd1 | rd2 | rd3;

assign data = mem_d4rd;

endmodule                 
