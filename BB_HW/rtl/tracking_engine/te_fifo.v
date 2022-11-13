//----------------------------------------------------------------------
// te_fifo.v:
//   TE FIFO module
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module te_fifo #(parameter FIFO_SIZE = 10240, parameter ADDR_WIDTH = 14, parameter DATA_WIDTH = 8, parameter TRIGGER_WIDTH = 3)
(
// system signals
input clk,   // system clock
input rst_b, // reset signal, low active

// control signals
input te_enable,	// enable flag from tracking engines
input fifo_clear,	// clear FIFO from external
input [TRIGGER_WIDTH-1:0] fifo_trigger_in,	// trigger signal from other FIFO
output fifo_trigger_out,	// trigger signal to other FIFO

// latch signals				
input cpu_latch,		//latch write address by cpu
input em_latch,			//latch write address by external event message
input pps_latch,		//latch write address by pps
input ae_latch,			//latch write address for ae

// input sample data
input sample_valid,	// sample data valid signal
input [DATA_WIDTH-1:0]	sample_data,	// sample data

// interface to TE
output fifo_ready,			// fifo data ready
output reg fifo_last_data,	// indicate the last data of FIFO
input fifo_read,				// read FIFO command, indicate TE ready to accept data
input fifo_rewind,			// FIFO read address rewind
input fifo_skip,				// FIFO skip one block
output reg fifo_data_valid,	// FIFO data valid signal
output reg [DATA_WIDTH-1:0] fifo_data,	// data output to TE

// CPU register read/write interface
input fifo_cs,	// FIFO cs signal
input fifo_wr,	// FIFO write valid
input fifo_rd,	// FIFO read valid
input [4:0] fifo_addr,	// FIFO registers address offset
input [31:0] fifo_d4wt,	// FIFO registers write data
output reg [31:0] fifo_d4rd	// FIFO registers read data
);

localparam CLK_COUNT_WIDTH = 20 - ADDR_WIDTH;		// bit width for clk count;
localparam DATA_COUNT_WIDTH = ADDR_WIDTH + 1;		// bit width for data count
localparam GUARD_WIDTH = ADDR_WIDTH - 8;

//----------------------------------------------------------
// internal registers
//----------------------------------------------------------
reg [TRIGGER_WIDTH-1:0] trigger_source;
reg dummy_write;
reg fifo_clear_bit;
reg wait_trigger_bit;
reg wait_trigger;

wire guard_alarm_flag;
reg clear_overflow_flag;
reg overflow_flag;

reg [GUARD_WIDTH-1:0] fifo_guard;
reg [ADDR_WIDTH-1:0] read_addr;
reg [ADDR_WIDTH-1:0] write_addr;
reg [ADDR_WIDTH-1:0] block_size;
reg [11:0] write_addr_round;
reg [DATA_COUNT_WIDTH-1:0] fifo_data_count;
reg [CLK_COUNT_WIDTH-1:0] data_clk_count;
reg [7:0]  fifo_block_adjust;

reg[19:0] cpu_lwaddr;
reg[19:0] em_lwaddr;
reg[19:0] pps_lwaddr;
reg[19:0] ae_lwaddr;
reg[11:0]  cpu_round_lwaddr;
reg[11:0]  em_round_lwaddr;
reg[11:0]  pps_round_lwaddr;
reg[11:0]  ae_round_lwaddr;

wire clear;
wire fifo_enable;
wire [ADDR_WIDTH-1:0] real_block_size;
wire [GUARD_WIDTH-1:0] fifo_guard_th;

// write registers
always @(posedge clk or negedge rst_b)
	if (!rst_b) begin
		trigger_source      <= 'h0; 
		dummy_write         <= 1'b0;
		fifo_clear_bit      <= 1'b0;
		wait_trigger_bit    <= 1'b0;
		clear_overflow_flag <= 1'b0;
		fifo_guard          <= 8'h0;
		block_size          <= 16'h0;
		fifo_block_adjust   <= 8'h0;
	end
	else if (fifo_cs & fifo_wr)
	begin
		case (fifo_addr[4:0])
			`TE_FIFO_CONFIG:
			begin
				trigger_source     <= fifo_d4wt[TRIGGER_WIDTH+7:8];
				fifo_clear_bit     <= fifo_d4wt[2];
				wait_trigger_bit   <= fifo_d4wt[1];
				dummy_write        <= fifo_d4wt[0];
			end
			`TE_FIFO_STATUS      : clear_overflow_flag <= fifo_d4wt[0];
			`TE_FIFO_GUARD       : fifo_guard <= fifo_d4wt[GUARD_WIDTH+7:8];	// only 8MSB effective
			`TE_FIFO_BLOCK_SIZE  : block_size <= fifo_d4wt[ADDR_WIDTH-1:0];
			`TE_FIFO_BLOCK_ADJUST: fifo_block_adjust <= fifo_d4wt[7:0];
		endcase
	end
	else if (fifo_skip)
		fifo_block_adjust <= 8'd0;
	else
	begin
		fifo_clear_bit     <= 1'b0;
		wait_trigger_bit   <= 1'b0;
	end

//read registers
always @ (*) begin
  if(fifo_rd & fifo_cs)
		case (fifo_addr[4:0])
			`TE_FIFO_CONFIG       : fifo_d4rd = {16'h0, trigger_source, 6'h0, wait_trigger, dummy_write};
			`TE_FIFO_STATUS       : fifo_d4rd = {{(24-DATA_COUNT_WIDTH){1'b0}}, fifo_data_count, 5'h0, fifo_enable, guard_alarm_flag, overflow_flag};
			`TE_FIFO_GUARD        : fifo_d4rd = {{(24-GUARD_WIDTH){1'b0}}, fifo_guard, 8'h0};
			`TE_FIFO_READ_ADDR    : fifo_d4rd = {{(32-ADDR_WIDTH){1'b0}}, read_addr};
			`TE_FIFO_WRITE_ADDR   : fifo_d4rd = {write_addr_round, write_addr, {CLK_COUNT_WIDTH{1'b0}}};
			`TE_FIFO_BLOCK_SIZE   : fifo_d4rd = {{(32-ADDR_WIDTH){1'b0}}, block_size};
			`TE_FIFO_BLOCK_ADJUST : fifo_d4rd = {{24{fifo_block_adjust[7]}}, fifo_block_adjust};
			`TE_FIFO_LWADDR_CPU   : fifo_d4rd = {cpu_round_lwaddr, cpu_lwaddr};
			`TE_FIFO_LWADDR_EM    : fifo_d4rd = {em_round_lwaddr, em_lwaddr};
			`TE_FIFO_LWADDR_PPS   : fifo_d4rd = {pps_round_lwaddr, pps_lwaddr};
			`TE_FIFO_LWADDR_AE    : fifo_d4rd = {ae_round_lwaddr, ae_lwaddr};
			default:                fifo_d4rd = 32'h0;
		endcase
	else
		fifo_d4rd = 32'h0;
end

// wire derived from register value
assign clear = fifo_clear | fifo_clear_bit;
assign fifo_enable = ~wait_trigger & te_enable;		
assign real_block_size = block_size + {{(ADDR_WIDTH-8){fifo_block_adjust[7]}}, fifo_block_adjust};
assign fifo_guard_th = (FIFO_SIZE >> 8) - fifo_guard;

//----------------------------------------------------------
// FIFO sync
//----------------------------------------------------------
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		wait_trigger <= 1'b0;
	else if (wait_trigger_bit)
		wait_trigger <= 1'b1;
	else if (|(trigger_source & fifo_trigger_in))
		wait_trigger <= 1'b0;

//----------------------------------------------------------
// FIFO RAM
//----------------------------------------------------------
reg read_fifo_valid;
reg write_fifo_valid;
reg [4*DATA_WIDTH-1:0] data_to_write;
wire [4*DATA_WIDTH-1:0] ram_read_data;
reg [ADDR_WIDTH-3:0] addr_to_write;
reg [ADDR_WIDTH-1:0] cur_read_addr;

//spram #(.RAM_SIZE(FIFO_SIZE/4), .ADDR_WIDTH(ADDR_WIDTH-2), .DATA_WIDTH(DATA_WIDTH*4)) fifo_ram
te_fifo_ram_2560x32_wrapper fifo_ram
(
		.clk (clk),
		.en (read_fifo_valid | write_fifo_valid),
		.we (write_fifo_valid),
		.addr (write_fifo_valid ? addr_to_write : cur_read_addr[ADDR_WIDTH-1:2]),
		.wdata (data_to_write),
		.rdata (ram_read_data)
);

//----------------------------------------------------------
// write FIFO
//----------------------------------------------------------
wire to_do_read;
wire fifo_write_en;
assign fifo_write_en = sample_valid & fifo_enable;	// FIFO write valid

wire [ADDR_WIDTH-1:0] write_addr_next;
wire write_rewind;
assign write_addr_next = write_addr + 1;
assign write_rewind = (write_addr_next == FIFO_SIZE);

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		write_addr <= 'd0;
	else if (clear)
		write_addr <= 'd0;
	else if (fifo_write_en)
		write_addr <= write_rewind ? 'd0 : write_addr_next;

// system clock counter between two samples
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		data_clk_count <= 'd0;
	else if (clear | fifo_write_en)
		data_clk_count <= 'd0;
	else
		data_clk_count <= data_clk_count + 1;

// write address round
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		write_addr_round <= 'd0;
	else if (clear)
		write_addr_round <= 'd0;
	else if (fifo_write_en & write_rewind)
		write_addr_round <= write_addr_round + 1;

// write buffering
reg write_pending;
reg [3*DATA_WIDTH-1:0] write_shift;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		write_pending <= 1'b0;
	else if (clear)
		write_pending <= 1'b0;
	else if (fifo_write_en && write_addr[1:0] == 2'b11 && ~dummy_write)
		write_pending <= 1'b1;
	else if (~to_do_read)
		write_pending <= 1'b0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		write_fifo_valid <= 1'b0;
	else if (clear)
		write_fifo_valid <= 1'b0;
	else if (write_pending && ~to_do_read)
		write_fifo_valid <= 1'b1;
	else
		write_fifo_valid <= 1'b0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		write_shift <= 'd0;
	else if (fifo_write_en && ~dummy_write)
		write_shift <= {write_shift[2*DATA_WIDTH-1:0], sample_data};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		data_to_write <= 'd0;
	else if (fifo_write_en && write_addr[1:0] == 2'b11 && ~dummy_write)
		data_to_write <= {write_shift, sample_data};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		addr_to_write <= 'd0;
	else if (fifo_write_en && write_addr[1:0] == 2'b11 && ~dummy_write)
		addr_to_write <= write_addr[ADDR_WIDTH-1:2];

//----------------------------------------------------------
// FIFO data count
//----------------------------------------------------------
// skip signal may be valid at the same time with fifo_write_en
// delay skip one clock cycle under this condition
reg fifo_skip_de1ay;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		fifo_skip_de1ay <= 1'b0;
	else if (fifo_skip & fifo_write_en)	// skip and fifo_write_en on same cycle, enable fifo_skip_delay
		fifo_skip_de1ay <= 1'b1;	
	else if(~fifo_write_en)	// clear when fifo_write_en disabled
		fifo_skip_de1ay <= 1'b0;

wire decrease_data_count;
assign decrease_data_count = (fifo_skip | fifo_skip_de1ay) & (~fifo_write_en);
		
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		fifo_data_count <= 'd0;
	else if (clear)
		fifo_data_count <= 'd0;
	else if (fifo_write_en)
		fifo_data_count <= fifo_data_count + 1;
	else if (decrease_data_count)
		fifo_data_count <= fifo_data_count - real_block_size;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		overflow_flag <= 1'b0;
	else if (fifo_write_en && ~overflow_flag)
		overflow_flag <= (fifo_data_count > FIFO_SIZE) ? 1 : 0;
	else if (clear | clear_overflow_flag)
		overflow_flag <= 1'b0;

assign fifo_ready       = (fifo_data_count >= real_block_size) & fifo_enable;		
assign fifo_trigger_out = (fifo_data_count == real_block_size);	 
assign guard_alarm_flag = (fifo_data_count[ADDR_WIDTH-1:8] >= fifo_guard_th);

//----------------------------------------------------------
// read FIFO
//----------------------------------------------------------
reg [ADDR_WIDTH-1:0] read_count;
wire [ADDR_WIDTH-1:0] read_count_next;
assign read_count_next = read_count + 1;

reg fifo_output, fifo_output_d;
assign to_do_read = fifo_read || ((cur_read_addr[1:0] == 2'b11) && fifo_output);

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		fifo_output <= 1'b0;
	else if (fifo_read)
		fifo_output <= 1'b1;
	else if (read_count_next == real_block_size)
		fifo_output <= 1'b0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		fifo_output_d <= 1'b0;
	else
		fifo_output_d <= fifo_output;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		fifo_data_valid <= 1'b0;
	else
		fifo_data_valid <= fifo_output_d;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		read_fifo_valid <= 1'b0;
	else if (clear)
		read_fifo_valid <= 1'b0;
	else if (~fifo_enable)
		read_fifo_valid <= 1'b0; 
	else if (read_count_next == real_block_size)	// deactivate when one block completed
		read_fifo_valid <= 1'b0;
	else if (to_do_read)
		read_fifo_valid <= 1'b1;
	else
		read_fifo_valid <= 1'b0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		read_count <= 'd0;
	else if (clear | ~fifo_output)
		read_count <= 'd0;
	else
		read_count <= read_count_next;

wire [ADDR_WIDTH-1:0] read_addr_next;
assign read_addr_next = cur_read_addr + 1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cur_read_addr <= 'd0;
	else if (clear)
		cur_read_addr <= 'd0;
	else if (fifo_rewind)
		cur_read_addr <= read_addr;
	else if (fifo_output)
		cur_read_addr <= (read_addr_next == FIFO_SIZE) ? 'd0 : read_addr_next;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		read_addr <= 'd0;
	else if (clear)
		read_addr <= 'd0;
	else if (fifo_skip)
		read_addr <= cur_read_addr;

// output data select
reg ram_data_valid;
reg [4*DATA_WIDTH-1:0] fifo_data_latch;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		ram_data_valid <= 1'b0;
	else
		ram_data_valid <= read_fifo_valid;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		fifo_data_latch <= 'd0;
	else if (ram_data_valid)
		fifo_data_latch <= ram_read_data;

// delay 2 cycles for data select and last data
reg [1:0] read_addr_d, data_sel;
reg last_output;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		read_addr_d <= 'd0;
	else
		read_addr_d <= cur_read_addr[1:0];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		data_sel <= 'd0;
	else
		data_sel <= read_addr_d;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		last_output <= 'd0;
	else
		last_output <= (fifo_output_d && (read_count == real_block_size));

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		fifo_last_data <= 'd0;
	else
		fifo_last_data <= last_output;

always @ (*)
	case(data_sel)
		2'b00  : fifo_data = fifo_data_latch[4*DATA_WIDTH-1:3*DATA_WIDTH];
		2'b01  : fifo_data = fifo_data_latch[3*DATA_WIDTH-1:2*DATA_WIDTH];
		2'b10  : fifo_data = fifo_data_latch[2*DATA_WIDTH-1:1*DATA_WIDTH];
		2'b11  : fifo_data = fifo_data_latch[1*DATA_WIDTH-1:0];
		default: fifo_data = 'd0;
	endcase

//----------------------------------------------------------
// write address latch on rise edge of signal
//----------------------------------------------------------
reg cpu_latch_d;
reg em_latch_d;
reg pps_latch_d;
reg ae_latch_d;

always @(posedge clk or negedge rst_b)
	if (!rst_b) begin
		cpu_latch_d   <= 1'b0;
		em_latch_d    <= 1'b0;
		pps_latch_d	  <= 1'b0;
		ae_latch_d    <= 1'b0;
	end
	else begin
		cpu_latch_d   <= cpu_latch;
		em_latch_d    <= em_latch;
		pps_latch_d	  <= pps_latch;	
		ae_latch_d    <= ae_latch;
end

always @(posedge clk or negedge rst_b)
	if (!rst_b) begin
		cpu_lwaddr    <= 'h0;
		cpu_round_lwaddr <= 'h0;
	end
	else
		if(~cpu_latch_d & cpu_latch)
		begin
			cpu_lwaddr <= {write_addr, data_clk_count};
			cpu_round_lwaddr <= write_addr_round;
		end

always @(posedge clk or negedge rst_b)
	if (!rst_b) begin
		em_lwaddr    <= 'h0;
		em_round_lwaddr <= 'h0;
	end
	else
		if(~em_latch_d & em_latch)
		begin
			em_lwaddr <= {write_addr, data_clk_count};
			em_round_lwaddr <= write_addr_round;
		end

always @(posedge clk or negedge rst_b)
	if (!rst_b) begin
		pps_lwaddr    <= 'h0;
		pps_round_lwaddr <= 'h0;
	end
	else
		if(~pps_latch_d & pps_latch)
		begin
			pps_lwaddr <= {write_addr, data_clk_count};
			pps_round_lwaddr <= write_addr_round;
		end

always @(posedge clk or negedge rst_b)
	if (!rst_b) begin
		ae_lwaddr    <= 'h0;
		ae_round_lwaddr <= 'h0;
	end
	else
		if(~ae_latch_d & ae_latch)
		begin
			ae_lwaddr <= {write_addr, data_clk_count};
			ae_round_lwaddr <= write_addr_round;
		end

endmodule