//----------------------------------------------------------------------
// gnss_top.v:
//   GNSS Top level module
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module gnss_top
(
// system signals
input clk,   // system clock
input rst_b, // reset signal, low active

// adc input
input adc_clk,
input [7:0] adc_data,

// host interface
input host_cs,
input host_rd,
input host_wr,
input [13:0] host_addr,	// DWORD address for 64kB address space
input [31:0] host_d4wt,
output [31:0] host_d4rd,

// interactive signals
input event_mark,
output pps_pulse,
output irq	// level trigger
);

//----------------------------------------------------------
// global registers
//----------------------------------------------------------
reg te_enable;
reg [9:0] meas_number;
reg [9:0] meas_count;
reg [9:0] request_count;
reg [3:0] int_mask;
reg ae_int_flag, request_int_flag, meas_int_flag, data_ready_int_flag;
wire [3:0] int_flag;
assign int_flag = {ae_int_flag, request_int_flag, meas_int_flag, data_ready_int_flag};

reg te_start;
wire te_running;
wire te_ready;
wire te_over;

wire host_cs_global, host_cs_ae, host_cs_te_fifo, host_cs_te;
wire host_cs_te_buffer, host_cs_ae_buffer;
assign host_cs_global    = host_cs & (host_addr[13:10] == (`GLB_BASE_ADDR     >> 10));
assign host_cs_ae        = host_cs & (host_addr[13:10] == (`AE_BASE_ADDR      >> 10));
assign host_cs_te_fifo   = host_cs & (host_addr[13:10] == (`TE_FIFO_BASE_ADDR >> 10));
assign host_cs_te        = host_cs & (host_addr[13:10] == (`TE_BASE_ADDR      >> 10));
assign host_cs_te_buffer = host_cs & (host_addr[13:10] == (`TE_BUFFER_ADDR    >> 10));
assign host_cs_ae_buffer = host_cs & (host_addr[13:10] == (`AE_BUFFER_ADDR    >> 10));

// write registers
always @(posedge clk or negedge rst_b)
	if (!rst_b) begin
		te_enable   <= 'h0;
		meas_number <= 'h0;
		int_mask    <= 'h0;
	end
	else if (host_cs_global & host_wr)
	begin
		case(host_addr[4:0])
			`GLB_BB_ENABLE       : te_enable   <= host_d4wt[8];
			`GLB_MEAS_NUMBER     : meas_number <= host_d4wt[9:0];
			`GLB_INTERRUPT_MASK  : int_mask    <= host_d4wt[11:8];
		endcase
	end

//read registers
reg [31:0] global_reg_d4rd;
always @ (*) begin
	if (host_cs_global & host_rd)
		case(host_addr[4:0])
			`GLB_BB_ENABLE           : global_reg_d4rd = {23'h0, te_enable, 8'h0};
			`GLB_TRACKING_START      : global_reg_d4rd = {31'h0, te_running};
			`GLB_MEAS_NUMBER         : global_reg_d4rd = {22'h0, meas_number};
			`GLB_MEAS_COUNT          : global_reg_d4rd = {22'h0, meas_count};
			`GLB_INTERRUPT_FLAG      : global_reg_d4rd = {20'h0, int_flag, 8'h0};
			`GLB_REQUEST_COUNT       : global_reg_d4rd = {22'h0, request_count};
			`GLB_INTERRUPT_MASK      : global_reg_d4rd = {20'h0, int_mask, 8'h0};
			`GLB_BB_VERSION          : global_reg_d4rd = {`MAJOR_VERSION, `MINOR_VERSION, `RELEASE_VERSION};
			default                  : global_reg_d4rd = 32'h0;
		endcase
	else
		global_reg_d4rd = 32'h0;
end

// write actions
wire reset_fifo, reset_te, reset_ae;
wire clear_fifo, latch_fifo;
wire start_te;
wire assign_meas_count, assign_request_count;
wire clear_int;
assign reset_fifo = host_cs_global && host_wr && (host_addr[4:0] == `GLB_BB_RESET) && host_d4wt[8];
assign reset_te   = host_cs_global && host_wr && (host_addr[4:0] == `GLB_BB_RESET) && host_d4wt[1];
assign reset_ae   = host_cs_global && host_wr && (host_addr[4:0] == `GLB_BB_RESET) && host_d4wt[0];
assign clear_fifo = host_cs_global && host_wr && (host_addr[4:0] == `GLB_FIFO_CLEAR) && host_d4wt[8];
assign latch_fifo = host_cs_global && host_wr && (host_addr[4:0] == `GLB_FIFO_CLEAR) && host_d4wt[0];
assign start_te   = host_cs_global && host_wr && (host_addr[4:0] == `GLB_TRACKING_START) && host_d4wt[0];
assign assign_meas_count    = host_cs_global && host_wr && (host_addr[4:0] == `GLB_MEAS_COUNT);
assign assign_request_count = host_cs_global && host_wr && (host_addr[4:0] == `GLB_REQUEST_COUNT);
assign clear_int            = host_cs_global && host_wr && (host_addr[4:0] == `GLB_INTERRUPT_FLAG);

//----------------------------------------------------------
// host read/write signal
//----------------------------------------------------------
// register read value multiplexing and latch
wire [31:0] ae_reg_d4rd;
wire [31:0] te_fifo_reg_d4rd;
wire [31:0] te_reg_d4rd;
wire [31:0] ae_rd_buffer;
wire [31:0] te_rd_buffer;
reg [31:0] reg_d4rd;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		reg_d4rd <= 32'h0;
	else begin
		case (host_addr[13:10])
			(`GLB_BASE_ADDR     >> 10): reg_d4rd <= global_reg_d4rd;
			(`AE_BASE_ADDR      >> 10): reg_d4rd <= ae_reg_d4rd;
			(`TE_FIFO_BASE_ADDR >> 10): reg_d4rd <= te_fifo_reg_d4rd;
			(`TE_BASE_ADDR      >> 10): reg_d4rd <= te_reg_d4rd;
			default                   : reg_d4rd <= 32'h0;
		endcase
end

// multiplexing of register read value and buffer read value
assign host_d4rd = host_addr[13] ? (host_addr[12] ? ae_rd_buffer : te_rd_buffer) : reg_d4rd;

//----------------------------------------------------------
// synchronize ADC data by sampling input data and clock
//----------------------------------------------------------
wire sample_valid;
wire [7:0]	sample_data;

sync_data #(.DATA_WIDTH(8)) u_sync_data
(
	.clk           (clk          ),
	.rst_b         (rst_b        ),
	.adc_clk       (adc_clk      ),
	.adc_data      (adc_data     ),
	.sample_valid  (sample_valid ),
	.sample_data   (sample_data  )
);

reg event_mark_latch;
wire em_latch;
wire ae_latch;
assign em_latch = event_mark && ~event_mark_latch;	// rising edge of event mark

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		event_mark_latch <= 1'b0;
	else
		event_mark_latch <= event_mark;

wire fifo_ready;
wire fifo_last_data;
wire fifo_read;
wire fifo_rewind;
wire fifo_skip;
wire fifo_data_valid;
wire [7:0] fifo_data;

te_fifo #(.FIFO_SIZE(10240), .ADDR_WIDTH(14), .DATA_WIDTH(8), .TRIGGER_WIDTH(1)) u_te_fifo
(
	.clk              (clk                 ),
	.rst_b            (rst_b && ~reset_fifo),

	.te_enable        (te_enable           ),
	.fifo_clear       (clear_fifo          ),
	.fifo_trigger_in  (1'b1                ),
	.fifo_trigger_out (                    ),

	.cpu_latch        (latch_fifo          ),
	.em_latch         (em_latch            ),
	.pps_latch        (1'b0                ),
	.ae_latch         (ae_latch            ),

	.sample_valid     (sample_valid        ),
	.sample_data      (sample_data         ),

	.fifo_ready       (fifo_ready          ),
	.fifo_last_data   (fifo_last_data      ),
	.fifo_read        (fifo_read           ),
	.fifo_rewind      (fifo_rewind         ),
	.fifo_skip        (fifo_skip           ),
	.fifo_data_valid  (fifo_data_valid     ),
	.fifo_data        (fifo_data           ),

	.fifo_cs          (host_cs_te_fifo     ),
	.fifo_wr          (host_wr             ),
	.fifo_rd          (host_rd             ),
	.fifo_addr        (host_addr[4:0]      ),
	.fifo_d4wt        (host_d4wt           ),
	.fifo_d4rd        (te_fifo_reg_d4rd    )
);

//----------------------------------------------------------
// instance of tracking engine
//----------------------------------------------------------
wire te_weil_type;
wire [9:0] te_legendre_addr;
wire te_legendre_rd;
wire [15:0] te_legendre_data;
wire [13:0] te_memcode_addr;
wire te_memcode_rd;
wire [31:0] te_memcode_data;

tracking_engine u_tracking_engine
(
	.clk              (clk                 ),
	.rst_b            (rst_b && ~reset_te  ),

	.te_start         (te_start            ),	// TODO: auto trigger if no interrupt generated, or wait CPU trigger
	.te_running       (te_running          ),
	.te_ready         (te_ready            ),
	.te_over          (te_over             ),

	.fifo_ready       (fifo_ready          ),
	.fifo_last_data   (fifo_last_data      ),
	.fifo_read        (fifo_read           ),
	.fifo_rewind      (fifo_rewind         ),
	.fifo_skip        (fifo_skip           ),
	.fifo_data_valid  (fifo_data_valid     ),
	.fifo_data        (fifo_data           ),

	.te_reg_cs        (host_cs_te          ),
	.te_buffer_cs     (host_cs_te_buffer   ),
	.te_rd            (host_rd             ),
	.te_wr            (host_wr             ),
	.te_addr          (host_addr           ),
	.te_d4wt          (host_d4wt           ),
	.te_rd_buffer     (te_rd_buffer        ),
	.te_reg_d4rd      (te_reg_d4rd         ),

	.legendre_addr    ({te_weil_type, te_legendre_addr}),
	.legendre_rd      (te_legendre_rd      ),
	.legendre_data    (te_legendre_data    ),
	.memcode_addr     (te_memcode_addr     ),
	.memcode_rd       (te_memcode_rd       ),
	.memcode_data     (te_memcode_data     )
);

//----------------------------------------------------------
// instance of acquire engine
//----------------------------------------------------------
wire [14:0] ae_ram_addr;
wire [31:0] ae_ram_d4wt;
wire [31:0] ae_ram_d4rd;
wire ae_ram_en;
wire ae_ram_we;

spram #(.RAM_SIZE(32768), .ADDR_WIDTH(15), .DATA_WIDTH(32)) ae_buffer
(
		.clk      (clk           ),
		.en       (ae_ram_ena    ),
		.we       (ae_ram_we     ),
		.addr     (ae_ram_addr   ),
		.wdata    (ae_ram_d4wt   ),
		.rdata    (ae_ram_d4rd   )
);

wire ae_weil_type;
wire [9:0] ae_legendre_addr;
wire ae_legendre_rd;
wire legendre_read_valid;
wire [15:0] ae_legendre_data;
wire [13:0] ae_memcode_addr;
wire ae_memcode_rd;
wire [31:0] ae_memcode_data;
wire memcode_read_valid;    

wire ae_finish;

ae_top #(.AE_BUFFER_SIZE(16*256)) u_ae_top
(
	.clk                 (clk                  ),
	.rst_b               (rst_b && ~reset_ae   ),

	.ae_reg_cs           (host_cs_ae           ),
	.ae_buffer_cs        (host_cs_ae_buffer    ),
	.ae_rd               (host_rd              ),
	.ae_wr               (host_wr              ),
	.ae_addr             (host_addr[7:0]       ),
	.ae_d4wt             (host_d4wt            ),
	.ae_rd_buffer        (ae_rd_buffer         ),
	.ae_reg_d4rd         (ae_reg_d4rd          ),

	.ae_ram_ena          (ae_ram_ena           ),
	.ae_ram_we           (ae_ram_we            ),
	.ae_ram_addr         (ae_ram_addr          ),
	.ae_ram_d4wt         (ae_ram_d4wt          ),
	.ae_ram_d4rd         (ae_ram_d4rd          ),

	.legendre_addr       ({ae_weil_type, ae_legendre_addr}),
	.legendre_rd         (ae_legendre_rd       ),
	.legendre_read_valid (legendre_read_valid  ),
	.legendre_data       (ae_legendre_data     ),

	.memcode_addr        (ae_memcode_addr      ),
	.memcode_rd          (ae_memcode_rd        ),
	.memcode_read_valid  (ae_memcode_read_valid),
	.memcode_data        (ae_memcode_data      ),

	.sample_valid        (sample_valid         ),
	.sample_data         (sample_data          ),

	.fill_start          (ae_latch             ),
	.ae_finish           (ae_finish            )
);

//----------------------------------------------------------
// arbiter and ROM instance of Legendre code and memory code
//----------------------------------------------------------
// arbiter for Legendre and memory code
wire [9:0] legendre_addr_b1c, legendre_addr_l1c;
wire legendre_rd_b1c, legendre_rd_l1c;
wire [15:0] legendre_data_b1c, legendre_data_l1c;
wire [13:0] memcode_addr;
wire memcode_rd;
wire [31:0] memcode_data;

wire te_legendre_rd_b1c, te_legendre_rd_l1c, ae_legendre_rd_b1c, ae_legendre_rd_l1c;
assign te_legendre_rd_b1c = ~te_weil_type & te_legendre_rd;
assign te_legendre_rd_l1c = te_weil_type & te_legendre_rd;
assign ae_legendre_rd_b1c = ~ae_weil_type & ae_legendre_rd;
assign ae_legendre_rd_l1c = ae_weil_type & ae_legendre_rd;

assign legendre_read_valid = ((~te_legendre_rd_b1c) & ae_legendre_rd_b1c) || ((~te_legendre_rd_l1c) & ae_legendre_rd_l1c);
assign legendre_addr_b1c = te_legendre_rd_b1c ? te_legendre_addr : ae_legendre_addr;
assign legendre_rd_b1c = te_legendre_rd_b1c | ae_legendre_rd_b1c;
assign legendre_addr_l1c = te_legendre_rd_l1c ? te_legendre_addr : ae_legendre_addr;
assign legendre_rd_l1c = te_legendre_rd_l1c | ae_legendre_rd_l1c;
assign te_legendre_data = te_weil_type ? legendre_data_l1c : legendre_data_b1c;
assign ae_legendre_data = ae_weil_type ? legendre_data_l1c : legendre_data_b1c;

assign memcode_read_valid = (~te_memcode_rd) & ae_memcode_rd;
assign memcode_addr = te_memcode_rd ? te_memcode_addr : ae_memcode_addr;
assign memcode_rd = te_memcode_rd | ae_memcode_rd;
assign te_memcode_data = memcode_data;
assign ae_memcode_data = memcode_data;

// Legendre and memory code
sprom #(.ROM_SIZE(640), .ADDR_WIDTH(10), .DATA_WIDTH(16)) b1c_legendre_data
(
	.clk (clk),
	.rd (legendre_rd_b1c),
	.addr (legendre_addr_b1c),
	.rdata (legendre_data_b1c)
);

sprom #(.ROM_SIZE(640), .ADDR_WIDTH(10), .DATA_WIDTH(16)) l1c_legendre_data
(
	.clk (clk),
	.rd (legendre_rd_l1c),
	.addr (legendre_addr_l1c),
	.rdata (legendre_data_l1c)
);

sprom #(.ROM_SIZE(12800), .ADDR_WIDTH(14), .DATA_WIDTH(32)) memory_code_rom
(
	.clk (clk),
	.rd (memcode_rd),
	.addr (memcode_addr),
	.rdata (memcode_data)
);

//----------------------------------------------------------
// status, counter and trigger signal
//----------------------------------------------------------
// measurement count
wire [9:0] meas_count_next;
assign meas_count_next = meas_count + 1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		meas_count <= 'd0;
	else if (assign_meas_count)	// CPU assignment
		meas_count <= host_d4wt[9:0];
	else if (te_over)
		meas_count <= (meas_count_next == meas_number) ? 'd0 : meas_count_next;

// request count
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		request_count <= 'd0;
	else if (assign_request_count)	// CPU assignment
		request_count <= host_d4wt[9:0];
	else if (te_over && |request_count)
		request_count <= request_count - 1;

// interrupt flags
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		ae_int_flag <= 1'b0;
	else if (ae_finish)
		ae_int_flag <= 1'b1;
	else if (clear_int)	// CPU clear
		ae_int_flag <= ~host_d4wt[11] && ae_int_flag;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		request_int_flag <= 1'b0;
	else if (te_over && (request_count == 'd1))
		request_int_flag <= 1'b1;
	else if (clear_int)	// CPU clear
		request_int_flag <= ~host_d4wt[10] && request_int_flag;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		meas_int_flag <= 1'b0;
	else if (te_over && (meas_count_next == meas_number))
		meas_int_flag <= 1'b1;
	else if (clear_int)	// CPU clear
		meas_int_flag <= ~host_d4wt[9] && meas_int_flag;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		data_ready_int_flag <= 1'b0;
	else if (te_over && te_ready)
		data_ready_int_flag <= 1'b1;
	else if (clear_int)	// CPU clear
		data_ready_int_flag <= ~host_d4wt[8] && data_ready_int_flag;

// generate signal to start/restart TE
reg te_over_d;
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		te_over_d <= 1'b0;
	else
		te_over_d <= te_over;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		te_start <= 1'b0;
	else if (te_over_d && !request_int_flag && !meas_int_flag && !data_ready_int_flag)	// if TE over not trigger interrupt, generate te_start
		te_start <= 1'b1;
	else if (start_te && !te_running)	// CPU trigger if TE is not running
		te_start <= 1'b1;
	else	// only active one clock cycle
		te_start <= 1'b0;

assign irq = |(int_flag & int_mask);

assign pps_pulse = 0;	// TODO: add PPS module

endmodule