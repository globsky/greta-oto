//----------------------------------------------------------------------
// ae_core.v:
//   acquisition engine control and FSM module
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

`define MATCH_FILTER_DEPTH 682
`define CODE_LENGTH (`MATCH_FILTER_DEPTH/2)
//`define GPS_MAX_PHASE (`MATCH_FILTER_DEPTH-1)
//`define BDS_MAX_PHASE (`MATCH_FILTER_DEPTH*2-1)
//`define GLO_MAX_PHASE (`MATCH_FILTER_DEPTH-3)

module ae_core #(parameter BUFFER_ADDR_WIDTH = 15, ADDR_WIDTH = 8)	// total channel number should not exceed 2^(ADDR_WIDTH-3)
(
// system signals				
input clk,	//system clock
input rst_b,	//reset signal, low active

// control parameters and signal
input [5:0] channel_number,
input start_acquisition,

// config buffer interface
output config_buffer_rd,
output config_buffer_wt,
output [ADDR_WIDTH-1:0] config_buffer_addr,
input [31:0] config_buffer_d4rd,
output reg [31:0] config_buffer_d4wt,

// interface of sample access to AE buffer
output start_load_sample,
output [BUFFER_ADDR_WIDTH+2:0] buffer_read_address,	// each DWORD has 8 samples
output ae_buffer_read,		// indicator of read next AE buffer sample, also as current sample valid flag
input [3:0] ae_buffer_out,	// current sample valid on current clock cycle, change on ae_buffer_read high
input sample_ready,

// Legendre code ROM interface
output [10:0] legendre_addr,
output legendre_rd,
input legendre_read_valid,
input [15:0] legendre_data,

// memory code ROM interface
output [13:0] memcode_addr,
output memcode_rd,
input memcode_read_valid,
input [31:0] memcode_data,

// signal to top module
output [3:0] state_output,
output [4:0] channel_output,
output reg ae_finish_flag
);

// FSM states
reg [3:0] cur_state;
reg [3:0] next_state;
reg [1:0] result_state;
reg [1:0] result_next_state;
reg [1:0] preload_state;
reg [1:0] preload_next_state;

//--------------------------------------------
// AE channel control registers
//--------------------------------------------
reg [ADDR_WIDTH-3:0] channel_count;
reg [2:0] peak_ratio;
reg early_terminate;
reg [1:0] prn_select;
reg [5:0] svid;
reg [5:0] stride_number, stride_count;
reg [4:0] code_span, code_round;
reg [5:0] coh_number, coh_count;
reg [6:0] noncoh_number, noncoh_count;
reg [1:0] segment_count;
reg code_select;
reg [19:0] center_freq;
reg [10:0] dft_freq;
reg [4:0] read_address;
reg [21:0] stride_interval;
reg [31:0] carrier_freq;
reg [31:0] carrier_freq_pos;
reg [31:0] carrier_freq_neg;
reg [ADDR_WIDTH-3:0] channel_save;
reg [2:0] channel_param_addr;
reg channel_save_pending;
reg latch_channel_param;
reg coh_read_to_noncoh;
reg success_flag;

wire fill_sample_finish;
wire fill_code_finish;
wire segment_round_finish;
wire coh_read_finish;
wire coh_write_finish;
wire last_read_finish;

assign state_output = cur_state;
assign channel_output = channel_count[4:0];

//----------------------------------------------------------
// AE control state machine
//----------------------------------------------------------
localparam
		IDLE               = 4'd0,
		NEXT_CHANNEL       = 4'd1,
		LOAD_CHANNEL_PARAM = 4'd2,
		NEXT_STRIDE        = 4'd3,
		NEXT_CODE_ROUND    = 4'd4,
		FILL_SAMPLE        = 4'd5,
		NEXT_NONCOH        = 4'd6,
		NEXT_COH           = 4'd7,
		NEXT_SEGMENT       = 4'd8,
		WAIT_SEGMENT_ROUND = 4'd9,
		LAST_COH_FINISH    = 4'd10,
		FORCE_OUTPUT       = 4'd11,
		WAIT_SEARCH_FINISH = 4'd12;

localparam
		RESULT_IDLE         = 2'd0,
		RESULT_DETERMINE    = 2'd1,
		RESULT_SAVE         = 2'd2;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cur_state <= IDLE;
	else  
		cur_state <= next_state;

always @(*)
	case(cur_state)
	IDLE:
		next_state = start_acquisition ? NEXT_CHANNEL : IDLE;
	NEXT_CHANNEL:
		next_state = (channel_count == channel_number) ? (coh_read_to_noncoh ? LAST_COH_FINISH : IDLE) : (result_state == RESULT_IDLE) ? LOAD_CHANNEL_PARAM : NEXT_CHANNEL;
	LOAD_CHANNEL_PARAM:
		next_state = (channel_param_addr == 3'b011) ? NEXT_STRIDE : LOAD_CHANNEL_PARAM;
	NEXT_STRIDE:
		next_state = (stride_count == stride_number) ? NEXT_CHANNEL : NEXT_CODE_ROUND;
	NEXT_CODE_ROUND:
		next_state = FILL_SAMPLE;
	FILL_SAMPLE:
		next_state = (fill_sample_finish && fill_code_finish) ? NEXT_NONCOH : FILL_SAMPLE;
	NEXT_NONCOH:
		next_state = (noncoh_count == noncoh_number) ? ((code_round == code_span) ? NEXT_STRIDE : NEXT_CODE_ROUND) : NEXT_COH;
	NEXT_COH:
		next_state = (coh_count == coh_number) ? NEXT_NONCOH : NEXT_SEGMENT;
	NEXT_SEGMENT:
		next_state = (segment_count == 2'b11) ? NEXT_COH : (fill_code_finish ? WAIT_SEGMENT_ROUND : NEXT_SEGMENT);
	WAIT_SEGMENT_ROUND:
		next_state = success_flag ? NEXT_CHANNEL : (segment_round_finish ? NEXT_SEGMENT : WAIT_SEGMENT_ROUND);
	LAST_COH_FINISH:
		next_state = coh_write_finish ? FORCE_OUTPUT : LAST_COH_FINISH;
	FORCE_OUTPUT:
		next_state = last_read_finish ? WAIT_SEARCH_FINISH : FORCE_OUTPUT;
	WAIT_SEARCH_FINISH:
		next_state = (~channel_save_pending && (result_state == RESULT_IDLE)) ? IDLE : WAIT_SEARCH_FINISH;
	default:
		next_state = IDLE;
	endcase

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		ae_finish_flag <= 1'b0;
	else if (cur_state == IDLE && start_acquisition)
		ae_finish_flag <= 1'b0;
	else if ((cur_state == WAIT_SEARCH_FINISH) && (~channel_save_pending) && (result_state == RESULT_IDLE))		// go back to idle after last channel force output and result write done
		ae_finish_flag <= 1'b1;
	else if ((cur_state == NEXT_CHANNEL) && (channel_count == channel_number) && ~coh_read_to_noncoh)	// last channel early terminate
		ae_finish_flag <= 1'b1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		channel_count <= 'd0;
	else if (cur_state == IDLE && start_acquisition)
		channel_count <= 'd0;
	else if ((cur_state == NEXT_STRIDE) && (stride_count == stride_number))
		channel_count <= channel_count + 1;
	else if ((cur_state == WAIT_SEGMENT_ROUND) && success_flag)
		channel_count <= channel_count + 1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		stride_count <= 'd0;
	else if (cur_state == NEXT_CHANNEL)
		stride_count <= 'd0;
	else if ((cur_state == NEXT_NONCOH) && (noncoh_count == noncoh_number) && (code_round == code_span))
		stride_count <= stride_count + 1;

wire carrier_freq_assign;		// first assignment
assign carrier_freq_assign = (latch_channel_param && channel_param_addr[1:0] == 2'b00);

reg  carrier_freq_adjust_pos;
reg  carrier_freq_adjust_neg;
always @(posedge clk or negedge rst_b)
	if (!rst_b) begin
		carrier_freq_adjust_pos <= 'd0;
		carrier_freq_adjust_neg <= 'd0;
	end
	else begin
		carrier_freq_adjust_pos <= ((cur_state == NEXT_STRIDE) && ~stride_count[0]);
		carrier_freq_adjust_neg <= ((cur_state == NEXT_STRIDE) && stride_count[0]);
	end

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		carrier_freq_pos <= 'd0;
	else
		if(carrier_freq_assign)
			carrier_freq_pos <= {center_freq, 12'h000};
		else if(carrier_freq_adjust_pos)
			carrier_freq_pos <= carrier_freq_pos + stride_interval;
		
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		carrier_freq_neg <= 'd0;
	else
		if(carrier_freq_assign)
			carrier_freq_neg <= {center_freq, 12'h000};
		else if(carrier_freq_adjust_neg)
			carrier_freq_neg <= carrier_freq_neg - stride_interval;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		channel_param_addr <= 3'd0;
	else if (cur_state == IDLE && start_acquisition)
		channel_param_addr <= 3'd0;
	else if ((cur_state == NEXT_STRIDE) && (stride_count == stride_number))
		channel_param_addr <= 3'd0;
	else if (result_state == RESULT_DETERMINE)
		channel_param_addr <= 3'd4;
	else if ((cur_state == LOAD_CHANNEL_PARAM) || (result_state == RESULT_SAVE))
		channel_param_addr <= channel_param_addr + 1;

//----------------------------------------------------------
// adjust channel control registers
//----------------------------------------------------------
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		carrier_freq <= 32'd0;
	else if (carrier_freq_assign)	// first assignment
		carrier_freq <= {center_freq, 12'h000};
	else if (cur_state == NEXT_STRIDE)	// choose positive or negative
		carrier_freq <= stride_count[0] ? carrier_freq_pos : carrier_freq_neg;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		noncoh_count <= 'd0;
	else if (cur_state == NEXT_CODE_ROUND)
		noncoh_count <= 'd0;
	else if ((cur_state == NEXT_COH) && (coh_count == coh_number))
		noncoh_count <= noncoh_count + 1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coh_count <= 'd0;
	else if (cur_state == NEXT_NONCOH)
		coh_count <= 'd0;
	else if ((cur_state == NEXT_SEGMENT) && (segment_count == 2'b11))
		coh_count <= coh_count + 1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		segment_count <= 'd0;
	else if (cur_state == NEXT_COH)
		segment_count <= 'd0;
	else if ((cur_state == WAIT_SEGMENT_ROUND) && segment_round_finish)
		segment_count <= segment_count + 1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		code_round <= 'd0;
	else if (cur_state == NEXT_STRIDE)
		code_round <= 'd0;
	else if (cur_state == NEXT_CODE_ROUND)
		code_round <= code_round + 1;

wire last_acc;
assign last_acc = (segment_count == 2'b10) && ((coh_count + 1) == coh_number) && ((noncoh_count + 1) == noncoh_number);

//----------------------------------------------------------
// latch channel parameter
//----------------------------------------------------------
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		latch_channel_param <= 1'b0;
	else if (cur_state == LOAD_CHANNEL_PARAM)
		latch_channel_param <= 1'b1;
	else
		latch_channel_param <= 1'b0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		early_terminate <= 1'b0;
	else if (latch_channel_param && channel_param_addr[1:0] == 2'b01)
		early_terminate <= config_buffer_d4rd[27];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		peak_ratio <= 3'd0;
	else if (latch_channel_param && channel_param_addr[1:0] == 2'b01)
		peak_ratio <= config_buffer_d4rd[26:24];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		noncoh_number <= 10'd0;
	else if (latch_channel_param && channel_param_addr[1:0] == 2'b01)
		noncoh_number <= config_buffer_d4rd[22:16];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coh_number <= 8'd0;
	else if (latch_channel_param && channel_param_addr[1:0] == 2'b01)
		coh_number <= config_buffer_d4rd[13:8];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		stride_number <= 6'd0;
	else if (latch_channel_param && channel_param_addr[1:0] == 2'b01)
		stride_number <= config_buffer_d4rd[5:0];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		prn_select <= 2'b00;
	else if (latch_channel_param && channel_param_addr[1:0] == 2'b10)
		prn_select <= config_buffer_d4rd[31:30];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		svid <= 6'd0;
	else if (latch_channel_param && channel_param_addr[1:0] == 2'b10)
		svid <= config_buffer_d4rd[29:24];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		center_freq <= 20'd0;
	else if (latch_channel_param && channel_param_addr[1:0] == 2'b10)
		center_freq <= config_buffer_d4rd[19:0];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		dft_freq <= 11'd0;
	else if (latch_channel_param && channel_param_addr[1:0] == 2'b11)
		dft_freq <= config_buffer_d4rd[30:20];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		read_address <= 5'd0;
	else if (latch_channel_param && channel_param_addr[1:0] == 2'b11)
		read_address <= config_buffer_d4rd[12:8];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		code_span <= 5'd0;
	else if (latch_channel_param && channel_param_addr[1:0] == 2'b11)
		code_span <= config_buffer_d4rd[4:0];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		stride_interval <= 22'd0;
	else if (latch_channel_param && channel_param_addr[1:0] == 2'b00)
		stride_interval <= config_buffer_d4rd[21:0];

// calculate buffer_read_address = address_offset * 682
wire [5:0] address_offset;
wire [9:0] addr_mul1, addr_mul2, addr_mul3;
wire [14:0] addr_mul;
assign address_offset = read_address + code_round;
assign addr_mul1 = {address_offset[1:0], address_offset[1:0], address_offset[1:0], address_offset[1:0], address_offset[1:0]};
assign addr_mul2 = {address_offset[3:2], address_offset[3:2], address_offset[3:2], address_offset[3:2], address_offset[3:2]};
assign addr_mul3 = {address_offset[5:4], address_offset[5:4], address_offset[5:4], address_offset[5:4], address_offset[5:4]};
assign addr_mul = {1'b0, addr_mul3, 4'b0000} + {3'b000, addr_mul2, 2'b00} + {5'b00000, addr_mul1};
assign buffer_read_address = {{(BUFFER_ADDR_WIDTH-13){1'b0}}, addr_mul, 1'b0};

// interface to channel config memory
assign config_buffer_rd = (cur_state == LOAD_CHANNEL_PARAM);
assign config_buffer_wt = (result_state == RESULT_SAVE);
assign config_buffer_addr = {config_buffer_wt ? channel_save[ADDR_WIDTH-4:0] : channel_count[ADDR_WIDTH-4:0], channel_param_addr};

//----------------------------------------------------------
// load samples
//----------------------------------------------------------
reg [9:0] sample_count;		// sample count for preload and within segment
reg sample_gap;			// load sample once another cycle when processing
//reg segment_first_sample;
//reg code_round_first_sample;
wire segment_last_sample;
wire preload_last_sample;
wire signed [5:0] sample_out_i;
wire signed [5:0] sample_out_q;
wire sample_out_valid;

// extra 3 clock cycles after last sample
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		sample_gap <= 1'b0;
	else if (cur_state == NEXT_SEGMENT)
		sample_gap <= 1'b0;
	else if (ae_buffer_read && (cur_state == WAIT_SEGMENT_ROUND))	// insert one gap cycle
		sample_gap <= 1'b1;
	else if (sample_gap)
		sample_gap <= 1'b0;

assign segment_last_sample = (sample_count == (`MATCH_FILTER_DEPTH-1)) && ae_buffer_read;
assign segment_round_finish = segment_last_sample;
assign start_load_sample = (cur_state == NEXT_CODE_ROUND);
assign preload_last_sample = (sample_count == (`MATCH_FILTER_DEPTH-2));	// preload count from 0 to `MATCH_FILTER_DEPTH-2
assign fill_sample_finish = preload_last_sample;
assign ae_buffer_read = preload_state[1] || ((cur_state == WAIT_SEGMENT_ROUND) && ~sample_gap);	// active high when preload or segment processing

localparam
		PRELOAD_IDLE        = 2'd0,
		PRELOAD_WAIT_READY  = 2'd1,
		PRELOAD_PROCESS     = 2'd2;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		preload_state <= PRELOAD_IDLE;
	else  
		preload_state <= preload_next_state;

always @(*)
	case(preload_state)
	PRELOAD_IDLE:
		preload_next_state = start_load_sample ? PRELOAD_WAIT_READY : PRELOAD_IDLE;
	PRELOAD_WAIT_READY:
		preload_next_state = sample_ready ? PRELOAD_PROCESS : PRELOAD_WAIT_READY;
	PRELOAD_PROCESS:	// stay in this state for `MATCH_FILTER_DEPTH-1 cycles, deduct first sample, total preload `MATCH_FILTER_DEPTH-2 samples
		preload_next_state = preload_last_sample ? PRELOAD_IDLE : PRELOAD_PROCESS;
	default:
		preload_next_state = PRELOAD_IDLE;
	endcase

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		sample_count <= 'd0;
	else if (start_load_sample)
		sample_count <= 'd0;
	else if (cur_state == NEXT_SEGMENT)
		sample_count <= 'd0;
	else if (ae_buffer_read)
		sample_count <= sample_count + 1;

// following signals used to calculate DFT twiddle factor and determine behavior of coherent sum,
// they have two clock cycle ahead of valid data, so after go through match filter, output sync signal
// also have two clock cycle ahead of valid MF out data
// giving once clock cycle delay to calculate dft_nco and one clock cycle delay to calculate acc_nco and do sin/cos LUT
// DFT twiddle factor latch will align to MF out data latch in coherent sum module
wire segment_first_sample;
wire coh_first_sample;
wire noncoh_first_sample;
assign segment_first_sample = (cur_state == NEXT_SEGMENT) && ~(segment_count == 2'b11);
assign coh_first_sample = (cur_state == NEXT_SEGMENT) && (segment_count == 2'b00) && (coh_count == 'd0);
assign noncoh_first_sample = coh_first_sample && (noncoh_count == 'd0);

read_sample u_read_sample
(
	.clk               (clk                 ),
	.rst_b             (rst_b               ),
	.clear             (start_load_sample   ),
	.carrier_freq      (carrier_freq        ),
	.sample_in         (ae_buffer_out       ),
	.sample_in_valid   (ae_buffer_read      ),
	.sample_out_i      (sample_out_i        ),
	.sample_out_q      (sample_out_q        ),
	.sample_out_valid  (sample_out_valid    )
);

//----------------------------------------------------------
// load PRN codes
//----------------------------------------------------------
wire prn_code_out;
wire preload_code;
wire start_load_code;		// signal align to cur_state
wire toggle_code;				// signal align to cur_state
assign preload_code = (cur_state == NEXT_CODE_ROUND);
// start_load_code condition: at NEXT_SEGMENT state and fill previous code finish without following condition:
// last_acc: last segment of coh-noncoh loop
// segment_count == 2'b11: going out from segment loop to upper loop
assign start_load_code = (cur_state == NEXT_SEGMENT) && fill_code_finish && ~(segment_count == 2'b11) && ~last_acc;
assign toggle_code = (cur_state == WAIT_SEGMENT_ROUND) && segment_round_finish;

reg [`CODE_LENGTH-1:0] prn_code0;
reg [`CODE_LENGTH-1:0] prn_code1;
reg load_code0, load_code1;
reg [8:0] load_code_count;

assign fill_code_finish = code_select ? (~load_code1) : (~load_code0);

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		code_select <= 1'b0;
	else if (cur_state == NEXT_CODE_ROUND)
		code_select <= 1'b0;
	else if (toggle_code)
		code_select <= ~code_select;

// start_load_code and code_select all align to cur_state == NEXT_SEGMENT
// toggle code for MF will delay 3 cycles:
// AE buffer valid sample has one cycle delay
// shift to MF buffer has one cycle delay
// xor with Q branch value has one cycle delay
reg [2:0] start_load_code_d;
reg [2:0] code_select_d;

// delay for 3 clock cycles
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		start_load_code_d <= 3'b000;
	else
		start_load_code_d <= {start_load_code_d[1:0], start_load_code};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		code_select_d <= 3'b000;
	else
		code_select_d <= {code_select_d[1:0], code_select};

// assign load code 0 and 1 signal
wire start_load_code0, start_load_code1;
assign start_load_code0 = start_load_code_d[2] && code_select_d[2];
assign start_load_code1 = start_load_code_d[2] && ~code_select_d[2];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		load_code0 <= 1'b0;
	else if (preload_code || start_load_code0)
		load_code0 <= 1'b1;
	else if (load_code0 && load_code_count == `CODE_LENGTH - 1)
		load_code0 <= 1'b0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		load_code1 <= 1'b0;
	else if (start_load_code1)
		load_code1 <= 1'b1;
	else if (load_code1 && load_code_count == `CODE_LENGTH - 1)
		load_code1 <= 1'b0;

wire ready_to_shift;
wire shift_code;
assign shift_code = (load_code0 || load_code1) && ready_to_shift;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		load_code_count <= 'd0;
	else if (preload_code)
		load_code_count <= 'd0;
	else if (load_code_count == `CODE_LENGTH - 1)
		load_code_count <= 'd0;
	else if (shift_code)
		load_code_count <= load_code_count + 1;

ae_prn_gen u_ae_prn_gen
(
	.clk                 (clk                 ),
	.rst_b               (rst_b               ),

	.phase_init          (preload_code        ),
	.prn_select          (prn_select          ),
	.svid                (svid                ),
	.shift_code          (shift_code          ),
	.prn_code            (prn_code_out        ),
	.ready_to_shift      (ready_to_shift      ),

	.legendre_addr       (legendre_addr       ),
	.legendre_rd         (legendre_rd         ),
	.legendre_read_valid (legendre_read_valid ),
	.legendre_data       (legendre_data       ),

	.memcode_addr        (memcode_addr        ),
	.memcode_rd          (memcode_rd          ),
	.memcode_read_valid  (memcode_read_valid  ),
	.memcode_data        (memcode_data        )
);

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		prn_code0 <= 'd0;
	else if (load_code0 && ready_to_shift)
		prn_code0 <= {prn_code0[`CODE_LENGTH-2:0], prn_code_out};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		prn_code1 <= 'd0;
	else if (load_code1 && ready_to_shift)
		prn_code1 <= {prn_code1[`CODE_LENGTH-2:0], prn_code_out};

//----------------------------------------------------------
// match filter core
//----------------------------------------------------------
reg mf_preload;
wire mf_segment_first;
wire mf_coh_first;
wire mf_noncoh_first;
wire signed [15:0] mf_out_i;
wire signed [15:0] mf_out_q;
wire mf_out_valid;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		mf_preload <= 1'b0;
	else if (start_load_sample)
		mf_preload <= 1'b1;
	else if (mf_preload && (cur_state == NEXT_SEGMENT))
		mf_preload <= 1'b0;

// new segment_count value align to NEXT_SEGMENT state
// dealy 1 clock cycle to align with first sample
reg value_select;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		value_select <= 1'b0;
	else
		value_select <= segment_count[1];

mf_core	#(.SYNC_SIGNAL_NUMBER(4)) u_mf_core
(
	.clk          (clk             ),
	.rst_b        (rst_b           ),
	.sync_in      ({value_select, noncoh_first_sample, coh_first_sample, segment_first_sample}),
	.sync_out     ({value_select_out, mf_noncoh_first, mf_coh_first, mf_segment_first}),
	.value_select (value_select_out),
	.sample_i     (sample_out_i    ),
	.sample_q     (sample_out_q    ),
	.sample_valid (sample_out_valid),
	.preload      (mf_preload      ),
	.code         (code_select_d[2] ? prn_code1 : prn_code0),
	.out_i        (mf_out_i        ),
	.out_q        (mf_out_q        ),
	.out_valid    (mf_out_valid    )
);

//----------------------------------------------------------
// coherent buffer last read
//----------------------------------------------------------
reg [9:0] last_read_addr;
reg last_read_en;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		last_read_addr <= 'd0;
	else if ((cur_state == LAST_COH_FINISH) && coh_write_finish)
		last_read_addr <= 'd0;
	else if (last_read_finish)
		last_read_addr <= 'd0;
	else if (last_read_en)
		last_read_addr <= last_read_addr + 1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		last_read_en <= 1'b0;
	else if (last_read_en)	// insert one clock gap
		last_read_en <= 1'b0;
	else if ((cur_state == FORCE_OUTPUT) && ~last_read_finish)
		last_read_en <= 1'b1;
	else
		last_read_en <= 1'b0;

assign last_read_finish = (last_read_addr == `MATCH_FILTER_DEPTH);

//----------------------------------------------------------
// coherent accumulation
//----------------------------------------------------------
// coherent buffer
wire [9:0] coh_buffer_addr;
wire [9:0] coh_ram_addr;
wire [191:0] coh_buffer_d4rd;
wire [191:0] coh_buffer_d4wt;
wire coh_buffer_rd;
wire coh_buffer_we;
wire coh_rd;
wire [3:0] max_exp;
wire next_twiddle;
//reg first_result;

assign coh_ram_addr = (cur_state == FORCE_OUTPUT) ? last_read_addr : coh_buffer_addr;
assign coh_rd = coh_buffer_rd | last_read_en;

spram #(.RAM_SIZE(`MATCH_FILTER_DEPTH), .ADDR_WIDTH(10), .DATA_WIDTH(192)) coh_buffer_sram
(
	.clk   (clk            ),
	.en    (coh_rd | coh_buffer_we),
	.we    (coh_buffer_we  ),
	.addr  (coh_ram_addr   ),
	.rdata (coh_buffer_d4rd),
	.wdata (coh_buffer_d4wt)
);

reg [1:0] coh_sum_segment;
reg [13:0] dft_nco;
reg [13:0] acc_nco1, acc_nco2;
reg [2:0] code_round_d;
wire inc_coh_count;
assign inc_coh_count = mf_segment_first && (coh_sum_segment == 2'b10);

// accumulate segment number at coherent sum stage
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coh_sum_segment <= 2'b00;
	else if (mf_coh_first)
		coh_sum_segment <= 2'b00;
	else if (mf_segment_first)
		coh_sum_segment <= {coh_sum_segment[0], (coh_sum_segment == 2'b00)};	// 00 -> 01 -> 10 -> 00

// accumulate DFT NCO
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		dft_nco <= 'd0;
	else if (mf_coh_first)	// reset DFT NCO on first coherent round
		dft_nco <= 'd0;
	else if (inc_coh_count)	// increase DFT NCO every 3 segment round
		dft_nco <= dft_nco + {3'b000, dft_freq};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		code_round_d  <= 3'b000;
	else
		code_round_d  <= {code_round_d[1:0], mf_coh_first || inc_coh_count};

// first_result of segment, first segment (SegmentCount == 0) and first_acc (CohCount == 0) signal generation align with data
reg mf_segment_first_d, mf_coh_first_d;
reg coh_round_first;
reg coh_first_result, coh_first_segment, coh_first_acc;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		mf_segment_first_d <= 1'b0;
	else
		mf_segment_first_d <= mf_segment_first;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		mf_coh_first_d <= 1'b0;
	else
		mf_coh_first_d <= mf_coh_first;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coh_round_first <= 1'b0;
	else if (mf_coh_first || inc_coh_count)
		coh_round_first <= 1'b1;
	else
		coh_round_first <= 1'b0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coh_first_result <= 1'b0;
	else
		coh_first_result <= mf_segment_first_d;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coh_first_segment <= 1'b0;
	else if (mf_segment_first_d) begin
		if (coh_sum_segment == 2'b00)
			coh_first_segment <= 1'b1;
		else
			coh_first_segment <= 1'b0;
	end

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coh_first_acc <= 1'b0;
	else if (coh_round_first) begin
		if (mf_coh_first_d)
			coh_first_acc <= 1'b1;
		else
			coh_first_acc <= 1'b0;
	end

// calculate acc_nco as 1/3/5/7 x dft_nco;
// accumulate DFT NCO
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		acc_nco1 <= 'd0;
	else if (code_round_d[0])
		acc_nco1 <= dft_nco;	// x1
	else if (code_round_d[1])
		acc_nco1 <= acc_nco1 + {dft_nco[11:0], 2'b00};	// x5

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		acc_nco2 <= 'd0;
	else if (code_round_d[0])
		acc_nco2 <= dft_nco + {dft_nco[12:0], 1'b0};	// x3
	else if (code_round_d[1])
		acc_nco2 <= acc_nco2 + {dft_nco[11:0], 2'b00};	// x7

// LUT for DFT sin/cos value
wire cos_sign1, sin_sign1, cos_sign2, sin_sign2;
wire [8:0] cos_mag1, sin_mag1, cos_mag2, sin_mag2;
sincos_lut_256x10 u1_sincos_lut_256x10
(
	.phase    (acc_nco1[13:6]),
	.cos_mag  (cos_mag1      ),
	.sin_mag  (sin_mag1      ),
	.cos_sign (cos_sign1     ),
	.sin_sign (sin_sign1     )
);

sincos_lut_256x10 u2_sincos_lut_256x10
(
	.phase    (acc_nco2[13:6]),
	.cos_mag  (cos_mag2      ),
	.sin_mag  (sin_mag2      ),
	.cos_sign (cos_sign2     ),
	.sin_sign (sin_sign2     )
);

// latch four sin/cos twiddle factor (one clock cycle delay to match filter output)
// match filter output will be latched in choherent sum module and align to twiddle factor
// for first acc, do not use twiddle factor, mute value to 0 to prevent multiplier logic toggle
reg sign_cos1, sign_sin1, sign_cos3, sign_sin3, sign_cos5, sign_sin5, sign_cos7, sign_sin7;
reg [8:0] mag_cos1, mag_sin1, mag_cos3, mag_sin3, mag_cos5, mag_sin5, mag_cos7, mag_sin7;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		{sign_cos1, mag_cos1, sign_sin1, mag_sin1} <= 20'd0;
	else if (code_round_d[1])
		{sign_cos1, mag_cos1, sign_sin1, mag_sin1} <= coh_first_acc ? 20'd0 : {cos_sign1, cos_mag1, ~sin_sign1, sin_mag1};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		{sign_cos3, mag_cos3, sign_sin3, mag_sin3} <= 20'd0;
	else if (code_round_d[1])
		{sign_cos3, mag_cos3, sign_sin3, mag_sin3} <= coh_first_acc ? 20'd0 : {cos_sign2, cos_mag2, ~sin_sign2, sin_mag2};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		{sign_cos5, mag_cos5, sign_sin5, mag_sin5} <= 20'd0;
	else if (code_round_d[2])
		{sign_cos5, mag_cos5, sign_sin5, mag_sin5} <= coh_first_acc ? 20'd0 : {cos_sign1, cos_mag1, ~sin_sign1, sin_mag1};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		{sign_cos7, mag_cos7, sign_sin7, mag_sin7} <= 20'd0;
	else if (code_round_d[2])
		{sign_cos7, mag_cos7, sign_sin7, mag_sin7} <= coh_first_acc ? 20'd0 : {cos_sign2, cos_mag2, ~sin_sign2, sin_mag2};

// coherent sum module
coh_acc #(.COH_DATA_NUMBER(`MATCH_FILTER_DEPTH)) u_coh_acc
(
	.clk              (clk              ),
	.rst_b            (rst_b            ),

	.cor_result_i     (mf_out_i         ),
	.cor_result_q     (mf_out_q         ),
	.cor_result_valid (mf_out_valid     ),

	.sign_cos1        (sign_cos1        ),
	.mag_cos1         (mag_cos1         ),
	.sign_sin1        (sign_sin1        ),
	.mag_sin1         (mag_sin1         ),
	.sign_cos3        (sign_cos3        ),
	.mag_cos3         (mag_cos3         ),
	.sign_sin3        (sign_sin3        ),
	.mag_sin3         (mag_sin3         ),
	.sign_cos5        (sign_cos5        ),
	.mag_cos5         (mag_cos5         ),
	.sign_sin5        (sign_sin5        ),
	.mag_sin5         (mag_sin5         ),
	.sign_cos7        (sign_cos7        ),
	.mag_cos7         (mag_cos7         ),
	.sign_sin7        (sign_sin7        ),
	.mag_sin7         (mag_sin7         ),

	.first_result     (coh_first_result ),
	.first_segment    (coh_first_segment),
	.first_acc        (coh_first_acc    ),

	.read_finish      (coh_read_finish  ),
	.write_finish     (coh_write_finish ),

	.max_exp          (max_exp          ),
	.rd               (coh_buffer_rd    ),
	.we               (coh_buffer_we    ),
	.addr             (coh_buffer_addr  ),
	.d4wt             (coh_buffer_d4wt  ),
	.d4rd             (coh_buffer_d4rd  )
);

//----------------------------------------------------------
// non-coherent sum
//----------------------------------------------------------
// generate control signals for noncoherent module
wire coh_first_rd, coh_last_rd;		// first sample read on first coherent acc round, last coherent acc read back is the same except for the very first one
reg noncoh_valid;
assign coh_first_rd = coh_rd && (coh_ram_addr == 'd0) && ((coh_first_segment && coh_first_acc) || (cur_state == FORCE_OUTPUT));
assign coh_last_rd = coh_first_rd && coh_read_to_noncoh;	// skip the first time coherent RAM read at beginning

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coh_read_to_noncoh <= 1'b0;
	else if (start_acquisition)
		coh_read_to_noncoh <= 1'b0;
	else if ((cur_state == WAIT_SEGMENT_ROUND) && success_flag)		// if current channel early terminate, should skip first acc read when process next channel
		coh_read_to_noncoh <= 1'b0;
	else if (coh_first_rd)
		coh_read_to_noncoh <= 1'b1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		noncoh_valid <= 1'b0;
	else if (coh_last_rd)
		noncoh_valid <= 1'b1;
	else if (coh_read_finish || last_read_finish)
		noncoh_valid <= 1'b0;

reg todo_first_acc;		// indicator for next acc to be first time noncoherent
reg noncoh_first_acc;
reg todo_last_acc;		// indicator for next acc to be last time noncoherent
reg noncoh_last_acc;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		todo_first_acc <= 1'b0;
	else if (start_acquisition)
		todo_first_acc <= 1'b0;
	else if (cur_state == NEXT_NONCOH && (noncoh_count == 'd1))
		todo_first_acc <= 1'b1;
	else if (noncoh_first_acc)
		todo_first_acc <= 1'b0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		noncoh_first_acc <= 1'b0;
	else if (coh_last_rd)
		noncoh_first_acc <= todo_first_acc;
	else if (coh_read_finish || last_read_finish)
		noncoh_first_acc <= 1'b0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		todo_last_acc <= 1'b0;
	else if (start_acquisition)
		todo_last_acc <= 1'b0;
	else if (cur_state == NEXT_NONCOH && (noncoh_count == noncoh_number))
		todo_last_acc <= 1'b1;
	else if (noncoh_last_acc)
		todo_last_acc <= 1'b0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		noncoh_last_acc <= 1'b0;
	else if (coh_last_rd)
		noncoh_last_acc <= todo_last_acc;
	else if (coh_read_finish || last_read_finish)
		noncoh_last_acc <= 1'b0;

reg [3:0] max_exp_r;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		max_exp_r <= 'd0;
	else if (coh_write_finish)
		max_exp_r <= max_exp;

// non-coherent RAM access signals
wire [9:0] noncoh_buffer_addr;
wire [63:0] noncoh_buffer_d4rd;
wire [63:0] noncoh_buffer_d4wt;
wire noncoh_buffer_rd;
wire noncoh_buffer_we;

wire [7:0] noncoh_out_amp;
wire [3:0] noncoh_out_exp;
wire [9:0] noncoh_out_pos;
wire [2:0] noncoh_out_freq;
wire noncoh_out_valid;
wire [17:0] noise_floor;
wire noise_floor_valid;

noncoh_acc #(.DATA_LENGTH(`MATCH_FILTER_DEPTH)) u_noncoh_acc
(
	.clk               (clk               ),
	.rst_b             (rst_b             ),

	.coh_rd_valid      (coh_rd            ),
	.coh_read_addr     (coh_ram_addr      ),
	.coh_d4rd          (coh_buffer_d4rd   ),

	.active_acc        (noncoh_valid      ),
	.first_acc         (noncoh_first_acc  ),
	.last_acc          (noncoh_last_acc   ),
	.max_exp           (max_exp_r         ),

	.noncoh_out_amp    (noncoh_out_amp    ),
	.noncoh_out_exp    (noncoh_out_exp    ),
	.noncoh_out_pos    (noncoh_out_pos    ),
	.noncoh_out_freq   (noncoh_out_freq   ),
	.noncoh_out_valid  (noncoh_out_valid  ),
	.noise_floor       (noise_floor       ),
//	.noise_floor_valid (noise_floor_valid ),

	.rd                (noncoh_buffer_rd  ),
	.we                (noncoh_buffer_we  ),
	.addr              (noncoh_buffer_addr),
	.d4wt              (noncoh_buffer_d4wt),
	.d4rd              (noncoh_buffer_d4rd)
);

spram #(.RAM_SIZE(682), .ADDR_WIDTH(10), .DATA_WIDTH(64)) noncoh_buffer_sram
(
	.clk               (clk               ),
	.en                (noncoh_buffer_rd | noncoh_buffer_we),
	.we                (noncoh_buffer_we  ),
	.addr              (noncoh_buffer_addr),
	.rdata             (noncoh_buffer_d4rd),
	.wdata             (noncoh_buffer_d4wt)
);

//----------------------------------------------------------
// peak sorter
//----------------------------------------------------------
// stride offset and code offset to calculate full code position
reg [5:0] stride_offset;
reg [14:0] todo_code_offset, cur_code_offset;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		stride_offset <= 'd0;
	else if (start_acquisition)
		stride_offset <= 'd0;
	else if ((cur_state == NEXT_COH) && (coh_count == coh_number))
		stride_offset <= ({1'b0, stride_count[5:1]} ^ {6{~stride_count[0]}}) + 1;		// convert 0,1,2,3,4... to 0,1,-1,2,-2...

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		todo_code_offset <= 'd0;
	else if (cur_state == NEXT_STRIDE)
		todo_code_offset <= 'd0;
	else if ((cur_state == NEXT_NONCOH) && (noncoh_count == noncoh_number))
		todo_code_offset <= todo_code_offset + 682;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cur_code_offset <= 'd0;
	else if ((cur_state == NEXT_NONCOH) && (noncoh_count != 0))
		cur_code_offset <= todo_code_offset;

// latch value insert into peak sorter
reg [7:0] insert_amp;
reg [3:0] insert_exp;
reg [14:0] full_code_pos;
reg [8:0] full_freq_pos;
reg insert_valid;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		insert_amp <= 'd0;
	else if (noncoh_out_valid)
		insert_amp <= noncoh_out_amp;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		insert_exp <= 'd0;
	else if (noncoh_out_valid)
		insert_exp <= noncoh_out_exp;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		full_code_pos <= 'd0;
	else if (noncoh_out_valid)
		full_code_pos <= cur_code_offset + noncoh_out_pos;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		full_freq_pos <= 'd0;
	else if (noncoh_out_valid)
		full_freq_pos <= {stride_offset, noncoh_out_freq};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		insert_valid <= 1'b0;
	else
		insert_valid <= noncoh_out_valid;

// generate clear signal
// first segment round on each channel will output noncoherent result to peak sorter
// consider processing delay, skip the second segment round
// clear peak sorter on start of the third segment round
wire clear_peak;
assign clear_peak = (cur_state == NEXT_SEGMENT) && (segment_count == 2'b10) && (coh_count == 0) && (noncoh_count == 0) && (code_round == 1) && (stride_count == 0);

wire [7:0] peak1_amp, peak2_amp, peak3_amp;
wire [8:0] peak1_freq, peak2_freq, peak3_freq;
wire [14:0] peak1_cor, peak2_cor, peak3_cor;
wire [3:0] peak_exp;

peak_sorter u_peak_sorter
(
	.clk               (clk             ),
	.rst_b             (rst_b           ),

	.clear             (clear_peak      ),

	.input_amp         (insert_amp      ),
	.input_exp         (insert_exp      ),
	.code_pos          (full_code_pos   ),
	.freq_pos          (full_freq_pos   ),
	.peak_valid        (insert_valid    ),

	.peak1_amp         (peak1_amp       ),
	.peak2_amp         (peak2_amp       ),
	.peak3_amp         (peak3_amp       ),
	.peak1_freq        (peak1_freq      ),
	.peak2_freq        (peak2_freq      ),
	.peak3_freq        (peak3_freq      ),
	.peak1_cor         (peak1_cor       ),
	.peak2_cor         (peak2_cor       ),
	.peak3_cor         (peak3_cor       ),
	.peak_exp          (peak_exp        )
);

//----------------------------------------------------------
// result determination state machine
//----------------------------------------------------------
// determine threshold
reg last_insert, peak_final;
reg [2:0] peak_ratio_r;
reg early_terminate_r;
reg [8:0] peak_threshold;
wire success;
assign success = ({1'b0, peak1_amp} >= peak_threshold);

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		last_insert <= 1'b0;
	else
		last_insert <= insert_valid && (noncoh_out_pos == (`MATCH_FILTER_DEPTH-1));

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		peak_final <= 1'b0;
	else
		peak_final <= last_insert;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		peak_ratio_r <= 3'd0;
	else if (clear_peak)
		peak_ratio_r <= peak_ratio;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		early_terminate_r <= 1'b0;
	else if (clear_peak)
		early_terminate_r <= early_terminate;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		peak_threshold <= 9'd1;
	else if (clear_peak)
		peak_threshold <= 9'd1;
	else if (peak_final)
		peak_threshold <= {1'b0, peak3_amp} + {4'h0, peak3_amp[7:3]} + 1 +
		                   (peak_ratio_r[0] ? {4'h0, peak3_amp[7:3]} : 0) +
		                   (peak_ratio_r[1] ? {3'h0, peak3_amp[7:2]} : 0) +
		                   (peak_ratio_r[2] ? {2'h0, peak3_amp[7:1]} : 0);

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		channel_save <= 'd0;
	else if (clear_peak)
		channel_save <= channel_count;

// determine end of one channel search
reg last_stride;
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		last_stride <= 1'b0;
	else if (start_acquisition)
		last_stride <= 1'b0;
	else if ((cur_state == NEXT_STRIDE) && (stride_count == stride_number))
		last_stride <= 1'b1;
	else if (clear_peak)
		last_stride <= 1'b0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		channel_save_pending <= 1'b0;
	else if (start_acquisition)
		channel_save_pending <= 1'b0;
	else if (last_read_finish)
		channel_save_pending <= 1'b1;
	else if ((result_state == RESULT_SAVE) && (channel_param_addr == 3'b111))
		channel_save_pending <= 1'b0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		success_flag <= 1'b0;
	else if (cur_state == NEXT_CHANNEL)
		success_flag <= 1'b0;
	else if (result_state == RESULT_DETERMINE)
		success_flag <= success && early_terminate_r;

// result state machine
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		result_state <= RESULT_IDLE;
	else  
		result_state <= result_next_state;

always @(*)
	case(result_state)
	RESULT_IDLE:
		result_next_state = peak_final ? RESULT_DETERMINE : RESULT_IDLE;
	RESULT_DETERMINE:
		result_next_state = ((success && early_terminate_r) | last_stride) ? RESULT_SAVE : RESULT_IDLE;
	RESULT_SAVE:
		result_next_state = (channel_param_addr == 3'b111) ? RESULT_IDLE : RESULT_SAVE;
	default:
		result_next_state = RESULT_IDLE;
	endcase

// latch peak sorter result for output
reg [31:0] peak_value1, peak_value2, peak_value3;
reg [3:0] global_exp;
reg [18:0] noise_floor_save;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		peak_value1 <= 'd0;
	else if (peak_final)
		peak_value1 <= {peak1_amp, peak1_freq, peak1_cor};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		peak_value2 <= 'd0;
	else if (peak_final)
		peak_value2 <= {peak2_amp, peak2_freq, peak2_cor};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		peak_value3 <= 'd0;
	else if (peak_final)
		peak_value3 <= {peak3_amp, peak3_freq, peak3_cor};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		global_exp <= 4'd0;
	else if (peak_final)
		global_exp <= peak_exp;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		noise_floor_save <= 'd0;
	else if (peak_final)
		noise_floor_save <= noise_floor;

// write result back to channel config buffer
always @(*)
	case(channel_param_addr[1:0])
	2'd0:
		config_buffer_d4wt = {success, 3'h0, global_exp, 5'h00, noise_floor_save};
	2'd1:
		config_buffer_d4wt = peak_value1;
	2'd2:
		config_buffer_d4wt = peak_value2;
	2'd3:
		config_buffer_d4wt = peak_value3;
	default:
		config_buffer_d4wt = 32'd0;
	endcase

endmodule
