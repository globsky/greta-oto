//----------------------------------------------------------------------
// tracking_engine.v:
//   Tracking engine module
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module tracking_engine
(
// system signals
input clk,   // system clock
input rst_b, // reset signal, low active

// signal of TE control and state
input te_start,
output te_running,
output te_ready,
output te_over,

// interface to TE FIFO
input fifo_ready,					// fifo data ready
input fifo_last_data,			// indicate the last data of FIFO
output reg fifo_read,			// read FIFO command, indicate TE ready to accept data
output reg fifo_rewind,		// FIFO read address rewind
output reg fifo_skip,			// FIFO skip one block
input fifo_data_valid,		// FIFO data valid signal
input [7:0] fifo_data,		// FIFO data input to TE

// control
input te_reg_cs,	  // TE register
input te_buffer_cs,	// TE buffer
input te_rd,  			// TE host read valid
input te_wr,			  // TE host write valid
input [13:0] te_addr,  // TE host access address
input [31:0] te_d4wt,  // TE host write data
output [31:0] te_rd_buffer,	// TE host read data from TE buffer, valid next cycle of te_rd
output reg [31:0] te_reg_d4rd,	  // TE host read data, valid same cycle of te_rd

// interface for Legendre and memory code buffer
output [10:0] legendre_addr,	// MSB is weil type
output legendre_rd,
input [15:0] legendre_data,
output [13:0] memcode_addr,
output memcode_rd,
input [31:0] memcode_data
);

genvar i_gen;

reg [3:0] cur_te_state;
reg [3:0] next_te_state;

// states
localparam
		IDLE           = 4'd0,
		START          = 4'd1,
		FIND_CHANNEL   = 4'd2,
		FILL_STATE     = 4'd3,
		WAIT_COR_READY = 4'd4,
		READ_FIFO      = 4'd5,
		CORRELATION    = 4'd6,
		COHERENT_SUM   = 4'd7,
		DUMP_STATE     = 4'd8,
		COR_FINISH     = 4'd9,
		TE_FINISH      = 4'd10,
		WAIT_CPU       = 4'd11,
		STAND_BY       = 4'd12;

//----------------------------------------------------------
// registers value read/write
//----------------------------------------------------------
reg [31:0] te_channel_enable;
reg [31:0] te_coh_data_ready;
reg [31:0] te_polynomial;
reg [31:0] te_code_length;
reg [31:0] te_polynomial2;
reg [31:0] te_code_length2;
reg [31:0] te_channel_ow_protect;
reg [9:0] te_ow_protect_addr;
reg [31:0] te_ow_protect_value;

// write registers
always @(posedge clk or negedge rst_b)
	if (!rst_b) begin
		te_channel_enable <= 32'h0;
		te_polynomial     <= 32'h0;
		te_code_length    <= 32'h0;
		te_polynomial2    <= 32'h0;
		te_code_length2   <= 32'h0;
	end
	else if (te_reg_cs & te_wr)
	begin
		case(te_addr)
			`TE_CHANNEL_ENABLE_ADDR: te_channel_enable <= te_d4wt;
			`TE_POLYNOMIAL_ADDR    : te_polynomial     <= te_d4wt;
			`TE_CODE_LENGTH_ADDR   : te_code_length    <= te_d4wt;
			`TE_POLYNOMIAL2_ADDR   : te_polynomial2    <= te_d4wt;
			`TE_CODE_LENGTH2_ADDR  : te_code_length2   <= te_d4wt;
		endcase
	end

//read registers
always @ (*) begin
	if (te_reg_cs & te_rd)
		case(te_addr)
			`TE_CHANNEL_ENABLE_ADDR   : te_reg_d4rd = te_channel_enable;
			`TE_COH_DATA_READY_ADDR   : te_reg_d4rd = te_coh_data_ready;
			`TE_OW_PROTECT_CHAN_ADDR  : te_reg_d4rd = te_channel_ow_protect;
			`TE_OW_PROTECT_ADDR_ADDR  : te_reg_d4rd = {20'h0, te_ow_protect_addr, 2'b00};
			`TE_OW_PROTECT_VALUE_ADDR : te_reg_d4rd = te_ow_protect_value;
			`TE_POLYNOMIAL_ADDR       : te_reg_d4rd = te_polynomial;
			`TE_CODE_LENGTH_ADDR      : te_reg_d4rd = te_code_length;
			`TE_POLYNOMIAL2_ADDR      : te_reg_d4rd = te_polynomial2;
			`TE_CODE_LENGTH2_ADDR     : te_reg_d4rd = te_code_length2;
			`TE_CURR_STATE_MACHINE    : te_reg_d4rd = {12'h0, next_te_state, 12'h0, cur_te_state};
			default                   : te_reg_d4rd = 32'h0;
		endcase
	else
		te_reg_d4rd = 32'h0;
end

//----------------------------------------------------------
// state machine
//----------------------------------------------------------
wire find_channel_done;
wire fill_state_done;
wire cor_ready_all;
wire cor_done;
wire coherent_sum_done;
wire dump_state_done;
wire te_done;
wire dumping_state;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cur_te_state <= STAND_BY;
	else
		cur_te_state <= next_te_state;

always @ (*) begin
	case(cur_te_state)
		STAND_BY       : next_te_state = te_start ? IDLE : STAND_BY;
		IDLE           : next_te_state = fifo_ready ? START : IDLE;
		START          : next_te_state = (|te_channel_enable) ? FIND_CHANNEL : TE_FINISH;
		FIND_CHANNEL   : next_te_state = find_channel_done ? FILL_STATE : FIND_CHANNEL;
		FILL_STATE     : next_te_state = fill_state_done ? WAIT_COR_READY : FILL_STATE;
		WAIT_COR_READY : next_te_state = cor_ready_all ? READ_FIFO : WAIT_COR_READY;
		READ_FIFO      : next_te_state = CORRELATION;
		CORRELATION    : next_te_state = cor_done ? COHERENT_SUM : CORRELATION;
		COHERENT_SUM   : next_te_state = coherent_sum_done ? DUMP_STATE : COHERENT_SUM;
		DUMP_STATE     : next_te_state = dump_state_done ? COR_FINISH : DUMP_STATE;
		COR_FINISH     : next_te_state = te_done ? TE_FINISH : FIND_CHANNEL;
		TE_FINISH      : next_te_state = WAIT_CPU;
		WAIT_CPU       : next_te_state = te_start ? IDLE : WAIT_CPU;
		default        : next_te_state = STAND_BY;
	endcase
end

assign te_over  = (cur_te_state == TE_FINISH);
assign te_running = (cur_te_state != WAIT_CPU);
assign dumping_state = (cur_te_state == DUMP_STATE);

// wait 5 clock cycles after last FIFO data for correlator finish
reg [4:0] fifo_last_data_d;
always @(posedge clk or negedge rst_b)
	if (~rst_b)
		fifo_last_data_d <= 'h0;
	else
		fifo_last_data_d <= {fifo_last_data_d[3:0], fifo_last_data};

assign cor_done = fifo_last_data_d[4];

//----------------------------------------------------------
// find valid channel
//----------------------------------------------------------
wire latch_enable_channel;
wire find_channel_start;
wire [31:0] channel_remain;
assign latch_enable_channel = (next_te_state == START);
assign find_channel_start = (cur_te_state != FIND_CHANNEL) && (next_te_state == FIND_CHANNEL);

wire [3:0] physical_channel_en;
wire [4:0] logic_channel_index [3:0];
wire [31:0] logic_channel_mask [3:0];

find_channel u_find_channel
(
	.clk                  (clk                   ),
	.rst_b                (rst_b                 ),
	.latch_enable_channel (latch_enable_channel  ),
	.start_find           (find_channel_start    ),
	.find_channel_done    (find_channel_done     ),
	.te_channel_enable    (te_channel_enable     ),
	.channel_remain       (channel_remain        ),
	.physical_channel_en  (physical_channel_en   ),
	.logic_channel_index0 (logic_channel_index[0]),
	.logic_channel_index1 (logic_channel_index[1]),
	.logic_channel_index2 (logic_channel_index[2]),
	.logic_channel_index3 (logic_channel_index[3]),
	.logic_channel_mask0  (logic_channel_mask[0] ),
	.logic_channel_mask1  (logic_channel_mask[1] ),
	.logic_channel_mask2  (logic_channel_mask[2] ),
	.logic_channel_mask3  (logic_channel_mask[3] )
);

assign te_done = ~(|channel_remain);

//----------------------------------------------------------
// fill and dump state buffer
//----------------------------------------------------------
wire fill_start;
wire dump_start;
assign fill_start = (cur_te_state == FIND_CHANNEL) && find_channel_done;
assign dump_start = (cur_te_state == COHERENT_SUM) && coherent_sum_done;

wire [1:0] physical_channel_index;
wire state_rd;
wire state_wr;
wire [9:0] state_addr;
wire [31:0] state_d4rd;

fill_dump_ctrl u_fill_dump_ctrl
(
	.clk                    (clk                   ),
	.rst_b                  (rst_b                 ),

	.physical_channel_en    (physical_channel_en   ),
	.logic_channel_index0   (logic_channel_index[0]),
	.logic_channel_index1   (logic_channel_index[1]),
	.logic_channel_index2   (logic_channel_index[2]),
	.logic_channel_index3   (logic_channel_index[3]),

	.fill_start             (fill_start            ),
	.dump_start             (dump_start            ),

	.physical_channel_index (physical_channel_index),
	.fill_state_done        (fill_state_done       ),
	.dump_state_done        (dump_state_done       ),
	.state_rd               (state_rd              ),
	.state_wr               (state_wr              ),
	.state_addr             (state_addr            )
);

// fill state signals and fill state module instances
wire [3:0] prn_state_en;
wire [3:0] prn_count_en;
wire [3:0] carrier_phase_en;
wire [3:0] carrier_count_en;
wire [3:0] code_phase_en;
wire [3:0] prn_code_load_en;
wire [3:0] corr_state_load_en;
wire [3:0] ms_data_sum_en;
wire [3:0] prn2_state_en;
wire [3:0] acc_en;

wire [31:0]  carrier_freq [3:0];
wire [31:0]  code_freq [3:0];
wire [1:0]   pre_shift_bits [3:0];
wire enable_boc [3:0];
wire data_in_q [3:0];
wire enable_2nd_prn [3:0];
wire [1:0]  narrow_factor [3:0];
wire [15:0] dump_length [3:0];
wire [24:0] nh_code [3:0];
wire [4:0] 	nh_length [3:0];
wire [19:0] nh_code2 [3:0];
wire [4:0]  ms_data_number [3:0];
wire [4:0]	coherent_number [3:0];
wire [1:0]  post_shift_bits [3:0];
wire [31:0] prn_config [3:0];
wire [31:0] prn2_config [3:0];

reg [3:0] cur_fill_select;
always @(*) begin
	case (physical_channel_index)
		2'b00:   cur_fill_select = 4'b0001;
		2'b01:   cur_fill_select = 4'b0010;
		2'b10:   cur_fill_select = 4'b0100;
		2'b11:   cur_fill_select = 4'b1000;
		default: cur_fill_select = 4'b0000;
	endcase
end

generate
	for (i_gen = 0; i_gen < 4; i_gen = i_gen + 1)
	begin: fill_state_gen
		fill_state u_fill_state
		(
			.clk                (clk                        ),
			.rst_b              (rst_b                      ),

			.fill_enable        (cur_fill_select[i_gen]     ),
			.state_rd           (state_rd                   ),
			.state_addr         (state_addr[4:0]            ),
			.state_d4rd         (state_d4rd                 ),

			.carrier_freq       (carrier_freq[i_gen]        ),
			.code_freq          (code_freq[i_gen]           ),
			.pre_shift_bits     (pre_shift_bits[i_gen]      ),
			.enable_boc         (enable_boc[i_gen]          ),
			.data_in_q          (data_in_q[i_gen]           ),
			.enable_2nd_prn     (enable_2nd_prn[i_gen]      ),
			.narrow_factor      (narrow_factor[i_gen]       ),
			.dump_length        (dump_length[i_gen]         ),
			.nh_code            (nh_code[i_gen]             ),
			.nh_length          (nh_length[i_gen]           ),
			.nh_code2           (nh_code2[i_gen]            ),
			.ms_data_number     (ms_data_number[i_gen]      ),
			.coherent_number    (coherent_number[i_gen]     ),
			.post_shift_bits    (post_shift_bits[i_gen]     ),
			.prn_config         (prn_config[i_gen]          ),
			.prn2_config        (prn2_config[i_gen]         ),

			.prn_state_en       (prn_state_en[i_gen]        ),
			.prn_count_en       (prn_count_en[i_gen]        ),
			.carrier_phase_en   (carrier_phase_en[i_gen]    ),
			.carrier_count_en   (carrier_count_en[i_gen]    ),
			.code_phase_en      (code_phase_en[i_gen]       ),
			.prn_code_load_en   (prn_code_load_en[i_gen]    ),
			.corr_state_load_en (corr_state_load_en[i_gen]  ),
			.ms_data_sum_en     (ms_data_sum_en[i_gen]      ),
			.prn2_state_en      (prn2_state_en[i_gen]       ),
			.acc_en             (acc_en[i_gen]              )
		);
	end
endgenerate

// dump state signals and dump state module instance
wire [31:0] state_d4wt;
wire [31:0] prn_state [3:0];
wire [31:0] prn_count [3:0];
wire [31:0] carrier_phase [3:0];
wire [31:0] carrier_count [3:0];
wire [31:0] code_phase [3:0];
wire [15:0] dump_count [3:0];
wire [7:0]  jump_count [3:0];
wire [7:0]  prn_code [3:0];
wire [4:0]  nh_count [3:0];
wire [4:0]  coherent_count [3:0];
wire [4:0]  ms_data_count [3:0];
wire [3:0]  prn_code2 [3:0];
wire [2:0]  current_cor [3:0];
wire [3:0]  code_sub_phase;
wire [3:0]  dumping;
wire [3:0]  msdata_done;
wire [3:0]  coherent_done;
wire [15:0] ms_data_sum [3:0];
wire [31:0] prn2_state [3:0];
wire [15:0] i_acc [3:0];
wire [15:0] q_acc [3:0];

dump_state u_dump_state
(
		.clk                    (clk                       ),
		.rst_b                  (rst_b                     ),
	
		.physical_channel_index (physical_channel_index    ),
		.state_addr             (state_addr[4:0]           ),
		.state_d4wt             (state_d4wt                ),
	
		.prn_state_0            (prn_state[0]              ),
		.prn_count_0            (prn_count[0]              ),
		.carrier_phase_0        (carrier_phase[0]          ),
		.carrier_count_0        (carrier_count[0]          ),
		.code_phase_0           (code_phase[0]             ),
		.dump_count_0           (dump_count[0]             ),
		.jump_count_0           (jump_count[0]             ),
		.prn_code_0             (prn_code[0]               ),
		.nh_count_0             (nh_count[0]               ),
		.coherent_count_0       (coherent_count[0]         ),
		.ms_data_count_0        (ms_data_count[0]          ),
		.prn_code2_0            (prn_code2[0]              ),
		.current_cor_0          (current_cor[0]            ),
		.code_sub_phase_0       (code_sub_phase[0]         ),
		.dumping_0              (dumping[0]                ),
		.msdata_done_0          (msdata_done[0]            ),
		.coherent_done_0        (coherent_done[0]          ),
		.ms_data_sum_0          (ms_data_sum[0]            ),
		.prn2_state_0           (prn2_state[0]             ),
		.i_acc_0                (i_acc[0]                  ),
		.q_acc_0                (q_acc[0]                  ),

		.prn_state_1            (prn_state[1]              ),
		.prn_count_1            (prn_count[1]              ),
		.carrier_phase_1        (carrier_phase[1]          ),
		.carrier_count_1        (carrier_count[1]          ),
		.code_phase_1           (code_phase[1]             ),
		.dump_count_1           (dump_count[1]             ),
		.jump_count_1           (jump_count[1]             ),
		.prn_code_1             (prn_code[1]               ),
		.nh_count_1             (nh_count[1]               ),
		.coherent_count_1       (coherent_count[1]         ),
		.ms_data_count_1        (ms_data_count[1]          ),
		.prn_code2_1            (prn_code2[1]              ),
		.current_cor_1          (current_cor[1]            ),
		.code_sub_phase_1       (code_sub_phase[1]         ),
		.dumping_1              (dumping[1]                ),
		.msdata_done_1          (msdata_done[1]            ),
		.coherent_done_1        (coherent_done[1]          ),
		.ms_data_sum_1          (ms_data_sum[1]            ),
		.prn2_state_1           (prn2_state[1]             ),
		.i_acc_1                (i_acc[1]                  ),
		.q_acc_1                (q_acc[1]                  ),

		.prn_state_2            (prn_state[2]              ),
		.prn_count_2            (prn_count[2]              ),
		.carrier_phase_2        (carrier_phase[2]          ),
		.carrier_count_2        (carrier_count[2]          ),
		.code_phase_2           (code_phase[2]             ),
		.dump_count_2           (dump_count[2]             ),
		.jump_count_2           (jump_count[2]             ),
		.prn_code_2             (prn_code[2]               ),
		.nh_count_2             (nh_count[2]               ),
		.coherent_count_2       (coherent_count[2]         ),
		.ms_data_count_2        (ms_data_count[2]          ),
		.prn_code2_2            (prn_code2[2]              ),
		.current_cor_2          (current_cor[2]            ),
		.code_sub_phase_2       (code_sub_phase[2]         ),
		.dumping_2              (dumping[2]                ),
		.msdata_done_2          (msdata_done[2]            ),
		.coherent_done_2        (coherent_done[2]          ),
		.ms_data_sum_2          (ms_data_sum[2]            ),
		.prn2_state_2           (prn2_state[2]             ),
		.i_acc_2                (i_acc[2]                  ),
		.q_acc_2                (q_acc[2]                  ),

		.prn_state_3            (prn_state[3]              ),
		.prn_count_3            (prn_count[3]              ),
		.carrier_phase_3        (carrier_phase[3]          ),
		.carrier_count_3        (carrier_count[3]          ),
		.code_phase_3           (code_phase[3]             ),
		.dump_count_3           (dump_count[3]             ),
		.jump_count_3           (jump_count[3]             ),
		.prn_code_3             (prn_code[3]               ),
		.nh_count_3             (nh_count[3]               ),
		.coherent_count_3       (coherent_count[3]         ),
		.ms_data_count_3        (ms_data_count[3]          ),
		.prn_code2_3            (prn_code2[3]              ),
		.current_cor_3          (current_cor[3]            ),
		.code_sub_phase_3       (code_sub_phase[3]         ),
		.dumping_3              (dumping[3]                ),
		.msdata_done_3          (msdata_done[3]            ),
		.coherent_done_3        (coherent_done[3]          ),
		.ms_data_sum_3          (ms_data_sum[3]            ),
		.prn2_state_3           (prn2_state[3]             ),
		.i_acc_3                (i_acc[3]                  ),
		.q_acc_3                (q_acc[3]                  )
);

//----------------------------------------------------------
// correlator
//----------------------------------------------------------
//correlation instance
wire [3:0] acc_dump_en;
assign acc_dump_en = physical_channel_en & {4{state_addr[4]}};

wire [9:0] cor_legendre_addr [3:0];
wire [3:0] cor_legendre_rd;
wire [3:0] cor_legendre_read_valid;
wire [15:0] cor_legendre_data;
wire [13:0] cor_memcode_addr [3:0];
wire [3:0] cor_memcode_rd;
wire [3:0] cor_memcode_read_valid;
wire [31:0] cor_memcode_data;

wire [ 4:0] cor_dump_index [3:0];
wire [15:0] i_coherent_sum [3:0];
wire [15:0] q_coherent_sum [3:0];
wire [3:0] coherent_sum_valid;
wire [3:0] cor_ready;
wire [3:0] overwrite_protect;

generate
	for (i_gen = 0; i_gen < 4; i_gen = i_gen + 1)
	begin: correlator_gen
	correlator u_correlator
	(
		.clk                    (clk                       ),
		.rst_b                  (rst_b                     ),
	
		.fifo_data_en           (fifo_data_valid           ),
		.fifo_data              (fifo_data                 ),
	
		.carrier_freq           (carrier_freq[i_gen]       ),
		.code_freq              (code_freq[i_gen]          ),
		.pre_shift_bits         (pre_shift_bits[i_gen]     ),
		.enable_boc             (enable_boc[i_gen]         ),
		.data_in_q              (data_in_q[i_gen]          ),
		.enable_2nd_prn         (enable_2nd_prn[i_gen]     ),
		.narrow_factor          (narrow_factor[i_gen]      ),
		.dump_length            (dump_length[i_gen]        ),
		.nh_code                (nh_code[i_gen]            ),
		.nh_length              (nh_length[i_gen]          ),
		.nh_code2               (nh_code2[i_gen]           ),
		.ms_data_number         (ms_data_number[i_gen]     ),
		.coherent_number        (coherent_number[i_gen]    ),
		.post_shift_bits        (post_shift_bits[i_gen]    ),
	
		.carrier_phase_en       (carrier_phase_en[i_gen]   ),
		.carrier_phase_i        (state_d4rd                ),
		.carrier_count_en       (carrier_count_en[i_gen]   ),
		.carrier_count_i        (state_d4rd                ),
		.code_phase_en          (code_phase_en[i_gen]      ),
		.code_phase_i           (state_d4rd                ),
		.carrier_phase_o        (carrier_phase[i_gen]      ),
		.carrier_count_o        (carrier_count[i_gen]      ),
		.code_phase_o           (code_phase[i_gen]         ),
	
		.prn_code_load_en       (prn_code_load_en[i_gen]   ),
		.dump_count_i           (state_d4rd[31:16]         ),
		.jump_count_i           (state_d4rd[15:8]          ),
		.prn_code_i             (state_d4rd[7:0]           ),
		.dump_count_o           (dump_count[i_gen]         ),
		.jump_count_o           (jump_count[i_gen]         ),
		.prn_code_o             (prn_code[i_gen]           ),

		.corr_state_load_en     (corr_state_load_en[i_gen] ),
		.nh_count_i             (state_d4rd[31:27]         ),
		.coherent_count_i       (state_d4rd[25:21]         ),
		.ms_data_count_i        (state_d4rd[20:16]         ),
		.prn_code2_i            (state_d4rd[15:12]         ),
		.current_cor_i          (state_d4rd[6:4]           ),
		.code_sub_phase_i       (state_d4rd[8]             ),
		.dumping_i              (state_d4rd[7]             ),
		.nh_count_o             (nh_count[i_gen]           ),
		.coherent_count_o       (coherent_count[i_gen]     ),
		.ms_data_count_o        (ms_data_count[i_gen]      ),
		.prn_code2_o            (prn_code2[i_gen]          ),
		.current_cor_o          (current_cor[i_gen]        ),
		.code_sub_phase_o       (code_sub_phase[i_gen]     ),
		.dumping_o              (dumping[i_gen]            ),

		.ms_data_sum_en         (ms_data_sum_en[i_gen]     ),
		.ms_data_sum_i          (state_d4rd[15:0]          ),
		.ms_data_sum_o          (ms_data_sum[i_gen]        ),

		.acc_en                 (acc_en[i_gen]             ),
		.acc_dump               (~state_addr[3] & state_addr[4]),
		.acc_sel                (state_addr[2:0]           ),
		.i_acc_i                (state_d4rd[31:16]         ),
		.q_acc_i                (state_d4rd[15:0]          ),
		.i_acc_o                (i_acc[i_gen]              ),
		.q_acc_o                (q_acc[i_gen]              ),
	
		.te_polynomial          (te_polynomial             ),
		.te_code_length         (te_code_length            ),
		.te_polynomial2         (te_polynomial2            ),
		.te_code_length2        (te_code_length2           ),
		.prn_config             (prn_config[i_gen]         ),
		.prn2_config            (prn2_config[i_gen]        ),
		.prn_state_en           (prn_state_en[i_gen]       ),
		.prn_state_i            (state_d4rd                ),
		.prn_state_o            (prn_state[i_gen]          ),
		.prn_count_en           (prn_count_en[i_gen]       ),
		.prn_count_i            (state_d4rd                ),
		.prn_count_o            (prn_count[i_gen]          ),
		.prn2_state_en          (prn2_state_en[i_gen]      ),
		.prn2_state_i           (state_d4rd                ),
		.prn2_state_o           (prn2_state[i_gen]         ),
	
		.legendre_addr          (cor_legendre_addr[i_gen]  ),
		.legendre_rd            (cor_legendre_rd[i_gen]    ),
		.legendre_read_valid    (cor_legendre_read_valid[i_gen]),
		.legendre_data          (cor_legendre_data         ),
	
		.memcode_addr           (cor_memcode_addr[i_gen]   ),
		.memcode_rd             (cor_memcode_rd[i_gen]     ),
		.memcode_read_valid     (cor_memcode_read_valid[i_gen]),
		.memcode_data           (cor_memcode_data   ),

		.cor_dump_index         (cor_dump_index[i_gen]     ),
		.i_coherent_sum         (i_coherent_sum[i_gen]     ),
		.q_coherent_sum         (q_coherent_sum[i_gen]     ),
		.coherent_sum_valid     (coherent_sum_valid[i_gen] ),
	
		.fill_finished          (fill_state_done           ),
		.cor_ready              (cor_ready[i_gen]          ),
		.msdata_done_o          (msdata_done[i_gen]        ),
		.coherent_done_o        (coherent_done[i_gen]      ),
		.overwrite_protect      (overwrite_protect[i_gen]  )
	);
	end
endgenerate

// ROM arbiter for Legendre code and memory code
m_rom_arbiter #(.ADDR_WIDTH(11), .DATA_WIDTH(16)) rom_arbiter_legendre
(
		.mem_rd0_i (cor_legendre_rd[0]),
		.mem_addr0_i ({prn_config[0][29], cor_legendre_addr[0]}),
		.mem_accept0_o (cor_legendre_read_valid[0]),
		.mem_rd1_i (cor_legendre_rd[1]),
		.mem_addr1_i ({prn_config[1][29], cor_legendre_addr[1]}),
		.mem_accept1_o (cor_legendre_read_valid[1]),
		.mem_rd2_i (cor_legendre_rd[2]),
		.mem_addr2_i ({prn_config[2][29], cor_legendre_addr[2]}),
		.mem_accept2_o (cor_legendre_read_valid[2]),
		.mem_rd3_i (cor_legendre_rd[3]),
		.mem_addr3_i ({prn_config[3][29], cor_legendre_addr[3]}),
		.mem_accept3_o (cor_legendre_read_valid[3]),
		.mem_d4rd_o (cor_legendre_data),
		.mem_rd_o (legendre_rd),
		.mem_addr_o (legendre_addr),
		.mem_accept_i (1'b1),
		.mem_d4rd_i (legendre_data)
);

m_rom_arbiter #(.ADDR_WIDTH(14), .DATA_WIDTH(32)) rom_arbiter_memcode
(
		.mem_rd0_i (cor_memcode_rd[0]),
		.mem_addr0_i (cor_memcode_addr[0]),
		.mem_accept0_o (cor_memcode_read_valid[0]),
		.mem_rd1_i (cor_memcode_rd[1]),
		.mem_addr1_i (cor_memcode_addr[1]),
		.mem_accept1_o (cor_memcode_read_valid[1]),
		.mem_rd2_i (cor_memcode_rd[2]),
		.mem_addr2_i (cor_memcode_addr[2]),
		.mem_accept2_o (cor_memcode_read_valid[2]),
		.mem_rd3_i (cor_memcode_rd[3]),
		.mem_addr3_i (cor_memcode_addr[3]),
		.mem_accept3_o (cor_memcode_read_valid[3]),
		.mem_d4rd_o (cor_memcode_data),
		.mem_rd_o (memcode_rd),
		.mem_addr_o (memcode_addr),
		.mem_accept_i (1'b1),
		.mem_d4rd_i (memcode_data)
);

assign cor_ready_all = cor_ready[0] & cor_ready[1] & cor_ready[2] & cor_ready[3];

//----------------------------------------------------------
// coherent data ready and overwrite protect registers
//----------------------------------------------------------
wire [31:0] coh_ready_channel;
wire [31:0] ow_protect_channel;
assign coh_ready_channel = ((coherent_done[0] & physical_channel_en[0]) ? logic_channel_mask[0] : 32'h0) |
                           ((coherent_done[1] & physical_channel_en[1]) ? logic_channel_mask[1] : 32'h0) |
                           ((coherent_done[2] & physical_channel_en[2]) ? logic_channel_mask[2] : 32'h0) |
                           ((coherent_done[3] & physical_channel_en[3]) ? logic_channel_mask[3] : 32'h0);
assign ow_protect_channel = ((overwrite_protect[0] & physical_channel_en[0]) ? logic_channel_mask[0] : 32'h0) |
                            ((overwrite_protect[1] & physical_channel_en[1]) ? logic_channel_mask[1] : 32'h0) |
                            ((overwrite_protect[2] & physical_channel_en[2]) ? logic_channel_mask[2] : 32'h0) |
                            ((overwrite_protect[3] & physical_channel_en[3]) ? logic_channel_mask[3] : 32'h0);

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		te_coh_data_ready <= 32'h0;
	else if (te_reg_cs & te_wr & (te_addr == `TE_COH_DATA_READY_ADDR))
		te_coh_data_ready <= te_d4wt;
	else if ((cur_te_state == COHERENT_SUM) && coherent_sum_done)
		te_coh_data_ready <= te_coh_data_ready | coh_ready_channel;
	else if (next_te_state == START)	// clear on start of each round
		te_coh_data_ready <= 32'h0;

assign te_ready = |te_coh_data_ready;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		te_channel_ow_protect <= 32'h0;
	else if (te_reg_cs & te_wr & (te_addr == `TE_OW_PROTECT_CHAN_ADDR))
		te_channel_ow_protect <= te_d4wt;
	else if ((cur_te_state == COHERENT_SUM) && coherent_sum_done)
		te_channel_ow_protect <= te_channel_ow_protect | ow_protect_channel;
	else if (next_te_state == START)	// clear on start of each round
		te_channel_ow_protect <= 32'h0;

wire [3:0] ow_protect_valid;
assign ow_protect_valid = overwrite_protect & coherent_sum_valid & physical_channel_en;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		te_ow_protect_addr  <= 10'h0;
	else if (te_reg_cs & te_wr & (te_addr == `TE_OW_PROTECT_ADDR_ADDR))
		te_ow_protect_addr  <= te_d4wt[11:2];
	else begin
		if (ow_protect_valid[3])
    	te_ow_protect_addr  <= {logic_channel_index[3], 2'b11, cor_dump_index[3][4:2]};
		else if (ow_protect_valid[2])
    	te_ow_protect_addr  <= {logic_channel_index[2], 2'b11, cor_dump_index[2][4:2]};
		else if (ow_protect_valid[1])
    	te_ow_protect_addr  <= {logic_channel_index[1], 2'b11, cor_dump_index[1][4:2]};
		else if (ow_protect_valid[0])
    	te_ow_protect_addr  <= {logic_channel_index[0], 2'b11, cor_dump_index[0][4:2]};
	end

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		te_ow_protect_value <= 32'h0;
	else if (te_reg_cs & te_wr & (te_addr == `TE_OW_PROTECT_VALUE_ADDR))
		te_ow_protect_value <= te_d4wt;
	else begin
		if (ow_protect_valid[3])
  		te_ow_protect_value <= {i_coherent_sum[3], q_coherent_sum[3]};
		else if (ow_protect_valid[2])
  		te_ow_protect_value <= {i_coherent_sum[2], q_coherent_sum[2]};
		else if (ow_protect_valid[1])
  		te_ow_protect_value <= {i_coherent_sum[1], q_coherent_sum[1]};
		else if (ow_protect_valid[0])
  		te_ow_protect_value <= {i_coherent_sum[0], q_coherent_sum[0]};
	end

//----------------------------------------------------------
// coherent sum
//----------------------------------------------------------
// fifo for coherent data buffering
wire [3:0] coh_fifo_rd;
wire [3:0] coh_fifo_wr;
wire [3:0] coh_fifo_empty;
wire [43:0] fifo_out [3:0];		// 12MSB for address, 32LSB for data

assign coh_fifo_wr = coherent_sum_valid & physical_channel_en & (~overwrite_protect);

// four instances of coherent FIFO
generate
	for (i_gen = 0; i_gen < 4; i_gen = i_gen + 1)
	begin: coherent_fifo_gen
		coherent_fifo #(.DATA_WIDTH(44)) u_coherent_fifo_inst
		(
				.clk (clk),
				.rst_b (rst_b),
				.wr_req (coh_fifo_wr[i_gen]),
				.data_in ({logic_channel_index[i_gen], 2'b11, cor_dump_index[i_gen], i_coherent_sum[i_gen], q_coherent_sum[i_gen]}),
				.rd_req (coh_fifo_rd[i_gen]),
				.data_out (fifo_out[i_gen]),
				.empty (coh_fifo_empty[i_gen])
		);
	end
endgenerate

wire coherent_rd;
wire coherent_wr;
wire [9:0] coherent_addr;
wire [31:0] coherent_d4wt;
wire [31:0] coherent_d4rd;

coherent_sum u_coherent_sum
(
		.clk                (clk              ),
		.rst_b              (rst_b            ),
		.coh_fifo_rd        (coh_fifo_rd      ),
		.coh_fifo_empty     (coh_fifo_empty   ),
		.fifo_data0         (fifo_out[0]      ),
		.fifo_data1         (fifo_out[1]      ),
		.fifo_data2         (fifo_out[2]      ),
		.fifo_data3         (fifo_out[3]      ),
		.coherent_rd        (coherent_rd      ),
		.coherent_wr        (coherent_wr      ),
		.coherent_addr      (coherent_addr    ),
		.coherent_d4wt      (coherent_d4wt    ),
		.coherent_d4rd      (coherent_d4rd    ),
		.coherent_sum_done  (coherent_sum_done)
);

//----------------------------------------------------------
// TE FIFO interface signal
//----------------------------------------------------------
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		fifo_read <= 1'b0;
	else if (next_te_state == READ_FIFO)
		fifo_read <= 1'b1;
	else
		fifo_read <= 1'b0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		fifo_rewind <= 1'b0;
	else if ((next_te_state == COR_FINISH) && ~te_done)
		fifo_rewind <= 1'b1;
	else
		fifo_rewind <= 1'b0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		fifo_skip <= 1'b0;
	else if (next_te_state == TE_FINISH)
		fifo_skip <= 1'b1;
	else
		fifo_skip <= 1'b0;

//----------------------------------------------------------
// State and coherent sum buffer
//----------------------------------------------------------
wire coh_buffer_rd;
wire coh_buffer_wr;
wire [9:0] coh_buffer_addr;
wire [31:0] coh_buffer_wdata;
wire [31:0] coh_buffer_rdata;

spram #(.RAM_SIZE(1024), .ADDR_WIDTH(10), .DATA_WIDTH(32)) coherent_buffer
(
		.clk (clk),
		.en (coh_buffer_rd | coh_buffer_wr),
		.we (coh_buffer_wr),
		.addr (coh_buffer_addr),
		.wdata (coh_buffer_wdata),
		.rdata (coh_buffer_rdata)
);

// delay of write and address signal for dump state use
reg state_wr_d;
reg [9:0] state_addr_d;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		state_wr_d <= 1'b0;
	else
		state_wr_d <= state_wr;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		state_addr_d <= 10'h0;
	else
		state_addr_d <= state_addr;

wire coherent_access;
assign coherent_access = (cur_te_state == CORRELATION) || (cur_te_state == COHERENT_SUM);

assign coh_buffer_rd    = coherent_access ? coherent_rd : (cur_te_state == FILL_STATE) ? state_rd : (te_buffer_cs & te_rd); // state buffer read signal
assign coh_buffer_wr    = coherent_access ? coherent_wr : (cur_te_state == DUMP_STATE) ? state_wr_d : (te_buffer_cs & te_wr); // state buffer write signal
assign coh_buffer_addr  = coherent_access ? coherent_addr : (cur_te_state == FILL_STATE) ? state_addr : (cur_te_state == DUMP_STATE) ? state_addr_d : te_addr[9:0]; // state buffer address
assign coh_buffer_wdata = coherent_access ? coherent_d4wt : (cur_te_state == DUMP_STATE) ? state_d4wt : te_d4wt; // state buffer write data

assign coherent_d4rd = coh_buffer_rdata;
assign state_d4rd = coh_buffer_rdata;
assign te_buffer_d4rd = coh_buffer_rdata;

endmodule