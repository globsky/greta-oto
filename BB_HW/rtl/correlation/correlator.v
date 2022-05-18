//----------------------------------------------------------------------
// correlator.v:
//   Universal correlator module
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module correlator(
// system signals
input clk,   // system clock
input rst_b, // reset signal, low active

// input data from fifo
input fifo_data_en, 		// input fifo data enable signal
input [7:0] fifo_data,	// input fifo data 4bit S/M format, 4MSB for I and 4LSB for Q

// configuration from state buffer
// parameters
input [31:0]  carrier_freq,	// carrier frequence
input [31:0]  code_freq,		// code frequence
input [1:0]   pre_shift_bits,		// pre shift bits
input [1:0]   post_shift_bits,  // post shift bits
input data_in_q,			// data in Q branch
input enable_2nd_prn,	// enable second prn
input enable_boc,			// enable BOC
input [1:0]   decode_bit,			// bit number of decoded data
input [1:0]   narrow_factor,	// narrow correlator factor
input [4:0]   bit_length,		// length (ms) of one bit for data decode
input [5:0]		coherent_number,  // coherent number
input [24:0]  nh_code,  // NH code
input [4:0] 	nh_length,  // NH data lenth
input [15:0]  dump_length,  	// dump data length

// variables of carrier and code
input carrier_phase_en, // carrier phase load enable
input [31:0]  carrier_phase_i,  // carrier phase load value
input carrier_count_en, // carrier count load enable
input [31:0]  carrier_count_i,  // carrier count load value
input code_phase_en,  // code phase load enable
input [31:0]  code_phase_i, // code phase load value
output [31:0]  carrier_phase_o,  // carrier phase output
output [31:0]  carrier_count_o,  // carrier count output
output [31:0]  code_phase_o,		 // code phase output

// variables of PRN_CODE
input prn_code_load_en,  // load enable
input [15:0] dump_count_i,		// dump count load value
input [7:0]  jump_count_i, 		// jump count load value
input [7:0]  prn_code_i, 		// prn code load value
output [15:0] dump_count_o, // dump count output
output [7:0]  jump_count_o,	// jump count output
output [7:0]  prn_code_o, 	// prn code output

// variables of CORR_STATE
input corr_state_load_en, // load enable
input [4:0] nh_count_i, 	// NH code count load value
input [5:0] coherent_count_i, // coherent count load value
input [4:0] bit_count_i,	// number of ms within data bit load value
input [3:0]  prn_code2_i, 		// prn code2 load value
input [2:0] current_cor_i,	  // current correlator load value
input code_sub_phase_i,  // code sub-phase load value
input dumping_i,		// dumping load value
output [4:0] nh_count_o, // NH code count output
output [5:0] coherent_count_o,	// coherent count output
output [4:0] bit_count_o,		// number of ms within data bit output
output [3:0] prn_code2_o,				// prn code2 output
output [2:0] current_cor_o,			// current correlator output
output code_sub_phase_o,  // code sub phase output
output dumping_o,  // dumping flag output

// variables of MS_DATA
input decode_data_en,		// load enable
input [31:0] decode_data_i, 	// decode data sum load value
output reg [31:0] decode_data_o,	// decode data sum output

// variables of partial accumulate data
input acc_en, // acc partial result load enable, valid data will be in i_acc_i and i_acc_q in next 8 clock cycles, one cor per cycle
input acc_dump,	// acc result dump enable
input [2:0] acc_sel,    // indicate which acc data to dump
input [15:0]  i_acc_i,  // acc I channel data load value
input [15:0]  q_acc_i,  // acc Q channel data load value
output [15:0] i_acc_o,  // acc I channel data output
output [15:0] q_acc_o,  // acc Q channel data output

// parameters and varables for prn generator
input [31:0]  te_polynomial,  // te polynomial register
input [31:0]  te_code_length, // te code length register
input [31:0]  te_polynomial2, // te polynomial register for secondary PRN
input [31:0]  te_code_length2,// te code length register for secondary PRN
input [31:0]  prn_config,			// PRN config
input [31:0]  prn2_config,		// PRN2 config
input prn_state_en,  // PRN state load enable
input [31:0]  prn_state_i, // PRN state load value
output [31:0] prn_state_o, // PRN state output
input prn_count_en,	// PRN count load enable
input [31:0] prn_count_i,		// PRN counter load value
output [31:0] prn_count_o,	// PRN counter output
input prn2_state_en,  // PRN2 state load enable
input [31:0]  prn2_state_i, // PRN2 state load value
output [31:0] prn2_state_o, // PRN2 state output

// Legendre code ROM interface
output [9:0] legendre_addr,
output legendre_rd,
input legendre_read_valid,
input [15:0] legendre_data,

// memory code ROM interface
output [13:0] memcode_addr,
output memcode_rd,
input memcode_read_valid,
input [31:0] memcode_data,

// output data for coherent sum
output [ 4:0] cor_dump_index,	// index of correlator to dump
output [15:0] i_coherent_sum,	// I channel data for coherent sum
output [15:0] q_coherent_sum,	// Q channel data for coherent sum
output coherent_sum_valid, // coherent sum valid flag

// output data for noise calculation
output data_down_en,
output [5:0] i_data_down,
output [5:0] q_data_down,
output shift_code,

// control and state signals
input	fill_finished,			// set this signal when all physical channel fill finish
input coh_finished,				// set this signal when coherent sum finish and decode data is ready
input [31:0] coh_acc_data,		// coherent acc data from coherent sum module
output     cor_ready,			// correlator ready to accept input sample
output     coherent_done_o,    //coherent data is ready
output     overwrite_protect
);

genvar i_gen;
//----------------------------------------------------------
// down_converter instance
//----------------------------------------------------------
m_down_converter u_down_converter
(
  .clk              (clk             ),
  .rst_b            (rst_b           ),
  .carrier_freq     (carrier_freq    ),
  .pre_shift_bits   (pre_shift_bits  ),
  .carrier_phase_en (carrier_phase_en),
  .carrier_phase_i  (carrier_phase_i ),
  .carrier_phase_o  (carrier_phase_o ),
  .carrier_count_en (carrier_count_en),
  .carrier_count_i  (carrier_count_i ),
  .carrier_count_o  (carrier_count_o ),
  .input_sample_i   (fifo_data       ),
  .sample_en_i      (fifo_data_en    ),
  .i_data_o         (i_data_down     ),
  .q_data_o         (q_data_down     ),
  .data_en_o        (data_down_en    )
);

//--------------------------------------------------------------
// overflow generation
//--------------------------------------------------------------
//code nco
wire overflow;
overflow_gen u_overflow_gen
(
  .clk             (clk              ),
  .rst_b           (rst_b            ),
  .data_down_en    (data_down_en     ),
  .code_freq       (code_freq        ),
  .code_phase_en   (code_phase_en    ),
  .code_phase_i    (code_phase_i     ),
  .code_phase_o    (code_phase_o     ),
  .fill_finished   (fill_finished    ),
  .jump_count_en   (prn_code_load_en ),
  .jump_count_i    (jump_count_i     ),
  .jump_count_o    (jump_count_o     ),
  .overflow        (overflow         )
);

//--------------------------------------------------------------
// prn generate and shift
//--------------------------------------------------------------
wire prn_reset;
wire prn_code1, prn_code2;

m_prn_code u_prn_code
(
  .clk                 (clk                ),
  .rst_b               (rst_b              ),
	.te_polynomial       (te_polynomial      ),
	.te_code_length      (te_code_length     ),
	.te_polynomial2      (te_polynomial2     ),
	.te_code_length2     (te_code_length2    ),
	.prn_config          (prn_config         ),
	.prn2_config         (prn2_config        ),
	.prn_state_en        (prn_state_en       ),
	.prn_state_i         (prn_state_i        ),
	.prn_state_o         (prn_state_o        ),
	.prn_count_en        (prn_count_en       ),
	.prn_count_i         (prn_count_i        ),
	.prn_count_o         (prn_count_o        ),
	.prn2_state_en       (prn2_state_en      ),
	.prn2_state_i        (prn2_state_i       ),
	.prn2_state_o        (prn2_state_o       ),
	.legendre_addr       (legendre_addr      ),
	.legendre_rd         (legendre_rd        ),
	.legendre_read_valid (legendre_read_valid),
	.legendre_data       (legendre_data      ),
	.memcode_addr        (memcode_addr       ),
	.memcode_rd          (memcode_rd         ),
	.memcode_read_valid  (memcode_read_valid ),
	.memcode_data        (memcode_data       ),
	.shift_code          (shift_code         ),
	.enable_2nd_prn      (enable_2nd_prn     ),
	.prn_reset           (prn_reset          ),
	.prn_code1           (prn_code1          ),
	.prn_code2           (prn_code2          )
);

// PRN shift signal
reg code_sub_phase_r;

always @(posedge clk or negedge rst_b)
	if (~rst_b)
		code_sub_phase_r <= 1'b0;
	else if (corr_state_load_en)
		code_sub_phase_r <= code_sub_phase_i;
	else if (overflow)
		code_sub_phase_r <= ~code_sub_phase_r;

assign shift_code = code_sub_phase_r & overflow;
assign code_sub_phase_o = code_sub_phase_r;

//--------------------------------------------------------------
// NH code used for correlation
//--------------------------------------------------------------
wire cur_nh_code1, cur_nh_code2;

nh_code_gen u_nh_code_gen
(
    .clk            (clk                 ),
    .rst_b          (rst_b               ),
    .nh_code1       (nh_code             ),
    .nh_code2       (20'h0               ),
    .nh_increase    (prn_reset&shift_code),
    .nh_length      (nh_length           ),
    .nh_count_en    (corr_state_load_en  ),
    .nh_count_i     (nh_count_i          ),
    .nh_count_o     (nh_count_o          ),
    .cur_nh_code1   (cur_nh_code1        ),
    .cur_nh_code2   (cur_nh_code2        )
);

//--------------------------------------------------------------
// prn code used for correlation
//--------------------------------------------------------------
wire [7:0] prn_bits;

prn_code_cor u_prn_code_cor
(
    .clk                 (clk                ),
    .rst_b               (rst_b              ),
		.enable_boc          (enable_boc         ),
		.enable_2nd_prn      (enable_2nd_prn     ),
		.narrow_factor       (narrow_factor      ),
		.code_sub_phase      (code_sub_phase_r   ),
		.code_phase          (code_phase_o[31:30]),
		.overflow            (overflow           ),
		.prn_code1           (prn_code1          ),
		.prn_code2           (prn_code2          ),
		.nh_code1            (cur_nh_code1       ),
		.nh_code2            (cur_nh_code2       ),
		.prn_code_load_en    (prn_code_load_en   ),
		.prn_code_i          (prn_code_i         ),
		.prn_code_o          (prn_code_o         ),
		.corr_state_load_en  (corr_state_load_en ),
		.prn_code2_i         (prn_code2_i        ),
		.prn_code2_o         (prn_code2_o        ),
		.prn_bits            (prn_bits           )
);

//--------------------------------------------------------------
// dumping signal generating
//--------------------------------------------------------------
wire [4:0] cor_index;
wire do_ms_data_sum;
wire dumping_valid;
wire data_decode_valid;
wire [15:0] i_acc_shift;
wire [15:0] q_acc_shift;

dumping_logic u_dumping_logic
(
    .clk                 (clk               ),
    .rst_b               (rst_b             ),
    .overflow            (overflow          ),
    .shift_code          (shift_code        ),
    .bit_length          (bit_length        ),
    .coherent_number     (coherent_number   ),
		.enable_2nd_prn      (enable_2nd_prn    ),
    .dump_length         (dump_length       ),
    .dump_count_en       (prn_code_load_en  ),
    .dump_count_i        (dump_count_i      ),
    .dump_count_o        (dump_count_o      ),
    .dumping_en          (corr_state_load_en),
    .dumping_i           (dumping_i         ),
    .dumping_o           (dumping_o         ),
    .current_cor_en      (corr_state_load_en),
    .current_cor_i       (current_cor_i     ),
    .current_cor_o       (current_cor_o     ),
    .coherent_count_en   (corr_state_load_en),
    .coherent_count_i    (coherent_count_i  ),
    .coherent_count_o    (coherent_count_o  ),
    .bit_count_en        (corr_state_load_en),
    .bit_count_i         (bit_count_i       ),
    .bit_count_o         (bit_count_o       ),
    .i_acc_shift         (i_acc_shift       ),
    .q_acc_shift         (q_acc_shift       ),
    .coherent_sum_valid  (coherent_sum_valid),
    .cor_index           (cor_dump_index    ),
    .i_coherent_sum      (i_coherent_sum    ),
    .q_coherent_sum      (q_coherent_sum    ),
//    .do_ms_data_sum      (do_ms_data_sum    ),
    .dumping_valid       (dumping_valid     ),
    .coherent_done_o     (coherent_done_o   ),
    .overwrite_protect   (overwrite_protect ),
    .data_decode_valid   (data_decode_valid )
);

//--------------------------------------------------------------
// I/Q data accumulation
//--------------------------------------------------------------
reg [7:0] acc_load_select;
wire [7:0] acc_clear_select;
wire [7:0] data_acc_en;
assign acc_clear_select = (8'b1 << current_cor_o) & {8{dumping_valid}};
assign data_acc_en = (data_down_en & ~jump_count_o[7]);

wire [5:0] i_data_pos;
wire [5:0] q_data_pos;
wire [5:0] i_data_neg;
wire [5:0] q_data_neg;
assign i_data_pos = data_acc_en ? i_data_down : 6'd0;
assign q_data_pos = data_acc_en ? q_data_down : 6'd0;
assign i_data_neg = -i_data_pos;
assign q_data_neg = -q_data_pos;

wire [15:0] i_acc_result [7:0];
wire [15:0] q_acc_result [7:0];

// shift in acc_en as acc load selection for each accumulator
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		acc_load_select <= 8'h0;
	else
		acc_load_select <= {acc_load_select[6:0], acc_en};

generate
	for (i_gen = 0; i_gen < 8; i_gen = i_gen + 1)
	begin: data_acc_gen
		data_acc u_data_acc
		(
			.clk         (clk                     ),
			.rst_b       (rst_b                   ),
			.acc_in_en   (acc_load_select[i_gen]  ),
			.i_acc_i     (i_acc_i                 ),
			.q_acc_i     (q_acc_i                 ),
			.acc_clear   (acc_clear_select[i_gen] ),
			.i_data_pos  (i_data_pos              ),
			.q_data_pos  (q_data_pos              ),
			.i_data_neg  (i_data_neg              ),
			.q_data_neg  (q_data_neg              ),
			.prn_code    (prn_bits[i_gen]         ),
			.i_acc_o     (i_acc_result[i_gen]     ),
			.q_acc_o     (q_acc_result[i_gen]     )
		);
	end
endgenerate

// select data to dumping logic
wire [2:0] acc_data_select;
assign acc_data_select = acc_dump ? acc_sel : current_cor_o;

reg [15:0] i_acc_result_sel;
reg [15:0] q_acc_result_sel;

always @ (*)
	case(acc_data_select)
		3'h0 : i_acc_result_sel = i_acc_result[0];
		3'h1 : i_acc_result_sel = i_acc_result[1];
		3'h2 : i_acc_result_sel = i_acc_result[2];
		3'h3 : i_acc_result_sel = i_acc_result[3];
		3'h4 : i_acc_result_sel = i_acc_result[4];
		3'h5 : i_acc_result_sel = i_acc_result[5];
		3'h6 : i_acc_result_sel = i_acc_result[6];
		3'h7 : i_acc_result_sel = i_acc_result[7];
		default: i_acc_result_sel = 16'd0;
	endcase

always @ (*)
	case(acc_data_select)
		3'h0 : q_acc_result_sel = q_acc_result[0];
		3'h1 : q_acc_result_sel = q_acc_result[1];
		3'h2 : q_acc_result_sel = q_acc_result[2];
		3'h3 : q_acc_result_sel = q_acc_result[3];
		3'h4 : q_acc_result_sel = q_acc_result[4];
		3'h5 : q_acc_result_sel = q_acc_result[5];
		3'h6 : q_acc_result_sel = q_acc_result[6];
		3'h7 : q_acc_result_sel = q_acc_result[7];
		default: q_acc_result_sel = 16'd0;
	endcase

assign i_acc_o = i_acc_result_sel;
assign q_acc_o = q_acc_result_sel;

// data shift to dump
assign i_acc_shift = ($signed(i_acc_result_sel) >>> post_shift_bits);
assign q_acc_shift = ($signed(q_acc_result_sel) >>> post_shift_bits);

//--------------------------------------------------------------
// data decode
//--------------------------------------------------------------
wire [2:0] length_index;
wire [2:0] shift_bits, bit_select;
assign length_index = bit_length[4] ? 3'd2 : (bit_length[3] ? 3'd1 : 3'd0);
assign shift_bits = {1'b0, pre_shift_bits} + {1'b0, post_shift_bits};
assign bit_select = (shift_bits >= length_index) ? (shift_bits - length_index) : 3'd0;

wire [15:0] data_select;
assign data_select = data_in_q ? coh_acc_data[15:0] : coh_acc_data[31:16];

// 1bit data
wire data_dec1;
assign data_dec1 = data_select[15];

// 2bit data
reg [7:0] data_dec2_sel;
always @ (*)
	case (bit_select)
	    3'd0:  data_dec2_sel = {{6{data_select[15]}}, data_select[15:14]} + data_select[13];
	    3'd1:  data_dec2_sel = {{5{data_select[15]}}, data_select[15:13]} + data_select[12];
	    3'd2:  data_dec2_sel = {{4{data_select[15]}}, data_select[15:12]} + data_select[11];
	    3'd3:  data_dec2_sel = {{3{data_select[15]}}, data_select[15:11]} + data_select[10];
	    3'd4:  data_dec2_sel = {{2{data_select[15]}}, data_select[15:10]} + data_select[9];
	    3'd5:  data_dec2_sel = {data_select[15], data_select[15:9]} + data_select[8];
	    3'd6:  data_dec2_sel = data_select[15:8] + data_select[7];
	    default: data_dec2_sel = data_select[15:8] + data_select[7];
	endcase

wire [1:0] data_dec2;	// clip
assign data_dec2 = ((data_dec2_sel[7:1] == 7'h0) || (data_dec2_sel[7:1] == 7'h7f)) ? data_dec2_sel[1:0] : (data_select[15] ? 2'b10 : 2'b01);

// 4bit data
reg [9:0] data_dec4_sel;
always @ (*)
	case (bit_select)
	    3'd0:  data_dec4_sel = {{6{data_select[15]}}, data_select[15:12]} + data_select[11];
	    3'd1:  data_dec4_sel = {{5{data_select[15]}}, data_select[15:11]} + data_select[10];
	    3'd2:  data_dec4_sel = {{4{data_select[15]}}, data_select[15:10]} + data_select[9];
	    3'd3:  data_dec4_sel = {{3{data_select[15]}}, data_select[15:9]} + data_select[8];
	    3'd4:  data_dec4_sel = {{2{data_select[15]}}, data_select[15:8]} + data_select[7];
	    3'd5:  data_dec4_sel = {data_select[15], data_select[15:7]} + data_select[6];
	    3'd6:  data_dec4_sel = data_select[15:6] + data_select[5];
	    default: data_dec4_sel = data_select[15:6] + data_select[5];
	endcase

wire [3:0] data_dec4;	// clip
assign data_dec4 = ((data_dec4_sel[9:3] == 7'h0) || (data_dec4_sel[9:3] == 7'h7f)) ? data_dec4_sel[3:0] : (data_select[15] ? 4'b1000 : 4'b0111);

// 8bit data
reg [13:0] data_dec8_sel;
always @ (*)
	case (bit_select)
	    3'd0:  data_dec8_sel = {{6{data_select[15]}}, data_select[15:8]};
	    3'd1:  data_dec8_sel = {{5{data_select[15]}}, data_select[15:7]};
	    3'd2:  data_dec8_sel = {{4{data_select[15]}}, data_select[15:6]};
	    3'd3:  data_dec8_sel = {{3{data_select[15]}}, data_select[15:5]};
	    3'd4:  data_dec8_sel = {{2{data_select[15]}}, data_select[15:4]};
	    3'd5:  data_dec8_sel = {data_select[15], data_select[15:3]};
	    3'd6:  data_dec8_sel = data_select[15:2];
	    default: data_dec8_sel = data_select[15:2];
	endcase

wire [7:0] data_dec8;	// clip
assign data_dec8 = ((data_dec8_sel[13:7] == 7'h0) || (data_dec8_sel[13:7] == 7'h7f)) ? data_dec8_sel[7:0] : (data_select[15] ? 8'h80 : 8'h7f);

reg [31:0] decode_data;
always @ (*)
	case (decode_bit)
	    2'd0:  decode_data = {decode_data_o[30:0], data_dec1};
	    2'd1:  decode_data = {decode_data_o[29:0], data_dec2};
	    2'd2:  decode_data = {decode_data_o[28:0], data_dec4};
	    2'd3:  decode_data = {decode_data_o[24:0], data_dec8};
	    default: decode_data = decode_data_o;
	endcase

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		decode_data_o <= 32'd0;
	else if (decode_data_en)
		decode_data_o <= decode_data_i;
	else if (coh_finished && data_decode_valid)
		decode_data_o <= decode_data;
		
assign cor_ready = (jump_count_o[7])|(~(|jump_count_o[6:0]));

endmodule
