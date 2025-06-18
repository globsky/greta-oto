//----------------------------------------------------------------------
// prn_weil.v:
//   Weil PRN code generation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module m_prn_weil
(
input	clk,
input	rst_b,

// parameter control
input	weil_type,
input [13:0] insertion_index,
input [13:0] weil_index,

// load register signal
input [15:0] legendre_code1_i,
input [15:0] legendre_code2_i,
input [13:0] code_phase_i,
input	code_load,
input	phase_load,
input phase_init,

// ROM interface
output reg [9:0] legendre_addr1,
output legendre_rd1,
output preempt1,
output reg [9:0] legendre_addr2,
output legendre_rd2,
output preempt2,
input legendre_read_valid1,
input legendre_read_valid2,
input [15:0] legendre_data,

// shift control
input	shift_code,

// output state and count
output [15:0] legendre_code1_o,
output [15:0] legendre_code2_o,
output [13:0] code_phase_o,
output prn_reset,
output ready_to_shift,

output prn_code
);

//--------------------------------------------
// internal registers
//--------------------------------------------
reg [15:0] legendre_code1_r;
reg [15:0] legendre_code2_r;
reg [13:0] code_phase_r;
reg [13:0] code_index1;
reg [13:0] code_index2;
reg [13:0] code_index1_init;
reg [13:0] code_index2_init;
reg [2:0] insert_count;
reg [15:0] legendre_code1_preload;
reg [15:0] legendre_code2_preload;
wire insert_flag;

wire [13:0] legendre_length;
wire [9:0] rewind_addr;
assign legendre_length = weil_type ? 14'd10223 : 14'd10243;
assign rewind_addr = weil_type ? 10'h27f : 10'h281;

wire phase_reset;
assign phase_reset = (code_phase_r == 10229);

//--------------------------------------------
// Legendre code register
//--------------------------------------------
wire [13:0] next_code_index1;
wire [13:0] next_code_index2;
assign next_code_index1 = code_index1 + 1;
assign next_code_index2 = code_index2 + 1;

reg phase_init_flag;
wire code1_reload, code2_reload;
wire load_init_data;
reg load_init_data_d;
assign code1_reload = shift_code && !insert_flag && (code_index1[3:0] == 4'hf || (next_code_index1 == legendre_length) || phase_reset);
assign code2_reload = shift_code && !insert_flag && (code_index2[3:0] == 4'hf || (next_code_index2 == legendre_length) || phase_reset);

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		legendre_code1_r <= 0;
	else if (code_load)
		legendre_code1_r <= legendre_code1_i;
	else if (code1_reload || load_init_data)
		legendre_code1_r <= legendre_code1_preload;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		legendre_code2_r <= 0;
	else if (code_load)
		legendre_code2_r <= legendre_code2_i;
	else if (code2_reload || load_init_data_d)
		legendre_code2_r <= legendre_code2_preload;

//--------------------------------------------
// code phase and insertion value
//--------------------------------------------
reg phase_load_d;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		phase_load_d <= 0;
	else
		phase_load_d <= (phase_load | phase_init);

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		code_phase_r <= 0;
	else if (phase_init)
		code_phase_r <= 0;
	else if (phase_load)
		code_phase_r <= code_phase_i;
	else if (shift_code)
		code_phase_r <= phase_reset ? 0 : (code_phase_r + 1);

wire [14:0] phase_cmp0, phase_cmp7;
assign phase_cmp0 = {1'b0, code_phase_r} - {1'b0, insertion_index};
assign phase_cmp7 = phase_cmp0 - 7;

assign insert_flag = (phase_cmp0[14] ^ phase_cmp7[14]) & weil_type;
wire [2:0] insert_count_next;
assign insert_count_next = insert_count + 1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		insert_count <= 0;
	else if (phase_load_d)
		insert_count <= insert_flag ? phase_cmp0[2:0] : (phase_cmp0[14] ? 3'd0 : 3'd7);
	else if (shift_code)
	begin
		if (phase_reset)
			insert_count <= 0;
		else if (insert_flag)
			insert_count <= (insert_count_next == 7) ? 3'd0 : insert_count_next;
	end

//--------------------------------------------
// code index load value and initial value calculation
//--------------------------------------------
// Legendre round back logic
wire [13:0] index_in, index_out, index_add;
wire [14:0] index_sum, index_sub;
assign index_sum = {1'b0, index_in} + {1'b0, index_add};
assign index_sub = index_sum - {1'b0, legendre_length};
assign index_out = index_sub[14] ? index_sum[13:0] : index_sub[13:0];

reg [1:0] index_calc_phase;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		index_calc_phase <= 0;
	else if (phase_load_d || index_calc_phase != 2'b00)
		index_calc_phase <= index_calc_phase + 1;

// input select for round back logic
wire [13:0] code_phase_select;
assign code_phase_select = index_calc_phase[1] ? code_phase_r : 14'd0;
assign index_in = index_calc_phase[0] ? (index_calc_phase[1] ? code_index1 : code_index1_init) : code_phase_select;
assign index_add = index_calc_phase[0] ? weil_index : insertion_index;

// assignment for code index registers and their initial value
wire code_index1_reset, code_index2_reset;
assign code_index1_reset = (next_code_index1 == legendre_length);
assign code_index2_reset = (next_code_index2 == legendre_length);

wire [13:0] index_for_l1c;
assign index_for_l1c = code_phase_r - insert_count;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		code_index1_init <= 0;
	else if (phase_load_d)
		code_index1_init <= weil_type ? 14'd0 : index_out;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		code_index2_init <= 0;
	else if (index_calc_phase == 2'b01)
		code_index2_init <= index_out;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		code_index1 <= 0;
	else if (index_calc_phase == 2'b10)
		code_index1 <= weil_type ? index_for_l1c : index_out;
	else if (shift_code)
	begin
		if (phase_reset)
			code_index1 <= code_index1_init;
		else if (!insert_flag)
			code_index1 <= code_index1_reset ? 0 : next_code_index1;
	end

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		code_index2 <= 0;
	else if (index_calc_phase == 2'b11)
		code_index2 <= index_out;
	else if (shift_code)
	begin
		if (phase_reset)
			code_index2 <= code_index2_init;
		else if (!insert_flag)
			code_index2 <= code_index2_reset ? 0 : next_code_index2;
	end

//--------------------------------------------
// preload code registers
//--------------------------------------------
wire [13:0] count_to_reset;	// count of bit to reset code phase
wire [4:0] count_to_reset_clip;	// clip to 16
wire count_exceed;
assign count_to_reset = 14'd10229 - code_phase_r;
assign count_exceed = |count_to_reset[13:4];
assign count_to_reset_clip = {count_exceed, count_exceed ? 4'b0000 : count_to_reset[3:0]};

wire [4:0] next_rewind_fill1, next_rewind_fill2;	// MSB indicate data to be loaded at next address or init address at fill stage
assign next_rewind_fill1 = count_to_reset_clip + {1'b0, code_index1[3:0]} + ((code_index1[13:4] == 10'h280) ? 5'd13 : 5'd0);
assign next_rewind_fill2 = count_to_reset_clip + {1'b0, code_index2[3:0]} + ((code_index2[13:4] == 10'h280) ? 5'd13 : 5'd0);

wire [4:0] next_rewind_load1, next_rewind_load2;	// MSB indicate data to be loaded at next address or init address at continuous run stage
// calculate whether reloaded bits enough to output till reset, code in address in 280 only has 3 bits
assign next_rewind_load1 = ~(count_to_reset_clip - ((legendre_addr1 == 10'h280) ? 5'h3 : 5'h10));
assign next_rewind_load2 = ~(count_to_reset_clip - ((legendre_addr2 == 10'h280) ? 5'h3 : 5'h10));

wire [9:0] next_addr_fill1, next_addr_fill2;
wire [9:0] next_addr_load1, next_addr_load2;
assign next_addr_fill1 = (next_rewind_fill1[4] && ~phase_init_flag) ? (code_index1[13:4] + 1) : code_index1_init[13:4];
assign next_addr_fill2 = (next_rewind_fill2[4] && ~phase_init_flag) ? (code_index2[13:4] + 1) : code_index2_init[13:4];
assign next_addr_load1 = next_rewind_load1[4] ? (code_index1_reset ? 10'd1 : (next_code_index1[13:4] + 1)) : (phase_reset ? (code_index1_init[13:4] + 1) : code_index1_init[13:4]);
assign next_addr_load2 = next_rewind_load2[4] ? (code_index2_reset ? 10'd1 : (next_code_index2[13:4] + 1)) : (phase_reset ? (code_index2_init[13:4] + 1) : code_index2_init[13:4]);

// valid indicator of preload data register
wire init_preload1;
reg init_preload2;
assign init_preload1 = (index_calc_phase == 2'b11);

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		init_preload2 <= 0;
	else
		init_preload2 <= init_preload1;

reg preload_valid1, preload_valid2;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		preload_valid1 <= 1;
	else if (code1_reload || init_preload1 || load_init_data_d)
		preload_valid1 <= 0;
	else if (legendre_rd1 && legendre_read_valid1)
		preload_valid1 <= 1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		preload_valid2 <= 1;
	else if (code2_reload || init_preload2 || load_init_data_d)
		preload_valid2 <= 0;
	else if (legendre_rd2 && legendre_read_valid2)
		preload_valid2 <= 1;

// address to next
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		legendre_addr1 <= 10'd0;
	else if (init_preload1 || load_init_data_d)	// preload at register load stage
		legendre_addr1 <= (next_addr_fill1 == rewind_addr) ? 10'h000 : next_addr_fill1;
	else if (code1_reload)	// preload at preload data fill into legendre code
		legendre_addr1 <= (next_addr_load1 == rewind_addr) ? 10'h000 : next_addr_load1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		legendre_addr2 <= 10'd0;
	else if (init_preload2 || load_init_data_d)	// preload at register load stage
		legendre_addr2 <= (next_addr_fill2 == rewind_addr) ? 10'h000 : next_addr_fill2;
	else if (code2_reload)	// preload at preload data fill into legendre code
		legendre_addr2 <= (next_addr_load2 == rewind_addr) ? 10'h000 : next_addr_load2;

// indicator for read value valid
reg read_valid1, read_valid2;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		read_valid1 <= 0;
	else if (legendre_rd1 && legendre_read_valid1)
		read_valid1 <= 1;
	else
		read_valid1 <= 0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		read_valid2 <= 0;
	else if (legendre_rd2 && legendre_read_valid2)
		read_valid2 <= 1;
	else
		read_valid2 <= 0;

// preload data register assignment
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		legendre_code1_preload <= 16'd0;
	else if (read_valid1)
		legendre_code1_preload <= (legendre_addr1 == 10'd640) ? 16'hc000 : legendre_data;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		legendre_code2_preload <= 16'd0;
	else if (read_valid2)
		legendre_code2_preload <= (legendre_addr2 == 10'd640) ? 16'hc000 : legendre_data;

assign legendre_rd1 = ~preload_valid1;
assign legendre_rd2 = ~preload_valid2;
assign preempt1 = (code_index1_init[3:0] == 4'hf) ? 1'b1 : 1'b0;
assign preempt2 = (code_index2_init[3:0] == 4'hf) ? 1'b1 : 1'b0;

// indicator of preload value ready
reg preload_value_ready1, preload_value_ready2;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		preload_value_ready1 <= 1;
	else if (code1_reload || phase_load || phase_init || load_init_data)
		preload_value_ready1 <= 0;
	else if (read_valid1)
		preload_value_ready1 <= 1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		preload_value_ready2 <= 1;
	else if (code2_reload || phase_load || phase_init || load_init_data)
		preload_value_ready2 <= 0;
	else if (read_valid2)
		preload_value_ready2 <= 1;

wire code1_critical, code2_critical;
assign code1_critical = (code_index1[3:0] == 4'hf) || (code_index1[13:11] == 3'b101) || phase_reset;		// last bit or index exceed 0x280 or phase reset
assign code2_critical = (code_index2[3:0] == 4'hf) || (code_index2[13:11] == 3'b101) || phase_reset;		// last bit or index exceed 0x280 or phase reset
assign ready_to_shift = (preload_value_ready1 || ~code1_critical) && (preload_value_ready2 || ~code2_critical) && ~phase_init_flag;

// load preload value into legendre code register on phase init
wire preload_value_ready;
assign preload_value_ready = (preload_value_ready1 && preload_value_ready2);

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		phase_init_flag <= 0;
	else if (phase_init)
		phase_init_flag <= 1;
	else if (preload_value_ready)
		phase_init_flag <= 0;

assign load_init_data = phase_init_flag && preload_value_ready;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		load_init_data_d <= 0;
	else
		load_init_data_d <= load_init_data;

//--------------------------------------------
// output prn code
//--------------------------------------------
reg code1_output, code2_output;

always @(*)
	case(code_index1[3:0])
	4'b0000:
		code1_output = legendre_code1_r[15];
	4'b0001:
		code1_output = legendre_code1_r[14];
	4'b0010:
		code1_output = legendre_code1_r[13];
	4'b0011:
		code1_output = legendre_code1_r[12];
	4'b0100:
		code1_output = legendre_code1_r[11];
	4'b0101:
		code1_output = legendre_code1_r[10];
	4'b0110:
		code1_output = legendre_code1_r[ 9];
	4'b0111:
		code1_output = legendre_code1_r[ 8];
	4'b1000:
		code1_output = legendre_code1_r[ 7];
	4'b1001:
		code1_output = legendre_code1_r[ 6];
	4'b1010:
		code1_output = legendre_code1_r[ 5];
	4'b1011:
		code1_output = legendre_code1_r[ 4];
	4'b1100:
		code1_output = legendre_code1_r[ 3];
	4'b1101:
		code1_output = legendre_code1_r[ 2];
	4'b1110:
		code1_output = legendre_code1_r[ 1];
	4'b1111:
		code1_output = legendre_code1_r[ 0];
	endcase

always @(*)
	case(code_index2[3:0])
	4'b0000:
		code2_output = legendre_code2_r[15];
	4'b0001:
		code2_output = legendre_code2_r[14];
	4'b0010:
		code2_output = legendre_code2_r[13];
	4'b0011:
		code2_output = legendre_code2_r[12];
	4'b0100:
		code2_output = legendre_code2_r[11];
	4'b0101:
		code2_output = legendre_code2_r[10];
	4'b0110:
		code2_output = legendre_code2_r[ 9];
	4'b0111:
		code2_output = legendre_code2_r[ 8];
	4'b1000:
		code2_output = legendre_code2_r[ 7];
	4'b1001:
		code2_output = legendre_code2_r[ 6];
	4'b1010:
		code2_output = legendre_code2_r[ 5];
	4'b1011:
		code2_output = legendre_code2_r[ 4];
	4'b1100:
		code2_output = legendre_code2_r[ 3];
	4'b1101:
		code2_output = legendre_code2_r[ 2];
	4'b1110:
		code2_output = legendre_code2_r[ 1];
	4'b1111:
		code2_output = legendre_code2_r[ 0];
	endcase

/*reg insert_output;

always @(*)
	case(insert_count)
	3'b000:
		insert_output = 1'b0;
	3'b001:
		insert_output = 1'b1;
	3'b010:
		insert_output = 1'b1;
	3'b011:
		insert_output = 1'b0;
	3'b100:
		insert_output = 1'b1;
	3'b101:
		insert_output = 1'b0;
	3'b110:
		insert_output = 1'b0;
	3'b111:
		insert_output = 1'b1;
	default:
		insert_output = 1'b0;
	endcase*/

// this is alternative way to calculate inserted PRN
wire insert_output;
assign insert_output = insert_count[0] ^ insert_count[1] ^ insert_count[2];

assign prn_code = insert_flag ? insert_output : (code1_output ^ code2_output);

assign legendre_code1_o = legendre_code1_r;
assign legendre_code2_o = legendre_code2_r;
assign code_phase_o = code_phase_r;
assign prn_reset = phase_reset;

endmodule
