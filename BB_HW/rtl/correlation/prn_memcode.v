//----------------------------------------------------------------------
// prn_memcode.v:
//   Memory PRN code generation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module m_prn_memcode
(
input	clk,
input	rst_b,

// parameter control
input [11:0] start_index,
input [3:0] length,

// load register signal
input [31:0] current_code_i,
input [13:0] current_phase_i,
input	code_load,
input	phase_load,
input phase_init,

// ROM interface
output reg [13:0] memcode_addr,
output memcode_rd,
input memcode_read_valid,
input [31:0] memcode_data,

// shift control
input	shift_code,

// output state and count
output [31:0] current_code_o,
output [13:0] current_phase_o,
output prn_reset,
output reg ready_to_shift,

output reg prn_code
);

//--------------------------------------------
// internal registers
//--------------------------------------------
reg [31:0] current_code_r;
reg [4:0] bit_index;
reg [4:0] word_index;
reg [3:0] segment_index;
reg [31:0] code_preload;

wire [4:0] bit_index_next;
wire [4:0] word_index_next;
wire [3:0] segment_index_next;
wire skip;
assign bit_index_next = bit_index + 1;
assign word_index_next = word_index + 1;
assign segment_index_next = segment_index + 1;
assign skip = ({word_index, bit_index_next} == 10'h3ff);

//--------------------------------------------
// code register
//--------------------------------------------
reg phase_init_flag;
reg load_init_data;

wire code_reload;
assign code_reload = (shift_code && (bit_index == 5'h1f || skip)) || load_init_data;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		current_code_r <= 0;
	else if (code_load)
		current_code_r <= current_code_i;
	else if (code_reload)
		current_code_r <= code_preload;

//--------------------------------------------
// index counting
//--------------------------------------------
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		bit_index <= 0;
	else if (phase_init)
		bit_index <= 0;
	else if (phase_load)
		bit_index <= current_phase_i[4:0];
	else if (shift_code)
	begin
		if (skip)
			bit_index <= 0;
		else
			bit_index <= bit_index_next;
	end

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		word_index <= 0;
	else if (phase_init)
		word_index <= 0;
	else if (phase_load)
		word_index <= current_phase_i[9:5];
	else if (shift_code)
	begin
		if (skip)
			word_index <= 0;
		else if (bit_index == 5'h1f)
			word_index <= word_index_next;
	end

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		segment_index <= 0;
	else if (phase_init)
		segment_index <= 0;
	else if (phase_load)
		segment_index <= current_phase_i[13:10];
	else if (shift_code && skip)
	begin
		if (segment_index_next == length)
			segment_index <= 0;
		else
			segment_index <= segment_index_next;
	end

//--------------------------------------------
// preload code registers
//--------------------------------------------
reg [8:0] next_addr;

always @(*)
	if (word_index_next == 5'h0)
	begin
		if (segment_index_next == length)
			next_addr = 9'd0;
		else
			next_addr = {segment_index_next, 5'h0};
	end
	else
			next_addr = {segment_index, word_index_next};

reg phase_load_d;
reg code_reload_d;
reg preload_valid;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		phase_load_d <= 0;
	else
		phase_load_d <= phase_load;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		code_reload_d <= 0;
	else
		code_reload_d <= code_reload;

// valid indicator of preload data register
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		preload_valid <= 1;
	else if (code_reload_d || phase_load_d || phase_init)
		preload_valid <= 0;
	else if (memcode_rd && memcode_read_valid)
		preload_valid <= 1;

// address to next
wire [13:0] base_address;
assign base_address = {start_index[8:0], 5'h0};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		memcode_addr <= 10'd0;
	else if (phase_init)
		memcode_addr <= base_address;
	else if (code_reload_d || phase_load_d)
		memcode_addr <= base_address + {5'h0, next_addr};

// indicator for read value valid
reg read_valid;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		read_valid <= 0;
	else if (memcode_rd && memcode_read_valid)
		read_valid <= 1;
	else
		read_valid <= 0;

// preload data register assignment
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		code_preload <= 32'd0;
	else if (read_valid)
		code_preload <= memcode_data;

assign memcode_rd = ~preload_valid;

// load preload value into code register on phase init
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		phase_init_flag <= 0;
	else if (phase_init)
		phase_init_flag <= 1;
	else if (read_valid)
		phase_init_flag <= 0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		load_init_data <= 0;
	else if (phase_init_flag && read_valid)
		load_init_data <= 1;
	else
		load_init_data <= 0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		ready_to_shift <= 1;
	else if (phase_init)
		ready_to_shift <= 0;
	else if (load_init_data)
		ready_to_shift <= 1;

//--------------------------------------------
// output prn code
//--------------------------------------------
reg code1_output, code2_output;

always @(*)
	case(bit_index)
	5'b00000:
		prn_code = current_code_r[31];
	5'b00001:
		prn_code = current_code_r[30];
	5'b00010:
		prn_code = current_code_r[29];
	5'b00011:
		prn_code = current_code_r[28];
	5'b00100:
		prn_code = current_code_r[27];
	5'b00101:
		prn_code = current_code_r[26];
	5'b00110:
		prn_code = current_code_r[25];
	5'b00111:
		prn_code = current_code_r[24];
	5'b01000:
		prn_code = current_code_r[23];
	5'b01001:
		prn_code = current_code_r[22];
	5'b01010:
		prn_code = current_code_r[21];
	5'b01011:
		prn_code = current_code_r[20];
	5'b01100:
		prn_code = current_code_r[19];
	5'b01101:
		prn_code = current_code_r[18];
	5'b01110:
		prn_code = current_code_r[17];
	5'b01111:
		prn_code = current_code_r[16];
	5'b10000:
		prn_code = current_code_r[15];
	5'b10001:
		prn_code = current_code_r[14];
	5'b10010:
		prn_code = current_code_r[13];
	5'b10011:
		prn_code = current_code_r[12];
	5'b10100:
		prn_code = current_code_r[11];
	5'b10101:
		prn_code = current_code_r[10];
	5'b10110:
		prn_code = current_code_r[ 9];
	5'b10111:
		prn_code = current_code_r[ 8];
	5'b11000:
		prn_code = current_code_r[ 7];
	5'b11001:
		prn_code = current_code_r[ 6];
	5'b11010:
		prn_code = current_code_r[ 5];
	5'b11011:
		prn_code = current_code_r[ 4];
	5'b11100:
		prn_code = current_code_r[ 3];
	5'b11101:
		prn_code = current_code_r[ 2];
	5'b11110:
		prn_code = current_code_r[ 1];
	5'b11111:
		prn_code = current_code_r[ 0];
	default:
		prn_code = 1'b0;
	endcase

assign current_code_o = current_code_r;
assign current_phase_o = {segment_index, word_index, bit_index};
assign prn_reset = skip && (segment_index_next == length);

endmodule
