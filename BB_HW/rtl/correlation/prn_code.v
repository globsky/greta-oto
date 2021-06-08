//----------------------------------------------------------------------
// prn_code.v:
//   Encapsulated PRN code generation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module m_prn_code
(
// system signals
input	clk,
input	rst_b,
// polynimial and length
input [31:0]  te_polynomial,  // te polynomial register
input [31:0]  te_code_length, // te code length register
input [31:0]  te_polynomial2, // te polynomial register for secondary PRN
input [31:0]  te_code_length2,// te code length register for secondary PRN
input [31:0]  prn_config,			// PRN config
input [31:0]  prn2_config,		// PRN2 config
input prn_state_en,  // PRN state load enable
input [31:0]  prn_state_i, // PRN state load value
output reg [31:0] prn_state_o, // PRN state output
input prn_count_en,	// PRN count load enable
input [31:0] prn_count_i,		// PRN counter load value
output reg [31:0] prn_count_o,	// PRN counter output
input prn2_state_en,  // PRN2 state load enable
input [31:0]  prn2_state_i, // PRN2 state load value
output reg [31:0] prn2_state_o, // PRN2 state output
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
// control signal
input shift_code,
input enable_2nd_prn,	// enable second prn
output reg prn_reset,		// PRN code round back
// PRN code output
output reg prn_code1,
output reg prn_code2
);

//--------------------------------------------------------------
// code, state and count multiplex
//--------------------------------------------------------------
wire select_general, select_weil, select_memcode;
assign select_general = (prn_config[31:30] == 2'b00) ? 1 : 0;
assign select_weil = (prn_config[31:30] == 2'b10) ? 1 : 0;
assign select_memcode = (prn_config[31:30] == 2'b11) ? 1 : 0;

wire shift_code_general, shift_code_weil, shift_code_memcode;
assign shift_code_general = select_general & shift_code;
assign shift_code_weil = select_weil & shift_code;
assign shift_code_memcode = select_memcode & shift_code;

wire general_prn_reset, weil_prn_reset, memcode_prn_reset;
wire general_prn_code1, general_prn_code2;
wire weil_prn_code1, weil_prn_code2;
wire memcode_prn_code1, memcode_prn_code2;

always @(*)
	case(prn_config[31:30])
		2'b00: prn_code1 = general_prn_code1;
		2'b10: prn_code1 = weil_prn_code1;
		2'b11: prn_code1 = memcode_prn_code1;
		default: prn_code1 = general_prn_code1;
	endcase

always @(*)
	case(prn_config[31:30])
		2'b00: prn_code2 = general_prn_code2;
		2'b10: prn_code2 = weil_prn_code2;
		2'b11: prn_code2 = memcode_prn_code2;
		default: prn_code2 = general_prn_code2;
	endcase

always @(*)
	case(prn_config[31:30])
		2'b00: prn_reset = general_prn_reset;
		2'b10: prn_reset = weil_prn_reset;
		2'b11: prn_reset = memcode_prn_reset;
		default: prn_reset = general_prn_reset;
	endcase

wire [13:0] g1_state1_o, g2_state1_o;
wire [13:0] g1_state2_o, g2_state2_o;
wire [31:0] gx_count_o;
wire [15:0] legendre_code11_o, legendre_code12_o;
wire [15:0] legendre_code21_o, legendre_code22_o;
wire [13:0] weil_code_phase_o;
wire [31:0] memcode_code1_o;
wire [31:0] memcode_code2_o;
wire [13:0] memcode_phase_o;

always @(*)
	case(prn_config[31:30])
		2'b00: prn_state_o = {4'h0, g2_state1_o, g1_state1_o};
		2'b10: prn_state_o = {legendre_code12_o, legendre_code11_o};
		2'b11: prn_state_o = memcode_code1_o;
		default: prn_state_o = 32'h0;
	endcase

always @(*)
	case(prn_config[31:30])
		2'b00: prn2_state_o = {4'h0, g2_state2_o, g1_state2_o};
		2'b10: prn2_state_o = {legendre_code22_o, legendre_code21_o};
		2'b11: prn2_state_o = memcode_code2_o;
		default: prn2_state_o = 32'h0;
	endcase

always @(*)
	case(prn_config[31:30])
		2'b00: prn_count_o = gx_count_o;
		2'b10: prn_count_o = {18'h0, weil_code_phase_o};
		2'b11: prn_count_o = {18'h0, memcode_phase_o};
		default: prn_count_o = 32'h0;
	endcase

// phase will be used to count Legendre index or calculate address of preload memory code
// for PRN2, prn2_config is valid after load DWORD14, which is later than phase load
// to store current phase for second PRN and make phase load valid right after code load
reg [13:0] phase_save;
always @(posedge clk or negedge rst_b)
	if (~rst_b)
		phase_save <= 'h0;
	else if (prn_count_en)
		phase_save <= prn_count_i[13:0];

reg phase2_load;
always @(posedge clk or negedge rst_b)
	if (~rst_b)
		phase2_load <= 1'b0;
	else
		phase2_load <= prn2_state_en;

//--------------------------------------------------------------
// general prn code generation
//--------------------------------------------------------------
m_prn_general #(.G12_LENGTH(14), .GLOBAL_LENGTH(18)) general_prn1
(
	.clk             (clk                  ),
	.rst_b           (rst_b                ),
	.serial_parallel (te_polynomial[31]    ),
	.g1_poly         (te_polynomial[13:0]  ),
	.g2_poly         (te_polynomial[27:14] ),
	.gx_length       (te_code_length       ),
	.g1_init_state   (prn_config[13:0]     ),
	.g2_init_state   (prn_config[27:14]    ),
	.g1_state_i      (prn_state_i[13:0]    ),
	.g2_state_i      (prn_state_i[27:14]   ),
	.gx_count_i      (prn_count_i          ),
	.state_load      (prn_state_en         ),
	.phase_load      (prn_count_en         ),
	.phase_init      (1'b0                 ),
	.shift_code      (shift_code_general   ),
	.g1_state_o      (g1_state1_o          ),
	.g2_state_o      (g2_state1_o          ),
	.gx_count_o      (gx_count_o           ),
	.prn_reset       (general_prn_reset    ),
	.prn_code        (general_prn_code1    )
);

m_prn_general #(.G12_LENGTH(14), .GLOBAL_LENGTH(18)) general_prn2
(
	.clk             (clk                  ),
	.rst_b           (rst_b                ),
	.serial_parallel (te_polynomial2[31]   ),
	.g1_poly         (te_polynomial2[13:0] ),
	.g2_poly         (te_polynomial2[27:14]),
	.gx_length       (te_code_length2      ),
	.g1_init_state   (prn2_config[13:0]    ),
	.g2_init_state   (prn2_config[27:14]   ),
	.g1_state_i      (prn2_state_i[13:0]   ),
	.g2_state_i      (prn2_state_i[27:14]  ),
	.gx_count_i      (prn_count_i          ),
	.state_load      (prn2_state_en        ),
	.phase_load      (prn_count_en         ),
	.phase_init      (1'b0                 ),
	.shift_code      (shift_code_general & enable_2nd_prn),
	.g1_state_o      (g1_state2_o          ),
	.g2_state_o      (g2_state2_o          ),
	.gx_count_o      (                     ),
	.prn_reset       (                     ),
	.prn_code        (general_prn_code2    )
);

//--------------------------------------------------------------
// Weil prn code generation
//--------------------------------------------------------------
// signals for Weil code
wire [9:0] legendre_addr1, legendre_addr2;
wire [9:0] legendre_addr3, legendre_addr4;
wire legendre_rd1, legendre_rd2;
wire legendre_rd3, legendre_rd4;
wire preempt1, preempt2;
wire preempt3, preempt4;
wire legendre_read_valid1;
wire legendre_read_valid2;
wire legendre_read_valid3;
wire legendre_read_valid4;
wire [15:0] legendre_read_data;

m_prn_weil weil_prn1
(
	.clk             (clk                 ),
	.rst_b           (rst_b               ),
	.weil_type       (prn_config[29]      ),
	.insertion_index (prn_config[27:14]   ),
	.weil_index      (prn_config[13:0]    ),
	.legendre_code1_i(prn_state_i[15:0]   ),
	.legendre_code2_i(prn_state_i[31:16]  ),
	.code_phase_i    (prn_count_i[13:0]   ),
	.code_load       (prn_state_en        ),
	.phase_load      (prn_count_en        ),
	.phase_init      (1'b0                ),
	.legendre_addr1  (legendre_addr1      ),
	.legendre_rd1    (legendre_rd1        ),
	.preempt1        (preempt1            ),
	.legendre_addr2  (legendre_addr2      ),
	.legendre_rd2    (legendre_rd2        ),
	.preempt2        (preempt2            ),
	.legendre_read_valid1 (legendre_read_valid1),
	.legendre_read_valid2 (legendre_read_valid2),
	.legendre_data   (legendre_read_data  ),
	.shift_code      (shift_code_weil     ),
	.legendre_code1_o(legendre_code11_o   ),
	.legendre_code2_o(legendre_code12_o   ),
	.code_phase_o    (weil_code_phase_o   ),
	.prn_reset       (weil_prn_reset      ),
	.ready_to_shift  (                    ),
	.prn_code        (weil_prn_code1      )
);

m_prn_weil weil_prn2
(
	.clk             (clk                 ),
	.rst_b           (rst_b               ),
	.weil_type       (prn2_config[29]     ),
	.insertion_index (prn2_config[27:14]  ),
	.weil_index      (prn2_config[13:0]   ),
	.legendre_code1_i(prn2_state_i[15:0]  ),
	.legendre_code2_i(prn2_state_i[31:16] ),
	.code_phase_i    (phase_save          ),
	.code_load       (prn2_state_en       ),
	.phase_load      (phase2_load         ),
	.phase_init      (1'b0                ),
	.legendre_addr1  (legendre_addr3      ),
	.legendre_rd1    (legendre_rd3        ),
	.preempt1        (preempt3            ),
	.legendre_addr2  (legendre_addr4      ),
	.legendre_rd2    (legendre_rd4        ),
	.preempt2        (preempt4            ),
	.legendre_read_valid1 (legendre_read_valid3),
	.legendre_read_valid2 (legendre_read_valid4),
	.legendre_data   (legendre_read_data  ),
	.shift_code      (shift_code_weil & enable_2nd_prn),
	.legendre_code1_o(legendre_code21_o   ),
	.legendre_code2_o(legendre_code22_o   ),
	.code_phase_o    (                    ),
	.prn_reset       (                    ),
	.ready_to_shift  (                    ),
	.prn_code        (weil_prn_code2      )
);

// ROM arbiter for Weil code
m_rom_arbiter_preempt #(.ADDR_WIDTH(10), .DATA_WIDTH(16)) u_rom_arbiter
(
	.rd0           (legendre_rd1        ),
	.preempt0      (preempt1            ),
	.addr0         (legendre_addr1      ),
	.accept0       (legendre_read_valid1),
	.rd1           (legendre_rd2        ),
	.preempt1      (preempt2            ),
	.addr1         (legendre_addr2      ),
	.accept1       (legendre_read_valid2),
	.rd2           (legendre_rd3        ),
	.preempt2      (preempt3            ),
	.addr2         (legendre_addr3      ),
	.accept2       (legendre_read_valid3),
	.rd3           (legendre_rd4        ),
	.preempt3      (preempt4            ),
	.addr3         (legendre_addr4      ),
	.accept3       (legendre_read_valid4),
	.data          (legendre_read_data  ),
	.mem_rd        (legendre_rd         ),
	.mem_addr      (legendre_addr       ),
	.mem_accept    (legendre_read_valid ),
	.mem_d4rd      (legendre_data       )
);

//--------------------------------------------------------------
// memory PRN code1 generation
//--------------------------------------------------------------
// signals for memory code
wire [13:0] memcode_addr1, memcode_addr2;
wire memcode_rd1, memcode_rd2;
wire memcode_read_valid1;
wire memcode_read_valid2;

m_prn_memcode memcode_prn1
(
	.clk             (clk                 ),
	.rst_b           (rst_b               ),
	.start_index     (prn_config[15:4]    ),
	.length          (prn_config[3:0]     ),
	.current_code_i  (prn_state_i         ),
	.current_phase_i (prn_count_i[13:0]   ),
	.code_load       (prn_state_en        ),
	.phase_load      (prn_count_en        ),
	.phase_init      (1'b0                ),
	.memcode_addr    (memcode_addr1       ),
	.memcode_rd      (memcode_rd1         ),
	.memcode_read_valid (memcode_read_valid1),
	.memcode_data    (memcode_data        ),
	.shift_code      (shift_code_memcode  ),
	.current_code_o  (memcode_code1_o     ),
	.current_phase_o (memcode_phase_o     ),
  .prn_reset       (memcode_prn_reset   ),
	.ready_to_shift  (                    ),
	.prn_code        (memcode_prn_code1   )
);

m_prn_memcode memcode_prn2
(
	.clk             (clk                 ),
	.rst_b           (rst_b               ),
	.start_index     (prn2_config[15:4]   ),
	.length          (prn2_config[3:0]    ),
	.current_code_i  (prn2_state_i        ),
	.current_phase_i (phase_save          ),
	.code_load       (prn2_state_en       ),
	.phase_load      (phase2_load         ),
	.phase_init      (1'b0                ),
	.memcode_addr    (memcode_addr2       ),
	.memcode_rd      (memcode_rd2         ),
	.memcode_read_valid (memcode_read_valid2),
	.memcode_data    (memcode_data        ),
	.shift_code      (shift_code_memcode & enable_2nd_prn),
	.current_code_o  (memcode_code2_o     ),
	.current_phase_o (                    ),
  .prn_reset       (                    ),
	.ready_to_shift  (                    ),
	.prn_code        (memcode_prn_code2   )
);

// arbiter for memory code read signal
assign memcode_read_valid1 = memcode_read_valid & memcode_rd1;
assign memcode_read_valid2 = memcode_read_valid & (~memcode_rd1) & memcode_rd2;
assign memcode_addr = memcode_rd1 ? memcode_addr1 : memcode_addr2;
assign memcode_rd = memcode_rd1 | memcode_rd2;


endmodule
