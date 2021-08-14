//----------------------------------------------------------------------
// ae_prn_gen.v:
//   AE PRN code generation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module ae_prn_gen
(
// system signals
input	clk,
input	rst_b,
// interface to set PRN parameters
input phase_init,
input [1:0] prn_select,
input [5:0] svid,
input shift_code,
output reg ready_to_shift,
output reg prn_code,
// Legendre code ROM interface
output [10:0] legendre_addr,	// MSB as weil select
output legendre_rd,
input legendre_read_valid,
input [15:0] legendre_data,
// memory code ROM interface
output [13:0] memcode_addr,
output memcode_rd,
input memcode_read_valid,
input [31:0] memcode_data
);

//--------------------------------------------------------------
// code, state and count multiplex
//--------------------------------------------------------------
wire select_general, select_weil, select_memcode;
assign select_general = (prn_select == 2'b00) ? 1 : 0;
assign select_memcode = (prn_select == 2'b01) ? 1 : 0;
assign select_weil = prn_select[1];

wire phase_init_general, phase_init_memcode, phase_init_weil;
assign phase_init_general = select_general & phase_init;
assign phase_init_memcode = select_memcode & phase_init;
assign phase_init_weil = select_weil & phase_init;

wire shift_code_general, shift_code_memcode, shift_code_weil;
assign shift_code_general = select_general & shift_code;
assign shift_code_memcode = select_memcode & shift_code;
assign shift_code_weil = select_weil & shift_code;

wire general_prn_code;
wire memcode_prn_code;
wire weil_prn_code;
wire general_ready;
wire memcode_ready;
wire weil_ready;

always @(*)
	case(prn_select)
		2'b00: prn_code = general_prn_code;
		2'b01: prn_code = memcode_prn_code;
		2'b10: prn_code = weil_prn_code;
		2'b11: prn_code = weil_prn_code;
		default: prn_code = general_prn_code;
	endcase

always @(*)
	case(prn_select)
		2'b00: ready_to_shift = 1'b1;
		2'b01: ready_to_shift = memcode_ready;
		2'b10: ready_to_shift = weil_ready;
		2'b11: ready_to_shift = weil_ready;
		default: ready_to_shift = general_ready;
	endcase

//--------------------------------------------------------------
// prn code generation instances
//--------------------------------------------------------------
reg [13:0] g1_init, g2_init;
reg [13:0] insert_index_b1c, weil_index_b1c;
reg [13:0] insert_index_l1c, weil_index_l1c;

wire [9:0] legendre_addr1, legendre_addr2;
wire legendre_rd1, legendre_rd2;
wire legendre_read_valid1, legendre_read_valid2;

m_prn_general #(.G12_LENGTH(14), .GLOBAL_LENGTH(18)) u_general_prn
(
  .clk             (clk                 ),
  .rst_b           (rst_b               ),
  .serial_parallel (1'b0                ),
  .g1_poly         (14'h204             ),
  .g2_poly         (14'h3a6             ),
  .gx_length       (32'h00ffc000        ),
  .g1_init_state   (g1_init             ),
  .g2_init_state   (g2_init             ),
  .g1_state_i      (g1_init             ),
  .g2_state_i      (g2_init             ),
  .gx_count_i      (32'd0               ),
  .state_load      (1'b0                ),
  .phase_load      (1'b0                ),
  .phase_init      (phase_init_general  ),
  .shift_code      (shift_code_general  ),
  .g1_state_o      (                    ),
  .g2_state_o      (                    ),
  .gx_count_o      (                    ),
  .prn_reset       (                    ),
  .prn_code        (general_prn_code    )
);

m_prn_memcode u_memcode_prn
(
	.clk             (clk                 ),
	.rst_b           (rst_b               ),
	.start_index     ({3'b000, 7'd49+svid, 2'b00}),
	.length          (4'd4                ),
	.current_code_i  (32'h0               ),
	.current_phase_i (14'h0               ),
	.code_load       (1'b0                ),
	.phase_load      (1'b0                ),
  .phase_init      (phase_init_memcode  ),
	.memcode_addr    (memcode_addr        ),
	.memcode_rd      (memcode_rd          ),
	.memcode_read_valid (memcode_read_valid),
	.memcode_data    (memcode_data        ),
	.shift_code      (shift_code_memcode  ),
	.current_code_o  (                    ),
	.current_phase_o (                    ),
  .prn_reset       (                    ),
  .ready_to_shift  (memcode_ready       ),
	.prn_code        (memcode_prn_code    )
);

m_prn_weil u_weil_prn
(
	.clk             (clk                 ),
	.rst_b           (rst_b               ),
	.weil_type       (prn_select[0]       ),
	.insertion_index (prn_select[0] ? insert_index_l1c : insert_index_b1c),
	.weil_index      (prn_select[0] ? weil_index_l1c : weil_index_b1c),
	.legendre_code1_i(16'h0               ),
	.legendre_code2_i(16'h0               ),
	.code_phase_i    (14'h0               ),
	.code_load       (1'b0                ),
	.phase_load      (1'b0                ),
  .phase_init      (phase_init_weil     ),
	.legendre_addr1  (legendre_addr1      ),
	.legendre_rd1    (legendre_rd1        ),
	.preempt1        (                    ),
	.legendre_addr2  (legendre_addr2      ),
	.legendre_rd2    (legendre_rd2        ),
	.preempt2        (                    ),
	.legendre_read_valid1 (legendre_read_valid1),
	.legendre_read_valid2 (legendre_read_valid2),
	.legendre_data   (legendre_data       ),
	.shift_code      (shift_code_weil     ),
	.legendre_code1_o(                    ),
	.legendre_code2_o(                    ),
	.code_phase_o    (                    ),
  .prn_reset       (                    ),
  .ready_to_shift  (weil_ready          ),
	.prn_code        (weil_prn_code       )
);

// arbiter for weil data read signal
assign legendre_read_valid1 = legendre_read_valid & legendre_rd1;
assign legendre_read_valid2 = legendre_read_valid & (~legendre_rd1) & legendre_rd2;
assign legendre_addr = {prn_select[0], (legendre_rd1 ? legendre_addr1 : legendre_addr2)};
assign legendre_rd = legendre_rd1 | legendre_rd2;

//--------------------------------------------
// G1 and G2 initial value selection
//--------------------------------------------
always @(*)
	case (svid)
		6'd1 : {g2_init, g1_init} = 28'h37ffff1;	// for PRN1 
		6'd2 : {g2_init, g1_init} = 28'h1bffff1;	// for PRN2 
		6'd3 : {g2_init, g1_init} = 28'h0dffff1;	// for PRN3 
		6'd4 : {g2_init, g1_init} = 28'h06ffff1;	// for PRN4 
		6'd5 : {g2_init, g1_init} = 28'h6903ff1;	// for PRN5 
		6'd6 : {g2_init, g1_init} = 28'h3483ff1;	// for PRN6 
		6'd7 : {g2_init, g1_init} = 28'h69bbff1;	// for PRN7 
		6'd8 : {g2_init, g1_init} = 28'h34dfff1;	// for PRN8 
		6'd9 : {g2_init, g1_init} = 28'h1a6fff1;	// for PRN9 
		6'd10: {g2_init, g1_init} = 28'h2eefff1;	// for PRN10
		6'd11: {g2_init, g1_init} = 28'h1777ff1;	// for PRN11
		6'd12: {g2_init, g1_init} = 28'h05dfff1;	// for PRN12
		6'd13: {g2_init, g1_init} = 28'h02efff1;	// for PRN13
		6'd14: {g2_init, g1_init} = 28'h0177ff1;	// for PRN14
		6'd15: {g2_init, g1_init} = 28'h00bbff1;	// for PRN15
		6'd16: {g2_init, g1_init} = 28'h005fff1;	// for PRN16
		6'd17: {g2_init, g1_init} = 28'h6447ff1;	// for PRN17
		6'd18: {g2_init, g1_init} = 28'h3223ff1;	// for PRN18
		6'd19: {g2_init, g1_init} = 28'h1913ff1;	// for PRN19
		6'd20: {g2_init, g1_init} = 28'h0c8bff1;	// for PRN20
		6'd21: {g2_init, g1_init} = 28'h0647ff1;	// for PRN21
		6'd22: {g2_init, g1_init} = 28'h0323ff1;	// for PRN22
		6'd23: {g2_init, g1_init} = 28'h7333ff1;	// for PRN23
		6'd24: {g2_init, g1_init} = 28'h0e67ff1;	// for PRN24
		6'd25: {g2_init, g1_init} = 28'h0733ff1;	// for PRN25
		6'd26: {g2_init, g1_init} = 28'h039bff1;	// for PRN26
		6'd27: {g2_init, g1_init} = 28'h01cfff1;	// for PRN27
		6'd28: {g2_init, g1_init} = 28'h00e7ff1;	// for PRN28
		6'd29: {g2_init, g1_init} = 28'h6a23ff1;	// for PRN29
		6'd30: {g2_init, g1_init} = 28'h3513ff1;	// for PRN30
		6'd31: {g2_init, g1_init} = 28'h1a8bff1;	// for PRN31
		6'd32: {g2_init, g1_init} = 28'h0d47ff1;	// for PRN32
		6'd33: {g2_init, g1_init} = 28'h91a7ff1;	// for PRN120
		6'd34: {g2_init, g1_init} = 28'ha863ff1;	// for PRN121
		6'd35: {g2_init, g1_init} = 28'h2dcfff1;	// for PRN122
		6'd36: {g2_init, g1_init} = 28'h2693ff1;	// for PRN123
		6'd37: {g2_init, g1_init} = 28'he3e3ff1;	// for PRN124
		6'd38: {g2_init, g1_init} = 28'h8f87ff1;	// for PRN125
		6'd39: {g2_init, g1_init} = 28'hfd27ff1;	// for PRN126
		6'd40: {g2_init, g1_init} = 28'h73d7ff1;	// for PRN127
		6'd41: {g2_init, g1_init} = 28'hd6afff1;	// for PRN128
		6'd42: {g2_init, g1_init} = 28'haa37ff1;	// for PRN129
		6'd43: {g2_init, g1_init} = 28'h3857ff1;	// for PRN130
		6'd44: {g2_init, g1_init} = 28'h5a57ff1;	// for PRN131
		6'd45: {g2_init, g1_init} = 28'h5433ff1;	// for PRN132
		6'd46: {g2_init, g1_init} = 28'hf67bff1;	// for PRN133
		6'd47: {g2_init, g1_init} = 28'h7183ff1;	// for PRN134
		6'd48: {g2_init, g1_init} = 28'ha387ff1;	// for PRN135
		6'd49: {g2_init, g1_init} = 28'h7833ff1;	// for PRN136
		6'd50: {g2_init, g1_init} = 28'h81e3ff1;	// for PRN137
		6'd51: {g2_init, g1_init} = 28'h4a13ff1;	// for PRN138
		default:  {g2_init, g1_init} = 28'h0;
	endcase

//--------------------------------------------
// B1C initial value selection
//--------------------------------------------
always @(*)
	case (svid)
		6'd1 : {insert_index_b1c, weil_index_b1c} = 28'h765831c;	// for PRN01
		6'd2 : {insert_index_b1c, weil_index_b1c} = 28'h250009c;	// for PRN02
		6'd3 : {insert_index_b1c, weil_index_b1c} = 28'h58dd066;	// for PRN03
		6'd4 : {insert_index_b1c, weil_index_b1c} = 28'h0868f65;	// for PRN04
		6'd5 : {insert_index_b1c, weil_index_b1c} = 28'h237455e;	// for PRN05
		6'd6 : {insert_index_b1c, weil_index_b1c} = 28'h722453a;	// for PRN06
		6'd7 : {insert_index_b1c, weil_index_b1c} = 28'h64e0729;	// for PRN07
		6'd8 : {insert_index_b1c, weil_index_b1c} = 28'h61b49d9;	// for PRN08
		6'd9 : {insert_index_b1c, weil_index_b1c} = 28'h582cc67;	// for PRN09
		6'd10: {insert_index_b1c, weil_index_b1c} = 28'h6f380a8;	// for PRN10
		6'd11: {insert_index_b1c, weil_index_b1c} = 28'h15e4a9b;	// for PRN11
		6'd12: {insert_index_b1c, weil_index_b1c} = 28'h56d1138;	// for PRN12
		6'd13: {insert_index_b1c, weil_index_b1c} = 28'h5a0cc58;	// for PRN13
		6'd14: {insert_index_b1c, weil_index_b1c} = 28'h10c0aec;	// for PRN14
		6'd15: {insert_index_b1c, weil_index_b1c} = 28'h6d601cb;	// for PRN15
		6'd16: {insert_index_b1c, weil_index_b1c} = 28'h5c54e0a;	// for PRN16
		6'd17: {insert_index_b1c, weil_index_b1c} = 28'h9d2d2cd;	// for PRN17
		6'd18: {insert_index_b1c, weil_index_b1c} = 28'h2a5424a;	// for PRN18
		6'd19: {insert_index_b1c, weil_index_b1c} = 28'h1824594;	// for PRN19
		6'd20: {insert_index_b1c, weil_index_b1c} = 28'h6b98943;	// for PRN20
		6'd21: {insert_index_b1c, weil_index_b1c} = 28'h1d688ed;	// for PRN21
		6'd22: {insert_index_b1c, weil_index_b1c} = 28'h57b0d31;	// for PRN22
		6'd23: {insert_index_b1c, weil_index_b1c} = 28'h4f15365;	// for PRN23
		6'd24: {insert_index_b1c, weil_index_b1c} = 28'h1034ec3;	// for PRN24
		6'd25: {insert_index_b1c, weil_index_b1c} = 28'h9ee51c3;	// for PRN25
		6'd26: {insert_index_b1c, weil_index_b1c} = 28'h654c66e;	// for PRN26
		6'd27: {insert_index_b1c, weil_index_b1c} = 28'h1ad4596;	// for PRN27
		6'd28: {insert_index_b1c, weil_index_b1c} = 28'h279825f;	// for PRN28
		6'd29: {insert_index_b1c, weil_index_b1c} = 28'h1214846;	// for PRN29
		6'd30: {insert_index_b1c, weil_index_b1c} = 28'h0835265;	// for PRN30
		6'd31: {insert_index_b1c, weil_index_b1c} = 28'h728847d;	// for PRN31
		6'd32: {insert_index_b1c, weil_index_b1c} = 28'h5b4ccd3;	// for PRN32
		6'd33: {insert_index_b1c, weil_index_b1c} = 28'h64589a9;	// for PRN33
		6'd34: {insert_index_b1c, weil_index_b1c} = 28'h6cdc3ee;	// for PRN34
		6'd35: {insert_index_b1c, weil_index_b1c} = 28'h13fce56;	// for PRN35
		6'd36: {insert_index_b1c, weil_index_b1c} = 28'h1cb4719;	// for PRN36
		6'd37: {insert_index_b1c, weil_index_b1c} = 28'h1f10303;	// for PRN37
		6'd38: {insert_index_b1c, weil_index_b1c} = 28'h650c87d;	// for PRN38
		6'd39: {insert_index_b1c, weil_index_b1c} = 28'h20a82e4;	// for PRN39
		6'd40: {insert_index_b1c, weil_index_b1c} = 28'h18b0599;	// for PRN40
		6'd41: {insert_index_b1c, weil_index_b1c} = 28'h16b099a;	// for PRN41
		6'd42: {insert_index_b1c, weil_index_b1c} = 28'h61acd83;	// for PRN42
		6'd43: {insert_index_b1c, weil_index_b1c} = 28'h6f4486b;	// for PRN43
		6'd44: {insert_index_b1c, weil_index_b1c} = 28'h78784b5;	// for PRN44
		6'd45: {insert_index_b1c, weil_index_b1c} = 28'h70bc19d;	// for PRN45
		6'd46: {insert_index_b1c, weil_index_b1c} = 28'h210036a;	// for PRN46
		6'd47: {insert_index_b1c, weil_index_b1c} = 28'h111899f;	// for PRN47
		6'd48: {insert_index_b1c, weil_index_b1c} = 28'h196c452;	// for PRN48
		6'd49: {insert_index_b1c, weil_index_b1c} = 28'h1ac0636;	// for PRN49
		6'd50: {insert_index_b1c, weil_index_b1c} = 28'h5f54f21;	// for PRN50
		6'd51: {insert_index_b1c, weil_index_b1c} = 28'h5fa8fba;	// for PRN51
		6'd52: {insert_index_b1c, weil_index_b1c} = 28'h5ed50b0;	// for PRN52
		6'd53: {insert_index_b1c, weil_index_b1c} = 28'h1168de4;	// for PRN53
		6'd54: {insert_index_b1c, weil_index_b1c} = 28'h7db8080;	// for PRN54
		6'd55: {insert_index_b1c, weil_index_b1c} = 28'h6a284b0;	// for PRN55
		6'd56: {insert_index_b1c, weil_index_b1c} = 28'h2838082;	// for PRN56
		6'd57: {insert_index_b1c, weil_index_b1c} = 28'h00d118e;	// for PRN57
		6'd58: {insert_index_b1c, weil_index_b1c} = 28'h1b0074f;	// for PRN58
		6'd59: {insert_index_b1c, weil_index_b1c} = 28'h63ccc01;	// for PRN59
		6'd60: {insert_index_b1c, weil_index_b1c} = 28'h0aa5122;	// for PRN60
		6'd61: {insert_index_b1c, weil_index_b1c} = 28'h56f1002;	// for PRN61
		6'd62: {insert_index_b1c, weil_index_b1c} = 28'h6fdc783;	// for PRN62
		6'd63: {insert_index_b1c, weil_index_b1c} = 28'h2390498;	// for PRN63
		default: {insert_index_b1c, weil_index_b1c} = 28'h0;
	endcase

//--------------------------------------------
// B1C initial value selection
//--------------------------------------------
always @(*)
	case (svid)
		6'd1 : {insert_index_l1c, weil_index_l1c} = 28'h02d13e9;	// for PRN01
		6'd2 : {insert_index_l1c, weil_index_l1c} = 28'h05993f6;	// for PRN02
		6'd3 : {insert_index_l1c, weil_index_l1c} = 28'h011d3d7;	// for PRN03
		6'd4 : {insert_index_l1c, weil_index_l1c} = 28'h1155133;	// for PRN04
		6'd5 : {insert_index_l1c, weil_index_l1c} = 28'h171d019;	// for PRN05
		6'd6 : {insert_index_l1c, weil_index_l1c} = 28'h4ea53b3;	// for PRN06
		6'd7 : {insert_index_l1c, weil_index_l1c} = 28'h48353b2;	// for PRN07
		6'd8 : {insert_index_l1c, weil_index_l1c} = 28'h00013f0;	// for PRN08
		6'd9 : {insert_index_l1c, weil_index_l1c} = 28'h470934c;	// for PRN09
		6'd10: {insert_index_l1c, weil_index_l1c} = 28'h0ce53ab;	// for PRN10
		6'd11: {insert_index_l1c, weil_index_l1c} = 28'h622d114;	// for PRN11
		6'd12: {insert_index_l1c, weil_index_l1c} = 28'h41893c8;	// for PRN12
		6'd13: {insert_index_l1c, weil_index_l1c} = 28'h05bd3dc;	// for PRN13
		6'd14: {insert_index_l1c, weil_index_l1c} = 28'h00013b8;	// for PRN14
		6'd15: {insert_index_l1c, weil_index_l1c} = 28'h4aed356;	// for PRN15
		6'd16: {insert_index_l1c, weil_index_l1c} = 28'h082939b;	// for PRN16
		6'd17: {insert_index_l1c, weil_index_l1c} = 28'h02593d4;	// for PRN17
		6'd18: {insert_index_l1c, weil_index_l1c} = 28'h0b20e98;	// for PRN18
		6'd19: {insert_index_l1c, weil_index_l1c} = 28'h99e5381;	// for PRN19
		6'd20: {insert_index_l1c, weil_index_l1c} = 28'h59953c4;	// for PRN20
		6'd21: {insert_index_l1c, weil_index_l1c} = 28'h00853c5;	// for PRN21
		6'd22: {insert_index_l1c, weil_index_l1c} = 28'h5ff53e8;	// for PRN22
		6'd23: {insert_index_l1c, weil_index_l1c} = 28'h02f5377;	// for PRN23
		6'd24: {insert_index_l1c, weil_index_l1c} = 28'h0a0d2af;	// for PRN24
		6'd25: {insert_index_l1c, weil_index_l1c} = 28'h074937f;	// for PRN25
		6'd26: {insert_index_l1c, weil_index_l1c} = 28'h541d2cf;	// for PRN26
		6'd27: {insert_index_l1c, weil_index_l1c} = 28'h0c8115b;	// for PRN27
		6'd28: {insert_index_l1c, weil_index_l1c} = 28'h09452a1;	// for PRN28
		6'd29: {insert_index_l1c, weil_index_l1c} = 28'h458530f;	// for PRN29
		6'd30: {insert_index_l1c, weil_index_l1c} = 28'h937131e;	// for PRN30
		6'd31: {insert_index_l1c, weil_index_l1c} = 28'h4349379;	// for PRN31
		6'd32: {insert_index_l1c, weil_index_l1c} = 28'h5c453c0;	// for PRN32
		6'd33: {insert_index_l1c, weil_index_l1c} = 28'h05e5339;	// for PRN33
		6'd34: {insert_index_l1c, weil_index_l1c} = 28'h939d3ac;	// for PRN34
		6'd35: {insert_index_l1c, weil_index_l1c} = 28'h935d2cc;	// for PRN35
		6'd36: {insert_index_l1c, weil_index_l1c} = 28'h5b612e6;	// for PRN36
		6'd37: {insert_index_l1c, weil_index_l1c} = 28'h56a92f7;	// for PRN37
		6'd38: {insert_index_l1c, weil_index_l1c} = 28'h9525328;	// for PRN38
		6'd39: {insert_index_l1c, weil_index_l1c} = 28'h8ead291;	// for PRN39
		6'd40: {insert_index_l1c, weil_index_l1c} = 28'h0649183;	// for PRN40
		6'd41: {insert_index_l1c, weil_index_l1c} = 28'h3ad534e;	// for PRN41
		6'd42: {insert_index_l1c, weil_index_l1c} = 28'h00092cd;	// for PRN42
		6'd43: {insert_index_l1c, weil_index_l1c} = 28'h0aad35d;	// for PRN43
		6'd44: {insert_index_l1c, weil_index_l1c} = 28'h97b920a;	// for PRN44
		6'd45: {insert_index_l1c, weil_index_l1c} = 28'h053123d;	// for PRN45
		6'd46: {insert_index_l1c, weil_index_l1c} = 28'h5fad369;	// for PRN46
		6'd47: {insert_index_l1c, weil_index_l1c} = 28'h9f9d3a7;	// for PRN47
		6'd48: {insert_index_l1c, weil_index_l1c} = 28'h42693ae;	// for PRN48
		6'd49: {insert_index_l1c, weil_index_l1c} = 28'h9a91284;	// for PRN49
		6'd50: {insert_index_l1c, weil_index_l1c} = 28'h9a6cfe9;	// for PRN50
		6'd51: {insert_index_l1c, weil_index_l1c} = 28'h48492eb;	// for PRN51
		6'd52: {insert_index_l1c, weil_index_l1c} = 28'h4581373;	// for PRN52
		6'd53: {insert_index_l1c, weil_index_l1c} = 28'h9915303;	// for PRN53
		6'd54: {insert_index_l1c, weil_index_l1c} = 28'h0f61364;	// for PRN54
		6'd55: {insert_index_l1c, weil_index_l1c} = 28'h42bd3a1;	// for PRN55
		6'd56: {insert_index_l1c, weil_index_l1c} = 28'h01f51e3;	// for PRN56
		6'd57: {insert_index_l1c, weil_index_l1c} = 28'h9c9d126;	// for PRN57
		6'd58: {insert_index_l1c, weil_index_l1c} = 28'h06c529b;	// for PRN58
		6'd59: {insert_index_l1c, weil_index_l1c} = 28'h1011204;	// for PRN59
		6'd60: {insert_index_l1c, weil_index_l1c} = 28'h08c12b0;	// for PRN60
		6'd61: {insert_index_l1c, weil_index_l1c} = 28'h0480e84;	// for PRN61
		6'd62: {insert_index_l1c, weil_index_l1c} = 28'h09f525f;	// for PRN62
		6'd63: {insert_index_l1c, weil_index_l1c} = 28'h44012f3;	// for PRN63
		default: {insert_index_l1c, weil_index_l1c} = 28'h0;
	endcase

endmodule
