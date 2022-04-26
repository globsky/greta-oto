//----------------------------------------------------------------------
// noise_calc.v:
//   Noise floor calculation module
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module noise_calc
(
// system signals
input clk,   // system clock
input rst_b, // reset signal, low active
// input data
input data_down_en,
input [5:0] i_data_down,
input [5:0] q_data_down,
// control signal
input shift_code,
input [1:0] smooth_factor,
input set_noise_floor,
input [15:0] noise_floor_i,
// output port
output [15:0] noise_floor
);

reg [23:0] noise_floor_r;
reg [9:0] prn_code;
reg [15:0] acc_i, acc_q;

//----------------------------------------------------------
// m serial
//----------------------------------------------------------
wire feedback;
assign feedback = prn_code[9] ^ prn_code[2];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		prn_code <= 10'h3ff;
	else if (shift_code)
		prn_code <= {prn_code[8:0], feedback};

wire last_sample;
reg [2:0] last_sample_d;
assign last_sample = (&{prn_code[8:0], feedback}) & shift_code;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		last_sample_d <= 3'b00;
	else
		last_sample_d <= {last_sample_d[1:0], last_sample};


//----------------------------------------------------------
// accumulate sample
//----------------------------------------------------------
wire signed [5:0] data_add_i, data_add_q;
assign data_add_i = prn_code[9] ? -i_data_down : i_data_down;
assign data_add_q = prn_code[9] ? -q_data_down : q_data_down;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		acc_i <= 'd0;
	else if (last_sample_d[0]) begin
		if (data_down_en)
			acc_i <= data_add_i;
		else
			acc_i <= 'd0;
	end
	else if (data_down_en)
		acc_i <= acc_i + {{10{data_add_i[5]}}, data_add_i};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		acc_q <= 'd0;
	else if (last_sample_d[0]) begin
		if (data_down_en)
			acc_q <= data_add_q;
		else
			acc_q <= 'd0;
	end
	else if (data_down_en)
			acc_q <= acc_q + {{10{data_add_q[5]}}, data_add_q};

//----------------------------------------------------------
// calculate smoothed noise floor
//----------------------------------------------------------
wire [15:0] amp_value;
reg [15:0] amp_value_r;
wire signed [15:0] adjust;
reg signed [15:0] adjust_scale;

amplitude #(.DATA_WIDTH(16)) u_amplitude
(
		.clk           (clk          ),
		.rst_b         (rst_b        ),
		.data_real     (last_sample_d[0] ? acc_i : 16'd0),
		.data_imag     (last_sample_d[0] ? acc_q : 16'd0),
		.data_amp      (amp_value    )
);

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		amp_value_r <= 'd0;
	else if (last_sample_d[1])
		amp_value_r <= amp_value;

assign adjust = amp_value_r - noise_floor;

always @(*)
	case(smooth_factor)
	2'b00:  adjust_scale = adjust;
	2'b01:  adjust_scale = {{2{adjust[15]}}, adjust[15:2]};
	2'b10:  adjust_scale = {{4{adjust[15]}}, adjust[15:4]};
	2'b11:  adjust_scale = {{6{adjust[15]}}, adjust[15:6]};
	default: adjust_scale = adjust;
	endcase

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		noise_floor_r <= {16'd784, 8'd0};
	else if (set_noise_floor)
		noise_floor_r <= {noise_floor_i, 8'd0};
	else if (last_sample_d[2])
		noise_floor_r <= noise_floor_r + {{8{adjust_scale[15]}}, adjust_scale};

assign 	noise_floor = noise_floor_r[23:8];

endmodule
