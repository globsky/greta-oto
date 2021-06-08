//----------------------------------------------------------------------
// noncoh_acc.v:
//   non-coherent accumulate module
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module noncoh_acc #(parameter DATA_LENGTH = 682)
(
// system signals				
input clk,	//system clock
input rst_b,	//reset signal, low active

// interface to coherent memory
input	coh_rd_valid,		// coherent read address
input	[9:0] coh_read_addr,	// address read from coherent RAM
input	[191:0] coh_d4rd,

// interface to control the behavior of non-coherent sum
input active_acc,	// valid coherent data to accumulate into noncoherent RAM, align with coh_d4rd
input	first_acc,	// first round accumulation, equavelent to NoncohCount == 0, align with coh_d4rd
input	last_acc,	// last round accumulation, equavelent to NoncohCount == (NonCoherentNumber - 1), align with coh_d4rd
input	[3:0] max_exp,	// max exponential of coherent data

// non-coherent result output
output [7:0] noncoh_out_amp,
output [3:0] noncoh_out_exp,
output [9:0] noncoh_out_pos,
output [2:0] noncoh_out_freq,
output noncoh_out_valid,
// output noise floor
output reg [17:0] noise_floor,
//output reg noise_floor_valid,

// interface to non-coherent memory
output rd,
output we,
output [9:0] addr,	// 682 depth
output [63:0] d4wt,	// 8x8 width
input [63:0] d4rd
);

//----------------------------------------------------------
// calculate coherent and noncoherent shift bit number
//----------------------------------------------------------
reg first_acc_d;
reg [3:0] noncoh_exp;
reg first_coh;
reg [3:0] shift_coh, shift_noncoh;
reg exceed;
wire [4:0] exp_diff;
assign exp_diff = max_exp - noncoh_exp;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		first_acc_d <= 1'b0;
	else
		first_acc_d <= first_acc;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		first_coh <= 1'b0;
	else
		first_coh <= coh_rd_valid && (coh_read_addr == 0);

// assign noncoh_exp
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		noncoh_exp <= 4'd0;
	else if (first_acc && ~first_acc_d)		// clear noncoh_exp on rising edge of first_acc
		noncoh_exp <= max_exp;
	else if (first_coh && active_acc)
		noncoh_exp <= exp_diff[4] ? noncoh_exp : max_exp;
	else if (exceed)
		noncoh_exp <= noncoh_exp + 1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		shift_coh <= 4'd0;
	else if (first_acc && ~first_acc_d)		// clear noncoh_exp on rising edge of first_acc
		shift_coh <= 4'd1;
	else if (first_coh && active_acc)
		shift_coh <= exp_diff[4] ? (~exp_diff[3:0] + 2) : 4'd1;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		shift_noncoh <= 4'd0;
	else if (first_acc && ~first_acc_d)		// clear noncoh_exp on rising edge of first_acc
		shift_noncoh <= max_exp;
	else if (first_coh && active_acc)
		shift_noncoh <= exp_diff[4] ? 4'd0 : exp_diff[3:0];

//----------------------------------------------------------
// latch coherent data and put into noncoh_sum
//----------------------------------------------------------
reg coh_rd_valid_d;
wire coh_data_valid;
reg [7:0] coh_data_valid_d;
assign coh_data_valid = coh_rd_valid_d & active_acc;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coh_rd_valid_d <= 1'b0;
	else
		coh_rd_valid_d <= coh_rd_valid;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coh_data_valid_d <= 7'h0;
	else
		coh_data_valid_d <= {coh_data_valid_d[6:0], coh_data_valid};

reg [191:0] coh_data;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coh_data <= 'd0;
	else if (coh_data_valid)
		coh_data <= coh_d4rd;

//----------------------------------------------------------
// noncoherent RAM access signals and read data
//----------------------------------------------------------
assign rd = coh_data_valid_d[0] & ~first_acc_d;	// two clock cycle delay to coherent read and skip first acc
reg [1:0] read_d;
reg [5:0] write_noncoh;
reg [6:0] output_noncoh;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		read_d <= 2'b00;
	else
		read_d <= {read_d[0], rd};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		write_noncoh <= 6'h0;
	else
		write_noncoh <= {write_noncoh[4:0], coh_data_valid & ~last_acc};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		output_noncoh <= 7'h0;
	else
		output_noncoh <= {output_noncoh[5:0], coh_data_valid & last_acc};

reg [9:0] coh_addr_d, read_addr, cor_count;
wire [9:0] write_addr;
assign write_addr = cor_count;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coh_addr_d <= 'd0;
	else
		coh_addr_d <= coh_read_addr;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		read_addr <= 'd0;
	else if (active_acc & ~first_acc)
		read_addr <= coh_addr_d;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cor_count <= 'd0;
	else if ((first_coh && active_acc))
		cor_count <= 'd0;
	else if (coh_data_valid_d[5])
		cor_count <= cor_count + 1;

assign we = write_noncoh[5];
assign addr = rd ? read_addr : write_addr;

reg [63:0] noncoh_read_data;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		noncoh_read_data <= 'd0;
	else if (first_acc)		// clear to 0 on first acc
		noncoh_read_data <= 'd0;
	else if (read_d[0])
		noncoh_read_data <= d4rd;

//----------------------------------------------------------
// instances of noncoherent sum
//----------------------------------------------------------
wire [3:0] exceed_flag;
wire [8:0] noncoh_out[3:0];
reg exceed1, extra_shift;
wire exceed2, extra_shift_new;
reg [9:0] exp_inc_cor, exp_inc_pos;

assign exceed2 = (|exceed_flag);
assign extra_shift_new = extra_shift | exceed;

wire [10:0] cor_count_cmp;
reg shift_noncoh_select;
reg [3:0] shift_noncoh_r;

assign cor_count_cmp = {1'b0, read_addr} - {1'b0, exp_inc_pos};

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		shift_noncoh_select <= 'd0;
	else if (rd)
		shift_noncoh_select <= cor_count_cmp[10];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		shift_noncoh_r <= 'd0;
	else
		shift_noncoh_r <= shift_noncoh_select ? shift_noncoh + 1 : shift_noncoh;

noncoh_sum u1_noncoh_sum
(
	.clk            (clk                 ),
	.rst_b          (rst_b               ),
	.coh_valid      (coh_data_valid_d[0] | coh_data_valid_d[1]),
	.coh_data       (coh_data_valid_d[0] ? coh_data[23:0] : coh_data[119:96]),
	.coh_shift      (shift_coh + max_exp),
	.noncoh_data    (read_d[1] ? noncoh_read_data[7:0] : noncoh_read_data[39:32]),
	.noncoh_shift   (shift_noncoh_r      ),
	.extra_shift    (extra_shift_new     ),
	.exceed         (exceed_flag[0]      ),
	.noncoh_out     (noncoh_out[0]       )
);

noncoh_sum u2_noncoh_sum
(
	.clk            (clk                 ),
	.rst_b          (rst_b               ),
	.coh_valid      (coh_data_valid_d[0] | coh_data_valid_d[1]),
	.coh_data       (coh_data_valid_d[0] ? coh_data[47:24] : coh_data[143:120]),
	.coh_shift      (shift_coh + max_exp),
	.noncoh_data    (read_d[1] ? noncoh_read_data[15:8] : noncoh_read_data[47:40]),
	.noncoh_shift   (shift_noncoh_r      ),
	.extra_shift    (extra_shift_new     ),
	.exceed         (exceed_flag[1]      ),
	.noncoh_out     (noncoh_out[1]       )
);

noncoh_sum u3_noncoh_sum
(
	.clk            (clk                 ),
	.rst_b          (rst_b               ),
	.coh_valid      (coh_data_valid_d[0] | coh_data_valid_d[1]),
	.coh_data       (coh_data_valid_d[0] ? coh_data[71:48] : coh_data[167:144]),
	.coh_shift      (shift_coh + max_exp),
	.noncoh_data    (read_d[1] ? noncoh_read_data[23:16] : noncoh_read_data[55:48]),
	.noncoh_shift   (shift_noncoh_r      ),
	.extra_shift    (extra_shift_new     ),
	.exceed         (exceed_flag[2]      ),
	.noncoh_out     (noncoh_out[2]       )
);

noncoh_sum u4_noncoh_sum
(
	.clk            (clk                 ),
	.rst_b          (rst_b               ),
	.coh_valid      (coh_data_valid_d[0] | coh_data_valid_d[1]),
	.coh_data       (coh_data_valid_d[0] ? coh_data[95:72] : coh_data[191:168]),
	.coh_shift      (shift_coh + max_exp),
	.noncoh_data    (read_d[1] ? noncoh_read_data[31:24] : noncoh_read_data[63:56]),
	.noncoh_shift   (shift_noncoh_r      ),
	.extra_shift    (extra_shift_new     ),
	.exceed         (exceed_flag[3]      ),
	.noncoh_out     (noncoh_out[3]       )
);

//----------------------------------------------------------
// signals control shift value
//----------------------------------------------------------
// latch exceed flag and extra_shift
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		exceed1 <= 1'b0;
	else if (coh_data_valid_d[2])
		exceed1 <= exceed2;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		exceed <= 1'b0;
	else if (coh_data_valid_d[3])
		exceed <= exceed1 | exceed2;
	else if (coh_data_valid_d[4])
		exceed <= 1'b0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		extra_shift <= 1'b0;
	else if (first_coh && active_acc)
		extra_shift <= 1'b0;
	else if (coh_data_valid_d[4])
		extra_shift <= extra_shift_new;

// latch exp_inc_cor and exp_inc_pos
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		exp_inc_cor <= 'd0;
	else if (first_coh && active_acc)
		exp_inc_cor <= 'd0;
	else if (exceed)
		exp_inc_cor <= cor_count;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		exp_inc_pos <= 'd0;
	else if (first_acc && ~first_acc_d)		// clear exp_inc_pos on rising edge of first_acc
		exp_inc_pos <= 'd0;
	else if (first_coh && active_acc)
		exp_inc_pos <= exp_inc_cor;

//----------------------------------------------------------
// noncoherent acc result output
//----------------------------------------------------------
// latch coherent sum result
reg [8:0] noncoh_out_r [7:0];

always @(posedge clk or negedge rst_b)
	if (!rst_b) begin
		noncoh_out_r[0] <= 9'd0;
		noncoh_out_r[1] <= 9'd0;
		noncoh_out_r[2] <= 9'd0;
		noncoh_out_r[3] <= 9'd0;
	end
	else if (coh_data_valid_d[3]) begin
		noncoh_out_r[0] <= noncoh_out[0];
		noncoh_out_r[1] <= noncoh_out[1];
		noncoh_out_r[2] <= noncoh_out[2];
		noncoh_out_r[3] <= noncoh_out[3];
	end

always @(posedge clk or negedge rst_b)
	if (!rst_b) begin
		noncoh_out_r[4] <= 9'd0;
		noncoh_out_r[5] <= 9'd0;
		noncoh_out_r[6] <= 9'd0;
		noncoh_out_r[7] <= 9'd0;
	end
	else if (coh_data_valid_d[4]) begin
		noncoh_out_r[4] <= noncoh_out[0];
		noncoh_out_r[5] <= noncoh_out[1];
		noncoh_out_r[6] <= noncoh_out[2];
		noncoh_out_r[7] <= noncoh_out[3];
	end

// determine whether shift one bit on output
reg output_shift;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		output_shift <= 1'b0;
	else// if (coh_data_valid_d[4])
		output_shift <= exceed;

wire [7:0] noncoh_write [7:0];
assign noncoh_write[0] = output_shift ? (noncoh_out_r[0][8:1] + noncoh_out_r[0][0]) : noncoh_out_r[0][7:0];
assign noncoh_write[1] = output_shift ? (noncoh_out_r[1][8:1] + noncoh_out_r[1][0]) : noncoh_out_r[1][7:0];
assign noncoh_write[2] = output_shift ? (noncoh_out_r[2][8:1] + noncoh_out_r[2][0]) : noncoh_out_r[2][7:0];
assign noncoh_write[3] = output_shift ? (noncoh_out_r[3][8:1] + noncoh_out_r[3][0]) : noncoh_out_r[3][7:0];
assign noncoh_write[4] = output_shift ? (noncoh_out_r[4][8:1] + noncoh_out_r[4][0]) : noncoh_out_r[4][7:0];
assign noncoh_write[5] = output_shift ? (noncoh_out_r[5][8:1] + noncoh_out_r[5][0]) : noncoh_out_r[5][7:0];
assign noncoh_write[6] = output_shift ? (noncoh_out_r[6][8:1] + noncoh_out_r[6][0]) : noncoh_out_r[6][7:0];
assign noncoh_write[7] = output_shift ? (noncoh_out_r[7][8:1] + noncoh_out_r[7][0]) : noncoh_out_r[7][7:0];

assign d4wt = {noncoh_write[7], noncoh_write[6], noncoh_write[5], noncoh_write[4], noncoh_write[3], noncoh_write[2], noncoh_write[1], noncoh_write[0]};

//----------------------------------------------------------
// calculate noise floor
//----------------------------------------------------------
reg [10:0] amp_sum_cor;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		amp_sum_cor <= 'd0;
	else if (output_noncoh[5])
		amp_sum_cor <= {3'b000, noncoh_write[0]} + {3'b000, noncoh_write[1]} + {3'b000, noncoh_write[2]} + {3'b000, noncoh_write[3]} + 
		               {3'b000, noncoh_write[4]} + {3'b000, noncoh_write[5]} + {3'b000, noncoh_write[6]} + {3'b000, noncoh_write[7]};

reg exceed_d, exceed_d2;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		exceed_d <= 1'b0;
	else
		exceed_d <= exceed;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		exceed_d2 <= 1'b0;
	else
		exceed_d2 <= exceed_d;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		noise_floor <= 'd0;
	else if (first_coh && active_acc)
		noise_floor <= 'd0;
	else if (exceed_d)	// right shift noise_floor one bit, this time slot is put between two output_noncoh[6] active slot
		noise_floor <= {1'b0, noise_floor[17:1]};
	else if (output_noncoh[6])
		noise_floor <= noise_floor + amp_sum_cor[10:3];

//----------------------------------------------------------
// output maximum amplitude and corresponding cor and freq
//----------------------------------------------------------
wire [8:0] max_amp;
wire [1:0] freq_index;

amp_compare u_amp_compare
(
	.amp1     (coh_data_valid_d[4] ? noncoh_out_r[0]: noncoh_out_r[4]),
	.amp2     (coh_data_valid_d[4] ? noncoh_out_r[1]: noncoh_out_r[5]),
	.amp3     (coh_data_valid_d[4] ? noncoh_out_r[2]: noncoh_out_r[6]),
	.amp4     (coh_data_valid_d[4] ? noncoh_out_r[3]: noncoh_out_r[7]),
	.max_amp  (max_amp),
	.index    (freq_index)
);

reg [8:0] max_amp_r;
reg [2:0] freq_index_r;
reg [9:0] cor_pos_r;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		max_amp_r <= 'd0;
	else if (coh_data_valid_d[4])
		max_amp_r <= max_amp;
	else if (coh_data_valid_d[5])
		max_amp_r <= (max_amp > max_amp_r) ? max_amp : max_amp_r;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		freq_index_r <= 'd0;
	else if (coh_data_valid_d[4])
		freq_index_r <= {1'b0, freq_index};
	else if (coh_data_valid_d[5])
		freq_index_r <= (max_amp > max_amp_r) ? {1'b1, freq_index} : freq_index_r;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cor_pos_r <= 'd0;
	else if (coh_data_valid_d[5])
		cor_pos_r <= cor_count;

assign noncoh_out_amp = exceed_d2 ? {max_amp_r[8:1] + max_amp_r[0]} : max_amp_r[7:0];
assign noncoh_out_exp = noncoh_exp;
assign noncoh_out_pos = cor_pos_r;
assign noncoh_out_freq = freq_index_r;
assign noncoh_out_valid = coh_data_valid_d[6];

endmodule
