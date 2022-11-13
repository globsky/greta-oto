//----------------------------------------------------------------------
// ae_top.v:
//   acquisition engine top level module
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module ae_top #(parameter AE_BUFFER_SIZE = 128*256)
(
// system signals				
input clk,	//system clock
input rst_b,	//reset signal, low active

// read/write interface
input ae_reg_cs,	  // AE register
input ae_buffer_cs,	// AE buffer
input ae_rd,  			// AE host read valid
input ae_wr,			  // AE host write valid
input [7:0] ae_addr,  // AE host access address
input [31:0] ae_d4wt,  // AE host write data
output [31:0] ae_rd_buffer,	// AE host read data from AE buffer, valid next cycle of ae_rd
output reg [31:0] ae_reg_d4rd,	  // AE host read data, valid same cycle of ae_rd

// interface to AE RAM
output [14:0] ae_ram_addr,
output [31:0] ae_ram_d4wt,
input [31:0] ae_ram_d4rd,
output ae_ram_ena,
output ae_ram_we,

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

// ADC data interface
input sample_valid,
input [7:0] sample_data,

// sync signal
output reg fill_start,
output ae_finish
);

reg [5:0] channel_num;
reg [6:0] buffer_threshold;

reg [23:0] code_rate_ratio;
reg [31:0] carrier_freq;
reg [7:0] threshold;

wire ae_finish_flag;
reg fill_enable;
wire ae_buffer_full;
wire ae_buffer_reach_th;
wire [3:0] ae_current_state;
wire [4:0] ae_current_channel;

wire [24:0] code_nco;
reg [14:0] ae_ram_wr_addr;
reg [15:0] input_sample_count;
reg [15:0] output_sample_count;

//----------------------------------------------------------
// registers value read/write
//----------------------------------------------------------
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		channel_num <= 6'h0;
	else if (ae_reg_cs & ae_wr & (ae_addr[7:0] == `AE_CONTROL))
		channel_num <= ae_d4wt[5:0];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		buffer_threshold <= 7'h0;
	else if (ae_reg_cs & ae_wr & (ae_addr[7:0] == `AE_BUFFER_CONTROL))
		buffer_threshold <= ae_d4wt[6:0];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		carrier_freq <= 32'h0;
	else if (ae_reg_cs & ae_wr & (ae_addr[7:0] == `AE_CARRIER_FREQ))
		carrier_freq <= ae_d4wt;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		code_rate_ratio <= 24'h0;
	else if (ae_reg_cs & ae_wr & (ae_addr[7:0] == `AE_CODE_RATIO))
		code_rate_ratio <= ae_d4wt[23:0];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		threshold <= 8'h0;
	else if (ae_reg_cs & ae_wr & (ae_addr[7:0] == `AE_THRESHOLD))
		threshold <= ae_d4wt[7:0];

//read registers
always @ (*) begin
	if (ae_reg_cs & ae_rd)
		case(ae_addr)
			`AE_CONTROL         : ae_reg_d4rd = {26'h0, channel_num};
			`AE_BUFFER_CONTROL  : ae_reg_d4rd = {25'h0, buffer_threshold};
			`AE_STATUS          : ae_reg_d4rd = {12'h0, ae_finish_flag, ae_buffer_full, ae_buffer_reach_th, fill_enable, 7'h0, ae_current_channel, ae_current_state};
			`AE_CARRIER_FREQ    : ae_reg_d4rd = carrier_freq;
			`AE_CODE_RATIO      : ae_reg_d4rd = {8'h0, code_rate_ratio};
			`AE_THRESHOLD       : ae_reg_d4rd = {24'h0, threshold};
			default             : ae_reg_d4rd = 32'h0;
		endcase
	else
		ae_reg_d4rd = 32'h0;
end

reg fill_pending;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		fill_pending <= 1'b0;
	else if (ae_reg_cs & ae_wr & (ae_addr[7:0] == `AE_BUFFER_CONTROL))
		fill_pending <= ae_d4wt[8];
	else if (fill_start)
		fill_pending <= 1'b0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		fill_start <= 1'b0;
	else if (sample_valid && fill_pending)
		fill_start <= 1'b1;
	else if (sample_valid && fill_start)
		fill_start <= 1'b0;

wire data_quant_valid;
wire [3:0] data_quant;
wire init_nco;
wire start_acquisition;

assign init_nco = (ae_reg_cs & ae_wr & (ae_addr[7:0] == `AE_BUFFER_CONTROL)) ? ae_d4wt[9] : 0;
assign start_acquisition = (ae_reg_cs & ae_wr & (ae_addr[7:0] == `AE_CONTROL)) ? ae_d4wt[8] : 0;

//----------------------------------------------------------
// rate adaptor
//----------------------------------------------------------
rate_adaptor u_rate_adaptor
(
		.clk               (clk                ),
		.rst_b             (rst_b              ),

		.sample_valid      (sample_valid & fill_enable),
		.sample_data       (sample_data        ),
		.data_valid        (data_quant_valid   ),
		.data_quant        (data_quant         ),

		.init_nco          (init_nco           ),
		.code_rate_ratio   (code_rate_ratio    ),
		.carrier_freq      (carrier_freq       ),
		.threshold         (threshold          )
);

//----------------------------------------------------------
// AE buffer read/write control
//----------------------------------------------------------
wire start_load_sample;
wire [17:0] buffer_read_address;
wire ae_buffer_read;
wire [3:0] ae_buffer_out;
wire sample_ready;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		fill_enable <= 1'b0;
	else if (fill_pending)
		fill_enable <= 1'b1;
	else if (ae_buffer_full)
		fill_enable <= 1'b0;

ae_buffer_rw #(.RAM_SIZE(AE_BUFFER_SIZE), .ADDR_WIDTH(15)) u_ae_buffer
(
		.clk              (clk                   ),
		.rst_b            (rst_b                 ),
		.sample_in        (data_quant            ),
		.sample_valid     (data_quant_valid & fill_enable),
		.refill           (fill_start            ),
		.write_full       (ae_buffer_full        ),
		.threshold        (buffer_threshold      ),
		.reach_threshold  (ae_buffer_reach_th    ),
		.address_set      (start_load_sample     ),
		.read_address     (buffer_read_address   ),
		.read_next        (ae_buffer_read        ),
		.sample_out       (ae_buffer_out         ),
		.sample_ready     (sample_ready          ),
		.en               (ae_ram_ena            ),
		.we               (ae_ram_we             ),
		.addr             (ae_ram_addr           ),
		.d4wt             (ae_ram_d4wt           ),
		.d4rd             (ae_ram_d4rd           )
);


//----------------------------------------------------------
// AE channel config buffer RAM
//----------------------------------------------------------
wire	config_buffer_rd;
wire	config_buffer_wt;
wire [7:0] config_buffer_addr;
wire [31:0] config_buffer_d4rd;
wire [31:0] config_buffer_d4wt;

//dpram_full #(.RAM_SIZE(256), .ADDR_WIDTH(8), .DATA_WIDTH(32))	ae_config_buffer
ae_config_buffer_256x32_wrapper ae_config_buffer
(
   	.clk_a      (clk                 ),
   	.addr_a     (config_buffer_addr  ),
   	.en_a       (config_buffer_rd    ),
   	.wr_a       (config_buffer_wt | config_buffer_wt),
   	.wdata_a    (config_buffer_d4wt  ),
   	.rdata_a    (config_buffer_d4rd  ),
   	.clk_b      (clk                 ),
   	.addr_b     (ae_addr             ),
   	.en_b       ((ae_rd | ae_wr) & ae_buffer_cs),
   	.wr_b       (ae_wr & ae_buffer_cs),
   	.wdata_b    (ae_d4wt             ),
   	.rdata_b    (ae_rd_buffer        )
);

//----------------------------------------------------------
// AE state control and core module
//----------------------------------------------------------
ae_core #(.BUFFER_ADDR_WIDTH(15), .ADDR_WIDTH(8)) u_ae_core
(
	.clk                 (clk                 ),
	.rst_b               (rst_b               ),

	.channel_number      (channel_num         ),
	.start_acquisition   (start_acquisition   ),

	.config_buffer_rd    (config_buffer_rd    ),
	.config_buffer_wt    (config_buffer_wt    ),
	.config_buffer_addr  (config_buffer_addr  ),
	.config_buffer_d4rd  (config_buffer_d4rd  ),
	.config_buffer_d4wt  (config_buffer_d4wt  ),
	
	.start_load_sample   (start_load_sample   ),
	.buffer_read_address (buffer_read_address ),
	.ae_buffer_read      (ae_buffer_read      ),
	.ae_buffer_out       (ae_buffer_out       ),
	.sample_ready        (sample_ready        ),
	
	.legendre_addr       (legendre_addr       ),
	.legendre_rd         (legendre_rd         ),
	.legendre_read_valid (legendre_read_valid ),
	.legendre_data       (legendre_data       ),

	.memcode_addr        (memcode_addr        ),
	.memcode_rd          (memcode_rd          ),
	.memcode_read_valid  (memcode_read_valid  ),
	.memcode_data        (memcode_data        ),

	.state_output        (ae_current_state    ),
	.channel_output      (ae_current_channel  ),
	.ae_finish_flag      (ae_finish_flag      )
);

reg ae_finish_flag_d;
reg ae_finish_flag_d2;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		ae_finish_flag_d <= 1'b0;
	else
		ae_finish_flag_d <= ae_finish_flag;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		ae_finish_flag_d2 <= 1'b0;
	else
		ae_finish_flag_d2 <= ae_finish_flag_d;

assign ae_finish = ~ae_finish_flag_d & ae_finish_flag;	// 1 cycles of high pulse

endmodule