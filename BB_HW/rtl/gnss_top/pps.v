//----------------------------------------------------------------------
// pps.v:
//   PPS generation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

// PPS clock pps_clk and system clock clk can be asynchronize
module pps
(
// system signals
input clk,   // system clock
input pps_clk,   // PPS clock
input rst_b, // reset signal, low active

// host interface
input host_cs,
input host_rd,
input host_wr,
input [5:0] host_addr,	// DWORD address for 256B address space
input [31:0] host_d4wt,
output reg [31:0] host_d4rd,

// PPS input event and output event
input event_mark,
input cpu_latch,
output pps_pulse1,
output pps_pulse2,
output pps_pulse3,
output reg pps_event,	// PPS event to TE FIFO
output pps_irq	// edge trigger, pps_event masked by int_enable
);

//----------------------------------------------------------
// control registers write
//----------------------------------------------------------
// write flag toggle and address/data latch in system clock domain
reg write_flag;
reg [5:0] write_addr;
reg [31:0] write_data;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		write_flag <= 1'b0;
  else if (host_cs & host_wr)
		write_flag <= ~write_flag;

always @(posedge clk or negedge rst_b)
	if (!rst_b) begin
		write_addr <= 'd0;
		write_data <= 'd0;
	end
  else if (host_cs & host_wr) begin
		write_addr <= host_addr;
		write_data <= host_d4wt;
	end

// write flag toggle detection in PPS clock domain
reg write_flag_d1;
reg write_flag_d2;
wire write_flag_pps;
assign write_flag_pps = write_flag_d1 ^ write_flag_d2;

always @(posedge pps_clk or negedge rst_b)
	if (!rst_b) begin
		write_flag_d1 <= 1'b0;
		write_flag_d2 <= 1'b0;
	end
  else begin
		write_flag_d1 <= write_flag;
		write_flag_d2 <= write_flag_d1;
	end

// convert cpu_latch signal to toggle signal
reg latch_flag;
reg latch_flag_d1, latch_flag_d2;
wire cpu_latch_flag;
assign cpu_latch_flag = latch_flag_d1 ^ latch_flag_d2;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		latch_flag <= 1'b0;
  else if (cpu_latch)
		latch_flag <= ~latch_flag;

always @(posedge pps_clk or negedge rst_b)
	if (!rst_b) begin
		latch_flag_d1 <= 1'b0;
		latch_flag_d2 <= 1'b0;
	end
  else begin
		latch_flag_d1 <= latch_flag;
		latch_flag_d2 <= latch_flag_d1;
	end

//----------------------------------------------------------
// registers read/write
//----------------------------------------------------------
reg pps_enable, int_enable;
reg em_enable, em_polar;
reg [29:0] pulse_interval;
reg [29:0] pulse_adjust;
reg pulse1_enable, pulse1_polar;
reg [15:0] pulse1_delay;
reg [29:0] pulse1_width;
reg pulse2_enable, pulse2_polar;
reg [15:0] pulse2_delay;
reg [29:0] pulse2_width;
reg pulse3_enable, pulse3_polar;
reg [15:0] pulse3_delay;
reg [29:0] pulse3_width;
reg [30:0] clk_count_latch_cpu;
reg [7:0] pulse_count_latch_cpu;
reg [30:0] clk_count_latch_em;
reg [7:0] pulse_count_latch_em;

// write registers
always @(posedge pps_clk or negedge rst_b)
	if (!rst_b) begin
		pps_enable      <= 1'b0;
		int_enable      <= 1'b0;
		em_enable       <= 1'b0;
		em_polar        <= 1'b0;
		pulse_interval  <= 'd0;
		pulse1_enable   <= 1'b0;
		pulse1_polar    <= 1'b0;
		pulse1_delay    <= 'd0;
		pulse1_width    <= 'd0;
		pulse2_enable   <= 1'b0;
		pulse2_polar    <= 1'b0;
		pulse2_delay    <= 'd0;
		pulse2_width    <= 'd0;
		pulse3_enable   <= 1'b0;
		pulse3_polar    <= 1'b0;
		pulse3_delay    <= 'd0;
		pulse3_width    <= 'd0;
	end
	else if (write_flag_pps) begin
		case (write_addr)
			`PPS_CTRL           : {int_enable, pps_enable} <= write_data[1:0];
			`PPS_EM_CTRL        : {em_polar, em_enable} <= write_data[1:0];
			`PPS_PULSE_INTERVAL : pulse_interval <= write_data[29:0];
			`PPS_PULSE_CTRL1    : {pulse1_polar, pulse1_enable, pulse1_delay} <= write_data[17:0];
			`PPS_PULSE_WIDTH1   : pulse1_width   <= write_data[29:0];
			`PPS_PULSE_CTRL2    : {pulse2_polar, pulse2_enable, pulse2_delay} <= write_data[17:0];
			`PPS_PULSE_WIDTH2   : pulse2_width   <= write_data[29:0];
			`PPS_PULSE_CTRL3    : {pulse3_polar, pulse3_enable, pulse3_delay} <= write_data[17:0];
			`PPS_PULSE_WIDTH3   : pulse3_width   <= write_data[29:0];
		endcase
	end

//read registers
always @ (*) begin
	case (host_addr)
		`PPS_CTRL                  : host_d4rd = {30'h0, int_enable, pps_enable};
		`PPS_EM_CTRL               : host_d4rd = {30'h0, em_polar, em_enable};
		`PPS_PULSE_INTERVAL        : host_d4rd = {2'b00, pulse_interval};
		`PPS_PULSE_CTRL1           : host_d4rd = {14'h0, pulse1_polar, pulse1_enable, pulse1_delay};
		`PPS_PULSE_WIDTH1          : host_d4rd = {2'b00, pulse1_width};
		`PPS_PULSE_CTRL2           : host_d4rd = {14'h0, pulse2_polar, pulse2_enable, pulse2_delay};
		`PPS_PULSE_WIDTH2          : host_d4rd = {2'b00, pulse2_width};
		`PPS_PULSE_CTRL3           : host_d4rd = {14'h0, pulse3_polar, pulse3_enable, pulse3_delay};
		`PPS_PULSE_WIDTH3          : host_d4rd = {2'b00, pulse3_width};
		`PPS_CLK_COUNT_LATCH_CPU   : host_d4rd = {1'b0, clk_count_latch_cpu};
		`PPS_PULSE_COUNT_LATCH_CPU : host_d4rd = {24'h0, pulse_count_latch_cpu};
		`PPS_CLK_COUNT_LATCH_EM    : host_d4rd = {1'b0, clk_count_latch_em};
		`PPS_PULSE_COUNT_LATCH_EM  : host_d4rd = {24'h0, pulse_count_latch_em};
		default                    : host_d4rd = 32'h0;
	endcase
end

//----------------------------------------------------------
// clock counter and pulse counter
//----------------------------------------------------------
reg [30:0] clk_counter;
reg [7:0] pulse_counter;
wire [30:0] counter_next;
wire [30:0] interval_sum;
wire count_match;
assign counter_next = clk_counter + 1;
assign interval_sum = {1'b0, pulse_interval} + {pulse_adjust[29], pulse_adjust};
assign count_match = (counter_next == interval_sum);

always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		clk_counter <= 'd0;
	else if (count_match)
		clk_counter <= 'd0;
	else if (pps_enable)
		clk_counter <= counter_next;

always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		pulse_counter <= 'd0;
	else if (count_match)
		pulse_counter <= pulse_counter + 1;

always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		pulse_adjust <= 'd0;
	else if (count_match)
		pulse_adjust <= 'd0;
	else if (write_flag_pps && (write_addr == `PPS_PULSE_ADJUST))
		pulse_adjust <= write_data[29:0];

// generate edge of PPS event with enough hold time (32 PPS clock cycles)
reg [4:0] event_counter;

always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		pps_event <= 1'b0;
	else if (count_match)
		pps_event <= 1'b1;
	else if (event_counter == 5'h1f)
		pps_event <= 1'b0;

always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		event_counter <= 'd0;
	else if (pps_event)
		event_counter <= event_counter + 1;

assign pps_irq = pps_event & int_enable;

//----------------------------------------------------------
// pulse generation
//----------------------------------------------------------
reg pps_signal1, pps_signal2, pps_signal3;
reg signal_pending1, signal_pending2, signal_pending3;
reg [15:0] delay_counter1, delay_counter2, delay_counter3;
reg [29:0] width_counter1, width_counter2, width_counter3;

// first PPS signal
always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		pps_signal1 <= 1'b0;
	else if (count_match && (pulse1_delay == 0))	// in case of delay==0 set PPS immediately
		pps_signal1 <= 1'b1;
	else if (signal_pending1 && (delay_counter1 == pulse1_delay))	// otherwise wait period of delay
		pps_signal1 <= 1'b1;
	else if (width_counter1 == pulse1_width)	// clear after width time
		pps_signal1 <= 1'b0;

always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		signal_pending1 <= 1'b0;
	else if (count_match && (pulse1_delay != 0))	// set pending if delay!=0
		signal_pending1 <= 1'b1;
	else if (signal_pending1 && (delay_counter1 == pulse1_delay))	// clear pending after delay time
		signal_pending1 <= 1'b0;

always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		delay_counter1 <= 'd0;
	else if (signal_pending1 && (delay_counter1 == pulse1_delay))	// clear counter after delay time
		delay_counter1 <= 'd0;
	else if (signal_pending1)	// increase counter when signal pending
		delay_counter1 <= delay_counter1 + 1;

always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		width_counter1 <= 'd0;
	else if (width_counter1 == pulse1_width)	// clear counter when clear PPS
		width_counter1 <= 'd0;
	else if (pps_signal1)	// increase counter when PPS signal active
		width_counter1 <= width_counter1 + 1;

// second PPS signal
always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		pps_signal2 <= 1'b0;
	else if (count_match && (pulse2_delay == 0))	// in case of delay==0 set PPS immediately
		pps_signal2 <= 1'b1;
	else if (signal_pending2 && (delay_counter2 == pulse2_delay))	// otherwise wait period of delay
		pps_signal2 <= 1'b1;
	else if (width_counter2 == pulse2_width)	// clear after width time
		pps_signal2 <= 1'b0;

always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		signal_pending2 <= 1'b0;
	else if (count_match && (pulse2_delay != 0))	// set pending if delay!=0
		signal_pending2 <= 1'b1;
	else if (signal_pending2 && (delay_counter2 == pulse2_delay))	// clear pending after delay time
		signal_pending2 <= 1'b0;

always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		delay_counter2 <= 'd0;
	else if (signal_pending2 && (delay_counter2 == pulse2_delay))	// clear counter after delay time
		delay_counter2 <= 'd0;
	else if (signal_pending2)	// increase counter when signal pending
		delay_counter2 <= delay_counter2 + 1;

always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		width_counter2 <= 'd0;
	else if (width_counter2 == pulse2_width)	// clear counter when clear PPS
		width_counter2 <= 'd0;
	else if (pps_signal2)	// increase counter when PPS signal active
		width_counter2 <= width_counter2 + 1;

// first PPS signal
always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		pps_signal3 <= 1'b0;
	else if (count_match && (pulse3_delay == 0))	// in case of delay==0 set PPS immediately
		pps_signal3 <= 1'b1;
	else if (signal_pending3 && (delay_counter3 == pulse3_delay))	// otherwise wait period of delay
		pps_signal3 <= 1'b1;
	else if (width_counter3 == pulse3_width)	// clear after width time
		pps_signal3 <= 1'b0;

always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		signal_pending3 <= 1'b0;
	else if (count_match && (pulse3_delay != 0))	// set pending if delay!=0
		signal_pending3 <= 1'b1;
	else if (signal_pending3 && (delay_counter3 == pulse3_delay))	// clear pending after delay time
		signal_pending3 <= 1'b0;

always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		delay_counter3 <= 'd0;
	else if (signal_pending3 && (delay_counter3 == pulse3_delay))	// clear counter after delay time
		delay_counter3 <= 'd0;
	else if (signal_pending3)	// increase counter when signal pending
		delay_counter3 <= delay_counter3 + 1;

always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		width_counter3 <= 'd0;
	else if (width_counter3 == pulse3_width)	// clear counter when clear PPS
		width_counter3 <= 'd0;
	else if (pps_signal3)	// increase counter when PPS signal active
		width_counter3 <= width_counter3 + 1;

assign pps_pulse1 = (pps_signal1 & pulse1_enable) ^ pulse1_polar;
assign pps_pulse2 = (pps_signal2 & pulse2_enable) ^ pulse2_polar;
assign pps_pulse3 = (pps_signal3 & pulse3_enable) ^ pulse3_polar;

//----------------------------------------------------------
// latch counter
//----------------------------------------------------------
always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		clk_count_latch_cpu <= 'd0;
	else if (cpu_latch_flag)
		clk_count_latch_cpu <= clk_counter;

always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		pulse_count_latch_cpu <= 'd0;
	else if (cpu_latch_flag)
		pulse_count_latch_cpu <= pulse_counter;

wire em_signal;
reg em_signal_d;
wire em_latch_flag;
assign em_signal = event_mark ^ em_polar;
assign em_latch_flag = em_signal & (~em_signal_d);	// rising edge

always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		em_signal_d <= 'd0;
	else
		em_signal_d <= em_signal;

always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		clk_count_latch_em <= 'd0;
	else if (em_latch_flag)
		clk_count_latch_em <= clk_counter;

always @(posedge pps_clk or negedge rst_b)
	if (!rst_b)
		pulse_count_latch_em <= 'd0;
	else if (em_latch_flag)
		pulse_count_latch_em <= pulse_counter;

endmodule
