//----------------------------------------------------------------------
// sync_data.v:
//   Synchronize input ADC data
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

// this module assumes system clock frequency is much higher than ADC clock frequency
module sync_data #(parameter DATA_WIDTH = 8)
(
// system signals
input clk,   // system clock
input rst_b, // reset signal, low active

// ADC clock and data
input adc_clk,
input [DATA_WIDTH-1:0] adc_data,

// synchronized ADC data output       
output sample_valid,
output [DATA_WIDTH-1:0]	sample_data
);

// the following registers are ensured not to be optimized away
(*syn_preserve = 1*) reg data_valid;
(*syn_preserve = 1*) reg clk_d0, clk_d1;
(*syn_preserve = 1*) reg [DATA_WIDTH-1:0] data_d0;
(*syn_preserve = 1*) reg [DATA_WIDTH-1:0] data_d1;
(*syn_preserve = 1*) reg [DATA_WIDTH-1:0] data_output;

// two stage of ADC clock and data latch
always @(posedge clk or negedge rst_b)
	if(!rst_b) begin
		clk_d0 <= 1'b0;
		clk_d1 <= 1'b0;
	end
  else begin
		clk_d0 <= adc_clk;
		clk_d1 <= clk_d0;
	end

always @(posedge clk or negedge rst_b)
	if(!rst_b) begin
		data_d0 <= 'd0;
		data_d1 <= 'd0;
	end
	else begin
		data_d0 <= adc_data;
		data_d1 <= data_d0;
	end

// assign synchronized data and valid flag on rising edge of ADC clock
wire clock_rise_edge;
assign clock_rise_edge = clk_d0 && ~clk_d1;

always @(posedge clk or negedge rst_b)
	if(!rst_b)
		data_valid <= 1'b0;
	else
		data_valid <= clock_rise_edge;

always @(posedge clk or negedge rst_b)
	if(!rst_b)
		data_output <= 'd0;
	else if (clock_rise_edge)
		data_output <= data_d1;

assign sample_valid = data_valid;
assign sample_data = data_output;

endmodule
