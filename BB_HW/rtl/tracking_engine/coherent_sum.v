//----------------------------------------------------------------------
// coherent_sum.v:
//   Module accept 4 correlation result and do coherent sum
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module coherent_sum
(
// system signals
input clk,   // system clock
input rst_b, // reset signal, low active
// coherent FIFO interface
output [3:0] coh_fifo_rd,
input [3:0] coh_fifo_empty,
input [43:0] fifo_data0,
input [43:0] fifo_data1,
input [43:0] fifo_data2,
input [43:0] fifo_data3,
// correlator 0 sum result
output reg [31:0] coh_acc_data0,
output reg [31:0] coh_acc_data1,
output reg [31:0] coh_acc_data2,
output reg [31:0] coh_acc_data3,
// coherent RAM access interface
output reg coherent_rd,	// coherent buffer read signal
output reg coherent_wr,	// coherent buffer write signal
output [9:0] coherent_addr,		// coherent buffer address
output [31:0] coherent_d4wt,	// coherent buffer write data
input [31:0] coherent_d4rd,		// coherent buffer read data
output coherent_sum_done	// finish all coherent sum request
);

//------------------------------------------------------
//parameters 
//------------------------------------------------------
//states
localparam
		IDLE           = 3'h0,
		FIFO_SEL       = 3'h1,
		READ_FIFO      = 3'h2,
		READ_SUM_BUF   = 3'h3,
		DO_COH_SUM     = 3'h4;

//----------------------------------------------------------
// state machine
//----------------------------------------------------------
reg [2:0] cur_state;
reg [2:0] next_state;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cur_state <= IDLE;
	else
		cur_state <= next_state;
		
always @ (*) begin
 	case(cur_state)
		IDLE          : next_state = (&coh_fifo_empty) ? IDLE : FIFO_SEL;
		FIFO_SEL      : next_state = READ_FIFO;
		READ_FIFO     : next_state = READ_SUM_BUF;
		READ_SUM_BUF  : next_state = DO_COH_SUM;
		DO_COH_SUM    : next_state = (&coh_fifo_empty) ? IDLE : FIFO_SEL;
	endcase
end

//----------------------------------------------------------
// FIFO select, round robin
//----------------------------------------------------------
reg [3:0] cur_fifo_sel;
reg [3:0] next_fifo_sel;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		cur_fifo_sel <= 4'h0;
	else if (next_state == IDLE)	// reset selection on idle state
		cur_fifo_sel <= 4'h0;
	else if (next_state == FIFO_SEL)
		cur_fifo_sel <= next_fifo_sel;

always @(*) begin
	case (cur_fifo_sel)
		4'b0000:	// no current selection
			casez (coh_fifo_empty)
				4'b???0: next_fifo_sel = 4'b0001;
				4'b??01: next_fifo_sel = 4'b0010;
				4'b?011: next_fifo_sel = 4'b0100;
				4'b0111: next_fifo_sel = 4'b1000;
				default: next_fifo_sel = 4'b0000;
			endcase
		4'b0001:	// current select FIFO 0
			casez (coh_fifo_empty)
				4'b??0?: next_fifo_sel = 4'b0010;
				4'b?01?: next_fifo_sel = 4'b0100;
				4'b011?: next_fifo_sel = 4'b1000;
				4'b1110: next_fifo_sel = 4'b0001;
				default: next_fifo_sel = 4'b0000;
			endcase
		4'b0010:	// current select FIFO 1
			casez (coh_fifo_empty)
				4'b?0??: next_fifo_sel = 4'b0100;
				4'b01??: next_fifo_sel = 4'b1000;
				4'b11?0: next_fifo_sel = 4'b0001;
				4'b1101: next_fifo_sel = 4'b0010;
				default: next_fifo_sel = 4'b0000;
			endcase
		4'b0100:	// current select FIFO 2
			casez (coh_fifo_empty)
				4'b0???: next_fifo_sel = 4'b1000;
				4'b1??0: next_fifo_sel = 4'b0001;
				4'b1?01: next_fifo_sel = 4'b0010;
				4'b1011: next_fifo_sel = 4'b0100;
				default: next_fifo_sel = 4'b0000;
			endcase
		4'b1000:	// current select FIFO 3
			casez (coh_fifo_empty)
				4'b???0: next_fifo_sel = 4'b0001;
				4'b??01: next_fifo_sel = 4'b0010;
				4'b?011: next_fifo_sel = 4'b0100;
				4'b0111: next_fifo_sel = 4'b1000;
				default: next_fifo_sel = 4'b0000;
			endcase
	endcase
end

//----------------------------------------------------------
// read FIFO and select input data
//----------------------------------------------------------
assign coh_fifo_rd = (cur_state == READ_FIFO) ? cur_fifo_sel : 4'h0;

reg [43:0] fifo_data;
always @ (*) begin
	case(1'b1)
		cur_fifo_sel[0]: fifo_data = fifo_data0;
		cur_fifo_sel[1]: fifo_data = fifo_data1;
		cur_fifo_sel[2]: fifo_data = fifo_data2;
		cur_fifo_sel[3]: fifo_data = fifo_data3;
		default:         fifo_data = 'h0;
	endcase
end

//----------------------------------------------------------
// read, add and write back
//----------------------------------------------------------
// read coherent sum buffer address and read signal
reg [11:0] coh_sum_addr;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coh_sum_addr <= 10'h0;
	else if (coherent_rd)
		coh_sum_addr <= fifo_data[43:32];	// 10MSB as buffer address, then overwrite protect and first coherent data indicator

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coherent_rd <= 1'b0;
	else if (|coh_fifo_rd)
		coherent_rd <= 1'b1;
	else
		coherent_rd <= 1'b0;

assign coherent_addr = coherent_wr ? coh_sum_addr[11:2] : fifo_data[43:34];

// coherent sum
reg [31:0] coherent_sum_data;
wire [15:0] coherent_sum_i, coherent_sum_q;
assign coherent_sum_i = coherent_d4rd[31:16] + coherent_sum_data[31:16];
assign coherent_sum_q = coherent_d4rd[15:0] + coherent_sum_data[15:0];

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coherent_sum_data <= 32'h0;
	else if (coherent_rd)	// latch FIFO input
		coherent_sum_data <= fifo_data[31:0];
	else if (cur_state == DO_COH_SUM) begin
		if (coh_sum_addr[1])	// overwrite protect, no add operation, load coherent RAM directly
			coherent_sum_data <= coherent_d4rd;
		else if (~coh_sum_addr[0])	// not first coherent, load sum
			coherent_sum_data <= {coherent_sum_i, coherent_sum_q};
	end

// write coherent sum back
always @(posedge clk or negedge rst_b)
	if (!rst_b)
		coherent_wr <= 1'b0;
	else if (cur_state == DO_COH_SUM)
		coherent_wr <= 1'b1;
	else 
		coherent_wr <= 1'b0;

assign coherent_d4wt = coherent_sum_data;
assign coherent_sum_done = ((cur_state == IDLE) && (&coh_fifo_empty));

//----------------------------------------------------------
// latch correlator 0 sum result
//----------------------------------------------------------
wire is_cor0;
assign is_cor0 = (coh_sum_addr[4:2] == 3'b000);

// indicator of latch correlator 0
reg [3:0] latch_cor0;

always @(posedge clk or negedge rst_b)
	if (!rst_b)
		latch_cor0 <= 4'h0;
	else if (cur_state == DO_COH_SUM)
		latch_cor0 <= cur_fifo_sel & {4{is_cor0}};
	else 
		latch_cor0 <= 4'h0;

always @(posedge clk or negedge rst_b)
	if (~rst_b) begin
		coh_acc_data0 <= 'd0;
		coh_acc_data1 <= 'd0;
		coh_acc_data2 <= 'd0;
		coh_acc_data3 <= 'd0;
	end
	else begin
		case (1'b1)
			latch_cor0[0]: coh_acc_data0 <= coherent_sum_data;
			latch_cor0[1]: coh_acc_data1 <= coherent_sum_data;
			latch_cor0[2]: coh_acc_data2 <= coherent_sum_data;
			latch_cor0[3]: coh_acc_data3 <= coherent_sum_data;
		endcase
	end

endmodule
