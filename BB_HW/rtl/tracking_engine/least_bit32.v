//----------------------------------------------------------------------
// least_bit32.v:
//   Find first 1 position from LSB
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module least_bit32
(
input [31:0] bits,	// input 8 bit
output [4:0] position,	// bit position of l
output active		// any bit active in input
);

wire active0, active1, active2, active3;
wire [2:0] position0, position1, position2, position3;
wire [2:0] position_msb;
reg [2:0] position_lsb;

least_bit8 u_least_bit8_0
(
	.bits (bits[7:0]),
	.position (position0),
	.active (active0)
);

least_bit8 u_least_bit8_1
(
	.bits (bits[15:8]),
	.position (position1),
	.active (active1)
);

least_bit8 u_least_bit8_2
(
	.bits (bits[23:16]),
	.position (position2),
	.active (active2)
);

least_bit8 u_least_bit8_3
(
	.bits (bits[31:24]),
	.position (position3),
	.active (active3)
);

least_bit8 u_least_bit8_top
(
	.bits ({4'b0000, active3, active2, active1, active0}),
	.position (position_msb),
	.active (active)
);

always @ (*) begin
	case (position_msb[1:0])
		2'b00:   position_lsb = position0;
		2'b01:   position_lsb = position1;
		2'b10:   position_lsb = position2;
		2'b11:   position_lsb = position3;
		default: position_lsb = position0;
	endcase
end

assign position = {position_msb[1:0], position_lsb};

endmodule
