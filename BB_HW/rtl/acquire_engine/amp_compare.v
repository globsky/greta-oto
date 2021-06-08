//----------------------------------------------------------------------
// amp_compare.v:
//   Compare four amplitude and output index of the largest
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

module amp_compare
(
input [8:0] amp1,
input [8:0] amp2,
input [8:0] amp3,
input [8:0] amp4,
output [8:0] max_amp,
input [1:0] index
);

wire [9:0] cmp1, cmp2;
assign cmp1 = (amp2 > amp1) ? {1'b1, amp2} : {1'b0, amp1};
assign cmp2 = (amp4 > amp3) ? {1'b1, amp4} : {1'b0, amp3};

assign max_amp = (cmp2[8:0] > cmp1[8:0]) ? cmp2[8:0] : cmp1[8:0];
assign index = (cmp2[8:0] > cmp1[8:0]) ? {1'b1, cmp2[9]} : {1'b0, cmp1[9]};

endmodule
