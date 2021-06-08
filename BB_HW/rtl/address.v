//----------------------------------------------------------------------
// address.v:
//   definition of register address offset
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

//global register
`define GLB_REG_ENA_ADDR          16'h0
`define GLB_BB_RESET              16'h1
`define GLB_FIFO_CLEAR            16'h2
`define GLB_TRACKING_START        16'h3
`define GLB_MEAS_NUMBER           16'h4
`define GLB_REG_MEAS_CNT_ADDR     16'h5
`define GLB_INTERRUPT_FLAG        16'h6
`define GLB_REQUEST_COUNT         16'h7
`define GLB_DATA_READY_MASK_ADDR  16'h8
//if
`define IF_BASE_ADDR              16'h400
`define IF_DATA_BITS               5'h0
`define IF_DATA_FORMAT             5'h1
//pp
`define PP_BASE_ADDR              16'h800
`define PP_NOISE_CTRL              5'hc
`define PP_NOISE_POWER             5'hd
//ae
`define AE_BASE_ADDR              16'h1000
`define AE_BUFFER_ADDR            16'h1100
`define AE_CONFIG                  8'h0
`define AE_CONTROL                 8'h1
`define AE_BUFFER_CONTROL          8'h2
`define AE_STATUS                  8'h3
`define AE_CARRIER_FREQ            8'h4
`define AE_CODE_RATIO              8'h5
`define AE_THRESHOLD               8'h6
//te fifo
`define TE_FIFO_BASE_ADDR      16'h1400
`define TE_FIFO_CONFIG             5'h0
`define TE_FIFO_STATUS             5'h1
`define TE_FIFO_SIZE               5'h3
`define TE_FIFO_GUARD              5'h4
`define TE_FIFO_READ_ADDR          5'h5
`define TE_FIFO_WRITE_ADDR         5'h6
`define TE_FIFO_BLOCK_SIZE         5'ha
`define TE_FIFO_BLOCK_ADJUST       5'hb
`define TE_FIFO_LWADDR_CPU         5'h10
`define TE_FIFO_LWADDR_EM          5'h11
`define TE_FIFO_LWADDR_PPS         5'h12
`define TE_FIFO_LWADDR_AE          5'h13
`define TE_FIFO_DEBUG_DAT          5'h14
//te group
`define TE_BASE_ADDR               16'h1800
`define TE_CHANNEL_ENABLE_ADDR     4'h0
`define TE_COH_DATA_READY_ADDR     4'h1
`define TE_OW_PROTECT_CHAN_ADDR    4'h2
`define TE_OW_PROTECT_ADDR_ADDR    4'h4
`define TE_OW_PROTECT_VALUE_ADDR   4'h5
`define TE_POLYNOMIAL_ADDR         4'h8
`define TE_CODE_LENGTH_ADDR        4'h9
`define TE_POLYNOMIAL2_ADDR        4'ha
`define TE_CODE_LENGTH2_ADDR       4'hb
`define TE_CURR_STATE_MACHINE      4'hf
//pps
`define  PPS_BASE                  16'h1c00
`define  PPS_CTRL_ADDR             5'h0
`define  PPS_PULSE_CTRL_ADDR       5'h1
`define  PPS_PULSE_WIDTH_ADDR      5'h2
`define  PPS_OVF_CNT_ADDR          5'h3
`define  PPS_OVF_NUM_ADDR          5'h4
`define  PPS_PULSE_CNT_ADDR        5'h5
`define  PPS_CNT_TH_ADDR           5'h6
`define  PPS_CNT_VALUE_ADDR        5'h7
`define  PPS_CNT_ADJ_ADDR          5'h8
`define  PPS_CNT_LATCH_CPU_ADDR    5'h9
`define  PPS_OVFCNT_LATCH_CPU_ADDR 5'ha
`define  PPS_PCNT_LATCH_CPU_ADDR   5'hb
`define  PPS_CNT_LATCH_EM_ADDR     5'hc
`define  PPS_OVFCNT_LATCH_EM_ADDR  5'hd
`define  PPS_PCNT_LATCH_EM_ADDR    5'he
`define  PPS_INT_FLAG_ADDR         5'hf
`define  EM_CTRL_ADDR              5'h10      
// te buffer
`define TE_BUFFER_ADDR             16'h4000
