//----------------------------------------------------------------------
// address.v:
//   definition of register address offset
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

`define MAJOR_VERSION             8'h1
`define MINOR_VERSION             8'h0
`define RELEASE_VERSION           16'h1628

// global registers
`define GLB_BASE_ADDR	            'h0000
`define GLB_BB_ENABLE             'h0
`define GLB_BB_RESET              'h1
`define GLB_FIFO_CLEAR            'h2
`define GLB_TRACKING_START        'h3
`define GLB_MEAS_NUMBER           'h4
`define GLB_MEAS_COUNT            'h5
`define GLB_INTERRUPT_FLAG        'h6
`define GLB_REQUEST_COUNT         'h7
`define GLB_INTERRUPT_MASK        'h8
`define GLB_BB_VERSION            'h10
// AE registers
`define AE_BASE_ADDR              'h1000
`define AE_CONFIG                 'h0
`define AE_CONTROL                'h1
`define AE_BUFFER_CONTROL         'h2
`define AE_STATUS                 'h3
`define AE_CARRIER_FREQ           'h4
`define AE_CODE_RATIO             'h5
`define AE_THRESHOLD              'h6
// TE FIFO registers
`define TE_FIFO_BASE_ADDR         'h1400
`define TE_FIFO_CONFIG            'h0
`define TE_FIFO_STATUS            'h1
`define TE_FIFO_GUARD             'h4
`define TE_FIFO_READ_ADDR         'h5
`define TE_FIFO_WRITE_ADDR        'h6
`define TE_FIFO_BLOCK_SIZE        'ha
`define TE_FIFO_BLOCK_ADJUST      'hb
`define TE_FIFO_LWADDR_CPU        'h10
`define TE_FIFO_LWADDR_EM         'h11
`define TE_FIFO_LWADDR_PPS        'h12
`define TE_FIFO_LWADDR_AE         'h13
// TE registers
`define TE_BASE_ADDR              'h1800
`define TE_CHANNEL_ENABLE         'h0
`define TE_COH_DATA_READY         'h1
`define TE_OW_PROTECT_CHANNEL     'h2
`define TE_OW_PROTECT_ADDR        'h4
`define TE_OW_PROTECT_VALUE       'h5
`define TE_POLYNOMIAL             'h8
`define TE_CODE_LENGTH            'h9
`define TE_POLYNOMIAL2            'ha
`define TE_CODE_LENGTH2           'hb
`define TE_CURR_STATE_MACHINE     'h10
// Peripheral registers
`define  PERIPHERAL_BASE          'h1c00
// te buffer
`define TE_BUFFER_ADDR            'h2000
`define AE_BUFFER_ADDR            'h3000
