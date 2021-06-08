//----------------------------------------------------------------------
// WeilPrn.cpp:
//   Weil code PRN generator class implementation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include "CommonOps.h"
#include "WeilPrn.h"

const unsigned short CWeilPrn::LegendreL1C[640] = {
0x7bca, 0xa5d9, 0x8c32, 0xe787, 0xc5f0, 0x5e0d, 0xac3b, 0xc06b, 0xe033, 0xfb01, 0x76ed, 0x05b6, 0xd9a5, 0x4ede, 0xf451, 0x39db, 0xf904, 0x0b1e, 0xeb8f, 0x1107, 
0x7a2d, 0xf9a2, 0x0173, 0x9e6c, 0xf296, 0xc862, 0x74bd, 0xf2fc, 0xbf65, 0x3207, 0x1fc7, 0xf39f, 0xbfd6, 0x4421, 0x458e, 0x53b9, 0xfcda, 0xd4ea, 0x1746, 0x502b, 
0x3bdd, 0x59e2, 0xfad7, 0x8858, 0x5447, 0x2b4a, 0xc6fc, 0x68e5, 0xfb58, 0xc729, 0xa190, 0x291c, 0x3f61, 0xdbf2, 0xbf0c, 0xbfe5, 0x8baf, 0x2866, 0x4e08, 0x113e, 
0x52ee, 0xf06e, 0xab5e, 0x97ea, 0x9fab, 0xa279, 0x6130, 0x1c12, 0x6026, 0x80f9, 0x731e, 0x9f92, 0xbef0, 0xa289, 0xe761, 0xbc98, 0x137f, 0x2428, 0x7245, 0x58cb, 
0x4b8e, 0xb2f6, 0x2296, 0xed19, 0xabd9, 0xf77f, 0x81c4, 0x63d4, 0x6261, 0x313e, 0x49df, 0x35d9, 0xa16d, 0xfbe4, 0x3cc1, 0xfd36, 0xabdb, 0x6695, 0xf12b, 0x1d87, 
0x9842, 0x8245, 0x5997, 0x47e1, 0x0beb, 0x7852, 0xf7cf, 0xea0d, 0xdfeb, 0x51f4, 0xceab, 0xbc77, 0x819a, 0x9caa, 0x48c5, 0x7d7c, 0x21b9, 0x55d0, 0x1306, 0x1ebc, 
0x235c, 0xedfc, 0xff44, 0x78ec, 0x9cda, 0x22b9, 0xd36a, 0xb9d8, 0xc3ae, 0xc99a, 0xc818, 0x7e92, 0x7847, 0x4a55, 0x07e4, 0x4318, 0x6c50, 0x1879, 0xd011, 0xbad6, 
0x6f0a, 0x57a8, 0x83fa, 0x934c, 0x9ee8, 0xff54, 0xc849, 0x9587, 0xbd6b, 0x2c06, 0xcea0, 0xd780, 0x464a, 0x2fae, 0x1c64, 0x59c0, 0x2b49, 0x6437, 0x3695, 0xb1ca, 
0x64de, 0xd4a8, 0x8f59, 0xfe29, 0x1919, 0x8279, 0xadb7, 0x1797, 0xcd8e, 0xe783, 0xab2f, 0x2fba, 0xc016, 0xa571, 0x3c1b, 0xb364, 0x2849, 0x2946, 0x0e43, 0x4ba9, 
0x31c2, 0xf3ba, 0x1b63, 0xb6d2, 0xd852, 0x39e2, 0xaf9e, 0xbc70, 0x4bf0, 0xe153, 0xffb6, 0x0a29, 0x88cb, 0xb78e, 0x283c, 0xc236, 0xfa56, 0x5cce, 0x57a2, 0xd52f, 
0xc6c0, 0x255d, 0xc149, 0x6432, 0x3682, 0x826a, 0x256a, 0xad02, 0x519b, 0xfcde, 0x7ac4, 0x2648, 0xfb6a, 0xe4eb, 0xac88, 0x54f3, 0xe3ef, 0xf9da, 0x6603, 0xea30, 
0xf0a8, 0x8c9e, 0x9fb1, 0x6f7f, 0x9046, 0xc6c9, 0x93e5, 0xccc9, 0x65d4, 0xe576, 0x6ba7, 0x6fa5, 0x5943, 0x9ed3, 0x3772, 0xb211, 0x134f, 0x5578, 0x43f8, 0x8ae5, 
0x191a, 0x67e4, 0xa9b3, 0xeaf0, 0xfabb, 0x6131, 0x6e80, 0xe9a5, 0xc7f0, 0xf28d, 0x5c4d, 0xcad6, 0xa30e, 0x2cd8, 0xded6, 0xb6c5, 0xe45f, 0xc9ac, 0xa082, 0x8799, 
0xb5c0, 0x5780, 0x2ef9, 0xd709, 0x7a84, 0x603e, 0x20cc, 0x6762, 0x142f, 0xed31, 0x741a, 0x4790, 0x69a4, 0x2741, 0x0695, 0x6e86, 0xa700, 0x4752, 0x9f8d, 0xb66c, 
0x2cfb, 0x05d9, 0x627f, 0xc9d1, 0x944f, 0xbed9, 0xd64f, 0x71e1, 0x87ac, 0xac95, 0xfaaf, 0x3721, 0xb1c4, 0x6197, 0xd772, 0xc42e, 0x8ae7, 0x28cf, 0x0ce4, 0x057c, 
0xa1e8, 0xc854, 0xb27a, 0x8114, 0x213d, 0x218c, 0x18ee, 0x9ca9, 0x47f4, 0x3c35, 0x3697, 0xb445, 0x488a, 0x3493, 0x2874, 0x5e6e, 0x5b38, 0xc232, 0xdb43, 0xe4d8, 
0x3d24, 0xb3bc, 0xe325, 0x8985, 0xd1eb, 0x36d3, 0xaeed, 0x5dd2, 0x1693, 0x53c3, 0xd01d, 0x6ac6, 0x88e7, 0xce7b, 0x437b, 0xd77e, 0xa1b2, 0xd5ec, 0xe87a, 0xc15f, 
0xd8cf, 0x0ceb, 0x18ae, 0x8bdc, 0xb114, 0x1679, 0xdc72, 0x7b13, 0x0aa0, 0x56ca, 0xca1e, 0x7871, 0x0d94, 0x6482, 0x0dd6, 0x746c, 0x01b9, 0x645f, 0x20cb, 0xc992, 
0x4e06, 0xb51d, 0xff1a, 0x9e89, 0x569f, 0x7d1b, 0xda69, 0xf61d, 0xa7d1, 0x7348, 0x0bd7, 0xb919, 0xccfb, 0x83f9, 0xdea1, 0x6f14, 0x608b, 0xfe15, 0xfc52, 0x661e, 
0xbefa, 0xca6c, 0x05d8, 0x5c92, 0x9484, 0xe4cb, 0x8f3a, 0x94ac, 0x4dc5, 0x4eb0, 0xf01c, 0x5a68, 0xfe89, 0x7379, 0x22a0, 0xf0a8, 0x326a, 0xd819, 0xa767, 0x58ae, 
0xe03d, 0xe155, 0x0d37, 0x77b2, 0xb113, 0x3486, 0x3d65, 0x5a09, 0x1a29, 0x9158, 0xd459, 0x6ccc, 0x5836, 0x6c9c, 0x9df6, 0x0109, 0x7206, 0x86ce, 0xeaf0, 0xf3a8, 
0x3f99, 0xa460, 0x0838, 0x30d5, 0xeeca, 0x28d8, 0xa920, 0xed9b, 0xdca1, 0x84c0, 0x2675, 0xbf4a, 0xa95b, 0xa9be, 0xbe93, 0xb3d9, 0x6d7c, 0x455b, 0xfc9c, 0x0b54, 
0xba15, 0x8cc5, 0x95a0, 0x93bc, 0xc3eb, 0x8e12, 0x2cee, 0x6baf, 0x9200, 0x3578, 0xf02d, 0xf1c2, 0x860a, 0xb863, 0xb5e4, 0xb492, 0x3927, 0xa230, 0xbc73, 0x6a2d, 
0x3d8f, 0x9d6b, 0x6deb, 0xd932, 0x27c3, 0x715a, 0x97fc, 0xa20b, 0x0b2a, 0x3e18, 0x8e4c, 0x1617, 0x124a, 0x61be, 0x6767, 0x6b80, 0x650e, 0xead4, 0x84d9, 0xac72, 
0x5693, 0x13d9, 0x6d2b, 0xfc65, 0xd9c7, 0x8a0b, 0xad9d, 0xfe14, 0xfa8c, 0x9fcb, 0x2942, 0x1e56, 0x6dec, 0xd500, 0xe886, 0xcd36, 0xa03e, 0xea15, 0xaf09, 0x94a2, 
0x77f4, 0x61e7, 0xf5c9, 0xe73d, 0xd81f, 0x55ad, 0x1de1, 0xb681, 0xe7ec, 0xa66c, 0x8a3c, 0xe462, 0xa934, 0x62bb, 0xa4c6, 0xc8e1, 0xdd00, 0xc048, 0xc53b, 0xc287, 
0x9f37, 0xf455, 0x627b, 0xc141, 0x5ced, 0xaac6, 0xa67e, 0x11c2, 0x2a8c, 0xd075, 0x2804, 0x4fa8, 0x0c10, 0xb5e1, 0x282f, 0x781d, 0x1665, 0x5dbe, 0xbde6, 0x1e47, 
0x2b70, 0x5699, 0x242a, 0x9340, 0x7cc3, 0xd820, 0x497a, 0x6453, 0x046d, 0x8373, 0x79b9, 0xd439, 0xdc7e, 0x0110, 0x642a, 0x6748, 0x96bb, 0x90b2, 0x8e2d, 0x2ce5, 
0x5db1, 0xebdb, 0x0137, 0xe6c2, 0x7918, 0x6eba, 0xf082, 0xb606, 0x8731, 0x60fe, 0x9bf9, 0xb7c7, 0xf379, 0x61ba, 0x2a06, 0xa816, 0x852a, 0x89f0, 0x88b5, 0x8377, 
0xef8d, 0x99eb, 0x0a2e, 0x5802, 0xcf02, 0xb024, 0x7903, 0xc76b, 0xf67a, 0x6b1c, 0xe520, 0x58e9, 0xc09c, 0xad2b, 0x1dd5, 0xe5ee, 0x14a0, 0xb865, 0x4423, 0x2bf5, 
0x9d17, 0xa8d4, 0xa4c0, 0x6235, 0x8e5d, 0x7bdd, 0x9402, 0x0630, 0x1c07, 0x1fb3, 0x5902, 0xc0b0, 0x42d1, 0xb9ec, 0x96b0, 0xc986, 0x317f, 0xba60, 0x4ba1, 0x1f77, 
0x0e28, 0x872f, 0xdf60, 0x2463, 0x75d0, 0x848d, 0x5a64, 0x925f, 0x4891, 0x7f20, 0x33f8, 0x29fc, 0x23ca, 0x4f85, 0xf05c, 0x1e18, 0xb3ce, 0x645a, 0xac20, 0x0000, 
};

const unsigned short CWeilPrn::LegendreB1C[640] = {
0x4a77, 0xd4c5, 0x188b, 0x4a8c, 0xac6a, 0x7a60, 0x9a37, 0x3e5b, 0x634b, 0x8363, 0xd476, 0xd2fe, 0x7c66, 0xf585, 0xe547, 0xc931, 0x96e5, 0xdb35, 0x7ba5, 0x87b5, 
0x1c8b, 0x9496, 0x1ce3, 0x1153, 0xc45f, 0xd2c6, 0x45c8, 0x3ad9, 0x03c8, 0xcf90, 0x0e3c, 0xe4e8, 0x3c97, 0x429d, 0x1d60, 0xb4cc, 0x8024, 0x76c9, 0x7a84, 0x618c, 
0xe91f, 0x7a61, 0x6dca, 0x2986, 0xf80a, 0x47e5, 0xa9f9, 0x8da0, 0x5b8a, 0xdd45, 0x58a2, 0x0b97, 0xdad8, 0x1a7b, 0xa037, 0x583c, 0xfae1, 0x5e6a, 0x1a51, 0x3cfa, 
0xfa16, 0xf41f, 0x538a, 0x173e, 0xa41e, 0x38c4, 0x9ea6, 0x2d0c, 0xe959, 0x92ab, 0x308b, 0x0b0b, 0x6ffa, 0xf7ca, 0x9083, 0x0a2d, 0x9463, 0x6ece, 0xc2f8, 0x7a0b, 
0x473c, 0xf901, 0x9423, 0x93e8, 0x924d, 0x5b66, 0xe77d, 0x6ec6, 0x107f, 0xaf27, 0x9bd0, 0x46d8, 0x223d, 0x4168, 0x3e4d, 0x77bf, 0x9d20, 0x6f36, 0x1c48, 0x8fcd, 
0x8c3a, 0x22b2, 0xfa24, 0x3984, 0x4926, 0x5c3e, 0xac26, 0xd064, 0x32eb, 0xb4d1, 0xdc7e, 0xa50e, 0x5562, 0x43f9, 0xc903, 0xc676, 0xf866, 0xcce9, 0xb01f, 0x5026, 
0x0063, 0xb896, 0x10db, 0xe801, 0x8ce0, 0x6b32, 0xe8c0, 0xf002, 0x77cf, 0xb953, 0xf13f, 0x5a8e, 0x6d13, 0x77d6, 0xe75d, 0xaf4e, 0x0369, 0xdc29, 0x78e3, 0x6324, 
0xb5fb, 0x3b64, 0xab30, 0xef75, 0xc615, 0x5173, 0x5195, 0x0f26, 0x68af, 0x6eb0, 0xef37, 0xf619, 0x7dce, 0x96f1, 0x8353, 0x0a03, 0x0aa7, 0x552a, 0x9177, 0xfb31, 
0xdf84, 0xb50b, 0x517d, 0xfefc, 0x3cca, 0xa2b1, 0x7df1, 0x162f, 0x2cb3, 0x8f4c, 0xcd70, 0xc792, 0x47c0, 0xd10c, 0xd713, 0x4ec2, 0xa9aa, 0xd000, 0x7700, 0xb7d4, 
0x6c21, 0x1dff, 0x9a97, 0x0d3e, 0xa6b3, 0xa048, 0xcead, 0x937a, 0xf547, 0xdf0c, 0x9094, 0x3454, 0x794c, 0xe7fb, 0x9654, 0xb187, 0xed4a, 0xce6e, 0x7b10, 0x0e1c, 
0x6a5f, 0xb072, 0xa3f6, 0x74e7, 0x0463, 0xe38a, 0xa169, 0x7e8b, 0xdf38, 0xf7c2, 0xd91f, 0xb416, 0x674a, 0xf7c7, 0x59ba, 0xc7cb, 0xe1e6, 0x0265, 0x35de, 0x0cbc, 
0x590a, 0xd547, 0x729d, 0xaf56, 0xdd98, 0x93f7, 0xceb1, 0x4429, 0x1b38, 0xbfa1, 0x5f83, 0xc0c2, 0x056b, 0xd2c7, 0x4b4f, 0x5729, 0x31af, 0xe805, 0x9dfe, 0xe393, 
0xeafe, 0x83f0, 0x202e, 0x2d97, 0xe9fb, 0x4970, 0x533a, 0xeef9, 0x7f4a, 0x16af, 0x9264, 0xe1e6, 0x173f, 0x5feb, 0x50fa, 0xeeb7, 0xc0c4, 0x4a45, 0x206c, 0x98a4, 
0x10f8, 0xa554, 0xdd62, 0x6e47, 0xd31d, 0xeda4, 0x94d1, 0x19d2, 0x5290, 0xc809, 0x3705, 0x9b43, 0xbbf5, 0x8678, 0x0d4e, 0xb33c, 0x957f, 0x56f4, 0x86a1, 0xa2db, 
0x60cc, 0x5565, 0xb020, 0x87cf, 0x6220, 0xa0ae, 0x5311, 0x9499, 0x1b82, 0xf9d8, 0xcdac, 0x85e4, 0xd8f8, 0x2c8c, 0xfa00, 0xf7c2, 0xc32e, 0x3740, 0xd207, 0x64fa, 
0x5250, 0xe5d0, 0x1587, 0xb879, 0x8559, 0x0b46, 0x2c87, 0x54fd, 0x6ea5, 0x99a1, 0xbb32, 0xfaf5, 0xaa77, 0x73d5, 0xdcdd, 0xe626, 0x6db8, 0x84c5, 0x1060, 0xb5fc, 
0x5c05, 0x2f9f, 0x75cd, 0xee24, 0x99b9, 0x844c, 0x4543, 0x111a, 0xa50a, 0x0b32, 0x27a6, 0x65a8, 0x940d, 0x51ec, 0xb9d2, 0xf655, 0xe61e, 0x21e5, 0x7f45, 0x8f5b, 
0x5a0d, 0x91fb, 0x4fd1, 0x38b3, 0xcbc1, 0x0ffa, 0x0cec, 0xbe0e, 0x4d85, 0xeca4, 0xce46, 0x0be2, 0x766d, 0x6773, 0x58af, 0xafbb, 0x90c1, 0xefbf, 0x2595, 0x5ccf, 
0x924b, 0xa7a9, 0xed09, 0x5015, 0x6c33, 0x28d4, 0xfe19, 0xe502, 0x23d2, 0x65f1, 0x36fe, 0xcf6b, 0x5b46, 0x774d, 0x6da4, 0x8473, 0x41d8, 0x9b94, 0x4d55, 0xae0f, 
0x7dae, 0x6c9f, 0xb5da, 0xddcf, 0xc128, 0x8a0f, 0x5280, 0x5031, 0x7987, 0x8d9b, 0x60a9, 0x7ad0, 0x1608, 0x8a33, 0x5f16, 0xd206, 0x8164, 0xb8bf, 0xbf03, 0xe80a, 
0x8363, 0x8804, 0x65fe, 0x80a7, 0x36b1, 0x50d2, 0xd1cb, 0x4295, 0xfbcf, 0xc3e0, 0x57a0, 0x2e32, 0x76bd, 0xd728, 0xc103, 0x6e64, 0x4950, 0xa46b, 0x11d5, 0x4af6, 
0x5c2c, 0xf845, 0x359b, 0xf987, 0x82c1, 0xca26, 0x51c1, 0x0ad1, 0x997d, 0x2076, 0x4bc1, 0x0e30, 0x42e8, 0x1697, 0xaae3, 0x839d, 0xf18d, 0x1903, 0xab1f, 0x205a, 
0x9c78, 0xff72, 0x1898, 0xcad4, 0x81e7, 0x2d59, 0x6201, 0x8cd6, 0x1d5d, 0x3d6f, 0x6cf0, 0x41d5, 0x0a13, 0x64a8, 0xcedf, 0xa329, 0xa834, 0xf16a, 0x6004, 0x77bc, 
0x9d41, 0x2ff1, 0x1fff, 0x4aa6, 0xabc8, 0xd371, 0x4cf7, 0x4fc1, 0xdb61, 0xcf14, 0xccd0, 0xe32c, 0xb0b9, 0x7704, 0x172b, 0xaacc, 0x3c08, 0x0417, 0x52f5, 0x2de0, 
0x4732, 0x0117, 0x6ab5, 0x51aa, 0xf3fa, 0xf353, 0xe709, 0x68c4, 0x1679, 0x0130, 0x8f28, 0x90ae, 0x99b0, 0xf567, 0x5317, 0x5579, 0xc510, 0x8f32, 0xad92, 0x3205, 
0x2db3, 0x938e, 0x16bc, 0x4693, 0xf8d0, 0xa451, 0x8941, 0x1374, 0x98ea, 0x5037, 0x0356, 0x20c1, 0x1bff, 0x0fce, 0x8b32, 0x9f8c, 0xe7fe, 0x824f, 0x796e, 0x239f, 
0xf9bf, 0x507f, 0x268c, 0xc99e, 0x0919, 0xc3f6, 0xc603, 0xdb95, 0x58f5, 0xa81c, 0x474d, 0x228b, 0x3d9f, 0x49bc, 0xa83c, 0x59b6, 0xdde6, 0x3dba, 0x0b2b, 0xba3c, 
0xe4c0, 0xeedc, 0x7930, 0x9fb4, 0x6021, 0x14d8, 0x3e97, 0xd43b, 0xbe49, 0xdf42, 0x61b0, 0xa01f, 0x79c8, 0x9411, 0x8992, 0x54db, 0x6e83, 0x63bd, 0x67f6, 0x0c31, 
0xd2fa, 0x1e0b, 0xc8c8, 0x939d, 0x64ba, 0xf3ef, 0x6ac1, 0x0a00, 0x92f2, 0xf2ef, 0x32ab, 0x6656, 0x8cf4, 0xb9a8, 0x6dce, 0x387d, 0xa831, 0x7ae3, 0x507d, 0x097a, 
0x0a0c, 0x375a, 0x7a98, 0x578a, 0x0c3e, 0x513f, 0xa21a, 0x7e4a, 0x4162, 0xfbae, 0x55d4, 0x4ae2, 0x5fa4, 0xe606, 0xa581, 0xdafe, 0x09e6, 0xbac4, 0x979a, 0x1076, 
0x8ce7, 0x9dea, 0x16c9, 0x1dbf, 0xeccd, 0x2f94, 0x746b, 0xd16c, 0x3e8d, 0x8c38, 0xff60, 0xcec3, 0xf64a, 0x3ec5, 0xd9cb, 0x405d, 0xc357, 0x738c, 0x796d, 0x62ec, 
0x7521, 0xe5a2, 0x1532, 0x4589, 0x6736, 0xc1d5, 0x85e5, 0x099c, 0x180b, 0x491d, 0x4393, 0xe2d3, 0x9258, 0x313a, 0x6f9a, 0x1a9c, 0xacea, 0xd2ee, 0x75cd, 0x411a, 
};

CWeilPrn::CWeilPrn()
{
	Reset();
}

CWeilPrn::~CWeilPrn()
{
}

void CWeilPrn::Reset()
{
	LegendreCode1 = LegendreCode2 = 0;
	CurrentPhase = 0;
}

// Status buffer contents:
// address 00: bit0~13: Weil index, bit14~27: insertion index
// address 04: bit0~15: Legendre code 1, bit14~27: Legendre code 2
// address 08: bit0~13: Current count
void CWeilPrn::FillState(unsigned int *StateBuffer)
{
	WeilType = EXTRACT_UINT(StateBuffer[0], 29, 1);
	InsertionIndex = EXTRACT_UINT(StateBuffer[0], 14, 14);
	WeilIndex = EXTRACT_UINT(StateBuffer[0], 0, 14);
	LegendreCode1 = EXTRACT_UINT(StateBuffer[1], 0, 16);
	LegendreCode2 = EXTRACT_UINT(StateBuffer[1], 16, 16);
	CurrentPhase = EXTRACT_UINT(StateBuffer[2], 0, 14);

	InsertCode = ResetCounter();
}

void CWeilPrn::DumpState(unsigned int *StateBuffer)
{
	StateBuffer[1] = LegendreCode1 | (LegendreCode2 << 16);
	StateBuffer[2] = CurrentPhase & 0x3fff;
}

int CWeilPrn::GetCode()
{
	const int InsertBit[7] = { 0, 1, 1, 0, 1, 0, 0 };
	int BitIndex1 = CodeIndex1 & 0xf;
	int BitIndex2 = CodeIndex2 & 0xf;
	int LegendreBit1 = (LegendreCode1 & (1 << (15 - BitIndex1))) ? 1 : 0;
	int LegendreBit2 = (LegendreCode2 & (1 << (15 - BitIndex2))) ? 1 : 0;
	if (InsertCode)
		return InsertBit[InsertCount];
	else
		return (LegendreBit1 ^ LegendreBit2);
}

int CWeilPrn::ShiftCode()
{
	int NextInsertCode = 0;
	int InsertFlag = 0;
	int PhaseCmp0, PhaseCmp7;
	int NewRound = 0;

	CurrentPhase ++;
	PhaseCmp0 = (int)CurrentPhase - (int)InsertionIndex;
	PhaseCmp7 = PhaseCmp0 - 7;
	InsertFlag = ((PhaseCmp0 ^ PhaseCmp7) & 0x80000000) ? WeilType : 0;	// PhaseCmp0 and PhaseCmp7 have different sign

	if (InsertFlag)
		InsertFlag = 1;
	if (InsertCode)
		if (++InsertCount == 7)
			InsertCount = 0;
	if (!InsertCode)
	{
		if (++CodeIndex1 >= LEGENDRE_LENGTH(WeilType))
			CodeIndex1 = 0;
		if (++CodeIndex2 >= LEGENDRE_LENGTH(WeilType))
			CodeIndex2 = 0;
	}
	if (CurrentPhase == 10230)
	{
		CurrentPhase = 0;
		InsertFlag = ResetCounter();
		InsertCode = 0;
		NewRound = 1;
	}
	if (!InsertCode)
	{
		if ((CodeIndex1 & 0xf) == 0 || CurrentPhase == 0)
			LegendreCode1 = ((CodeIndex1 & 0x3ff0) == 0x2800) ? 0xc000 : (WeilType ? LegendreL1C[CodeIndex1 >> 4] : LegendreB1C[CodeIndex1 >> 4]);
		if ((CodeIndex2 & 0xf) == 0 || CurrentPhase == 0)
			LegendreCode2 = ((CodeIndex2 & 0x3ff0) == 0x2800) ? 0xc000 : (WeilType ? LegendreL1C[CodeIndex2 >> 4] : LegendreB1C[CodeIndex2 >> 4]);
	}
	InsertCode = InsertFlag;

	return NewRound;
}

void CWeilPrn::PhaseInit(unsigned int PrnConfig)
{
	WeilType = EXTRACT_UINT(PrnConfig, 29, 1);
	InsertionIndex = EXTRACT_UINT(PrnConfig, 14, 14);
	WeilIndex = EXTRACT_UINT(PrnConfig, 0, 14);
	CurrentPhase = 0;
	InsertCode = ResetCounter();
	LegendreCode1 = (WeilType ? LegendreL1C[CodeIndex1 >> 4] : LegendreB1C[CodeIndex1 >> 4]);
	LegendreCode2 = (WeilType ? LegendreL1C[CodeIndex2 >> 4] : LegendreB1C[CodeIndex2 >> 4]);
}

int CWeilPrn::ResetCounter()
{
	int InsertFlag = 0;

	if (WeilType == 0)	// B1C
	{
		InsertCount = InsertCode = 0;
		CodeIndex1 = CurrentPhase + InsertionIndex;
		if (CodeIndex1 >= B1C_LEGENDRE_LENGTH)
			CodeIndex1 -= B1C_LEGENDRE_LENGTH;
		CodeIndex2 = CodeIndex1 + WeilIndex;
		if (CodeIndex2 >= B1C_LEGENDRE_LENGTH)
			CodeIndex2 -= B1C_LEGENDRE_LENGTH;
	}
	else	// L1C
	{
		InsertCount = CurrentPhase - InsertionIndex;
		InsertFlag = (InsertCount < 0) ? 0 : (InsertCount >= 7) ? 0 : 1;
		InsertCount = (InsertCount < 0) ? 0 : (InsertCount > 7) ? 7 : InsertCount;
		CodeIndex1 = CurrentPhase - InsertCount;
		CodeIndex2 = CodeIndex1 + WeilIndex;
		if (CodeIndex2 >= L1C_LEGENDRE_LENGTH)
			CodeIndex2 -= L1C_LEGENDRE_LENGTH;
	}

	return InsertFlag;
}
