//----------------------------------------------------------------------
// InitSet.cpp:
//   Array of initial values for PRN generator
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

unsigned int CAPrnInit[32] = {
	0x037ffff1,		// for PRN1
	0x01bffff1,		// for PRN2
	0x00dffff1,		// for PRN3
	0x006ffff1,		// for PRN4
	0x06903ff1,		// for PRN5
	0x03483ff1,		// for PRN6
	0x069bbff1,		// for PRN7
	0x034dfff1,		// for PRN8
	0x01a6fff1,		// for PRN9
	0x02eefff1,		// for PRN10
	0x01777ff1,		// for PRN11
	0x005dfff1,		// for PRN12
	0x002efff1,		// for PRN13
	0x00177ff1,		// for PRN14
	0x000bbff1,		// for PRN15
	0x0005fff1,		// for PRN16
	0x06447ff1,		// for PRN17
	0x03223ff1,		// for PRN18
	0x01913ff1,		// for PRN19
	0x00c8bff1,		// for PRN20
	0x00647ff1,		// for PRN21
	0x00323ff1,		// for PRN22
	0x07333ff1,		// for PRN23
	0x00e67ff1,		// for PRN24
	0x00733ff1,		// for PRN25
	0x0039bff1,		// for PRN26
	0x001cfff1,		// for PRN27
	0x000e7ff1,		// for PRN28
	0x06a23ff1,		// for PRN29
	0x03513ff1,		// for PRN30
	0x01a8bff1,		// for PRN31
	0x00d47ff1,		// for PRN32
};

unsigned int WaasPrnInit[19] = {
	0x091a7ff1,		// for PRN120
	0x0a863ff1,		// for PRN121
	0x02dcfff1,		// for PRN122
	0x02693ff1,		// for PRN123
	0x0e3e3ff1,		// for PRN124
	0x08f87ff1,		// for PRN125
	0x0fd27ff1,		// for PRN126
	0x073d7ff1,		// for PRN127
	0x0d6afff1,		// for PRN128
	0x0aa37ff1,		// for PRN129
	0x03857ff1,		// for PRN130
	0x05a57ff1,		// for PRN131
	0x05433ff1,		// for PRN132
	0x0f67bff1,		// for PRN133
	0x07183ff1,		// for PRN134
	0x0a387ff1,		// for PRN135
	0x07833ff1,		// for PRN136
	0x081e3ff1,		// for PRN137
	0x04a13ff1,		// for PRN138
};

unsigned int L5IInit[37] = {
	0x02753ffe,		// for PRN1
	0x0ac1fffe,		// for PRN2
	0x01013ffe,		// for PRN3
	0x0646fffe,		// for PRN4
	0x0ebbbffe,		// for PRN5
	0x05f37ffe,		// for PRN6
	0x0f92fffe,		// for PRN7
	0x025efffe,		// for PRN8
	0x0d4fbffe,		// for PRN9
	0x07bf7ffe,		// for PRN10
	0x05c83ffe,		// for PRN11
	0x09f3bffe,		// for PRN12
	0x039c3ffe,		// for PRN13
	0x0e417ffe,		// for PRN14
	0x05ab7ffe,		// for PRN15
	0x093c3ffe,		// for PRN16
	0x0f197ffe,		// for PRN17
	0x0787bffe,		// for PRN18
	0x0f89fffe,		// for PRN19
	0x0b6b3ffe,		// for PRN20
	0x01027ffe,		// for PRN21
	0x0f7bfffe,		// for PRN22
	0x07f0fffe,		// for PRN23
	0x02d1fffe,		// for PRN24
	0x0b65fffe,		// for PRN25
	0x069abffe,		// for PRN26
	0x07b53ffe,		// for PRN27
	0x06af3ffe,		// for PRN28
	0x087d7ffe,		// for PRN29
	0x0ed0fffe,		// for PRN30
	0x07947ffe,		// for PRN31
	0x09d07ffe,		// for PRN32
	0x0815bffe,		// for PRN33
	0x09fdbffe,		// for PRN34
	0x03b7fffe,		// for PRN35
	0x0134bffe,		// for PRN36
	0x00967ffe,		// for PRN37
};

unsigned int L5QInit[37] = {
	0x0334bffe,		// for PRN1
	0x06f13ffe,		// for PRN2
	0x0c47bffe,		// for PRN3
	0x056e7ffe,		// for PRN4
	0x04de7ffe,		// for PRN5
	0x09553ffe,		// for PRN6
	0x081ffffe,		// for PRN7
	0x016b7ffe,		// for PRN8
	0x0c2ebffe,		// for PRN9
	0x06127ffe,		// for PRN10
	0x0a043ffe,		// for PRN11
	0x0a353ffe,		// for PRN12
	0x0a597ffe,		// for PRN13
	0x0fc2fffe,		// for PRN14
	0x0f1ebffe,		// for PRN15
	0x0fa5fffe,		// for PRN16
	0x0133bffe,		// for PRN17
	0x0276bffe,		// for PRN18
	0x0da67ffe,		// for PRN19
	0x08e1bffe,		// for PRN20
	0x009b3ffe,		// for PRN21
	0x071a7ffe,		// for PRN22
	0x0be8bffe,		// for PRN23
	0x0cfb3ffe,		// for PRN24
	0x0d917ffe,		// for PRN25
	0x03d57ffe,		// for PRN26
	0x05f0fffe,		// for PRN27
	0x042ffffe,		// for PRN28
	0x02453ffe,		// for PRN29
	0x09e0bffe,		// for PRN30
	0x0a7d7ffe,		// for PRN31
	0x0544bffe,		// for PRN32
	0x0226bffe,		// for PRN33
	0x0227bffe,		// for PRN34
	0x0cd37ffe,		// for PRN35
	0x0f5e3ffe,		// for PRN36
	0x08b23ffe,		// for PRN37
};

unsigned int E5aIInit[50] = {
	0xc317fff,		// for PRN1
	0x6273fff,		// for PRN2
	0xba2ffff,		// for PRN3
	0x85fffff,		// for PRN4
	0x9b2bfff,		// for PRN5
	0xdccffff,		// for PRN6
	0x6e33fff,		// for PRN7
	0x557ffff,		// for PRN8
	0x0d5ffff,		// for PRN9
	0xc27bfff,		// for PRN10
	0xbb93fff,		// for PRN11
	0x3aebfff,		// for PRN12
	0xf3fffff,		// for PRN13
	0x789bfff,		// for PRN14
	0x3473fff,		// for PRN15
	0x6c17fff,		// for PRN16
	0xa2abfff,		// for PRN17
	0x4e67fff,		// for PRN18
	0xa7fbfff,		// for PRN19
	0x0663fff,		// for PRN20
	0x4dc3fff,		// for PRN21
	0x7aebfff,		// for PRN22
	0xbc97fff,		// for PRN23
	0xcf0bfff,		// for PRN24
	0x582bfff,		// for PRN25
	0x6407fff,		// for PRN26
	0xe75ffff,		// for PRN27
	0x965ffff,		// for PRN28
	0xc64ffff,		// for PRN29
	0xbabbfff,		// for PRN30
	0x0d43fff,		// for PRN31
	0x6227fff,		// for PRN32
	0xccd7fff,		// for PRN33
	0x91d3fff,		// for PRN34
	0xdd3bfff,		// for PRN35
	0x177ffff,		// for PRN36
	0x8b3bfff,		// for PRN37
	0xec57fff,		// for PRN38
	0xee6ffff,		// for PRN39
	0xa6b7fff,		// for PRN40
	0x60b3fff,		// for PRN41
	0xb85ffff,		// for PRN42
	0x3613fff,		// for PRN43
	0xccb7fff,		// for PRN44
	0xe4d7fff,		// for PRN45
	0xaaeffff,		// for PRN46
	0x87cffff,		// for PRN47
	0xcf47fff,		// for PRN48
	0x7b2bfff,		// for PRN49
	0x5afffff,		// for PRN50
};

unsigned int E5aQInit[50] = {
	0xaeabfff,		// for PRN1 
	0x298bfff,		// for PRN2 
	0xa74ffff,		// for PRN3 
	0xcfa7fff,		// for PRN4 
	0xbbdbfff,		// for PRN5 
	0xa6c3fff,		// for PRN6	
	0xdeb7fff,		// for PRN7 
	0xbca3fff,		// for PRN8 
	0x3e5bfff,		// for PRN9 
	0x0f17fff,		// for PRN10
	0x573ffff,		// for PRN11
	0xd14bfff,		// for PRN12
	0x70f7fff,		// for PRN13
	0x7693fff,		// for PRN14
	0xfdbbfff,		// for PRN15
	0x14fffff,		// for PRN16
	0x12d7fff,		// for PRN17
	0x3463fff,		// for PRN18
	0xa89bfff,		// for PRN19
	0x5777fff,		// for PRN20
	0x22cbfff,		// for PRN21
	0x4a63fff,		// for PRN22
	0x007ffff,		// for PRN23
	0x317ffff,		// for PRN24
	0x232bfff,		// for PRN25
	0x861bfff,		// for PRN26
	0x49cbfff,		// for PRN27
	0x92abfff,		// for PRN28
	0xc56ffff,		// for PRN29
	0xa633fff,		// for PRN30
	0x3fdffff,		// for PRN31
	0xd717fff,		// for PRN32
	0x28abfff,		// for PRN33
	0xbdaffff,		// for PRN34
	0x1f27fff,		// for PRN35
	0x1087fff,		// for PRN36
	0xe7f7fff,		// for PRN37
	0x2af3fff,		// for PRN38
	0xfbbbfff,		// for PRN39
	0x7217fff,		// for PRN40
	0xf2e3fff,		// for PRN41
	0x3603fff,		// for PRN42
	0xb7effff,		// for PRN43
	0x7bf7fff,		// for PRN44
	0xeadffff,		// for PRN45
	0xf2b7fff,		// for PRN46
	0x5093fff,		// for PRN47
	0xb48bfff,		// for PRN48
	0x8e47fff,		// for PRN49
	0xac27fff,		// for PRN50
};

unsigned int B1CDataInit[63] = {
 2678 + ((  699 - 1) << 14) + 0x80000000,	// for PRN01
 4802 + ((  694 - 1) << 14) + 0x80000000,	// for PRN02
  958 + (( 7318 - 1) << 14) + 0x80000000,	// for PRN03
  859 + (( 2127 - 1) << 14) + 0x80000000,	// for PRN04
 3843 + ((  715 - 1) << 14) + 0x80000000,	// for PRN05
 2232 + (( 6682 - 1) << 14) + 0x80000000,	// for PRN06
  124 + (( 7850 - 1) << 14) + 0x80000000,	// for PRN07
 4352 + (( 5495 - 1) << 14) + 0x80000000,	// for PRN08
 1816 + (( 1162 - 1) << 14) + 0x80000000,	// for PRN09
 1126 + (( 7682 - 1) << 14) + 0x80000000,	// for PRN10
 1860 + (( 6792 - 1) << 14) + 0x80000000,	// for PRN11
 4800 + (( 9973 - 1) << 14) + 0x80000000,	// for PRN12
 2267 + (( 6596 - 1) << 14) + 0x80000000,	// for PRN13
  424 + (( 2092 - 1) << 14) + 0x80000000,	// for PRN14
 4192 + ((   19 - 1) << 14) + 0x80000000,	// for PRN15
 4333 + ((10151 - 1) << 14) + 0x80000000,	// for PRN16
 2656 + (( 6297 - 1) << 14) + 0x80000000,	// for PRN17
 4148 + (( 5766 - 1) << 14) + 0x80000000,	// for PRN18
  243 + (( 2359 - 1) << 14) + 0x80000000,	// for PRN19
 1330 + (( 7136 - 1) << 14) + 0x80000000,	// for PRN20
 1593 + (( 1706 - 1) << 14) + 0x80000000,	// for PRN21
 1470 + (( 2128 - 1) << 14) + 0x80000000,	// for PRN22
  882 + (( 6827 - 1) << 14) + 0x80000000,	// for PRN23
 3202 + ((  693 - 1) << 14) + 0x80000000,	// for PRN24
 5095 + (( 9729 - 1) << 14) + 0x80000000,	// for PRN25
 2546 + (( 1620 - 1) << 14) + 0x80000000,	// for PRN26
 1733 + (( 6805 - 1) << 14) + 0x80000000,	// for PRN27
 4795 + ((  534 - 1) << 14) + 0x80000000,	// for PRN28
 4577 + ((  712 - 1) << 14) + 0x80000000,	// for PRN29
 1627 + (( 1929 - 1) << 14) + 0x80000000,	// for PRN30
 3638 + (( 5355 - 1) << 14) + 0x80000000,	// for PRN31
 2553 + (( 6139 - 1) << 14) + 0x80000000,	// for PRN32
 3646 + (( 6339 - 1) << 14) + 0x80000000,	// for PRN33
 1087 + (( 1470 - 1) << 14) + 0x80000000,	// for PRN34
 1843 + (( 6867 - 1) << 14) + 0x80000000,	// for PRN35
  216 + (( 7851 - 1) << 14) + 0x80000000,	// for PRN36
 2245 + (( 1162 - 1) << 14) + 0x80000000,	// for PRN37
  726 + (( 7659 - 1) << 14) + 0x80000000,	// for PRN38
 1966 + (( 1156 - 1) << 14) + 0x80000000,	// for PRN39
  670 + (( 2672 - 1) << 14) + 0x80000000,	// for PRN40
 4130 + (( 6043 - 1) << 14) + 0x80000000,	// for PRN41
   53 + (( 2862 - 1) << 14) + 0x80000000,	// for PRN42
 4830 + ((  180 - 1) << 14) + 0x80000000,	// for PRN43
  182 + (( 2663 - 1) << 14) + 0x80000000,	// for PRN44
 2181 + (( 6940 - 1) << 14) + 0x80000000,	// for PRN45
 2006 + (( 1645 - 1) << 14) + 0x80000000,	// for PRN46
 1080 + (( 1582 - 1) << 14) + 0x80000000,	// for PRN47
 2288 + ((  951 - 1) << 14) + 0x80000000,	// for PRN48
 2027 + (( 6878 - 1) << 14) + 0x80000000,	// for PRN49
  271 + (( 7701 - 1) << 14) + 0x80000000,	// for PRN50
  915 + (( 1823 - 1) << 14) + 0x80000000,	// for PRN51
  497 + (( 2391 - 1) << 14) + 0x80000000,	// for PRN52
  139 + (( 2606 - 1) << 14) + 0x80000000,	// for PRN53
 3693 + ((  822 - 1) << 14) + 0x80000000,	// for PRN54
 2054 + (( 6403 - 1) << 14) + 0x80000000,	// for PRN55
 4342 + ((  239 - 1) << 14) + 0x80000000,	// for PRN56
 3342 + ((  442 - 1) << 14) + 0x80000000,	// for PRN57
 2592 + (( 6769 - 1) << 14) + 0x80000000,	// for PRN58
 1007 + (( 2560 - 1) << 14) + 0x80000000,	// for PRN59
  310 + (( 2502 - 1) << 14) + 0x80000000,	// for PRN60
 4203 + (( 5072 - 1) << 14) + 0x80000000,	// for PRN61
  455 + (( 7268 - 1) << 14) + 0x80000000,	// for PRN62
 4318 + ((  341 - 1) << 14) + 0x80000000,	// for PRN63
};

unsigned int B1CPilotInit[63] = {
  796 + (( 7575 - 1) << 14) + 0x80000000,	// for PRN01
  156 + (( 2369 - 1) << 14) + 0x80000000,	// for PRN02
 4198 + (( 5688 - 1) << 14) + 0x80000000,	// for PRN03
 3941 + ((  539 - 1) << 14) + 0x80000000,	// for PRN04
 1374 + (( 2270 - 1) << 14) + 0x80000000,	// for PRN05
 1338 + (( 7306 - 1) << 14) + 0x80000000,	// for PRN06
 1833 + (( 6457 - 1) << 14) + 0x80000000,	// for PRN07
 2521 + (( 6254 - 1) << 14) + 0x80000000,	// for PRN08
 3175 + (( 5644 - 1) << 14) + 0x80000000,	// for PRN09
  168 + (( 7119 - 1) << 14) + 0x80000000,	// for PRN10
 2715 + (( 1402 - 1) << 14) + 0x80000000,	// for PRN11
 4408 + (( 5557 - 1) << 14) + 0x80000000,	// for PRN12
 3160 + (( 5764 - 1) << 14) + 0x80000000,	// for PRN13
 2796 + (( 1073 - 1) << 14) + 0x80000000,	// for PRN14
  459 + (( 7001 - 1) << 14) + 0x80000000,	// for PRN15
 3594 + (( 5910 - 1) << 14) + 0x80000000,	// for PRN16
 4813 + ((10060 - 1) << 14) + 0x80000000,	// for PRN17
  586 + (( 2710 - 1) << 14) + 0x80000000,	// for PRN18
 1428 + (( 1546 - 1) << 14) + 0x80000000,	// for PRN19
 2371 + (( 6887 - 1) << 14) + 0x80000000,	// for PRN20
 2285 + (( 1883 - 1) << 14) + 0x80000000,	// for PRN21
 3377 + (( 5613 - 1) << 14) + 0x80000000,	// for PRN22
 4965 + (( 5062 - 1) << 14) + 0x80000000,	// for PRN23
 3779 + (( 1038 - 1) << 14) + 0x80000000,	// for PRN24
 4547 + ((10170 - 1) << 14) + 0x80000000,	// for PRN25
 1646 + (( 6484 - 1) << 14) + 0x80000000,	// for PRN26
 1430 + (( 1718 - 1) << 14) + 0x80000000,	// for PRN27
  607 + (( 2535 - 1) << 14) + 0x80000000,	// for PRN28
 2118 + (( 1158 - 1) << 14) + 0x80000000,	// for PRN29
 4709 + (( 526  - 1) << 14) + 0x80000000,	// for PRN30
 1149 + (( 7331 - 1) << 14) + 0x80000000,	// for PRN31
 3283 + (( 5844 - 1) << 14) + 0x80000000,	// for PRN32
 2473 + (( 6423 - 1) << 14) + 0x80000000,	// for PRN33
 1006 + (( 6968 - 1) << 14) + 0x80000000,	// for PRN34
 3670 + (( 1280 - 1) << 14) + 0x80000000,	// for PRN35
 1817 + (( 1838 - 1) << 14) + 0x80000000,	// for PRN36
  771 + (( 1989 - 1) << 14) + 0x80000000,	// for PRN37
 2173 + (( 6468 - 1) << 14) + 0x80000000,	// for PRN38
  740 + (( 2091 - 1) << 14) + 0x80000000,	// for PRN39
 1433 + (( 1581 - 1) << 14) + 0x80000000,	// for PRN40
 2458 + (( 1453 - 1) << 14) + 0x80000000,	// for PRN41
 3459 + (( 6252 - 1) << 14) + 0x80000000,	// for PRN42
 2155 + (( 7122 - 1) << 14) + 0x80000000,	// for PRN43
 1205 + (( 7711 - 1) << 14) + 0x80000000,	// for PRN44
  413 + (( 7216 - 1) << 14) + 0x80000000,	// for PRN45
  874 + (( 2113 - 1) << 14) + 0x80000000,	// for PRN46
 2463 + (( 1095 - 1) << 14) + 0x80000000,	// for PRN47
 1106 + (( 1628 - 1) << 14) + 0x80000000,	// for PRN48
 1590 + (( 1713 - 1) << 14) + 0x80000000,	// for PRN49
 3873 + (( 6102 - 1) << 14) + 0x80000000,	// for PRN50
 4026 + (( 6123 - 1) << 14) + 0x80000000,	// for PRN51
 4272 + (( 6070 - 1) << 14) + 0x80000000,	// for PRN52
 3556 + (( 1115 - 1) << 14) + 0x80000000,	// for PRN53
  128 + (( 8047 - 1) << 14) + 0x80000000,	// for PRN54
 1200 + (( 6795 - 1) << 14) + 0x80000000,	// for PRN55
  130 + (( 2575 - 1) << 14) + 0x80000000,	// for PRN56
 4494 + ((   53 - 1) << 14) + 0x80000000,	// for PRN57
 1871 + (( 1729 - 1) << 14) + 0x80000000,	// for PRN58
 3073 + (( 6388 - 1) << 14) + 0x80000000,	// for PRN59
 4386 + ((  682 - 1) << 14) + 0x80000000,	// for PRN60
 4098 + (( 5565 - 1) << 14) + 0x80000000,	// for PRN61
 1923 + (( 7160 - 1) << 14) + 0x80000000,	// for PRN62
 1176 + (( 2277 - 1) << 14) + 0x80000000,	// for PRN63
};

unsigned int L1CDataInit[63] = {
 5111 + ((  412 - 1) << 14) + 0xa0000000,	// for PRN01
 5109 + ((  161 - 1) << 14) + 0xa0000000,	// for PRN02
 5108 + ((    1 - 1) << 14) + 0xa0000000,	// for PRN03
 5106 + ((  303 - 1) << 14) + 0xa0000000,	// for PRN04
 5103 + ((  207 - 1) << 14) + 0xa0000000,	// for PRN05
 5101 + (( 4971 - 1) << 14) + 0xa0000000,	// for PRN06
 5100 + (( 4496 - 1) << 14) + 0xa0000000,	// for PRN07
 5098 + ((    5 - 1) << 14) + 0xa0000000,	// for PRN08
 5095 + (( 4557 - 1) << 14) + 0xa0000000,	// for PRN09
 5094 + ((  485 - 1) << 14) + 0xa0000000,	// for PRN10
 5093 + ((  253 - 1) << 14) + 0xa0000000,	// for PRN11
 5091 + (( 4676 - 1) << 14) + 0xa0000000,	// for PRN12
 5090 + ((    1 - 1) << 14) + 0xa0000000,	// for PRN13
 5081 + ((   66 - 1) << 14) + 0xa0000000,	// for PRN14
 5080 + (( 4485 - 1) << 14) + 0xa0000000,	// for PRN15
 5069 + ((  282 - 1) << 14) + 0xa0000000,	// for PRN16
 5068 + ((  193 - 1) << 14) + 0xa0000000,	// for PRN17
 5054 + (( 5211 - 1) << 14) + 0xa0000000,	// for PRN18
 5044 + ((  729 - 1) << 14) + 0xa0000000,	// for PRN19
 5027 + (( 4848 - 1) << 14) + 0xa0000000,	// for PRN20
 5026 + ((  982 - 1) << 14) + 0xa0000000,	// for PRN21
 5014 + (( 5955 - 1) << 14) + 0xa0000000,	// for PRN22
 5004 + (( 9805 - 1) << 14) + 0xa0000000,	// for PRN23
 4980 + ((  670 - 1) << 14) + 0xa0000000,	// for PRN24
 4915 + ((  464 - 1) << 14) + 0xa0000000,	// for PRN25
 4909 + ((   29 - 1) << 14) + 0xa0000000,	// for PRN26
 4893 + ((  429 - 1) << 14) + 0xa0000000,	// for PRN27
 4885 + ((  394 - 1) << 14) + 0xa0000000,	// for PRN28
 4832 + ((  616 - 1) << 14) + 0xa0000000,	// for PRN29
 4824 + (( 9457 - 1) << 14) + 0xa0000000,	// for PRN30
 4591 + (( 4429 - 1) << 14) + 0xa0000000,	// for PRN31
 3706 + (( 4771 - 1) << 14) + 0xa0000000,	// for PRN32
 5092 + ((  365 - 1) << 14) + 0xa0000000,	// for PRN33
 4986 + (( 9705 - 1) << 14) + 0xa0000000,	// for PRN34
 4965 + (( 9489 - 1) << 14) + 0xa0000000,	// for PRN35
 4920 + (( 4193 - 1) << 14) + 0xa0000000,	// for PRN36
 4917 + (( 9947 - 1) << 14) + 0xa0000000,	// for PRN37
 4858 + ((  824 - 1) << 14) + 0xa0000000,	// for PRN38
 4847 + ((  864 - 1) << 14) + 0xa0000000,	// for PRN39
 4790 + ((  347 - 1) << 14) + 0xa0000000,	// for PRN40
 4770 + ((  677 - 1) << 14) + 0xa0000000,	// for PRN41
 4318 + (( 6544 - 1) << 14) + 0xa0000000,	// for PRN42
 4126 + (( 6312 - 1) << 14) + 0xa0000000,	// for PRN43
 3961 + (( 9804 - 1) << 14) + 0xa0000000,	// for PRN44
 3790 + ((  278 - 1) << 14) + 0xa0000000,	// for PRN45
 4911 + (( 9461 - 1) << 14) + 0xa0000000,	// for PRN46
 4881 + ((  444 - 1) << 14) + 0xa0000000,	// for PRN47
 4827 + (( 4839 - 1) << 14) + 0xa0000000,	// for PRN48
 4795 + (( 4144 - 1) << 14) + 0xa0000000,	// for PRN49
 4789 + (( 9875 - 1) << 14) + 0xa0000000,	// for PRN50
 4725 + ((  197 - 1) << 14) + 0xa0000000,	// for PRN51
 4675 + (( 1156 - 1) << 14) + 0xa0000000,	// for PRN52
 4539 + (( 4674 - 1) << 14) + 0xa0000000,	// for PRN53
 4535 + ((10035 - 1) << 14) + 0xa0000000,	// for PRN54
 4458 + (( 4504 - 1) << 14) + 0xa0000000,	// for PRN55
 4197 + ((    5 - 1) << 14) + 0xa0000000,	// for PRN56
 4096 + (( 9937 - 1) << 14) + 0xa0000000,	// for PRN57
 3484 + ((  430 - 1) << 14) + 0xa0000000,	// for PRN58
 3481 + ((    5 - 1) << 14) + 0xa0000000,	// for PRN59
 3393 + ((  355 - 1) << 14) + 0xa0000000,	// for PRN60
 3175 + ((  909 - 1) << 14) + 0xa0000000,	// for PRN61
 2360 + (( 1622 - 1) << 14) + 0xa0000000,	// for PRN62
 1852 + (( 6284 - 1) << 14) + 0xa0000000,	// for PRN63
};

unsigned int L1CPilotInit[63] = {
 5097 + ((  181 - 1) << 14) + 0xa0000000,	// for PRN01
 5110 + ((  359 - 1) << 14) + 0xa0000000,	// for PRN02
 5079 + ((   72 - 1) << 14) + 0xa0000000,	// for PRN03
 4403 + (( 1110 - 1) << 14) + 0xa0000000,	// for PRN04
 4121 + (( 1480 - 1) << 14) + 0xa0000000,	// for PRN05
 5043 + (( 5034 - 1) << 14) + 0xa0000000,	// for PRN06
 5042 + (( 4622 - 1) << 14) + 0xa0000000,	// for PRN07
 5104 + ((    1 - 1) << 14) + 0xa0000000,	// for PRN08
 4940 + (( 4547 - 1) << 14) + 0xa0000000,	// for PRN09
 5035 + ((  826 - 1) << 14) + 0xa0000000,	// for PRN10
 4372 + (( 6284 - 1) << 14) + 0xa0000000,	// for PRN11
 5064 + (( 4195 - 1) << 14) + 0xa0000000,	// for PRN12
 5084 + ((  368 - 1) << 14) + 0xa0000000,	// for PRN13
 5048 + ((    1 - 1) << 14) + 0xa0000000,	// for PRN14
 4950 + (( 4796 - 1) << 14) + 0xa0000000,	// for PRN15
 5019 + ((  523 - 1) << 14) + 0xa0000000,	// for PRN16
 5076 + ((  151 - 1) << 14) + 0xa0000000,	// for PRN17
 3736 + ((  713 - 1) << 14) + 0xa0000000,	// for PRN18
 4993 + (( 9850 - 1) << 14) + 0xa0000000,	// for PRN19
 5060 + (( 5734 - 1) << 14) + 0xa0000000,	// for PRN20
 5061 + ((   34 - 1) << 14) + 0xa0000000,	// for PRN21
 5096 + (( 6142 - 1) << 14) + 0xa0000000,	// for PRN22
 4983 + ((  190 - 1) << 14) + 0xa0000000,	// for PRN23
 4783 + ((  644 - 1) << 14) + 0xa0000000,	// for PRN24
 4991 + ((  467 - 1) << 14) + 0xa0000000,	// for PRN25
 4815 + (( 5384 - 1) << 14) + 0xa0000000,	// for PRN26
 4443 + ((  801 - 1) << 14) + 0xa0000000,	// for PRN27
 4769 + ((  594 - 1) << 14) + 0xa0000000,	// for PRN28
 4879 + (( 4450 - 1) << 14) + 0xa0000000,	// for PRN29
 4894 + (( 9437 - 1) << 14) + 0xa0000000,	// for PRN30
 4985 + (( 4307 - 1) << 14) + 0xa0000000,	// for PRN31
 5056 + (( 5906 - 1) << 14) + 0xa0000000,	// for PRN32
 4921 + ((  378 - 1) << 14) + 0xa0000000,	// for PRN33
 5036 + (( 9448 - 1) << 14) + 0xa0000000,	// for PRN34
 4812 + (( 9432 - 1) << 14) + 0xa0000000,	// for PRN35
 4838 + (( 5849 - 1) << 14) + 0xa0000000,	// for PRN36
 4855 + (( 5547 - 1) << 14) + 0xa0000000,	// for PRN37
 4904 + (( 9546 - 1) << 14) + 0xa0000000,	// for PRN38
 4753 + (( 9132 - 1) << 14) + 0xa0000000,	// for PRN39
 4483 + ((  403 - 1) << 14) + 0xa0000000,	// for PRN40
 4942 + (( 3766 - 1) << 14) + 0xa0000000,	// for PRN41
 4813 + ((    3 - 1) << 14) + 0xa0000000,	// for PRN42
 4957 + ((  684 - 1) << 14) + 0xa0000000,	// for PRN43
 4618 + (( 9711 - 1) << 14) + 0xa0000000,	// for PRN44
 4669 + ((  333 - 1) << 14) + 0xa0000000,	// for PRN45
 4969 + (( 6124 - 1) << 14) + 0xa0000000,	// for PRN46
 5031 + ((10216 - 1) << 14) + 0xa0000000,	// for PRN47
 5038 + (( 4251 - 1) << 14) + 0xa0000000,	// for PRN48
 4740 + (( 9893 - 1) << 14) + 0xa0000000,	// for PRN49
 4073 + (( 9884 - 1) << 14) + 0xa0000000,	// for PRN50
 4843 + (( 4627 - 1) << 14) + 0xa0000000,	// for PRN51
 4979 + (( 4449 - 1) << 14) + 0xa0000000,	// for PRN52
 4867 + (( 9798 - 1) << 14) + 0xa0000000,	// for PRN53
 4964 + ((  985 - 1) << 14) + 0xa0000000,	// for PRN54
 5025 + (( 4272 - 1) << 14) + 0xa0000000,	// for PRN55
 4579 + ((  126 - 1) << 14) + 0xa0000000,	// for PRN56
 4390 + ((10024 - 1) << 14) + 0xa0000000,	// for PRN57
 4763 + ((  434 - 1) << 14) + 0xa0000000,	// for PRN58
 4612 + (( 1029 - 1) << 14) + 0xa0000000,	// for PRN59
 4784 + ((  561 - 1) << 14) + 0xa0000000,	// for PRN60
 3716 + ((  289 - 1) << 14) + 0xa0000000,	// for PRN61
 4703 + ((  638 - 1) << 14) + 0xa0000000,	// for PRN62
 4851 + (( 4353 - 1) << 14) + 0xa0000000,	// for PRN63
};

unsigned int B2aDataInit[63] = {
	0x0a40bffe,	// for PRN01
	0x02c0bffe,	// for PRN02
	0x0b50bffe,	// for PRN03
	0x0f28bffe,	// for PRN04
	0x0aa8bffe,	// for PRN05
	0x0758fffe,	// for PRN06
	0x0778fffe,	// for PRN07
	0x0df8bffe,	// for PRN08
	0x094cbffe,	// for PRN09
	0x05bcfffe,	// for PRN10
	0x0ac2bffe,	// for PRN11
	0x0222bffe,	// for PRN12
	0x0aa2bffe,	// for PRN13
	0x0da2bffe,	// for PRN14
	0x03a2fffe,	// for PRN15
	0x0c52fffe,	// for PRN16
	0x0ef2fffe,	// for PRN17
	0x080afffe,	// for PRN18
	0x07cafffe,	// for PRN19
	0x0d5abffe,	// for PRN20
	0x08dabffe,	// for PRN21
	0x0ca6bffe,	// for PRN22
	0x0466bffe,	// for PRN23
	0x0196bffe,	// for PRN24
	0x06d6bffe,	// for PRN25
	0x04f6fffe,	// for PRN26
	0x0ff6fffe,	// for PRN27
	0x048ebffe,	// for PRN28
	0x03cebffe,	// for PRN29
	0x085efffe,	// for PRN30
	0x013ebffe,	// for PRN31
	0x02bebffe,	// for PRN32
	0x0d7ebffe,	// for PRN33
	0x0cfefffe,	// for PRN34
	0x08a1bffe,	// for PRN35
	0x0291bffe,	// for PRN36
	0x0ed1bffe,	// for PRN37
	0x0889fffe,	// for PRN38
	0x0989fffe,	// for PRN39
	0x0d59fffe,	// for PRN40
	0x08d9fffe,	// for PRN41
	0x04b9fffe,	// for PRN42
	0x0aa5fffe,	// for PRN43
	0x02e5bffe,	// for PRN44
	0x0d35bffe,	// for PRN45
	0x0eadfffe,	// for PRN46
	0x02c3fffe,	// for PRN47
	0x0c13fffe,	// for PRN48
	0x0d13fffe,	// for PRN49
	0x0c53fffe,	// for PRN50
	0x0153fffe,	// for PRN51
	0x0dcbfffe,	// for PRN52
	0x0e9bbffe,	// for PRN53
	0x0127fffe,	// for PRN54
	0x0297fffe,	// for PRN55
	0x0997fffe,	// for PRN56
	0x05b7fffe,	// for PRN57
	0x01f7bffe,	// for PRN58
	0x0ff7fffe,	// for PRN59
	0x0adffffe,	// for PRN60
	0x04023ffe,	// for PRN61
	0x0afdbffe,	// for PRN62
	0x04bc7ffe,	// for PRN63
};

unsigned int B2aPilotInit[63] = {
	0x0a40fffe,	// for PRN01
	0x02c0bffe,	// for PRN02
	0x0b50fffe,	// for PRN03
	0x0f28fffe,	// for PRN04
	0x0aa8bffe,	// for PRN05
	0x0758bffe,	// for PRN06
	0x0778fffe,	// for PRN07
	0x0df8bffe,	// for PRN08
	0x094cbffe,	// for PRN09
	0x05bcbffe,	// for PRN10
	0x0ac2fffe,	// for PRN11
	0x0222bffe,	// for PRN12
	0x0aa2fffe,	// for PRN13
	0x0da2bffe,	// for PRN14
	0x03a2bffe,	// for PRN15
	0x0c52bffe,	// for PRN16
	0x0ef2fffe,	// for PRN17
	0x080afffe,	// for PRN18
	0x07cabffe,	// for PRN19
	0x0d5afffe,	// for PRN20
	0x08dabffe,	// for PRN21
	0x0ca6bffe,	// for PRN22
	0x0466bffe,	// for PRN23
	0x0196fffe,	// for PRN24
	0x06d6fffe,	// for PRN25
	0x04f6bffe,	// for PRN26
	0x0ff6fffe,	// for PRN27
	0x048efffe,	// for PRN28
	0x03cefffe,	// for PRN29
	0x085ebffe,	// for PRN30
	0x013efffe,	// for PRN31
	0x02befffe,	// for PRN32
	0x0d7ebffe,	// for PRN33
	0x0cfebffe,	// for PRN34
	0x08a1fffe,	// for PRN35
	0x0291fffe,	// for PRN36
	0x0ed1bffe,	// for PRN37
	0x0889fffe,	// for PRN38
	0x0989fffe,	// for PRN39
	0x0d59fffe,	// for PRN40
	0x08d9bffe,	// for PRN41
	0x04b9bffe,	// for PRN42
	0x0aa5fffe,	// for PRN43
	0x02e5fffe,	// for PRN44
	0x0d35bffe,	// for PRN45
	0x0eadfffe,	// for PRN46
	0x02c3bffe,	// for PRN47
	0x0c13fffe,	// for PRN48
	0x0d13fffe,	// for PRN49
	0x0c53bffe,	// for PRN50
	0x0153bffe,	// for PRN51
	0x0dcbfffe,	// for PRN52
	0x0e9bbffe,	// for PRN53
	0x0127bffe,	// for PRN54
	0x0297fffe,	// for PRN55
	0x0997bffe,	// for PRN56
	0x05b7fffe,	// for PRN57
	0x01f7fffe,	// for PRN58
	0x0ff7fffe,	// for PRN59
	0x0adfbffe,	// for PRN60
	0x0612bffe,	// for PRN61
	0x01fa7ffe,	// for PRN62
	0x0aac7ffe,	// for PRN63
};
