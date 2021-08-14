# greta-oto
 An open source GNSS receiver

This project is an open source project of a consumer level GNSS receiver.
It has the capability to receive L1 band signals (including GPS/QZSS L1C/A, GPS/QZSS L1C, Galileo E1, BDS B1C and SBAS)
and also extendable to receive L5 band signals (GPS/QZSS L5, Galileo E5a and BDS B2a).

First published on 6/8/2021 with initial version of C model and RTL of tracking engine and acquisition engine.
Basic tests have been passed to verify the function of the code.

Change list as below:
6/25/2021
	Add top level module and other minor changes.
	Now this IP is a complete IP with exception of PPS to be added
7/14/2021
	Change dual port SRAM in TE FIFO to single port SRAM
	Fix bug in gnss_top module
7/23/2021
	In C model Change clock counter latch value bit from 4 to 6 to match RTL
	Put Galileo E1 code in GNSS top as ROM
	Reduce unnecessary register value change in TE FIFO RTL on dummy write
	Other minor changes
7/28/2021
	Preview version of firmware added. Just a demo code to show how firmware is constructed.
7/31/2021
	Add measurement interrupt process and baseband measurement extraction
	Add basic data stream decode
8/13/2021
	Add baseband simulation model based on SignalSim, firmware can run based on controlled scenario
	Comments update
	Fix typo error in RTL
