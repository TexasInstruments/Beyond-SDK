Goal:
	This example is created to test PCIE-Gen2 devices. 
	https://www.ti.com/tool/TMDS64EVM is used for both EP and RC
		EP: PCIE End Point device
		RC: PCIE Root Controller device
	
	1. using_udma_triggered: This code will use UDMA event triggered call back method for UDMA transfer completion.
	2. using_udma_polling: This code will not use UDMA event triggered call back method but keep on polling for UDMA completion.

Here, 
	EP will wrie data in DDR at location dst_buf[BUF_SIZE]
	RC will read this and copy into DDR at location ddr_buf[NUM_CHANNEL][BUF_SIZE]
		NUM_Channel is set to :  16
		BUFF_SIZE is set to : BUF_SIZE (0x4000000u / NUM_CHANNEL)
		
Test Environment:
	This code is build and tested with https://www.ti.com/tool/download/MCU-PLUS-SDK-AM64X/08.06.00.45

