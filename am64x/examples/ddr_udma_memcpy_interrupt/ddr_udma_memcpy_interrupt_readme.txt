Goal:

This code is written for benchamrking DDR read and write latency using UDMA channels

This example performs UDMA block copy transfer using Type 15 Transfer Record (TR15)
using Transfer Record Packet Descriptor (TRPD) in interrupt mode.

The application opens and configures a BCDMA channel using SysConfig.
It also configures the interrupt mode of operation through the SysConfig which ensures that all required interrupt configuration are done.
The callback function app_udmaEventDmaCbis registered via SysConfig.

Then the application prepares a TRPD for a 1D transfer from source to destination buffer, submits the request to DMA, waits for the DMA to complete
by waiting on a semaphore which is posted in the callback function.

Once the transfer it completes, it does cache operation for data coherency and compares the source and destination buffers for any data mismatch.

Note:
UDMA TR packet descriptor memory size - with one TR

This example is applicable for even (1.2,4,8,16) count of TRPDs only
#define NUM_CHANNEL 1 //it should be same as CONFIG_UDMA0_NUM_BLKCOPY_CH
