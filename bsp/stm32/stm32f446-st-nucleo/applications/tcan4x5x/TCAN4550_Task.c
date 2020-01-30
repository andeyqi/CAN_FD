#include <rtthread.h>
#include <board.h>
#include <drv_spi.h>
#include <rtdevice.h>
#include <rthw.h>
#include "TCAN4550.h"

static struct rt_thread tcan4550;
static rt_uint8_t tcan4550_stack[512];
struct rt_spi_device *spi_dev_com;	  /* SPI 设备句柄 */

/* defined the RESET: PA0 */
#define TCAN_RST    GET_PIN(A,0)
static void tcandump(int argc, char**argv);

/*
 * Configure the TCAN4550
 */
void
Init_CAN(void)
{
	bool ret;
    TCAN4x5x_Device_ClearSPIERR();                              // Clear any SPI ERR flags that might be set as a result of our pin mux changing during MCU startup

	rt_kprintf("0c 0x %x\n",AHB_READ_32(REG_SPI_STATUS));
    /* Step one attempt to clear all interrupts */
	TCAN4x5x_Device_Interrupt_Enable dev_ie = {0};				// Initialize to 0 to all bits are set to 0.
	ret = TCAN4x5x_Device_ConfigureInterruptEnable(&dev_ie);	        // Disable all non-MCAN related interrupts for simplicity
	if(ret == false)
	{
		rt_kprintf("1 # \n");
	}
	TCAN4x5x_Device_Interrupts dev_ir = {0};					// Setup a new MCAN IR object for easy interrupt checking
	TCAN4x5x_Device_ReadInterrupts(&dev_ir);					// Request that the struct be updated with current DEVICE (not MCAN) interrupt values
	rt_kprintf("dev isr %08x\n",dev_ir.word);
	if (dev_ir.PWRON)                                           // If the Power On interrupt flag is set
		TCAN4x5x_Device_ClearInterrupts(&dev_ir);               // Clear it because if it's not cleared within ~4 minutes, it goes to sleep
	rt_kprintf("dev isr %08x\n",AHB_READ_32(REG_DEV_IR));
	/* Configure the CAN bus speeds */
	TCAN4x5x_MCAN_Nominal_Timing_Simple TCANNomTiming = {0};	// 500k arbitration with a 40 MHz crystal ((40E6 / 2) / (32 + 8) = 500E3)
	TCANNomTiming.NominalBitRatePrescaler = 2;
	TCANNomTiming.NominalTqBeforeSamplePoint = 32;
	TCANNomTiming.NominalTqAfterSamplePoint = 8;

	TCAN4x5x_MCAN_Data_Timing_Simple TCANDataTiming = {0};		// 2 Mbps CAN FD with a 40 MHz crystal (40E6 / (15 + 5) = 2E6)
	TCANDataTiming.DataBitRatePrescaler = 1;
	TCANDataTiming.DataTqBeforeSamplePoint = 15;
	TCANDataTiming.DataTqAfterSamplePoint = 5;

	/* Configure the MCAN core settings */
	TCAN4x5x_MCAN_CCCR_Config cccrConfig = {0};					// Remember to initialize to 0, or you'll get random garbage!
	cccrConfig.FDOE = 0;										// CAN FD mode enable
	cccrConfig.BRSE = 0;										// CAN FD Bit rate switch enable

	/* Configure the default CAN packet filtering settings */
	TCAN4x5x_MCAN_Global_Filter_Configuration gfc = {0};
	gfc.RRFE = 1;                                               // Reject remote frames (TCAN4x5x doesn't support this)
	gfc.RRFS = 1;                                               // Reject remote frames (TCAN4x5x doesn't support this)
	gfc.ANFE = TCAN4x5x_GFC_ACCEPT_INTO_RXFIFO0;                // Default behavior if incoming message doesn't match a filter is to accept into RXFIO0 for extended ID messages (29 bit IDs)
	gfc.ANFS = TCAN4x5x_GFC_ACCEPT_INTO_RXFIFO0;                // Default behavior if incoming message doesn't match a filter is to accept into RXFIO0 for standard ID messages (11 bit IDs)

	/* ************************************************************************
	 * In the next configuration block, we will set the MCAN core up to have:
	 *   - 1 SID filter element
	 *   - 1 XID Filter element
	 *   - 5 RX FIFO 0 elements
	 *   - RX FIFO 0 supports data payloads up to 64 bytes
	 *   - RX FIFO 1 and RX Buffer will not have any elements, but we still set their data payload sizes, even though it's not required
	 *   - No TX Event FIFOs
	 *   - 2 Transmit buffers supporting up to 64 bytes of data payload
	 */
	TCAN4x5x_MRAM_Config MRAMConfiguration = {0};
	MRAMConfiguration.SIDNumElements = 1;						// Standard ID number of elements, you MUST have a filter written to MRAM for each element defined
	MRAMConfiguration.XIDNumElements = 1;						// Extended ID number of elements, you MUST have a filter written to MRAM for each element defined
	MRAMConfiguration.Rx0NumElements = 5;						// RX0 Number of elements
	MRAMConfiguration.Rx0ElementSize = MRAM_64_Byte_Data;		// RX0 data payload size
	MRAMConfiguration.Rx1NumElements = 0;						// RX1 number of elements
	MRAMConfiguration.Rx1ElementSize = MRAM_64_Byte_Data;		// RX1 data payload size
	MRAMConfiguration.RxBufNumElements = 0;						// RX buffer number of elements
	MRAMConfiguration.RxBufElementSize = MRAM_64_Byte_Data;		// RX buffer data payload size
	MRAMConfiguration.TxEventFIFONumElements = 0;				// TX Event FIFO number of elements
	MRAMConfiguration.TxBufferNumElements = 2;					// TX buffer number of elements
	MRAMConfiguration.TxBufferElementSize = MRAM_64_Byte_Data;	// TX buffer data payload size


	/* Configure the MCAN core with the settings above, the changes in this block are write protected registers,      *
	 * so it makes the most sense to do them all at once, so we only unlock and lock once                             */

	ret = TCAN4x5x_MCAN_EnableProtectedRegisters();					// Start by making protected registers accessible
	if(false == ret)
	{
		rt_kprintf("2#\n");
	}
	ret = TCAN4x5x_MCAN_ConfigureCCCRRegister(&cccrConfig);			// Enable FD mode and Bit rate switching
	if(false == ret)
	{
		rt_kprintf("3#\n");
	}
	ret = TCAN4x5x_MCAN_ConfigureGlobalFilter(&gfc);                  // Configure the global filter configuration (Default CAN message behavior)
	if(false == ret)
	{
		rt_kprintf("4#\n");
	}
	ret =TCAN4x5x_MCAN_ConfigureNominalTiming_Simple(&TCANNomTiming);// Setup nominal/arbitration bit timing
	if(false == ret)
	{
		rt_kprintf("5#\n");
	}

	TCAN4x5x_MCAN_ConfigureDataTiming_Simple(&TCANDataTiming);	// Setup CAN FD timing
	TCAN4x5x_MRAM_Clear();										// Clear all of MRAM (Writes 0's to all of it)
	ret = TCAN4x5x_MRAM_Configure(&MRAMConfiguration);				// Set up the applicable registers related to MRAM configuration
	if(false == ret)
	{
		rt_kprintf("6#\n");
	}

	ret = TCAN4x5x_MCAN_DisableProtectedRegisters();					// Disable protected write and take device out of INIT mode
	if(false == ret)
	{
		rt_kprintf("7#\n");
	}


	/* Set the interrupts we want to enable for MCAN */
	TCAN4x5x_MCAN_Interrupt_Enable mcan_ie = {0};				// Remember to initialize to 0, or you'll get random garbage!
	mcan_ie.RF0NE = 1;											// RX FIFO 0 new message interrupt enable

	TCAN4x5x_MCAN_ConfigureInterruptEnable(&mcan_ie);			// Enable the appropriate registers


	/* Setup filters, this filter will mark any message with ID 0x055 as a priority message */
	TCAN4x5x_MCAN_SID_Filter SID_ID = {0};
	SID_ID.SFT = TCAN4x5x_SID_SFT_CLASSIC;						// SFT: Standard filter type. Configured as a classic filter
	SID_ID.SFEC = TCAN4x5x_SID_SFEC_PRIORITYSTORERX0;			// Standard filter element configuration, store it in RX fifo 0 as a priority message
	SID_ID.SFID1 = 0x055;										// SFID1 (Classic mode Filter)
	SID_ID.SFID2 = 0x7FF;										// SFID2 (Classic mode Mask)
	ret = TCAN4x5x_MCAN_WriteSIDFilter(0, &SID_ID);					// Write to the MRAM
	if(false == ret)
	{
		rt_kprintf("8#\n");
	}


	/* Store ID 0x12345678 as a priority message */
	TCAN4x5x_MCAN_XID_Filter XID_ID = {0};
	XID_ID.EFT = TCAN4x5x_XID_EFT_CLASSIC;                      // EFT
	XID_ID.EFEC = TCAN4x5x_XID_EFEC_PRIORITYSTORERX0;           // EFEC
	XID_ID.EFID1 = 0x12345678;                                  // EFID1 (Classic mode filter)
	XID_ID.EFID2 = 0x1FFFFFFF;                                  // EFID2 (Classic mode mask)
	ret = TCAN4x5x_MCAN_WriteXIDFilter(0, &XID_ID);                   // Write to the MRAM
	if(false == ret)
	{
		rt_kprintf("9#\n");
	}

	/* Configure the TCAN4550 Non-CAN-related functions */
	TCAN4x5x_DEV_CONFIG devConfig = {0};                        // Remember to initialize to 0, or you'll get random garbage!
	devConfig.SWE_DIS = 0;                                      // Keep Sleep Wake Error Enabled (it's a disable bit, not an enable)
	devConfig.DEVICE_RESET = 0;                                 // Not requesting a software reset
	devConfig.WD_EN = 0;                                        // Watchdog disabled
	devConfig.nWKRQ_CONFIG = 0;                                 // Mirror INH function (default)
	devConfig.INH_DIS = 0;                                      // INH enabled (default)
	devConfig.GPIO1_GPO_CONFIG = TCAN4x5x_DEV_CONFIG_GPO1_MCAN_INT1;    // MCAN nINT 1 (default)
	devConfig.FAIL_SAFE_EN = 0;                                 // Failsafe disabled (default)
	devConfig.GPIO1_CONFIG = TCAN4x5x_DEV_CONFIG_GPIO1_CONFIG_GPO;      // GPIO set as GPO (Default)
	devConfig.WD_ACTION = TCAN4x5x_DEV_CONFIG_WDT_ACTION_nINT;  // Watchdog set an interrupt (default)
	devConfig.WD_BIT_RESET = 0;                                 // Don't reset the watchdog
	devConfig.nWKRQ_VOLTAGE = 0;                                // Set nWKRQ to internal voltage rail (default)
	devConfig.GPO2_CONFIG = TCAN4x5x_DEV_CONFIG_GPO2_NO_ACTION; // GPO2 has no behavior (default)
	devConfig.CLK_REF = 1;                                      // Input crystal is a 40 MHz crystal (default)
	devConfig.WAKE_CONFIG = TCAN4x5x_DEV_CONFIG_WAKE_BOTH_EDGES;// Wake pin can be triggered by either edge (default)
	ret = TCAN4x5x_Device_Configure(&devConfig);                      // Configure the device with the above configuration
	if(false == ret)
	{
		rt_kprintf("10#\n");
	}

	ret = TCAN4x5x_Device_SetMode(TCAN4x5x_DEVICE_MODE_NORMAL);       // Set to normal mode, since configuration is done. This line turns on the transceiver
	if(false == ret)
	{
		rt_kprintf("11#\n");
	}

	TCAN4x5x_MCAN_ClearInterruptsAll();                         // Resets all MCAN interrupts (does NOT include any SPIERR interrupts)
}



static void tcan4550_thread_entry(void* parameter)
{
	struct rt_spi_configuration cfg;
	rt_err_t ret;
	bool retv;

	cfg.data_width = 8;
	cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
	cfg.max_hz = 8*1000*1000;							 /* 1M */
	
    rt_hw_spi_device_attach("spi1", "spi10", GPIOB, GPIO_PIN_6);

	 /* 查找 spi 设备获取设备句柄 */
    spi_dev_com = (struct rt_spi_device *)rt_device_find("spi10");
    if(!spi_dev_com)
    {
        rt_kprintf("Not find spi10 device\n");
    }
    ret = rt_spi_configure(spi_dev_com, &cfg);
    if(ret != RT_EOK)
    {
        rt_kprintf("Config spi10 failed\n");
    }

	rt_pin_mode(TCAN_RST, PIN_MODE_OUTPUT);
	rt_pin_write(TCAN_RST, PIN_HIGH);
	rt_thread_delay(10);
	rt_pin_write(TCAN_RST, PIN_LOW);
	rt_thread_delay(10);

	uint32_t device_id0;
	device_id0 = AHB_READ_32(REG_SPI_DEVICE_ID0);
	rt_kprintf("device_id0 %x\n",device_id0);
  	rt_thread_delay(100);

	Init_CAN();

			/* Define the CAN message we want to send*/
		TCAN4x5x_MCAN_TX_Header header = {0};			// Remember to initialize to 0, or you'll get random garbage!
		uint8_t data[8] = {0x55, 0x66, 0x77, 0x88};		// Define the data payload
		header.DLC = MCAN_DLC_8B;						// Set the DLC to be equal to or less than the data payload (it is ok to pass a 64 byte data array into the WriteTXFIFO function if your DLC is 8 bytes, only the first 8 bytes will be read)
		header.ID = 0x100;								// Set the ID
		header.FDF = 0;									// CAN FD frame enabled
		header.BRS = 0;									// Bit rate switch enabled
		header.EFC = 0;
		header.MM  = 0;
		header.RTR = 0;
		header.XTD = 0;									// We are not using an extended ID in this example
		header.ESI = 0;									// Error state indicator


		//retv = TCAN4x5x_MCAN_WriteTXBuffer(0, &header, data);	// This function actually writes the header and data payload to the TCAN's MRAM in the specified TX queue number. It returns the bit necessary to write to TXBAR,
		rt_kprintf("%d\n",TCAN4x5x_MCAN_WriteTXBuffer(0, &header, data));												// but does not necessarily require you to use it. In this example, we won't, so that we can send the data queued up at a later point.
	

		retv = TCAN4x5x_MCAN_TransmitBufferContents(0);		// Now we can send the TX FIFO element 0 data that we had queued up earlier but didn't send.
		if(retv == false)
    	{
    		rt_kprintf("13\n");
    	}

    while (1)
    {
    	TCAN4x5x_Device_Interrupts dev_ir = {0};					// Setup a new MCAN IR object for easy interrupt checking
		TCAN4x5x_Device_ReadInterrupts(&dev_ir);					// Request that the struct be updated with current DEVICE (not MCAN) interrupt values
		if (dev_ir.word)                                           // If the Power On interrupt flag is set
			TCAN4x5x_Device_ClearInterrupts(&dev_ir);
    	TCAN4x5x_MCAN_TransmitBufferContents(0);
   #if  0 
		/* Define the CAN message we want to send*/
		TCAN4x5x_MCAN_TX_Header header = {0};			// Remember to initialize to 0, or you'll get random garbage!
		uint8_t data[8] = {0x55, 0x66, 0x77, 0x88};		// Define the data payload
		header.DLC = MCAN_DLC_8B;						// Set the DLC to be equal to or less than the data payload (it is ok to pass a 64 byte data array into the WriteTXFIFO function if your DLC is 8 bytes, only the first 8 bytes will be read)
		header.ID = 0x100;								// Set the ID
		header.FDF = 0;									// CAN FD frame enabled
		header.BRS = 0;									// Bit rate switch enabled
		header.EFC = 0;
		header.MM  = 0;
		header.RTR = 0;
		header.XTD = 0;									// We are not using an extended ID in this example
		header.ESI = 0;									// Error state indicator


		//retv = TCAN4x5x_MCAN_WriteTXBuffer(0, &header, data);	// This function actually writes the header and data payload to the TCAN's MRAM in the specified TX queue number. It returns the bit necessary to write to TXBAR,
		rt_kprintf("%d\n",TCAN4x5x_MCAN_WriteTXBuffer(0, &header, data));												// but does not necessarily require you to use it. In this example, we won't, so that we can send the data queued up at a later point.
	

		retv = TCAN4x5x_MCAN_TransmitBufferContents(0);		// Now we can send the TX FIFO element 0 data that we had queued up earlier but didn't send.
		if(retv == false)
    	{
    		rt_kprintf("13\n");
    	}
#endif
		rt_thread_delay(10);
    }
}


int thread_tcan4550_init(void)
{
    rt_err_t result;

    result = rt_thread_init(&tcan4550, "tcan4550", 
        tcan4550_thread_entry, NULL, 
        &tcan4550_stack[0], sizeof(tcan4550_stack),
        (RT_THREAD_PRIORITY_MAX / 3), 10);
    if (result == RT_EOK) 
        rt_thread_startup(&tcan4550);
	return 0;
}

static void tcandump(int argc, char**argv)
{
	uint16_t reg;
	if(argc > 1)
	{
		sscanf(argv[1],"%x",&reg);
		rt_kprintf("reg value %x\n",AHB_READ_32(reg));
	}
	
}

MSH_CMD_EXPORT(tcandump, tcandump sample: tcandump <data>);



