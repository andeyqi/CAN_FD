#include <rtthread.h>
#include <board.h>
#include <drv_spi.h>
#include <rtdevice.h>
#include <rthw.h>
#include "TCAN4550.h"

static struct rt_thread tcan4550;
static rt_uint8_t tcan4550_stack[512];
struct rt_spi_device *spi_dev_com;	  /* SPI 设备句柄 */

static void tcan4550_thread_entry(void* parameter)
{
	struct rt_spi_configuration cfg;
	rt_err_t ret;

	cfg.data_width = 8;
	cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
	cfg.max_hz = 1*1000*1000;							 /* 1M */
	
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
	
    while (1)
    {
    	uint32_t device_id0,device_id1;
    	device_id0 = AHB_READ_32(REG_SPI_DEVICE_ID0);
        rt_kprintf("device_id0 %x\n",device_id0);
		
        //rt_thread_delay(100);
		
    	device_id1 = AHB_READ_32(REG_SPI_DEVICE_ID1);
        rt_kprintf("device_id1 %x\n",device_id1);

        rt_thread_delay(1000);
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
