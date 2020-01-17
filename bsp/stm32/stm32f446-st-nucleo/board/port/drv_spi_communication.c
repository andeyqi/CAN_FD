/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-27     zylx         first version
 */
 
#include <board.h>
#include <drv_spi.h>
#include <rtdevice.h>
#include <rthw.h>
#include <finsh.h>

static int rt_hw_spi_communication_init(void)
{
    rt_hw_spi_device_attach("spi1", "spi10", GPIOB, GPIO_PIN_6);

    return RT_EOK;
}
INIT_PREV_EXPORT(rt_hw_spi_communication_init);


static void spisend(int argc, char**argv)
{
    static rt_uint8_t isconfigflag = 0,sendsize;
    static struct rt_spi_device *spi_dev_com;     /* SPI 设备句柄 */
    struct rt_spi_configuration cfg;
    rt_err_t ret;
    rt_uint8_t sendbuff[100];
    
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
    cfg.max_hz = 20*1000*1000;                           /* 20M */

    if (argc < 2)
    {
        rt_kprintf("Not input send data will send 0X55 0XAA'\n");
        sendbuff[0] = 0x55;
        sendbuff[1] = 0xaa;
        sendbuff[2] = 0x55;
        sendbuff[3] = 0xaa;
        sendbuff[4] = 0x55;
        sendbuff[5] = 0xaa;
        sendbuff[6] = 0x55;
        sendbuff[7] = 0xaa;
        sendbuff[8] = 0x55;
        sendbuff[9] = 0xaa;
        sendbuff[10] = 0x55;
        sendbuff[11] = 0xaa;
    }
    else
    {
        rt_kprintf("Start send data\n");
    }
    if(!isconfigflag)
    {
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
    }
    sendsize = rt_spi_transfer(spi_dev_com,(void *)sendbuff,NULL,12);
    rt_kprintf("SPI BUS send data len is %d\n",sendsize);
}

MSH_CMD_EXPORT(spisend, spisend sample: spisend <data>);
