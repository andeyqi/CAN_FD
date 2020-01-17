#include <rtthread.h>



static struct rt_thread tcan4550;
static rt_uint8_t tcan4550_stack[512];

static void tcan4550_thread_entry(void* parameter)
{

    while (1)
    {
        rt_kprintf("in  tcan4550_thread_entry\n");

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


