#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "drivers/rt_drv_pwm.h"


/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    /* Output a message on console using printf function */
    rt_kprintf("Hello world!\n");


    /* Infinite loop */
    while (1)
    {
        rt_kprintf("This is a RT-Thread on BF0 board.\n");
        rt_thread_mdelay(1000);
    }
    return 0;
}