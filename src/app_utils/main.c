/*
 * 属于你自己的智能手表项目起点
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

int main(void)
{
    /* * 这里是系统的最底层入口。
     * 当程序运行到这里时，底层的时钟、内存、RT-Thread 操作系统都已经全自动初始化完毕了。
     */
    
    rt_kprintf("=================================\n");
    rt_kprintf("  My SmartWatch Project Booting! \n");
    rt_kprintf("=================================\n");

    /* 你可以在这里初始化你的传感器、创建你自己的 LVGL 线程等 */

    // 主循环
    while (1)
    {
        // 心跳打印，证明系统还活着
        rt_kprintf("System is running...\n");
        
        // 延时 1000 毫秒 (1秒)
        // 注意：在 RTOS 中必须用延时函数交出 CPU 使用权，绝对不能写死循环！
        rt_thread_mdelay(1000); 
    }

    return 0;
}