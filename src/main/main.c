#include "rtthread.h"
#include "system_diag.h"
#include "app_main.h"


/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_err_t result;
    /* 1. 打印应用入口日志 */
    rt_kprintf("[BOOT][00][INFO] application entered\n");

    /* 2. 初始化并打印系统版本、SDK和复位信息 */
    result=system_diag_init();
    if (result== RT_EOK)
    {
        system_diag_print_boot_info();
        rt_kprintf("[BOOT][01][OK] system information ready\n");
    }
    else
    {
        rt_kprintf("[BOOT][01][WARN] system diagnostic init failed\n");
    }
    /* 3. 初始化应用事件队列、线程、RGB和按键 */
    result = app_main_init();
    /* 4. 核心初始化失败时打印错误并返回错误码 */
    if (result != RT_EOK)
    {
        rt_kprintf("[BOOT][02][ERROR] application init failed: %d\n",(int)result);
        return (int)result;
    }
    else
    {
        rt_kprintf("[BOOT][02][OK] application initialized\n");
    }

    /* 5. 启动成功后打印ready并返回0 */
    rt_kprintf("[BOOT][03][INFO] application ready\n");

    return 0;
}