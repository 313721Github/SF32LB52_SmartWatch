#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "system_diag.h"
#include "indicator_port.h"
#include "button_port.h"

#define LCD_WIDTH  390
#define LCD_HEIGHT 450
#define CHUNK_ROWS 50

/* Chunk buffer: CHUNK_ROWS lines × LCD_WIDTH pixels × 2 bytes/pixel (RGB565) */
static uint16_t chunk_buf[CHUNK_ROWS * LCD_WIDTH];

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_kprintf("Hello world!\n");
    rt_kprintf("[BOOT][00][INFO] application entered\n");

    //0、系统诊断信息测试
    if (system_diag_init() == RT_EOK)
    {
        system_diag_print_boot_info();
        rt_kprintf("[BOOT][01][OK] system information ready\n");
    }
    else
    {
        rt_kprintf("[BOOT][01][ERR] system diagnostic init failed\n");
    }


    //1、RGB指示灯测试
    if (indicator_init() == RT_EOK)
    {
        //1、黑色——关闭
        if(indicator_set_state(INDICATOR_STATE_OFF) == RT_EOK)
        {
            rt_kprintf("[BOOT][02][OK] indicator ready\n");
        }
        rt_thread_mdelay(1000);
        
        //2、蓝色——启动中
        if(indicator_set_state(INDICATOR_STATE_BOOTING) == RT_EOK)
        {
            rt_kprintf("[BOOT][02][OK] BOOTING ready\n");
        }
        rt_thread_mdelay(1000);

        //3、绿色——正常
        if(indicator_set_state(INDICATOR_STATE_READY) == RT_EOK)
        {
            rt_kprintf("[BOOT][02][OK] READY ready\n");
        }
        rt_thread_mdelay(1000);

        //4、黄色——降级
        if(indicator_set_state(INDICATOR_STATE_DEGRADED) == RT_EOK)
        {
            rt_kprintf("[BOOT][02][OK] DEGRADED ready\n");
        }
        rt_thread_mdelay(1000);

        //5、红色——严重错误
        if(indicator_set_state(INDICATOR_STATE_FATAL) == RT_EOK)
        {
            rt_kprintf("[BOOT][02][OK] FATAL ready\n");
        }
        rt_thread_mdelay(1000);

        //6、黑色——关闭
        if(indicator_set_state(INDICATOR_STATE_OFF) == RT_EOK)
        {
            rt_kprintf("[BOOT][02][OK] OFF\n");
        }
        rt_thread_mdelay(1000);

    }
    else
    {
        rt_kprintf("[BOOT][02][ERR] indicator init failed\n");
    }   

    rt_err_t result = button_port_init();

    if (result == RT_EOK)
    {
        rt_kprintf("[BUTTON][INIT][OK]\n");
    }
    else
    {
        rt_kprintf("[BUTTON][INIT][ERROR] result=%d\n", result);
    }



    /* Step 1: Open LCD graphic device to power up and initialize the panel */
    rt_device_t lcd_dev = rt_device_find("lcd");
    if (lcd_dev == RT_NULL)
    {
        rt_kprintf("Error: Can't find 'lcd' device!\n");
        return 0;
    }

    if (rt_device_open(lcd_dev, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Error: Can't open 'lcd' device!\n");
        return 0;
    }
    rt_kprintf("LCD panel opened.\n");

    /* Wait for panel power-up and init sequence to complete */
    rt_thread_mdelay(100);


/*
 * Step 2: Set the current AMOLED brightness through CO5300.
 * rt_device_open("lcd") is asynchronous, so this synchronous control call
 * also waits until lcd_task finishes the OPEN operation.
 */
    uint32_t br = 100;
    rt_device_control(lcd_dev, RTGRAPHIC_CTRL_SET_BRIGHTNESS, &br);
    rt_kprintf("LCD ready, brightness set.\n");

    /*
     * Step 3: Set framebuffer pixel format BEFORE drawing.
     * The LCD_MSG_OPEN handler calls HAL_LCDC_LayerReset which clears the
     * layer data_format to 0. This control call reconfigures it for RGB565.
     */
    uint16_t format = RTGRAPHIC_PIXEL_FORMAT_RGB565;
    rt_device_control(lcd_dev, RTGRAPHIC_CTRL_SET_BUF_FORMAT, &format);

    /* Step 4: Fill screen white in chunks.
     * Panel RAMWR (0x2C) resets address to window start each call,
     * so batch CHUNK_ROWS rows per set_window+draw_rect pair. */
    memset(chunk_buf, 0xFF, sizeof(chunk_buf));  /* 0xFFFF = white in RGB565 */

    struct rt_device_graphic_ops *ops = rt_graphix_ops(lcd_dev);
    for (int y = 0; y < LCD_HEIGHT; y += CHUNK_ROWS)
    {
        int rows = (y + CHUNK_ROWS <= LCD_HEIGHT) ? CHUNK_ROWS : LCD_HEIGHT - y;
        int y1 = y + rows - 1;
        ops->set_window(0, y, LCD_WIDTH - 1, y1);
        ops->draw_rect((const char *)chunk_buf, 0, y, LCD_WIDTH - 1, y1);
    }
    rt_kprintf("Screen filled with white.\n");

    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(1000);
    }
    return 0;
}
