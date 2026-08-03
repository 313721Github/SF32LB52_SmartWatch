#include "button_port.h"
#include "board.h"
#include "rtdevice.h"

#ifdef RT_USING_FINSH
#include "finsh.h"
#endif

#define BUTTON_POLL_INTERVAL_MS  2U
#define BUTTON_POLL_DURATION_MS  20000U
#define BUTTON_POLL_SAMPLE_COUNT \
    (BUTTON_POLL_DURATION_MS / BUTTON_POLL_INTERVAL_MS)


//KEY1 KEY2轮询验证

static rt_bool_t g_button_initialized = RT_FALSE; //标记是否已初始化

static const rt_base_t g_button_pin_table[BUTTON_PORT_KEY_COUNT] = 
{   [BUTTON_PORT_KEY1] = BSP_KEY1_PIN,
    [BUTTON_PORT_KEY2] = BSP_KEY2_PIN
}; //KEY1/KEY2对应的GPIO引脚号



rt_err_t button_port_init(void);    //初始化配置KEY1/KEY2为输入，并记录启动时原始电平
rt_err_t button_port_read_raw(button_port_key_t key,rt_uint8_t *level); //读取未消抖的原始电平

/* 私有函数声明 */
static int button_poll_test(int argc, char **argv);  //轮询测试按键原始电平，打印到串口


//初始化按键输入
rt_err_t button_port_init()
{
    rt_device_t pin_device;
    rt_uint8_t key1_level;
    rt_uint8_t key2_level;
    rt_err_t result;

    if(g_button_initialized==RT_TRUE)
    {
        return RT_EOK; //已经初始化过了
    }

    pin_device = rt_device_find("pin");
    if(pin_device==RT_NULL)
    {
        rt_kprintf("[KEY][ERR] pin device not found\n");
        return -RT_ERROR;
    }

    /*
     * BSP已经将PA34/PA43复用为GPIO。
     * 此处只配置GPIO方向为输入、无内部上下拉。
    */
    rt_pin_mode(g_button_pin_table[BUTTON_PORT_KEY1], PIN_MODE_INPUT);
    rt_pin_mode(g_button_pin_table[BUTTON_PORT_KEY2], PIN_MODE_INPUT);

    g_button_initialized = RT_TRUE;

    //读取启动时原始电平
    result = button_port_read_raw(BUTTON_PORT_KEY1, &key1_level);
    if(result != RT_EOK)
    {
        g_button_initialized = RT_FALSE;
        return result;
    }

    result = button_port_read_raw(BUTTON_PORT_KEY2, &key2_level);
    if(result != RT_EOK)
    {
        g_button_initialized = RT_FALSE;
        return result;
    }


    rt_kprintf("[KEY][INIT] KEY1 pin=%d raw=%d active=high\n",
               BSP_KEY1_PIN, key1_level);
    rt_kprintf("[KEY][INIT] KEY2 pin=%d raw=%d active=high\n",
               BSP_KEY2_PIN, key2_level);

    return RT_EOK;

}


//读取按键原始电平
rt_err_t button_port_read_raw(button_port_key_t key,rt_uint8_t *level)
{
    rt_base_t raw_level;

    if(level==RT_NULL)
    {
        return -RT_EINVAL;
    }

    if(g_button_initialized==RT_FALSE)
    {
        return -RT_ERROR; //未初始化
    }

    if(key<BUTTON_PORT_KEY1 || key>=BUTTON_PORT_KEY_COUNT)
    {
        return -RT_EINVAL; //无效的按键编号
    }


    raw_level = rt_pin_read(g_button_pin_table[key]);

    if(raw_level==PIN_LOW)
    {
        *level = 0;
    }
    else
    {
        *level = 1;
    }

    return RT_EOK;
}


static int button_poll_test(int argc, char **argv)
{

    rt_uint8_t previous_key1;
    rt_uint8_t previous_key2;
    rt_uint8_t current_key1;
    rt_uint8_t current_key2;
    rt_uint32_t key1_transitions = 0;
    rt_uint32_t key2_transitions = 0;
    rt_uint32_t sample;
    rt_err_t result;


    //防止未初始化
    result = button_port_init();
    if (result != RT_EOK)
    {
        rt_kprintf("[KEY][TEST][ERR] init failed: %d\n",
                   (int)result);
        return (int)result;
    }

    //读取一次
    result = button_port_read_raw(BUTTON_PORT_KEY1,
                                  &previous_key1);
    if (result != RT_EOK)
    {
        return (int)result;
    }

    result = button_port_read_raw(BUTTON_PORT_KEY2,
                                  &previous_key2);
    if (result != RT_EOK)
    {
        return (int)result;
    }

    rt_kprintf(
        "[KEY][RAW] start, duration=%u ms, interval=%u ms\n",
        BUTTON_POLL_DURATION_MS,
        BUTTON_POLL_INTERVAL_MS);

    rt_kprintf("[KEY][RAW] initial KEY1=%d KEY2=%d\n",
               previous_key1,
               previous_key2);

    //开始轮询测试
    for (sample = 0;
         sample < BUTTON_POLL_SAMPLE_COUNT;
         sample++)
    {
        result = button_port_read_raw(BUTTON_PORT_KEY1,
                                      &current_key1);
        if (result != RT_EOK)
        {
            return (int)result;
        }

        result = button_port_read_raw(BUTTON_PORT_KEY2,
                                      &current_key2);
        if (result != RT_EOK)
        {
            return (int)result;
        }

        if (current_key1 != previous_key1)
        {
            rt_kprintf("[KEY][RAW] tick=%lu KEY1 %d->%d\n",
                       (unsigned long)rt_tick_get(),
                       previous_key1,
                       current_key1);

            previous_key1 = current_key1;
            key1_transitions++;
        }

        if (current_key2 != previous_key2)
        {
            rt_kprintf("[KEY][RAW] tick=%lu KEY2 %d->%d\n",
                       (unsigned long)rt_tick_get(),
                       previous_key2,
                       current_key2);

            previous_key2 = current_key2;
            key2_transitions++;
        }

        rt_thread_mdelay(BUTTON_POLL_INTERVAL_MS);
    }

    rt_kprintf(
        "[KEY][RAW] done, KEY1 transitions=%lu, "
        "KEY2 transitions=%lu\n",
        (unsigned long)key1_transitions,
        (unsigned long)key2_transitions);

    return 0;

}


#ifdef RT_USING_FINSH
MSH_CMD_EXPORT(button_poll_test,
               poll raw KEY1 and KEY2 levels for 20 seconds);
#endif
