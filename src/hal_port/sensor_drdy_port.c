#include "sensor_drdy_port.h"

#include "bf0_hal.h"
#include "rtdevice.h"
#include "rthw.h"

#include <string.h>

#define SENSOR_DRDY_PIN             31
#define SENSOR_DRDY_ACTIVE_LEVEL    PIN_HIGH


typedef struct
{
    rt_bool_t initialized; //PA31是否已配置并注册中断回调
    rt_bool_t enabled;     //PA31中断当前是否已使能

    sensor_drdy_isr_handler_t handler; //DRDY断言时调用的ISR安全通知函数
    void *user_data;                   //传给通知函数的调用者上下文

    sensor_drdy_port_stats_t stats;    //PA31中断与通知的累计统计
} sensor_drdy_port_context_t;

static sensor_drdy_port_context_t g_sensor_drdy_port;
//公开函数声明
rt_err_t sensor_drdy_port_init(sensor_drdy_isr_handler_t handler, void *user_data); //配置PA31并注册DRDY中断，默认不使能

rt_err_t sensor_drdy_port_enable(void); //使能PA31双边沿中断

rt_err_t sensor_drdy_port_disable(void); //禁用PA31中断，保留注册信息

rt_bool_t sensor_drdy_port_is_enabled(void); //查询PA31中断是否已使能

rt_err_t sensor_drdy_port_get_level(rt_uint8_t *level_value); //读取PA31当前数字电平

rt_err_t sensor_drdy_port_get_stats(sensor_drdy_port_stats_t *stats); //复制DRDY中断统计到调用者缓冲区

rt_err_t sensor_drdy_port_reset_stats(void); //清零DRDY中断统计
//私有函数声明
static void sensor_drdy_port_gpio_isr(void *args);

//---------------------------------------------------------------公开函数---------------------------------------------------------------
/*
 * 简介：配置PA31并注册DRDY GPIO中断。
 * 输入：handler为高电平DRDY断言时调用的ISR安全处理函数；
 *       user_data为原样传回handler的上下文。
 * 返回值：成功返回RT_EOK；参数无效返回-RT_EINVAL；
 *         PIN设备不存在返回-RT_ENOSYS；
 *         已绑定其他处理函数返回-RT_EBUSY；
 *         其他失败返回RT-Thread实际错误。
 * 说明：初始化后中断保持禁用。必须由sensor线程先完成器件初始化，
 *       再按规定顺序开启GPIO中断和器件INT1路由。
 */
rt_err_t sensor_drdy_port_init(sensor_drdy_isr_handler_t handler,void *user_data)
{
    rt_device_t pin_device;
    rt_err_t result;
    rt_base_t irq_level;

    if (handler == RT_NULL)
    {
        return -RT_EINVAL;
    }

    irq_level = rt_hw_interrupt_disable();

    if (g_sensor_drdy_port.initialized == RT_TRUE)
    {
        if ((g_sensor_drdy_port.handler == handler) && (g_sensor_drdy_port.user_data == user_data))
        {
            rt_hw_interrupt_enable(irq_level);
            return RT_EOK;
        }

        rt_hw_interrupt_enable(irq_level);
        return -RT_EBUSY;
    }

    rt_hw_interrupt_enable(irq_level);

    pin_device = rt_device_find("pin");
    if (pin_device == RT_NULL)
    {
        return -RT_ENOSYS;
    }

    /*
     * 实物R57未贴装，PA31不存在经过确认的外部上拉。
     * INT1使用高有效推挽输出，因此PA31配置为无内部上下拉输入。
     */
    HAL_PIN_Set(PAD_PA31, GPIO_A31, PIN_NOPULL, 1);
    rt_pin_mode(SENSOR_DRDY_PIN, PIN_MODE_INPUT);

    /*
     * 使用双边沿是为了：
     * 1. SDK配置GPIO_NOPULL；
     * 2. 上升沿记录DRDY断言；
     * 3. 下降沿记录读取后的释放。
     */
    result = rt_pin_attach_irq(SENSOR_DRDY_PIN,PIN_IRQ_MODE_RISING_FALLING,sensor_drdy_port_gpio_isr,RT_NULL);
    if (result != RT_EOK)
    {
        return result;
    }

    irq_level = rt_hw_interrupt_disable();

    g_sensor_drdy_port.handler = handler;
    g_sensor_drdy_port.user_data = user_data;
    g_sensor_drdy_port.initialized = RT_TRUE;
    g_sensor_drdy_port.enabled = RT_FALSE;

    memset(&g_sensor_drdy_port.stats,0,sizeof(g_sensor_drdy_port.stats));

    rt_hw_interrupt_enable(irq_level);

    return RT_EOK;
}

/*
 * 简介：使能PA31双边沿GPIO中断。
 * 输入：无。
 * 返回值：成功返回RT_EOK；端口尚未初始化返回-RT_ENOSYS；
 *         其他失败返回RT-Thread实际错误。
 * 说明：重复使能直接返回RT_EOK。
 */
rt_err_t sensor_drdy_port_enable(void)
{
    rt_err_t result;
    rt_base_t irq_level;

    irq_level = rt_hw_interrupt_disable();

    if (g_sensor_drdy_port.initialized == RT_FALSE)
    {
        rt_hw_interrupt_enable(irq_level);
        return -RT_ENOSYS;
    }

    if (g_sensor_drdy_port.enabled == RT_TRUE)
    {
        rt_hw_interrupt_enable(irq_level);
        return RT_EOK;
    }

    rt_hw_interrupt_enable(irq_level);

    //使能PA31 GPIO中断
    result = rt_pin_irq_enable(SENSOR_DRDY_PIN,PIN_IRQ_ENABLE);
    if (result != RT_EOK)
    {
        return result;
    }

    irq_level = rt_hw_interrupt_disable();
    g_sensor_drdy_port.enabled = RT_TRUE;
    rt_hw_interrupt_enable(irq_level);

    return RT_EOK;
}


/*
 * 简介：禁用PA31 GPIO中断。
 * 输入：无。
 * 返回值：成功返回RT_EOK；端口尚未初始化返回-RT_ENOSYS；
 *         其他失败返回RT-Thread实际错误。
 * 说明：本函数只关闭中断，不解除回调，也不提供虚假deinit。
 */
rt_err_t sensor_drdy_port_disable(void)
{
    rt_err_t result;
    rt_base_t irq_level;

    irq_level = rt_hw_interrupt_disable();

    if (g_sensor_drdy_port.initialized == RT_FALSE)
    {
        rt_hw_interrupt_enable(irq_level);
        return -RT_ENOSYS;
    }

    if (g_sensor_drdy_port.enabled == RT_FALSE)
    {
        rt_hw_interrupt_enable(irq_level);
        return RT_EOK;
    }

    rt_hw_interrupt_enable(irq_level);

    //禁用PA31 GPIO中断
    result = rt_pin_irq_enable(SENSOR_DRDY_PIN,PIN_IRQ_DISABLE);
    if (result != RT_EOK)
    {
        return result;
    }

    irq_level = rt_hw_interrupt_disable();
    g_sensor_drdy_port.enabled = RT_FALSE;
    rt_hw_interrupt_enable(irq_level);

    return RT_EOK;
}

/*
 * 简介：查询PA31 GPIO中断是否已使能。
 * 输入：无。
 * 返回值：已使能返回RT_TRUE，否则返回RT_FALSE。
 */
rt_bool_t sensor_drdy_port_is_enabled(void)
{
    rt_bool_t enabled;
    rt_base_t irq_level;

    irq_level = rt_hw_interrupt_disable();
    enabled = g_sensor_drdy_port.enabled;
    rt_hw_interrupt_enable(irq_level);

    return enabled;
}


/*
 * 简介：读取PA31当前数字电平。
 * 输入：level_value为调用者提供的输出缓冲区。
 * 返回值：成功返回RT_EOK；空指针返回-RT_EINVAL；
 *         端口尚未初始化返回-RT_ENOSYS。
 * 说明：该结果是MCU数字输入观察值，不能代替模拟电压测量。
 */
rt_err_t sensor_drdy_port_get_level(rt_uint8_t *level_value)
{
    rt_base_t irq_level;

    if (level_value == RT_NULL)
    {
        return -RT_EINVAL;
    }

    irq_level = rt_hw_interrupt_disable();

    if (g_sensor_drdy_port.initialized == RT_FALSE)
    {
        rt_hw_interrupt_enable(irq_level);
        return -RT_ENOSYS;
    }

    rt_hw_interrupt_enable(irq_level);

    *level_value =(rt_pin_read(SENSOR_DRDY_PIN) == PIN_LOW) ?PIN_LOW : PIN_HIGH;

    return RT_EOK;
}
/*
 * 简介：复制PA31 DRDY端口累计统计。
 * 输入：stats为调用者提供的输出缓冲区。
 * 返回值：成功返回RT_EOK；空指针返回-RT_EINVAL；
 *         端口尚未初始化返回-RT_ENOSYS。
 * 说明：统计在短临界区内整体复制，不返回内部对象指针。
 */
rt_err_t sensor_drdy_port_get_stats(sensor_drdy_port_stats_t *stats)
{
    rt_base_t irq_level;

    if (stats == RT_NULL)
    {
        return -RT_EINVAL;
    }

    irq_level = rt_hw_interrupt_disable();

    if (g_sensor_drdy_port.initialized == RT_FALSE)
    {
        rt_hw_interrupt_enable(irq_level);
        return -RT_ENOSYS;
    }

    *stats = g_sensor_drdy_port.stats;

    rt_hw_interrupt_enable(irq_level);

    return RT_EOK;
}


/*
 * 简介：清零PA31 DRDY端口累计统计。
 * 输入：无。
 * 返回值：成功返回RT_EOK；端口尚未初始化返回-RT_ENOSYS。
 * 说明：只清零统计，不改变中断注册和使能状态。
 */
rt_err_t sensor_drdy_port_reset_stats(void)
{
    rt_base_t irq_level;

    irq_level = rt_hw_interrupt_disable();

    if (g_sensor_drdy_port.initialized == RT_FALSE)
    {
        rt_hw_interrupt_enable(irq_level);
        return -RT_ENOSYS;
    }

    memset(&g_sensor_drdy_port.stats,0,sizeof(g_sensor_drdy_port.stats));

    rt_hw_interrupt_enable(irq_level);

    return RT_EOK;
}
//---------------------------------------------------------------私有函数---------------------------------------------------------------
/*
 * 简介：处理PA31双边沿GPIO中断。
 * 输入：args当前未使用。
 * 返回值：无。
 * 说明：本函数运行在GPIO ISR上下文。
 *       高电平表示锁存DRDY已经断言，记录时间并通知上层；
 *       低电平表示输出数据读取后INT1已经释放，只记录统计。
 *       ISR禁止执行I2C、日志、延时、互斥锁和LVGL操作。
 */
static void sensor_drdy_port_gpio_isr(void *args)
{
    sensor_drdy_isr_handler_t handler;
    void *user_data;
    rt_tick_t now;
    rt_uint8_t level_value;
    rt_err_t notify_result;
    rt_base_t irq_level;

    (void)args;

    now = rt_tick_get();

    level_value =(rt_pin_read(SENSOR_DRDY_PIN) == PIN_LOW) ?PIN_LOW : PIN_HIGH;

    irq_level = rt_hw_interrupt_disable();

    g_sensor_drdy_port.stats.irq_count++;
    g_sensor_drdy_port.stats.last_level = level_value;

    handler = g_sensor_drdy_port.handler;
    user_data = g_sensor_drdy_port.user_data;

    if (level_value == SENSOR_DRDY_ACTIVE_LEVEL)
    {
        g_sensor_drdy_port.stats.assert_count++;
        g_sensor_drdy_port.stats.last_assert_tick = now;
    }
    else
    {
        g_sensor_drdy_port.stats.release_count++;
        g_sensor_drdy_port.stats.last_release_tick = now;
    }

    rt_hw_interrupt_enable(irq_level);

    /*
     * 只有高电平断言才通知sensor线程。
     * 低电平释放不能再次触发一次采样。
     */
    if ((level_value == SENSOR_DRDY_ACTIVE_LEVEL) &&(handler != RT_NULL))
    {
        notify_result = handler(now, user_data);

        irq_level = rt_hw_interrupt_disable();

        if (notify_result == RT_EOK)
        {
            g_sensor_drdy_port.stats.notify_success_count++;
        }
        else
        {
            g_sensor_drdy_port.stats.notify_failure_count++;
        }

        rt_hw_interrupt_enable(irq_level);
    }
}
