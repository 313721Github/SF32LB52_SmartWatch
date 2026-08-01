#include "indicator_port.h"
#include "stdint.h"

#include "bf0_hal.h"
#include "drivers/rt_drv_pwm.h"
#include "drv_rgbled.h"

rt_err_t indicator_init(void);                              //RGB信号指示初始化
rt_err_t indicator_set_state(indicator_state_t state);      //设置RGB信号指示状态
static rt_err_t indicator_write_color(rt_uint32_t color);   //内部私有设置RGB颜色函数


static rt_device_t g_rgbled_device = RT_NULL;                   //RGB LED设备句柄
static indicator_state_t g_current_state = INDICATOR_STATE_OFF; //RGB状态表
static const rt_uint32_t g_indicator_color_table[INDICATOR_STATE_COUNT] =
{
    [INDICATOR_STATE_OFF]      = RGB_COLOR_BLACK,
    [INDICATOR_STATE_BOOTING]  = RGB_COLOR_BLUE,
    [INDICATOR_STATE_READY]    = RGB_COLOR_GREEN,
    [INDICATOR_STATE_DEGRADED]  = RGB_COLOR_YELLOW,
    [INDICATOR_STATE_FATAL]    = RGB_COLOR_RED
};//RGB状态表对应的颜色值

 //内部私有设置RGB颜色函数
 //输入：color RGB颜色值
 //返回：RT_EOK表示成功，负错误码表示失败
static rt_err_t indicator_write_color(rt_uint32_t color)
{
    struct rt_rgbled_configuration config={0};
    if(g_rgbled_device==RT_NULL)
    {
        rt_kprintf("[RGB][ERR] rgb device is null\n");
        return -RT_ERROR;
    }

    config.color_rgb=color;
    return rt_device_control(g_rgbled_device,PWM_CMD_SET_COLOR,&config);

}


//初始化RGB硬件路径
//输入：无
//返回：RT_EOK表示成功，负错误码表示失败
rt_err_t indicator_init(void)
{
    
    rt_err_t result;

     /* 1. PA32切换为PWM输出功能 */
    HAL_PIN_Set(PAD_PA32, GPTIM2_CH1, PIN_NOPULL, 1); 
     /* 2. 打开外围3.3V LDO，为RGB灯珠供电 */
    HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO3_3V3, true, true);
    /* 等待短暂的电源稳定时间 */
    rt_thread_mdelay(10);

    /* 4. 查找官方rgbled设备 */
    g_rgbled_device = rt_device_find("rgbled");
    if(g_rgbled_device==RT_NULL)
    {
        rt_kprintf("[RGB][ERR] failed to find rgbled device\n");
        return -RT_ERROR;
    }
    else
    {
        rt_kprintf("[RGB][OK] rgbled device found\n");
    }
    
    result = indicator_write_color(RGB_COLOR_BLACK);
    if (result != RT_EOK)
    {
        rt_kprintf("[RGB][ERR] failed to turn indicator off, result=%d\n",
                (int)result);
        return result;
    }

    g_current_state = INDICATOR_STATE_OFF;
    rt_kprintf("[RGB][OK] indicator initialized\n");
    return RT_EOK;

}

//设置RGB信号指示状态
rt_err_t indicator_set_state(indicator_state_t state)
{
    rt_err_t result;
    if(state<INDICATOR_STATE_OFF || state>=INDICATOR_STATE_COUNT)
    {
        rt_kprintf("[RGB][ERR] invalid indicator state\n");
        return -RT_EINVAL;
    }
  
    result= indicator_write_color(g_indicator_color_table[state]);
    if (result == RT_EOK)
    {
        g_current_state = state;
    }

    return result;
}   
