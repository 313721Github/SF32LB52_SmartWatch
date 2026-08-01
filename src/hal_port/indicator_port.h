#ifndef INDICATOR_PORT_H
#define INDICATOR_PORT_H

//应用层只表达“系统现在是什么状态”，由indicator模块决定RGB显示什么颜色，并隐藏PA32、PWM3、DMA和WS2812细节
#include "rtthread.h"


//RGB状态表
typedef enum
{
    INDICATOR_STATE_OFF = 0,
    INDICATOR_STATE_BOOTING,
    INDICATOR_STATE_READY,
    INDICATOR_STATE_DEGRADED,
    INDICATOR_STATE_FATAL,
    INDICATOR_STATE_COUNT

}indicator_state_t;
/**
 * 初始化板载状态指示灯。
 *
 * 成功：返回RT_EOK，并将指示灯设置为关闭。
 * 失败：返回负错误码，系统其他功能可以继续运行。
 *
 * 只能在线程上下文调用。
 */
rt_err_t indicator_init(void);
/**
 * 设置系统指示状态。
 *
 * state必须是有效的indicator_state_t。
 * 模块未初始化或底层控制失败时返回负错误码。
 *
 * 只能在线程上下文调用，不在ISR中直接调用。
 */
rt_err_t indicator_set_state(indicator_state_t state);


#endif // INDICATOR_PORT_H