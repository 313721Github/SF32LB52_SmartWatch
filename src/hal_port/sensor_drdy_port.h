#ifndef SENSOR_DRDY_PORT_H
#define SENSOR_DRDY_PORT_H

#include "rtthread.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * DRDY处理函数运行在GPIO ISR上下文。
 *
 * 实现只允许记录标量或发送ISR安全的RT-Thread事件。
 * 禁止执行I2C、日志、延时、互斥锁和LVGL操作。
 */
typedef rt_err_t (*sensor_drdy_isr_handler_t)(rt_tick_t assert_tick,void *user_data);

//PA31 DRDY端口累计统计
typedef struct
{
    rt_uint32_t irq_count;              //PA31原始GPIO中断总数，包括断言和释放

    rt_uint32_t assert_count;           //PA31高电平DRDY断言次数

    rt_uint32_t release_count;          //读取输出数据后PA31恢复低电平的次数

    rt_uint32_t notify_success_count;   //ISR成功通知上层的次数

    rt_uint32_t notify_failure_count;   //ISR通知上层失败的次数

    rt_tick_t last_assert_tick;         //最近一次高电平DRDY断言的系统tick

    rt_tick_t last_release_tick;        //最近一次恢复低电平的系统tick

    rt_uint8_t last_level;              //最近一次GPIO中断时读取到的PA31电平
} sensor_drdy_port_stats_t;

rt_err_t sensor_drdy_port_init(sensor_drdy_isr_handler_t handler, void *user_data); //配置PA31并注册DRDY中断，默认不使能

rt_err_t sensor_drdy_port_enable(void); //使能PA31双边沿中断

rt_err_t sensor_drdy_port_disable(void); //禁用PA31中断，保留注册信息

rt_bool_t sensor_drdy_port_is_enabled(void); //查询PA31中断是否已使能

rt_err_t sensor_drdy_port_get_level(rt_uint8_t *level_value); //读取PA31当前数字电平

rt_err_t sensor_drdy_port_get_stats(sensor_drdy_port_stats_t *stats); //复制DRDY中断统计到调用者缓冲区

rt_err_t sensor_drdy_port_reset_stats(void); //清零DRDY中断统计

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_DRDY_PORT_H */