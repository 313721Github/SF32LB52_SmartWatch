#ifndef DISPLAY_PORT_H
#define DISPLAY_PORT_H
#include <rtthread.h>

//项目侧需要显示的display_lcd信息
typedef struct
{
    rt_uint16_t width;          //LCD的水平分辨率，单位为像素。当前预期值为 390
    rt_uint16_t height;         //LCD的垂直分辨率，单位为像素。当前预期值为 450
    rt_uint8_t pixel_format;    //LCD使用的像素格式编号，例如RGB565
    rt_uint8_t bits_per_pixel;  //每个像素占用的比特位数
    rt_uint8_t draw_align;      //驱动要求的绘制地址或像素数量对齐要求
    rt_uint8_t is_round;        //表示屏幕是否为圆形，0：非圆形 非 0：圆形 当前390×450 AMOLED预期属于矩形屏幕，因此通常应为 0
    rt_uint32_t bandwidth;      //底层图形设备报告的显示传输带宽或相关能力值
} display_port_info_t;

//项目测试统计信息
typedef struct
{
    rt_uint32_t init_attempt;   //初始化尝试次数
    rt_uint32_t init_success;   //初始化成功次数
    rt_uint32_t init_fail;      //初始化失败次数

    rt_uint32_t draw_attempt;   //绘制尝试次数
    rt_uint32_t draw_success;   //绘制成功次数
    rt_uint32_t draw_fail;      //绘制失败次数
    rt_uint32_t strip_count;    //累计处理的分条数量

    rt_uint32_t brightness_attempt; //亮度设置尝试次数
    rt_uint32_t brightness_success; //亮度设置成功次数
    rt_uint32_t brightness_fail;    //亮度设置失败次数

    rt_uint32_t power_on_attempt;   //开屏尝试次数
    rt_uint32_t power_on_success;   //开屏成功次数
    rt_uint32_t power_on_fail;      //开屏失败次数

    rt_uint32_t power_off_attempt;  //关屏尝试次数
    rt_uint32_t power_off_success;  //关屏成功次数
    rt_uint32_t power_off_fail;     //关屏失败次数

    rt_uint32_t invalid_count;  //公开接口收到非法输入或状态不允许操作的次数
    rt_uint32_t close_fail;     //关闭LCD设备失败的次数
} display_port_stats_t;


//项目侧状态枚举
typedef enum
{
    DISPLAY_PORT_STATE_CLOSED=0,        //设备没有由display_port保持打开
    DISPLAY_PORT_STATE_INITIALIZED,    //设备已经初始化，但尚未通过当前测试重绘确认显示为ON
    DISPLAY_PORT_STATE_ON,             //显示开启，可以执行绘制和亮度控制
    DISPLAY_PORT_STATE_OFF             //已经执行POWEROFF，禁止绘制和设置亮度
}display_port_state_t;

//控制状态查询信息
typedef struct
{
    display_port_state_t display_state;
    rt_uint8_t last_brightness;
    rt_bool_t brightness_valid;
} display_port_control_state_t;

//查找、打开并验证lcd设备，设置RGB565格式。
rt_err_t  display_port_init(void);
//矩形同步填充函数
rt_err_t display_port_fill_rect(rt_int16_t x0,rt_int16_t y0,rt_int16_t x1,rt_int16_t y1,rt_uint16_t color);
//冒烟测试
rt_err_t display_port_run_smoke_test(void);
//关闭函数
rt_err_t display_port_deinit(void);

rt_err_t display_port_get_info(display_port_info_t *info);
rt_err_t display_port_get_stats(display_port_stats_t *stats);

//设置屏幕亮度值
rt_err_t display_port_set_brightness(rt_uint8_t percent);
//关屏
rt_err_t display_port_power_off(void);
//开屏
rt_err_t display_port_power_on(void);
//查询当前状态
rt_err_t display_port_get_control_state(display_port_control_state_t *state);
//运行控制测试
rt_err_t display_port_run_control_test(void);

#endif
