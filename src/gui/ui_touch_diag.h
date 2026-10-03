#ifndef UI_TOUCH_DIAG_H
#define UI_TOUCH_DIAG_H
#include <rtthread.h>
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif
#define UI_TOUCH_DIAG_TARGET_COUNT 9U

// 触摸诊断事件类型枚举
typedef enum
{
    UI_TOUCH_DIAG_EVENT_NONE = 0,
    UI_TOUCH_DIAG_EVENT_PRESSED,    // 按下
    UI_TOUCH_DIAG_EVENT_PRESSING,   // 持续按下
    UI_TOUCH_DIAG_EVENT_RELEASED,   // 松开
    UI_TOUCH_DIAG_EVENT_COUNT
} ui_touch_diag_event_t;

// 触摸目标类型枚举
typedef enum
{
    UI_TOUCH_TARGET_CENTER = 0,     // 中心
    UI_TOUCH_TARGET_TOP_LEFT,       // 左上角
    UI_TOUCH_TARGET_TOP_RIGHT,      // 右上角
    UI_TOUCH_TARGET_BOTTOM_LEFT,    // 左下角
    UI_TOUCH_TARGET_BOTTOM_RIGHT,   // 右下角
    UI_TOUCH_TARGET_TOP_EDGE,       // 上边缘
    UI_TOUCH_TARGET_BOTTOM_EDGE,    // 下边缘
    UI_TOUCH_TARGET_LEFT_EDGE,      // 左边缘
    UI_TOUCH_TARGET_RIGHT_EDGE,     // 右边缘
    UI_TOUCH_TARGET_NONE,           // 没有目标
    UI_TOUCH_TARGET_OUT_OF_RANGE,   // 超出范围
    UI_TOUCH_TARGET_COUNT
} ui_touch_target_t;

// 触摸诊断统计信息结构体
typedef struct
{
    rt_uint32_t pressed_count;      // 按下次数
    rt_uint32_t pressing_count;     // 持续按下次数
    rt_uint32_t released_count;     // 松开次数

    rt_uint32_t completed_sequence_count;       // 完整的按下-持续按下-松开序列次数
    rt_uint32_t release_without_press_count;    // 松开前没有按下的次数
    rt_uint32_t press_without_release_count;    // 按下前没有松开的次数
    rt_uint32_t out_of_range_count;             // 超出范围的次数
    rt_uint32_t none_hit_count;                 // 没有命中任何目标的次数

    rt_uint32_t target_hit_count[UI_TOUCH_DIAG_TARGET_COUNT]; // 各个目标命中次数，索引对应 ui_touch_target_t 枚举值

    rt_uint32_t event_sequence;                 // 事件序列
    rt_tick_t last_tick;                        // 上次事件的时间戳

    rt_int16_t last_x;                          // 上次触摸点的 X 坐标
    rt_int16_t last_y;                          // 上次触摸点的 Y 坐标

    ui_touch_diag_event_t last_event;           // 上次触摸事件类型
    ui_touch_target_t last_target;              // 上次触摸目标类型

    rt_bool_t created;                          // 是否已创建
    rt_bool_t enabled;                          // 是否启用
    rt_bool_t pressed_active;                   // 是否处于按下状态

    rt_uint32_t press_lost_count;               // 按下意外丢失次数

    rt_err_t last_error;                        // 上次错误码
} ui_touch_diag_stats_t;


rt_err_t ui_touch_diag_create(lv_obj_t *parent);                // 创建触摸诊断界面
rt_err_t ui_touch_diag_set_enabled(rt_bool_t enabled);          // 设置触摸诊断界面启用状态
rt_err_t ui_touch_diag_get_stats(ui_touch_diag_stats_t *stats); // 获取触摸诊断统计信息

#ifdef __cplusplus
}
#endif

#endif
