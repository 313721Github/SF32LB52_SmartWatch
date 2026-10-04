#ifndef UI_PAGE_H
#define UI_PAGE_H

#include "rtthread.h"
#include "ui_event.h"

//ui页面ID
typedef enum
{
    UI_PAGE_WATCHFACE = 0,      //表盘页面
    UI_PAGE_MENU,               //菜单页面
    UI_PAGE_SYSTEM_INFO,        //系统信息页面
    UI_PAGE_COUNT
}ui_page_id_t;

//ui页面统计信息
typedef struct
{
    rt_bool_t created;              //ui界面是否创建
    ui_page_id_t current_page;      //当前页面

    rt_uint32_t create_attempt;     //页面创建尝试次数
    rt_uint32_t create_success;     //页面创建成功次数
    rt_uint32_t create_fail;        //页面创建失败次数

    rt_uint32_t switch_attempt;     //页面切换尝试次数
    rt_uint32_t switch_success;     //页面切换成功次数
    rt_uint32_t switch_fail;        //页面切换失败次数
    rt_uint32_t switch_noop;        //页面切换无操作次数

    rt_uint32_t touch_action_attempt;   //触摸动作尝试次数
    rt_uint32_t touch_action_success;   //触摸动作成功次数
    rt_uint32_t touch_action_fail;      //触摸动作失败次数
} ui_page_stats_t;

//创建ui界面
rt_err_t ui_page_create(void);
//ui事件处理函数
rt_err_t ui_page_handle_event(const ui_event_t *event);
//获取ui界面统计信息
rt_err_t ui_page_get_stats(ui_page_stats_t *stats);

/*
 * 页面触摸对象只产生统一UI语义。
 * ui_page不直接依赖ui_runtime，也不直接访问UI消息队列。
 */
typedef rt_err_t (*ui_page_action_handler_t)(ui_action_t action,rt_tick_t request_tick,void *user_data);
rt_err_t ui_page_register_action_handler(ui_page_action_handler_t handler,void *user_data);
#endif
