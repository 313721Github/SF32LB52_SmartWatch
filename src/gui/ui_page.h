#ifndef UI_PAGE_H
#define UI_PAGE_H

#include "rtthread.h"
#include "ui_event.h"
#include "ui_notification.h"

// UI 页面 ID。枚举值用于标识页面，切换顺序由导航规则决定。
typedef enum
{
    UI_PAGE_WATCHFACE = 0,         // 表盘页，上电后默认显示
    UI_PAGE_MENU,                  // 列表菜单，上下滑动浏览，点按进入详情
    UI_PAGE_SYSTEM_INFO,           // 系统信息页，展示固件、SDK 和板型信息
    UI_PAGE_SENSOR_DIAG,           // 传感器诊断页，展示三颗传感器的数据和状态
    UI_PAGE_MOTION,                // 运动页，从表盘左滑进入，展示步数和运行状态
    UI_PAGE_RECENT_NOTIFICATIONS,  // 最近通知页，从表盘右滑进入，回看最近一条通知
    UI_PAGE_SETTINGS,              // 设置页，通过菜单进入
    UI_PAGE_COUNT                  // 页面数量，用于范围检查，不对应实际页面
} ui_page_id_t;

typedef enum
{
    UI_NOTICE_CLOSE_MANUAL = 0, // 关闭按钮或KEY1长按
    UI_NOTICE_CLOSE_TIMEOUT,   // 当前通知到期
    UI_NOTICE_CLOSE_DIAG       // 进入触摸诊断
} ui_notice_close_reason_t;
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

    rt_uint32_t clock_timer_fire_count;       // 表盘timer回调次数
    rt_uint32_t clock_hidden_skip_count;      // 未创建或隐藏时跳过次数
    rt_uint32_t clock_render_count;           // 实际修改标签文本或颜色的次数
    rt_uint32_t clock_refresh_failure_count;  // 对象、快照或转换失败次数

    rt_uint32_t notice_missing_glyph_count; // 成功文本转换中的缺字替换累计次数
    rt_uint32_t notice_shown_count;         // 从无弹层到显示的次数
    rt_uint32_t notice_replaced_count;      // 不同内容替换弹层的次数
    rt_uint32_t notice_merged_count;        // 重复内容合并次数
    rt_uint32_t notice_manual_close_count;  // 手动关闭次数
    rt_uint32_t notice_timeout_close_count; // 超时关闭次数
    rt_uint32_t notice_diag_close_count;    // 进入诊断而关闭的次数
    rt_uint32_t notice_stale_close_count;   // 迟到或重复关闭请求次数
} ui_page_stats_t;

/*
 * 传感器诊断页面刷新统计。
 *
 * 本结构只包含值类型数据，不包含LVGL对象、timer或sensor内部指针，
 * 因此可以安全复制给FinSH等诊断调用者。
 */
typedef struct
{
    rt_uint32_t timer_fire_count;          //500ms定时器触发次数
    rt_uint32_t visible_refresh_count;     //诊断页可见时的刷新尝试次数
    rt_uint32_t hidden_skip_count;         //页面隐藏或尚未创建时跳过次数

    rt_uint32_t snapshot_success_count;    //成功取得一致快照次数
    rt_uint32_t snapshot_failure_count;    //快照接口失败次数

    rt_uint32_t unchanged_skip_count;      //publish_seq未变化次数
    rt_uint32_t coalesced_publish_count;   //两次UI刷新之间合并的发布次数
    rt_uint32_t render_count;              //完成三器件文本更新次数

    rt_uint32_t last_publish_seq;          //最近一次显示的服务发布序号
    rt_err_t last_snapshot_error;          //最近一次快照读取结果

    rt_tick_t last_snapshot_copy_ticks;    //最近一次快照复制耗时
    rt_tick_t max_snapshot_copy_ticks;     //最大快照复制耗时

    rt_tick_t last_render_ticks;           //最近一次LVGL文本渲染耗时
    rt_tick_t max_render_ticks;            //最大LVGL文本渲染耗时
} ui_sensor_diag_stats_t;
typedef struct
{
    rt_bool_t visible;
    rt_bool_t has_latest;
    uint32_t display_id;
    uint32_t latest_id;
    ui_page_id_t return_page;
    rt_tick_t deadline_tick;
    rt_tick_t sample_tick;
} ui_notice_view_t;

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
typedef rt_err_t (*ui_page_action_handler_t)(ui_action_t action,rt_tick_t request_tick,rt_uint32_t value,void *user_data);
rt_err_t ui_page_register_action_handler(ui_page_action_handler_t handler,void *user_data);
rt_err_t ui_page_get_sensor_diag_stats(ui_sensor_diag_stats_t *stats);  //获取传感器诊断页面刷新统计

rt_err_t ui_page_notification_present(const ui_notice_message_t *message, ui_notice_outcome_t *outcome);

rt_err_t ui_page_notification_close(uint32_t expected_id, ui_notice_close_reason_t reason);
/* 仅UI线程在触摸诊断由关闭变为启用后调用。 */
rt_err_t ui_page_on_touch_diag_enabled(void);
rt_err_t ui_page_get_notice_view(ui_notice_view_t *view);   //复制通知显示状态，供串口诊断查询
#endif
