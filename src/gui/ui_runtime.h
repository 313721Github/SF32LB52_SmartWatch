#ifndef UI_RUNTIME_H
#define UI_RUNTIME_H
#include "rtthread.h"
#include "ui_event.h"
//ui运行状态
typedef enum
{
    UI_RUNTIME_STATE_STOPPED = 0,         //尚未启动
    UI_RUNTIME_STATE_STARTING,            //静态线程已经启动，但LVGL运行条件尚未全部确认
    UI_RUNTIME_STATE_READY,               //LVGL适配、显示对象、绘制缓冲和页面均建立完成
    UI_RUNTIME_STATE_FAILED,              //初始化阶段发生不可继续的错误
    UI_RUNTIME_STATE_COUNT
} ui_runtime_state_t;


//ui事件
typedef enum
{
    UI_RUNTIME_EVENT_READY = 0,
    UI_RUNTIME_EVENT_FAILED,
    UI_RUNTIME_EVENT_COUNT
} ui_runtime_event_t;

//ui运行错误字段
typedef enum
{
    UI_FAILURE_NONE = 0,            //尚未发生错误
    UI_FAILURE_QUEUE_CREATE,        //事件队列创建失败
    UI_FAILURE_THREAD_INIT,         //创建线程失败
    UI_FAILURE_THREAD_STARTUP,      //线程启动失败
    UI_FAILURE_LVGL_PORT_INIT,      //LVGL适配初始化失败
    UI_FAILURE_DEFAULT_DISPLAY,     //默认显示对象创建失败
    UI_FAILURE_DRAW_BUFFER,         //绘制缓冲创建失败
    UI_FAILURE_RESOLUTION,          //显示分辨率不符合预期
    UI_FAILURE_PAGE_CREATE,         //页面创建失败
    UI_FAILURE_HANDLER_REGISTER,    //ui回调注册失败
    UI_FAILURE_STAGE_COUNT
} ui_failure_stage_t;

//ui统计
typedef struct
{
    ui_runtime_state_t state;               //运行状态

    rt_uint32_t init_attempt;               //初始化尝试次数
    rt_uint32_t init_success;               //初始化成功次数
    rt_uint32_t init_fail;                  //初始化失败次数


    //handle统计
    rt_uint32_t handler_count;              //处理器调用次数
    rt_uint32_t last_handler_return_ms;     //lv_timer_handler返回的建议等待时间
    rt_uint32_t last_wait_ms;               //上一次处理器调用等待耗时
    rt_uint32_t min_wait_ms;                //最小等待耗时
    rt_uint32_t max_wait_ms;                //最大等待耗时

    rt_err_t last_error;                    //最后一次错误码
    ui_failure_stage_t failure_stage;       //失败阶段

    void *thread;                           //运行线程句柄
    rt_tick_t ready_tick;                   //运行线程进入READY状态的tick数

    //显示信息
    void *display;                          //默认显示对象句柄
    void *draw_buf;                         //绘制缓冲句柄
    void *buf1;                             //绘制缓冲1
    void *buf2;                             //绘制缓冲2

    rt_uint32_t draw_buf_pixels;            //绘制缓冲像素数
    rt_uint32_t draw_buf_bytes;             //绘制缓冲字节数
    rt_uint16_t hor_res;                    //水平分辨率
    rt_uint16_t ver_res;                    //垂直分辨率

    //ui事件相关统计
    void *event_queue;                      //事件队列句柄
    rt_uint16_t queue_depth;                //事件队列深度
    rt_bool_t queue_ready;                  //事件队列是否可用

    rt_uint32_t post_attempt;               //事件投递尝试次数
    rt_uint32_t posted;                     //成功进入队列的次数
    rt_uint32_t consumed;                   //成功从队列取出的次数
    rt_uint32_t dropped;                    //队列未就绪、UI未READY或队列满而未入队
    rt_uint32_t invalid;                    //事件类型或来源非法
    rt_uint32_t handled;                    //页面处理成功的次数
    rt_uint32_t handle_fail;                //页面处理入口返回失败的次数
    rt_uint32_t late;                       //消费延迟超过约定阈值的次数
    rt_uint32_t recv_fail;                  //接收失败次数，除-RT_ETIMEOUT外的接收错误次数

    rt_tick_t last_latency_ticks;           //上一次事件从投递到消费的延迟tick数（最近一条事件的排队时间）
    rt_tick_t max_latency_ticks;            //最大延迟tick数，观测到的最大排队时间

    rt_uint32_t max_inflight;               /* 最大未完成UI事件数 */

    /* 最近一条合法事件的类型和来源。 */
    ui_action_t last_action;
    ui_event_source_t last_source;

    /* 请求进入UI路径到页面处理完成的延迟。 */
    rt_tick_t last_complete_latency_ticks;
    rt_tick_t max_complete_latency_ticks;

} ui_runtime_stats_t;

//UI线程把READY或FAILED异步通知给app_main
typedef rt_err_t (*ui_runtime_event_handler_t)(
    ui_runtime_event_t evnt,
    ui_failure_stage_t stage,
    rt_err_t error,
    void *user_data);

//UI运行时初始化
rt_err_t ui_runtime_init(void);
rt_err_t ui_runtime_register_handler(ui_runtime_event_handler_t handler,void *user_data);           //回调注册函数
ui_runtime_state_t ui_runtime_get_state(void);                                                      //查询ui模块状态
rt_err_t ui_runtime_get_stats(ui_runtime_stats_t *stats);                                           //获取ui模块信息
rt_err_t ui_event_post(const ui_event_t *event);                                                    //ui事件投递入口

#endif
