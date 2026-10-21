#include "app_main.h"
#include "indicator_port.h"
#include "button_port.h"
#include "rthw.h"
#include "ui_runtime.h"
#include "ui_event.h"
#include "sensor_service.h"

#define APP_EVENT_QUEUE_DEPTH  8U
#define APP_THREAD_STACK_SIZE  2048U
#define APP_THREAD_PRIORITY    18U
#define APP_THREAD_TIMESLICE   10U

// Event stress test parameters
#define APP_EVENT_STRESS_ATTEMPTS   (APP_EVENT_QUEUE_DEPTH * 2U)
#define APP_EVENT_STRESS_TIMEOUT_MS 1000U
#define APP_EVENT_STRESS_POLL_MS    10U
#define APP_EVENT_RECOVERY_VALUE    0xA1100001U

static rt_mq_t g_event_queue=RT_NULL;
static rt_thread_t g_app_thread=RT_NULL;

static rt_uint32_t g_event_post_count;
static rt_uint32_t g_event_drop_count;
static rt_uint32_t g_consume_count;             //接收队列消息数量
static rt_uint32_t g_event_post_attempt_count;  //尝试投递事件次数
static rt_uint32_t g_event_invalid_count;       //无效事件数量
static rt_uint32_t g_event_max_inflight;        //最大在途事件数

static rt_bool_t g_indicator_available=RT_FALSE;                    // 硬件能不能控制
static indicator_state_t g_app_indicator_state=INDICATOR_STATE_OFF; // 当前显示什么状态

static rt_bool_t g_button_available = RT_FALSE;                     //按键初始化完成
static rt_bool_t g_app_initialized = RT_FALSE;                      //应用核心初始化成功
static rt_bool_t g_ui_available = RT_FALSE;                         //ui模块是否初始化完成

rt_err_t app_main_init(void);
rt_err_t app_event_post(const app_event_t *event);
static rt_err_t app_ui_event_handler(ui_runtime_event_t event,ui_failure_stage_t stage,rt_err_t error,void *user_data);//ui层回调
static rt_err_t app_button_event_handler(const button_event_t *button_event,void *user_data);//应用层按键回调
static void app_thread_entry(void *parameter);          //g_app_thread线程入口,接收mq
static void app_handle_event(const app_event_t *event); //接收消息后的处理函数
static rt_err_t app_indicator_selftest(void);            //RGB自检
static rt_err_t indicator_selftest(void);                //向队列投递RGB自检
static rt_err_t app_request_ui_action(ui_action_t action,ui_event_source_t source,rt_uint32_t value);    //构造局部ui_event_t

static void app_record_post_attempt(void);
static void app_record_post_success(void);
static void app_record_post_drop(void);
static void app_record_invalid(void);
static void app_record_consumed(void);

//shell测试
static rt_err_t app_diag_request(void);
static int app_stats(void);
static int task11_resources(void);
static int app_event_flood(void);

//初始化
rt_err_t app_main_init(void)
{
    rt_err_t result;

    if (g_app_initialized == RT_TRUE)
    {
        rt_kprintf("[APP][INIT][WARN] application already initialized\n");
        return RT_EOK;
    }

    g_button_available = RT_FALSE;
    g_indicator_available = RT_FALSE;
    g_app_indicator_state = INDICATOR_STATE_OFF;

    rt_kprintf("[APP][INIT] start\n");

    /*
     * 1. 先创建队列。
     * 每条消息大小为sizeof(app_event_t)，最多存放APP_EVENT_QUEUE_DEPTH条消息。
     */
    g_event_queue=rt_mq_create(
        "app_evt",
        sizeof(app_event_t),
        APP_EVENT_QUEUE_DEPTH,
        RT_IPC_FLAG_FIFO);
    if(g_event_queue==RT_NULL)
    {
        rt_kprintf("[APP][INIT][ERROR] event queue create failed\n");
        return -RT_ENOMEM;
    }
    /*
     * 2. 初始化RGB指示模块。
     * RGB失败不阻止系统继续启动，但要保存错误日志。
     */
    result=indicator_init();
    if (result != RT_EOK)
    {
        rt_kprintf("[APP][INIT][WARN] indicator init failed: %d\n",
                   result);
        g_indicator_available = RT_FALSE;
    }
    else
    {
        g_indicator_available = RT_TRUE;

    }
    /*
     * 3. 创建应用线程。
     * 队列必须在线程和按键初始化之前创建。
     */
    g_app_thread=rt_thread_create(
        "app",
        app_thread_entry,
        RT_NULL,
        APP_THREAD_STACK_SIZE,
        APP_THREAD_PRIORITY,
        APP_THREAD_TIMESLICE
    );
    if (g_app_thread == RT_NULL)
    {
        rt_kprintf("[APP][INIT][ERROR] app thread create failed\n");

        rt_mq_delete(g_event_queue);
        g_event_queue = RT_NULL;

        return -RT_ENOMEM;
    }

    result=rt_thread_startup(g_app_thread);
    if (result != RT_EOK)
    {
        rt_kprintf("[APP][INIT][ERROR] app thread startup failed: %d\n",
                   result);

        rt_thread_delete(g_app_thread);
        g_app_thread = RT_NULL;

        rt_mq_delete(g_event_queue);
        g_event_queue = RT_NULL;

        return result;
    }

    /*
     * 4. 最后初始化按键。
     * 此时队列和消费者线程都已经准备完成。
     */
    result = button_port_register_handler(app_button_event_handler, RT_NULL);
    if (result != RT_EOK)
    {
        g_button_available=RT_FALSE;
        rt_kprintf("[APP][INIT][WARN] button handler register failed: %d\n",
                result);
    }
    else
    {
        result = button_port_init();
        if (result != RT_EOK)
        {
            g_button_available=RT_FALSE;
            rt_kprintf("[APP][INIT][WARN] button init failed: %d\n",
                    result);
        }
        else
        {
            g_button_available=RT_TRUE;
        }
    }

    g_app_initialized=RT_TRUE;
    if ((g_indicator_available == RT_TRUE) &&
        (g_button_available == RT_TRUE))
    {
        rt_kprintf("[APP][INIT][OK] application core ready, starting UI\n");
    }
    else
    {
        rt_kprintf("[APP][INIT][WARN] "
                "application ready in degraded mode "
                "button=%u indicator=%u\n",
                (unsigned int)g_button_available,
                (unsigned int)g_indicator_available);
    }

    //ui模块初始化
    app_event_t ui_event;
    rt_err_t post_result;
    g_ui_available = RT_FALSE;

    result = ui_runtime_register_handler(app_ui_event_handler,RT_NULL);

    if (result != RT_EOK)
    {
        /*
        * TODO：
        * 打印注册失败。
        * 因为回调没有注册成功，这种情况无法由ui_runtime回调通知。
        * 由app_main主动向app_evt投递一次UI_FAILED。
        * 不要直接在这里宣称系统READY。
        */
        rt_kprintf("[APP][ui][WARN] ui handler register failed: %d\n",result);
        ui_event.source=APP_EVENT_SOURCE_UI;
        ui_event.tick=rt_tick_get();
        ui_event.type=APP_EVENT_UI_FAILED;
        ui_event.value=UI_FAILURE_HANDLER_REGISTER;
        post_result= app_event_post(&ui_event);
        if(post_result!=RT_EOK)
        {
            rt_kprintf("[APP][ui][WARN] ui handler register Msg post failed: %d\n",post_result);
        }
    }
    else
    {
        result = ui_runtime_init();

        if (result != RT_EOK)
        {
            /*
            * 只打印启动失败。
            * ui_runtime_init已经通过已注册回调发送UI_FAILED，
            * 这里不要重复投递。
            */
            rt_kprintf(
            "[APP][UI][ERROR] runtime start failed: %d\n",
            result);
        }
    }
    /*
     * 启动统一sensor服务。
     *
     * 设计边界：
     * 1. app层只启动服务，不访问I2C或具体器件驱动；
     * 2. RT_EOK只表示sensor线程和服务资源已经启动，
     *    不表示三颗器件均已READY或已有有效样本；
     * 3. sensor启动失败属于降级运行，不能破坏已经建立的
     *    app队列、按键、UI线程和触摸功能；
     * 4. UI诊断页会在500ms刷新中显示WAIT或快照错误；
     * 5. sensor_service_init()具有幂等语义，服务已经运行时
     *    再次调用仍返回RT_EOK，不会创建第二个sensor线程。
     */
    result = sensor_service_init();

    if (result != RT_EOK)
    {
        rt_kprintf(
            "[APP][SENSOR][WARN] "
            "service start failed: %d, "
            "application remains available\n",
            (int)result);
    }
    else
    {
        rt_kprintf(
            "[APP][SENSOR][OK] "
            "service thread started, "
            "device readiness is asynchronous\n");
    }

    return RT_EOK;

}


//应用层按键回调
static rt_err_t app_button_event_handler(const button_event_t *button_event,void *user_data)
{
    app_event_t app_event={0};
    (void)user_data;

    if (button_event == RT_NULL)
    {
        return -RT_EINVAL;
    }


    app_event.value=button_event->duration_ms;

    if(button_event->key==BUTTON_PORT_KEY1)
    {
        app_event.source=APP_EVENT_SOURCE_KEY1;
        app_event.tick=button_event->tick;
        if(button_event->type==BUTTON_EVENT_SHORT)
        {
            app_event.type=APP_EVENT_KEY_SHORT;
        }
        else if(button_event->type==BUTTON_EVENT_LONG)
        {
            app_event.type=APP_EVENT_KEY_LONG;

        }
        else if(button_event->type==BUTTON_EVENT_DOWN)
        {
            app_event.type=APP_EVENT_KEY_DOWN;
        }
        else if(button_event->type==BUTTON_EVENT_UP)
        {
            app_event.type=APP_EVENT_KEY_UP;
        }
        else
        {
            return RT_EOK;
        }


    }
    else if(button_event->key==BUTTON_PORT_KEY2)
    {
         app_event.source=APP_EVENT_SOURCE_KEY2;
         app_event.tick=button_event->tick;
        if(button_event->type==BUTTON_EVENT_SHORT)
        {
            app_event.type=APP_EVENT_KEY_SHORT;
        }
        else if(button_event->type==BUTTON_EVENT_LONG)
        {
            app_event.type=APP_EVENT_KEY_LONG;

        }
        else if(button_event->type==BUTTON_EVENT_DOWN)
        {
            app_event.type=APP_EVENT_KEY_DOWN;
        }
        else if(button_event->type==BUTTON_EVENT_UP)
        {
            app_event.type=APP_EVENT_KEY_UP;
        }
        else
        {
            return RT_EOK;
        }


    }
    else
    {
        return RT_EOK;
    }

    return app_event_post(&app_event);




}

//ui层回调函数
static rt_err_t app_ui_event_handler(ui_runtime_event_t event,ui_failure_stage_t stage,rt_err_t error,void *user_data)
{
    app_event_t app_event = {0};

    (void)error;
    (void)user_data;

    app_event.source = APP_EVENT_SOURCE_UI;
    app_event.tick = rt_tick_get();

    if (event == UI_RUNTIME_EVENT_READY)
    {
        app_event.type = APP_EVENT_UI_READY;
        app_event.value = 0U;
    }
    else if (event == UI_RUNTIME_EVENT_FAILED)
    {
        app_event.type = APP_EVENT_UI_FAILED;
        app_event.value = (rt_uint32_t)stage;
    }
    else
    {
        return -RT_EINVAL;
    }

    return app_event_post(&app_event);
}


//消息队列投递
rt_err_t app_event_post(const app_event_t *event)
{
    rt_err_t result;
    app_event_t event_copy;

    /* 防止调用者传入空指针 */
    if(event==RT_NULL)
    {
        return -RT_EINVAL;
    }


    app_record_post_attempt();

    if ((event->source >= APP_EVENT_SOURCE_COUNT) ||
    (event->type <= APP_EVENT_NONE) ||
    (event->type >= APP_EVENT_TYPE_COUNT))
    {
        app_record_invalid();
        return -RT_EINVAL;
    }

    /* 队列尚未创建，说明应用层没有完成初始化 */
    if(g_event_queue==RT_NULL)
    {
        app_record_post_drop();
        return -RT_ERROR;
    }
    /*
     * rt_mq_send()不等待空闲空间。
     * 队列已满时立即返回错误，适合按键回调或定时器回调。
     */
    event_copy=*event;

    /*
     * 重要：正常业务发送必须保持非阻塞。
     * 队列满时安全丢弃，不重试、不覆盖旧消息。
     */
    result =rt_mq_send(
        g_event_queue,
        &event_copy,
        sizeof(app_event_t)
    );
    if(result != RT_EOK)
    {
        app_record_post_drop();
        return result;

    }

    app_record_post_success();
    return RT_EOK;

}


//g_app_thread线程入口,接收mq
static void app_thread_entry(void *parameter)
{
    app_event_t event;
    rt_err_t result;

    (void)parameter;

    rt_kprintf("[APP][THREAD] started\n");

    while (1)
    {
        /* code */
        result=rt_mq_recv(
            g_event_queue,
            &event,
            sizeof(event),
            RT_WAITING_FOREVER
        );
        if(result==RT_EOK)
        {
            app_record_consumed();
            rt_kprintf("[APP][EVENT] seq=%u type=%d source=%d "
           "tick=%u value=%u\n",
           (unsigned int)g_consume_count,
           (int)event.type,
           (int)event.source,
           (unsigned int)event.tick,
           (unsigned int)event.value);
            app_handle_event(&event);
        }
        else
        {
            rt_kprintf("[APP][QUEUE][ERROR] receive failed: %d\n",
                       result);

        }

    }


}


//接收消息后的处理函数
static void app_handle_event(const app_event_t *event)
{

    indicator_state_t new_state;
    rt_err_t set_state_result;
    rt_err_t ui_result;
    if(event==RT_NULL)
    {
        return;
    }

    switch (event->type)
    {
        case APP_EVENT_KEY_SHORT:
            if(event->source==APP_EVENT_SOURCE_KEY1)
            {
                /*
                * KEY1保留HOME/PWRKEY语义。
                * 当前只记录，不控制RGB。
                * app线程只发布UI语义事件，不直接操作LVGL对象
                */
                rt_kprintf("[APP][KEY1] HOME short press\n");
                //进行ui动作
                ui_result=app_request_ui_action(UI_ACTION_HOME,UI_EVENT_SOURCE_KEY,0u);
                if(ui_result!=RT_EOK)
                {
                    rt_kprintf("[APP][KEY1][WARN] ui action failed:%d\n",(int)ui_result);
                }
                break;
            }
            else if(event->source==APP_EVENT_SOURCE_KEY2)
            {
                if (g_app_indicator_state == INDICATOR_STATE_OFF)
                {
                    new_state = INDICATOR_STATE_READY;
                }
                else
                {
                    new_state = INDICATOR_STATE_OFF;
                }
                if (g_indicator_available == RT_TRUE)
                {
                    set_state_result=indicator_set_state(new_state);
                    if ( set_state_result== RT_EOK)
                    {
                        g_app_indicator_state = new_state;
                    }
                    else
                    {
                        rt_kprintf("[APP][KEY2][WARN] indicator_set_state failed:%d goal:%d\n",set_state_result,new_state);
                    }
                }
                else
                {
                    rt_kprintf("[APP][INDICATOR][WARN] device unavailable\n");
                }
                /*
                 * RGB业务成功、失败或不可用，都不阻止UI请求。
                 * 两条业务链相互独立。
                 */
                //进行ui动作
                ui_result=app_request_ui_action(UI_ACTION_NEXT,UI_EVENT_SOURCE_KEY,0U);
                if(ui_result!=RT_EOK)
                {
                    rt_kprintf("[APP][KEY2][WARN] ui action failed:%d\n",ui_result);
                }


                break;
            }
            else
            {
                /*
                 * APP_EVENT_KEY_SHORT只接受KEY1或KEY2来源。
                 * 不向UI队列发送来源不明的动作。
                 */
                rt_kprintf(
                    "[APP][KEY][WARN] "
                    "short event has invalid source: %d\n",
                    (int)event->source);
            }

            break;
        case APP_EVENT_KEY_LONG:
            if(event->source==APP_EVENT_SOURCE_KEY1)
            {
                rt_kprintf("[APP][KEY1] long press -> UI BACK\n");
                ui_result=app_request_ui_action(UI_ACTION_BACK,UI_EVENT_SOURCE_KEY,0U);
                if(ui_result!=RT_EOK)
                {
                    rt_kprintf("[APP][KEY1][WARN] ui action failed:%d\n",ui_result);
                }
                break;
            }
            else if(event->source==APP_EVENT_SOURCE_KEY2)
            {
                if (g_indicator_available == RT_TRUE)
                {
                    set_state_result=indicator_set_state(INDICATOR_STATE_OFF);
                    if ( set_state_result== RT_EOK)
                    {
                        g_app_indicator_state = INDICATOR_STATE_OFF;
                    }
                    else
                    {
                        rt_kprintf("[APP][KEY2][WARN] indicator_set_state failed:%d goal:%d\n",set_state_result,INDICATOR_STATE_OFF);

                    }
                }
                break;
            }
            break;
        case APP_EVENT_DIAG_REQUEST:
            rt_kprintf("[APP][DIAG] posted=%u dropped=%u\n",
                    g_event_post_count,
                    g_event_drop_count);
            break;
        case APP_EVENT_INDICATOR_SELF_TEST:
            app_indicator_selftest();
            break;
        case APP_EVENT_UI_READY:
            /*
            * TODO：
            * g_ui_available = RT_TRUE；打印UI ready；
            * indicator可用时设置READY；
            * 成功后更新g_app_indicator_state。
            */
           g_ui_available=RT_TRUE;
           rt_kprintf("[APP][UI][READY] runtime available\n");
           if(g_indicator_available == RT_TRUE)
           {
                set_state_result=indicator_set_state(INDICATOR_STATE_READY);
                if ( set_state_result== RT_EOK)
                {
                    g_app_indicator_state = INDICATOR_STATE_READY;
                }
                else
                {
                    rt_kprintf("[APP][UI][WARN] indicator_set_state failed:%d goal:%d\n",set_state_result,INDICATOR_STATE_READY);
                }
           }

            break;

        case APP_EVENT_UI_FAILED:
            /*
            * TODO：
            * g_ui_available = RT_FALSE；
            * 打印event->value中的失败阶段；
            * indicator可用时设置DEGRADED；
            * 成功后更新g_app_indicator_state。
            */
           g_ui_available=RT_FALSE;
           rt_kprintf("[APP][UI] UI failed; failed stage:%u\n", (unsigned int)event->value);
           if(g_indicator_available == RT_TRUE)
           {
            set_state_result=indicator_set_state(INDICATOR_STATE_DEGRADED);
            if (set_state_result== RT_EOK)
            {
                g_app_indicator_state = INDICATOR_STATE_DEGRADED;
            }
            else
            {
                 rt_kprintf("[APP][UI][WARN] indicator_set_state failed:%d goal:%d\n",set_state_result,INDICATOR_STATE_DEGRADED);

            }
           }
            break;

        default:
            /* DOWN和UP现阶段可以只记录，不产生业务动作 */
            break;
    }



}

//RGB自检
static rt_err_t app_indicator_selftest(void)
{
    indicator_state_t state;
    indicator_state_t saved_state;
    rt_err_t result;
    rt_err_t restore_result;

    if (g_indicator_available != RT_TRUE)
    {
        rt_kprintf(
            "[APP][SELFTEST][ERROR] indicator unavailable\n");
        return -RT_ERROR;
    }

    saved_state = g_app_indicator_state;

    rt_kprintf(
        "[APP][SELFTEST] indicator test start, saved_state=%d\n",
        (int)saved_state);

    result = RT_EOK;

    for (state = INDICATOR_STATE_BOOTING;
         state < INDICATOR_STATE_COUNT;
         state++)
    {
        rt_kprintf(
            "[APP][SELFTEST] set state=%d\n",
            (int)state);

        result = indicator_set_state(state);
        if (result != RT_EOK)
        {
            rt_kprintf(
                "[APP][SELFTEST][ERROR] "
                "set state=%d failed: %d\n",
                (int)state,
                (int)result);
            break;
        }

        rt_thread_mdelay(100);
    }

    /*
     * 自检状态只是临时显示。
     * 无论自检成功还是失败，都尝试恢复应用状态。
     */
    restore_result = indicator_set_state(saved_state);
    if (restore_result != RT_EOK)
    {
        rt_kprintf(
            "[APP][SELFTEST][ERROR] "
            "restore state=%d failed: %d\n",
            (int)saved_state,
            (int)restore_result);

        g_indicator_available = RT_FALSE;
        return restore_result;
    }

    rt_kprintf(
        "[APP][SELFTEST][OK] restored state=%d\n",
        (int)saved_state);

    return result;
}

//-----------------------------------------------------shell测试-----------------------------------------------------

static rt_err_t app_diag_request(void)
{
    app_event_t event=
    {
        .type   = APP_EVENT_DIAG_REQUEST,
        .source = APP_EVENT_SOURCE_SYSTEM,
        .tick   = rt_tick_get(),
        .value  = 0
    };

    return app_event_post(&event);

}


static int app_stats(void)
{
    rt_bool_t initialized;
    rt_bool_t queue_ready;
    rt_bool_t thread_ready;
    rt_bool_t button_available;
    rt_bool_t indicator_available;
    rt_bool_t ui_available;

    indicator_state_t indicator_state;

    rt_uint32_t posted;
    rt_uint32_t consumed;
    rt_uint32_t dropped;

    rt_uint32_t post_attempt;
    rt_uint32_t invalid;
    rt_uint32_t max_inflight;
    rt_uint32_t inflight;

    rt_bool_t sensor_running;

    rt_base_t level;

    //短暂关闭中断取得快照
    level = rt_hw_interrupt_disable();

    initialized = g_app_initialized;
    queue_ready = (g_event_queue != RT_NULL) ?
                  RT_TRUE : RT_FALSE;
    thread_ready = (g_app_thread != RT_NULL) ?
                   RT_TRUE : RT_FALSE;

    button_available = g_button_available;
    indicator_available = g_indicator_available;
    indicator_state = g_app_indicator_state;
    ui_available=g_ui_available;

    post_attempt = g_event_post_attempt_count;
    invalid = g_event_invalid_count;
    max_inflight = g_event_max_inflight;

    posted = g_event_post_count;
    consumed = g_consume_count;
    dropped = g_event_drop_count;

    inflight = posted - consumed;

    rt_hw_interrupt_enable(level);

    //sensor_running只表示统一服务线程处于RUNNING，不表示IMU、MMC和ALS全部READY
    sensor_running =sensor_service_is_running();

    rt_kprintf(
        "[APP][STATS] initialized=%u queue=%u thread=%u\n",
        (unsigned int)initialized,
        (unsigned int)queue_ready,
        (unsigned int)thread_ready);

    rt_kprintf(
        "[APP][STATS] "
        "button=%u indicator=%u ui=%u sensor=%u "
        "indicator_state=%d\n",
        (unsigned int)button_available,
        (unsigned int)indicator_available,
        (unsigned int)ui_available,
        (unsigned int)sensor_running,
        (int)indicator_state);

    rt_kprintf(
        "[APP][QUEUE] attempt=%u posted=%u consumed=%u "
        "dropped=%u invalid=%u inflight=%u max_inflight=%u\n",
        (unsigned int)post_attempt,
        (unsigned int)posted,
        (unsigned int)consumed,
        (unsigned int)dropped,
        (unsigned int)invalid,
        (unsigned int)inflight,
        (unsigned int)max_inflight);

    return 0;
}

/*
 * 简介：输出任务11资源快照
 *
 * 输出：
 * 1. RT-Thread堆当前使用量和历史峰值；
 * 2. app/UI队列累计统计和当前inflight；
 * 3. app/UI线程句柄和配置栈大小；
 * 4. UI排队延迟和动作完成延迟；
 * 5. 提醒使用list_thread读取真实栈高水位。
 *
 * 本函数只读取项目状态，不调用任何LVGL对象API。
 */
static int task11_resources(void)
{
    ui_runtime_stats_t ui_stats_snapshot;
    rt_thread_t ui_thread;
    rt_err_t result;
    rt_base_t level;

    rt_uint32_t heap_total;
    rt_uint32_t heap_used;
    rt_uint32_t heap_max_used;
    rt_uint32_t heap_free;

    rt_uint32_t app_posted;
    rt_uint32_t app_consumed;
    rt_uint32_t app_dropped;
    rt_uint32_t app_invalid;
    rt_uint32_t app_max_inflight;
    rt_uint32_t app_inflight;

    rt_uint32_t ui_inflight;
    rt_uint32_t app_stack_size;
    rt_uint32_t ui_stack_size;

    /*
     * rt_memory_info()由RT-Thread堆实现提供同步保护。
     * max_used是启动以来历史峰值，不会在释放内存后下降。
     */
    rt_memory_info(
        &heap_total,
        &heap_used,
        &heap_max_used);

    heap_free = heap_total - heap_used;

    result = ui_runtime_get_stats(&ui_stats_snapshot);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[TASK11][RESOURCE][ERROR] "
            "get UI stats failed: %d\n",
            (int)result);

        return (int)result;
    }

    /*
     * 只在短临界区中复制共享计数和线程字段。
     * 临界区内不打印、不访问LVGL。
     */
    level = rt_hw_interrupt_disable();

    app_posted = g_event_post_count;
    app_consumed = g_consume_count;
    app_dropped = g_event_drop_count;
    app_invalid = g_event_invalid_count;
    app_max_inflight = g_event_max_inflight;

    app_stack_size =
        (g_app_thread != RT_NULL) ?
        g_app_thread->stack_size : 0U;

    ui_thread =
        (rt_thread_t)ui_stats_snapshot.thread;

    ui_stack_size =
        (ui_thread != RT_NULL) ?
        ui_thread->stack_size : 0U;

    rt_hw_interrupt_enable(level);

    app_inflight = app_posted - app_consumed;
    ui_inflight =
        ui_stats_snapshot.posted -
        ui_stats_snapshot.consumed;

    rt_kprintf(
        "[TASK11][HEAP] total=%u used=%u free=%u max_used=%u\n",
        (unsigned int)heap_total,
        (unsigned int)heap_used,
        (unsigned int)heap_free,
        (unsigned int)heap_max_used);

    rt_kprintf(
        "[TASK11][APP_QUEUE] posted=%u consumed=%u "
        "dropped=%u invalid=%u inflight=%u max_inflight=%u\n",
        (unsigned int)app_posted,
        (unsigned int)app_consumed,
        (unsigned int)app_dropped,
        (unsigned int)app_invalid,
        (unsigned int)app_inflight,
        (unsigned int)app_max_inflight);

    rt_kprintf(
        "[TASK11][UI_QUEUE] posted=%u consumed=%u "
        "dropped=%u invalid=%u inflight=%u max_inflight=%u\n",
        (unsigned int)ui_stats_snapshot.posted,
        (unsigned int)ui_stats_snapshot.consumed,
        (unsigned int)ui_stats_snapshot.dropped,
        (unsigned int)ui_stats_snapshot.invalid,
        (unsigned int)ui_inflight,
        (unsigned int)ui_stats_snapshot.max_inflight);

    rt_kprintf(
        "[TASK11][LATENCY] tick_hz=%u "
        "queue_last=%u queue_max=%u "
        "complete_last=%u complete_max=%u\n",
        (unsigned int)RT_TICK_PER_SECOND,
        (unsigned int)ui_stats_snapshot.last_latency_ticks,
        (unsigned int)ui_stats_snapshot.max_latency_ticks,
        (unsigned int)
            ui_stats_snapshot.last_complete_latency_ticks,
        (unsigned int)
            ui_stats_snapshot.max_complete_latency_ticks);

    rt_kprintf(
        "[TASK11][THREAD] app=%p stack=%u "
        "ui=%p stack=%u\n",
        g_app_thread,
        (unsigned int)app_stack_size,
        ui_thread,
        (unsigned int)ui_stack_size);

    /*
     * max used需要使用RT-Thread现有高水位扫描结果。
     * 紧接着执行list_thread，并保存app和ui两行。
     */
    rt_kprintf(
        "[TASK11][THREAD] run list_thread "
        "and record app/ui max used\n");

    return 0;
}
/*
 * 简介：受控验证app消息队列满路径、排空能力和压力后恢复能力
 *
 * 测试步骤：
 * 1. 要求测试开始前app队列为空；
 * 2. 锁住调度，连续发送队列深度两倍的事件；
 * 3. 验证队列只接受APP_EVENT_QUEUE_DEPTH条，其余安全丢弃；
 * 4. 恢复调度并有界等待成功事件全部消费；
 * 5. 再发送一条恢复标记事件；
 * 6. 验证恢复事件能够正常进入并被消费；
 * 7. 最终确认队列重新回到空状态。
 *
 * 注意：
 * - 测试期间不要操作实体按键；
 * - 临界区内不打印、不延时；
 * - 不直接修改消息队列内部字段；
 * - 正常发送仍使用公开的app_event_post()。
 */
static int app_event_flood(void)
{
    app_event_t event;
    rt_err_t result;
    rt_base_t level;
    rt_tick_t start_tick;
    rt_tick_t timeout_ticks;

    rt_uint32_t index;
    rt_uint32_t send_success;
    rt_uint32_t send_full;
    rt_uint32_t send_other_fail;

    rt_uint32_t before_attempt;
    rt_uint32_t before_posted;
    rt_uint32_t before_consumed;
    rt_uint32_t before_dropped;

    rt_uint32_t after_attempt;
    rt_uint32_t after_posted;
    rt_uint32_t after_consumed;
    rt_uint32_t after_dropped;

    rt_uint32_t final_posted;
    rt_uint32_t final_consumed;
    rt_uint32_t final_dropped;

    rt_bool_t drained;
    rt_bool_t recovered;
    rt_bool_t burst_ok;

    if ((g_app_initialized != RT_TRUE) ||
        (g_event_queue == RT_NULL) ||
        (g_app_thread == RT_NULL))
    {
        rt_kprintf(
            "[APP][STRESS][ERROR] app is not ready\n");

        return -RT_EBUSY;
    }

    /* 取得测试前的一致统计快照。 */
    level = rt_hw_interrupt_disable();

    before_attempt = g_event_post_attempt_count;
    before_posted = g_event_post_count;
    before_consumed = g_consume_count;
    before_dropped = g_event_drop_count;

    rt_hw_interrupt_enable(level);

    /*
     * 测试前必须为空，避免正常业务事件改变
     * “8条成功、8条丢弃”的确定性结果。
     */
    if (before_posted != before_consumed)
    {
        rt_kprintf(
            "[APP][STRESS][ERROR] queue is not empty: "
            "posted=%u consumed=%u inflight=%u\n",
            (unsigned int)before_posted,
            (unsigned int)before_consumed,
            (unsigned int)(before_posted - before_consumed));

        return -RT_EBUSY;
    }

    event.type = APP_EVENT_DIAG_REQUEST;
    event.source = APP_EVENT_SOURCE_TEST;
    event.tick = 0;
    event.value = 0;

    send_success = 0;
    send_full = 0;
    send_other_fail = 0;

    /*
     * 重要：只锁调度，不长时间关闭硬件中断。
     * app线程暂时不能消费，因此可确定性填满队列。
     */
    rt_enter_critical();

    for (index = 0;
         index < APP_EVENT_STRESS_ATTEMPTS;
         index++)
    {
        event.tick = rt_tick_get();
        event.value = index;

        result = app_event_post(&event);

        if (result == RT_EOK)
        {
            send_success++;
        }
        else if (result == -RT_EFULL)
        {
            send_full++;
        }
        else
        {
            send_other_fail++;
        }
    }

    /* 必须先恢复调度，再打印和等待。 */
    rt_exit_critical();

    level = rt_hw_interrupt_disable();

    after_attempt = g_event_post_attempt_count;
    after_posted = g_event_post_count;
    after_consumed = g_consume_count;
    after_dropped = g_event_drop_count;

    rt_hw_interrupt_enable(level);

    rt_kprintf(
        "[APP][STRESS] burst attempts=%u "
        "success=%u full=%u other_fail=%u\n",
        (unsigned int)APP_EVENT_STRESS_ATTEMPTS,
        (unsigned int)send_success,
        (unsigned int)send_full,
        (unsigned int)send_other_fail);

    rt_kprintf(
        "[APP][STRESS] counter delta: "
        "attempt=%u posted=%u dropped=%u\n",
        (unsigned int)(after_attempt - before_attempt),
        (unsigned int)(after_posted - before_posted),
        (unsigned int)(after_dropped - before_dropped));

    burst_ok = RT_TRUE;

    if ((after_attempt - before_attempt) !=
        APP_EVENT_STRESS_ATTEMPTS)
    {
        burst_ok = RT_FALSE;
    }

    if (send_success != APP_EVENT_QUEUE_DEPTH)
    {
        burst_ok = RT_FALSE;
    }

    if (send_full !=
        (APP_EVENT_STRESS_ATTEMPTS -
         APP_EVENT_QUEUE_DEPTH))
    {
        burst_ok = RT_FALSE;
    }

    if (send_other_fail != 0U)
    {
        burst_ok = RT_FALSE;
    }

    if ((after_posted - before_posted) != send_success)
    {
        burst_ok = RT_FALSE;
    }

    if ((after_dropped - before_dropped) != send_full)
    {
        burst_ok = RT_FALSE;
    }

    timeout_ticks =
        rt_tick_from_millisecond(
            APP_EVENT_STRESS_TIMEOUT_MS);

    if (timeout_ticks == 0)
    {
        timeout_ticks = 1;
    }

    /*
     * 等待本轮成功投递的事件全部消费。
     * Shell线程可以等待，但正常app_event_post()仍然不阻塞。
     */
    start_tick = rt_tick_get();
    drained = RT_FALSE;

    while ((rt_tick_get() - start_tick) < timeout_ticks)
    {
        level = rt_hw_interrupt_disable();

        after_posted = g_event_post_count;
        after_consumed = g_consume_count;

        rt_hw_interrupt_enable(level);

        if (after_consumed >=
            (before_consumed + send_success))
        {
            drained = RT_TRUE;
            break;
        }

        rt_thread_mdelay(APP_EVENT_STRESS_POLL_MS);
    }

    if (drained != RT_TRUE)
    {
        rt_kprintf(
            "[APP][STRESS][ERROR] drain timeout: "
            "expected=%u consumed_delta=%u\n",
            (unsigned int)send_success,
            (unsigned int)(
                after_consumed - before_consumed));

        return -RT_ETIMEOUT;
    }

    if (after_posted != after_consumed)
    {
        rt_kprintf(
            "[APP][STRESS][ERROR] queue did not return empty: "
            "posted=%u consumed=%u\n",
            (unsigned int)after_posted,
            (unsigned int)after_consumed);

        burst_ok = RT_FALSE;
    }

    rt_kprintf(
        "[APP][STRESS] drain complete: "
        "posted_delta=%u consumed_delta=%u\n",
        (unsigned int)(after_posted - before_posted),
        (unsigned int)(after_consumed - before_consumed));

    /*
     * 压力结束后发送单独的恢复事件。
     * 特殊value用于在UART日志中识别本条消息。
     */
    event.type = APP_EVENT_DIAG_REQUEST;
    event.source = APP_EVENT_SOURCE_TEST;
    event.tick = rt_tick_get();
    event.value = APP_EVENT_RECOVERY_VALUE;

    result = app_event_post(&event);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[APP][STRESS][ERROR] recovery post failed: %d\n",
            (int)result);

        return (int)result;
    }

    start_tick = rt_tick_get();
    recovered = RT_FALSE;

    while ((rt_tick_get() - start_tick) < timeout_ticks)
    {
        level = rt_hw_interrupt_disable();

        final_posted = g_event_post_count;
        final_consumed = g_consume_count;
        final_dropped = g_event_drop_count;

        rt_hw_interrupt_enable(level);

        if (final_consumed > after_consumed)
        {
            recovered = RT_TRUE;
            break;
        }

        rt_thread_mdelay(APP_EVENT_STRESS_POLL_MS);
    }

    if (recovered != RT_TRUE)
    {
        rt_kprintf(
            "[APP][STRESS][ERROR] "
            "recovery event consume timeout\n");

        return -RT_ETIMEOUT;
    }

    rt_kprintf(
        "[APP][STRESS] recovery posted=%u consumed=%u "
        "dropped=%u\n",
        (unsigned int)(final_posted - after_posted),
        (unsigned int)(final_consumed - after_consumed),
        (unsigned int)(final_dropped - after_dropped));

    if (((final_posted - after_posted) != 1U) ||
        ((final_consumed - after_consumed) != 1U) ||
        (final_dropped != after_dropped) ||
        (final_posted != final_consumed))
    {
        rt_kprintf(
            "[APP][STRESS][ERROR] "
            "recovery counters do not match\n");

        return -RT_ERROR;
    }

    if (burst_ok != RT_TRUE)
    {
        rt_kprintf(
            "[APP][STRESS][FAIL] "
            "burst result does not match "
            "depth=%u attempts=%u\n",
            (unsigned int)APP_EVENT_QUEUE_DEPTH,
            (unsigned int)APP_EVENT_STRESS_ATTEMPTS);

        return -RT_ERROR;
    }

    rt_kprintf(
        "[APP][STRESS][PASS] "
        "success=%u dropped=%u recovery=ok\n",
        (unsigned int)send_success,
        (unsigned int)send_full);

    return 0;
}


static rt_err_t indicator_selftest(void)
{
    app_event_t event =
    {
        .type   = APP_EVENT_INDICATOR_SELF_TEST,
        .source = APP_EVENT_SOURCE_SYSTEM,
        .tick   = rt_tick_get(),
        .value  = 0
    };

    return app_event_post(&event);
}

/* 简介：构造并发布一条UI语义事件
 * 输入：
 *     action：UI动作类型
 *     source：UI事件的抽象来源
 *     value：动作需要的值类型载荷
 * 返回值：
 *     RT_EOK：事件成功进入UI队列
 *     其他：ui_event_post()返回的真实错误码
 */
static rt_err_t app_request_ui_action(ui_action_t action,ui_event_source_t source,rt_uint32_t value)
{
    ui_event_t event =
    {
        .type = action,
        .source = source,

        /*
         * 重要：任务11测量的是统一语义进入UI路径，
         * 到项目页面状态处理完成的时间。
         * 原始GPIO事件tick仍保留在app事件日志中。
         */
        .tick = rt_tick_get(),

        .value = value
    };

    return ui_event_post(&event);
}

/*
 * 重要：以下函数只保护统计字段。
 * 临界区内不得调用消息队列、日志或任何可能阻塞的接口。
 */
static void app_record_post_attempt(void)
{
    rt_base_t level = rt_hw_interrupt_disable();

    g_event_post_attempt_count++;

    rt_hw_interrupt_enable(level);
}

static void app_record_invalid(void)
{
    rt_base_t level = rt_hw_interrupt_disable();

    g_event_invalid_count++;

    rt_hw_interrupt_enable(level);
}

static void app_record_post_drop(void)
{
    rt_base_t level = rt_hw_interrupt_disable();

    g_event_drop_count++;

    rt_hw_interrupt_enable(level);
}

static void app_record_post_success(void)
{
    rt_uint32_t inflight;
    rt_base_t level = rt_hw_interrupt_disable();

    g_event_post_count++;

    /*
     * max_inflight是“尚未完成处理的事件数”高水位，
     * 不是直接读取RT-Thread消息队列内部entry字段。
     */
    inflight = g_event_post_count - g_consume_count;
    if (inflight > g_event_max_inflight)
    {
        g_event_max_inflight = inflight;
    }

    rt_hw_interrupt_enable(level);
}

static void app_record_consumed(void)
{
    rt_base_t level = rt_hw_interrupt_disable();

    g_consume_count++;

    rt_hw_interrupt_enable(level);
}

#ifdef RT_USING_FINSH
MSH_CMD_EXPORT(app_stats,
                show application event queue statistics);
MSH_CMD_EXPORT(task11_resources,
                show task11 heap queue latency and thread baseline);
MSH_CMD_EXPORT(app_event_flood,
                inject events to verify queue full handling);
MSH_CMD_EXPORT(indicator_selftest,
                Perform RGB short-term self-test);

#endif