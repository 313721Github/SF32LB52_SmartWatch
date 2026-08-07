#include "app_main.h"
#include "indicator_port.h"
#include "button_port.h"

#include "rthw.h"

#define APP_EVENT_QUEUE_DEPTH  8U  
#define APP_THREAD_STACK_SIZE  2048U
#define APP_THREAD_PRIORITY    18U
#define APP_THREAD_TIMESLICE   10U

static rt_mq_t g_event_queue=RT_NULL;  
static rt_thread_t g_app_thread=RT_NULL;

static rt_uint32_t g_event_post_count;
static rt_uint32_t g_event_drop_count;
static rt_uint32_t g_consume_count;         //接收队列消息数量

static rt_bool_t g_indicator_available=RT_FALSE;                    // 硬件能不能控制
static indicator_state_t g_app_indicator_state=INDICATOR_STATE_OFF; // 当前显示什么状态

static rt_bool_t g_button_available = RT_FALSE;                     //按键初始化完成
static rt_bool_t g_app_initialized = RT_FALSE;                      //应用核心初始化成功

rt_err_t app_main_init(void);
rt_err_t app_event_post(const app_event_t *event);
static rt_err_t app_button_event_handler(const button_event_t *button_event,void *user_data);//应用层按键回调
static void app_thread_entry(void *parameter);          //g_app_thread线程入口,接收mq
static void app_handle_event(const app_event_t *event); //接收消息后的处理函数
static rt_err_t app_indicator_selftest(void);            //RGB自检
static rt_err_t indicator_selftest(void);                //向队列投递RGB自检

//shell测试
static rt_err_t app_diag_request(void);
static int app_stats(void);
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
        rt_kprintf("[APP][INIT][OK] application ready\n");
    }
    else
    {
        rt_kprintf("[APP][INIT][WARN] "
                "application ready in degraded mode "
                "button=%u indicator=%u\n",
                (unsigned int)g_button_available,
                (unsigned int)g_indicator_available);
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

     /* 队列尚未创建，说明应用层没有完成初始化 */
    if(g_event_queue==RT_NULL)
    {
        return -RT_ERROR;
    }
   
    if ((event->source >= APP_EVENT_SOURCE_COUNT) ||
    (event->type <= APP_EVENT_NONE) ||
    (event->type >= APP_EVENT_TYPE_COUNT))
    {
        return -RT_EINVAL;
    }

    /*
     * rt_mq_send()不等待空闲空间。
     * 队列已满时立即返回错误，适合按键回调或定时器回调。
     */
    event_copy=*event;
    result =rt_mq_send(
        g_event_queue,
        &event_copy,
        sizeof(app_event_t)
    );
    if(result != RT_EOK)
    {
        g_event_drop_count++;
        return result;

    }
    
    g_event_post_count++;
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
            g_consume_count++;
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
                */
                rt_kprintf("[APP][KEY1] HOME short press\n");  
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
                    if (indicator_set_state(new_state) == RT_EOK)
                    {
                        g_app_indicator_state = new_state;
                    }
                }
                else
                {
                    rt_kprintf("[APP][INDICATOR][WARN] device unavailable\n");
                }


                break;
            }           
            break;
        case APP_EVENT_KEY_LONG:
            if(event->source==APP_EVENT_SOURCE_KEY1)
            {
                rt_kprintf("[APP][KEY1] long press reported\n");
                break;
            }
            else if(event->source==APP_EVENT_SOURCE_KEY2)
            {
                if (g_indicator_available == RT_TRUE)
                {
                    if (indicator_set_state(INDICATOR_STATE_OFF) == RT_EOK)
                    {
                        g_app_indicator_state = INDICATOR_STATE_OFF;
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

    indicator_state_t indicator_state;

    rt_uint32_t posted;
    rt_uint32_t consumed;
    rt_uint32_t dropped;

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

    posted = g_event_post_count;
    consumed = g_consume_count;
    dropped = g_event_drop_count;

    rt_hw_interrupt_enable(level);

    rt_kprintf(
        "[APP][STATS] initialized=%u queue=%u thread=%u\n",
        (unsigned int)initialized,
        (unsigned int)queue_ready,
        (unsigned int)thread_ready);

    rt_kprintf(
        "[APP][STATS] button=%u indicator=%u state=%d\n",
        (unsigned int)button_available,
        (unsigned int)indicator_available,
        (int)indicator_state);

    rt_kprintf(
        "[APP][STATS] posted=%u consumed=%u dropped=%u\n",
        (unsigned int)posted,
        (unsigned int)consumed,
        (unsigned int)dropped);

    return 0;
}


//队列满实验
static int app_event_flood(void)
{
    app_event_t event;
    rt_uint32_t before_post;
    rt_uint32_t before_drop;
    int index;

    //统计队列满成功和丢弃的消息数
    before_post = g_event_post_count;
    before_drop = g_event_drop_count;

    event.type   = APP_EVENT_DIAG_REQUEST;
    event.source = APP_EVENT_SOURCE_TEST;
    event.value  = 0;

    /*
     * 仅用于队列满测试：
     * 暂停线程调度，使app线程不能在投递过程中消费队列。
     */
    rt_enter_critical();

    for (index = 0; index < APP_EVENT_QUEUE_DEPTH * 2; index++)
    {
        event.tick  = rt_tick_get();
        event.value = (rt_uint32_t)index;
        app_event_post(&event);
    }

    rt_exit_critical();

    rt_kprintf("[APP][FLOOD] attempts=%u posted_delta=%u drop_delta=%u\n",
               (unsigned int)(APP_EVENT_QUEUE_DEPTH * 2),
               (unsigned int)(g_event_post_count - before_post),
               (unsigned int)(g_event_drop_count - before_drop));

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

#ifdef RT_USING_FINSH


#endif

#ifdef RT_USING_FINSH
MSH_CMD_EXPORT(app_stats,
               show application event queue statistics);

MSH_CMD_EXPORT(app_event_flood,
               inject events to verify queue full handling);
MSH_CMD_EXPORT(indicator_selftest,
               Perform RGB short-term self-test);

#endif