#include "ui_runtime.h"
#include "ui_page.h"
#include "rthw.h"
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lv_ex_data.h"

#ifdef RT_USING_FINSH
#include "finsh.h"
#endif

//ui线程相关宏
#define UI_THREAD_NAME          "ui"
#define UI_THREAD_STACK_SIZE    (16U * 1024U)
#define UI_THREAD_PRIORITY      RT_THREAD_PRIORITY_MIDDLE
#define UI_THREAD_TIMESLICE     10U

#define UI_EXPECTED_WIDTH       390U
#define UI_EXPECTED_HEIGHT      450U

#define UI_WAIT_MIN_MS          1U
#define UI_WAIT_MAX_MS          20U
//ui事件队列相关宏
#define UI_EVENT_QUEUE_NAME        "ui_evt"
#define UI_EVENT_QUEUE_DEPTH       8U
#define UI_EVENT_BATCH_MAX         4U
#define UI_EVENT_LATE_MS           100U
//ui事件压力测试宏
#define UI_EVENT_STRESS_ATTEMPTS       16U
#define UI_EVENT_STRESS_TIMEOUT_MS     2000U
#define UI_EVENT_STRESS_POLL_MS        10U

//ui线程
static struct rt_thread g_ui_thread;
//ui队列
static rt_mq_t g_ui_event_queue = RT_NULL;

ALIGN(RT_ALIGN_SIZE)
//ui线程栈
static rt_uint8_t g_ui_thread_stack[UI_THREAD_STACK_SIZE];

//ui运行时上下文
typedef struct
{
    ui_runtime_event_handler_t g_ui_event_handler;  //回调函数句柄
    void *g_ui_event_user_data;                     //回调函数私有资源

    rt_bool_t g_ui_event_queue_ready;               //事件队列是否就绪
    rt_bool_t g_ui_wait_sample_valid;               //线程等待时间样本值是否有效
    ui_runtime_stats_t stats;                       //统计信息

} ui_runtime_context_t;
static ui_runtime_context_t g_ui_runtime_context =
{
    .stats.state = UI_RUNTIME_STATE_STOPPED,

    .g_ui_event_handler=RT_NULL,
    .g_ui_event_user_data=RT_NULL,

    .g_ui_event_queue_ready=RT_FALSE,
    .g_ui_wait_sample_valid=RT_FALSE,

    .stats.last_error = RT_EOK,
    .stats.failure_stage = UI_FAILURE_NONE

};



//公开函数声明
rt_err_t ui_runtime_init(void);                                                                     //启动静态线程
rt_err_t ui_runtime_register_handler(ui_runtime_event_handler_t handler,void *user_data);           //回调注册函数
ui_runtime_state_t ui_runtime_get_state(void);                                                      //查询ui模块状态
rt_err_t ui_runtime_get_stats(ui_runtime_stats_t *stats);                                           //获取ui模块信息
//线程安全的ui事件入口
rt_err_t ui_event_post(const ui_event_t *event);                                                    //ui事件投递入口

//私有函数声明
static void ui_thread_entry(void *parameter);                                                       //ui线程入口
static void ui_runtime_record_error(ui_failure_stage_t stage,rt_err_t error);                       //记录第一处错误
static void ui_runtime_set_state(ui_runtime_state_t state);                                         //设置运行状态
static rt_err_t ui_runtime_notify(ui_runtime_event_t event,ui_failure_stage_t stage,rt_err_t error);    //生命周期通知
static rt_uint32_t ui_runtime_normalize_wait(rt_uint32_t handler_return_ms);                        //归一化等待时间
static void ui_runtime_update_handler_stats(rt_uint32_t handler_return_ms,rt_uint32_t wait_ms);     //更新handler统计
static rt_err_t ui_runtime_capture_display_info(void);                                              //捕获显示信息
static void ui_runtime_thread_fail(ui_failure_stage_t stage,rt_err_t error);                        //ui线程失败处理
static int ui_stats(void);                                                                          //shell函数声明

static rt_err_t ui_runtime_handle_event(const ui_event_t *event);                                   //ui事件处理函数
static void ui_runtime_update_event_stats(rt_tick_t latency_ticks,rt_bool_t event_valid,rt_err_t handle_result);    //更新事件统计
static void ui_runtime_stat_increment(rt_uint32_t *counter);                                        //线程安全地增加一个32位统计计数
static rt_bool_t ui_event_is_valid(const ui_event_t *event);                                        //ui事件合法性判断
static void ui_runtime_release_event_queue(void);                                                   //队列清理函数
#ifdef RT_USING_FINSH
static int ui_event_stress(void);                                                                   //ui事件压力测试
#endif
//---------------------------------------------------------------公开函数---------------------------------------------------------------

/*简介：启动静态线程:注册ui线程并启动
  输入：无
  返回值：无
*/
rt_err_t ui_runtime_init(void)
{
    rt_err_t result;
    /*
     * TODO 1：
     * 检查当前状态必须为STOPPED。
     * 检查生命周期回调已注册。
     */
    if(g_ui_runtime_context.stats.state!=UI_RUNTIME_STATE_STOPPED)
    {
        rt_kprintf("[UI][WARN] ui_runtime_init called when state=%d\n",(int)g_ui_runtime_context.stats.state);
        return -RT_EBUSY;
    }
    if(g_ui_runtime_context.g_ui_event_handler==RT_NULL)
    {
        rt_kprintf("[UI][WARN] g_ui_event_handler not registered!!!");
        return -RT_EINVAL;
    }
    /*
     * TODO 2：
     * 清理本次运行统计：
     * failure_stage = NONE
     * last_error = RT_EOK
     * g_ui_wait_sample_valid = RT_FALSE
     *
     * 不要清掉刚注册的回调。
     */
    g_ui_runtime_context.stats.failure_stage=UI_FAILURE_NONE;
    g_ui_runtime_context.stats.last_error=RT_EOK;
    g_ui_runtime_context.g_ui_wait_sample_valid=RT_FALSE;

    //创建ui队列
    g_ui_event_queue = rt_mq_create(UI_EVENT_QUEUE_NAME, sizeof(ui_event_t), UI_EVENT_QUEUE_DEPTH, RT_IPC_FLAG_FIFO);
    if (g_ui_event_queue == RT_NULL)
    {
        ui_runtime_thread_fail(UI_FAILURE_QUEUE_CREATE,-RT_ENOMEM);
        return -RT_ENOMEM;
    }
    else
    {
        g_ui_runtime_context.g_ui_event_queue_ready=RT_TRUE;
    }

    //注册线程
    result = rt_thread_init(
        &g_ui_thread,
        UI_THREAD_NAME,
        ui_thread_entry,
        RT_NULL,
        g_ui_thread_stack,
        sizeof(g_ui_thread_stack),
        UI_THREAD_PRIORITY,
        UI_THREAD_TIMESLICE);

    if (result != RT_EOK)
    {
        /*
        * rt_thread_init()没有成功，因此不能对g_ui_thread调用rt_thread_detach()。这里只释放已经创建的UI队列。
        */
        ui_runtime_release_event_queue();
        ui_runtime_thread_fail(UI_FAILURE_THREAD_INIT,result);
        return result;
    }

    /*
     * TODO：
     * 保存thread地址；
     * 设置STARTING。
     */
    g_ui_runtime_context.stats.thread=&g_ui_thread;
    g_ui_runtime_context.stats.state=UI_RUNTIME_STATE_STARTING;

    //启动UI线程
    result = rt_thread_startup(&g_ui_thread);
    if (result != RT_EOK)
    {
        rt_base_t level;

        /*
        * rt_thread_init()已经成功，可以脱离静态线程对象。
        * 当前线程尚未启动，因此可以安全释放UI队列。
        */
        (void)rt_thread_detach(&g_ui_thread);

        level = rt_hw_interrupt_disable();
        g_ui_runtime_context.stats.thread = RT_NULL;
        rt_hw_interrupt_enable(level);

        ui_runtime_release_event_queue();
        ui_runtime_thread_fail(UI_FAILURE_THREAD_STARTUP,result);
        return result;
    }

    return RT_EOK;

}




/*简介：ui事件投递入口
  输入：event：ui事件
  返回值：rt_err_t：错误类型
*/
rt_err_t ui_event_post(const ui_event_t *event)
{
    rt_mq_t queue;
    rt_bool_t queue_ready;
    ui_runtime_state_t state;
    ui_event_t queued_event;
    rt_err_t result;
    rt_base_t level;

    /* 空指针不进入post_attempt统计。 */
    if(event==RT_NULL)
    {
        return -RT_EINVAL;
    }

    ui_runtime_stat_increment(&g_ui_runtime_context.stats.post_attempt);

    //检查事件合法性
    if(ui_event_is_valid(event)!=RT_TRUE)
    {
        ui_runtime_stat_increment(&g_ui_runtime_context.stats.invalid);

        return -RT_EINVAL;
    }

    /*
     * 对共享运行状态进行一次短临界区快照。
     * 临界区内不能调用rt_mq_send()。
     */
    level = rt_hw_interrupt_disable();

    queue = g_ui_event_queue;
    queue_ready =g_ui_runtime_context.g_ui_event_queue_ready;
    state =g_ui_runtime_context.stats.state;

    rt_hw_interrupt_enable(level);

    //检查队列是否就绪
    if(queue_ready!=RT_TRUE||queue == RT_NULL)
    {
        ui_runtime_stat_increment(&g_ui_runtime_context.stats.dropped);
        return -RT_ERROR;
    }
    //检查UI状态是否READY
    if(state !=UI_RUNTIME_STATE_READY)
    {
        ui_runtime_stat_increment(&g_ui_runtime_context.stats.dropped);
        return -RT_EBUSY;
    }
    //投递事件
    queued_event = *event;
    result = rt_mq_send(
        queue,
        &queued_event,
        sizeof(queued_event));
    if(result!=RT_EOK)
    {
        /*
         * 包括预期的-RT_EFULL及其他底层发送错误。
         * 不重试、不覆盖旧消息、不阻塞调用线程。
         */
        ui_runtime_stat_increment(&g_ui_runtime_context.stats.dropped);
        return result;
    }

    ui_runtime_stat_increment(&g_ui_runtime_context.stats.posted);

    return RT_EOK;

}

/*简介：回调注册函数
  输入：handler:回调函数句柄
        user_data：回调函数私有资源
  返回值：无
*/
rt_err_t ui_runtime_register_handler(ui_runtime_event_handler_t handler,void *user_data)
{
    rt_base_t level;

    if (handler == RT_NULL)
    {
        return -RT_EINVAL;
    }

    level = rt_hw_interrupt_disable();

    /*
     * TODO：
     * 只允许在STOPPED状态注册。如果已经STARTING/READY/FAILED，恢复中断后返回-RT_EBUSY。
     */
    if(g_ui_runtime_context.stats.state!=UI_RUNTIME_STATE_STOPPED)
    {
        rt_hw_interrupt_enable(level);
        return -RT_EBUSY;
    }

    /*
     * TODO：
     * 保存handler和user_data。
     */
    g_ui_runtime_context.g_ui_event_handler=handler;
    g_ui_runtime_context.g_ui_event_user_data=user_data;


    rt_hw_interrupt_enable(level);

    return RT_EOK;

}

/*简介：查询ui模块状态
  输入：无
  返回值：ui_runtime_state_t：ui模块状态
*/
ui_runtime_state_t ui_runtime_get_state(void)
{
    ui_runtime_state_t state;
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    state = g_ui_runtime_context.stats.state;
    rt_hw_interrupt_enable(level);

    return state;
}


/*简介：获取ui模块信息
  输入：stats：ui模块信息
  返回值：rt_err_t：错误类型
*/
rt_err_t ui_runtime_get_stats(ui_runtime_stats_t *stats)
{
    rt_base_t level;

    if (stats == RT_NULL)
    {
        return -RT_EINVAL;
    }

    level = rt_hw_interrupt_disable();

    *stats = g_ui_runtime_context.stats;

    /*
     * 队列句柄和ready标志的权威存储位于ui_runtime私有状态。
     * 这里只生成查询快照，不建立第二份权威状态。
     */
    stats->event_queue = g_ui_event_queue;
    stats->queue_depth = UI_EVENT_QUEUE_DEPTH;
    stats->queue_ready =g_ui_runtime_context.g_ui_event_queue_ready;
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

//---------------------------------------------------------------私有函数---------------------------------------------------------------

/*简介：ui线程入口
  输入：parameter:用户参数
  返回值：无
*/
static void ui_thread_entry(void *parameter)
{
    rt_err_t result;
    rt_uint32_t handler_return_ms;
    rt_uint32_t wait_ms;
    rt_base_t level;
    rt_tick_t wait_ticks;
    ui_event_t event;
    rt_uint32_t batch_count;

    (void)parameter;

    rt_kprintf("[UI][THREAD] started\n");

    //统计信息更新
    level = rt_hw_interrupt_disable();
    g_ui_runtime_context.stats.init_attempt++;
    rt_hw_interrupt_enable(level);

    //初始化ui模块
    result = littlevgl2rtt_init("lcd");
    if (result != RT_EOK)
    {
        ui_runtime_thread_fail(
            UI_FAILURE_LVGL_PORT_INIT,
            result);
        return;
    }

    lv_ex_data_pool_init();
    rt_kprintf("[UI][INIT] LVGL extension data pool initialized\n");

    result = ui_runtime_capture_display_info();
    if (result != RT_EOK)
    {
        /*
         * capture函数已经记录了精确阶段。
         * TODO：取得已经保存的failure_stage和last_error，再调用统一失败处理。
         *      在此处发生错误说明之前初始化阶段未发生错误即未退出，
         *      此时记录的failure_stage和last_error便都是ui_runtime_capture_display_info()的错误
         * 注意不要把阶段统一覆盖成DRAW_BUFFER。
         */
        ui_runtime_thread_fail(g_ui_runtime_context.stats.failure_stage,g_ui_runtime_context.stats.last_error);
        return;
    }

    //创建LVGL界面
    result = ui_page_create();
    if (result != RT_EOK)
    {
        ui_runtime_thread_fail(
            UI_FAILURE_PAGE_CREATE,
            result);
        return;
    }

    /*
     * 第一次handler：
     * TODO 1：调用lv_timer_handler()。
     * TODO 2：归一化等待时间。
     * TODO 3：更新handler统计。
     */



    handler_return_ms=lv_task_handler();
    wait_ms=ui_runtime_normalize_wait(handler_return_ms);
    //更新handle统计信息
    ui_runtime_update_handler_stats(handler_return_ms,wait_ms);

    /*
     * TODO：
     * init_success加1；
     * 保存ready_tick；
     * state改为READY。
     */

    level = rt_hw_interrupt_disable();

    g_ui_runtime_context.stats.init_success++;
    g_ui_runtime_context.stats.ready_tick=rt_tick_get();
    g_ui_runtime_context.stats.state=UI_RUNTIME_STATE_READY;

    rt_hw_interrupt_enable(level);

    rt_kprintf("[UI][INIT][READY] runtime initialized\n");

    ui_runtime_notify(
        UI_RUNTIME_EVENT_READY,
        UI_FAILURE_NONE,
        RT_EOK);


    while (1)
    {
        /*
        * 1. 运行LVGL定时处理器。
        *
        * 无论当前是否存在UI事件，每轮都必须回到这里，
        * 保证LVGL刷新、定时器和输入设备处理不会停止。
        */
        handler_return_ms=lv_task_handler();

        /*
        * 2. 将LVGL建议的等待时间限制在1～20 ms，
        * 并更新原有handler运行统计。
        */
        wait_ms=ui_runtime_normalize_wait(handler_return_ms);
        //更新handle统计信息
        ui_runtime_update_handler_stats(handler_return_ms,wait_ms);

        /*
        * 3. 将毫秒转换为RT-Thread tick。
        *
        * 在系统tick频率较低时，较小的毫秒值可能被转换成0，
        * 因此必须保证至少等待1 tick。
        */
        wait_ticks=rt_tick_from_millisecond(wait_ms);
        if (wait_ticks == 0)
        {
            wait_ticks = 1;
        }

        /*
        * 4. 等待第一条UI事件。
        *
        * 等待时间使用LVGL handler建议的时间：
        * - 有事件时，消息队列立即唤醒UI线程；
        * - 没有事件时，等待超时后重新运行LVGL handler；
        * - 禁止使用RT_WAITING_FOREVER。
        *
        * 该消息队列等待替代原来的rt_thread_mdelay()，
        * 不能在本轮末尾再进行一次相同延时。
        */
        result=rt_mq_recv(g_ui_event_queue,&event,sizeof(event),wait_ticks);
        if (result == RT_EOK)
        {
            /*
            * 5. 第一条事件已经成功出队。
            *
            * consumed、延迟、late、handled和handle_fail
            * 由ui_runtime_handle_event()统一更新。
            */
            (void)ui_runtime_handle_event(&event);
            /*
            * 第一条已经处理，因此batch_count从1开始。
            * UI_EVENT_BATCH_MAX为4时，本轮最多再处理3条。
            */
            for (batch_count = 1;batch_count < UI_EVENT_BATCH_MAX;batch_count++)
            {
                /*
                * 后续事件使用非阻塞接收。
                *
                * 这样可以处理短时突发，但不会因为队列中持续有消息
                * 而长时间阻止下一次lv_task_handler()运行。
                */
                result = rt_mq_recv(g_ui_event_queue,&event,sizeof(event),0);

                if (result == RT_EOK)
                {
                    (void)ui_runtime_handle_event(&event);
                }
                else if (result == -RT_ETIMEOUT)
                {
                    /*
                    * 队列已经为空。
                    * timeout=0时返回-RT_ETIMEOUT表示当前没有消息，
                    * 属于正常结束本轮批处理，不计入recv_fail。
                    */
                    break;
                }
                else
                {
                    /*
                    * 出现了除“队列为空”之外的接收错误。
                    * 记录错误后结束本轮批处理，让线程重新运行handler。
                    */
                    ui_runtime_stat_increment(
                        &g_ui_runtime_context.stats.recv_fail);

                    rt_kprintf(
                        "[UI][QUEUE][ERROR] "
                        "non-blocking receive failed: %d\n",
                        (int)result);

                    break;
                }
            }

        }
        else if (result == -RT_ETIMEOUT)
        {
            /*
            * 在LVGL建议的等待时间内没有收到UI事件。
            *
            * 这是正常运行路径：
            * 不增加recv_fail，直接进入下一轮handler。
            */
        }
        else
        {
            /*
            * 第一条消息的有限等待接收出现异常。
            * 不能把它当作普通队列为空。
            */
            ui_runtime_stat_increment(
                &g_ui_runtime_context.stats.recv_fail);

            rt_kprintf(
                "[UI][QUEUE][ERROR] receive failed: %d\n",
                (int)result);

            /*
            * 防止消息队列对象异常时立即反复返回错误，
            * 导致UI线程形成高速日志循环。
            */
            rt_thread_mdelay(UI_WAIT_MIN_MS);
        }

    }
}

/* 简介：处理一条已经从ui_evt成功取出的UI事件
 * 输入：event：ui事件
 * 返回值：rt_err_t：错误类型
 */
static rt_err_t ui_runtime_handle_event(const ui_event_t *event)
{

    rt_tick_t latency_ticks;
    rt_bool_t event_valid;
    rt_err_t handle_result;


    /*
     * 空指针不可能代表一条正常出队消息，
     * 因此不增加consumed。
     */
    if (event == RT_NULL)
    {
        ui_runtime_stat_increment(
            &g_ui_runtime_context.stats.invalid);

        return -RT_EINVAL;
    }

    /*
     * 使用无符号tick减法计算延迟。
     * 在事件延迟未超过tick计数周期的前提下，
     * 可以自然处理rt_tick_t回绕。
     */
    latency_ticks = rt_tick_get() - event->tick;

    /* 消费端再次进行防御性合法性检查。 */
    event_valid = ui_event_is_valid(event);

    if (event_valid != RT_TRUE)
    {
        /*
         * 非法事件不得进入ui_page，
         * 防止错误动作访问LVGL页面对象。
         */
        handle_result = -RT_EINVAL;
    }
    else
    {
        /*
         * 当前函数只由UI线程调用，
         * 因此可以在这里安全操作LVGL页面对象。
         */
        handle_result = ui_page_handle_event(event);
    }

    /*
     * 对本次已出队事件进行一次性统计，
     * 防止consumed、latency和handled出现分散更新。
     */
    ui_runtime_update_event_stats(
        latency_ticks,
        event_valid,
        handle_result);

    return handle_result;

}


/* 简介：统一更新一条已出队UI事件的统计信息
 * 输入：
 *     latency_ticks：事件从产生到UI线程消费的延迟tick数
 *     event_valid：事件类型、来源和载荷是否合法
 *     handle_result：页面事件处理函数的返回值
 * 返回值：无
 *
 * 说明：
 * 1. 本函数只针对已经由rt_mq_recv()成功取出的事件调用；
 * 2. 因此无论事件是否合法，consumed都需要增加；
 * 3. 非法事件不进入页面处理成功/失败统计；
 * 4. 所有关联字段在同一个短临界区中更新，避免查询到半更新状态。
 */
static void ui_runtime_update_event_stats(
    rt_tick_t latency_ticks,
    rt_bool_t event_valid,
    rt_err_t handle_result)
{
    rt_base_t level;
    rt_tick_t late_threshold_ticks;

    /*
     * 将100 ms阈值转换成当前RT-Thread tick。
     * 不直接假设1 tick等于1 ms。
     */
    late_threshold_ticks =
        rt_tick_from_millisecond(UI_EVENT_LATE_MS);

    /*
     * 在低tick频率配置下，毫秒转换理论上可能得到0。
     * 至少使用1 tick，避免所有非零延迟都被误判为超时。
     */
    if (late_threshold_ticks == 0)
    {
        late_threshold_ticks = 1;
    }

    /*
     * 以下字段属于同一条事件的统计结果，
     * 必须在同一个短临界区内完成更新。
     *
     * 临界区内只进行整数比较和赋值：
     * 不打印日志；
     * 不调用消息队列；
     * 不调用LVGL；
     * 不执行可能阻塞的函数。
     */
    level = rt_hw_interrupt_disable();

    /*
     * 调用本函数意味着消息已经成功从ui_evt中取出。
     * 即使消息内容非法，也属于已消费消息。
     */
    g_ui_runtime_context.stats.consumed++;

    /* 保存最近一次事件的排队延迟。 */
    g_ui_runtime_context.stats.last_latency_ticks =
        latency_ticks;

    /* 更新启动以来观测到的最大排队延迟。 */
    if (latency_ticks >
        g_ui_runtime_context.stats.max_latency_ticks)
    {
        g_ui_runtime_context.stats.max_latency_ticks =
            latency_ticks;
    }

    /*
     * 超过约定的100 ms阈值时增加late。
     * 事件超时只用于诊断，不表示必须丢弃该事件。
     */
    if (latency_ticks > late_threshold_ticks)
    {
        g_ui_runtime_context.stats.late++;
    }

    if (event_valid != RT_TRUE)
    {
        /*
         * 消费端的防御性校验失败。
         * 不把非法事件计入handled或handle_fail。
         */
        g_ui_runtime_context.stats.invalid++;
    }
    else if (handle_result == RT_EOK)
    {
        /* 合法事件已经由ui_page成功处理。 */
        g_ui_runtime_context.stats.handled++;
    }
    else
    {
        /* 合法事件进入页面层，但页面处理失败。 */
        g_ui_runtime_context.stats.handle_fail++;
    }

    rt_hw_interrupt_enable(level);
}

/* 简介：线程安全地增加一个32位统计计数
 * 输入：counter：需要递增的统计字段
 * 返回值：无
 */
static void ui_runtime_stat_increment(rt_uint32_t *counter)
{
    rt_base_t level;

    if (counter == RT_NULL)
    {
        return;
    }

    level = rt_hw_interrupt_disable();
    (*counter)++;
    rt_hw_interrupt_enable(level);
}


/*简介：保存ui的第一次错误
  输入：stage：错误字段
        rt_err_t：错误类型
  返回值：无
*/
static void ui_runtime_record_error(ui_failure_stage_t stage,rt_err_t error)
{
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    if((g_ui_runtime_context.stats.failure_stage==UI_FAILURE_NONE)&&(error!=RT_EOK))
    {
        g_ui_runtime_context.stats.last_error=error;
        g_ui_runtime_context.stats.failure_stage=stage;
    }

    rt_hw_interrupt_enable(level);
}

/*简介：设置ui运行状态
  输入：state：新的运行状态
  返回值：无
*/
static void ui_runtime_set_state(ui_runtime_state_t state)
{
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    if(state>=UI_RUNTIME_STATE_COUNT)
    {
        rt_hw_interrupt_enable(level);
        rt_kprintf("[UI][ERR] ui_runtime_set_state called with invalid state=%d\n",(int)state);
        return;
    }
    g_ui_runtime_context.stats.state = state;

    rt_hw_interrupt_enable(level);
}

/*简介：生命周期通知
  输入：event：事件类型
        stage：错误阶段
        error：错误码
  返回值：无
*/
static rt_err_t ui_runtime_notify(
    ui_runtime_event_t event,
    ui_failure_stage_t stage,
    rt_err_t error)
{
    ui_runtime_event_handler_t handler;
    void *user_data;
    rt_base_t level;
    rt_err_t result;

    result=RT_EOK;
    level = rt_hw_interrupt_disable();

    /*
     * TODO：
     * 取得g_ui_event_handler和g_ui_event_user_data快照。
     */
    handler = g_ui_runtime_context.g_ui_event_handler;
    user_data = g_ui_runtime_context.g_ui_event_user_data;

    rt_hw_interrupt_enable(level);

    if (handler == RT_NULL)
    {
        rt_kprintf("[UI][EVENT][WARN] handler not registered\n");
        return -RT_ERROR;
    }

    /*
     * TODO：
     * 调用handler(event, stage, error, user_data)。
     * 检查返回值。
     * 投递失败只打印日志，不覆盖LVGL初始化的第一处错误。
     */
    result=handler(event,stage,error,user_data);
    if(result!=RT_EOK)
    {

        rt_kprintf("[UI][EVENT][error] Message delivery failed!! event:%d stage:%d result:%d\n",event,stage,result);
        return result;
    }

    return RT_EOK;

}

/*简介：归一化等待时间
  输入：handler_return_ms：等待时间ms
  返回值：无
*/
static rt_uint32_t ui_runtime_normalize_wait(
    rt_uint32_t handler_return_ms)
{
    rt_uint32_t wait_ms;

    if (handler_return_ms == 0U)
    {
        wait_ms = UI_WAIT_MIN_MS;
    }
    else if (handler_return_ms == LV_NO_TIMER_READY)
    {
        wait_ms = UI_WAIT_MAX_MS;
    }
    else if (handler_return_ms > UI_WAIT_MAX_MS)
    {
        wait_ms = UI_WAIT_MAX_MS;
    }
    else
    {
        wait_ms = handler_return_ms;
    }

    return wait_ms;

}

/*简介：更新handler统计
  输入：handler_return_ms上一次处理器调用返回耗时
        wait_ms:         上一次处理器调用等待耗时
  返回值：无
*/
static void ui_runtime_update_handler_stats(
    rt_uint32_t handler_return_ms,
    rt_uint32_t wait_ms)
{
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    /*
     * TODO 1：
     * handler_count加1。
     */
    g_ui_runtime_context.stats.handler_count++;
    /*
     * TODO 2：
     * 保存last_handler_return_ms和last_wait_ms。
     */
    g_ui_runtime_context.stats.last_handler_return_ms=handler_return_ms;
    g_ui_runtime_context.stats.last_wait_ms=wait_ms;

    /*
     * TODO 3：
     * 如果这是第一个等待样本：
     *     min_wait_ms = wait_ms；
     *     max_wait_ms = wait_ms；
     *     g_ui_wait_sample_valid = RT_TRUE。
     *
     * 否则分别更新min和max。
     */
    if (g_ui_runtime_context.g_ui_wait_sample_valid==RT_FALSE)
    {
        g_ui_runtime_context.stats.max_wait_ms=wait_ms;
        g_ui_runtime_context.stats.min_wait_ms=wait_ms;
        g_ui_runtime_context.g_ui_wait_sample_valid=RT_TRUE;
    }
    else
    {
        if(wait_ms>g_ui_runtime_context.stats.max_wait_ms)
        {
            g_ui_runtime_context.stats.max_wait_ms=wait_ms;
        }
        else if(wait_ms<g_ui_runtime_context.stats.min_wait_ms)
        {
            g_ui_runtime_context.stats.min_wait_ms=wait_ms;
        }
    }

    rt_hw_interrupt_enable(level);
}

/*简介：捕获显示信息
  输入：无
  返回值：rt_err_t：错误类型
*/
static rt_err_t ui_runtime_capture_display_info(void)
{
    lv_disp_t *display;
    lv_disp_draw_buf_t *draw_buf;
    rt_uint16_t hor_res;
    rt_uint16_t ver_res;
    rt_base_t level;

    display = lv_disp_get_default();
    if (display == RT_NULL)
    {
        ui_runtime_record_error(
            UI_FAILURE_DEFAULT_DISPLAY,
            -RT_ERROR);
        rt_kprintf("[UI][error] lv_disp_get_default failed!!\n");
        return -RT_ERROR;
    }

    draw_buf = lv_disp_get_draw_buf(display);
    if ((draw_buf == RT_NULL) ||
        (draw_buf->buf1 == RT_NULL))
    {
        ui_runtime_record_error(
            UI_FAILURE_DRAW_BUFFER,
            -RT_ERROR);
        rt_kprintf("[UI][error] lv_disp_get_draw_buf failed!!\n");
        return -RT_ERROR;
    }

    /*
     * TODO 1：
     * 使用LVGL接口取得水平和垂直分辨率。
     */
    hor_res=lv_disp_get_hor_res(display);
    ver_res=lv_disp_get_ver_res(display);
    if ((hor_res != UI_EXPECTED_WIDTH) ||
        (ver_res != UI_EXPECTED_HEIGHT))
    {
        ui_runtime_record_error(
            UI_FAILURE_RESOLUTION,
            -RT_ERROR);
        rt_kprintf("[UI][error] lv_disp_get_hor_res&ver_res failed!!\n");
        return -RT_ERROR;
    }

    level = rt_hw_interrupt_disable();

    /*
     * TODO 2：
     * 保存：
     * display
     * draw_buf
     * draw_buf->buf1
     * draw_buf->buf2
     * draw_buf->size
     * draw_buf->size * sizeof(lv_color_t)
     * hor_res
     * ver_res
     */
    g_ui_runtime_context.stats.display=display;

    g_ui_runtime_context.stats.draw_buf=draw_buf;
    g_ui_runtime_context.stats.buf1=draw_buf->buf1;
    g_ui_runtime_context.stats.buf2=draw_buf->buf2;
    g_ui_runtime_context.stats.draw_buf_pixels=draw_buf->size;
    g_ui_runtime_context.stats.draw_buf_bytes=draw_buf->size*sizeof(lv_color_t);

    g_ui_runtime_context.stats.hor_res=hor_res;
    g_ui_runtime_context.stats.ver_res=ver_res;

    rt_hw_interrupt_enable(level);

    /*
     * TODO 3：
     * 退出临界区后打印缓冲地址、像素数、字节数和分辨率。
     */
    rt_kprintf("[UI][INFO] draw buf  : buf1=%p buf2=%p\n",
               g_ui_runtime_context.stats.buf1,
               g_ui_runtime_context.stats.buf2);
    rt_kprintf("[UI][INFO] draw size : pixels=%u bytes=%u\n",
               (unsigned int)g_ui_runtime_context.stats.draw_buf_pixels,
               (unsigned int)g_ui_runtime_context.stats.draw_buf_bytes);
    rt_kprintf("[UI][INFO] resolution: %ux%u\n",
               (unsigned int)g_ui_runtime_context.stats.hor_res,
               (unsigned int)g_ui_runtime_context.stats.ver_res);

    return RT_EOK;
}

/*简介：ui线程失败处理
  输入：stage：错误字段
        error：错误类型
  返回值：rt_err_t：错误类型
*/
static void ui_runtime_thread_fail(
    ui_failure_stage_t stage,
    rt_err_t error)
{
    rt_base_t level;

    ui_runtime_record_error(stage, error);

    level = rt_hw_interrupt_disable();

    /*
     * TODO:
     * init_fail加1；
     * state改为UI_RUNTIME_FAILED。
     */
    g_ui_runtime_context.stats.init_fail++;
    g_ui_runtime_context.stats.state=UI_RUNTIME_STATE_FAILED;

    rt_hw_interrupt_enable(level);

    /*
     * TODO：
     * 打印stage和error。
     * 通知UI_RUNTIME_EVENT_FAILED。
     */

    rt_kprintf(
        "[UI][INIT][ERROR] stage=%d error=%d\n",
        (int)stage,
        (int)error);

    ui_runtime_notify(UI_RUNTIME_EVENT_FAILED,stage,error);

    /*
     * 本函数既可能由app初始化线程调用，也可能由UI线程调用。
     * 因此这里不脱离线程、不删除队列、不关闭LCD。
     *
     * UI线程中的调用者随后直接return，使线程自然结束；
     * FAILED状态会阻止新的UI事件进入队列。
     */
}

/*简介：ui事件合法性检查
  输入：event：ui事件
  返回值：rt_err_t：错误类型
*/
static rt_bool_t ui_event_is_valid(const ui_event_t *event)
{

    if (event == RT_NULL)
    {
        return RT_FALSE;
    }

    if (event->type <= UI_ACTION_NONE)
    {
        return RT_FALSE;
    }

    if (event->type >= UI_ACTION_COUNT)
    {
        return RT_FALSE;
    }

    if ((event->source < UI_EVENT_SOURCE_APP)||(event->source >= UI_EVENT_SOURCE_COUNT))
    {
        return RT_FALSE;
    }
    if (event->type == UI_ACTION_HOME ||
        event->type == UI_ACTION_NEXT ||
        event->type == UI_ACTION_BACK)
    {
        if (event->value != 0)
        {
            return RT_FALSE;
        }
    }

    return RT_TRUE;

}

/*简介：ui队列清理函数
        只用于UI线程尚未成功启动时的初始化失败清理。
         UI线程已经运行后，不得由通用失败函数异步删除队列。
  输入：无
  返回值：无
*/
static void ui_runtime_release_event_queue(void)
{
    rt_mq_t queue;
    rt_base_t level;

    level = rt_hw_interrupt_disable();

    queue = g_ui_event_queue;
    g_ui_event_queue = RT_NULL;
    g_ui_runtime_context.g_ui_event_queue_ready = RT_FALSE;

    rt_hw_interrupt_enable(level);

    if (queue != RT_NULL)
    {
        (void)rt_mq_delete(queue);
    }

}

#ifdef RT_USING_FINSH

/*
 * 简介：受控验证UI消息队列满路径和压力后恢复能力
 *
 * 测试过程：
 * 1. 确认UI已经READY且队列为空；
 * 2. 锁住调度，在UI线程无法消费期间连续发布16条事件；
 * 3. 深度8的队列应当接受8条并安全丢弃8条；
 * 4. 恢复调度，等待成功发布的事件全部被消费；
 * 5. 再发布一条HOME事件，验证队列和页面处理已经恢复。
 *
 * 注意：
 * - 临界区内不打印、不延时、不调用LVGL；
 * - 所有发送均通过公开的ui_event_post()；
 * - 不直接访问或修改消息队列内部链表。
 */
static int ui_event_stress(void)
{
    ui_runtime_stats_t before;
    ui_runtime_stats_t after_burst;
    ui_runtime_stats_t after_drain;
    ui_runtime_stats_t final_stats;
    ui_event_t event;
    rt_err_t result;
    rt_uint32_t index;
    rt_uint32_t send_success;
    rt_uint32_t send_full;
    rt_uint32_t send_other_fail;
    rt_uint32_t attempts_delta;
    rt_uint32_t posted_delta;
    rt_uint32_t consumed_delta;
    rt_uint32_t dropped_delta;
    rt_uint32_t recovery_posted_delta;
    rt_uint32_t recovery_consumed_delta;
    rt_tick_t start_tick;
    rt_tick_t timeout_ticks;
    rt_bool_t drained;
    rt_bool_t recovered;
    rt_bool_t burst_result_ok;

    result = ui_runtime_get_stats(&before);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[UI][STRESS][ERROR] get initial stats failed: %d\n",
            (int)result);

        return (int)result;
    }

    if (before.state != UI_RUNTIME_STATE_READY)
    {
        rt_kprintf(
            "[UI][STRESS][ERROR] UI is not ready: state=%d\n",
            (int)before.state);

        return -RT_EBUSY;
    }

    if ((before.queue_ready != RT_TRUE) ||
        (before.event_queue == RT_NULL))
    {
        rt_kprintf(
            "[UI][STRESS][ERROR] queue is not ready: "
            "queue=%p ready=%u\n",
            before.event_queue,
            (unsigned int)before.queue_ready);

        return -RT_ERROR;
    }

    /*
     * 测试开始前要求队列为空。
     * 避免已有正常业务事件改变预期的8成功、8丢弃结果。
     */
    if (before.posted != before.consumed)
    {
        rt_kprintf(
            "[UI][STRESS][ERROR] queue is not empty: "
            "posted=%u consumed=%u pending=%u\n",
            (unsigned int)before.posted,
            (unsigned int)before.consumed,
            (unsigned int)(before.posted - before.consumed));

        return -RT_EBUSY;
    }

    send_success = 0;
    send_full = 0;
    send_other_fail = 0;

    event.type = UI_ACTION_REFRESH_STATUS;
    event.source = UI_EVENT_SOURCE_TEST;
    event.tick = 0;
    event.value = 0;

    /*
     * 锁住调度但不长时间关闭硬件中断。
     *
     * ui_event_post()仍可执行非阻塞rt_mq_send()，
     * 但UI线程在rt_exit_critical()前不能运行和消费消息，
     * 因而可以确定性填满深度为8的队列。
     */
    rt_enter_critical();

    for (index = 0;
         index < UI_EVENT_STRESS_ATTEMPTS;
         index++)
    {
        event.tick = rt_tick_get();
        event.value = index;

        result = ui_event_post(&event);

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

    /*
     * 必须先恢复调度，再打印日志或等待UI线程消费。
     */
    rt_exit_critical();

    result = ui_runtime_get_stats(&after_burst);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[UI][STRESS][ERROR] get burst stats failed: %d\n",
            (int)result);

        return (int)result;
    }

    attempts_delta =
        after_burst.post_attempt - before.post_attempt;
    posted_delta =
        after_burst.posted - before.posted;
    dropped_delta =
        after_burst.dropped - before.dropped;

    rt_kprintf(
        "[UI][STRESS] burst attempts=%u "
        "success=%u full=%u other_fail=%u\n",
        (unsigned int)UI_EVENT_STRESS_ATTEMPTS,
        (unsigned int)send_success,
        (unsigned int)send_full,
        (unsigned int)send_other_fail);

    rt_kprintf(
        "[UI][STRESS] counter delta: "
        "attempts=%u posted=%u dropped=%u\n",
        (unsigned int)attempts_delta,
        (unsigned int)posted_delta,
        (unsigned int)dropped_delta);

    burst_result_ok = RT_TRUE;

    if (attempts_delta != UI_EVENT_STRESS_ATTEMPTS)
    {
        burst_result_ok = RT_FALSE;
    }

    if (send_success != UI_EVENT_QUEUE_DEPTH)
    {
        burst_result_ok = RT_FALSE;
    }

    if (send_full !=
        (UI_EVENT_STRESS_ATTEMPTS -
         UI_EVENT_QUEUE_DEPTH))
    {
        burst_result_ok = RT_FALSE;
    }

    if (send_other_fail != 0U)
    {
        burst_result_ok = RT_FALSE;
    }

    if (posted_delta != send_success)
    {
        burst_result_ok = RT_FALSE;
    }

    if (dropped_delta != send_full)
    {
        burst_result_ok = RT_FALSE;
    }

    /*
     * 恢复调度后，等待本轮成功发布的消息全部消费。
     */
    timeout_ticks =
        rt_tick_from_millisecond(
            UI_EVENT_STRESS_TIMEOUT_MS);

    if (timeout_ticks == 0)
    {
        timeout_ticks = 1;
    }

    start_tick = rt_tick_get();
    drained = RT_FALSE;

    while ((rt_tick_get() - start_tick) <
           timeout_ticks)
    {
        result = ui_runtime_get_stats(&after_drain);
        if (result != RT_EOK)
        {
            rt_kprintf(
                "[UI][STRESS][ERROR] "
                "get drain stats failed: %d\n",
                (int)result);

            return (int)result;
        }

        consumed_delta =
            after_drain.consumed - before.consumed;

        if (consumed_delta >= send_success)
        {
            drained = RT_TRUE;
            break;
        }

        rt_thread_mdelay(
            UI_EVENT_STRESS_POLL_MS);
    }

    if (drained != RT_TRUE)
    {
        rt_kprintf(
            "[UI][STRESS][ERROR] drain timeout: "
            "expected=%u consumed_delta=%u\n",
            (unsigned int)send_success,
            (unsigned int)(
                after_drain.consumed -
                before.consumed));

        return -RT_ETIMEOUT;
    }

    consumed_delta =
        after_drain.consumed - before.consumed;

    rt_kprintf(
        "[UI][STRESS] drain complete: "
        "posted_delta=%u consumed_delta=%u "
        "handler_delta=%u\n",
        (unsigned int)posted_delta,
        (unsigned int)consumed_delta,
        (unsigned int)(
            after_drain.handler_count -
            before.handler_count));

    if (consumed_delta != send_success)
    {
        burst_result_ok = RT_FALSE;
    }

    if (after_drain.posted !=
        after_drain.consumed)
    {
        rt_kprintf(
            "[UI][STRESS][ERROR] "
            "queue did not return to empty: "
            "posted=%u consumed=%u\n",
            (unsigned int)after_drain.posted,
            (unsigned int)after_drain.consumed);

        burst_result_ok = RT_FALSE;
    }

    if (after_drain.handler_count <=
        before.handler_count)
    {
        rt_kprintf(
            "[UI][STRESS][ERROR] "
            "handler_count did not increase\n");

        burst_result_ok = RT_FALSE;
    }

    /*
     * 压力结束后再发布一条正常HOME事件。
     * 屏幕最终应显示“UI HOME”。
     */
    event.type = UI_ACTION_HOME;
    event.source = UI_EVENT_SOURCE_TEST;
    event.tick = rt_tick_get();
    event.value = 0;

    result = ui_event_post(&event);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[UI][STRESS][ERROR] "
            "recovery HOME post failed: %d\n",
            (int)result);

        return (int)result;
    }

    start_tick = rt_tick_get();
    recovered = RT_FALSE;

    while ((rt_tick_get() - start_tick) <
           timeout_ticks)
    {
        result = ui_runtime_get_stats(&final_stats);
        if (result != RT_EOK)
        {
            rt_kprintf(
                "[UI][STRESS][ERROR] "
                "get recovery stats failed: %d\n",
                (int)result);

            return (int)result;
        }

        if (final_stats.consumed >
            after_drain.consumed)
        {
            recovered = RT_TRUE;
            break;
        }

        rt_thread_mdelay(
            UI_EVENT_STRESS_POLL_MS);
    }

    if (recovered != RT_TRUE)
    {
        rt_kprintf(
            "[UI][STRESS][ERROR] "
            "recovery HOME consume timeout\n");

        return -RT_ETIMEOUT;
    }

    recovery_posted_delta =
        final_stats.posted -
        after_drain.posted;
    recovery_consumed_delta =
        final_stats.consumed -
        after_drain.consumed;

    rt_kprintf(
        "[UI][STRESS] recovery: "
        "posted_delta=%u consumed_delta=%u "
        "dropped_delta=%u\n",
        (unsigned int)recovery_posted_delta,
        (unsigned int)recovery_consumed_delta,
        (unsigned int)(
            final_stats.dropped -
            after_drain.dropped));

    if ((recovery_posted_delta != 1U) ||
        (recovery_consumed_delta != 1U) ||
        (final_stats.dropped !=
         after_drain.dropped))
    {
        rt_kprintf(
            "[UI][STRESS][ERROR] "
            "recovery counters do not match\n");

        return -RT_ERROR;
    }

    if (final_stats.posted !=
        final_stats.consumed)
    {
        rt_kprintf(
            "[UI][STRESS][ERROR] "
            "pending event remains after recovery\n");

        return -RT_ERROR;
    }

    if (burst_result_ok != RT_TRUE)
    {
        rt_kprintf(
            "[UI][STRESS][FAIL] "
            "burst result does not match "
            "depth=%u attempts=%u\n",
            (unsigned int)UI_EVENT_QUEUE_DEPTH,
            (unsigned int)UI_EVENT_STRESS_ATTEMPTS);

        return -RT_ERROR;
    }

    rt_kprintf(
        "[UI][STRESS][PASS] "
        "success=%u dropped=%u consumed=%u "
        "recovery=ok\n",
        (unsigned int)send_success,
        (unsigned int)send_full,
        (unsigned int)consumed_delta);

    return 0;
}

#endif

/*简介：shell函数
  输入：无
  返回值：rt_err_t：错误类型
*/
static int ui_stats(void)
{
    ui_runtime_stats_t stats;
    ui_page_stats_t ui_page_stats;
    rt_err_t result;

    result = ui_runtime_get_stats(&stats);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[UI][STATS][ERROR] get stats failed: %d\n",
            (int)result);
        return (int)result;
    }

    rt_kprintf(
        "[UI][STATS] state=%d thread=%p ready_tick=%u\n",
        (int)stats.state,
        stats.thread,
        (unsigned int)stats.ready_tick);

    rt_kprintf(
        "[UI][STATS] init_attempt=%u init_success=%u init_fail=%u\n",
        (unsigned int)stats.init_attempt,
        (unsigned int)stats.init_success,
        (unsigned int)stats.init_fail);

    rt_kprintf(
        "[UI][STATS] handler_count=%u "
        "last_return=%u last_wait=%u min_wait=%u max_wait=%u\n",
        (unsigned int)stats.handler_count,
        (unsigned int)stats.last_handler_return_ms,
        (unsigned int)stats.last_wait_ms,
        (unsigned int)stats.min_wait_ms,
        (unsigned int)stats.max_wait_ms);

    rt_kprintf(
        "[UI][STATS] failure_stage=%d last_error=%d\n",
        (int)stats.failure_stage,
        (int)stats.last_error);

    rt_kprintf(
        "[UI][STATS] display=%p draw_buf=%p buf1=%p buf2=%p\n",
        stats.display,
        stats.draw_buf,
        stats.buf1,
        stats.buf2);

    rt_kprintf(
        "[UI][STATS] pixels=%u bytes=%u resolution=%ux%u\n",
        (unsigned int)stats.draw_buf_pixels,
        (unsigned int)stats.draw_buf_bytes,
        (unsigned int)stats.hor_res,
        (unsigned int)stats.ver_res);

    rt_uint32_t pending;

    /*
     * 正常稳定状态应当posted >= consumed。
     * 并发查询瞬间可能观察到消费者先更新的短暂窗口，
     * 因此这里避免无符号减法下溢。
     */
    if (stats.posted >= stats.consumed)
    {
        pending = stats.posted - stats.consumed;
    }
    else
    {
        pending = 0;
    }
    rt_kprintf(
        "[UI][EVENT] queue=%p ready=%u depth=%u\n",
        stats.event_queue,
        (unsigned int)stats.queue_ready,
        (unsigned int)stats.queue_depth);

    rt_kprintf(
        "[UI][EVENT] attempts=%u posted=%u "
        "consumed=%u pending=%u\n",
        (unsigned int)stats.post_attempt,
        (unsigned int)stats.posted,
        (unsigned int)stats.consumed,
        (unsigned int)pending);

    rt_kprintf(
        "[UI][EVENT] dropped=%u invalid=%u "
        "handled=%u handle_fail=%u\n",
        (unsigned int)stats.dropped,
        (unsigned int)stats.invalid,
        (unsigned int)stats.handled,
        (unsigned int)stats.handle_fail);

    rt_kprintf(
        "[UI][EVENT] late=%u recv_fail=%u "
        "last_latency_ticks=%u max_latency_ticks=%u\n",
        (unsigned int)stats.late,
        (unsigned int)stats.recv_fail,
        (unsigned int)stats.last_latency_ticks,
        (unsigned int)stats.max_latency_ticks);

    result = ui_page_get_stats(&ui_page_stats);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[UI][STATS][ERROR] ui page get stats failed: %d\n",
            (int)result);
        return (int)result;
    }

    rt_kprintf("[UI][PAGE] created=%d current=%d\n",
               (int)ui_page_stats.created,
               (int)ui_page_stats.current_page);

    rt_kprintf("[UI][PAGE] create attempt=%u success=%u fail=%u\n",
               (unsigned int)ui_page_stats.create_attempt,
               (unsigned int)ui_page_stats.create_success,
               (unsigned int)ui_page_stats.create_fail);

    rt_kprintf("[UI][PAGE] switch attempt=%u success=%u fail=%u noop=%u\n",
               (unsigned int)ui_page_stats.switch_attempt,
               (unsigned int)ui_page_stats.switch_success,
               (unsigned int)ui_page_stats.switch_fail,
               (unsigned int)ui_page_stats.switch_noop);



    return 0;
}

#ifdef RT_USING_FINSH

MSH_CMD_EXPORT(
    ui_stats,
    show UI runtime and LVGL buffer statistics);

MSH_CMD_EXPORT(
    ui_event_stress,
    verify UI event queue full and recovery paths);

#endif
