#include "ui_runtime.h"
#include "ui_page.h"
#include "rthw.h"
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lv_ex_data.h"

#ifdef RT_USING_FINSH
#include "finsh.h"
#endif

#define UI_THREAD_NAME          "ui"
#define UI_THREAD_STACK_SIZE    (16U * 1024U)
#define UI_THREAD_PRIORITY      RT_THREAD_PRIORITY_MIDDLE
#define UI_THREAD_TIMESLICE     10U

#define UI_EXPECTED_WIDTH       390U
#define UI_EXPECTED_HEIGHT      450U

#define UI_WAIT_MIN_MS          1U
#define UI_WAIT_MAX_MS          20U

//ui线程
static struct rt_thread g_ui_thread;

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t g_ui_thread_stack[UI_THREAD_STACK_SIZE];

//ui运行时上下文
typedef struct
{
    ui_runtime_event_handler_t g_ui_event_handler;  //回调函数句柄
    void *g_ui_event_user_data;                     //回调函数私有资源

    rt_bool_t g_ui_wait_sample_valid;               //线程等待时间样本值是否有效
    ui_runtime_stats_t stats;                       //统计信息

} ui_runtime_context_t;
static ui_runtime_context_t g_ui_runtime_context =
{
    .stats.state = UI_RUNTIME_STATE_STOPPED,

    .g_ui_event_handler=RT_NULL,
    .g_ui_event_user_data=RT_NULL,

    .g_ui_wait_sample_valid=RT_FALSE,

    .stats.last_error = RT_EOK,
    .stats.failure_stage = UI_FAILURE_NONE

};



//公开函数声明
rt_err_t ui_runtime_init(void);                                                                     //启动静态线程
rt_err_t ui_runtime_register_handler(ui_runtime_event_handler_t handler,void *user_data);           //回调注册函数
ui_runtime_state_t ui_runtime_get_state(void);                                                      //查询ui模块状态
rt_err_t ui_runtime_get_stats(ui_runtime_stats_t *stats);                                           //获取ui模块信息

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
         * TODO：
         * 保存THREAD_INIT；
         * init_fail加1；
         * 设置FAILED；
         * 打印错误；
         * 通知UI_FAILED；
         * 返回原始result。
         */
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

    //启动UI吸线程
    result = rt_thread_startup(&g_ui_thread);
    if (result != RT_EOK)
    {
        /*
         * TODO：
         * 保存THREAD_STARTUP；
         * init_fail加1；
         * 设置FAILED；
         * 打印并通知；
         * 返回原始result。
         */
        ui_runtime_thread_fail(UI_FAILURE_THREAD_STARTUP,result);
        return result;
    }

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

    /* TODO：整体复制g_ui_stats。 */
    *stats = g_ui_runtime_context.stats;
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

    rt_thread_mdelay(wait_ms);

    while (1)
    {
        /*
         * TODO：
         * 调用lv_timer_handler()；
         * 计算1～20 ms等待值；
         * 更新统计；
         * 延时。
         */
        handler_return_ms=lv_task_handler();
        wait_ms=ui_runtime_normalize_wait(handler_return_ms);
        //更新handle统计信息
        ui_runtime_update_handler_stats(handler_return_ms,wait_ms);
        rt_thread_mdelay(wait_ms);

    }
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
     * TODO：
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
    rt_kprintf("[UI][error] ui thread failed_stage:%d\n",stage);
    rt_kprintf("[UI][error] ui thread failed_error:%d\n",error);

    ui_runtime_notify(UI_RUNTIME_EVENT_FAILED,stage,error);

    /*
     * 不关闭LCD；
     * 不自动重试；
     * 不调用LVGL页面接口；
     * 返回UI线程入口后，线程自然结束。
     */
}

/*简介：shell函数
  输入：无
  返回值：rt_err_t：错误类型
*/
static int ui_stats(void)
{
    ui_runtime_stats_t stats;
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

    return 0;
}

#ifdef RT_USING_FINSH
MSH_CMD_EXPORT(ui_stats,
               show UI runtime and LVGL buffer statistics);
#endif
