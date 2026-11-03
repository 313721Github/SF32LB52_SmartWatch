#include "time_service.h"
#include "time_calendar.h"
#include "rtc_port.h"
#include "rthw.h"

#include <string.h>

#define TIME_THREAD_STACK_SIZE       2048U  // time线程栈大小，单位：字节
#define TIME_THREAD_PRIORITY         21U    // 线程优先级，数值越小优先级越高
#define TIME_THREAD_TIMESLICE         10U   // 同优先级线程的调度时间片，单位：tick

#define TIME_REQUEST_QUEUE_DEPTH      4U    // 请求队列最多容纳4条等待消息
#define TIME_RESULT_CAPACITY          8U    // 最多保留8条请求记录，在途记录不能覆盖

#define TIME_POLL_MS                250U    // 正常RTC读取周期，单位：毫秒
#define TIME_RETRY_MS              1000U    // RTC适配初始化失败后的重试间隔，单位：毫秒
#define TIME_STALE_MS              1500U    // 有效读值的新鲜度期限，单位：毫秒

#define TIME_FAULT_MAX_COUNT         16U    // 单次配置允许的最大故障注入次数
#define TIME_TRACE_MAX_SECONDS     3600U    // 后台跟踪允许的最大时长，单位：秒

#define TIME_SET_VERIFY_MAX_MS       2000U
#define TIME_JUMP_TOLERANCE_SECONDS     2U

// 队列中的控制请求；kind决定union中哪个成员有效。
typedef struct
{
    uint32_t request_id;        // 非零请求序号，用于关联处理结果
    time_request_kind_t kind;   // 请求类型

    union
    {
        struct
        {
            uint32_t utc_seconds;   // 请求设置的UTC秒值
            time_source_t source;   // 校时来源
        } set;                      // SET_UTC请求参数

        struct
        {
            time_test_fault_t fault; // 软件故障注入种类
            uint32_t count;          // 注入次数，按约定可用于清除
        } fault;                     // FAULT请求参数

        struct
        {
            uint32_t duration_seconds; // 跟踪总时长，0表示停止
            uint32_t interval_seconds; // 日志输出间隔，单位：秒
        } trace;                       // TRACE请求参数
    } data;                            // 不同请求共用的参数存储空间
} time_request_t;

// HCPU线程间共享；短临界区内只复制或修改值，不执行RTC、日志或分配操作。
typedef struct
{
    rt_bool_t initializing;     // 是否正在初始化，防止重复创建资源
    rt_bool_t started;          // 服务线程是否已成功启动，不代表RTC已校时

    uint32_t next_request_id;   // 请求序号生成状态，分配时跳过0并避免ID冲突
    uint32_t next_result_slot;  // 结果槽扫描起点，分配时不能覆盖在途请求

    time_snapshot_t snapshot;   // 最新时间与状态快照
    time_service_stats_t stats; // 服务累计统计
    time_request_result_t results[TIME_RESULT_CAPACITY]; // 有限容量的请求记录
} time_shared_t;

// 仅time线程访问，UI和Shell不能直接读写其中状态。
typedef struct
{
    rt_bool_t port_ready;       // RTC适配是否已初始化，不代表日历可信
    rt_tick_t next_poll_tick;   // 下一次正常读取的截止tick
    rt_tick_t next_retry_tick;  // 下一次适配初始化重试的截止tick

    uint32_t fault_remaining[TIME_FAULT_COUNT]; // 各类故障剩余注入次数，NONE下标不使用

    rt_bool_t trace_enabled;        // 是否启用后台跟踪
    rt_tick_t trace_end_tick;       // 跟踪结束的截止tick
    rt_tick_t next_trace_tick;      // 下一次日志输出的截止tick
    rt_tick_t trace_interval_ticks; // 跟踪输出间隔，单位：tick
} time_worker_t;

static rt_mq_t g_time_queue = RT_NULL;       // 时间服务请求队列句柄
static rt_thread_t g_time_thread = RT_NULL;  // 唯一time线程句柄

static time_shared_t g_time_shared; // 线程间共享状态，静态初始化为0
static time_worker_t g_time_worker; // time线程私有运行状态，静态初始化为0

//公开函数声明
rt_err_t time_service_init(void);// 创建服务资源并启动唯一time线程；成功不代表RTC已校时。

rt_err_t time_service_request_set_utc(uint32_t utc_seconds,
                                    time_source_t source,
                                    uint32_t *request_id);      // 非阻塞提交UTC校时请求；RT_EOK仅表示受理，执行结果按ID查询。

rt_err_t time_service_test_inject(time_test_fault_t fault,
                                    uint32_t count,
                                    uint32_t *request_id);      // 提交有限次数的软件故障注入配置；默认关闭，执行结果按ID查询

rt_err_t time_service_request_trace(uint32_t duration_seconds,
                                    uint32_t interval_seconds,
                                    uint32_t *request_id);      // 提交后台跟踪配置，立即返回；不占用Shell等待跟踪结束
rt_err_t time_service_get_snapshot(time_snapshot_t *snapshot);  // 复制时间快照并计算当前age；输出不可为空，可信性须检查快照字段。
rt_err_t time_service_get_stats(time_service_stats_t *stats);   // 复制累计统计；输出不可为空，不返回内部结构指针。

rt_err_t time_service_get_result(uint32_t request_id,
                            time_request_result_t *result);     // 查询仍被保留的请求记录；RT_EOK表示找到记录，还须检查state和error。


//私有函数声明
static rt_tick_t time_service_ms_to_ticks(uint32_t milliseconds);           // 将正数毫秒间隔向上换算为至少1tick
static rt_bool_t time_service_tick_due(rt_tick_t now, rt_tick_t deadline);  // 回绕安全地判断是否到期，比较跨度须小于半个tick量程
static rt_err_t time_service_post_request(time_request_t *request,
                                            uint32_t *request_id);          // 预留请求记录并非阻塞入队；成功仅表示受理，失败时输出ID为0
static void time_service_thread_entry(void *parameter);                     // 时间线程入口，统一调度RTC访问、请求处理、重探和后台跟踪
static void time_service_handle_request(const time_request_t *request);     // 根据kind分派控制请求，并关联其处理状态
static void time_service_handle_set(const time_request_t *request);         // 执行UTC写入及读回确认，管理校时代际和可信状态
static void time_service_poll(rt_tick_t now);                               // 执行到期的RTC读取或适配重试，记录结果及耗时
static rt_err_t time_service_publish_read(uint32_t utc_seconds,
                                            rt_tick_t sample_tick,
                                            time_source_t source,
                                            time_change_reason_t reason);   // 校验并发布时间读值，处理日期变化、可信状态和异常跳变
static void time_service_finish_request(uint32_t request_id,
                                        rt_err_t error,
                                        rt_bool_t readback_available,
                                        uint32_t readback_utc);             // 按ID提交请求终态，保存错误和实际取得的读回候选值
static void time_service_trace_if_due(rt_tick_t now);                       // 到达跟踪周期时输出日志，达到结束时间后自动停止

//---------------------------------------------------------------公开函数---------------------------------------------------------------
/*
 * 简介：创建时间服务资源并启动唯一线程。
 * 输入：无。
 * 返回值：已启动或启动成功返回RT_EOK；创建失败返回负错误码。
 * 说明：只建立资源，不同步读取RTC；失败释放本次部分资源，保留其他业务。
 */
rt_err_t time_service_init(void)
{
    rt_base_t level;
    rt_mq_t queue = RT_NULL;
    rt_thread_t thread = RT_NULL;
    rt_err_t result;

    // 原子检查初始化状态，避免两个调用者重复创建资源。
    level = rt_hw_interrupt_disable();

    if (g_time_shared.started == RT_TRUE)
    {
        rt_hw_interrupt_enable(level);
        return RT_EOK;
    }

    if (g_time_shared.initializing == RT_TRUE)
    {
        rt_hw_interrupt_enable(level);
        return -RT_EBUSY;
    }

    // 未启动却残留资源句柄，不能直接覆盖。
    if ((g_time_queue != RT_NULL) || (g_time_thread != RT_NULL))
    {
        rt_hw_interrupt_enable(level);
        return -RT_EBUSY;
    }

    g_time_shared.initializing = RT_TRUE;

    // 只在首次资源创建尝试时建立初始快照。
    if (g_time_shared.stats.init_attempts == 0U)
    {
        memset(&g_time_shared.snapshot,0,sizeof(g_time_shared.snapshot));

        g_time_shared.snapshot.state = TIME_CLOCK_BOOT;
        g_time_shared.snapshot.source = TIME_SOURCE_NONE;
    }

    g_time_shared.stats.init_attempts++;

    rt_hw_interrupt_enable(level);

    // 此时尚无运行中的worker，可以初始化其私有状态。
    memset(&g_time_worker, 0, sizeof(g_time_worker));
    // 创建请求队列；内存申请不能放在关中断临界区中。
    queue = rt_mq_create("time_req",
                        sizeof(time_request_t),
                        TIME_REQUEST_QUEUE_DEPTH,
                        RT_IPC_FLAG_FIFO);
    if (queue == RT_NULL)
    {
        result = -RT_ENOMEM;
        goto init_failed;
    }

    // 创建线程，此时线程尚未运行。
    thread = rt_thread_create("time",
                            time_service_thread_entry,
                            RT_NULL,
                            TIME_THREAD_STACK_SIZE,
                            TIME_THREAD_PRIORITY,
                            TIME_THREAD_TIMESLICE);
    if (thread == RT_NULL)
    {
        result = -RT_ENOMEM;
        goto init_failed;
    }
    // startup可能立即触发调度，必须提前准备worker使用的全局句柄。
    level = rt_hw_interrupt_disable();

    g_time_queue = queue;
    g_time_thread = thread;

    // 清除上次资源初始化失败信息；RTC状态随后由worker更新。
    g_time_shared.snapshot.error_stage = TIME_ERROR_NONE;
    g_time_shared.snapshot.last_error = RT_EOK;

    rt_hw_interrupt_enable(level);

    result = rt_thread_startup(thread);
    if (result != RT_EOK)
    {
        goto init_failed;
    }

    // 只发布启动标志，不覆盖worker可能已经更新的快照。
    level = rt_hw_interrupt_disable();
    g_time_shared.started = RT_TRUE;
    g_time_shared.initializing = RT_FALSE;

    rt_hw_interrupt_enable(level);

    return RT_EOK;
init_failed:

    // 保持initializing为真，清理结束前不允许另一个调用者重试。
    level = rt_hw_interrupt_disable();

    g_time_queue = RT_NULL;
    g_time_thread = RT_NULL;

    rt_hw_interrupt_enable(level);

    // 本函数失败路径只处理尚未成功启动的本次资源。
    // 先删除线程，避免它保留对即将删除队列的访问。
    if (thread != RT_NULL)
    {
        (void)rt_thread_delete(thread);
    }

    if (queue != RT_NULL)
    {
        (void)rt_mq_delete(queue);
    }

    level = rt_hw_interrupt_disable();

    g_time_shared.stats.init_failures++;

    g_time_shared.snapshot.state = TIME_CLOCK_BOOT;
    g_time_shared.snapshot.trusted = false;
    g_time_shared.snapshot.fresh = false;
    g_time_shared.snapshot.error_stage = TIME_ERROR_INIT;
    g_time_shared.snapshot.last_error = result;
    g_time_shared.snapshot.publish_seq++;

    g_time_shared.started = RT_FALSE;
    g_time_shared.initializing = RT_FALSE;

    rt_hw_interrupt_enable(level);

    return result;
}

/*
 * 简介：提交一次UTC校时请求。
 * 输入：UTC须在约定范围；source为SHELL或BLE；request_id不可为空。
 * 返回值：RT_EOK仅表示受理；参数非法返回-RT_EINVAL；
 *         其他投递失败返回实际错误码，失败时输出ID为0。
 * 说明：仅线程调用；校验或投递失败不触碰RTC、不改变日历代际。
 */
rt_err_t time_service_request_set_utc(uint32_t utc_seconds,time_source_t source,uint32_t *request_id)
{
    time_request_t request = {0};
    rt_base_t level;

    // 有效输出指针在失败路径中保持为0。
    if (request_id != RT_NULL)
    {
        *request_id = 0U;
    }

    // 明确检查允许来源，拒绝NONE、COUNT以及其他非法枚举值。
    if ((request_id == RT_NULL) ||
        (utc_seconds < TIME_UTC_MIN_SECONDS) ||(utc_seconds > TIME_UTC_MAX_SECONDS) ||
        ((source != TIME_SOURCE_SHELL) &&(source != TIME_SOURCE_BLE)))
    {
        level = rt_hw_interrupt_disable();
        g_time_shared.stats.invalid_requests++;
        rt_hw_interrupt_enable(level);

        return -RT_EINVAL;
    }

    // 构造值类型消息，不携带调用者缓冲区指针。
    request.kind = TIME_REQUEST_SET_UTC;
    request.data.set.utc_seconds = utc_seconds;
    request.data.set.source = source;

    // ID分配、结果槽预留、队列投递和投递统计统一由内部函数负责。
    return time_service_post_request(&request, request_id);
}
/*
 * 简介：提交有限次数的软件故障注入配置。
 * 输入：fault为合法种类，count为0～16；NONE配0清除全部；
 *       具体故障配0关闭该种注入；request_id不可为空。
 * 返回值：RT_EOK仅表示受理；参数非法返回-RT_EINVAL；
 *         其他投递失败返回实际错误码，失败时输出ID为0。
 * 说明：仅worker修改故障预算，不移除真实设备、不改硬件供电。
 */
rt_err_t time_service_test_inject(time_test_fault_t fault,uint32_t count,uint32_t *request_id)
{
    time_request_t request = {0};
    rt_base_t level;

    if (request_id != RT_NULL)
    {
        *request_id = 0U;
    }

    // 转为无符号后比较，也能拒绝传入的负枚举值。
    if ((request_id == RT_NULL) ||
        ((uint32_t)fault >= (uint32_t)TIME_FAULT_COUNT) ||
        (count > TIME_FAULT_MAX_COUNT) ||
        ((fault == TIME_FAULT_NONE) && (count != 0U)))
    {
        level = rt_hw_interrupt_disable();
        g_time_shared.stats.invalid_requests++;
        rt_hw_interrupt_enable(level);

        return -RT_EINVAL;
    }

    // 只构造控制消息，此处不直接修改g_time_worker。
    request.kind = TIME_REQUEST_FAULT;
    request.data.fault.fault = fault;
    request.data.fault.count = count;

    return time_service_post_request(&request, request_id);
}
/*
 * 简介：提交后台时间跟踪配置。
 * 输入：启动时duration为1～3600秒，interval为1～duration秒；
 *       duration为0表示停止，此时忽略interval；request_id不可为空。
 * 返回值：RT_EOK仅表示受理；参数非法返回-RT_EINVAL；
 *         其他投递失败返回实际错误码，失败时输出ID为0。
 * 说明：非阻塞投递；仅worker修改跟踪状态，不占用FinSH等待跟踪结束。
 */
rt_err_t time_service_request_trace(uint32_t duration_seconds,uint32_t interval_seconds,uint32_t *request_id)
{
    time_request_t request = {0};
    uint64_t duration_ticks;
    uint64_t interval_ticks;
    rt_base_t level;

    if (request_id != RT_NULL)
    {
        *request_id = 0U;
    }

    // 停止请求不检查间隔；启动请求要求间隔至少1秒且不超过总时长。
    if ((request_id == RT_NULL) ||
        (duration_seconds > TIME_TRACE_MAX_SECONDS) ||
        ((duration_seconds != 0U) && ((interval_seconds == 0U) ||(interval_seconds > duration_seconds))))
    {
        goto invalid_request;
    }

    if (duration_seconds != 0U)
    {
        // 秒数直接乘tick频率，无需先转换为毫秒。
        duration_ticks = (uint64_t)duration_seconds * (uint64_t)RT_TICK_PER_SECOND;

        interval_ticks = (uint64_t)interval_seconds * (uint64_t)RT_TICK_PER_SECOND;

        // 截止时间使用有符号差值比较，跨度必须小于32位tick半量程。
        if ((duration_ticks == 0U) ||
            (interval_ticks == 0U) ||
            (duration_ticks > (uint64_t)INT32_MAX) ||
            (interval_ticks > (uint64_t)INT32_MAX))
        {
            goto invalid_request;
        }
    }

    request.kind = TIME_REQUEST_TRACE;
    request.data.trace.duration_seconds = duration_seconds;

    // 将停止请求规范化，worker只需根据duration是否为0判断。
    request.data.trace.interval_seconds =(duration_seconds == 0U) ? 0U : interval_seconds;

    return time_service_post_request(&request, request_id);

invalid_request:

    level = rt_hw_interrupt_disable();
    g_time_shared.stats.invalid_requests++;
    rt_hw_interrupt_enable(level);

    return -RT_EINVAL;
}

/*
 * 简介：取得调用时刻的一致时间副本。
 * 输入：snapshot不可为空，输出缓冲由调用者持有。
 * 返回值：成功复制返回RT_EOK；空指针返回-RT_EINVAL。
 * 说明：仅线程调用；锁外计算新鲜度，不访问RTC、不修改共享快照。
 *       无有效样本时age_ms为UINT32_MAX，必须结合has_value解释。
 */
rt_err_t time_service_get_snapshot(time_snapshot_t *snapshot)
{
    rt_base_t level;
    rt_tick_t now;
    rt_tick_t elapsed;
    uint64_t age_ms;
    time_snapshot_t value;

    if (snapshot == RT_NULL)
    {
        return -RT_EINVAL;
    }

    // 在同一个短临界区内复制快照和服务启动标志。
    level = rt_hw_interrupt_disable();

    value = g_time_shared.snapshot;
    value.service_running = (g_time_shared.started == RT_TRUE);

    rt_hw_interrupt_enable(level);

    if (value.has_value)
    {
        // 在复制之后读取now，避免复制到比now更新的sample_tick。
        now = rt_tick_get();

        // 无符号减法支持正常tick回绕。
        elapsed = now - (rt_tick_t)value.sample_tick;

        // 先提升到64位，避免乘1000时溢出。
        age_ms = (uint64_t)elapsed * UINT64_C(1000) /
                 (uint64_t)RT_TICK_PER_SECOND;

        // 输出字段只有32位，超范围时饱和，不能截断成较小年龄。
        if (age_ms > (uint64_t)UINT32_MAX)
        {
            value.age_ms = UINT32_MAX;
        }
        else
        {
            value.age_ms = (uint32_t)age_ms;
        }

        // 达到期限即过期；服务未启动时也不能报告新鲜。
        value.fresh = value.service_running &&(age_ms < (uint64_t)TIME_STALE_MS);

        // 只投影普通可读状态，保留更明确的校时或故障状态。
        if ((age_ms >= (uint64_t)TIME_STALE_MS) &&
            ((value.state == TIME_CLOCK_READY) || (value.state == TIME_CLOCK_READ_ERROR)))
        {
            value.state = TIME_CLOCK_STALE;
        }
    }
    else
    {
        // 从未取得合法读值，年龄无有效含义，也不能认为新鲜。
        value.age_ms = UINT32_MAX;
        value.fresh = false;
    }

    // 不自动建立或撤销可信来源；新鲜度和可信来源分别表达。
    *snapshot = value;

    return RT_EOK;
}

/*
 * 简介：复制时间服务累计统计。
 * 输入：stats不可为空，输出由调用者持有。
 * 返回值：成功返回RT_EOK；参数错误返回-RT_EINVAL。
 * 说明：短临界区内只复制，不打印、不访问设备。
 */
rt_err_t time_service_get_stats(time_service_stats_t *stats)
{
    rt_base_t level;

    if (stats == RT_NULL)
    {
        return -RT_EINVAL;
    }

    // 保证各项统计来自同一个复制时刻。
    level = rt_hw_interrupt_disable();

    *stats = g_time_shared.stats;

    rt_hw_interrupt_enable(level);

    return RT_EOK;
}
/*
 * 简介：按序号查询一条仍被保留的请求记录。
 * 输入：request_id必须非零；result不可为空，归调用者所有。
 * 返回值：找到返回RT_EOK；参数错误返回-RT_EINVAL；
 *         已回收或不存在返回-RT_EEMPTY，失败时不修改输出。
 * 说明：RT_EOK仅表示找到记录，还须检查state判断请求是否完成。
 */
rt_err_t time_service_get_result(uint32_t request_id,time_request_result_t *result)
{
    rt_base_t level;
    uint32_t index;
    time_request_result_t value;

    if ((request_id == 0U) || (result == RT_NULL))
    {
        return -RT_EINVAL;
    }

    level = rt_hw_interrupt_disable();

    // 扫描期间保持保护，避免记录同时被完成或回收。
    for (index = 0U; index < TIME_RESULT_CAPACITY; index++)
    {
        if (g_time_shared.results[index].request_id == request_id)
        {
            value = g_time_shared.results[index];

            rt_hw_interrupt_enable(level);

            // 使用局部副本，恢复中断后再写入调用者缓冲区。
            *result = value;

            return RT_EOK;
        }
    }

    rt_hw_interrupt_enable(level);

    // 不返回其他请求的结果，也不修改调用者原有内容。
    return -RT_EEMPTY;
}



//---------------------------------------------------------------私有函数---------------------------------------------------------------
/*
 * 简介：运行时间服务调度循环。
 * 输入：parameter不使用，不保存外部指针。
 * 返回值：无。
 * 说明：唯一项目RTC调用上下文；通过poll处理初始化和读取，
 *       不持共享临界区执行设备访问、消息等待或日志输出。
 */
static void time_service_thread_entry(void *parameter)
{
    time_request_t request;
    rt_tick_t now;
    rt_tick_t deadline;
    rt_tick_t wait_ticks;
    rt_tick_t candidate_wait;
    rt_tick_t last_error_log_tick = 0U;
    rt_bool_t error_logged = RT_FALSE;
    rt_base_t level;
    rt_err_t result;

    (void)parameter;

    // 首轮立即进入poll，由它尝试初始化RTC适配。
    now = rt_tick_get();

    g_time_worker.port_ready = RT_FALSE;
    g_time_worker.next_poll_tick = now;
    g_time_worker.next_retry_tick = now;

    rt_kprintf("[TIME][THREAD] started\n");

    while (1)
    {
        now = rt_tick_get();

        // 设备就绪时检查读取deadline，否则检查初始化重试deadline。
        deadline = g_time_worker.port_ready ?g_time_worker.next_poll_tick : g_time_worker.next_retry_tick;

        if (time_service_tick_due(now, deadline) == RT_TRUE)
        {
            // poll必须在所有执行路径中推进相应deadline。
            time_service_poll(now);

            // 工作返回后再记录完成时间，不提前冒充有效进展。
            now = rt_tick_get();

            level = rt_hw_interrupt_disable();
            g_time_shared.stats.worker_completed_tick = now;
            rt_hw_interrupt_enable(level);
        }

        // 前面的设备操作可能耗时，使用新的tick检查跟踪任务。
        now = rt_tick_get();
        time_service_trace_if_due(now);

        // 跟踪输出也可能耗时，等待前再次读取当前tick。
        now = rt_tick_get();

        deadline = g_time_worker.port_ready ? g_time_worker.next_poll_tick : g_time_worker.next_retry_tick;

        // 已经到期时不继续等待，下一轮立即处理。
        if (time_service_tick_due(now, deadline) == RT_TRUE)
        {
            wait_ticks = 0U;
        }
        else
        {
            wait_ticks = deadline - now;
        }

        if (g_time_worker.trace_enabled == RT_TRUE)
        {
            // 跟踪结束时也需要及时唤醒，不能只关注下一次输出。
            deadline = g_time_worker.trace_end_tick;

            if (time_service_tick_due(now, deadline) == RT_TRUE)
            {
                candidate_wait = 0U;
            }
            else
            {
                candidate_wait = deadline - now;
            }

            if (candidate_wait < wait_ticks)
            {
                wait_ticks = candidate_wait;
            }

            // 再与下一次跟踪输出时间比较，选择最早截止点。
            deadline = g_time_worker.next_trace_tick;

            if (time_service_tick_due(now, deadline) == RT_TRUE)
            {
                candidate_wait = 0U;
            }
            else
            {
                candidate_wait = deadline - now;
            }

            if (candidate_wait < wait_ticks)
            {
                wait_ticks = candidate_wait;
            }
        }

        // 新请求到达可立即唤醒；无请求则等到最近的deadline。
        // 所有配置间隔均须小于INT32_MAX tick，转换后不会变为负超时。
        result = rt_mq_recv(g_time_queue,
                            &request,
                            sizeof(request),
                            (rt_int32_t)wait_ticks);

        if (result == RT_EOK)
        {
            // 每轮只处理一条，随后回到顶部检查周期任务。
            time_service_handle_request(&request);

            now = rt_tick_get();

            level = rt_hw_interrupt_disable();
            g_time_shared.stats.worker_completed_tick = now;
            rt_hw_interrupt_enable(level);
        }
        else if (result == -RT_ETIMEOUT)
        {
            // 正常到期唤醒，下一轮检查轮询、重探和跟踪任务。
            // 单纯等待超时不算完成了一项业务。
        }
        else
        {
            now = rt_tick_get();

            // 异常接收错误限频输出，避免故障期间持续刷屏。
            if ((error_logged == RT_FALSE) ||
                ((rt_tick_t)(now - last_error_log_tick) >=
                 time_service_ms_to_ticks(TIME_RETRY_MS)))
            {
                rt_kprintf("[TIME][QUEUE][ERROR] receive failed: %d\n",
                           (int)result);

                last_error_log_tick = now;
                error_logged = RT_TRUE;
            }

            // 异常路径适当退避，避免立即失败后不断重试。
            (void)rt_thread_delay(time_service_ms_to_ticks(TIME_POLL_MS));
        }
    }
}

/*
 * 简介：分派一条已受理的时间服务控制请求。
 * 输入：request应为非空的worker局部消息副本，仅在调用期间读取，
 *       本函数不保存该指针；空指针或零ID作为内部非法请求拒绝。
 * 返回值：无。
 * 说明：仅time线程调用；按ID将PENDING记录改为RUNNING后分派。
 *       SET_UTC由handle_set提交终态；FAULT和TRACE在本函数完成。
 *       临界区内只访问共享记录，不调用RTC、日志或完成函数。
 */
static void time_service_handle_request(const time_request_t *request)
{
    rt_base_t level;
    rt_tick_t now;
    uint32_t index;
    uint32_t slot = TIME_RESULT_CAPACITY;

    rt_bool_t can_run = RT_FALSE;
    rt_bool_t kind_matches = RT_FALSE;

    uint64_t duration_ticks;
    uint64_t interval_ticks;

    /* 无有效ID时，无法关联结果记录，只统计并拒绝执行。 */
    if ((request == RT_NULL) || (request->request_id == 0U))
    {
        level = rt_hw_interrupt_disable();
        g_time_shared.stats.invalid_requests++;
        rt_hw_interrupt_enable(level);

        rt_kprintf("[TIME][REQUEST][ERROR] null request or zero ID\n");
        return;
    }

    /*
     * 先查结果槽，再把等待状态改为处理中。
     * 已完成或已经处理中的记录不能被重复执行。
     */
    level = rt_hw_interrupt_disable();

    for (index = 0U; index < TIME_RESULT_CAPACITY; index++)
    {
        if (g_time_shared.results[index].request_id == request->request_id)
        {
            slot = index;
            break;
        }
    }

    if ((slot < TIME_RESULT_CAPACITY) && (g_time_shared.results[slot].state == TIME_RESULT_PENDING))
    {
        kind_matches =(g_time_shared.results[slot].kind == request->kind) ? RT_TRUE : RT_FALSE;

        g_time_shared.results[slot].state = TIME_RESULT_RUNNING;

        can_run = RT_TRUE;
    }

    if (can_run == RT_FALSE)
    {
        g_time_shared.stats.invalid_requests++;
    }

    rt_hw_interrupt_enable(level);

    /*
     * 找不到对应等待记录，或者这是重复消息：
     * 不执行副作用，也不改写已有终态。
     */
    if (can_run == RT_FALSE)
    {
        rt_kprintf("[TIME][REQUEST][ERROR] no pending record, id=%u\n",
                   (unsigned int)request->request_id);
        return;
    }

    /* ID对应的记录类型与消息类型不同，安全结束为失败。 */
    if (kind_matches == RT_FALSE)
    {
        goto invalid_request;
    }

    switch (request->kind)
    {
    case TIME_REQUEST_SET_UTC:
        /*
         * 校时包含写入、读回和可信状态处理。
         * handle_set负责成功或失败的终态，本函数不能重复完成。
         */
        time_service_handle_set(request);
        return;

    case TIME_REQUEST_FAULT:
        /*
         * 投递入口已经校验，这里仍保护数组边界，
         * 避免非法枚举被当成数组下标。
         */
        if (((uint32_t)request->data.fault.fault >= (uint32_t)TIME_FAULT_COUNT) ||
            (request->data.fault.count > TIME_FAULT_MAX_COUNT) ||
            ((request->data.fault.fault == TIME_FAULT_NONE) &&
             (request->data.fault.count != 0U)))
        {
            goto invalid_request;
        }

        if (request->data.fault.fault == TIME_FAULT_NONE)
        {
            /* 只清除故障预算，不清除整个worker上下文。 */
            memset(g_time_worker.fault_remaining,0,sizeof(g_time_worker.fault_remaining));
        }
        else
        {
            /*
             * 配置的是新的剩余次数，采用赋值，不累加。
             * count为0时关闭这一种故障。
             */
            g_time_worker.fault_remaining[request->data.fault.fault] = request->data.fault.count;
        }

        break;

    case TIME_REQUEST_TRACE:
        /*
         * 停止请求忽略interval，与当前公开接口约定一致。
         * 必须先处理停止，不能把0秒转换为至少1tick。
         */
        if (request->data.trace.duration_seconds == 0U)
        {
            g_time_worker.trace_enabled = RT_FALSE;
            g_time_worker.trace_end_tick = 0U;
            g_time_worker.next_trace_tick = 0U;
            g_time_worker.trace_interval_ticks = 0U;
            break;
        }

        if ((request->data.trace.duration_seconds >TIME_TRACE_MAX_SECONDS) ||
            (request->data.trace.interval_seconds == 0U) ||
            (request->data.trace.interval_seconds >request->data.trace.duration_seconds))
        {
            goto invalid_request;
        }

        /* 先提升为64位，避免乘法在较窄类型中发生溢出。 */
        duration_ticks =(uint64_t)request->data.trace.duration_seconds * (uint64_t)RT_TICK_PER_SECOND;

        interval_ticks = (uint64_t)request->data.trace.interval_seconds * (uint64_t)RT_TICK_PER_SECOND;

        /* 满足后续回绕安全deadline比较的跨度限制。 */
        if ((duration_ticks == 0U) ||
            (interval_ticks == 0U) ||
            (duration_ticks > (uint64_t)INT32_MAX) ||
            (interval_ticks > (uint64_t)INT32_MAX))
        {
            goto invalid_request;
        }

        /*
         * 跟踪从worker实际处理请求时开始计时，
         * 不从调用者投递时开始计时。
         */
        now = rt_tick_get();

        g_time_worker.trace_interval_ticks = (rt_tick_t)interval_ticks;

        g_time_worker.trace_end_tick = now + (rt_tick_t)duration_ticks;

        /* 下一轮立即输出首条，之后按interval输出。 */
        g_time_worker.next_trace_tick = now;

        g_time_worker.trace_enabled = RT_TRUE;
        break;

    default:
        goto invalid_request;
    }

    /*
     * FAULT/TRACE到这里表示配置已经应用。
     * 这两类请求没有校时读回值，不能填写readback_available=true。
     */
    time_service_finish_request(request->request_id,
                                RT_EOK,
                                RT_FALSE,
                                0U);
    return;

invalid_request:

    level = rt_hw_interrupt_disable();
    g_time_shared.stats.invalid_requests++;
    rt_hw_interrupt_enable(level);

    time_service_finish_request(request->request_id,-RT_EINVAL,RT_FALSE,0U);
}

/*
 * 简介：执行一次UTC校时、读回确认及可信状态提交。
 * 输入：request为SET_UTC请求副本，仅调用期间读取，不保存其指针；
 *       空指针或零ID作为非法内部调用拒绝。
 * 返回值：无。
 * 说明：仅time线程调用；写入前建立新日历代际并撤销信任。
 *       写入或确认失败保持UNCERTAIN，普通读取不能自动恢复信任。
 *       不持共享临界区执行RTC访问、日志或完成函数。
 */
static void time_service_handle_set(const time_request_t *request)
{
    rt_base_t level;
    rt_tick_t transaction_start_tick;
    rt_tick_t start_tick;
    rt_tick_t finish_tick;
    rt_tick_t elapsed_ticks;
    rt_tick_t io_ticks;

    uint32_t readback = 0U;
    uint64_t elapsed_seconds;
    int64_t difference;

    rt_bool_t readback_available = RT_FALSE;
    rt_bool_t software_injected = RT_FALSE;

    time_error_stage_t failure_stage = TIME_ERROR_WRITE;
    rt_err_t result;

    /* 没有有效ID时无法关联结果记录。 */
    if ((request == RT_NULL) || (request->request_id == 0U))
    {
        level = rt_hw_interrupt_disable();
        g_time_shared.stats.invalid_requests++;
        rt_hw_interrupt_enable(level);

        rt_kprintf("[TIME][SET][ERROR] null request or zero ID\n");
        return;
    }

    /* 防御性复核；这些失败尚未尝试写入，不增加日历代际。 */
    if ((request->kind != TIME_REQUEST_SET_UTC) ||
        (request->data.set.utc_seconds < TIME_UTC_MIN_SECONDS) ||
        (request->data.set.utc_seconds > TIME_UTC_MAX_SECONDS) ||
        ((request->data.set.source != TIME_SOURCE_SHELL) &&
         (request->data.set.source != TIME_SOURCE_BLE)))
    {
        level = rt_hw_interrupt_disable();
        g_time_shared.stats.invalid_requests++;
        rt_hw_interrupt_enable(level);

        time_service_finish_request(request->request_id,
                                    -RT_EINVAL,
                                    RT_FALSE,
                                    0U);
        return;
    }

    if (g_time_worker.port_ready == RT_FALSE)
    {
        time_service_finish_request(request->request_id,
                                    -RT_ERROR,
                                    RT_FALSE,
                                    0U);
        return;
    }

    /*
     * 即将进入可能修改RTC的阶段。
     * 保留last-good值及其sample_tick，但它不能继续作为可信当前时间。
     */
    level = rt_hw_interrupt_disable();

    g_time_shared.snapshot.calendar_generation++;
    g_time_shared.snapshot.trusted = false;
    g_time_shared.snapshot.source = TIME_SOURCE_NONE;
    g_time_shared.snapshot.state = TIME_CLOCK_SYNCING;

    g_time_shared.snapshot.last_change = TIME_CHANGE_SYNC_BEGIN;
    g_time_shared.snapshot.change_seq++;
    g_time_shared.snapshot.publish_seq++;

    g_time_shared.snapshot.error_stage = TIME_ERROR_NONE;
    g_time_shared.snapshot.last_error = RT_EOK;

    g_time_shared.stats.write_attempts++;

    rt_hw_interrupt_enable(level);

    transaction_start_tick = rt_tick_get();
    start_tick = transaction_start_tick;

    /* 软件WRITE注入发生在实际SDK调用前。 */
    if (g_time_worker.fault_remaining[TIME_FAULT_WRITE] != 0U)
    {
        g_time_worker.fault_remaining[TIME_FAULT_WRITE]--;
        software_injected = RT_TRUE;
        result = -RT_ERROR;
    }
    else
    {
        result = rtc_port_write_utc(request->data.set.utc_seconds);
    }

    finish_tick = rt_tick_get();
    io_ticks = finish_tick - start_tick;

    level = rt_hw_interrupt_disable();

    g_time_shared.stats.last_io_ticks = (uint32_t)io_ticks;
    if ((uint32_t)io_ticks > g_time_shared.stats.max_io_ticks)
    {
        g_time_shared.stats.max_io_ticks = (uint32_t)io_ticks;
    }

    if (result == RT_EOK)
    {
        g_time_shared.stats.write_success++;
    }
    else
    {
        g_time_shared.stats.write_failures++;
    }

    rt_hw_interrupt_enable(level);

    if (result != RT_EOK)
    {
        goto set_failed;
    }

    /*
     * SET返回成功后，仍须独立读取。
     * READBACK预算只在校时确认阶段消耗，不能被普通轮询拿走。
     */
    failure_stage = TIME_ERROR_VERIFY;

    level = rt_hw_interrupt_disable();
    g_time_shared.stats.read_attempts++;
    rt_hw_interrupt_enable(level);

    start_tick = rt_tick_get();

    if (g_time_worker.fault_remaining[TIME_FAULT_READBACK] != 0U)
    {
        g_time_worker.fault_remaining[TIME_FAULT_READBACK]--;
        software_injected = RT_TRUE;
        result = -RT_ERROR;
    }
    else
    {
        result = rtc_port_read_utc(&readback);
    }

    finish_tick = rt_tick_get();
    io_ticks = finish_tick - start_tick;

    level = rt_hw_interrupt_disable();

    g_time_shared.stats.last_io_ticks = (uint32_t)io_ticks;
    if ((uint32_t)io_ticks > g_time_shared.stats.max_io_ticks)
    {
        g_time_shared.stats.max_io_ticks = (uint32_t)io_ticks;
    }

    if (result == RT_EOK)
    {
        g_time_shared.stats.read_success++;
    }
    else
    {
        g_time_shared.stats.read_failures++;
    }

    rt_hw_interrupt_enable(level);

    if (result != RT_EOK)
    {
        goto verify_failed;
    }

    /* 表示实际取得了候选值，不表示该值已验证通过。 */
    readback_available = RT_TRUE;

    if ((readback < TIME_UTC_MIN_SECONDS) ||
        (readback > TIME_UTC_MAX_SECONDS))
    {
        failure_stage = TIME_ERROR_RANGE;
        result = -RT_EINVAL;

        level = rt_hw_interrupt_disable();
        g_time_shared.stats.range_failures++;
        rt_hw_interrupt_enable(level);

        goto verify_failed;
    }

    /*
     * 事务总耗时超过工程门限，即使最终返回也不报告及时确认成功。
     * 这只是返回后的检查，不是对SDK调用施加硬超时。
     */
    elapsed_ticks = finish_tick - transaction_start_tick;

    if (elapsed_ticks > time_service_ms_to_ticks(TIME_SET_VERIFY_MAX_MS))
    {
        result = -RT_ETIMEOUT;
        goto verify_failed;
    }

    /* 将经过时间向上取整为秒，用于秒级读回检查。 */
    elapsed_seconds =((uint64_t)elapsed_ticks +
                    (uint64_t)RT_TICK_PER_SECOND - UINT64_C(1)) /
                    (uint64_t)RT_TICK_PER_SECOND;

    /*
     * 先分别提升再相减，避免uint32_t减法先发生无符号回绕。
     * 允许1秒量化差，并考虑写入到读回期间经过的时间。
     */
    difference =(int64_t)readback - (int64_t)request->data.set.utc_seconds;

    if ((difference < -INT64_C(1)) ||(difference > (int64_t)elapsed_seconds + INT64_C(1)))
    {
        result = -RT_ERROR;
        goto verify_failed;
    }

    /*
     * 只有确认通过，才允许发布SYNC_OK和可信来源。
     * publish_read处理失败时，不能提交一个可信快照。
     */
    result = time_service_publish_read(readback,
                                        finish_tick,
                                        request->data.set.source,
                                        TIME_CHANGE_SYNC_OK);

    if (result != RT_EOK)
    {
        goto verify_failed;
    }

    time_service_finish_request(request->request_id,
                                RT_EOK,
                                RT_TRUE,
                                readback);
    return;

verify_failed:

    level = rt_hw_interrupt_disable();
    g_time_shared.stats.verify_failures++;
    rt_hw_interrupt_enable(level);

set_failed:

    level = rt_hw_interrupt_disable();

    g_time_shared.snapshot.trusted = false;
    g_time_shared.snapshot.source = TIME_SOURCE_NONE;
    g_time_shared.snapshot.state = TIME_CLOCK_UNCERTAIN;

    g_time_shared.snapshot.error_stage = failure_stage;
    g_time_shared.snapshot.last_error = result;
    g_time_shared.snapshot.last_change = TIME_CHANGE_SYNC_FAILED;

    g_time_shared.snapshot.change_seq++;
    g_time_shared.snapshot.publish_seq++;

    if (readback_available == RT_TRUE)
    {
        g_time_shared.snapshot.candidate_utc = readback;
    }

    /*
     * 不改last-good的utc_seconds/sample_tick。
     * 代际已在写入前增加，此处不能再增加一次。
     */
    rt_hw_interrupt_enable(level);

    rt_kprintf("[TIME][SET][FAIL] id=%u stage=%u error=%d "
               "software_injected=%u\n",
               (unsigned int)request->request_id,
               (unsigned int)failure_stage,
               (int)result,
               (unsigned int)software_injected);

    time_service_finish_request(request->request_id,
                                result,
                                readback_available,
                                readback);
}

/*
 * 简介：执行一次到期的RTC初始化重试或周期读取。
 * 输入：now为线程入口传入的当前单调tick，不涉及日历时间。
 * 返回值：无。
 * 说明：仅time线程调用；设备调用与日志均在共享临界区外执行。
 *       读取失败保留last-good及已有校时来源，新鲜度由获取快照时计算。
 *       普通读取不得清除校时失败形成的UNCERTAIN状态。
 */
static void time_service_poll(rt_tick_t now)
{
    uint32_t candidate = 0U;
    uint32_t skipped;

    rt_tick_t start_tick;
    rt_tick_t finish_tick;
    rt_tick_t io_ticks;
    rt_tick_t deadline;
    rt_tick_t period_ticks;

    rt_base_t level;
    rt_err_t result;
    time_source_t source;
    time_clock_state_t failure_state = TIME_CLOCK_READ_ERROR;

    rt_bool_t software_injected = RT_FALSE;
    rt_bool_t log_error = RT_FALSE;

    /*
     * 适配尚未就绪时，按重试deadline执行。
     * 资源初始化成功不代表RTC适配已经成功。
     */
    if (g_time_worker.port_ready == RT_FALSE)
    {
        if (time_service_tick_due(now,g_time_worker.next_retry_tick) == RT_FALSE)
        {
            return;
        }

        result = rtc_port_init();
        now = rt_tick_get();

        if (result != RT_EOK)
        {
            /* 从这次尝试返回时开始，至少间隔1秒再重试。 */
            g_time_worker.next_retry_tick = now + time_service_ms_to_ticks(TIME_RETRY_MS);

            level = rt_hw_interrupt_disable();

            log_error =((g_time_shared.snapshot.state != TIME_CLOCK_NO_DEVICE) ||
                        (g_time_shared.snapshot.error_stage != TIME_ERROR_INIT) ||
                        (g_time_shared.snapshot.last_error != result))
                        ? RT_TRUE : RT_FALSE;

            g_time_shared.snapshot.state = TIME_CLOCK_NO_DEVICE;
            g_time_shared.snapshot.error_stage = TIME_ERROR_INIT;
            g_time_shared.snapshot.last_error = result;
            g_time_shared.snapshot.publish_seq++;

            rt_hw_interrupt_enable(level);

            /* 状态或错误变化时才打印，避免每秒重复相同内容。 */
            if (log_error == RT_TRUE)
            {
                rt_kprintf("[TIME][INIT][FAIL] error=%d\n",
                           (int)result);
            }

            return;
        }

        g_time_worker.port_ready = RT_TRUE;

        /* 首次适配成功后，在本次调用中立即读取一次。 */
        g_time_worker.next_poll_tick = now;
    }

    /*
     * 初始化过程可能耗时，使用新的tick。
     * 该检查也让函数在被提前调用时安全返回。
     */
    now = rt_tick_get();
    deadline = g_time_worker.next_poll_tick;

    if (time_service_tick_due(now, deadline) == RT_FALSE)
    {
        return;
    }

    period_ticks = time_service_ms_to_ticks(TIME_POLL_MS);

    level = rt_hw_interrupt_disable();
    g_time_shared.stats.read_attempts++;
    source = g_time_shared.snapshot.source;
    rt_hw_interrupt_enable(level);

    start_tick = rt_tick_get();

    /*
     * MISSING优先模拟本轮设备不可用。
     * 不删除设备、不清空适配句柄，也不改变实际硬件供电。
     */
    if (g_time_worker.fault_remaining[TIME_FAULT_MISSING] != 0U)
    {
        g_time_worker.fault_remaining[TIME_FAULT_MISSING]--;
        software_injected = RT_TRUE;
        failure_state = TIME_CLOCK_NO_DEVICE;
        result = -RT_ERROR;
    }
    else if (g_time_worker.fault_remaining[TIME_FAULT_READ] != 0U)
    {
        g_time_worker.fault_remaining[TIME_FAULT_READ]--;
        software_injected = RT_TRUE;
        result = -RT_ERROR;
    }
    else
    {
        result = rtc_port_read_utc(&candidate);
    }

    finish_tick = rt_tick_get();
    io_ticks = finish_tick - start_tick;

    level = rt_hw_interrupt_disable();

    g_time_shared.stats.last_io_ticks = (uint32_t)io_ticks;
    if ((uint32_t)io_ticks > g_time_shared.stats.max_io_ticks)
    {
        g_time_shared.stats.max_io_ticks = (uint32_t)io_ticks;
    }

    if (result == RT_EOK)
    {
        /*
         * 只表示适配读取成功。
         * 范围、日期和可信状态仍由publish_read检查。
         */
        g_time_shared.stats.read_success++;
    }
    else
    {
        g_time_shared.stats.read_failures++;

        /*
         * 校时失败和非法候选状态比普通读失败更明确，
         * 不用READ_ERROR把原来的错误原因覆盖掉。
         */
        if ((g_time_shared.snapshot.state != TIME_CLOCK_UNCERTAIN) &&
            (g_time_shared.snapshot.state != TIME_CLOCK_INVALID))
        {
            log_error =((g_time_shared.snapshot.state != failure_state) ||
                        (g_time_shared.snapshot.error_stage != TIME_ERROR_READ) ||
                        (g_time_shared.snapshot.last_error != result))
                        ? RT_TRUE : RT_FALSE;

            g_time_shared.snapshot.state = failure_state;
            g_time_shared.snapshot.error_stage = TIME_ERROR_READ;
            g_time_shared.snapshot.last_error = result;
        }

        /*
         * 不更新utc_seconds/sample_tick，不撤销既有校时来源。
         * 获取快照时会根据旧sample_tick重新计算age/fresh。
         */
        g_time_shared.snapshot.publish_seq++;
    }

    rt_hw_interrupt_enable(level);

    if (result == RT_EOK)
    {
        /*
         * NONE表示普通读取，不是一次新的可信校时。
         * 该函数必须自行处理非法范围及发布失败状态。
         */
        (void)time_service_publish_read(candidate,
                                        finish_tick,
                                        source,
                                        TIME_CHANGE_NONE);
    }
    else if ((software_injected == RT_TRUE) ||
             (log_error == RT_TRUE))
    {
        rt_kprintf("[TIME][READ][FAIL] error=%d "
                   "software_injected=%u\n",
                   (int)result,
                   (unsigned int)software_injected);
    }

    /*
     * 按原计划周期向前推进，直接跳过已经错过的周期，
     * 不使用循环突发补读。
     */
    now = rt_tick_get();

    skipped = (uint32_t)((rt_tick_t)(now - deadline) / period_ticks);

    g_time_worker.next_poll_tick = deadline +
                    (rt_tick_t)(((uint64_t)skipped + UINT64_C(1)) *
                                (uint64_t)period_ticks);

    if (skipped != 0U)
    {
        level = rt_hw_interrupt_disable();
        g_time_shared.stats.skipped_poll_periods += skipped;
        rt_hw_interrupt_enable(level);
    }

    return;
}

/*
 * 简介：校验并发布一份RTC读取结果。
 * 输入：utc_seconds为RTC候选UTC秒，sample_tick为其读取完成tick；
 *       reason仅接受NONE或SYNC_OK；SYNC_OK的source须为SHELL或BLE。
 * 返回值：合法候选发布成功返回RT_EOK；参数、范围或转换失败返回负错误码。
 * 说明：仅time线程调用；锁外准备状态，短临界区一次提交。
 *       普通读取成功不等于可信校时成功；UNCERTAIN必须保持到再次确认校时。
 */
static rt_err_t time_service_publish_read(uint32_t utc_seconds,rt_tick_t sample_tick,
                                        time_source_t source,time_change_reason_t reason)
{
    time_snapshot_t next;
    time_calendar_t local_calendar = {0};

    uint32_t day_key;
    rt_tick_t elapsed_ticks;
    int64_t actual_delta;
    int64_t expected_delta;
    int64_t delta_error;

    rt_base_t level;
    rt_err_t error = -RT_EINVAL;
    int result;

    rt_bool_t had_value;
    rt_bool_t was_invalid;
    rt_bool_t keep_uncertain;
    rt_bool_t day_changed;
    rt_bool_t jumped = RT_FALSE;
    rt_bool_t range_invalid = RT_FALSE;

    /*
     * 当前只有两种合法调用：
     * NONE：普通轮询；
     * SYNC_OK：校时读回确认已经通过。
     */
    if ((reason != TIME_CHANGE_NONE) &&
        (reason != TIME_CHANGE_SYNC_OK))
    {
        return -RT_EINVAL;
    }

    if ((reason == TIME_CHANGE_SYNC_OK) &&
        (source != TIME_SOURCE_SHELL) &&
        (source != TIME_SOURCE_BLE))
    {
        return -RT_EINVAL;
    }

    level = rt_hw_interrupt_disable();
    next = g_time_shared.snapshot;
    rt_hw_interrupt_enable(level);

    /*
     * 成功校时必须来自已经开始的校时事务。
     * 普通轮询也不应在同一worker的校时事务中插入发布。
     */
    if ((reason == TIME_CHANGE_SYNC_OK) && (next.state != TIME_CLOCK_SYNCING))
    {
        return -RT_EINVAL;
    }

    if ((reason == TIME_CHANGE_NONE) && (next.state == TIME_CLOCK_SYNCING))
    {
        return -RT_EBUSY;
    }

    had_value = next.has_value ? RT_TRUE : RT_FALSE;
    was_invalid = (next.state == TIME_CLOCK_INVALID) ? RT_TRUE : RT_FALSE;
    keep_uncertain = (next.state == TIME_CLOCK_UNCERTAIN) ? RT_TRUE : RT_FALSE;

    /* 保存原始候选，失败时也能用于诊断。 */
    next.candidate_utc = utc_seconds;

    if ((utc_seconds < TIME_UTC_MIN_SECONDS) || (utc_seconds > TIME_UTC_MAX_SECONDS))
    {
        range_invalid = RT_TRUE;
        error = -RT_EINVAL;
        goto publish_failed;
    }

    result = time_calendar_from_utc(
        utc_seconds,
        TIME_DISPLAY_OFFSET_MINUTES,
        &local_calendar);

    if (result != TIME_CAL_OK)
    {
        error = (result == TIME_CAL_NOT_IMPLEMENTED) ? -RT_ENOSYS : -RT_EINVAL;
        goto publish_failed;
    }

    day_key = (uint32_t)local_calendar.year * UINT32_C(10000) +
                (uint32_t)local_calendar.month * UINT32_C(100) +
                (uint32_t)local_calendar.day;

    day_changed = ((had_value == RT_FALSE) ||
                (next.local_day_key != day_key))
                ? RT_TRUE : RT_FALSE;

    /*
     * 只对普通读取检查连续性。
     *
     * UNCERTAIN中可能保存着校时前的旧样本；
     * INVALID恢复时也需要重新建立合法基准。
     * 不能拿这些旧基准再次判断同一次跳变。
     */
    if ((reason == TIME_CHANGE_NONE) &&
        (had_value == RT_TRUE) &&
        (keep_uncertain == RT_FALSE) &&
        (was_invalid == RT_FALSE))
    {
        elapsed_ticks = sample_tick - (rt_tick_t)next.sample_tick;

        actual_delta = (int64_t)utc_seconds - (int64_t)next.utc_seconds;

        expected_delta = (int64_t)((uint64_t)elapsed_ticks / (uint64_t)RT_TICK_PER_SECOND);

        delta_error = actual_delta - expected_delta;

        if ((delta_error >
             (int64_t)TIME_JUMP_TOLERANCE_SECONDS) ||
            (delta_error <
             -(int64_t)TIME_JUMP_TOLERANCE_SECONDS))
        {
            jumped = RT_TRUE;
        }
    }

    /* 到这里候选值和显示日期均合法，可以建立新的样本。 */
    next.utc_seconds = utc_seconds;
    next.sample_tick = (uint32_t)sample_tick;
    next.age_ms = 0U;
    next.has_value = true;
    next.fresh = true;
    next.local_day_key = day_key;

    /*
     * 首次建立日期也递增day_seq。
     * 是否为自然跨日，还要结合last_change、trusted和generation。
     */
    if (day_changed == RT_TRUE)
    {
        next.day_seq++;
    }

    if (reason == TIME_CHANGE_SYNC_OK)
    {
        /* 代际已经由handle_set在写入前增加，此处不再增加。 */
        next.trusted = true;
        next.source = source;
        next.state = TIME_CLOCK_READY;

        next.error_stage = TIME_ERROR_NONE;
        next.last_error = RT_EOK;

        next.last_change = TIME_CHANGE_SYNC_OK;
        next.change_seq++;
    }
    else if (keep_uncertain == RT_TRUE)
    {
        /*
         * 普通读取不能证明之前的写入事务成功。
         * 保留原WRITE/VERIFY错误及SYNC_FAILED原因。
         */
        next.trusted = false;
        next.source = TIME_SOURCE_NONE;
        next.state = TIME_CLOCK_UNCERTAIN;
    }
    else if (jumped == RT_TRUE)
    {
        next.calendar_generation++;
        next.trusted = false;
        next.source = TIME_SOURCE_NONE;
        next.state = TIME_CLOCK_UNSET;

        next.last_change = TIME_CHANGE_EXTERNAL_JUMP;
        next.change_seq++;

        /* 这是逻辑连续性异常，不是底层GET接口失败。 */
        next.error_stage = TIME_ERROR_READ;
        next.last_error = -RT_ERROR;
    }
    else
    {
        /*
         * 普通读成功只恢复读取状态，
         * 不根据候选年份或传入source重新建立信任。
         */
        next.state = next.trusted ? TIME_CLOCK_READY : TIME_CLOCK_UNSET;

        if (!next.trusted)
        {
            next.source = TIME_SOURCE_NONE;
        }

        next.error_stage = TIME_ERROR_NONE;
        next.last_error = RT_EOK;

        if ((had_value == RT_TRUE) &&
            (day_changed == RT_TRUE) && next.trusted)
        {
            next.last_change = TIME_CHANGE_DAY;
            next.change_seq++;
        }

        /* 无日历事件时保留原last_change，不每次清为NONE。 */
    }

    next.publish_seq++;

    level = rt_hw_interrupt_disable();

    g_time_shared.snapshot = next;

    if (jumped == RT_TRUE)
    {
        g_time_shared.stats.external_jumps++;
    }

    rt_hw_interrupt_enable(level);

    return RT_EOK;

publish_failed:

    /*
     * 不修改last-good的utc_seconds/sample_tick/local_day_key。
     * 获取副本时仍会根据旧sample_tick计算age。
     */
    if (keep_uncertain == RT_TRUE)
    {
        next.trusted = false;
        next.source = TIME_SOURCE_NONE;
        next.state = TIME_CLOCK_UNCERTAIN;

        /* 保留原校时失败原因，不用后续读取问题将其覆盖。 */
    }
    else if (reason == TIME_CHANGE_SYNC_OK)
    {
        /*
         * 校时确认阶段的发布失败。
         * handle_set随后会记录SYNC_FAILED并完成失败请求。
         */
        next.trusted = false;
        next.source = TIME_SOURCE_NONE;
        next.state = TIME_CLOCK_UNCERTAIN;
        next.error_stage = TIME_ERROR_RANGE;
        next.last_error = error;
    }
    else if (range_invalid == RT_TRUE)
    {
        /*
         * 原始候选超范围，撤销信任。
         * 同一段持续INVALID不在每次轮询时重复增加代际。
         */
        if (was_invalid == RT_FALSE)
        {
            if (had_value == RT_TRUE)
            {
                next.calendar_generation++;
            }

            next.last_change = TIME_CHANGE_INVALID;
            next.change_seq++;
        }

        next.trusted = false;
        next.source = TIME_SOURCE_NONE;
        next.state = TIME_CLOCK_INVALID;
        next.error_stage = TIME_ERROR_RANGE;
        next.last_error = error;
    }
    else
    {
        /*
         * 原始UTC合法，但软件日历转换失败。
         * 不把软件转换错误直接当成RTC被重新设置；
         * 保留已有校时来源，旧值通过age逐步过期。
         */
        if (was_invalid == RT_FALSE)
        {
            next.state = TIME_CLOCK_READ_ERROR;
        }

        next.error_stage = TIME_ERROR_RANGE;
        next.last_error = error;
    }

    next.publish_seq++;

    level = rt_hw_interrupt_disable();

    g_time_shared.snapshot = next;

    if (range_invalid == RT_TRUE)
    {
        g_time_shared.stats.range_failures++;
    }

    rt_hw_interrupt_enable(level);

    return error;
}

/*
 * 简介：将一条正在处理的请求提交为成功或失败终态。
 * 输入：request_id必须非零；error为执行结果；
 *       readback_available仅说明是否实际取得读回值，不代表验证通过。
 * 返回值：无。
 * 说明：仅time线程调用；只更新匹配ID且状态为RUNNING的记录。
 *       不覆盖其他记录，不重复累计完成次数，不访问RTC。
 */
static void time_service_finish_request(uint32_t request_id,rt_err_t error,rt_bool_t readback_available,uint32_t readback_utc)
{
    rt_base_t level;
    uint32_t index;
    rt_tick_t now = rt_tick_get();

    rt_bool_t completed = RT_FALSE;
    rt_bool_t bad_completion = RT_FALSE;
    time_request_result_t *record;

    if (request_id == 0U)
    {
        level = rt_hw_interrupt_disable();
        g_time_shared.stats.invalid_requests++;
        rt_hw_interrupt_enable(level);

        rt_kprintf("[TIME][RESULT][ERROR] zero request ID\n");
        return;
    }

    level = rt_hw_interrupt_disable();

    for (index = 0U; index < TIME_RESULT_CAPACITY; index++)
    {
        record = &g_time_shared.results[index];

        if (record->request_id != request_id)
        {
            continue;
        }

        /* 已完成或尚未进入处理中的记录，不能直接改成新的终态。 */
        if (record->state != TIME_RESULT_RUNNING)
        {
            break;
        }

        /*
         * 成功的SET必须携带实际取得的读回值。
         * 这里只检查明显的调用错误；验证过程仍由handle_set负责。
         * 校时成功，但又说没有取得读回值，这两个信息矛盾
         * bad_completion表示完成函数收到的参数组合有问题
         */
        if ((record->kind == TIME_REQUEST_SET_UTC) &&
            (error == RT_EOK) &&
            (readback_available != RT_TRUE))
        {
            error = -RT_ERROR;
            bad_completion = RT_TRUE;
        }

        /*
         * FAULT/TRACE没有RTC读回值。
         * 如果调用者错误传入读回标志，记录为内部参数错误。
         */
        if ((record->kind != TIME_REQUEST_SET_UTC) &&
            (readback_available == RT_TRUE))
        {
            error = -RT_EINVAL;
            readback_available = RT_FALSE;
            bad_completion = RT_TRUE;
        }

        record->error = error;
        record->calendar_generation = g_time_shared.snapshot.calendar_generation;
        record->finished_tick = (uint32_t)now;

        record->readback_available = (readback_available == RT_TRUE);

        record->readback_utc = record->readback_available ? readback_utc : 0U;

        record->state = (error == RT_EOK) ? TIME_RESULT_OK : TIME_RESULT_FAILED;

        g_time_shared.stats.completed_requests++;
        g_time_shared.stats.worker_completed_tick = (uint32_t)now;

        if (bad_completion == RT_TRUE)
        {
            g_time_shared.stats.invalid_requests++;
        }

        completed = RT_TRUE;
        break;
    }

    if (completed == RT_FALSE)
    {
        g_time_shared.stats.invalid_requests++;
    }

    rt_hw_interrupt_enable(level);

    /* 日志必须放在临界区之外。 */
    if (completed == RT_FALSE)
    {
        rt_kprintf("[TIME][RESULT][ERROR] no RUNNING record, id=%u\n", (unsigned int)request_id);
    }
    else if (bad_completion == RT_TRUE)
    {
        rt_kprintf("[TIME][RESULT][ERROR] invalid completion, id=%u\n",
                   (unsigned int)request_id);
    }
}

/*
 * 简介：按跟踪配置输出一组时间服务诊断日志。
 * 输入：now为线程入口传入的单调tick。
 * 返回值：无。
 * 说明：仅time线程调用；到期才输出，日志在临界区外。
 *       到达结束时间关闭跟踪；输出延误时跳过旧周期，不突发补打。
 */
static void time_service_trace_if_due(rt_tick_t now)
{
    time_snapshot_t snapshot;
    time_service_stats_t stats;
    rt_err_t result;

    rt_tick_t deadline;
    rt_tick_t interval_ticks;
    rt_tick_t capture_tick;
    uint32_t skipped;

    if (g_time_worker.trace_enabled == RT_FALSE)
    {
        return;
    }

    /* 结束时间优先于下一次输出时间。 */
    if (time_service_tick_due(now,g_time_worker.trace_end_tick) == RT_TRUE)
    {
        g_time_worker.trace_enabled = RT_FALSE;

        rt_kprintf("[TIME][TRACE][END] t=%u\n",(unsigned int)now);
        return;
    }

    interval_ticks = g_time_worker.trace_interval_ticks;

    /* 防止内部配置错误导致除零或不安全的deadline计算。 */
    if ((interval_ticks == 0U) || (interval_ticks > (rt_tick_t)INT32_MAX))
    {
        g_time_worker.trace_enabled = RT_FALSE;

        rt_kprintf("[TIME][TRACE][ERROR] invalid interval\n");
        return;
    }

    deadline = g_time_worker.next_trace_tick;

    if (time_service_tick_due(now, deadline) == RT_FALSE)
    {
        return;
    }

    /*
     * 通过公开接口获取副本：
     * snapshot接口会重新计算age/fresh，而不是直接读取旧缓存。
     */
    result = time_service_get_snapshot(&snapshot);
    if (result != RT_EOK)
    {
        rt_kprintf("[TIME][TRACE][ERROR] snapshot=%d\n",(int)result);
        goto reschedule;
    }

    result = time_service_get_stats(&stats);
    if (result != RT_EOK)
    {
        rt_kprintf("[TIME][TRACE][ERROR] stats=%d\n",(int)result);
        goto reschedule;
    }

    capture_tick = rt_tick_get();

    /*
     * 快照、统计是先后复制的诊断副本，不宣称全部字段在同一硬件时刻采样。
     * 每条短日志使用同一个capture_tick，便于关联为一组。
     */
    rt_kprintf("[TIME][TRACE] t=%u sample=%u utc=%u raw=%u age=%u\n",
               (unsigned int)capture_tick,
               (unsigned int)snapshot.sample_tick,
               (unsigned int)snapshot.utc_seconds,
               (unsigned int)snapshot.candidate_utc,
               (unsigned int)snapshot.age_ms);

    rt_kprintf("[TIME][TRACE] t=%u st=%u src=%u tr=%u fr=%u "
               "gen=%u e=%u/%d\n",
               (unsigned int)capture_tick,
               (unsigned int)snapshot.state,
               (unsigned int)snapshot.source,
               (unsigned int)snapshot.trusted,
               (unsigned int)snapshot.fresh,
               (unsigned int)snapshot.calendar_generation,
               (unsigned int)snapshot.error_stage,
               (int)snapshot.last_error);

    rt_kprintf("[TIME][TRACE] t=%u read=%u/%u/%u write=%u/%u/%u\n",
               (unsigned int)capture_tick,
               (unsigned int)stats.read_attempts,
               (unsigned int)stats.read_success,
               (unsigned int)stats.read_failures,
               (unsigned int)stats.write_attempts,
               (unsigned int)stats.write_success,
               (unsigned int)stats.write_failures);

    rt_kprintf("[TIME][TRACE] t=%u day=%u ds=%u ch=%u/%u "
               "pub=%u vf=%u\n",
               (unsigned int)capture_tick,
               (unsigned int)snapshot.local_day_key,
               (unsigned int)snapshot.day_seq,
               (unsigned int)snapshot.change_seq,
               (unsigned int)snapshot.last_change,
               (unsigned int)snapshot.publish_seq,
               (unsigned int)stats.verify_failures);

    rt_kprintf("[TIME][TRACE] t=%u range=%u jump=%u skip=%u "
               "io=%u/%u\n",
               (unsigned int)capture_tick,
               (unsigned int)stats.range_failures,
               (unsigned int)stats.external_jumps,
               (unsigned int)stats.skipped_poll_periods,
               (unsigned int)stats.last_io_ticks,
               (unsigned int)stats.max_io_ticks);

reschedule:

    /*
     * 获取副本、输出日志可能耗时，使用新的tick推进调度。
     * 即使获取副本失败，也必须推进deadline，不能立即反复重试。
     */
    now = rt_tick_get();

    if (time_service_tick_due(now,g_time_worker.trace_end_tick) == RT_TRUE)
    {
        g_time_worker.trace_enabled = RT_FALSE;

        rt_kprintf("[TIME][TRACE][END] t=%u\n",
                   (unsigned int)now);
        return;
    }

    skipped =
        (uint32_t)((rt_tick_t)(now - deadline) / interval_ticks);

    g_time_worker.next_trace_tick =
        deadline +
        (rt_tick_t)(((uint64_t)skipped + UINT64_C(1)) *
                    (uint64_t)interval_ticks);
}

/*
 * 简介：将短时间毫秒间隔转换为非零tick。
 * 输入：milliseconds为本服务已限制范围的正数毫秒值。
 * 返回值：至少1tick的间隔。
 * 说明：使用64位中间计算并向上取整，不假定系统永远是1000Hz。
 */
static rt_tick_t time_service_ms_to_ticks(uint32_t milliseconds)
{
    uint64_t ticks;

    // tick数 = 向上取整(milliseconds × 每秒tick数 ÷ 1000)。
    ticks = ((uint64_t)milliseconds * (uint64_t)RT_TICK_PER_SECOND +
             UINT64_C(999)) / UINT64_C(1000);

    // 即使误传0，也保证非零间隔，避免产生忙等。
    if (ticks == 0U)
    {
        ticks = 1U;
    }

    return (rt_tick_t)ticks;

}

/*
 * 简介：判断单调tick是否到达截止点。
 * 输入：now和deadline为同一tick域；比较跨度必须小于半个32位量程。
 * 返回值：到达返回RT_TRUE，否则RT_FALSE。
 * 说明：使用回绕安全差值，不能使用普通now >= deadline替代。
 */
static rt_bool_t time_service_tick_due(rt_tick_t now,rt_tick_t deadline)
{
    return ((rt_int32_t)(now - deadline) >= 0) ? RT_TRUE : RT_FALSE;
}

/*
 * 简介：预留结果槽并非阻塞投递一条控制请求。
 * 输入：request和request_id不可为空；发送内容复制，函数不保存输入指针。
 * 返回值：受理返回RT_EOK及非零ID；未启动/容量不足等返回负错误码。
 * 说明：仅线程调用；临界区不发送消息，不覆盖等待或处理中的结果槽。
 */
static rt_err_t time_service_post_request(time_request_t *request,uint32_t *request_id)
{
    rt_base_t level;
    rt_mq_t queue;
    time_request_t message;

    uint32_t id;
    uint32_t slot = TIME_RESULT_CAPACITY;
    uint32_t candidate;
    uint32_t index;
    uint32_t attempt;
    rt_bool_t collision;
    rt_err_t result;

    // 失败时不向调用者暴露一个未被受理的请求ID。
    if (request_id != RT_NULL)
    {
        *request_id = 0U;
    }

    level = rt_hw_interrupt_disable();

    g_time_shared.stats.post_attempts++;

    if ((request == RT_NULL) || (request_id == RT_NULL))
    {
        g_time_shared.stats.invalid_requests++;
        g_time_shared.stats.post_rejected++;

        rt_hw_interrupt_enable(level);
        return -RT_EINVAL;
    }

    // 使用内部副本，稍后只修改副本中的request_id。
    message = *request;

    //检查投递参数是否合法
    if ((message.kind != TIME_REQUEST_SET_UTC) &&
        (message.kind != TIME_REQUEST_FAULT) &&
        (message.kind != TIME_REQUEST_TRACE))
    {
        g_time_shared.stats.invalid_requests++;
        g_time_shared.stats.post_rejected++;

        rt_hw_interrupt_enable(level);
        return -RT_EINVAL;
    }

    //检查线程状态
    if ((g_time_shared.started != RT_TRUE) || (g_time_queue == RT_NULL))
    {
        g_time_shared.stats.post_rejected++;

        rt_hw_interrupt_enable(level);
        return -RT_ERROR;
    }

    // 第一轮：从扫描起点寻找空槽，尽量保留已有终态记录。
    for (index = 0U; index < TIME_RESULT_CAPACITY; index++)
    {
        candidate = (g_time_shared.next_result_slot + index) % TIME_RESULT_CAPACITY;

        if (g_time_shared.results[candidate].request_id == 0U)
        {
            slot = candidate;
            break;
        }
    }

    // 第二轮：没有空槽时，轮转寻找可以回收的终态。
    if (slot == TIME_RESULT_CAPACITY)
    {
        for (index = 0U; index < TIME_RESULT_CAPACITY; index++)
        {
            candidate = (g_time_shared.next_result_slot + index) % TIME_RESULT_CAPACITY;

            if ((g_time_shared.results[candidate].state == TIME_RESULT_OK) ||
                (g_time_shared.results[candidate].state == TIME_RESULT_FAILED))
            {
                slot = candidate;
                break;
            }
        }
    }

    // 全部记录都在等待或处理中，不能强行覆盖。
    if (slot == TIME_RESULT_CAPACITY)
    {
        g_time_shared.stats.post_rejected++;

        rt_hw_interrupt_enable(level);
        return -RT_EBUSY;
    }
    // next_request_id保存最近一次分配值，从其后继续寻找非零ID。
    id = g_time_shared.next_request_id;
    collision = RT_TRUE;

    // 最多保留8个ID，因此检查9个不同候选值即可找到空闲ID。
    for (attempt = 0U;attempt < TIME_RESULT_CAPACITY + 1U;attempt++)
    {
        id++;
        // uint32_t回绕后跳过0，0专门表示空槽。
        if (id == 0U)
        {
            id = 1U;
        }

        collision = RT_FALSE;

        for (index = 0U; index < TIME_RESULT_CAPACITY; index++)
        {
            if (g_time_shared.results[index].request_id == id)
            {
                collision = RT_TRUE;
                break;
            }
        }

        if (collision == RT_FALSE)
        {
            break;
        }
    }

    if (collision == RT_TRUE)
    {
        g_time_shared.stats.post_rejected++;

        rt_hw_interrupt_enable(level);
        return -RT_EBUSY;
    }
    // 必须先登记结果，worker才能在收到消息后找到对应记录。
    memset(&g_time_shared.results[slot],0,sizeof(g_time_shared.results[slot]));
    g_time_shared.results[slot].request_id = id;
    g_time_shared.results[slot].kind = message.kind;
    g_time_shared.results[slot].state = TIME_RESULT_PENDING;

    g_time_shared.next_request_id = id;
    g_time_shared.next_result_slot = (slot + 1U) % TIME_RESULT_CAPACITY;

    queue = g_time_queue;

    rt_hw_interrupt_enable(level);

    // 队列发送在临界区外进行，RT-Thread会复制整条消息。
    message.request_id = id;

    result = rt_mq_send(queue,&message,sizeof(message));

    level = rt_hw_interrupt_disable();

    if (result == RT_EOK)
    {
        g_time_shared.stats.post_accepted++;

        // 不再修改结果state：worker此时可能已经完成了请求。
    }
    else
    {
        g_time_shared.stats.post_rejected++;

        // 发送失败，请求没有入队，只回收仍属于本次预留的记录。
        if ((g_time_shared.results[slot].request_id == id) &&
            (g_time_shared.results[slot].state == TIME_RESULT_PENDING))
        {
            memset(&g_time_shared.results[slot],0,sizeof(g_time_shared.results[slot]));
        }
    }

    rt_hw_interrupt_enable(level);

    if (result != RT_EOK)
    {
        return result;
    }

    // 成功入队后，才把可查询的ID交给调用者。
    *request_id = id;

    return RT_EOK;
}
