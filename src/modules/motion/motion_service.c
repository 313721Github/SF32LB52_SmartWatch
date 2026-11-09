#include "motion_service.h"
#include "motion_core.h"
#include "motion_trace.h"
#include "sensor_service.h"
#include "time_service.h"
#include "rthw.h"

#include <limits.h>
#include <string.h>

#define MOTION_THREAD_STACK_SIZE       3072U  // 运动线程栈大小，单位：字节
#define MOTION_THREAD_PRIORITY         20U    // 运动线程优先级，数值越小优先级越高
#define MOTION_THREAD_TIMESLICE        10U    // 同优先级线程的调度时间片，单位：tick

#define MOTION_SAMPLE_BATCH_MAX        4U     // worker每批最多处理的样本帧数
#define MOTION_WORKER_WAIT_MS          20U    // worker单次等待唤醒的最长时间，单位：ms
#define MOTION_SOURCE_POLL_MS          100U   // 轮询传感器数据源状态的间隔，单位：ms
#define MOTION_COPY_TIMEOUT_MS         5U     // 复制共享数据时允许的最长等待时间，单位：ms
#define MOTION_STOP_TIMEOUT_MS         1000U  // 等待停止请求完成的超时门限，单位：ms
#define MOTION_PUBLISH_STALE_MS        250U   // 运动快照自上次发布起的新鲜度门限，单位：ms

#define MOTION_WAKE_WORK               (1UL << 0) // 唤醒运动worker的事件位

/* 实际ABI下校验，不用packed强压结构体。 */
typedef char motion_frame_size_check[(sizeof(motion_sample_frame_t) <= MOTION_SAMPLE_FRAME_BUDGET_BYTES) ? 1 : -1];

typedef char motion_tick_size_check[(sizeof(rt_tick_t) == sizeof(uint32_t)) ? 1 : -1];

typedef struct
{
    uint32_t request_id;        // 控制请求编号，用于关联处理结果
    motion_request_kind_t kind; // 控制请求类型，如START或STOP
    motion_test_fault_t fault; // 仅TEST请求使用
    uint32_t argument;
} motion_control_message_t;

/*
 * 生产侧短临界区统计。
 * 不能为了复制整个motion_service_stats_t而长时间关闭中断。
 */
typedef struct
{
    uint64_t produced;             // 有效的样本提交尝试数
    uint64_t disabled_skipped;     // 因接纳门关闭而跳过的样本数
    uint64_t enqueue_ok;           // 成功放入样本队列的帧数
    uint64_t enqueue_reject_full;  // 因队列已满而拒绝的帧数
    uint64_t enqueue_reject_other; // 因其他入队错误而拒绝的帧数
    uint64_t invalid_submit_calls; // 参数非法等无效提交调用次数

    uint32_t inflight;             // 已开始但尚未完成记账的样本提交数
    uint32_t queue_high_water;     // 观测到的样本队列最大深度，单位：帧
    uint32_t max_submit_ticks;     // 单次样本提交的最大耗时，单位：tick
    bool saturated;                // 生产侧累计统计是否曾达到上限
} motion_ingress_stats_t;

/* 共享控制字段，仅通过短临界区访问。 */
typedef struct
{
    bool initializing;      // 是否正在初始化服务资源
    bool ready;             // 服务资源和worker是否已就绪
    bool accepting_samples; // 当前是否允许生产侧投递样本
    bool faulted;           // 服务是否已进入故障状态

    uint32_t admitted_generation; // 接纳门允许投递的运行代际

    uint32_t next_request_id;  // 分配控制请求编号的游标
    uint32_t control_inflight; // 尚在途的控制请求数

    uint32_t init_attempts;         // 服务初始化尝试次数
    uint32_t init_failures;         // 服务初始化失败次数
    uint64_t control_post_attempts; // 控制请求投递尝试次数
    uint64_t control_post_accepted; // 控制请求投递成功次数
    uint64_t control_post_rejected; // 控制请求投递拒绝次数
    uint64_t control_completed;     // 已完成处理并登记结果的请求数
    uint64_t wake_failures;         // 发送worker唤醒事件失败的次数
    bool counters_saturated;        // 共享控制统计是否曾达到上限

    motion_request_result_t results[MOTION_RESULT_SLOT_COUNT]; // 控制请求结果槽
    motion_ingress_stats_t ingress;                             // 样本生产侧统计

    /*
     * worker修改核心期间为true。
     * revision用于确认发布副本对应当前已完成的消费状态。
     */
    bool worker_busy;         // worker是否正在修改核心状态
    uint64_t worker_revision; // worker已完成处理的状态版本

    /*
    * 本轮最终发布是否成功覆盖了已完成的worker状态。
    * 中途发布成功不能将此标志置true。
    */
    bool publication_current;

    /* 从开始尝试出队到完成该帧记账，保持非0。 */
    uint32_t receive_inflight; // 正在出队或尚未完成消费记账的操作数
} motion_shared_t;

/* 发布区由publish_mutex保护；其他线程只能读取此处的副本。 */
typedef struct
{
    bool valid;                     // 是否已有可供读取的发布副本
    uint64_t worker_revision;       // 该副本对应的worker状态版本
    motion_snapshot_t snapshot;     // 已发布的运动业务快照
    motion_test_status_t test;      // 与摘要同锁发布的诊断状态
    motion_service_stats_t stats;   // 已发布的运动服务统计
} motion_published_t;

typedef struct
{
    motion_test_status_t status;
    bool middle_publish_ok;
    uint64_t middle_revision;
    uint64_t dequeued_before_batch;
    bool capture_pending;
    motion_service_stats_t witness_work;
} motion_test_worker_t;

/* 仅motion worker访问。大缓冲放静态区，避免挤占3072B线程栈。 */
typedef struct
{
    motion_core_t core;             // worker独占的计步核心实例
    motion_test_worker_t test;      // 默认全零关闭的有限注入状态

    uint32_t next_session_id;       // 分配新会话编号的游标
    uint32_t next_run_generation;   // 分配新运行代际的游标

    uint32_t pending_stop_id; // 0表示没有等待完成的STOP
    rt_tick_t stop_deadline;  // 当前STOP的超时截止tick

    rt_tick_t next_source_poll;      // 下次轮询传感器状态的截止tick
    rt_tick_t last_source_read_tick; // 最近一次读取传感器状态的tick
    bool has_source_snapshot;        // 是否持有可供检查的传感器快照副本
    sensor_snapshot_t source_snapshot; // 传感器状态轮询使用的快照缓冲

    uint64_t discard_internal;             // 因内部错误而丢弃的出队帧数
    uint64_t publish_failures;             // 发布运动快照或统计失败的次数
    uint64_t sensor_status_read_failures;   // 读取传感器状态失败的次数
    bool counters_saturated;              // worker侧累计统计是否曾达到上限

    uint32_t max_queue_wait_ticks; // 样本从入队到出队的最大等待时间，单位：tick
    uint32_t max_sample_age_ticks; // 处理时观测到的最大样本年龄，单位：tick
    uint32_t max_algorithm_ticks;  // 单帧算法处理的最大耗时，单位：tick

    uint32_t worker_progress_tick;    // 最近完成一个工作单元的tick
    uint32_t effective_progress_tick; // 最近取得有效业务处理进展的tick

    motion_sample_frame_t frame;             // 当前出队样本帧的工作缓冲
    motion_core_frame_result_t frame_result; // 当前帧的核心处理结果
    motion_core_stats_t core_stats_work;      // 复制核心统计使用的工作缓冲
    motion_snapshot_t snapshot_work;         // 组装待发布业务快照的工作缓冲
    motion_service_stats_t stats_work;       // 组装待发布服务统计的工作缓冲
} motion_worker_t;

static rt_mq_t g_motion_sample_queue = RT_NULL;      // 逐IMU样本队列句柄
static rt_mq_t g_motion_control_queue = RT_NULL;     // START、STOP等控制请求队列句柄
static rt_event_t g_motion_wake_event = RT_NULL;     // worker唤醒事件句柄
static rt_mutex_t g_motion_publish_mutex = RT_NULL; // 保护发布区的互斥锁
static rt_thread_t g_motion_thread = RT_NULL;       // 运动worker线程句柄

static motion_shared_t g_motion_shared;       // 生产侧与worker共享的控制状态
static motion_published_t g_motion_published; // 供其他线程读取的发布副本
static motion_worker_t g_motion_worker;       // worker独占的状态及静态工作缓冲

//公开函数声明
rt_err_t motion_service_init(void);
rt_err_t motion_service_request_start(uint32_t *request_id);            //非阻塞提交START请求
rt_err_t motion_service_request_stop(uint32_t *request_id);             //非阻塞提交STOP请求
rt_err_t motion_service_get_result(uint32_t request_id,
                                  motion_request_result_t *result);     //查询有限结果槽中的一条控制结果
rt_err_t motion_service_get_snapshot(motion_snapshot_t *snapshot);      //取得线程间发布的运动摘要副本
rt_err_t motion_service_get_stats(motion_service_stats_t *stats);       //合并已发布消费统计与短临界区生产/控制统计
rt_err_t motion_service_submit_imu(const motion_imu_sample_t *sample);  //从唯一sensor生产者复制并非阻塞投递一帧

//私有函数集中声明
static rt_tick_t motion_service_ms_to_ticks(uint32_t milliseconds);     //将正数毫秒向上换算为至少1tick
static void motion_service_add_u64(uint64_t *value,uint64_t increment,  //执行受保护的64位饱和累计
                                   bool *saturated);
static rt_err_t motion_service_wake(void);                              //发送合并唤醒位
static rt_err_t motion_service_read_calendar(motion_calendar_tag_t *tag);//把时间服务副本转换为运动日历标签
static rt_err_t motion_service_sensor_sink(const sensor_imu_sample_t *sample,
                                        rt_uint32_t imu_seq,void *user_data);//把sensor类型逐字段转换为motion输入并投递
static rt_err_t motion_service_post_request(motion_request_kind_t kind, motion_test_fault_t fault, uint32_t argument, uint32_t *request_id);          //预留结果记录并非阻塞提交控制消息
static void motion_service_thread_entry(void *parameter);               //运行唯一运动worker，串行处理控制、样本和状态发布
static void motion_service_update_inputs(void);                         //更新当前日历及有界轮询得到的数据源状态
static void motion_service_handle_control(const motion_control_message_t *message);//串行执行一条START或开始STOP握手
static void motion_service_poll_stop(void);         //推进已发起STOP的有界完成条件
static rt_err_t motion_service_process_one(void);   //非阻塞取出并处理最多一份样本
static rt_err_t motion_service_publish(void);       //构造并发布同一轮核心摘要和消费统计
static void motion_service_finish_request(uint32_t request_id,rt_err_t error);  //提交一条控制请求的成功或失败终态
static void motion_service_enter_fault(rt_err_t error); //让服务进入停止接纳的受控故障状态


static bool motion_service_test_args_valid(motion_test_fault_t fault, uint32_t argument);
static rt_err_t motion_service_test_apply(const motion_control_message_t *message);
static void motion_service_test_end(motion_test_end_t reason);
static void motion_service_test_poll(void);
static bool motion_service_test_pause_active(void);
static rt_err_t motion_service_test_prepare_round(void);
static bool motion_service_test_drop_frame(void);
static bool motion_service_test_fail_final(void);
static void motion_service_test_capture(void);
static bool motion_service_control_args_valid(motion_request_kind_t kind, motion_test_fault_t fault, uint32_t argument);

//---------------------------------------------------------------公开函数---------------------------------------------------------------
/*
 * 简介：建立运动服务资源，注册sensor回调并启动worker。
 * 输入：无。
 * 返回值：成功RT_EOK；初始化冲突-RT_EBUSY；
 *         资源不足-RT_ENOMEM；其他失败返回实际负错误码。
 * 说明：必须在sensor启动前、由统一启动流程调用。
 *       成功后业务STOPPED，START需要另外提交。
 *       worker启动成功后不再进入资源清理路径。
 */
rt_err_t motion_service_init(void)
{
    rt_base_t level;
    rt_err_t result;
    rt_err_t cleanup_result;
    int core_result;

    bool sink_registered;
    bool keep_resources;

    sink_registered = false;
    keep_resources = false;

    /* 第一阶段：占用初始化生命周期。 */
    level = rt_hw_interrupt_disable();

    if (g_motion_shared.ready)
    {
        result = g_motion_shared.faulted? -RT_ERROR: RT_EOK;

        rt_hw_interrupt_enable(level);
        return result;
    }

    if (g_motion_shared.initializing)
    {
        rt_hw_interrupt_enable(level);
        return -RT_EBUSY;
    }

    /*
     * 残留句柄不能直接覆盖。
     * faulted状态也不能通过重新初始化悄悄清除。
     */
    if (g_motion_shared.faulted ||
        (g_motion_sample_queue != RT_NULL) ||
        (g_motion_control_queue != RT_NULL) ||
        (g_motion_wake_event != RT_NULL) ||
        (g_motion_publish_mutex != RT_NULL) ||
        (g_motion_thread != RT_NULL))
    {
        rt_hw_interrupt_enable(level);
        return -RT_EBUSY;
    }

    g_motion_shared.initializing = true;
    g_motion_shared.accepting_samples = false;

    if (g_motion_shared.init_attempts < UINT32_MAX)
    {
        g_motion_shared.init_attempts++;
    }

    if (g_motion_shared.init_attempts == UINT32_MAX)
    {
        g_motion_shared.counters_saturated = true;
    }

    rt_hw_interrupt_enable(level);

    /*
     * 本函数要求sensor尚未启动。
     * 后面的注册接口还会再次检查sensor生命周期。
     */
    if (sensor_service_is_running())
    {
        result = -RT_EBUSY;
        goto init_failed;
    }

    /* 第二阶段：锁外创建资源。 */
    g_motion_sample_queue = rt_mq_create( "mot_smp",
                                            sizeof(motion_sample_frame_t),
                                            MOTION_SAMPLE_QUEUE_CAPACITY,
                                            RT_IPC_FLAG_FIFO);

    if (g_motion_sample_queue == RT_NULL)
    {
        result = -RT_ENOMEM;
        goto init_failed;
    }

    g_motion_control_queue = rt_mq_create("mot_ctl",
                                            sizeof(motion_control_message_t),
                                            MOTION_CONTROL_QUEUE_CAPACITY,
                                            RT_IPC_FLAG_FIFO);

    if (g_motion_control_queue == RT_NULL)
    {
        result = -RT_ENOMEM;
        goto init_failed;
    }

    g_motion_wake_event = rt_event_create("mot_evt", RT_IPC_FLAG_FIFO);

    if (g_motion_wake_event == RT_NULL)
    {
        result = -RT_ENOMEM;
        goto init_failed;
    }

    g_motion_publish_mutex = rt_mutex_create("mot_pub", RT_IPC_FLAG_PRIO);

    if (g_motion_publish_mutex == RT_NULL)
    {
        result = -RT_ENOMEM;
        goto init_failed;
    }

    /*
     * 此时没有运行中的worker，ready也是false。
     * 可以初始化worker私有状态和发布缓冲区。
     * 不清整个shared，避免抹掉初始化和拒绝统计。
     */
    memset(&g_motion_worker, 0, sizeof(g_motion_worker));
    memset(&g_motion_published, 0, sizeof(g_motion_published));

    core_result = motion_core_init( &g_motion_worker.core,
                                    RT_TICK_PER_SECOND,
                                    MOTION_ALGORITHM_B0,
                                    (uint32_t)rt_tick_get());

    if (core_result != MOTION_CORE_OK)
    {
        result = -RT_ERROR;
        goto init_failed;
    }

    motion_trace_worker_init();

    /*
     * 第三阶段：直接建立首次STOPPED发布副本。
     * 不依赖可能要求ready=true的服务发布函数。
     * 此时没有并发访问者，因此不需要获取publish_mutex。
     */
    core_result = motion_core_get_snapshot( &g_motion_worker.core,
                                            (uint32_t)rt_tick_get(),
                                            &g_motion_worker.snapshot_work);

    if (core_result != MOTION_CORE_OK)
    {
        result = -RT_ERROR;
        goto init_failed;
    }

    g_motion_published.snapshot = g_motion_worker.snapshot_work;

    g_motion_published.snapshot.publish_seq = 1U;
    g_motion_published.snapshot.published_tick = (uint32_t)rt_tick_get();

    /*
     * 核心刚初始化，消费累计均为0。
     * 生产、控制和初始化统计仍由get_stats按约定合并。
     */
    g_motion_published.stats.captured_tick = g_motion_published.snapshot.published_tick;

    g_motion_published.stats.accounting_stable = false;
    g_motion_published.worker_revision = 0U;
    g_motion_published.valid = true;

    level = rt_hw_interrupt_disable();

    g_motion_shared.worker_busy = false;
    g_motion_shared.publication_current = false;
    g_motion_shared.worker_revision = 0U;
    g_motion_shared.receive_inflight = 0U;
    g_motion_shared.admitted_generation = 0U;

    rt_hw_interrupt_enable(level);

    /* 创建线程对象，此时尚未运行。 */
    g_motion_thread = rt_thread_create( "motion",
                                        motion_service_thread_entry,
                                        RT_NULL,
                                        MOTION_THREAD_STACK_SIZE,
                                        MOTION_THREAD_PRIORITY,
                                        MOTION_THREAD_TIMESLICE);

    if (g_motion_thread == RT_NULL)
    {
        result = -RT_ENOMEM;
        goto init_failed;
    }

    /*
     * 第四阶段：注册逐样本入口。
     * 注册成功不代表sensor已经启动，也不直接产生样本。
     */
    result = sensor_service_register_imu_sink(motion_service_sensor_sink, RT_NULL);

    if (result != RT_EOK)
    {
        goto init_failed;
    }

    sink_registered = true;

    /*
     * 所有资源和私有状态都已准备好，最后启动线程。
     * worker即使立即运行，也会先停在ready门禁处。
     */
    result = rt_thread_startup(g_motion_thread);

    if (result != RT_EOK)
    {
        goto init_failed;
    }

    /*
     * 从这里开始，线程已经启动。
     * 后面不能再memset上下文或跳入资源清理。
     */
    level = rt_hw_interrupt_disable();

    g_motion_shared.ready = true;
    g_motion_shared.initializing = false;

    rt_hw_interrupt_enable(level);

    /*
     * 唤醒失败由辅助函数统计。
     * 线程已启动且有周期检查，不能因此销毁运行中的资源。
     */
    (void)motion_service_wake();

    return RT_EOK;

init_failed:
    /*
     * 只注销本次确实注册成功的回调。
     * 若注册失败，不能把原有的其他接收者注销。
     */
    if (sink_registered)
    {
        cleanup_result =
            sensor_service_register_imu_sink(RT_NULL, RT_NULL);

        if (cleanup_result != RT_EOK)
        {
            /*
             * 不能确认回调已经解除，不继续释放相关资源。
             * 保留句柄并锁存故障，禁止下一次初始化覆盖它们。
             */
            keep_resources = true;
            result = cleanup_result;
        }
    }

    if (!keep_resources)
    {
        /*
         * 仅用于线程尚未成功启动的失败路径。
         * 按依赖逆序清理，不用于运行期FAULT恢复。
         */
        if (g_motion_thread != RT_NULL)
        {
            (void)rt_thread_delete(g_motion_thread);
            g_motion_thread = RT_NULL;
        }

        if (g_motion_publish_mutex != RT_NULL)
        {
            (void)rt_mutex_delete(g_motion_publish_mutex);
            g_motion_publish_mutex = RT_NULL;
        }

        if (g_motion_wake_event != RT_NULL)
        {
            (void)rt_event_delete(g_motion_wake_event);
            g_motion_wake_event = RT_NULL;
        }

        if (g_motion_control_queue != RT_NULL)
        {
            (void)rt_mq_delete(g_motion_control_queue);
            g_motion_control_queue = RT_NULL;
        }

        if (g_motion_sample_queue != RT_NULL)
        {
            (void)rt_mq_delete(g_motion_sample_queue);
            g_motion_sample_queue = RT_NULL;
        }

        g_motion_published.valid = false;
    }

    level = rt_hw_interrupt_disable();

    g_motion_shared.ready = false;
    g_motion_shared.accepting_samples = false;
    g_motion_shared.initializing = false;

    if (keep_resources)
    {
        g_motion_shared.faulted = true;
    }

    if (g_motion_shared.init_failures < UINT32_MAX)
    {
        g_motion_shared.init_failures++;
    }

    if (g_motion_shared.init_failures == UINT32_MAX)
    {
        g_motion_shared.counters_saturated = true;
    }

    rt_hw_interrupt_enable(level);

    return result;
}

/*
 * 简介：非阻塞提交START请求。
 * 输入：request_id非RT_NULL，失败时输出0。
 * 返回值：RT_EOK仅表示受理；失败返回投递或参数错误。
 * 说明：实际会话建立和结果发布由worker完成。
 */
rt_err_t motion_service_request_start(uint32_t *request_id)
{
    return motion_service_post_request(MOTION_REQUEST_START, MOTION_TEST_NONE, 0U, request_id);
}

/*
 * 简介：非阻塞提交STOP请求。
 * 输入：request_id非RT_NULL，失败时输出0。
 * 返回值：RT_EOK仅表示受理，完成结果按ID查询。
 * 说明：不能在调用线程中直接修改core或等待sensor。
 */
rt_err_t motion_service_request_stop(uint32_t *request_id)
{
    return motion_service_post_request(MOTION_REQUEST_STOP, MOTION_TEST_NONE, 0U, request_id);
}

/*
 * 简介：查询有限结果槽中的一条控制结果。
 * 输入：request_id非0；result为非RT_NULL输出缓冲区。
 * 返回值：找到RT_EOK；未找到-RT_EEMPTY；参数非法-RT_EINVAL。
 * 说明：短临界区只复制一条记录；RT_EOK不等于请求已成功完成。
 */
rt_err_t motion_service_get_result(uint32_t request_id,motion_request_result_t *result)
{
    rt_base_t level;
    uint32_t index;
    motion_request_result_t value;

    /* 0是空槽标记，必须在扫描之前拒绝。 */
    if ((request_id == 0U) || (result == RT_NULL))
    {
        return -RT_EINVAL;
    }

    level = rt_hw_interrupt_disable();

    if (!g_motion_shared.ready)
    {
        rt_hw_interrupt_enable(level);
        return -RT_ERROR;
    }

    for (index = 0U; index < MOTION_RESULT_SLOT_COUNT; index++)
    {
        if (g_motion_shared.results[index].request_id == request_id)
        {
            /*
             * 匹配ID和复制记录在同一临界区完成。
             * 这样可避免复制期间worker写入终态，
             * 或投递方回收已完成的槽位。
             */
            value = g_motion_shared.results[index];

            rt_hw_interrupt_enable(level);

            *result = value;
            return RT_EOK;
        }
    }

    /* ID可能从未受理，也可能对应已被回收的终态记录。 */
    rt_hw_interrupt_enable(level);
    return -RT_EEMPTY;
}

/*
 * 简介：取得线程间发布的运动摘要副本。
 * 输入：snapshot为非RT_NULL输出缓冲区。
 * 返回值：成功RT_EOK；有界取锁失败-RT_EBUSY；其他见公开声明。
 * 说明：不访问core、sensor、RTC或LVGL；取锁只做固定长度复制。
 */
rt_err_t motion_service_get_snapshot(motion_snapshot_t *snapshot)
{
    motion_snapshot_t value;
    rt_mutex_t mutex;
    rt_base_t level;
    rt_tick_t now;
    uint32_t elapsed_ticks;
    uint64_t elapsed_ms;
    uint64_t age_ms;
    bool valid;
    rt_err_t result;

    if (snapshot == RT_NULL)
    {
        return -RT_EINVAL;
    }

    /* ready和发布锁句柄成对读取。 */
    level = rt_hw_interrupt_disable();
    mutex = g_motion_shared.ready ? g_motion_publish_mutex : RT_NULL;
    rt_hw_interrupt_enable(level);

    if (mutex == RT_NULL)
    {
        return -RT_ERROR;
    }

    result = rt_mutex_take(mutex,(rt_int32_t)motion_service_ms_to_ticks(MOTION_COPY_TIMEOUT_MS));

    if (result != RT_EOK)
    {
        return -RT_EBUSY;
    }

    valid = g_motion_published.valid;

    if (valid)
    {
        value = g_motion_published.snapshot;
    }

    if (rt_mutex_release(mutex) != RT_EOK)
    {
        return -RT_ERROR;
    }

    if (!valid)
    {
        /* 资源已就绪，但worker尚未完成首次发布。 */
        return -RT_EBUSY;
    }

    now = rt_tick_get();
    elapsed_ticks = (uint32_t)(now - value.published_tick);

    if (elapsed_ticks >= UINT32_C(0x80000000))
    {
        /* 时间跨度不可解释，返回过期状态和不可用年龄。 */
        value.quality_flags |= MOTION_QUALITY_INPUT_STALE;
        value.processed_age_ms = UINT32_MAX;
    }
    else
    {
        elapsed_ms = (uint64_t)elapsed_ticks * UINT64_C(1000) /
                     (uint64_t)RT_TICK_PER_SECOND;

        if (value.has_processed)
        {
            age_ms = (uint64_t)value.processed_age_ms + elapsed_ms;
            value.processed_age_ms =(age_ms > (uint64_t)UINT32_MAX) ?UINT32_MAX : (uint32_t)age_ms;
        }
        else
        {
            value.processed_age_ms = UINT32_MAX;
        }

        if (elapsed_ms > (uint64_t)MOTION_PUBLISH_STALE_MS)
        {
            value.quality_flags |= MOTION_QUALITY_INPUT_STALE;
        }
    }

    /* 成功路径才写调用者的输出。 */
    *snapshot = value;
    return RT_EOK;
}

/*
 * 简介：合并已发布消费统计与短临界区生产/控制统计。
 * 输入：stats为非RT_NULL输出缓冲区。
 * 返回值：成功取得副本RT_EOK；不能取得副本-RT_EBUSY。
 * 说明：跨组无法一致时accounting_stable=false，不阻塞生产者求一致。
 */
rt_err_t motion_service_get_stats(motion_service_stats_t *stats)
{
    motion_service_stats_t value;
    motion_ingress_stats_t ingress;
    rt_mutex_t mutex;
    uint64_t published_revision;
    uint64_t current_revision;
    uint64_t outstanding;
    uint32_t control_inflight;
    rt_base_t level;
    rt_err_t result;
    bool valid;
    bool worker_busy;
    bool shared_saturated;
    bool publication_current;

    if (stats == RT_NULL)
    {
        return -RT_EINVAL;
    }

    level = rt_hw_interrupt_disable();
    mutex = g_motion_shared.ready ? g_motion_publish_mutex : RT_NULL;
    rt_hw_interrupt_enable(level);

    if (mutex == RT_NULL)
    {
        return -RT_ERROR;
    }

    result = rt_mutex_take(mutex,(rt_int32_t)motion_service_ms_to_ticks(MOTION_COPY_TIMEOUT_MS));

    if (result != RT_EOK)
    {
        return -RT_EBUSY;
    }

    valid = g_motion_published.valid;

    if (valid)
    {
        /* 消费统计与版本必须从同一次发布取得。 */
        value = g_motion_published.stats;
        published_revision = g_motion_published.worker_revision;
    }

    if (rt_mutex_release(mutex) != RT_EOK)
    {
        return -RT_ERROR;
    }

    if (!valid)
    {
        return -RT_EBUSY;
    }

    /*
     * 发布锁已经释放。短临界区内只复制生产侧小结构和共享标量。
     * 版本、忙标志和在途计数在同一窗口内取得。
     */
    level = rt_hw_interrupt_disable();

    ingress = g_motion_shared.ingress;
    current_revision = g_motion_shared.worker_revision;
    worker_busy = g_motion_shared.worker_busy;
    control_inflight = g_motion_shared.control_inflight;
    shared_saturated = g_motion_shared.counters_saturated;
    publication_current = g_motion_shared.publication_current;

    value.receive_inflight = g_motion_shared.receive_inflight;
    value.init_attempts = g_motion_shared.init_attempts;
    value.init_failures = g_motion_shared.init_failures;
    value.control_post_attempts = g_motion_shared.control_post_attempts;
    value.control_post_accepted = g_motion_shared.control_post_accepted;
    value.control_post_rejected = g_motion_shared.control_post_rejected;
    value.control_completed = g_motion_shared.control_completed;
    value.wake_failures = g_motion_shared.wake_failures;
    value.captured_tick = (uint32_t)rt_tick_get();

    rt_hw_interrupt_enable(level);

    /* 这些字段是累计值，直接覆盖发布副本中的对应字段。 */
    value.produced = ingress.produced;
    value.disabled_skipped = ingress.disabled_skipped;
    value.enqueue_ok = ingress.enqueue_ok;
    value.enqueue_reject_full = ingress.enqueue_reject_full;
    value.enqueue_reject_other = ingress.enqueue_reject_other;
    value.invalid_submit_calls = ingress.invalid_submit_calls;

    value.submit_inflight = ingress.inflight;
    value.queue_high_water = ingress.queue_high_water;
    value.max_submit_ticks = ingress.max_submit_ticks;

    value.counters_saturated = value.counters_saturated || shared_saturated || ingress.saturated;

    /* 默认返回可供诊断的副本，闭合条件满足后才确认待消费帧数。 */
    value.accounting_base_ready = false;
    value.accounting_stable = false;
    value.outstanding_frames = 0U;

    if (!worker_busy &&
        (current_revision == published_revision) &&
        (value.submit_inflight == 0U) &&
        (value.receive_inflight == 0U) &&
        (control_inflight == 0U) &&
        !value.counters_saturated &&
        (value.enqueue_ok >= value.dequeued))
    {
        /* 先比较再相减，避免无符号下溢。 */
        outstanding = value.enqueue_ok - value.dequeued;

        if (outstanding <= (uint64_t)UINT32_MAX)
        {
            value.accounting_base_ready = true;
            if (publication_current)
            {
                value.outstanding_frames = (uint32_t)outstanding;
                value.accounting_stable = true;
            }
        }
    }

    *stats = value;
    return RT_EOK;
}

/*
 * 简介：从唯一sensor生产者复制并非阻塞投递一帧。
 * 输入：sample非RT_NULL，仅本次调用有效；服务不得保留地址。
 * 返回值：入队成功RT_EOK；满-RT_EFULL；停用-RT_EBUSY；
 *         参数非法-RT_EINVAL；服务不可用-RT_ERROR。
 * 说明：不取得publish_mutex，不执行算法，不打印或等待。
 */
rt_err_t motion_service_submit_imu(const motion_imu_sample_t *sample)
{
    motion_sample_frame_t frame;
    rt_mq_t queue;
    rt_base_t level;
    rt_tick_t started_tick;
    rt_tick_t finished_tick;
    rt_tick_t elapsed_ticks;
    rt_err_t result;
    uint32_t generation;

    started_tick = rt_tick_get();

    /* 第一段短临界区：检查状态，预留本次投递。 */
    level = rt_hw_interrupt_disable();

    if (sample == RT_NULL)
    {
        /*
         * 尚未初始化时不操作统计。
         * 空指针属于接口误用，不计入produced。
         */
        if (g_motion_shared.ready)
        {
            motion_service_add_u64(&g_motion_shared.ingress.invalid_submit_calls,
                                    UINT64_C(1),
                                    &g_motion_shared.ingress.saturated);
        }

        rt_hw_interrupt_enable(level);
        return -RT_EINVAL;
    }

    if (!g_motion_shared.ready || g_motion_shared.faulted ||
        (g_motion_sample_queue == RT_NULL))
    {
        rt_hw_interrupt_enable(level);
        return -RT_ERROR;
    }

    motion_service_add_u64(&g_motion_shared.ingress.produced,
                        UINT64_C(1),
                        &g_motion_shared.ingress.saturated);

    if (!g_motion_shared.accepting_samples)
    {
        motion_service_add_u64(&g_motion_shared.ingress.disabled_skipped,
                                UINT64_C(1),
                                &g_motion_shared.ingress.saturated);

        rt_hw_interrupt_enable(level);
        return -RT_EBUSY;
    }

    /*
     * 接纳门、运行代际、队列句柄一起捕获。
     * STOP随后即使关闭接纳门，也必须等待本次inflight结束。
     */
    g_motion_shared.ingress.inflight++;

    generation = g_motion_shared.admitted_generation;
    queue = g_motion_sample_queue;

    rt_hw_interrupt_enable(level);

    /*
     * 锁外准备完整值副本。
     * 清零也使结构体填充字节具有确定内容。
     */
    memset(&frame, 0, sizeof(frame));

    frame.imu = *sample;
    frame.run_generation = generation;

    /*
     * 日历不可用不阻断计步。
     * 防御性清除失败路径可能留下的部分标签。
     */
    result = motion_service_read_calendar(&frame.calendar);

    if (result != RT_EOK)
    {
        memset(&frame.calendar, 0, sizeof(frame.calendar));
    }

    /*
     * sample_tick保留采样完成时刻。
     * enqueue_tick另行记录投递时刻，二者不能互相替代。
     */
    frame.enqueue_tick = (uint32_t)rt_tick_get();

    /* 非阻塞发送；不能包在项目的关中断临界区中。 */
    result = rt_mq_send(queue, &frame, sizeof(frame));

    /* 第二段短临界区：登记本次发送的唯一结果。 */
    level = rt_hw_interrupt_disable();

    if (result == RT_EOK)
    {
        motion_service_add_u64(
            &g_motion_shared.ingress.enqueue_ok,
            UINT64_C(1),
            &g_motion_shared.ingress.saturated);

        /*
         * 当前SDK的entry表示实际队列驻留消息数量。
         * 本项目sensor优先级19，motion worker优先级20，
         * 唯一消费者不会在当前生产者运行期间抢占消费。
         */
        if ((uint32_t)queue->entry > g_motion_shared.ingress.queue_high_water)
        {
            g_motion_shared.ingress.queue_high_water = (uint32_t)queue->entry;
        }
    }
    else if (result == -RT_EFULL)
    {
        motion_service_add_u64(
            &g_motion_shared.ingress.enqueue_reject_full,
            UINT64_C(1),
            &g_motion_shared.ingress.saturated);
    }
    else
    {
        motion_service_add_u64(
            &g_motion_shared.ingress.enqueue_reject_other,
            UINT64_C(1),
            &g_motion_shared.ingress.saturated);
    }

    rt_hw_interrupt_enable(level);

    /*
     * 唤醒发生在临界区外。
     * 唤醒失败由辅助函数登记，不能覆盖MQ发送结果，
     * 否则调用者可能误以为没有入队，再次提交同一帧。
     */
    if (result == RT_EOK)
    {
        (void)motion_service_wake();
    }

    /* 第三段短临界区：完成耗时记录，结束本次在途投递。 */
    level = rt_hw_interrupt_disable();

    finished_tick = rt_tick_get();
    elapsed_ticks = finished_tick - started_tick;

    if ((uint32_t)elapsed_ticks >
        g_motion_shared.ingress.max_submit_ticks)
    {
        g_motion_shared.ingress.max_submit_ticks =
            (uint32_t)elapsed_ticks;
    }

    g_motion_shared.ingress.inflight--;

    rt_hw_interrupt_enable(level);

    return result;
}
//---------------------------------------------------------------私有函数---------------------------------------------------------------
/*
 * 简介：将正数毫秒向上换算为至少1tick。
 * 输入：milliseconds为需要等待的正数毫秒。
 * 返回值：非0tick数量。
 * 说明：使用64位中间值，避免小等待被截断为0。
 */
static rt_tick_t motion_service_ms_to_ticks(uint32_t milliseconds)
{
 uint64_t ticks;

    /* 加999后整除1000，实现向上取整。 */
    ticks = ((uint64_t)milliseconds * (uint64_t)RT_TICK_PER_SECOND +UINT64_C(999)) / UINT64_C(1000);

    /* 防御性处理，确保等待参数至少为1 tick。 */
    if (ticks == UINT64_C(0))
    {
        return (rt_tick_t)1U;
    }

    if (ticks > (uint64_t)INT32_MAX)
    {
        return (rt_tick_t)INT32_MAX;
    }

    return (rt_tick_t)ticks;
}

/*
 * 简介：执行受保护的64位饱和累计。
 * 输入：value/saturated非NULL，保护由调用者负责。
 * 返回值：无。
 * 说明：先检查剩余容量，达到上限时锁存，不静默回绕。
 */
static void motion_service_add_u64(uint64_t *value, uint64_t increment, bool *saturated)
{
    if (increment == UINT64_C(0))
    {
        return;
    }

    /* 先比较剩余容量，避免先相加产生回绕。 */
    if (increment >= (UINT64_MAX - *value))
    {
        *value = UINT64_MAX;
        *saturated = true;
        return;
    }

    *value += increment;
}

/*
 * 简介：发送合并唤醒位。
 * 输入：无。
 * 返回值：rt_event_send的实际结果。
 * 说明：失败只登记wake_failures，不改变已经成功入队的消息归属。
 */
static rt_err_t motion_service_wake(void)
{
    rt_err_t result;
    rt_base_t level;

    if (g_motion_wake_event == RT_NULL)
    {
        result = -RT_EINVAL;
    }
    else
    {
        /* 发送事件可能唤醒worker，放在共享状态临界区外。 */
        result = rt_event_send(g_motion_wake_event, MOTION_WAKE_WORK);
    }

    if (result != RT_EOK)
    {
        /* 只保护共享计数和饱和标志的更新。 */
        level = rt_hw_interrupt_disable();

        motion_service_add_u64(&g_motion_shared.wake_failures,UINT64_C(1),&g_motion_shared.counters_saturated);

        rt_hw_interrupt_enable(level);
    }

    return result;
}

/*
 * 简介：把时间服务副本转换为运动日历标签。
 * 输入：tag非RT_NULL，输出由调用者持有。
 * 返回值：成功读取RT_EOK；读取失败返回实际错误且标签保持无效。
 * 说明：只调用time_service_get_snapshot，不读RTC。
 *       合法年份本身不构成可信日期。
 */
static rt_err_t motion_service_read_calendar(motion_calendar_tag_t *tag)
{
    time_snapshot_t time_value;
    rt_err_t result;

    if (tag == RT_NULL)
    {
        return -RT_EINVAL;
    }

    /* 先清空输出，避免读取失败后沿用上一次的可信日期。 */
    memset(tag, 0, sizeof(*tag));

    result = time_service_get_snapshot(&time_value);

    if (result != RT_EOK)
    {
        return result;
    }

    /* 即使当前日期不可归属，也保留代际和日期变化序号。 */
    tag->calendar_generation = time_value.calendar_generation;
    tag->day_seq = time_value.day_seq;

    /*
     * READ_ERROR表示最近一次RTC读取失败。
     * 若上一次有效值仍可信且新鲜，允许继续使用其日期。
     */
    if (time_value.service_running &&
        time_value.has_value &&
        time_value.trusted &&
        time_value.fresh &&
        ((time_value.state == TIME_CLOCK_READY) ||
         (time_value.state == TIME_CLOCK_READ_ERROR)) &&
        (time_value.local_day_key != 0U))
    {
        tag->local_day_key = time_value.local_day_key;
        tag->day_valid = true;
    }

    return RT_EOK;
}

/*
 * 简介：把sensor类型逐字段转换为motion输入并投递。
 * 输入：sample只在回调期间有效；imu_seq为该份提交序号；
 *       user_data本方案不用，允许RT_NULL。
 * 返回值：motion_service_submit_imu的实际结果。
 * 说明：sensor线程、快照锁外；禁止保存指针或强制转换结构体指针。
 */
static rt_err_t motion_service_sensor_sink(const sensor_imu_sample_t *sample,rt_uint32_t imu_seq,void *user_data)
{
    motion_imu_sample_t value;

    /* 明确表示当前未使用该参数，避免未使用参数警告。 */
    (void)user_data;

    if (sample == RT_NULL)
    {
        return -RT_EINVAL;
    }

    memset(&value, 0, sizeof(value));

    /* 基础身份：采样时刻、成功提交序号和配置版本。 */
    value.sample_tick = (uint32_t)sample->sample_tick;
    value.imu_seq = imu_seq;
    value.config_version = sample->config_version;

    /* 数组轴序固定为：0=X，1=Y，2=Z。 */
    value.accel_raw[0] = sample->accel_raw.x;
    value.accel_raw[1] = sample->accel_raw.y;
    value.accel_raw[2] = sample->accel_raw.z;

    value.gyro_raw[0] = sample->gyro_raw.x;
    value.gyro_raw[1] = sample->gyro_raw.y;
    value.gyro_raw[2] = sample->gyro_raw.z;

    /* 已换算数据直接复制，不在这里重复换算单位。 */
    value.accel_ug[0] = sample->accel_ug.x;
    value.accel_ug[1] = sample->accel_ug.y;
    value.accel_ug[2] = sample->accel_ug.z;

    value.gyro_udps[0] = sample->gyro_udps.x;
    value.gyro_udps[1] = sample->gyro_udps.y;
    value.gyro_udps[2] = sample->gyro_udps.z;

    value.status_reg = sample->status_reg;
    value.valid = (sample->valid != RT_FALSE);

    /* submit会同步复制value，不会保存这个局部变量的地址。 */
    return motion_service_submit_imu(&value);
}

/*
 * 简介：预留结果记录并非阻塞提交控制消息。
 * 输入：kind为START/STOP/TEST/TRACE；fault/argument按请求种类验证；ID非空，失败输出0。
 * 返回值：RT_EOK表示受理；参数/资源/队列错误返回负值。
 * 说明：请求ID非0；不得回收PENDING/RUNNING槽位。
 */
static rt_err_t motion_service_post_request(motion_request_kind_t kind, motion_test_fault_t fault, uint32_t argument, uint32_t *request_id)
{
    motion_control_message_t message;
    rt_mq_t queue;

    uint32_t slot;
    uint32_t reusable_slot;
    uint32_t candidate_id;
    uint32_t index;
    uint32_t attempt;

    rt_base_t level;
    rt_err_t result;

    bool collision;

    if (request_id == RT_NULL)
    {
        return -RT_EINVAL;
    }

    *request_id = 0U;

    if (!motion_service_control_args_valid(kind, fault, argument)) return -RT_EINVAL;

    /*
     * 第一段临界区：检查服务、选择结果槽、分配ID，
     * 建立PENDING记录。
     */
    level = rt_hw_interrupt_disable();

    motion_service_add_u64( &g_motion_shared.control_post_attempts,
                            UINT64_C(1),
                            &g_motion_shared.counters_saturated);

    if (!g_motion_shared.ready || g_motion_shared.faulted ||
        (g_motion_control_queue == RT_NULL))
    {
        result = -RT_ERROR;
        goto reject_before_reservation;
    }

    /*
     * 优先使用空槽。
     * 没有空槽时，才回收一条已完成记录。
     */
    slot = MOTION_RESULT_SLOT_COUNT;            //最终选中的槽位；扫描时优先记录空槽
    reusable_slot = MOTION_RESULT_SLOT_COUNT;   //备用槽位，记录第一条已经完成、允许回收的结果

    for (index = 0U; index < MOTION_RESULT_SLOT_COUNT; index++)
    {
        if (g_motion_shared.results[index].request_id == 0U)
        {
            slot = index;
            break;
        }

        if ((reusable_slot == MOTION_RESULT_SLOT_COUNT) &&
            ((g_motion_shared.results[index].state == MOTION_RESULT_OK) ||
             (g_motion_shared.results[index].state == MOTION_RESULT_FAILED)))
        {
            reusable_slot = index;
        }
    }

    if (slot == MOTION_RESULT_SLOT_COUNT)
    {
        slot = reusable_slot;
    }

    if (slot == MOTION_RESULT_SLOT_COUNT)
    {
        /*
         * 所有槽都属于未完成请求，不能覆盖。
         * 此时还没有发送任何控制消息。
         */
        result = -RT_EFULL;
        goto reject_before_reservation;
    }

    /*
     * 分配非0请求ID，并避开所有仍保留的记录。
     *
     * 最多有8个已占用ID，检查9个不同候选就足够。
     * 循环有固定上界，不能无限搜索。
     */
    candidate_id = g_motion_shared.next_request_id;
    collision = true;

    for (attempt = 0U; attempt <= MOTION_RESULT_SLOT_COUNT; attempt++)
    {
        candidate_id++;

        if (candidate_id == 0U)
        {
            candidate_id = 1U;
        }

        collision = false;

        for (index = 0U; index < MOTION_RESULT_SLOT_COUNT; index++)
        {
            if (g_motion_shared.results[index].request_id == candidate_id)
            {
                collision = true;
                break;
            }
        }

        if (!collision)
        {
            break;
        }
    }

    if (collision)
    {
        /* 按当前容量设计不应发生，保留防御路径。 */
        result = -RT_EFULL;
        goto reject_before_reservation;
    }

    g_motion_shared.next_request_id = candidate_id;

    memset(&g_motion_shared.results[slot], 0,sizeof(g_motion_shared.results[slot]));

    g_motion_shared.results[slot].request_id = candidate_id;
    g_motion_shared.results[slot].kind = kind;
    g_motion_shared.results[slot].state = MOTION_RESULT_PENDING;

    /*
     * 在途标志覆盖预留、发送、结果记账和唤醒。
     * worker可能在发送返回之前就已经执行请求。
     */
    g_motion_shared.control_inflight++;

    queue = g_motion_control_queue;

    rt_hw_interrupt_enable(level);

    /* 锁外构造并发送控制消息。 */
    memset(&message, 0, sizeof(message));
    message.request_id = candidate_id;
    message.kind = kind;
    message.fault = fault;
    message.argument = argument;

    result = rt_mq_send(queue, &message, sizeof(message));

    /*
     * 第二段临界区：登记投递结果。
     * 成功时不能再次把结果槽写成PENDING：
     * worker此时可能已经将它更新为RUNNING或终态。
     */
    level = rt_hw_interrupt_disable();

    if (result == RT_EOK)
    {
        motion_service_add_u64(
            &g_motion_shared.control_post_accepted,
            UINT64_C(1),
            &g_motion_shared.counters_saturated);
    }
    else
    {
        /*
         * 仅撤销本次仍处于PENDING的预留记录。
         * 不修改其他请求，也不覆盖已经完成的记录。
         */
        if ((g_motion_shared.results[slot].request_id ==
             candidate_id) &&
            (g_motion_shared.results[slot].state ==
             MOTION_RESULT_PENDING))
        {
            memset(&g_motion_shared.results[slot],
                   0,
                   sizeof(g_motion_shared.results[slot]));
        }

        motion_service_add_u64(
            &g_motion_shared.control_post_rejected,
            UINT64_C(1),
            &g_motion_shared.counters_saturated);
    }

    rt_hw_interrupt_enable(level);

    if (result == RT_EOK)
    {
        *request_id = candidate_id;

        /*
         * 消息已经入队。
         * 唤醒失败不能把“受理成功”改成“投递失败”。
         */
        (void)motion_service_wake();
    }

    level = rt_hw_interrupt_disable();

    g_motion_shared.control_inflight--;

    rt_hw_interrupt_enable(level);

    return result;

reject_before_reservation:
    /*
     * 此路径尚未预留记录，也没有增加control_inflight。
     * 只登记拒绝并恢复中断。
     */
    motion_service_add_u64(
        &g_motion_shared.control_post_rejected,
        UINT64_C(1),
        &g_motion_shared.counters_saturated);

    rt_hw_interrupt_enable(level);

    return result;
}

/*
 * 简介：运行唯一运动worker，串行处理控制、样本和状态发布。
 * 输入：parameter本方案不用，允许RT_NULL。
 * 返回值：无，正常运行不退出。
 * 说明：初始化必须在线程启动前准备好全部资源及核心。
 *       核心和worker私有状态仅由本线程修改。
 *       故障后关闭接纳门，保留资源和统计，不在运行期删除资源。
 */
static void motion_service_thread_entry(void *parameter)
{
    motion_control_message_t control;
    rt_uint32_t received_events;

    rt_err_t result;
    rt_err_t pending_wait_error;

    rt_err_t final_publish_result;

    int core_result;
    uint32_t count;
    rt_base_t level;
    bool backlog;
    bool service_ready;

    (void)parameter;

    pending_wait_error = RT_EOK;

    for (;;)
    {
        level = rt_hw_interrupt_disable();
        service_ready = g_motion_shared.ready;
        rt_hw_interrupt_enable(level);

        if (service_ready)
        {
            break;
        }

        rt_thread_delay(1);
    }

    for (;;)
    {
        /*
         * 开始一轮修改。
         * 查询线程看到worker_busy时，不能宣称跨组统计一致。
         */
        level = rt_hw_interrupt_disable();

        g_motion_shared.worker_busy = true;
        g_motion_shared.publication_current = false;
        g_motion_shared.worker_revision++;

        rt_hw_interrupt_enable(level);

        /*
         * 等待阶段发生的错误，移到本轮受保护的修改窗口内处理。
         * 避免worker_busy=false时修改核心和控制结果。
         */
        if (pending_wait_error != RT_EOK)
        {
            motion_service_enter_fault(pending_wait_error);
            pending_wait_error = RT_EOK;
        }

        /* 更新当前日期和数据源健康状态，不用快照代替逐样本输入。 */
        motion_service_update_inputs();
        motion_service_test_poll();

        /*
         * 每轮最多处理一条控制消息。
         * STOP握手未结束时，不能越过它执行后续START。
         */
        if (g_motion_worker.pending_stop_id == 0U)
        {
            result = rt_mq_recv(g_motion_control_queue, &control, sizeof(control), 0);

            if (result == RT_EOK)
            {
                motion_service_handle_control(&control);
            }
            else if (result != -RT_ETIMEOUT)
            {
                motion_service_enter_fault(result);
            }
        }

        motion_trace_worker_poll(&g_motion_worker.core, g_motion_worker.test.status.active,
            g_motion_worker.pending_stop_id != 0U, (uint32_t)rt_tick_get());

        /*
         * 每轮有界处理样本。
         * 即使正在STOP，也继续取出残留帧，让核心完成丢弃记账。
         */
        result = motion_service_test_prepare_round();
        if (result != RT_EOK && result != -RT_EBUSY && result != -RT_ETIMEOUT)
            motion_service_enter_fault(result);
        for (count = 0U; count < MOTION_SAMPLE_BATCH_MAX && !motion_service_test_pause_active(); count++)
        {
            result = motion_service_process_one();

            if (result == -RT_ETIMEOUT)
            {
                /* 当前队列为空，正常结束本批。 */
                break;
            }

            if (result != RT_EOK)
            {
                /*
                 * process_one通常已执行故障处置。
                 * 防御性检查，避免漏掉尚未关闭接纳门的错误。
                 */
                if (g_motion_worker.core.snapshot.state != MOTION_SERVICE_FAULT)
                {
                    motion_service_enter_fault(result);
                }

                break;
            }
        }

        /*
         * 即使本轮没有样本，也推进输入超时。
         * 必须使用此时的新tick，不复用读取状态之前的旧值。
         */
        core_result = motion_core_tick( &g_motion_worker.core, (uint32_t)rt_tick_get());

        if (core_result != MOTION_CORE_OK)
        {
            motion_service_enter_fault(-RT_ERROR);
        }

        motion_trace_worker_poll(&g_motion_worker.core, g_motion_worker.test.status.active,
            g_motion_worker.pending_stop_id != 0U, (uint32_t)rt_tick_get());

        /* 不等待，只推进一次STOP完成条件。 */
        motion_service_poll_stop();

        /*
         * 本轮有限工作已经完成。
         * 此字段表示worker有执行进展，不单独代表业务健康。
         */
        g_motion_worker.worker_progress_tick = (uint32_t)rt_tick_get();

        final_publish_result = motion_service_test_fail_final() ?
                               -RT_EBUSY : motion_service_publish();

        if ((final_publish_result != RT_EOK) &&
            (final_publish_result != -RT_EBUSY) &&
            (final_publish_result != -RT_ETIMEOUT))
        {
            /*
             * 短暂取锁失败留待下一轮重试；
             * 其他发布错误进入受控故障。
             * 不在这里递归重试发布。
             */
            motion_service_enter_fault(final_publish_result);
        }

        /*
         * 结束本轮修改。
         * 中途发布可能同版本；末次失败由publication_current撤销一致性，
         * getter应保持accounting_stable=false。
         */
        level = rt_hw_interrupt_disable();

        g_motion_shared.publication_current = (final_publish_result == RT_EOK);
        g_motion_shared.worker_busy = false;

        /*
         * STOP尚未结束，或任一队列有消息，都需要继续推进。
         * entry只读取，不直接修改队列内部状态。
         */
        backlog = (g_motion_worker.pending_stop_id != 0U) ||
                  (g_motion_sample_queue->entry != 0U) ||
                  (g_motion_control_queue->entry != 0U);

        rt_hw_interrupt_enable(level);

        motion_service_test_capture();

        if (backlog)
        {
            /*
             * 有积压也不能一直占用CPU。
             * 真正阻塞1tick，为较低优先级线程提供运行机会。
             */
            rt_thread_delay(1);
        }
        else
        {
            /*
             * 无积压时等待事件，最多20ms。
             * 即使没有事件，也会周期醒来更新日期和超时。
             */
            received_events = 0U;

            result = rt_event_recv(g_motion_wake_event, MOTION_WAKE_WORK, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                                 (rt_int32_t)motion_service_ms_to_ticks( MOTION_WORKER_WAIT_MS),
                                &received_events);

            if ((result != RT_EOK) &&
                (result != -RT_ETIMEOUT))
            {
                /*
                 * 下一轮进入worker_busy窗口后再处置。
                 * 额外短暂阻塞，避免持续错误造成忙循环。
                 */
                pending_wait_error = result;
                rt_thread_delay(1);
            }
        }
    }
}

/*
 * 简介：更新当前日历及有界轮询得到的数据源状态。
 * 输入：无，使用worker私有缓存。
 * 返回值：无。
 * 说明：sensor快照只用于健康判断，不作为逐样本算法输入。
 */
static void motion_service_update_inputs(void)
{
    motion_calendar_tag_t calendar;
    motion_source_state_t source = MOTION_SOURCE_UNKNOWN;
    const sensor_device_status_t *imu_status;
    const rt_tick_t stale_ticks = motion_service_ms_to_ticks(MOTION_INPUT_STALE_MS);

    rt_tick_t now;
    rt_err_t result;
    int core_result;

    /*
     * 读取失败时，read_calendar已将标签清为无效。
     * 将这个无效状态交给核心，使当前日期视图及时失效。
     */
    (void)motion_service_read_calendar(&calendar);

    now = rt_tick_get();
    core_result = motion_core_update_calendar(&g_motion_worker.core, &calendar, now);

    if (core_result != MOTION_CORE_OK)
    {
        motion_service_enter_fault(-RT_ERROR);
        return;
    }

    now = rt_tick_get();

    /*
     * 无符号差小于半区间，表示已到达或经过截止时刻。
     * 每次尝试后推进截止时刻，避免读取失败时每轮都抢锁。
     */
    if ((rt_tick_t)(now - g_motion_worker.next_source_poll) < UINT32_C(0x80000000))
    {
        result = sensor_service_get_snapshot(&g_motion_worker.source_snapshot);

        now = rt_tick_get();
        g_motion_worker.next_source_poll = now + motion_service_ms_to_ticks(MOTION_SOURCE_POLL_MS);

        if (result == RT_EOK)
        {
            g_motion_worker.has_source_snapshot = true;
            g_motion_worker.last_source_read_tick = g_motion_worker.source_snapshot.snapshot_tick;
        }
        else if (result == -RT_ENOSYS)
        {
            /* sensor尚未提供快照，例如运动worker先于sensor启动。 */
            g_motion_worker.has_source_snapshot = false;
        }
        else
        {
            /*
             * 快照读取失败时，输出缓冲区保留原值。
             * 保留旧缓存的取得时刻，让它按原期限失效。
             */
            motion_service_add_u64(&g_motion_worker.sensor_status_read_failures, UINT64_C(1),
                                    &g_motion_worker.counters_saturated);
        }
    }

    now = rt_tick_get();

    /* 状态副本超过允许年龄后，撤销缓存，数据源回到UNKNOWN。 */
    if (g_motion_worker.has_source_snapshot && ((rt_tick_t)(now - g_motion_worker.last_source_read_tick) >stale_ticks))
    {
        g_motion_worker.has_source_snapshot = false;
    }

    if (g_motion_worker.has_source_snapshot)
    {
        imu_status = &g_motion_worker.source_snapshot.imu_status;

        if (imu_status->state == SENSOR_DEVICE_STATE_OFFLINE)
        {
            source = MOTION_SOURCE_OFFLINE;
        }
        else if ((imu_status->state != SENSOR_DEVICE_STATE_UNINITIALIZED) &&(imu_status->valid != RT_FALSE))
        {
            /*
             * 缓存的age_ticks只反映取得快照时的年龄。
             * 此处按当前tick重新计算样本年龄。
             */
            if ((imu_status->state == SENSOR_DEVICE_STATE_TEMP_FAILED) ||
                (imu_status->stale != RT_FALSE) ||
                ((rt_tick_t)(now - imu_status->last_success_tick) > stale_ticks))
            {
                source = MOTION_SOURCE_STALE;
            }
            else if (imu_status->state == SENSOR_DEVICE_STATE_READY)
            {
                source = MOTION_SOURCE_AVAILABLE;
            }
        }
    }

    /* 提交状态前重新取tick，保持核心调用的时间顺序。 */
    now = rt_tick_get();
    core_result = motion_core_update_source(&g_motion_worker.core, source, now);

    if (core_result != MOTION_CORE_OK)
    {
        motion_service_enter_fault(-RT_ERROR);
    }
}

/*
 * 简介：串行执行一条START或开始STOP握手。
 * 输入：message非RT_NULL，来自控制队列值副本。
 * 返回值：无。
 * 说明：仅motion worker调用；先标记请求RUNNING。
 *       START成功必须先发布摘要，再开放接纳门。
 *       STOP最终结果由poll_stop确认，不在这里等待。
 */
static void motion_service_handle_control( const motion_control_message_t *message)
{
    int core_result;
    rt_err_t publish_result;

    uint32_t session_id;
    uint32_t generation;
    uint32_t index;

    rt_base_t level;

    bool found;
    bool faulted;
    bool start_allowed;

    if ((message == RT_NULL) || (message->request_id == 0U))
    {
        motion_service_enter_fault(-RT_EINVAL);
        return;
    }

    /*
     * 第一步：确认消息与结果记录匹配，
     * 再把PENDING变成RUNNING。
     */
    found = false;

    level = rt_hw_interrupt_disable();

    //在8个结果槽中查找对应请求
    for (index = 0U; index < MOTION_RESULT_SLOT_COUNT; index++)
    {
        if (g_motion_shared.results[index].request_id == message->request_id)
        {
            if ((g_motion_shared.results[index].kind == message->kind) &&
                (g_motion_shared.results[index].state == MOTION_RESULT_PENDING))
            {
                g_motion_shared.results[index].state = MOTION_RESULT_RUNNING;

                found = true;
            }

            break;
        }
    }

    faulted = g_motion_shared.faulted;

    rt_hw_interrupt_enable(level);

    if (!found)
    {
        /*
         * 正常控制队列中的消息必须有对应PENDING记录。
         * 不匹配属于内部协议错误，不能临时创建结果槽掩盖。
         */
        motion_service_enter_fault(-RT_ERROR);
        return;
    }

    if (faulted ||!g_motion_worker.core.initialized ||
        (g_motion_worker.core.snapshot.state == MOTION_SERVICE_FAULT))
    {
        motion_service_finish_request(message->request_id, -RT_ERROR);
        return;
    }

    /*
     * 正常worker循环应在STOP完成前暂停取下一条控制消息。
     * 这里再防御一次，避免覆盖已有停止握手。
     */
    if (g_motion_worker.pending_stop_id != 0U)
    {
        motion_service_finish_request(message->request_id, -RT_EBUSY);
        return;
    }

    switch (message->kind)
    {
    case MOTION_REQUEST_TRACE_ARM:
    case MOTION_REQUEST_TRACE_CANCEL:
    case MOTION_REQUEST_TRACE_CLEAR:
        publish_result = motion_trace_worker_control(message->kind, message->request_id,
            message->argument, &g_motion_worker.core, g_motion_worker.test.status.active,
            g_motion_worker.pending_stop_id != 0U, (uint32_t)rt_tick_get());
        motion_service_finish_request(message->request_id, publish_result);
        return;

    case MOTION_REQUEST_TEST:
        publish_result = motion_service_test_apply(message);
        motion_service_finish_request(message->request_id, publish_result);
        return;

    case MOTION_REQUEST_START:
    {
        /*
         * 重复START属于业务状态不允许。
         * 不进入FAULT，也不能清正在运行的会话。
         */
        if (g_motion_worker.core.snapshot.state == MOTION_SERVICE_RUNNING)
        {
            motion_service_finish_request(message->request_id, -RT_EBUSY);
            return;
        }

        if (g_motion_worker.core.snapshot.state != MOTION_SERVICE_STOPPED)
        {
            motion_service_finish_request(message->request_id, -RT_ERROR);
            return;
        }

        /*
         * 新会话开始前，旧通道必须已经安静。
         * 短临界区只检查共享状态和实际队列数量。
         */
        level = rt_hw_interrupt_disable();

        start_allowed = !g_motion_shared.accepting_samples &&
                        !g_motion_shared.faulted &&
                        (g_motion_shared.ingress.inflight == 0U) &&
                        (g_motion_shared.receive_inflight == 0U) &&
                        (g_motion_sample_queue != RT_NULL);

        if (start_allowed)
        {
            start_allowed = (g_motion_sample_queue->entry == 0U);
        }

        rt_hw_interrupt_enable(level);

        if (!start_allowed)
        {
            motion_service_finish_request(message->request_id,
                                          -RT_EBUSY);
            return;
        }

        /* 分配新会话身份；0保留为无效值。 */
        session_id = g_motion_worker.next_session_id + 1U;

        if (session_id == 0U)
        {
            session_id = 1U;
        }

        generation = g_motion_worker.next_run_generation + 1U;

        if (generation == 0U)
        {
            generation = 1U;
        }

        g_motion_worker.next_session_id = session_id;
        g_motion_worker.next_run_generation = generation;

        core_result = motion_core_start( &g_motion_worker.core, generation,
                                        session_id, (uint32_t)rt_tick_get());

        if (core_result != MOTION_CORE_OK)
        {
            /*
             * 状态已预先检查，这里的失败属于核心/时间异常。
             * 核心错误码不直接当RT错误码使用。
             */
            motion_service_enter_fault(-RT_ERROR);
            motion_service_finish_request(message->request_id, -RT_ERROR);
            return;
        }

        /*
         * 先发布新会话。
         * 发布失败时保持接纳门关闭，不能继续正常启动。
         */
        publish_result = motion_service_publish();

        if (publish_result != RT_EOK)
        {
            motion_service_enter_fault(publish_result);
            motion_service_finish_request(message->request_id, publish_result);
            return;
        }

        /* 摘要已发布，最后才允许sensor投递新会话样本。 */
        level = rt_hw_interrupt_disable();

        g_motion_shared.admitted_generation = generation;
        g_motion_shared.accepting_samples = true;

        rt_hw_interrupt_enable(level);

        motion_service_finish_request(message->request_id,RT_EOK);
        return;
    }

    case MOTION_REQUEST_STOP:
    {
        motion_trace_worker_abort(&g_motion_worker.core, MOTION_TRACE_END_MOTION_STOP, 0, (uint32_t)rt_tick_get());
        motion_service_test_end(MOTION_TEST_END_CANCELLED);
        /*
         * 先阻止新的投递开始。
         * 已经inflight的投递仍可能继续完成，由poll_stop等待。
         */
        level = rt_hw_interrupt_disable();

        g_motion_shared.accepting_samples = false;

        rt_hw_interrupt_enable(level);

        core_result = motion_core_stop( &g_motion_worker.core, (uint32_t)rt_tick_get());

        if (core_result != MOTION_CORE_OK)
        {
            motion_service_enter_fault(-RT_ERROR);
            motion_service_finish_request(message->request_id,-RT_ERROR);
            return;
        }

        /*
         * 核心已经停止，但旧投递和残留队列可能尚未结束。
         * 请求保持RUNNING，不能现在登记OK。
         */
        g_motion_worker.pending_stop_id = message->request_id;

        g_motion_worker.stop_deadline = rt_tick_get() + motion_service_ms_to_ticks(MOTION_STOP_TIMEOUT_MS);
        //这里没有调用`finish_request(..., RT_EOK)`，因为STOP还没真正完成
        /*
        *后续由`poll_stop()`检查：
        *旧投递已经结束
        *残留队列已经处理完
        *没有在途消费
        *最终STOPPED摘要发布成功
        *条件全部满足，才能登记STOP成功。超时则登记失败并进入FAULT
        */
        return;
    }

    default:
        motion_service_finish_request(message->request_id,-RT_EINVAL);
        return;
    }
}

/*
 * 简介：推进已发起STOP的有界完成条件。
 * 输入：无。
 * 返回值：无。
 * 说明：不忙等、不持锁等待；只有旧投递和旧队列都已处理才能成功。
 */
static void motion_service_poll_stop(void)
{
    rt_base_t level;

    bool ingress_idle;
    bool queue_empty;
    bool stop_state_valid;

    rt_err_t result;
    rt_tick_t now;
    uint32_t request_id;

    request_id = g_motion_worker.pending_stop_id;

    if (request_id == 0U)
    {
        return;
    }

    /*
     * 正常发起STOP后，核心应已经处于STOPPED。
     * 不能通过此函数把仍然运行的核心冒充为已停止。
     */
    if (g_motion_worker.core.snapshot.state != MOTION_SERVICE_STOPPED)
    {
        result = -RT_ERROR;
        goto stop_failed;
    }

    /*
     * 一次性检查共享状态。
     * 接纳门关闭后，不会再有新的正常投递开始；
     * 已经开始的旧投递由inflight反映。
     */
    level = rt_hw_interrupt_disable();

    stop_state_valid = !g_motion_shared.accepting_samples &&
                       !g_motion_shared.faulted &&
                       (g_motion_sample_queue != RT_NULL);

    ingress_idle = (g_motion_shared.ingress.inflight == 0U) &&
                   (g_motion_shared.receive_inflight == 0U);

    queue_empty = false;

    if (g_motion_sample_queue != RT_NULL)
    {
        queue_empty = (g_motion_sample_queue->entry == 0U);
    }

    rt_hw_interrupt_enable(level);

    if (!stop_state_valid)
    {
        result = -RT_ERROR;
        goto stop_failed;
    }

    /*
     * 回绕安全的到期判断：
     * now恰好等于deadline时，差为0，也表示已经到期。
     * 约定期限和检查间隔均小于32位tick半区间。
     */
    now = rt_tick_get();

    if ((uint32_t)(now - g_motion_worker.stop_deadline) < UINT32_C(0x80000000))
    {
        result = -RT_ETIMEOUT;
        goto stop_failed;
    }

    /*
     * 尚有旧投递或残留消息，留给下一轮继续推进。
     * worker仍会调用process_one处理残留帧并完成消费记账。
     */
    if (!ingress_idle || !queue_empty)
    {
        return;
    }

    /*
     * 通道已经安静，发布最终STOPPED摘要和统计。
     * 发布操作不能放在关中断临界区中。
     */
    result = motion_service_publish();

    /*
     * 发布可能有最多5ms取锁等待，因此返回后重新检查期限。
     * 超过期限时，即使完成了发布，也不登记为按期成功。
     */
    now = rt_tick_get();

    if ((uint32_t)(now - g_motion_worker.stop_deadline) < UINT32_C(0x80000000))
    {
        result = -RT_ETIMEOUT;
        goto stop_failed;
    }

    if (result != RT_EOK)
    {
        /*
         * 短暂取锁失败可以在期限内重试。
         * publish自身负责登记publish_failures，这里不重复增加。
         */
        if ((result == -RT_EBUSY) ||
            (result == -RT_ETIMEOUT))
        {
            return;
        }

        goto stop_failed;
    }

    /*
     * 完成条件已经全部满足。
     * 清除握手状态后，提交请求成功终态。
     */
    g_motion_worker.pending_stop_id = 0U;
    g_motion_worker.stop_deadline = 0U;

    motion_service_finish_request(request_id, RT_EOK);
    return;

stop_failed:
    /*
     * 先保存的request_id仍有效。
     * 先清pending，避免enter_fault再次推进同一个STOP，
     * 或通过其他收尾路径形成递归处理。
     */
    g_motion_worker.pending_stop_id = 0U;
    g_motion_worker.stop_deadline = 0U;

    motion_service_enter_fault(result);
    motion_service_finish_request(request_id, result);
}

/*
 * 简介：非阻塞取出并处理最多一份样本。
 * 输入：无，使用worker静态工作缓冲。
 * 返回值：处理一帧RT_EOK；队列空-RT_ETIMEOUT；其他为实际错误。
 * 说明：真实出队后每帧必须完成一次消费记账，包含核心前置错误。
 */
static rt_err_t motion_service_process_one(void)
{
    rt_err_t result;
    int core_result = MOTION_CORE_ERR_NOT_IMPLEMENTED;

    rt_tick_t begin_tick;
    rt_tick_t end_tick;
    rt_base_t level;

    uint32_t queue_wait_ticks;
    uint32_t sample_age_ticks;
    uint32_t process_ticks;
    uint32_t index;

    bool effective;
    bool trace_core_called = false;
    bool trace_injected = false;

    if (g_motion_sample_queue == RT_NULL)
    {
        motion_service_enter_fault(-RT_ERROR);
        return -RT_ERROR;
    }

    /*
     * 出队前登记在途状态。
     * 即使本次最终发现队列为空，也通过统一出口清除。
     */
    level = rt_hw_interrupt_disable();
    g_motion_shared.receive_inflight = 1U;
    rt_hw_interrupt_enable(level);

    result = rt_mq_recv(g_motion_sample_queue,&g_motion_worker.frame,sizeof(g_motion_worker.frame),0);

    if (result != RT_EOK)
    {
        /*
         * timeout=0时，空队列返回-RT_ETIMEOUT。
         * 没有真实取出消息，因此不能增加任何消费数量。
         */
        if (result != -RT_ETIMEOUT)
        {
            motion_service_enter_fault(result);
        }

        goto finish;
    }

    /* 已经真正取出一帧，之后必须完成对应记账。 */
    begin_tick = rt_tick_get();
    motion_trace_worker_before_frame(&g_motion_worker.core, &g_motion_worker.frame, (uint32_t)begin_tick);

    queue_wait_ticks = (uint32_t)(begin_tick - (rt_tick_t)g_motion_worker.frame.enqueue_tick);

    sample_age_ticks = (uint32_t)(begin_tick - (rt_tick_t)g_motion_worker.frame.imu.sample_tick);

    /*
     * 排队时间落入半区间以上，无法解释为正常短时等待。
     * UINT32_MAX在此表示异常测量，不能解释为实测等待时间。
     */
    if (queue_wait_ticks >= UINT32_C(0x80000000))
    {
        g_motion_worker.max_queue_wait_ticks = UINT32_MAX;

        result = -RT_ERROR;
        goto internal_failure;
    }

    if (queue_wait_ticks > g_motion_worker.max_queue_wait_ticks)
    {
        g_motion_worker.max_queue_wait_ticks = queue_wait_ticks;
    }

    /*
     * 样本时刻异常交给核心分类。
     * 核心已有半区间年龄检查，不能在这里把它伪装成新鲜样本。
     */
    if (sample_age_ticks >= UINT32_C(0x80000000))
    {
        g_motion_worker.max_sample_age_ticks = UINT32_MAX;
    }
    else if (sample_age_ticks > g_motion_worker.max_sample_age_ticks)
    {
        g_motion_worker.max_sample_age_ticks = sample_age_ticks;
    }

    if (motion_service_test_drop_frame())
    {
        trace_injected = true;
        result = RT_EOK;
        goto frame_done; // 已由discard_injected记账，不能再调用核心。
    }

    /*
     * 紧邻核心调用重新取得now。
     * 不使用样本tick或队列enqueue_tick作为当前时间。
     */
    begin_tick = rt_tick_get();

    trace_core_called = true;
    core_result = motion_core_process(&g_motion_worker.core,
                                    &g_motion_worker.frame,
                                    (uint32_t)begin_tick,
                                    &g_motion_worker.frame_result);

    end_tick = rt_tick_get();

    process_ticks = (uint32_t)(end_tick - begin_tick);

    if (process_ticks > g_motion_worker.max_algorithm_ticks)
    {
        g_motion_worker.max_algorithm_ticks = process_ticks;
    }

    if (core_result != MOTION_CORE_OK)
    {
        /*
         * 当前核心约定：负值返回发生在消费分类记账之前。
         * 消息已经离开MQ，由服务层登记内部丢弃。
         *
         * 核心错误码与RT错误码不是同一套枚举，
         * 这里统一映射为-RT_ERROR，不直接返回core_result。
         */
        result = -RT_ERROR;
        goto internal_failure;
    }

    /*
     * 返回0说明核心已经登记dequeued和唯一分类。
     * 即使是stale、duplicate或旧代际丢弃，也不能再加一次。
     */
    if (g_motion_worker.frame_result.fatal_error)
    {
        /*
         * 该帧已经被核心分类，不再计discard_internal。
         * 立即关闭接纳门，防止继续接收正常业务输入。
         */
        result = -RT_ERROR;
        motion_service_enter_fault(result);
        goto frame_done;
    }

    /*
     * 有效进展要求两个算法都实际接受本帧。
     * 不要求产生事件：静止期间也应该持续有有效进展。
     */
    effective = (g_motion_worker.frame_result.disposition == MOTION_FRAME_PASSED_TO_ALGORITHM) &&
                (g_motion_worker.core.snapshot.state == MOTION_SERVICE_RUNNING);

    for (index = 0U; index < MOTION_ALGORITHM_COUNT; index++)
    {
        if (!g_motion_worker.frame_result.algorithm_called[index])
        {
            effective = false;
            break;
        }

        if ((g_motion_worker.frame_result.algorithm_result[index] != STEP_DETECTOR_PROCESS_ACCEPTED) &&
            (g_motion_worker.frame_result.algorithm_result[index] != STEP_DETECTOR_PROCESS_EVENT))
        {
            effective = false;
            break;
        }

        if (!g_motion_worker.frame_result
                 .algorithm_output[index].sample_accepted)
        {
            effective = false;
            break;
        }
    }

    if (effective)
    {
        g_motion_worker.effective_progress_tick = (uint32_t)end_tick;
    }

    result = RT_EOK;
    goto frame_done;

internal_failure:
    /*
     * 仅处理“已经出队，但核心尚未完成分类”的情况。
     * worker私有累计，无需为此获取publish_mutex。
     */
    motion_service_add_u64( &g_motion_worker.discard_internal,
                            UINT64_C(1),
                            &g_motion_worker.counters_saturated);

    motion_service_enter_fault(result);

frame_done:
    motion_trace_worker_after_frame(&g_motion_worker.core, &g_motion_worker.frame,
        trace_core_called, core_result,
        (trace_core_called && core_result == MOTION_CORE_OK) ? &g_motion_worker.frame_result : RT_NULL,
        trace_injected, (uint32_t)rt_tick_get());
    /*
     * 完成一次出队帧的处理或故障处置。
     * 这个进展不能单独证明业务健康。
     */
    g_motion_worker.worker_progress_tick = (uint32_t)rt_tick_get();

finish:
    /* 所有已设置在途标志的路径统一在这里清除。 */
    level = rt_hw_interrupt_disable();
    g_motion_shared.receive_inflight = 0U;
    rt_hw_interrupt_enable(level);

    return result;
}

/*
 * 简介：构造并发布同一轮核心摘要和消费统计。
 * 输入：无。
 * 返回值：发布成功RT_EOK；有界取锁失败返回负值。
 * 说明：先在锁外构造，锁内只复制；不从其他线程读取活的core。
 */
static rt_err_t motion_service_publish(void)
{
    motion_snapshot_t *snapshot = &g_motion_worker.snapshot_work;
    motion_service_stats_t *stats = &g_motion_worker.stats_work;
    motion_core_stats_t *core_stats = &g_motion_worker.core_stats_work;

    int core_result;
    rt_err_t result;
    rt_tick_t now;
    uint32_t index;
    uint64_t revision;
    rt_base_t level;

    /*
     * 使用同一个tick计算样本年龄和标记副本时间。
     * 读者从这个基准继续计算年龄，取锁等待也会计入。
     */
    now = rt_tick_get();
    core_result = motion_core_get_snapshot(&g_motion_worker.core, now, snapshot);

    if (core_result != MOTION_CORE_OK)
    {
        /* 核心错误使用RT错误码返回，避免与取锁超时混淆。 */
        result = -RT_ERROR;
        goto publish_failed;
    }

    core_result = motion_core_get_stats(&g_motion_worker.core, core_stats);

    if (core_result != MOTION_CORE_OK)
    {
        result = -RT_ERROR;
        goto publish_failed;
    }

    /*
     * 从空统计副本构造消费侧字段。
     * accounting_stable初始为false，由get_stats合并后判断。
     */
    memset(stats, 0, sizeof(*stats));

    stats->captured_tick = (uint32_t)now;
    stats->counters_saturated = core_stats->counters_saturated || g_motion_worker.counters_saturated;

    /*
     * 内部错误帧已从队列取出，但核心没完成分类。
     * 对外出队总数要计入这部分，才能覆盖全部已消费帧。
     */
    stats->dequeued = core_stats->dequeued;
    stats->discard_internal = g_motion_worker.discard_internal;
    stats->discard_injected = g_motion_worker.test.status.dropped_frames;

    motion_service_add_u64(&stats->dequeued,
                           stats->discard_internal,
                           &stats->counters_saturated);

    motion_service_add_u64(&stats->dequeued, stats->discard_injected, &stats->counters_saturated);
    stats->counters_saturated = stats->counters_saturated || g_motion_worker.test.status.counters_saturated;
    stats->discard_old_generation = core_stats->discard_old_generation;
    stats->discard_stopped = core_stats->discard_stopped;
    stats->discard_invalid = core_stats->discard_invalid;
    stats->discard_stale = core_stats->discard_stale;
    stats->discard_config = core_stats->discard_config;
    stats->discard_duplicate = core_stats->discard_duplicate;
    stats->passed_to_alg = core_stats->passed_to_alg;

    stats->seq_gap_events = core_stats->seq_gap_events;
    stats->seq_missing_samples = core_stats->seq_missing_samples;
    stats->seq_rollback_events = core_stats->seq_rollback_events;
    stats->time_gap_events = core_stats->time_gap_events;
    stats->source_offline_events = core_stats->source_offline_events;
    stats->source_recovery_events = core_stats->source_recovery_events;
    stats->calendar_change_events = core_stats->calendar_change_events;
    stats->day_bucket_evictions = core_stats->day_bucket_evictions;

    for (index = 0U; index < MOTION_ALGORITHM_COUNT; index++)
    {
        /* 两侧数组元素类型相同，逐项按值复制。 */
        stats->algorithm[index] = core_stats->algorithm[index];
    }

    stats->publish_failures = g_motion_worker.publish_failures;
    stats->sensor_status_read_failures = g_motion_worker.sensor_status_read_failures;

    stats->max_queue_wait_ticks = g_motion_worker.max_queue_wait_ticks;
    stats->max_sample_age_ticks = g_motion_worker.max_sample_age_ticks;
    stats->max_algorithm_ticks = g_motion_worker.max_algorithm_ticks;

    stats->worker_progress_tick = g_motion_worker.worker_progress_tick;
    stats->effective_progress_tick = g_motion_worker.effective_progress_tick;

    snapshot->published_tick = (uint32_t)now;
    snapshot->worker_progress_tick = stats->worker_progress_tick;

    /* 共享版本是64位，只在短临界区内取这个标量。 */
    level = rt_hw_interrupt_disable();
    revision = g_motion_shared.worker_revision;
    rt_hw_interrupt_enable(level);

    if (g_motion_publish_mutex == RT_NULL)
    {
        result = -RT_ERROR;
        goto publish_failed;
    }

    result = rt_mutex_take(g_motion_publish_mutex,(rt_int32_t)motion_service_ms_to_ticks(MOTION_COPY_TIMEOUT_MS));

    if (result != RT_EOK)
    {
        /* 取锁失败时，旧发布区仍可供读者读取。 */
        goto publish_failed;
    }

    /*
     * 发布号按已提交的副本递增，允许uint32_t自然回绕。
     * 读者使用同一把锁，所以只会取得成对的快照和统计。
     */
    snapshot->publish_seq =g_motion_published.snapshot.publish_seq + UINT32_C(1);

    g_motion_published.snapshot = *snapshot;
    g_motion_published.stats = *stats;
    g_motion_published.test = g_motion_worker.test.status;
    g_motion_published.test.published_tick = (uint32_t)now;
    g_motion_published.worker_revision = revision;
    g_motion_published.valid = true;

    result = rt_mutex_release(g_motion_publish_mutex);

    if (result != RT_EOK)
    {
        /* 两份副本已经写入，解锁错误由调用方按同步故障处理。 */
        goto publish_failed;
    }

    return RT_EOK;

publish_failed:
    motion_service_add_u64(&g_motion_worker.publish_failures,
                           UINT64_C(1),
                           &g_motion_worker.counters_saturated);
    return result;
}

/*
 * 简介：提交一条控制请求的成功或失败终态。
 * 输入：request_id非0；error为RT_EOK或实际负错误。
 * 返回值：无。
 * 说明：仅worker调用；短临界区更新单槽及控制完成累计。
 */
static void motion_service_finish_request(uint32_t request_id,rt_err_t error)
{
    motion_request_result_t *slot;
    rt_base_t level;
    uint32_t index;
    uint32_t finished_tick;
    bool already_faulted;

    finished_tick = (uint32_t)rt_tick_get();
    level = rt_hw_interrupt_disable();

    for (index = 0U; index < MOTION_RESULT_SLOT_COUNT; index++)
    {
        slot = &g_motion_shared.results[index];

        /* ID为0表示空槽，只处理与本次请求编号匹配的记录。 */
        if ((slot->request_id == 0U) ||(slot->request_id != request_id))
        {
            continue;
        }

        /* 重复完成时保留第一次的结果、时间和累计计数。 */
        if ((slot->state == MOTION_RESULT_OK) ||
            (slot->state == MOTION_RESULT_FAILED))
        {
            rt_hw_interrupt_enable(level);
            return;
        }

        /* 只有待处理或正在执行的请求可以提交终态。 */
        if ((slot->state != MOTION_RESULT_PENDING) &&
            (slot->state != MOTION_RESULT_RUNNING))
        {
            break;
        }

        /*
         * 记录请求结束时的会话和代际。
         * 例如START失败时，这里记录的仍是当时核心中的会话。
         */
        slot->error = (int32_t)error;
        slot->session_id = g_motion_worker.core.snapshot.session_id;
        slot->run_generation = g_motion_worker.core.snapshot.run_generation;
        slot->finished_tick = finished_tick;

        /* 结果字段填齐后提交终态，成功和失败都算完成一次。 */
        slot->state = (error == RT_EOK) ? MOTION_RESULT_OK : MOTION_RESULT_FAILED;

        motion_service_add_u64(&g_motion_shared.control_completed,UINT64_C(1),
                               &g_motion_shared.counters_saturated);

        rt_hw_interrupt_enable(level);
        return;
    }

    /*
     * 找不到对应槽，或匹配槽状态非法，属于内部记账错误。
     * 已在故障处理中时保留原故障，避免再次进入故障处理。
     */
    already_faulted = g_motion_shared.faulted;
    rt_hw_interrupt_enable(level);

    if (!already_faulted)
    {
        motion_service_enter_fault(-RT_ERROR);
    }
}

/*
 * 简介：让服务进入停止接纳的受控故障状态。
 * 输入：error为实际负错误码。
 * 返回值：无。
 * 说明：仅worker调用；保留队列和统计用于诊断，不运行期删除资源。
 */
static void motion_service_enter_fault(rt_err_t error)
{
    rt_base_t level;
    bool already_faulted;
    uint32_t stop_request_id;

    if (error >= RT_EOK)
    {
        error = -RT_ERROR;
    }

    /*
     * 先关门，防止sensor开始新的样本投递。
     * 已经开始的在途投递仍按原流程完成记账。
     */
    level = rt_hw_interrupt_disable();

    already_faulted = g_motion_shared.faulted;
    g_motion_shared.accepting_samples = false;
    g_motion_shared.faulted = true;

    rt_hw_interrupt_enable(level);

    motion_trace_worker_abort(&g_motion_worker.core, MOTION_TRACE_END_MOTION_FAULT, error, (uint32_t)rt_tick_get());
    motion_service_test_end(MOTION_TEST_END_FAULT);

    /* 重复进入时保留首次故障原因。 */
    if (already_faulted)
    {
        return;
    }

    /*
     * 核心由worker独占，可以在临界区外更新。
     * 后续出队帧仍可完成分类，但FAULT状态下不会进入计步算法。
     */
    g_motion_worker.core.snapshot.state = MOTION_SERVICE_FAULT;
    g_motion_worker.core.snapshot.last_error = (int32_t)error;

    /*
     * pending_stop_id对应已经出队、正在等待完成的STOP。
     * 先撤销握手状态，避免poll_stop再次处理同一请求。
     */
    stop_request_id = g_motion_worker.pending_stop_id;
    g_motion_worker.pending_stop_id = 0U;

    if (stop_request_id != 0U)
    {
        motion_service_finish_request(stop_request_id, error);
    }

    /*
     * 已入队的其他控制请求由handle_control逐条给出失败终态。
     * 本轮或后续轮次统一通过publish发布FAULT。
     */
}

/*
 * 简介：非阻塞提交一次有限诊断配置。
 * 输入：request_id非RT_NULL；fault/argument符合公开上限，失败输出ID为0。
 * 返回值：RT_EOK仅表示受理，其他为参数或队列错误。
 * 说明：仍使用统一控制队列，调用线程不访问worker私有状态。
 */
rt_err_t motion_service_request_test(motion_test_fault_t fault, uint32_t argument, uint32_t *request_id)
{
    return motion_service_post_request(MOTION_REQUEST_TEST, fault, argument, request_id);
}

/*
 * 简介：复制最近发布的有限注入状态。
 * 输入：status非RT_NULL，调用者持有缓冲区，不保留其地址。
 * 返回值：复制成功RT_EOK，参数错误-EINVAL，不可用-ERROR，繁忙-EBUSY。
 * 说明：发布锁内只复制值；不读取活的worker或core。
 */
rt_err_t motion_service_get_test_status(motion_test_status_t *status)
{
    motion_test_status_t value;
    rt_mutex_t mutex;
    rt_base_t level;
    rt_err_t result;
    bool valid;
    if (status == RT_NULL) return -RT_EINVAL;
    level = rt_hw_interrupt_disable();
    mutex = g_motion_shared.ready ? g_motion_publish_mutex : RT_NULL;
    rt_hw_interrupt_enable(level);
    if (mutex == RT_NULL) return -RT_ERROR;
    result = rt_mutex_take(mutex, (rt_int32_t)motion_service_ms_to_ticks(MOTION_COPY_TIMEOUT_MS));
    if (result != RT_EOK) return -RT_EBUSY;
    valid = g_motion_published.valid;
    if (valid) value = g_motion_published.test;
    result = rt_mutex_release(mutex);
    if (result != RT_EOK) return -RT_ERROR;
    if (!valid) return -RT_EBUSY;
    *status = value;
    return RT_EOK;
}

/*
 * 简介：验证注入类型和参数的完整组合。
 * 输入：fault/argument为值参数。
 * 返回值：合法true，否则false。
 * 说明：入口和worker都校验，不允许无限次数或未定义类型。
 */
static bool motion_service_test_args_valid(motion_test_fault_t fault, uint32_t argument)
{
    switch (fault)
    {
    case MOTION_TEST_NONE: return argument == 0U;
    case MOTION_TEST_PAUSE_CONSUMER: return argument >= 1U && argument <= MOTION_TEST_MAX_PAUSE_MS;
    case MOTION_TEST_DROP_FRAMES: return argument >= 1U && argument <= MOTION_TEST_MAX_DROP_FRAMES;
    case MOTION_TEST_FINAL_PUBLISH: return argument == 1U;
    default: return false;
    }
}

/*
 * 简介：在worker中安装一个有界注入或撤销当前注入。
 * 输入：message非RT_NULL，仅本次读取。
 * 返回值：配置成功RT_EOK，非法-EINVAL，状态不允许-EBUSY。
 * 说明：不覆盖活动注入；clear保留累计与witness；不改变正式算法参数。
 */
static rt_err_t motion_service_test_apply(const motion_control_message_t *message)
{
    motion_test_status_t *s = &g_motion_worker.test.status;
    rt_tick_t now;
    if (message == RT_NULL || !motion_service_test_args_valid(message->fault, message->argument)) return -RT_EINVAL;
    if (message->fault == MOTION_TEST_NONE)
    {
        motion_service_test_end(MOTION_TEST_END_CANCELLED);
        return RT_EOK;
    }
    if (motion_trace_worker_is_active() || s->active || g_motion_worker.pending_stop_id != 0U ||
        g_motion_worker.core.snapshot.state != MOTION_SERVICE_RUNNING ||
        g_motion_worker.core.source_state != MOTION_SOURCE_AVAILABLE ||
        !g_motion_worker.core.snapshot.has_processed || g_motion_worker.core.input_timed_out ||
        (g_motion_worker.core.snapshot.quality_flags & (MOTION_QUALITY_WARMUP | MOTION_QUALITY_CONFIG_MISMATCH)) != 0U)
        return -RT_EBUSY;
    now = rt_tick_get();
    s->kind = message->fault;
    s->active = true;
    s->end_reason = MOTION_TEST_END_NONE;
    s->request_id = message->request_id;
    s->argument = message->argument;
    s->remaining = message->argument;
    s->started_tick = (uint32_t)now;
    s->deadline_tick = (uint32_t)(now + motion_service_ms_to_ticks(
        message->fault == MOTION_TEST_PAUSE_CONSUMER ? message->argument : MOTION_TEST_ARM_TTL_MS));
    s->witness_valid = false;
    s->witness_read_error = 0;
    s->witness_accounting_stable = false;
    s->witness_other_conditions = false;
    s->witness_same_revision = false;
    s->witness_dequeued_delta = 0U;
    g_motion_worker.test.middle_publish_ok = false;
    g_motion_worker.test.capture_pending = false;
    if (message->fault == MOTION_TEST_PAUSE_CONSUMER)
        motion_service_add_u64(&s->pause_activations, 1U, &s->counters_saturated);
    return RT_EOK;
}

/*
 * 简介：结束当前注入并保留证据。
 * 输入：reason为有限注入结束原因。
 * 返回值：无。
 * 说明：仅worker调用；重复撤销不增加累计，正常停止和FAULT也撤销注入。
 */
static void motion_service_test_end(motion_test_end_t reason)
{
    motion_test_status_t *s = &g_motion_worker.test.status;
    if (!s->active) return;
    s->active = false;
    s->remaining = 0U;
    s->end_reason = reason;
    if (reason == MOTION_TEST_END_EXPIRED)
        motion_service_add_u64(&s->expirations, 1U, &s->counters_saturated);
    if (reason == MOTION_TEST_END_CANCELLED || reason == MOTION_TEST_END_FAULT)
        motion_service_add_u64(&s->cancellations, 1U, &s->counters_saturated);
    g_motion_worker.test.middle_publish_ok = false;
}

/*
 * 简介：即使没有样本也推进注入截止条件。
 * 输入：无。
 * 返回值：无。
 * 说明：使用真实单调tick、半区间内到期比较，不阻塞等待。
 */
static void motion_service_test_poll(void)
{
    motion_test_status_t *s = &g_motion_worker.test.status;
    uint32_t now, left;
    if (!s->active) return;
    now = (uint32_t)rt_tick_get();
    if ((uint32_t)(now - s->deadline_tick) < UINT32_C(0x80000000))
    {
        motion_service_test_end(s->kind == MOTION_TEST_PAUSE_CONSUMER ?
                                MOTION_TEST_END_COMPLETED : MOTION_TEST_END_EXPIRED);
    }
    else if (s->kind == MOTION_TEST_PAUSE_CONSUMER)
    {
        left = s->deadline_tick - now;
        s->remaining = (uint32_t)(((uint64_t)left * 1000U + RT_TICK_PER_SECOND - 1U) / RT_TICK_PER_SECOND);
    }
}

/*
 * 简介：判断是否只暂停样本消费批次。
 * 输入：无。
 * 返回值：仍在有限暂停期限内true，否则false。
 * 说明：worker继续控制、状态、tick和发布，sensor不等待、不停采。
 */
static bool motion_service_test_pause_active(void)
{
    motion_service_test_poll();
    return g_motion_worker.test.status.active &&
           g_motion_worker.test.status.kind == MOTION_TEST_PAUSE_CONSUMER;
}

/*
 * 简介：为FINAL试验建立同轮中途发布和消费基线。
 * 输入：无。
 * 返回值：真实中途发布结果，非活动FINAL时返回RT_EOK。
 * 说明：中途发布失败就不注入本轮末次失败，最迟2秒自动撤销。
 */
static rt_err_t motion_service_test_prepare_round(void)
{
    rt_err_t result;
    rt_base_t level;
    uint64_t count;
    bool saturated = false;
    g_motion_worker.test.middle_publish_ok = false;
    if (!g_motion_worker.test.status.active || g_motion_worker.test.status.kind != MOTION_TEST_FINAL_PUBLISH) return RT_EOK;
    count = g_motion_worker.core.stats.dequeued;
    motion_service_add_u64(&count, g_motion_worker.discard_internal, &saturated);
    motion_service_add_u64(&count, g_motion_worker.test.status.dropped_frames, &saturated);
    if (saturated) return -RT_ERROR;
    result = motion_service_publish();
    if (result == RT_EOK)
    {
        g_motion_worker.test.dequeued_before_batch = count;
        level = rt_hw_interrupt_disable();
        g_motion_worker.test.middle_revision = g_motion_shared.worker_revision;
        rt_hw_interrupt_enable(level);
        g_motion_worker.test.middle_publish_ok = true;
    }
    return result;
}

/*
 * 简介：消耗一次已出队帧的有限丢弃配额。
 * 输入：无，仅在已经成功取出一帧后调用。
 * 返回值：本帧由诊断丢弃true，否则false。
 * 说明：不调用核心，不记为sensor错误或队列拒绝；累计永不因clear清零。
 */
static bool motion_service_test_drop_frame(void)
{
    motion_test_status_t *s = &g_motion_worker.test.status;
    motion_service_test_poll();
    if (!s->active || s->kind != MOTION_TEST_DROP_FRAMES || s->remaining == 0U) return false;
    s->remaining--;
    motion_service_add_u64(&s->dropped_frames, 1U, &s->counters_saturated);
    if (s->remaining == 0U) motion_service_test_end(MOTION_TEST_END_COMPLETED);
    return true;
}

/*
 * 简介：在真实消费改变中途副本后，仅模拟一次末次发布失败。
 * 输入：无。
 * 返回值：本轮应跳过末次发布true，否则false。
 * 说明：不伪造真实mutex失败；实际触发数和消费差值都保留。
 */
static bool motion_service_test_fail_final(void)
{
    motion_test_status_t *s = &g_motion_worker.test.status;
    uint64_t count, revision;
    rt_base_t level;
    bool saturated = false;
    motion_service_test_poll();
    if (!s->active || s->kind != MOTION_TEST_FINAL_PUBLISH || !g_motion_worker.test.middle_publish_ok) return false;
    level = rt_hw_interrupt_disable();
    revision = g_motion_shared.worker_revision;
    /* 避开控制发送尚未记账的窗口，防止其他条件造成不稳定而掩盖回归。 */
    if (g_motion_shared.control_inflight != 0U || g_motion_shared.ingress.inflight != 0U ||
        g_motion_shared.receive_inflight != 0U)
    {
        rt_hw_interrupt_enable(level);
        return false;
    }
    rt_hw_interrupt_enable(level);
    if (revision != g_motion_worker.test.middle_revision) return false;
    count = g_motion_worker.core.stats.dequeued;
    motion_service_add_u64(&count, g_motion_worker.discard_internal, &saturated);
    motion_service_add_u64(&count, s->dropped_frames, &saturated);
    if (saturated || count <= g_motion_worker.test.dequeued_before_batch) return false;
    s->witness_dequeued_delta = count - g_motion_worker.test.dequeued_before_batch;
    motion_service_add_u64(&s->final_publish_failures, 1U, &s->counters_saturated);
    /* 被跳过的publish不会进入原失败出口，因此在此登记一次。 */
    motion_service_add_u64(&g_motion_worker.publish_failures, 1U, &g_motion_worker.counters_saturated);
    motion_service_test_end(MOTION_TEST_END_COMPLETED);
    g_motion_worker.test.capture_pending = true;
    return true;
}

/*
 * 简介：在失败窗口自动调用真实getter留下证据。
 * 输入：无。
 * 返回值：无。
 * 说明：仅在本轮busy清除后调用；不修改核心/消费统计，结果下一轮发布。
 *       不把预期false直接赋给观测值，锁失败也如实记录。
 */
static void motion_service_test_capture(void)
{
    motion_test_status_t *s = &g_motion_worker.test.status;
    rt_err_t result;
    rt_base_t level;
    uint64_t revision, published_revision;
    if (!g_motion_worker.test.capture_pending) return;
    g_motion_worker.test.capture_pending = false;
    result = motion_service_get_stats(&g_motion_worker.test.witness_work);
    s->witness_read_error = (int32_t)result;
    if (result != RT_EOK) return;
    s->witness_accounting_stable = g_motion_worker.test.witness_work.accounting_stable;
    s->witness_other_conditions = g_motion_worker.test.witness_work.accounting_base_ready;
    result = rt_mutex_take(g_motion_publish_mutex, (rt_int32_t)motion_service_ms_to_ticks(MOTION_COPY_TIMEOUT_MS));
    if (result != RT_EOK) { s->witness_read_error = (int32_t)result; return; }
    published_revision = g_motion_published.worker_revision;
    result = rt_mutex_release(g_motion_publish_mutex);
    if (result != RT_EOK) { s->witness_read_error = (int32_t)result; return; }
    level = rt_hw_interrupt_disable();
    revision = g_motion_shared.worker_revision;
    rt_hw_interrupt_enable(level);
    s->witness_same_revision = revision == published_revision;
    s->witness_valid = true;
}


/*
 * 简介：验证控制消息的完整参数组合。
 * 输入：kind/fault/argument为消息值，不保存外部地址。
 * 返回值：合法true，未定义请求或超限false。
 * 说明：保留START/STOP/TEST约束，TRACE复用队列但独立解释argument。
 */
static bool motion_service_control_args_valid(motion_request_kind_t kind, motion_test_fault_t fault, uint32_t argument)
{
    switch (kind)
    {
    case MOTION_REQUEST_START:
    case MOTION_REQUEST_STOP:
        return fault == MOTION_TEST_NONE && argument == 0U;
    case MOTION_REQUEST_TEST:
        return motion_service_test_args_valid(fault, argument);
    case MOTION_REQUEST_TRACE_ARM:
        return fault == MOTION_TEST_NONE && (argument & UINT32_C(0xFFFF)) >= 1U &&
            (argument & UINT32_C(0xFFFF)) <= MOTION_TRACE_MAX_SECONDS &&
            (argument >> 16) <= MOTION_TRACE_MAX_DELAY_SECONDS;
    case MOTION_REQUEST_TRACE_CANCEL:
    case MOTION_REQUEST_TRACE_CLEAR:
        return fault == MOTION_TEST_NONE && argument != 0U;
    default:
        return false;
    }
}

/*
 * 简介：非阻塞提交RAM诊断采集请求。
 * 输入：duration_s为1～30，delay_s为0～60；request_id非空，失败输出0。
 * 返回值：RT_EOK仅表示受理；参数/资源错误返回实际负值；裁剪模式-ENOSYS。
 * 说明：通过原控制队列及结果槽执行，不改业务会话、算法参数或采样配置。
 */
rt_err_t motion_service_request_trace_arm(uint32_t duration_s, uint32_t delay_s, uint32_t *request_id)
{
    if (request_id == RT_NULL) return -RT_EINVAL;
    *request_id = 0U;
#if MOTION_TRACE_ENABLED
    if (duration_s < 1U || duration_s > MOTION_TRACE_MAX_SECONDS || delay_s > MOTION_TRACE_MAX_DELAY_SECONDS) return -RT_EINVAL;
    return motion_service_post_request(MOTION_REQUEST_TRACE_ARM, MOTION_TEST_NONE,
        (delay_s << 16) | duration_s, request_id);
#else
    (void)duration_s; (void)delay_s; return -RT_ENOSYS;
#endif
}

/*
 * 简介：提交取消指定采集的请求。
 * 输入：capture_id非0；request_id非空，失败输出0。
 * 返回值：受理RT_EOK，其他为实际负错误或裁剪模式-ENOSYS。
 * 说明：worker冻结已采数据，运动业务继续；须查询异步执行结果。
 */
rt_err_t motion_service_request_trace_cancel(uint32_t capture_id, uint32_t *request_id)
{
    if (request_id == RT_NULL) return -RT_EINVAL;
    *request_id = 0U;
#if MOTION_TRACE_ENABLED
    if (capture_id == 0U) return -RT_EINVAL;
    return motion_service_post_request(MOTION_REQUEST_TRACE_CANCEL, MOTION_TEST_NONE, capture_id, request_id);
#else
    (void)capture_id; return -RT_ENOSYS;
#endif
}

/*
 * 简介：提交清理指定冻结采集的请求。
 * 输入：capture_id非0；request_id非空，失败输出0。
 * 返回值：受理RT_EOK，其他为实际负错误或裁剪模式-ENOSYS。
 * 说明：租约期间拒绝清理；不清正式算法或业务计数，不擦整个记录数组。
 */
rt_err_t motion_service_request_trace_clear(uint32_t capture_id, uint32_t *request_id)
{
    if (request_id == RT_NULL) return -RT_EINVAL;
    *request_id = 0U;
#if MOTION_TRACE_ENABLED
    if (capture_id == 0U) return -RT_EINVAL;
    return motion_service_post_request(MOTION_REQUEST_TRACE_CLEAR, MOTION_TEST_NONE, capture_id, request_id);
#else
    (void)capture_id; return -RT_ENOSYS;
#endif
}
