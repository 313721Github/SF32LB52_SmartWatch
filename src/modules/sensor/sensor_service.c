#include "sensor_service.h"
#include "rthw.h"
#include "sensor_bus_port.h"
#include "lsm6ds3tr_c.h"
#include "mmc5603nj.h"
#include "ltr303als01.h"

#include <string.h>
/*
 * sensor服务内部配置
 *
 * 仅供sensor_service.c使用，用于控制采样调度、故障恢复、
 * 快照发布和线程资源。app与UI不得依赖这些参数。
 */

/* 目标采样周期，单位ms。 */
#define SENSOR_SERVICE_IMU_PERIOD_MS              20U  /* 约50 Hz，匹配IMU当前52 Hz ODR */
#define SENSOR_SERVICE_MMC_PERIOD_MS              50U  /* 20 Hz，周期到达时启动单次测量 */
#define SENSOR_SERVICE_ALS_PERIOD_MS             200U  /* 匹配ALS当前200 ms更新周期 */

/*
 * 非阻塞ready检查。
 * 到期后每次只检查一次STATUS，未就绪时释放sensor线程，
 * 达到超时上限后将本次采样记为失败。
 */
#define SENSOR_SERVICE_MMC_POLL_MS                 1U  /* MMC ready检查间隔 */
#define SENSOR_SERVICE_MMC_READY_TIMEOUT_MS       30U  /* 从测量触发开始计时 */

#define SENSOR_SERVICE_ALS_POLL_MS                10U  /* ALS NEW检查间隔 */
#define SENSOR_SERVICE_ALS_READY_TIMEOUT_MS       250U /* 从ALS到期检查开始计时 */

/*
 * 故障恢复策略。
 * 单次失败进入暂时失败；连续失败达到阈值后进入离线；
 * 离线期间只按固定周期重探，避免形成总线访问风暴。
 */
#define SENSOR_SERVICE_TEMP_RETRY_MS            1000U  /* 暂时失败后的重试间隔 */
#define SENSOR_SERVICE_OFFLINE_REPROBE_MS       5000U  /* 离线后的重探间隔 */
#define SENSOR_SERVICE_FAILURE_THRESHOLD           3U  /* 连续失败离线阈值 */

/*
 * 快照与通知策略。
 * 快照锁内只允许复制结构体，禁止执行I2C、日志、回调和LVGL操作。
 */
#define SENSOR_SERVICE_STALE_MS                  1000U /* last-good样本过期阈值 */
#define SENSOR_SERVICE_NOTIFY_MIN_MS             500U  /* 普通更新通知的最小间隔 */
#define SENSOR_SERVICE_LOCK_TIMEOUT_MS             5U  /* 快照mutex等待上限 */

/*
 * sensor线程资源。
 * priority遵循RT-Thread数值越小优先级越高的规则；
 * timeslice单位为系统tick，不是毫秒或采样周期。
 */
#define SENSOR_SERVICE_THREAD_STACK_SIZE        3072U  /* 字节 */
#define SENSOR_SERVICE_THREAD_PRIORITY            19U
#define SENSOR_SERVICE_THREAD_TIMESLICE            10U  /* 系统tick */

/*
 * event只负责唤醒sensor线程。
 * 请求的参数保存在受保护的标量变量中，不通过event传递指针。
 */
#define SENSOR_SERVICE_EVENT_PROBE       (1UL << 0)         //存在器件重探请求
#define SENSOR_SERVICE_EVENT_CONTROL     (1UL << 1)         //故障注入等控制配置发生变化
#define SENSOR_SERVICE_EVENT_ALL         (SENSOR_SERVICE_EVENT_PROBE | SENSOR_SERVICE_EVENT_CONTROL)


// sensor服务生命周期状态
typedef enum
{
    SENSOR_SERVICE_STATE_STOPPED = 0,
    SENSOR_SERVICE_STATE_INITIALIZING,
    SENSOR_SERVICE_STATE_RUNNING
} sensor_service_state_t;

//单个器件的软件故障注入配置
typedef struct
{
    sensor_test_fault_t fault;
    rt_uint32_t remaining_count;
} sensor_fault_injection_t;

/*
 * MMC和ALS需要跨多个调度轮次等待ready。
 * WAIT_READY期间sensor线程仍可处理其他器件。
 */
typedef enum
{
    SENSOR_WAIT_PHASE_IDLE = 0,     //空闲阶段，可以开始新的测量事务
    SENSOR_WAIT_PHASE_READY         //测量已触发，正在等待数据就绪
} sensor_wait_phase_t;

/*
 * 单颗器件的内部运行状态。
 * 这些字段只由sensor线程修改，不需要额外加锁。
 */
typedef struct
{
    sensor_device_state_t state;          //器件当前运行状态

    rt_bool_t configured;                 //身份与冻结配置是否验证完成
    rt_bool_t recovery_pending;           //是否正在等待重探或故障恢复
    rt_bool_t retry_count_pending;

    /*
    * TEMP_FAILED安排了一次尚未开始的受控重试时为RT_TRUE。
    * 它保证一次退避期限只统计一次retry，不会因调度循环反复进入而重复计数。
    */
    rt_uint32_t consecutive_failures;     //当前连续失败次数

    rt_tick_t period_ticks;               //正常采样周期，单位tick
    rt_tick_t next_due_tick;              //下一次正常采样到期时刻
    rt_tick_t retry_due_tick;             //离线或失败后的下一次重试时刻

    sensor_wait_phase_t wait_phase;       //当前非阻塞测量等待阶段
    rt_tick_t next_poll_tick;             //下一次ready状态检查时刻
    rt_tick_t ready_deadline_tick;        //本轮ready等待的最晚截止时刻
} sensor_device_runtime_t;
/**
 * @brief sensor统一服务上下文
 *
 * 本结构拥有服务线程、同步资源、最新快照、累计统计、
 * 通知配置和异步控制请求。
 *
 * 同一个结构体中的字段仍属于不同同步域：
 *
 * 1. state、资源句柄、pending_probe_mask、faults和两个独立计数
 *    使用短临界区保护；
 * 2. snapshot和stats使用snapshot_mutex保护；
 * 3. 后续加入的器件调度状态只允许sensor线程修改。
 */
typedef struct
{
    //服务生命周期和RT-Thread资源
    sensor_service_state_t state;

    rt_mutex_t snapshot_mutex;
    rt_event_t wake_event;
    rt_thread_t thread;


    //通知回调只允许在STOPPED状态注册
    //sensor线程以后只能在snapshot mutex之外调用它
    sensor_notify_handler_t notify_handler;
    void *notify_user_data;

    //重探请求使用位图合并，调用者只提交请求，真实I2C操作由sensor线程执行
    rt_uint32_t pending_probe_mask;

    // 每颗器件独立保存故障类型和剩余注入次数，静态零初始化时fault默认是SENSOR_TEST_FAULT_NONE
    sensor_fault_injection_t faults[SENSOR_DEVICE_COUNT];

    //这两个计数可能在无法取得snapshot_mutex时更新，因此使用短临界区独立保护。
    rt_uint32_t lock_timeout_count;
    rt_uint32_t probe_request_count;
    /*
     * 下面字段只由唯一sensor线程修改。
     * 其他线程只能通过公开快照和统计接口观察结果。
     */
    sensor_device_runtime_t runtime[SENSOR_DEVICE_COUNT];

    //通知使用位图合并,urgent用于状态失败或恢复时绕过普通500 ms限频
    rt_uint32_t notify_pending_mask;
    rt_tick_t last_notify_tick;
    rt_bool_t notify_urgent;
    //最新一致快照和累计统计，二者只能在snapshot_mutex保护下整体复制或更新
    sensor_snapshot_t snapshot;
    sensor_service_stats_t stats;

} sensor_service_context_t;

//定义唯一服务实例
//未显式初始化的字段全部为0：请求、统计和快照全部清零。
static sensor_service_context_t g_sensor_service =
{
    .state = SENSOR_SERVICE_STATE_STOPPED,
    .snapshot_mutex = RT_NULL,
    .wake_event = RT_NULL,
    .thread = RT_NULL,
    .notify_handler = RT_NULL,
    .notify_user_data = RT_NULL
};

//公开函数声明
rt_err_t sensor_service_register_notify(sensor_notify_handler_t handler,void *user_data);//注册服务通知回调

/**
 * @brief 创建资源并启动唯一sensor线程
 *
 * RT_EOK只表示服务线程已经成功启动，不代表三颗器件都在线。
 * 重复调用时不得重复创建线程、mutex或event。
 */
rt_err_t sensor_service_init(void);                                 //创建统一sensor服务资源并启动唯一sensor线程

rt_bool_t sensor_service_is_running(void);                          //查询统一服务是否已经运行

rt_err_t sensor_service_get_snapshot(sensor_snapshot_t *snapshot);  //把最新一致快照复制到调用者缓冲区

rt_err_t sensor_service_get_stats(sensor_service_stats_t *stats);   //复制统一服务累计统计

rt_err_t sensor_service_request_probe(rt_uint32_t device_mask);     //请求sensor线程重探指定器件

rt_err_t sensor_service_test_inject(sensor_device_id_t device,sensor_test_fault_t fault,rt_uint32_t count);//为指定器件配置固定次数的软件故障注入

//私有函数声明
static void sensor_service_thread_entry(void *parameter);

static sensor_service_state_t sensor_service_get_state(void);           //在短临界区内读取sensor服务当前生命周期状态

static rt_tick_t sensor_service_ms_to_tick(rt_uint32_t milliseconds);   //将毫秒时间转换为非零的RT-Thread系统tick

static void sensor_service_update_status_age(sensor_device_status_t *status,
                                                        rt_tick_t read_tick);//计算单个器件last-good样本的年龄和过期状态

static void sensor_service_record_lock_timeout(void);                       //记录一次snapshot mutex获取超时

static void sensor_service_read_external_counters(rt_uint32_t *lock_timeout_count,
                                                rt_uint32_t *probe_request_count);                  //一致读取不受snapshot mutex保护的两个外部计数器

static rt_bool_t sensor_service_tick_reached(rt_tick_t now,rt_tick_t deadline);                     //判断当前tick是否已经到达或越过指定期限

static rt_uint32_t sensor_service_device_mask(sensor_device_id_t device);                           //把器件编号转换为对应的器件位图

static sensor_device_status_t *sensor_service_get_device_status(sensor_device_id_t device);         //取得指定器件在内部快照中的状态对象

static sensor_device_stats_t *sensor_service_get_device_stats_internal(sensor_device_id_t device);  //取得指定器件的内部累计统计对象

static rt_err_t sensor_service_lock_snapshot(void);                                                 //在有限时间内获取snapshot mutex

static rt_err_t sensor_service_consume_fault(sensor_device_id_t device);                            //为一次高层器件事务消费至多一次软件故障注入

static void sensor_service_record_operation(sensor_device_id_t device,rt_tick_t operation_ticks);   //记录一次不改变器件成功或失败状态的高层操作

static void sensor_service_record_not_ready(sensor_device_id_t device,rt_tick_t operation_ticks);   //记录一次器件尚未产生新数据的操作结果

static void sensor_service_record_failure(sensor_device_id_t device,rt_err_t error,rt_tick_t operation_ticks);//记录一次器件操作失败并安排有限重试或离线重探

static rt_err_t sensor_service_commit_sample(sensor_device_id_t device,const void *sample,rt_tick_t operation_ticks);//提交一份完整有效样本并原子更新状态和统计

static void sensor_service_record_late(sensor_device_id_t device);                                  //记录一次器件调度晚于计划期限

static void sensor_service_advance_period(sensor_device_id_t device,rt_tick_t now);                 //按绝对周期推进器件的下一次采样期限

static rt_err_t sensor_service_initialize_device(sensor_device_id_t device,rt_bool_t force);        //初始化或重新初始化指定器件并建立首次采样期限

static rt_uint32_t sensor_service_take_probe_mask(void);                                            //原子取得并清空当前待处理的器件重探位图

static void sensor_service_process_probe_requests(void);                                            //处理当前合并的三器件受控重探请求

static rt_bool_t sensor_service_prepare_device(sensor_device_id_t device,rt_tick_t now);            //判断指定器件当前是否可以执行下一次采样操作

static void sensor_service_process_imu(rt_tick_t now);          //执行一次IMU调度、采样、提交或失败处理
static void sensor_service_process_mmc(rt_tick_t now);          //推进MMC5603NJ单次测量状态机并提交有效磁场样本
static void sensor_service_process_als(rt_tick_t now);          //推进LTR303连续测量的NEW等待窗口并提交有效ALS样本

static void sensor_service_notify_if_due(void);

static void sensor_service_record_loop(rt_tick_t loop_ticks);   //记录sensor线程最近一轮调度处理时间和历史最大处理时间

static void sensor_service_record_wake(void);                   //记录sensor线程从event等待中返回的累计次数

static rt_tick_t sensor_service_calculate_wait(rt_tick_t now);  //计算sensor线程下一次event等待允许持续的tick数
static void sensor_service_record_retry(sensor_device_id_t device);  //记录TEMP_FAILED退避到期后真正开始的一次受控重试
//---------------------------------------------------------------公开函数---------------------------------------------------------------
/*
 * 简介：注册sensor服务的快照或状态变化通知回调。
 * 输入：handler为通知回调，允许为RT_NULL以取消注册；
 *       user_data为回调上下文，取消注册时会被清除。
 * 返回值：注册成功返回RT_EOK；服务已开始初始化或运行时
 *         返回-RT_EBUSY。
 * 说明：只能在STOPPED状态调用；handler和user_data在短临界区内
 *       成对更新，避免sensor线程取得不匹配的回调和上下文。
 */
rt_err_t sensor_service_register_notify(sensor_notify_handler_t handler,void *user_data)
{
    rt_base_t level;

    level = rt_hw_interrupt_disable();

    if (g_sensor_service.state !=SENSOR_SERVICE_STATE_STOPPED)
    {
        rt_hw_interrupt_enable(level);
        return -RT_EBUSY;
    }

    g_sensor_service.notify_handler = handler;

    g_sensor_service.notify_user_data =(handler != RT_NULL)? user_data: RT_NULL;

    rt_hw_interrupt_enable(level);
    return RT_EOK;
}

/*
 * 简介：创建统一sensor服务资源并启动唯一sensor线程
 * 输入：无。
 * 返回值：线程成功启动或服务已运行返回RT_EOK；
 *         正在初始化返回-RT_EBUSY；
 *         资源创建失败返回-RT_ENOMEM；
 *         线程启动失败返回RT-Thread实际错误。
 * 说明：RT_EOK只表示服务线程已经启动，不表示三颗器件均已
 *       初始化或已有有效样本；失败时按thread、event、mutex
 *       的反向顺序清理。当前没有退出握手，因此不提供deinit。
 */
rt_err_t sensor_service_init(void)
{
    rt_mutex_t snapshot_mutex = RT_NULL;
    rt_event_t wake_event = RT_NULL;
    rt_thread_t thread = RT_NULL;

    rt_base_t level;
    rt_tick_t now;
    rt_err_t result;

    /*
     * 先在短临界区检查并占用生命周期状态，
     * 防止多个调用者同时创建两套服务资源。
     */
    level = rt_hw_interrupt_disable();

    if (g_sensor_service.state ==SENSOR_SERVICE_STATE_RUNNING)
    {
        rt_hw_interrupt_enable(level);
        return RT_EOK;
    }

    if (g_sensor_service.state ==SENSOR_SERVICE_STATE_INITIALIZING)
    {
        rt_hw_interrupt_enable(level);
        return -RT_EBUSY;
    }

    g_sensor_service.state =SENSOR_SERVICE_STATE_INITIALIZING;

    rt_hw_interrupt_enable(level);

    /*
     * 服务尚未运行，可以初始化快照、统计和调度状态。
     * 不能清零整个context，否则会丢失预先注册的通知回调。
     */
    memset(&g_sensor_service.snapshot,0,sizeof(g_sensor_service.snapshot));

    memset(&g_sensor_service.stats,0,sizeof(g_sensor_service.stats));

    memset(g_sensor_service.runtime,0,sizeof(g_sensor_service.runtime));

    /*
     * 故障注入和跨上下文计数可能由其他公开接口访问，
     * 因此在短临界区内恢复为默认状态。
     */
    level = rt_hw_interrupt_disable();

    memset(g_sensor_service.faults,0,sizeof(g_sensor_service.faults));

    g_sensor_service.pending_probe_mask = 0U;
    g_sensor_service.lock_timeout_count = 0U;
    g_sensor_service.probe_request_count = 0U;

    rt_hw_interrupt_enable(level);

    now = rt_tick_get();

    /*
     * 冻结三个器件的初始状态和调度周期。
     * 首次到期时间由sensor线程初始化调度时确定。
     */
    g_sensor_service.runtime[SENSOR_DEVICE_IMU].state =SENSOR_DEVICE_STATE_UNINITIALIZED;
    g_sensor_service.runtime[SENSOR_DEVICE_IMU].period_ticks =sensor_service_ms_to_tick(SENSOR_SERVICE_IMU_PERIOD_MS);

    g_sensor_service.runtime[SENSOR_DEVICE_MMC].state =SENSOR_DEVICE_STATE_UNINITIALIZED;
    g_sensor_service.runtime[SENSOR_DEVICE_MMC].period_ticks =sensor_service_ms_to_tick(SENSOR_SERVICE_MMC_PERIOD_MS);

    g_sensor_service.runtime[SENSOR_DEVICE_ALS].state =SENSOR_DEVICE_STATE_UNINITIALIZED;
    g_sensor_service.runtime[SENSOR_DEVICE_ALS].period_ticks =sensor_service_ms_to_tick(SENSOR_SERVICE_ALS_PERIOD_MS);

    /*
     * 服务启动时尚无有效样本。
     * stale=true和-RT_ENOSYS共同表示器件仍未完成初始化。
     */
    g_sensor_service.snapshot.imu_status.state =SENSOR_DEVICE_STATE_UNINITIALIZED;
    g_sensor_service.snapshot.imu_status.stale = RT_TRUE;
    g_sensor_service.snapshot.imu_status.last_error =-RT_ENOSYS;

    g_sensor_service.snapshot.mmc_status.state =SENSOR_DEVICE_STATE_UNINITIALIZED;
    g_sensor_service.snapshot.mmc_status.stale = RT_TRUE;
    g_sensor_service.snapshot.mmc_status.last_error =-RT_ENOSYS;

    g_sensor_service.snapshot.als_status.state =SENSOR_DEVICE_STATE_UNINITIALIZED;
    g_sensor_service.snapshot.als_status.stale = RT_TRUE;
    g_sensor_service.snapshot.als_status.last_error =-RT_ENOSYS;

    /* 清空合并通知状态，并建立通知限频的时间基准。 */
    g_sensor_service.notify_pending_mask = 0U;
    g_sensor_service.notify_urgent = RT_FALSE;
    g_sensor_service.last_notify_tick = now;

    /*
     * 按依赖顺序创建资源：
     * snapshot mutex → wake event → sensor线程。
     */
    snapshot_mutex =rt_mutex_create("sns_snap", RT_IPC_FLAG_PRIO);
    if (snapshot_mutex == RT_NULL)
    {
        result = -RT_ENOMEM;
        goto fail;
    }

    wake_event =rt_event_create("sns_evt", RT_IPC_FLAG_PRIO);
    if (wake_event == RT_NULL)
    {
        result = -RT_ENOMEM;
        goto fail;
    }

    thread = rt_thread_create("sensor",
                                sensor_service_thread_entry,
                                RT_NULL,
                                SENSOR_SERVICE_THREAD_STACK_SIZE,
                                SENSOR_SERVICE_THREAD_PRIORITY,
                                SENSOR_SERVICE_THREAD_TIMESLICE);
    if (thread == RT_NULL)
    {
        result = -RT_ENOMEM;
        goto fail;
    }

    /*
     * 在线程启动前发布完整资源集合。
     * 线程入口统一从context读取句柄，不依赖本函数局部变量。
     */
    level = rt_hw_interrupt_disable();

    g_sensor_service.snapshot_mutex = snapshot_mutex;
    g_sensor_service.wake_event = wake_event;
    g_sensor_service.thread = thread;

    rt_hw_interrupt_enable(level);

    /*
     * startup失败时资源句柄已经发布，
     * 必须先撤销context中的引用，再删除实际资源。
     */
    result = rt_thread_startup(thread);
    if (result != RT_EOK)
    {
        goto fail_published;
    }

    /* 所有资源和线程启动成功后，服务才正式进入RUNNING。 */
    level = rt_hw_interrupt_disable();

    g_sensor_service.state =SENSOR_SERVICE_STATE_RUNNING;

    rt_hw_interrupt_enable(level);

    return RT_EOK;

fail_published:
    /*
     * 先清除全局可见句柄，防止失败资源继续被公开接口使用。
     */
    level = rt_hw_interrupt_disable();

    g_sensor_service.thread = RT_NULL;
    g_sensor_service.wake_event = RT_NULL;
    g_sensor_service.snapshot_mutex = RT_NULL;

    rt_hw_interrupt_enable(level);

fail:
    /*
     * 按创建顺序的反方向释放资源。
     * RT_NULL检查使本路径兼容任意创建阶段的失败。
     */
    if (thread != RT_NULL)
    {
        (void)rt_thread_delete(thread);
    }

    if (wake_event != RT_NULL)
    {
        (void)rt_event_delete(wake_event);
    }

    if (snapshot_mutex != RT_NULL)
    {
        (void)rt_mutex_delete(snapshot_mutex);
    }

    /* 清理结束后允许调用者再次尝试初始化。 */
    level = rt_hw_interrupt_disable();

    g_sensor_service.state =SENSOR_SERVICE_STATE_STOPPED;

    rt_hw_interrupt_enable(level);

    return result;
}

/*
 * 简介：查询sensor统一服务是否已经进入运行状态。
 * 输入：无。
 * 返回值：服务处于RUNNING状态返回RT_TRUE；
 *         其他状态返回RT_FALSE。
 * 说明：通过统一的状态读取函数在短临界区内取得生命周期状态，
 *       不直接访问共享state。
 */
rt_bool_t sensor_service_is_running(void)
{
    return (sensor_service_get_state() ==SENSOR_SERVICE_STATE_RUNNING)
        ? RT_TRUE
        : RT_FALSE;
}

/*
 * 简介：复制sensor服务当前的最新一致快照。
 * 输入：snapshot为调用者持有的输出缓冲区，不允许为RT_NULL。
 * 返回值：复制成功返回RT_EOK；空指针返回-RT_EINVAL；
 *         服务未运行或mutex未创建返回-RT_ENOSYS；
 *         获取或释放mutex失败时返回对应RT-Thread错误。
 * 说明：mutex内只复制内部快照；age和stale在锁外计算；
 *       全部处理成功后才更新调用者缓冲区，失败时不修改原内容。
 */
rt_err_t sensor_service_get_snapshot(sensor_snapshot_t *snapshot)
{
    sensor_snapshot_t candidate;
    rt_tick_t read_tick;
    rt_tick_t lock_timeout_tick;
    rt_err_t result;

    /* 输出缓冲区必须由调用者提供，服务不返回内部快照指针。 */
    if (snapshot == RT_NULL)
    {
        return -RT_EINVAL;
    }

    /*
     * 只有服务已经运行且快照mutex有效时才允许读取。
     * INITIALIZING期间的内部数据不能作为有效快照对外发布。
     */
    if ((sensor_service_is_running() == RT_FALSE) ||(g_sensor_service.snapshot_mutex == RT_NULL))
    {
        return -RT_ENOSYS;
    }

    /*
     * 将毫秒换算为非零tick，保证取锁等待有明确上界，
     * 避免消费者无限阻塞。
     */
    lock_timeout_tick =sensor_service_ms_to_tick(SENSOR_SERVICE_LOCK_TIMEOUT_MS);

    result = rt_mutex_take(g_sensor_service.snapshot_mutex,lock_timeout_tick);
    if (result != RT_EOK)
    {
        /*
         * 获取失败时不读取内部快照，也不修改调用者缓冲区。
         * 锁失败计数使用独立临界区记录，不能再次依赖该mutex。
         */
        sensor_service_record_lock_timeout();
        return result;
    }

    /*
     * 锁内只执行固定长度结构体复制。
     * 禁止在锁内执行I2C、日志、回调、延时或LVGL操作。
     */
    candidate = g_sensor_service.snapshot;

    result = rt_mutex_release(g_sensor_service.snapshot_mutex);
    if (result != RT_EOK)
    {
        /*
         * mutex释放异常时不向调用者发布candidate，
         * 防止把同步状态异常下取得的数据声明为成功结果。
         */
        return result;
    }

    /*
     * snapshot_tick表示调用者取得这份副本的时刻，
     * 不等同于任一传感器的sample_tick。
     */
    read_tick = rt_tick_get();
    candidate.snapshot_tick = read_tick;

    /*
     * 在锁外根据同一个read_tick计算三颗器件的数据年龄，
     * 保证三者使用相同的时间基准，同时缩短持锁时间。
     */
    sensor_service_update_status_age(&candidate.imu_status,read_tick);

    sensor_service_update_status_age(&candidate.mmc_status,read_tick);

    sensor_service_update_status_age(&candidate.als_status,read_tick);

    /*
     * 所有复制和派生计算成功后才整体更新调用者缓冲区，
     * 避免调用者看到半新半旧的快照。
     */
    *snapshot = candidate;

    return RT_EOK;
}

/*
 * 简介：复制sensor统一服务当前的累计运行统计。
 * 输入：stats为调用者持有的输出缓冲区，不允许为RT_NULL。
 * 返回值：复制成功返回RT_EOK；空指针返回-RT_EINVAL；
 *         服务未运行或mutex未创建返回-RT_ENOSYS；
 *         获取或释放mutex失败时返回对应RT-Thread错误。
 * 说明：主统计在snapshot mutex内整体复制；独立并发计数在锁外
 *       通过短临界区读取；全部成功后才更新调用者缓冲区。
 */
rt_err_t sensor_service_get_stats(sensor_service_stats_t *stats)
{
    sensor_service_stats_t candidate;
    rt_uint32_t lock_timeout_count;
    rt_uint32_t probe_request_count;
    rt_tick_t lock_timeout_tick;
    rt_err_t result;

    /* 输出缓冲区由调用者持有，服务不返回内部统计对象指针。 */
    if (stats == RT_NULL)
    {
        return -RT_EINVAL;
    }

    /*
     * 服务进入RUNNING且snapshot mutex有效后，
     * 内部累计统计才允许被外部读取。
     */
    if ((sensor_service_is_running() == RT_FALSE) ||(g_sensor_service.snapshot_mutex == RT_NULL))
    {
        return -RT_ENOSYS;
    }

    /*
     * 使用有限等待获取mutex，避免统计查询无限阻塞调用线程。
     */
    lock_timeout_tick =sensor_service_ms_to_tick(SENSOR_SERVICE_LOCK_TIMEOUT_MS);

    result = rt_mutex_take(g_sensor_service.snapshot_mutex,lock_timeout_tick);
    if (result != RT_EOK)
    {
        /*
         * 获取失败时不读取主统计，也不修改调用者缓冲区。
         * 锁失败次数使用独立临界区记录。
         */
        sensor_service_record_lock_timeout();
        return result;
    }

    /*
     * 在锁内整体复制由sensor线程维护的主统计，
     * 防止三颗器件的计数来自不同更新时间点。
     */
    candidate = g_sensor_service.stats;

    result = rt_mutex_release(g_sensor_service.snapshot_mutex);
    if (result != RT_EOK)
    {
        /*
         * mutex释放异常时不把candidate声明为成功结果。
         */
        return result;
    }

    /*
     * lock_timeout和probe_request可能在没有取得snapshot mutex时更新，因此通过独立短临界区取得两者的一致副本。
     */
    sensor_service_read_external_counters(&lock_timeout_count,&probe_request_count);

    /*
     * 用独立并发计数覆盖主统计副本中的对应字段，
     * 形成完整的对外统计结果。
     */
    candidate.snapshot_lock_timeout_count =lock_timeout_count;
    candidate.probe_request_count =probe_request_count;

    /*
     * 所有统计复制完成后才整体更新调用者缓冲区，
     * 失败路径不会留下部分新值。
     */
    *stats = candidate;

    return RT_EOK;
}

/*
 * 简介：向sensor线程提交指定器件的异步重探请求。
 * 输入：device_mask为待重探器件位图，可组合IMU、MMC和ALS；不允许为0或包含未定义位。
 * 返回值：请求成功提交并唤醒线程返回RT_EOK；
 *         位图无效返回-RT_EINVAL；
 *         服务未运行或event无效返回-RT_ENOSYS；
 *         event发送失败时返回对应RT-Thread错误。
 * 说明：请求通过位图OR合并，调用线程不执行I2C；实际身份检查、
 *       配置和首份有效样本确认由唯一sensor线程完成。
 */
rt_err_t sensor_service_request_probe(rt_uint32_t device_mask)
{
    rt_event_t wake_event;
    rt_base_t level;

    /*
     * 位图必须至少选择一个已定义器件。
     * 通过取反掩码检查是否包含保留位或未知器件位。
     */
    if ((device_mask == 0U) ||((device_mask & ~SENSOR_DEVICE_MASK_ALL) != 0U))
    {
        return -RT_EINVAL;
    }

    /*
     * 服务状态、event句柄、待处理位图和请求计数属于同一
     * 并发控制域，在短临界区内完成检查和更新。
     */
    level = rt_hw_interrupt_disable();

    /*
     * 只有服务已经运行且唤醒event有效时才能接收重探请求。
     * INITIALIZING期间不能提前向尚未启动的线程提交控制操作。
     */
    if ((g_sensor_service.state !=SENSOR_SERVICE_STATE_RUNNING) ||(g_sensor_service.wake_event == RT_NULL))
    {
        rt_hw_interrupt_enable(level);
        return -RT_ENOSYS;
    }

    /*
     * 使用OR合并尚未处理的请求。
     * 对同一器件连续请求多次只保留一个待处理位，
     * 避免创建队列对象或形成连续重探。
     */
    g_sensor_service.pending_probe_mask |= device_mask;
    g_sensor_service.probe_request_count++;

    /*
     * 在临界区内复制稳定句柄，退出临界区后再调用RT-Thread API，
     * 避免扩大关中断范围。
     *
     * 当前没有deinit，因此RUNNING状态下event句柄不会被删除。
     */
    wake_event = g_sensor_service.wake_event;

    rt_hw_interrupt_enable(level);

    /*
     * event只负责提前唤醒sensor线程，不携带器件数据。
     * 实际请求位图由sensor线程从pending_probe_mask取得。
     *
     * 若发送event失败，请求位仍然保留，线程后续因周期期限醒来时
     * 仍可处理它；本次调用返回event错误供调用者记录。
     */
    return rt_event_send(wake_event,SENSOR_SERVICE_EVENT_PROBE);
}

/*
 * 简介：为指定器件配置固定次数的软件故障注入。
 * 输入：device为目标器件；fault为注入的故障类型；
 *       count为剩余注入次数，非NONE故障时必须大于0。
 * 返回值：配置成功并唤醒sensor线程返回RT_EOK；
 *         器件、故障类型或次数无效返回-RT_EINVAL；
 *         服务未运行或event无效返回-RT_ENOSYS；
 *         event发送失败时返回对应RT-Thread错误。
 * 说明：故障注入只影响服务层看到的操作结果，不修改真实I2C线路；
 *       每次器件操作最多消费一次，次数耗尽后自动恢复NONE。
 */
rt_err_t sensor_service_test_inject(sensor_device_id_t device,sensor_test_fault_t fault,rt_uint32_t count)
{
    rt_event_t wake_event;
    rt_base_t level;

    /*
     * 枚举在C语言中仍可能由强制类型转换产生负数或越界值，
     * 因此同时检查下界和上界，防止访问faults数组越界。
     */
    if (((rt_int32_t)device <(rt_int32_t)SENSOR_DEVICE_IMU) ||(device >= SENSOR_DEVICE_COUNT))
    {
        return -RT_EINVAL;
    }

    /* 只接受当前接口明确定义的软件故障类型。 */
    if (((rt_int32_t)fault <(rt_int32_t)SENSOR_TEST_FAULT_NONE) ||(fault > SENSOR_TEST_FAULT_TIMEOUT))
    {
        return -RT_EINVAL;
    }

    /*
     * 非NONE故障至少需要注入一次。
     * fault为NONE时用于清除注入，count值不参与后续处理。
     */
    if ((fault != SENSOR_TEST_FAULT_NONE) &&(count == 0U))
    {
        return -RT_EINVAL;
    }

    /*
     * 服务状态、event句柄和faults数组属于同一控制域，
     * 在短临界区内完成状态检查和配置更新。
     */
    level = rt_hw_interrupt_disable();

    /*
     * INITIALIZING或STOPPED状态下没有可处理注入的sensor线程，
     * 因此不能接受故障注入配置。
     */
    if ((g_sensor_service.state !=SENSOR_SERVICE_STATE_RUNNING) ||
        (g_sensor_service.wake_event == RT_NULL))
    {
        rt_hw_interrupt_enable(level);
        return -RT_ENOSYS;
    }

    /* 为目标器件保存新的故障类型。 */
    g_sensor_service.faults[device].fault = fault;

    /*
     * 配置NONE时同时清零剩余次数；
     * 其他故障保存调用者指定的有限注入次数。
     */
    g_sensor_service.faults[device].remaining_count =(fault == SENSOR_TEST_FAULT_NONE)? 0U: count;

    /*
     * 在临界区内复制稳定的event句柄，
     * 退出临界区后再调用RT-Thread API，缩短关中断时间。
     *
     * 当前没有deinit，因此RUNNING期间句柄不会被删除。
     */
    wake_event = g_sensor_service.wake_event;

    rt_hw_interrupt_enable(level);

    /*
     * CONTROL事件只通知sensor线程控制状态已经变化。
     * 具体器件、故障类型和次数仍保存在faults数组中。
     *
     * 若event发送失败，故障配置仍然保留；sensor线程后续因周期
     * 期限醒来时仍可读取它，本次调用返回event错误供诊断记录。
     */
    return rt_event_send(wake_event,SENSOR_SERVICE_EVENT_CONTROL);
}
//---------------------------------------------------------------私有函数---------------------------------------------------------------
/*
 * 简介：统一sensor线程入口，负责三器件初始化、调度、重探和通知。
 * 输入：parameter当前未使用。
 * 返回值：无，线程启动后持续运行。
 * 说明：本线程是传感器I2C的唯一访问者；单器件失败不会终止线程。
 */
static void sensor_service_thread_entry(void *parameter)
{
    rt_uint32_t received_event;
    rt_tick_t loop_start;
    rt_tick_t wait_ticks;

    /* 当前不需要线程参数。 */
    (void)parameter;

    /*
     * 分别初始化三颗器件。
     * 失败状态和重试期限由initialize_device()内部记录。
     */
    (void)sensor_service_initialize_device(SENSOR_DEVICE_IMU,RT_FALSE);

    (void)sensor_service_initialize_device(SENSOR_DEVICE_MMC,RT_FALSE);

    (void)sensor_service_initialize_device(SENSOR_DEVICE_ALS,RT_FALSE);

    /*
     * 当前没有线程退出握手，因此服务启动后持续运行。
     */
    while (1)
    {
        /* 记录本轮处理开始时刻，不包含后面的event等待。 */
        loop_start = rt_tick_get();

        /*
         * 处理外部提交的重探位图。
         * 真实I2C操作仍然只在sensor线程中执行。
         */
        sensor_service_process_probe_requests();

        /*
         * 分别推进三颗器件的调度状态机。
         * 每次重新读取tick，避免前一个器件的耗时影响时间判断。
         */
        sensor_service_process_imu(rt_tick_get());

        sensor_service_process_mmc(rt_tick_get());

        sensor_service_process_als(rt_tick_get());

        /*
         * 按合并位图和最小间隔向app层发送标量通知。
         * 回调在snapshot mutex之外执行。
         */
        sensor_service_notify_if_due();

        /* 记录本轮调度耗时，用于检查是否破坏最快采样周期。 */
        sensor_service_record_loop((rt_tick_t)(rt_tick_get() -loop_start));

        /*
         * 根据next_due、next_poll或retry_due选择最近期限。
         * 返回值至少为1 tick，避免形成忙循环。
         */
        wait_ticks =sensor_service_calculate_wait(rt_tick_get());

        received_event = 0U;

        /*
         * 在最近期限到达前阻塞等待。
         * PROBE或CONTROL事件可以提前唤醒线程；
         * 等待超时属于正常调度，不属于传感器错误。
         */
        (void)rt_event_recv(g_sensor_service.wake_event,SENSOR_SERVICE_EVENT_ALL,RT_EVENT_FLAG_OR |RT_EVENT_FLAG_CLEAR,
                            wait_ticks,
                            &received_event);

        /*
         * 记录event等待返回次数。
         * event只负责唤醒，实际参数保存在service context中。
         */
        sensor_service_record_wake();
    }
}


/*
 * 简介：在短临界区内读取sensor服务当前生命周期状态。
 * 输入：无。
 * 返回值：当前sensor_service_state_t状态值。
 * 说明：只复制一个枚举值，不在临界区内执行I2C、日志、延时
 *       或RT-Thread阻塞操作。
 */
static sensor_service_state_t sensor_service_get_state(void)
{
    sensor_service_state_t state;
    rt_base_t level;

    level = rt_hw_interrupt_disable();

    state = g_sensor_service.state;

    rt_hw_interrupt_enable(level);

    return state;
}
/*
 * 简介：将毫秒时间转换为非零的RT-Thread系统tick。
 * 输入：milliseconds为毫秒数，当前只接收项目内部的小范围时间常量。
 * 返回值：对应的系统tick；非零毫秒值转换结果至少为1 tick；
 *         输入0时返回0。
 * 说明：统一处理RT_TICK_PER_SECOND带来的时间量化，避免非零等待
 *       被转换为0 tick后形成忙循环。
 */
static rt_tick_t sensor_service_ms_to_tick(rt_uint32_t milliseconds)
{
    rt_tick_t ticks;

    /* 使用RT-Thread接口按当前系统tick频率完成换算。 */
    ticks = rt_tick_from_millisecond((rt_int32_t)milliseconds);

    if ((milliseconds != 0U) && (ticks == 0U))
    {
        ticks = 1U;
    }

    return ticks;
}

/*
 * 简介：计算单个器件last-good样本的年龄和过期状态。
 * 输入：status为调用者快照中的器件状态，不允许为RT_NULL；
 *       read_tick为本次快照读取时刻。
 * 返回值：无。
 * 说明：只修改调用者的快照副本，不修改服务内部状态；
 *       valid为RT_FALSE时约定age_ticks=0、stale=RT_TRUE。
 */
static void sensor_service_update_status_age(sensor_device_status_t *status,rt_tick_t read_tick)
{
    rt_tick_t stale_ticks;

    if (status == RT_NULL)
    {
        return;
    }

    /*
     * 没有取得过完整有效样本时不存在年龄基准。
     * age_ticks=0只表示“无法计算”，不能解释为刚产生了样本。
     */
    if (status->valid == RT_FALSE)
    {
        status->age_ticks = 0U;
        status->stale = RT_TRUE;
        return;
    }

    /*
     * 以最近一次成功提交时刻为基准计算年龄。
     * rt_tick_t为无符号类型时，该减法可以处理一次正常tick回绕。
     *
     * 采样失败不能更新last_success_tick，因此失败期间age会持续增加。
     */
    status->age_ticks =(rt_tick_t)(read_tick -status->last_success_tick);

    stale_ticks =sensor_service_ms_to_tick(SENSOR_SERVICE_STALE_MS);

    /*
     * 年龄达到或超过阈值即标记为stale。
     * stale只说明last-good已经过期，不会清除valid或样本内容。
     */
    status->stale =(status->age_ticks >= stale_ticks)? RT_TRUE: RT_FALSE;
}
/*
 * 简介：记录一次snapshot mutex获取超时。
 * 输入：无。
 * 返回值：无。
 * 说明：获取snapshot mutex失败时不能再依赖该mutex更新统计，因此在短临界区内更新独立计数器。
 */
static void sensor_service_record_lock_timeout(void)
{
    rt_base_t level;

    level = rt_hw_interrupt_disable();

    g_sensor_service.lock_timeout_count++;

    rt_hw_interrupt_enable(level);
}

/*
 * 简介：一致读取不受snapshot mutex保护的两个外部计数器。
 * 输入：lock_timeout_count用于返回快照锁超时次数；
 *       probe_request_count用于返回重探请求次数；
 *       两个输出指针均不允许为RT_NULL。
 * 返回值：无。
 * 说明：两个计数器在同一个短临界区内复制，使调用者获得
 *       来自同一观察时刻的统计组合。
 */
static void sensor_service_read_external_counters(rt_uint32_t *lock_timeout_count,rt_uint32_t *probe_request_count)
{
    rt_base_t level;

    if ((lock_timeout_count == RT_NULL) ||(probe_request_count == RT_NULL))
    {
        return;
    }

    /*
     * 临界区只包含两个固定长度标量的读取，
     * 不执行I2C、日志、延时、回调或其他阻塞操作。
     */
    level = rt_hw_interrupt_disable();

    *lock_timeout_count =g_sensor_service.lock_timeout_count;

    *probe_request_count =g_sensor_service.probe_request_count;

    rt_hw_interrupt_enable(level);
}
/*
 * 简介：判断当前tick是否已经到达或越过指定期限。
 * 输入：now为当前tick；deadline为目标期限tick。
 * 返回值：已到期返回RT_TRUE；尚未到期返回RT_FALSE。
 * 说明：期限跨度远小于tick范围的一半，通过有符号差值处理
 *       一次正常计数回绕。
 */
static rt_bool_t sensor_service_tick_reached(rt_tick_t now,rt_tick_t deadline)
{
    /*
     * 差值为0表示刚好到期，正数表示已经逾期，
     * 负数表示deadline仍在未来。
     */
    return ((rt_int32_t)(now - deadline) >= 0)
        ? RT_TRUE
        : RT_FALSE;
}

/*
 * 简介：把器件编号转换为对应的器件位图。
 * 输入：device为目标器件编号。
 * 返回值：编号有效时返回对应位；编号无效时返回0。
 * 说明：统一生成重探、通知和故障控制所使用的器件位图。
 */
static rt_uint32_t sensor_service_device_mask(sensor_device_id_t device)
{
    /* 先验证编号，避免执行越界位移。 */
    if (device >= SENSOR_DEVICE_COUNT)
    {
        return 0U;
    }

    return (1UL << (rt_uint32_t)device);
}

/*
 * 简介：取得指定器件在内部快照中的状态对象。
 * 输入：device为目标器件编号。
 * 返回值：编号有效时返回内部状态指针；否则返回RT_NULL。
 * 说明：返回的是服务内部对象，只允许在snapshot mutex保护下
 *       短暂访问，不得保存该指针或提供给外部模块。
 */
static sensor_device_status_t *sensor_service_get_device_status(sensor_device_id_t device)
{
    switch (device)
    {
    case SENSOR_DEVICE_IMU:
        return &g_sensor_service.snapshot.imu_status;

    case SENSOR_DEVICE_MMC:
        return &g_sensor_service.snapshot.mmc_status;

    case SENSOR_DEVICE_ALS:
        return &g_sensor_service.snapshot.als_status;

    default:
        return RT_NULL;
    }
}

/*
 * 简介：取得指定器件的内部累计统计对象。
 * 输入：device为目标器件编号。
 * 返回值：编号有效时返回内部统计指针；否则返回RT_NULL。
 * 说明：返回的是服务内部对象，只允许在snapshot mutex保护下
 *       短暂访问，不得把该指针暴露给调用者。
 */
static sensor_device_stats_t *sensor_service_get_device_stats_internal(sensor_device_id_t device)
{
    switch (device)
    {
    case SENSOR_DEVICE_IMU:
        return &g_sensor_service.stats.imu;

    case SENSOR_DEVICE_MMC:
        return &g_sensor_service.stats.mmc;

    case SENSOR_DEVICE_ALS:
        return &g_sensor_service.stats.als;

    default:
        return RT_NULL;
    }
}

/*
 * 简介：在有限时间内获取snapshot mutex。
 * 输入：无。
 * 返回值：成功取得锁返回RT_EOK；失败时返回rt_mutex_take()
 *         的实际错误码。
 * 说明：获取失败时通过独立临界区记录锁超时，避免再次依赖
 *       当前无法取得的snapshot mutex。
 */
static rt_err_t sensor_service_lock_snapshot(void)
{
    rt_err_t result;

    /*
     * 使用统一的有限等待时间，防止调用线程永久阻塞。
     * 本函数只负责加锁，成功后的释放责任属于调用者。
     */
    result = rt_mutex_take(g_sensor_service.snapshot_mutex,sensor_service_ms_to_tick(SENSOR_SERVICE_LOCK_TIMEOUT_MS));

    if (result != RT_EOK)
    {
        sensor_service_record_lock_timeout();
    }

    return result;
}
/*
 * 简介：为一次高层器件事务消费至多一次软件故障注入。
 * 输入：device为本次操作对应的器件编号。
 * 返回值：无故障返回RT_EOK；总线或短传输故障返回-RT_EIO；
 *         超时故障返回-RT_ETIMEOUT；参数或故障类型无效
 *         返回-RT_EINVAL。
 * 说明：MMC和ALS进入ready等待后不再重复消费，确保一次故障
 *       只作用于一个完整高层事务。软件注入只能验证服务错误
 *       分支，不能证明真实线路产生了对应故障。
 */
static rt_err_t sensor_service_consume_fault(sensor_device_id_t device)
{
    sensor_test_fault_t fault;
    rt_base_t level;

    /* 防止访问faults数组时发生越界。 */
    if (device >= SENSOR_DEVICE_COUNT)
    {
        return -RT_EINVAL;
    }

    /*
     * fault和remaining_count必须作为一组原子更新，
     * 避免测试接口修改配置时sensor线程读取到不一致组合。
     */
    level = rt_hw_interrupt_disable();

    fault = g_sensor_service.faults[device].fault;

    if ((fault != SENSOR_TEST_FAULT_NONE) &&(g_sensor_service.faults[device].remaining_count > 0U))
    {
        /* 每个高层事务最多减少一次剩余注入次数。 */
        g_sensor_service.faults[device].remaining_count--;

        /*
         * 最后一次故障被消费后立即恢复为NONE，
         * 后续事务自动回到真实驱动路径。
         */
        if (g_sensor_service.faults[device].remaining_count == 0U)
        {
            g_sensor_service.faults[device].fault =SENSOR_TEST_FAULT_NONE;
        }
    }
    else
    {
        /*
         * NONE或剩余次数为0都视为本次不注入。
         * 局部fault归一化后再在临界区外转换错误码。
         */
        fault = SENSOR_TEST_FAULT_NONE;
    }

    rt_hw_interrupt_enable(level);

    /*
     * 临界区外完成错误码映射，缩短关中断时间。
     */
    switch (fault)
    {
    case SENSOR_TEST_FAULT_NONE:
        return RT_EOK;

    case SENSOR_TEST_FAULT_BUS_IO:
    case SENSOR_TEST_FAULT_SHORT_TRANSFER:
        /*
         * 底层完整长度检查会把短传输映射为-RT_EIO，
         * 因此两种注入在当前服务接口中使用相同错误码。
         */
        return -RT_EIO;

    case SENSOR_TEST_FAULT_TIMEOUT:
        return -RT_ETIMEOUT;

    default:
        /* 防御异常枚举值，避免把未知注入当作成功。 */
        return -RT_EINVAL;
    }
}

/*
 * 简介：记录一次不改变器件成功或失败状态的高层操作。
 * 输入：device为目标器件；operation_ticks为本次操作耗时。
 * 返回值：无。
 * 说明：只更新尝试次数和耗时统计，不修改样本、状态或序号。
 */
static void sensor_service_record_operation(sensor_device_id_t device,rt_tick_t operation_ticks)
{
    sensor_device_stats_t *stats;

    /* 统计对象属于统一快照，必须在snapshot mutex内修改。 */
    if (sensor_service_lock_snapshot() != RT_EOK)
    {
        return;
    }

    stats =sensor_service_get_device_stats_internal(device);

    if (stats != RT_NULL)
    {
        stats->attempt_count++;
        stats->last_operation_ticks = operation_ticks;

        /* 保存服务运行期间观察到的单次最大操作耗时。 */
        if (operation_ticks > stats->max_operation_ticks)
        {
            stats->max_operation_ticks = operation_ticks;
        }
    }

    (void)rt_mutex_release(g_sensor_service.snapshot_mutex);
}

/*
 * 简介：记录一次器件尚未产生新数据的操作结果。
 * 输入：device为目标器件；operation_ticks为本次检查耗时。
 * 返回值：无。
 * 说明：not-ready属于正常等待状态，不计入failure和timeout，
 *       也不改变器件状态、连续失败次数或last-good样本。
 */
static void sensor_service_record_not_ready(sensor_device_id_t device,rt_tick_t operation_ticks)
{
    sensor_device_stats_t *stats;

    /* 统计对象属于统一快照，必须在snapshot mutex内修改。 */
    if (sensor_service_lock_snapshot() != RT_EOK)
    {
        return;
    }

    stats =sensor_service_get_device_stats_internal(device);

    if (stats != RT_NULL)
    {
        stats->attempt_count++;
        stats->not_ready_count++;
        stats->last_operation_ticks = operation_ticks;

        if (operation_ticks > stats->max_operation_ticks)
        {
            stats->max_operation_ticks = operation_ticks;
        }
    }

    (void)rt_mutex_release(g_sensor_service.snapshot_mutex);
}

/*
 * 简介：记录一次器件操作失败并安排有限重试或离线重探。
 * 输入：device为目标器件；error为本次错误码；
 *       operation_ticks为本次操作耗时。
 * 返回值：无。
 * 说明：先更新sensor线程独占的调度状态，再尝试发布失败状态；
 *       即使快照锁获取失败，也不会对故障器件进行紧密重试。
 *       失败保留last-good样本、valid、seq和最近成功时间。
 */
static void sensor_service_record_failure(sensor_device_id_t device,rt_err_t error,rt_tick_t operation_ticks)
{
    sensor_device_runtime_t *runtime;
    sensor_device_status_t *status;
    sensor_device_stats_t *stats;
    rt_tick_t now;

    /* 防止访问runtime数组时发生越界。 */
    if (device >= SENSOR_DEVICE_COUNT)
    {
        return;
    }

    now = rt_tick_get();
    runtime = &g_sensor_service.runtime[device];

    /*
     * runtime只由sensor线程修改，不需要snapshot mutex。
     * 先退出ready等待阶段，避免失败后继续轮询旧事务。
     */
    runtime->consecutive_failures++;
    runtime->wait_phase = SENSOR_WAIT_PHASE_IDLE;

    /*
     * 连续失败达到阈值后转为OFFLINE并使用较长重探周期；
     * 阈值以内进入TEMP_FAILED并安排较短的有限重试。
     */
    if (runtime->consecutive_failures >=SENSOR_SERVICE_FAILURE_THRESHOLD)
    {
        runtime->state = SENSOR_DEVICE_STATE_OFFLINE;
        runtime->configured = RT_FALSE;

        /*
        * OFFLINE后执行的是较长周期重探，不再属于TEMP_FAILED受控重试。
        */
        runtime->retry_count_pending = RT_FALSE;
        runtime->retry_due_tick =now + sensor_service_ms_to_tick(SENSOR_SERVICE_OFFLINE_REPROBE_MS);
    }
    else
    {
        runtime->state =SENSOR_DEVICE_STATE_TEMP_FAILED;
        runtime->retry_count_pending = RT_TRUE;
        runtime->retry_due_tick =now + sensor_service_ms_to_tick(SENSOR_SERVICE_TEMP_RETRY_MS);
    }

    /* 第一份后续有效样本到达前，器件一直处于恢复过程。 */
    runtime->recovery_pending = RT_TRUE;

    /*
     * 调度安全已经建立；锁失败只会损失本次统计和状态发布，
     * 不会导致sensor线程立刻重试。
     */
    if (sensor_service_lock_snapshot() != RT_EOK)
    {
        return;
    }

    status = sensor_service_get_device_status(device);
    stats =sensor_service_get_device_stats_internal(device);

    if ((status != RT_NULL) && (stats != RT_NULL))
    {
        stats->attempt_count++;
        stats->failure_count++;

        stats->consecutive_failure_count =runtime->consecutive_failures;

        stats->last_operation_ticks = operation_ticks;

        if (operation_ticks > stats->max_operation_ticks)
        {
            stats->max_operation_ticks = operation_ticks;
        }

        /* timeout是failure的子集，只对明确超时错误单独计数。 */
        if (error == -RT_ETIMEOUT)
        {
            stats->timeout_count++;
        }

        /*
         * 只更新状态和错误原因。
         * 失败不能覆盖last-good数据，也不能增加器件样本序号。
         */
        status->state = runtime->state;
        status->last_error = error;

        /*
         * 状态变化同样属于一次发布，但不代表产生了新样本；
         * 因此只增加publish_seq，不增加status->seq。
         */
        g_sensor_service.snapshot.publish_seq++;
        g_sensor_service.stats.publish_count++;

        /*
         * 合并同一器件的重复通知，并把失败状态标记为紧急通知。
         */
        g_sensor_service.notify_pending_mask |=sensor_service_device_mask(device);

        g_sensor_service.notify_urgent = RT_TRUE;
    }

    (void)rt_mutex_release(g_sensor_service.snapshot_mutex);
}

/*
 * 简介：提交一份完整有效样本并原子更新状态和统计。
 * 输入：device为样本所属器件；sample指向对应器件的完整样本；
 *       operation_ticks为本次驱动操作耗时。
 * 返回值：提交并释放锁成功返回RT_EOK；参数无效返回-RT_EINVAL；
 *         获取或释放mutex失败返回RT-Thread实际错误。
 * 说明：sample的实际类型必须与device匹配；服务先复制完整样本，
 *       再一次性发布状态、序号和统计，调用者不能传入内部对象。
 *       若mutex释放失败，样本内容可能已经完成提交。
 */
static rt_err_t sensor_service_commit_sample(sensor_device_id_t device,const void *sample,rt_tick_t operation_ticks)
{
    sensor_device_runtime_t *runtime;
    sensor_device_status_t *status;
    sensor_device_stats_t *stats;
    rt_tick_t sample_tick;
    rt_uint32_t config_version;
    rt_bool_t recovered;
    rt_err_t result;

    /* 同时验证器件编号和样本缓冲区。 */
    if ((device >= SENSOR_DEVICE_COUNT) ||
        (sample == RT_NULL))
    {
        return -RT_EINVAL;
    }

    /*
     * 样本、状态和统计必须在同一临界范围内提交，
     * 防止调用者取得半新半旧的快照。
     */
    result = sensor_service_lock_snapshot();
    if (result != RT_EOK)
    {
        return result;
    }

    runtime = &g_sensor_service.runtime[device];
    status = sensor_service_get_device_status(device);
    stats = sensor_service_get_device_stats_internal(device);

    if ((status == RT_NULL) || (stats == RT_NULL))
    {
        (void)rt_mutex_release(g_sensor_service.snapshot_mutex);

        return -RT_EINVAL;
    }

    /*
     * 根据器件编号解释sample类型并整体复制。
     * 同时提取统一状态字段需要的采样时刻和配置版本。
     */
    switch (device)
    {
    case SENSOR_DEVICE_IMU:
    {
        const sensor_imu_sample_t *imu =(const sensor_imu_sample_t *)sample;

        g_sensor_service.snapshot.imu = *imu;
        sample_tick = imu->sample_tick;
        config_version = imu->config_version;
        break;
    }

    case SENSOR_DEVICE_MMC:
    {
        const sensor_magnetometer_sample_t *mmc =(const sensor_magnetometer_sample_t *)sample;

        g_sensor_service.snapshot.mmc = *mmc;
        sample_tick = mmc->sample_tick;
        config_version = mmc->config_version;
        break;
    }

    case SENSOR_DEVICE_ALS:
    {
        const sensor_als_sample_t *als =(const sensor_als_sample_t *)sample;

        g_sensor_service.snapshot.als = *als;
        sample_tick = als->sample_tick;
        config_version = als->config_version;
        break;
    }

    default:(void)rt_mutex_release(g_sensor_service.snapshot_mutex);

        return -RT_EINVAL;
    }

    /*
     * 只有TEMP_FAILED、OFFLINE或明确处于恢复过程的器件，
     * 成功取得第一份完整样本时才计为一次恢复。
     */
    recovered =((runtime->state ==SENSOR_DEVICE_STATE_TEMP_FAILED) ||
                (runtime->state ==SENSOR_DEVICE_STATE_OFFLINE) ||
                (runtime->recovery_pending == RT_TRUE))
        ? RT_TRUE
        : RT_FALSE;

    /*
     * 完整样本已经复制成功，器件恢复READY并清除连续失败状态。
     */
    runtime->state = SENSOR_DEVICE_STATE_READY;
    runtime->configured = RT_TRUE;
    runtime->recovery_pending = RT_FALSE;
    runtime->retry_count_pending = RT_FALSE;
    runtime->consecutive_failures = 0U;

    /*
     * 器件seq只在完整有效样本提交时增加。
     * sample_tick同时作为当前样本时刻和最近成功时刻。
     */
    status->state = SENSOR_DEVICE_STATE_READY;
    status->valid = RT_TRUE;
    status->stale = RT_FALSE;
    status->seq++;
    status->config_version = config_version;
    status->sample_tick = sample_tick;
    status->last_success_tick = sample_tick;
    status->age_ticks = 0U;
    status->last_error = RT_EOK;

    /* 成功事务更新器件累计统计并清除公开连续失败计数。 */
    stats->attempt_count++;
    stats->success_count++;
    stats->consecutive_failure_count = 0U;
    stats->last_operation_ticks = operation_ticks;

    if (operation_ticks > stats->max_operation_ticks)
    {
        stats->max_operation_ticks = operation_ticks;
    }

    /*
     * 恢复通知需要尽快发出；正常周期样本仍使用合并和限频机制。
     */
    if (recovered == RT_TRUE)
    {
        stats->recovery_count++;
        g_sensor_service.notify_urgent = RT_TRUE;
    }

    /*
     * publish_seq描述统一快照发布，独立于单器件样本seq。
     */
    g_sensor_service.snapshot.publish_seq++;
    g_sensor_service.stats.publish_count++;

    g_sensor_service.notify_pending_mask |=sensor_service_device_mask(device);

    result = rt_mutex_release(g_sensor_service.snapshot_mutex);

    return result;
}
/*
 * 简介：记录一次器件调度晚于计划期限。
 * 输入：device为发生晚采的器件编号。
 * 返回值：无。
 * 说明：late表示操作开始时已经超过本周期期限，但不一定跳过了
 *       完整周期；调用者必须保证device有效。
 */
static void sensor_service_record_late(sensor_device_id_t device)
{
    sensor_device_stats_t *stats;

    /* 调度统计属于统一统计快照，必须在mutex内修改。 */
    if (sensor_service_lock_snapshot() != RT_EOK)
    {
        return;
    }

    stats =sensor_service_get_device_stats_internal(device);

    if (stats != RT_NULL)
    {
        stats->late_count++;
    }

    (void)rt_mutex_release(g_sensor_service.snapshot_mutex);
}

/*
 * 简介：按绝对周期推进器件的下一次采样期限。
 * 输入：device为目标器件；now为本次操作完成时的当前tick。
 * 返回值：无。
 * 说明：调用者必须保证device有效且period_ticks非零。若已经落后
 *       一个或多个完整周期，则直接跳到未来最近期限并累计missed，
 *       不连续补采历史周期。
 */
static void sensor_service_advance_period(sensor_device_id_t device,rt_tick_t now)
{
    sensor_device_runtime_t *runtime;
    sensor_device_stats_t *stats;
    rt_tick_t next_due;
    rt_uint32_t missed = 0U;

    runtime = &g_sensor_service.runtime[device];

    /*
     * 基于上一次绝对期限推进一个周期，
     * 避免使用now重新计时造成长期调度漂移。
     */
    next_due =runtime->next_due_tick +runtime->period_ticks;

    /*
     * 如果推进一个周期后仍已到期，说明至少漏掉一个完整周期。
     * 直接计算需要跳过的周期数，不通过循环逐周期追赶。
     */
    if (sensor_service_tick_reached(now,next_due) == RT_TRUE)
    {
        missed =(rt_uint32_t)((now - next_due) /runtime->period_ticks) +1U;

        /*
         * 将期限一次推进到now之后最近的周期边界，
         * 保持原有绝对时间基准。
         */
        next_due +=(rt_tick_t)(missed *runtime->period_ticks);
    }

    /* runtime由唯一sensor线程维护，不需要snapshot mutex。 */
    runtime->next_due_tick = next_due;

    /* 没有跳过完整周期时无需更新统计。 */
    if (missed == 0U)
    {
        return;
    }

    /*
     * 调度期限已经完成推进；即使统计锁失败，
     * 服务也不会回头补采已经跳过的周期。
     */
    if (sensor_service_lock_snapshot() != RT_EOK)
    {
        return;
    }

    stats =sensor_service_get_device_stats_internal(device);

    if (stats != RT_NULL)
    {
        stats->missed_count += missed;
    }

    (void)rt_mutex_release(g_sensor_service.snapshot_mutex);
}
/*
 * 简介：初始化或重新初始化指定器件并建立首次采样期限。
 * 输入：device为目标器件；force为RT_TRUE时要求执行恢复初始化。
 * 返回值：初始化成功返回RT_EOK；器件编号无效返回-RT_EINVAL；
 *         其余失败返回故障注入、总线或器件驱动的实际错误码。
 * 说明：初始化成功只表示身份和冻结配置已经验证，不会立即把
 *       器件标记为READY；只有提交第一份完整有效样本后才能
 *       进入READY。调用上下文必须是唯一sensor线程。
 */
static rt_err_t sensor_service_initialize_device(sensor_device_id_t device,rt_bool_t force)
{
    sensor_device_runtime_t *runtime;
    rt_tick_t start_tick;
    rt_tick_t operation_ticks;
    rt_tick_t now;
    rt_err_t result;

    lsm6ds3tr_c_config_readback_t imu_readback = {0};
    ltr303als01_config_readback_t als_readback = {0};

    /* 防止访问runtime数组时发生越界。 */
    if (device >= SENSOR_DEVICE_COUNT)
    {
        return -RT_EINVAL;
    }

    runtime = &g_sensor_service.runtime[device];

    /*
     * 每次完整初始化事务最多消费一次故障注入。
     * 注入生效时不再进入真实总线和器件初始化路径。
     */
    result = sensor_service_consume_fault(device);

    start_tick = rt_tick_get();

    if (result == RT_EOK)
    {
        /*
         * 总线适配初始化必须保持幂等。
         * 上一次未找到i2c3时，后续重探仍可重新尝试。
         */
        result = sensor_bus_port_init();
    }

    if (result == RT_EOK)
    {
        /*
         * IMU初始化入口会完成冻结配置。
         * MMC和ALS在恢复场景下使用专用重新初始化入口。
         */
        switch (device)
        {
        case SENSOR_DEVICE_IMU:
            result =lsm6ds3tr_c_init(&imu_readback);
            break;

        case SENSOR_DEVICE_MMC:
            result = (force == RT_TRUE)? mmc5603nj_reinitialize(): mmc5603nj_init();
            break;

        case SENSOR_DEVICE_ALS:
            result = (force == RT_TRUE)? ltr303als01_reinitialize(&als_readback): ltr303als01_init(&als_readback);
            break;

        default:
            result = -RT_EINVAL;
            break;
        }
    }

    /*
     * operation_ticks覆盖故障注入判断之后的初始化路径，
     * 用于统计本次高层初始化事务的实际执行耗时。
     */
    operation_ticks =(rt_tick_t)(rt_tick_get() - start_tick);

    if (result != RT_EOK)
    {
        /*
         * 初始化失败后清除配置有效标记，
         * 再由统一失败路径安排有限重试或离线重探。
         */
        runtime->configured = RT_FALSE;

        sensor_service_record_failure(device,result,operation_ticks);

        return result;
    }

    /*
     * 初始化成功本身不是有效样本成功，
     * 因此只记录普通操作和耗时。
     */
    sensor_service_record_operation(device,operation_ticks);

    now = rt_tick_get();

    runtime->configured = RT_TRUE;
    runtime->wait_phase = SENSOR_WAIT_PHASE_IDLE;

    /*
     * 失败或离线后的初始化成功仍处于恢复过程。
     * 第一份完整有效样本提交后才清除此标志。
     */
    runtime->recovery_pending =((runtime->state ==SENSOR_DEVICE_STATE_TEMP_FAILED) ||
                                (runtime->state ==SENSOR_DEVICE_STATE_OFFLINE))
                                ? RT_TRUE
                                : RT_FALSE;

    /*
     * ALS进入Active后等待一个完整的200 ms更新周期。
     * IMU初始化已经完成启动样本处理；MMC需要立即触发
     * 第一轮单次测量，因此二者均从当前时刻开始调度。
     */
    runtime->next_due_tick =(device == SENSOR_DEVICE_ALS)? now + runtime->period_ticks: now;


    return RT_EOK;
}

/*
 * 简介：原子取得并清空当前待处理的器件重探位图。
 * 输入：无。
 * 返回值：返回本次取得的器件位图；没有请求时返回0。
 * 说明：请求提交者通过按位OR合并请求，sensor线程通过一次
 *       取走完成所有权转移；取走后到达的新请求保留到下一轮。
 */
static rt_uint32_t sensor_service_take_probe_mask(void)
{
    rt_uint32_t mask;
    rt_base_t level;

    /*
     * 读取和清零必须位于同一短临界区，
     * 防止并发提交的重探请求被覆盖。
     */
    level = rt_hw_interrupt_disable();

    mask = g_sensor_service.pending_probe_mask;
    g_sensor_service.pending_probe_mask = 0U;

    rt_hw_interrupt_enable(level);

    return mask;
}

/*
 * 简介：处理当前合并的三器件受控重探请求。
 * 输入：无。
 * 返回值：无。
 * 说明：只由sensor线程调用，按固定顺序执行真实初始化；
 *       单个器件失败由统一失败状态机记录，不阻止后续器件重探。
 */
static void sensor_service_process_probe_requests(void)
{
    rt_uint32_t mask;

    /* 一次性取得本轮请求，避免处理过程中长期关中断。 */
    mask = sensor_service_take_probe_mask();

    if ((mask & SENSOR_DEVICE_MASK_IMU) != 0U)
    {
        /*
         * 显式重探统一传入force。
         * IMU入口本身会重新验证身份和冻结配置。
         */
        (void)sensor_service_initialize_device( SENSOR_DEVICE_IMU,RT_TRUE);
    }

    if ((mask & SENSOR_DEVICE_MASK_MMC) != 0U)
    {
        (void)sensor_service_initialize_device(SENSOR_DEVICE_MMC,RT_TRUE);
    }

    if ((mask & SENSOR_DEVICE_MASK_ALS) != 0U)
    {
        (void)sensor_service_initialize_device(SENSOR_DEVICE_ALS,RT_TRUE);
    }
}
/*
 * 简介：判断指定器件当前是否可以执行下一次采样操作。
 * 输入：device为目标器件；now为本轮调度观察到的当前tick。
 * 返回值：器件已配置且已到采样或重试期限返回RT_TRUE；
 *         尚未到期、未配置或恢复初始化失败返回RT_FALSE。
 * 说明：本函数只处理调度准入、晚采统计和必要的恢复初始化，
 *       不执行实际采样。调用者必须保证device有效，并且只能
 *       从唯一sensor线程调用。
 */
static rt_bool_t sensor_service_prepare_device(sensor_device_id_t device,rt_tick_t now)
{
    sensor_device_runtime_t *runtime;
    rt_tick_t deadline;

    runtime = &g_sensor_service.runtime[device];

    /*
     * 暂时失败和离线器件使用retry_due_tick调度，
     * 不再按照正常采样期限进行紧密重试。
     */
    if ((runtime->state ==SENSOR_DEVICE_STATE_TEMP_FAILED) ||
        (runtime->state ==SENSOR_DEVICE_STATE_OFFLINE))
    {
        deadline = runtime->retry_due_tick;

        /* 重试期限尚未到达时保持退避。 */
        if (sensor_service_tick_reached(now,deadline) == RT_FALSE)
        {
            return RT_FALSE;
        }

        /*
         * 已超过预定重试期限时记录一次晚采。
         * 刚好在deadline执行不计为late。
         */
        if (now != deadline)
        {
            sensor_service_record_late(device);
        }

        /*
        * TEMP_FAILED的退避期限已经到达，现在才算真正开始一次受控重试。
        * 先清除pending再更新统计，避免快照锁失败时下一轮重复计数。
        */
        if ((runtime->state == SENSOR_DEVICE_STATE_TEMP_FAILED) &&
            (runtime->retry_count_pending == RT_TRUE))
        {
            runtime->retry_count_pending = RT_FALSE;
            sensor_service_record_retry(device);
        }
        /*
         * OFFLINE或配置已失效时，必须先重新验证身份和配置。
         * 初始化失败会由统一失败路径重新安排下一次重探。
         */
        if (runtime->configured == RT_FALSE)
        {
            if (sensor_service_initialize_device(device,RT_TRUE) != RT_EOK)
            {
                return RT_FALSE;
            }

            /*
             * 初始化可能包含阻塞访问和等待，
             * 因此必须重新获取当前tick再判断采样期限。
             */
            now = rt_tick_get();

            /*
             * IMU和MMC通常可以立即进入下一阶段；
             * ALS重初始化后必须等待一个完整测量更新周期。
             */
            return sensor_service_tick_reached(now,runtime->next_due_tick);
        }

        /*
         * 配置仍有效的暂时失败器件到达重试期限后，
         * 可以直接重试采样，无需重复初始化。
         */
        return RT_TRUE;
    }

    /*
     * 未初始化或配置无效的正常状态器件不能采样。
     * 初始配置和显式重探由其他调度路径负责。
     */
    if (runtime->configured == RT_FALSE)
    {
        return RT_FALSE;
    }

    /* 正常采样期限尚未到达。 */
    if (sensor_service_tick_reached(now,runtime->next_due_tick) == RT_FALSE)
    {
        return RT_FALSE;
    }

    /*
     * 已到期但不是刚好命中期限时记录late。
     * late不一定代表跳过了完整周期。
     */
    if (now != runtime->next_due_tick)
    {
        sensor_service_record_late(device);
    }

    return RT_TRUE;
}
/*
 * 简介：执行一次IMU调度、采样、提交或失败处理。
 * 输入：now为本轮调度开始时的当前tick。
 * 返回值：无。
 * 说明：只有读取成功且sample.valid为RT_TRUE时才提交快照；
 *       -RT_EBUSY表示当前没有新样本，不计为器件失败；
 *       其他错误进入统一有限重试和离线恢复路径。
 */
static void sensor_service_process_imu(rt_tick_t now)
{
    sensor_device_runtime_t *runtime;
    sensor_imu_sample_t sample = {0};

    rt_tick_t start_tick;
    rt_tick_t operation_ticks;
    rt_err_t result;

    runtime =&g_sensor_service.runtime[SENSOR_DEVICE_IMU];

    /*
     * 未到采样或重试期限、尚未配置或恢复初始化失败时，
     * 本轮不访问IMU。
     */
    if (sensor_service_prepare_device(SENSOR_DEVICE_IMU,now) == RT_FALSE)
    {
        return;
    }

    /*
     * 一次IMU高层读取最多消费一次故障注入。
     * 注入生效时不会进入真实驱动读取。
     */
    result = sensor_service_consume_fault(SENSOR_DEVICE_IMU);

    start_tick = rt_tick_get();

    if (result == RT_EOK)
    {
        result = lsm6ds3tr_c_read_sample(&sample);
    }

    /* 记录本次实际驱动读取或注入错误路径的执行耗时。 */
    operation_ticks =(rt_tick_t)(rt_tick_get() - start_tick);

    /*
     * 只有返回成功且驱动明确标记样本有效时，
     * 才允许发布完整IMU快照和增加器件样本序号。
     */
    if ((result == RT_EOK) &&(sample.valid == RT_TRUE))
    {
        (void)sensor_service_commit_sample(
            SENSOR_DEVICE_IMU,
            &sample,
            operation_ticks);

        /*
         * 无论通知消费者是否及时读取，都按照绝对周期
         * 推进下一期限，避免围绕当前样本重复采集。
         */
        sensor_service_advance_period(
            SENSOR_DEVICE_IMU,
            rt_tick_get());

        return;
    }

    /*
     * -RT_EBUSY表示当前尚无新数据，属于正常调度现象，
     * 不增加failure、timeout或连续失败次数。
     */
    if (result == -RT_EBUSY)
    {
        sensor_service_record_not_ready(SENSOR_DEVICE_IMU,operation_ticks);

        if ((runtime->state ==SENSOR_DEVICE_STATE_TEMP_FAILED) ||
            (runtime->state ==SENSOR_DEVICE_STATE_OFFLINE))
        {
            /*
             * 恢复阶段尚无新数据时，从当前时刻延后一个周期，
             * 防止线程立即重复读取状态寄存器。
             */
            runtime->retry_due_tick =rt_tick_get() + runtime->period_ticks;
        }
        else
        {
            /*
             * 正常状态下保持绝对周期推进，
             * 并统计已经跳过的完整周期。
             */
            sensor_service_advance_period(SENSOR_DEVICE_IMU,rt_tick_get());
        }

        return;
    }

    /*
     * 驱动返回成功但样本无效属于契约异常，
     * 统一转换为I/O错误，不能作为有效样本发布。
     */
    if (result == RT_EOK)
    {
        result = -RT_EIO;
    }

    /*
     * 真实驱动错误、注入错误和无效样本都进入统一失败路径，
     * 由连续失败阈值决定暂时失败或离线退避。
     */
    sensor_service_record_failure(SENSOR_DEVICE_IMU,result,operation_ticks);
}

/*
 * 简介：推进MMC5603NJ单次测量状态机并提交有效磁场样本。
 * 输入：now为本轮调度开始时的当前tick。
 * 返回值：无。
 * 说明：MMC采样分为“触发测量”和“等待ready并读取”两个阶段；
 *       ready检查使用有限期限和分段轮询，不阻塞sensor线程。
 *       每个完整测量事务只在触发阶段消费一次故障注入。
 */
static void sensor_service_process_mmc(rt_tick_t now)
{
    sensor_device_runtime_t *runtime;
    sensor_magnetometer_sample_t sample = {0};

    rt_tick_t start_tick;
    rt_tick_t operation_ticks;
    rt_err_t result;

    runtime = &g_sensor_service.runtime[SENSOR_DEVICE_MMC];

    /*
     * READY阶段表示单次测量已经触发，
     * 当前只需按计划检查数据是否就绪。
     */
    if (runtime->wait_phase ==SENSOR_WAIT_PHASE_READY)
    {
        /* 尚未到下一次ready检查时刻，本轮不访问I2C。 */
        if (sensor_service_tick_reached(now,runtime->next_poll_tick) == RT_FALSE)
        {
            return;
        }

        start_tick = rt_tick_get();

        /*
         * try接口只检查一次ready并尝试读取，
         * 内部不循环、不延时，避免阻塞其他器件调度。
         */
        result =mmc5603nj_try_read_sample(&sample);

        operation_ticks =(rt_tick_t)(rt_tick_get() - start_tick);

        /*
         * 返回成功且样本有效时，本次测量事务才真正完成。
         */
        if ((result == RT_EOK) && (sample.valid == RT_TRUE))
        {
            runtime->wait_phase = SENSOR_WAIT_PHASE_IDLE;

            (void)sensor_service_commit_sample(
                SENSOR_DEVICE_MMC,
                &sample,
                operation_ticks);

            /*
             * 从原有绝对期限推进下一采样周期，
             * 逾期时跳过历史周期，不连续补采。
             */
            sensor_service_advance_period(
                SENSOR_DEVICE_MMC,
                rt_tick_get());

            return;
        }

        /*
         * -RT_EBUSY表示本次状态检查时数据尚未就绪，
         * 不属于器件失败，也不会消费新的故障注入。
         */
        if (result == -RT_EBUSY)
        {
            now = rt_tick_get();

            /*
             * 到达ready总期限后结束本次测量，
             * 统一按超时进入失败、退避和恢复路径。
             */
            if (sensor_service_tick_reached(now,runtime->ready_deadline_tick) ==RT_TRUE)
            {
                sensor_service_record_failure(
                    SENSOR_DEVICE_MMC,
                    -RT_ETIMEOUT,
                    operation_ticks);
            }
            else
            {
                /*
                * 本次驱动调用确实访问了STATUS，只是数据尚未就绪。
                * 因此记录attempt和not_ready，但不增加failure或timeout。
                */
                sensor_service_record_not_ready(SENSOR_DEVICE_MMC,operation_ticks);
                /*
                 * 尚未超时时安排下一次检查。
                 * 当前线程可以继续调度IMU和ALS。
                 */
                runtime->next_poll_tick =now +sensor_service_ms_to_tick(SENSOR_SERVICE_MMC_POLL_MS);
            }

            return;
        }

        /*
         * 驱动返回成功但样本无效违反有效样本契约，
         * 统一转换为I/O错误，禁止发布该样本。
         */
        if (result == RT_EOK)
        {
            result = -RT_EIO;
        }

        sensor_service_record_failure(SENSOR_DEVICE_MMC,result,operation_ticks);

        return;
    }

    /*
     * IDLE阶段只有在器件已配置且采样或重试期限到达时，才允许触发下一次单次测量
     */
    if (sensor_service_prepare_device(SENSOR_DEVICE_MMC,now) == RT_FALSE)
    {
        return;
    }

    /*
     * 一个高层MMC测量事务只在触发前消费一次故障注入。
     * 后续ready轮询属于同一事务，不重复消费。
     */
    result =sensor_service_consume_fault(SENSOR_DEVICE_MMC);

    start_tick = rt_tick_get();

    if (result == RT_EOK)
    {
        result = mmc5603nj_start_measurement();
    }

    operation_ticks =(rt_tick_t)(rt_tick_get() - start_tick);

    /*
     * 触发失败时没有进入READY阶段，
     * 直接交给统一失败状态机安排有限重试。
     */
    if (result != RT_EOK)
    {
        sensor_service_record_failure(SENSOR_DEVICE_MMC,
                                    result,
                                    operation_ticks);

        return;
    }

    /*
     * 测量触发成功只说明命令已经发送，
     * 此时尚未产生完整有效样本，不能增加success_count或seq。
     */
    sensor_service_record_operation(SENSOR_DEVICE_MMC,operation_ticks);

    now = rt_tick_get();

    /*
     * 进入非阻塞READY等待阶段。
     * 下次轮询时间控制检查频率，总期限保证等待有界。
     */
    runtime->wait_phase =SENSOR_WAIT_PHASE_READY;

    runtime->next_poll_tick =now +sensor_service_ms_to_tick(SENSOR_SERVICE_MMC_POLL_MS);

    runtime->ready_deadline_tick =now +sensor_service_ms_to_tick(SENSOR_SERVICE_MMC_READY_TIMEOUT_MS);
}

/*
 * 简介：推进LTR303连续测量的NEW等待窗口并提交有效ALS样本。
 * 输入：now为本轮调度开始时的当前tick。
 * 返回值：无。
 * 说明：ALS始终由硬件连续测量，READY并不表示软件触发了转换；
 *       它只表示服务已经进入本采样周期的有限NEW轮询窗口。
 *       每个采样周期只消费一次故障注入。
 */
static void sensor_service_process_als(rt_tick_t now)
{
    sensor_device_runtime_t *runtime;
    sensor_als_sample_t sample = {0};

    rt_tick_t start_tick;
    rt_tick_t operation_ticks;
    rt_err_t result;

    runtime =&g_sensor_service.runtime[SENSOR_DEVICE_ALS];

    /*
     * IDLE表示当前没有正在等待的ALS样本周期。
     * 到达正常采样或重试期限后，建立新的有限NEW等待窗口。
     */
    if (runtime->wait_phase ==SENSOR_WAIT_PHASE_IDLE)
    {
        if (sensor_service_prepare_device(SENSOR_DEVICE_ALS,now) == RT_FALSE)
        {
            return;
        }

        /*
         * 每个ALS采样周期只消费一次软件故障。
         * 进入READY后的多次NEW检查属于同一事务，不重复消费。
         */
        result =sensor_service_consume_fault(SENSOR_DEVICE_ALS);

        if (result != RT_EOK)
        {
            /*
             * 注入故障发生在真实寄存器访问之前，
             * 因此本次实际操作耗时记为0 tick。
             */
            sensor_service_record_failure(SENSOR_DEVICE_ALS,
                                            result,
                                            0U);

            return;
        }

        /*
         * READY表示服务开始等待本周期的新ALS数据，
         * 不表示软件向LTR303发送了测量触发命令。
         */
        runtime->wait_phase =SENSOR_WAIT_PHASE_READY;

        /* 首次NEW检查可以在当前调度轮立即执行。 */
        runtime->next_poll_tick = now;

        /*
         * 为本周期建立总等待上界。
         * 到期后仍没有NEW将按超时进入统一恢复路径。
         */
        runtime->ready_deadline_tick =now +sensor_service_ms_to_tick(SENSOR_SERVICE_ALS_READY_TIMEOUT_MS);
    }

    /* 尚未到下一次NEW检查时刻，本轮不访问I2C。 */
    if (sensor_service_tick_reached(now,runtime->next_poll_tick) == RT_FALSE)
    {
        return;
    }

    start_tick = rt_tick_get();

    /*
     * try接口只检查一次NEW并在就绪时成对读取CH0/CH1，
     * 不在驱动内部循环或延时。
     */
    result =ltr303als01_try_read_sample(&sample);

    operation_ticks =(rt_tick_t)(rt_tick_get() - start_tick);

    /*
     * 只有NEW出现、双通道读取成功且样本有效时，
     * 才结束当前等待窗口并发布新快照。
     */
    if ((result == RT_EOK) &&(sample.valid == RT_TRUE))
    {
        runtime->wait_phase = SENSOR_WAIT_PHASE_IDLE;

        (void)sensor_service_commit_sample(
            SENSOR_DEVICE_ALS,
            &sample,
            operation_ticks);

        /*
        * LTR303处于自由连续测量模式，成功读取NEW后说明软件已经重新
        * 对齐到器件当前更新相位。
        *
        * 下一轮提前一个轮询间隔开始检查，使第一次检查位于预计NEW
        * 到达之前；若数据尚未更新，后续仍按10 ms有限轮询。
        *
        * 这里不沿用旧的绝对期限，避免把正常等待硬件NEW的时间误记为
        * 软件漏采。该调度仍然有250 ms等待上界，不会形成无限等待。
        */
        runtime->next_due_tick =rt_tick_get() +runtime->period_ticks -
                    sensor_service_ms_to_tick(SENSOR_SERVICE_ALS_POLL_MS);

        return;
    }

    /*
     * -RT_EBUSY表示当前没有NEW数据，属于正常等待，
     * 不增加失败或连续失败计数。
     */
    if (result == -RT_EBUSY)
    {

        now = rt_tick_get();

        /*
         * 到达本轮NEW等待期限后转为超时，
         * record_failure会把wait_phase恢复为IDLE并安排退避。
         */
        if (sensor_service_tick_reached(now,runtime->ready_deadline_tick) ==RT_TRUE)
        {
            sensor_service_record_failure(
                SENSOR_DEVICE_ALS,
                -RT_ETIMEOUT,
                operation_ticks);
        }
        else
        {
            /*
            * 本次NEW检查属于真实驱动调用。
            * 尚未到总期限时只记录not-ready并安排下一次检查。
            */
            sensor_service_record_not_ready(SENSOR_DEVICE_ALS,operation_ticks);
            /*
             * 尚未超时时只安排下一次单次检查，
             * 让唯一sensor线程继续服务IMU和MMC。
             */
            runtime->next_poll_tick =now +sensor_service_ms_to_tick(SENSOR_SERVICE_ALS_POLL_MS);
        }

        return;
    }

    /*
     * 驱动返回RT_EOK但样本无效，说明NEW虽然出现，
     * 器件状态、增益或原始数据没有满足发布契约。
     * 该样本不能覆盖last-good快照。
     */
    if (result == RT_EOK)
    {
        result = -RT_EIO;
    }

    /*
     * 总线错误、状态异常和无效样本统一进入失败路径，
     * 由连续失败阈值决定短期重试或离线重探。
     */
    sensor_service_record_failure(
        SENSOR_DEVICE_ALS,
        result,
        operation_ticks);
}
/*
 * 简介：按合并位图和最小通知间隔向上层发送sensor状态或快照变化通知。
 * 输入：无。
 * 返回值：无。
 * 说明：普通样本更新最多每500 ms通知一次；故障和恢复通过
 *       notify_urgent立即通知。回调必须在snapshot mutex之外执行，
 *       并且只能进行非阻塞的标量事件投递。
 */
static void sensor_service_notify_if_due(void)
{
    sensor_notify_handler_t handler;
    void *user_data;

    rt_uint32_t changed_mask;
    rt_uint32_t publish_seq;

    rt_tick_t now;
    rt_tick_t normal_due;
    rt_err_t result;

    /*
     * 没有任何器件样本或状态发生变化时直接返回。
     * pending mask由sensor线程持续OR合并，因此不会为每个样本
     * 单独执行一次通知。
     */
    if (g_sensor_service.notify_pending_mask == 0U)
    {
        return;
    }

    /*
     * 通知回调只允许在服务启动前注册，服务运行期间不会改变。
     * 这里复制回调及其上下文，后续调用时不需要持有snapshot mutex。
     */
    handler = g_sensor_service.notify_handler;
    user_data = g_sensor_service.notify_user_data;

    /*
     * 没有注册消费者时不需要保留通知位图。
     * 最新样本和状态仍然保存在snapshot中，不受通知缺失影响。
     */
    if (handler == RT_NULL)
    {
        g_sensor_service.notify_pending_mask = 0U;
        g_sensor_service.notify_urgent = RT_FALSE;
        return;
    }

    now = rt_tick_get();

    /*
     * 普通通知的下一允许时刻基于上一次实际尝试通知的tick计算。
     * 使用绝对期限可以避免通知执行时间逐渐累积成周期漂移。
     */
    normal_due =g_sensor_service.last_notify_tick +sensor_service_ms_to_tick(SENSOR_SERVICE_NOTIFY_MIN_MS);

    /*
     * 普通样本更新尚未达到最小通知间隔时继续合并位图。
     * 故障、离线或恢复等紧急状态变化通过notify_urgent绕过限频。
     */
    if ((g_sensor_service.notify_urgent == RT_FALSE) &&
        (sensor_service_tick_reached(now,normal_due) == RT_FALSE))
    {
        return;
    }

    /*
     * publish_seq和pending mask需要从同一个快照提交时刻取得。
     * 锁内只复制标量并更新通知控制状态，不执行回调。
     */
    if (sensor_service_lock_snapshot() != RT_EOK)
    {
        /*
         * 获取锁失败时保留pending mask和urgent状态。
         * sensor线程下次循环仍会再次尝试通知。
         */
        return;
    }

    changed_mask =g_sensor_service.notify_pending_mask;

    publish_seq =g_sensor_service.snapshot.publish_seq;

    /*
     * 当前合并通知即将发送，先清除已经取出的待通知状态。
     * sensor线程是这些控制字段的唯一写入者。
     */
    g_sensor_service.notify_pending_mask = 0U;
    g_sensor_service.notify_urgent = RT_FALSE;
    g_sensor_service.last_notify_tick = now;

    (void)rt_mutex_release(g_sensor_service.snapshot_mutex);

    /*
     * 回调必须在snapshot mutex之外执行，防止：
     * 1. 下游回调反向读取快照产生死锁；
     * 2. 慢消费者长期阻塞传感器快照发布；
     * 3. app/UI事件投递时间扩大快照临界区。
     *
     * 回调只能非阻塞地投递changed_mask和publish_seq等标量，
     * 不能直接操作I2C、LVGL对象或服务内部快照。
     */
    result =handler(changed_mask,publish_seq,user_data);

    /*
     * 回调完成后重新获取mutex，只记录通知结果。
     * 回调失败不会撤销已经发布的快照。
     */
    if (sensor_service_lock_snapshot() != RT_EOK)
    {
        return;
    }

    if (result == RT_EOK)
    {
        g_sensor_service.stats.notify_count++;
    }
    else
    {
        /*
         * 下游拒绝、队列已满或其他错误只计入通知丢弃统计。
         * 最新快照仍可由消费者主动调用get_snapshot()取得。
         */
        g_sensor_service.stats.notify_drop_count++;
    }

    (void)rt_mutex_release(g_sensor_service.snapshot_mutex);
}

/*
 * 简介：记录sensor线程最近一轮调度处理时间和历史最大处理时间。
 * 输入：loop_ticks为本轮从开始处理到进入event等待前经过的tick数。
 * 返回值：无。
 * 说明：该时间包含重探、器件操作、快照发布和通知处理，
 *       但不包含线程在event中的睡眠时间。它用于判断一轮处理
 *       是否可能破坏最快的20 ms IMU调度周期。
 */
static void sensor_service_record_loop(rt_tick_t loop_ticks)
{
    /*
     * loop统计属于公开stats的一部分，必须与其他统计字段使用
     * 相同的snapshot mutex保护。
     */
    if (sensor_service_lock_snapshot() != RT_EOK)
    {
        /*
         * 获取锁失败时放弃本次loop统计，不阻塞sensor线程。
         * 锁失败已经由sensor_service_lock_snapshot()单独计数。
         */
        return;
    }

    /* 每完成一轮调度处理，循环总数增加一次。 */
    g_sensor_service.stats.loop_count++;

    /* 保存最近一轮实际占用的tick数。 */
    g_sensor_service.stats.last_loop_ticks =loop_ticks;

    /*
     * 历史最大值只增不减，用于发现初始化、重探或错误处理中的
     * 异常慢路径。
     */
    if (loop_ticks >g_sensor_service.stats.max_loop_ticks)
    {
        g_sensor_service.stats.max_loop_ticks =loop_ticks;
    }

    (void)rt_mutex_release(g_sensor_service.snapshot_mutex);
}

/*
 * 简介：记录sensor线程从event等待中返回的累计次数。
 * 输入：无。
 * 返回值：无。
 * 说明：event等待可能因为采样期限超时返回，也可能因为重探请求
 *       或故障注入控制事件而提前返回。wake_count只表示唤醒次数，
 *       不等同于样本数量或外部事件数量。
 */
static void sensor_service_record_wake(void)
{
    /*
     * wake_count属于公开stats，需要在snapshot mutex内更新，
     * 防止消费者读取到撕裂的统计结构。
     */
    if (sensor_service_lock_snapshot() != RT_EOK)
    {
        /*
         * 统计失败不能影响下一轮传感器调度。
         * 锁失败次数由独立计数器记录。
         */
        return;
    }

    g_sensor_service.stats.wake_count++;

    (void)rt_mutex_release(g_sensor_service.snapshot_mutex);
}

/*
 * 简介：计算sensor线程下一次event等待允许持续的tick数。
 * 输入：now为本轮处理结束、准备进入event等待时的当前tick。
 * 返回值：距离最近调度期限的等待tick数，最小返回1 tick。
 * 说明：分别检查三颗器件的ready轮询、失败重试和正常采样期限，
 *       选择最早到期者；最长睡眠限制为IMU的20 ms周期。
 *       event仍可因重探或控制请求提前唤醒线程。
 */
static rt_tick_t sensor_service_calculate_wait(rt_tick_t now)
{
    rt_tick_t wait_ticks;
    rt_tick_t deadline;
    rt_tick_t delta;
    rt_uint32_t index;

    /*
     * 先将最长睡眠限制为IMU正常周期。
     * 即使当前只有ALS等慢速器件可用，线程也不会一次睡眠超过
     * 约20 ms；event请求仍然可以提前结束等待。
     */
    wait_ticks =sensor_service_ms_to_tick(SENSOR_SERVICE_IMU_PERIOD_MS);

    /*
     * 每颗器件都有独立的runtime，因此分别选择它当前真正生效的
     * 下一期限，再从三颗器件中取最近者。
     */
    for (index = 0U;index < SENSOR_DEVICE_COUNT;index++)
    {
        sensor_device_runtime_t *runtime =&g_sensor_service.runtime[index];

        if (runtime->wait_phase == SENSOR_WAIT_PHASE_READY)
        {
            /*
             * 当前存在尚未完成的非阻塞采样事务。
             *
             * MMC：等待下一次Meas_m_done检查；
             * ALS：等待下一次ALS_STATUS.NEW检查。
             *
             * WAIT_READY优先级最高，因为同一事务必须在
             * ready_deadline_tick之前继续推进。
             */
            deadline = runtime->next_poll_tick;
        }
        else if ((runtime->state ==SENSOR_DEVICE_STATE_TEMP_FAILED) ||
                 (runtime->state ==SENSOR_DEVICE_STATE_OFFLINE))
        {
            /*
             * 器件处于失败状态且当前没有等待中的采样事务。
             *
             * TEMP_FAILED使用约1秒的受控重试期限；
             * OFFLINE使用约5秒的重探期限。
             *
             * 失败期间不能继续使用正常next_due_tick紧密访问总线。
             */
            deadline = runtime->retry_due_tick;
        }
        else
        {
            /*
             * 器件正常运行或正在等待首份有效样本时，
             * 使用正常绝对采样期限。
             *
             * IMU：下一次状态检查和六轴burst期限；
             * MMC：下一次单次测量触发期限；
             * ALS：下一次进入NEW检查窗口的期限。
             */
            deadline = runtime->next_due_tick;
        }

        /*
         * 任一器件期限已经到达或逾期时，返回1 tick。
         * 不能返回0，否则event_recv可能形成没有阻塞的忙循环。
         *
         * 这里的1表示一个系统tick，不一定等于1 ms。
         */
        if (sensor_service_tick_reached(now,deadline) == RT_TRUE)
        {
            return 1U;
        }

        /*
         * 当前期限仍在未来，计算距离它剩余的tick数。
         * 只有在确认deadline未到期后才执行无符号减法。
         */
        delta = (rt_tick_t)(deadline - now);

        /*
         * 保存三颗器件全部候选期限中的最小值。
         * sensor线程将在最近期限到来时重新运行调度循环。
         */
        if (delta < wait_ticks)
        {
            wait_ticks = delta;
        }
    }

    /*
     * 理论上前面的到期检查已避免0，但保留最后防御：
     * 永远至少阻塞1 tick，避免异常配置导致忙循环。
     */
    return (wait_ticks == 0U)
        ? 1U
        : wait_ticks;
}
/*
 * 简介：记录TEMP_FAILED退避到期后真正开始的一次受控重试。
 * 输入：device为目标器件。
 * 返回值：无。
 * 说明：
 * 1. 本函数只更新统计，不执行I2C、延时、日志或回调；
 * 2. 调用前必须由唯一sensor线程清除retry_count_pending；
 * 3. 获取快照锁失败时由现有锁超时统计记录，不能在下一轮重复计数。
 */
static void sensor_service_record_retry(sensor_device_id_t device)
{
    sensor_device_stats_t *stats;

    if (device >= SENSOR_DEVICE_COUNT)
    {
        return;
    }

    if (sensor_service_lock_snapshot() != RT_EOK)
    {
        return;
    }

    stats = sensor_service_get_device_stats_internal(device);

    if (stats != RT_NULL)
    {
        stats->retry_count++;
    }

    (void)rt_mutex_release(g_sensor_service.snapshot_mutex);
}
