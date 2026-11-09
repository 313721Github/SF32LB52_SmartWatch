#ifndef SMARTWATCH_MOTION_TYPES_H
#define SMARTWATCH_MOTION_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 运动业务类型/数据格式版本，不代替算法版本和算法参数版本。 */
#define MOTION_FORMAT_VERSION             1U

/* B0、B1分别保存状态和统计，不能让两个算法共用一个context。 */
#define MOTION_ALGORITHM_COUNT            2U

/* 有限日桶数量；超过容量后必须登记淘汰和不完整历史。 */
#define MOTION_DAY_HISTORY_CAPACITY       8U

/*
 * 初始候选策略：
 * 排队后样本年龄超过250 ms，不再继续作为正常实时输入。
 * 这是运动业务的新鲜度门限，不修改算法已有100 ms时间缺口门限。
 * 正式场景采集前确认并冻结，不能看验证结果后悄悄调整。
 */
#define MOTION_INPUT_STALE_MS             250U

/*
 * 算法标识也作为固定数组下标。
 * 后续转换到step_detector_algorithm_t时显式映射，
 * 不依赖两个不同枚举恰好具有相同数值。
 */
typedef enum
{
    MOTION_ALGORITHM_B0 = 0,
    MOTION_ALGORITHM_B1,
    MOTION_ALGORITHM_INVALID
} motion_algorithm_id_t;

/*
 * 服务生命周期与数据质量分开描述。
 * RUNNING只表示业务会话正在运行，不表示数据一定连续、有效。
 */
typedef enum
{
    MOTION_SERVICE_BOOT = 0,     // 资源或worker尚未准备完成
    MOTION_SERVICE_STOPPED,      // 已准备，但不向算法送入新样本
    MOTION_SERVICE_RUNNING,      // 当前会话正在处理样本
    MOTION_SERVICE_FAULT         // 服务内部故障，不能继续正常处理
} motion_service_state_t;

/*
 * 当前质量标志，可以按位组合。
 * quality_flags表示当前问题；latched_quality_flags表示本次启动曾发生。
 * 当前问题解除不能抹掉累计统计和历史锁存标志。
 */
#define MOTION_QUALITY_NONE                UINT32_C(0)         // 无质量异常标记
#define MOTION_QUALITY_WARMUP              (UINT32_C(1) << 0)  // 算法预热中
#define MOTION_QUALITY_SAMPLE_GAP          (UINT32_C(1) << 1)  // 样本序列存在缺口
#define MOTION_QUALITY_INPUT_STALE         (UINT32_C(1) << 2)  // 输入样本已过期
#define MOTION_QUALITY_SOURCE_OFFLINE      (UINT32_C(1) << 3)  // 传感器数据源离线
#define MOTION_QUALITY_CONFIG_MISMATCH     (UINT32_C(1) << 4)  // 输入配置与算法要求不匹配
#define MOTION_QUALITY_TIME_UNASSIGNED     (UINT32_C(1) << 5)  // 步数尚未归属日历日期
#define MOTION_QUALITY_HISTORY_PARTIAL     (UINT32_C(1) << 6)  // 对应日期的历史计数不完整
#define MOTION_QUALITY_COUNTER_SATURATED   (UINT32_C(1) << 7)  // 计数已达上限，停止累加

/*
 * sensor交给运动服务的一份完整IMU值副本。
 *
 * 不包含sensor内部指针、设备句柄、mutex或LVGL对象。
 * 原始值供导出和交叉检查使用，当前计步算法主要使用accel_ug。
 * 所有数组下标固定为：0=X，1=Y，2=Z。
 */
typedef struct
{
    uint32_t sample_tick;        // 本次完整IMU读取完成tick，不是投递时刻
    uint32_t imu_seq;            // IMU成功提交序号，不是总快照publish_seq
    uint32_t config_version;    // 产生本样本时的IMU配置版本

    int16_t accel_raw[3];        // 三轴加速度原始寄存器值
    int16_t gyro_raw[3];         // 三轴角速度原始寄存器值

    int32_t accel_ug[3];         // 三轴加速度，单位：微g
    int32_t gyro_udps[3];        // 三轴角速度，单位：微度/秒

    uint8_t status_reg;         // 本次驱动取得的IMU状态寄存器值
    bool valid;                // 是否为完整有效样本，不能用last-good冒充新样本
} motion_imu_sample_t;

/*
 * 样本生产/投递时附加的日历标签。
 *
 * day_valid只能来自当时发布的可信、新鲜时间快照。
 * 它表示软件可确认的日历归属，不承诺精确对应物理午夜。
 * 后续校时不能回头修改已经入队样本的标签。
 */
typedef struct
{
    uint32_t local_day_key;        // YYYYMMDD；day_valid=false时不得用于归日
    uint32_t calendar_generation;  // 时间服务的日历代际，不是运动运行代际
    uint32_t day_seq;              // 时间服务日期变化序号，用于追踪
    bool day_valid;                // 当时是否具备可用于日计数的可信日期
} motion_calendar_tag_t;

/*
 * 专用样本队列中的值类型。
 *
 * imu.sample_tick用于算法dt和样本年龄。
 * enqueue_tick用于测量排队时间。
 * run_generation用于隔离停止/重新启动前后的旧样本。
 */
typedef struct
{
    motion_imu_sample_t imu;          // 本帧完整IMU样本副本
    motion_calendar_tag_t calendar;   // 本帧携带的日历归属与时间可信信息

    uint32_t enqueue_tick;           // 入队时的单调tick，用于计算排队耗时
    uint32_t run_generation;         // 运动运行代际，用于识别停启前遗留的旧帧
} motion_sample_frame_t;

/*
 * 每个算法分别维护业务累计。
 *
 * 这些计数独立于step_detector_t.step_count：
 * 重置算法滤波/预热状态时，不能清除这里的累计。
 * 全部为RAM计数，本阶段重新启动固件后清零。
 */
typedef struct
{
    uint64_t boot_steps;           // 本次系统启动以来累计事件数
    uint64_t session_steps;        // 当前或最近一次会话累计事件数
    uint64_t unassigned_steps;     // 事件产生时没有可信日期的累计数
    uint64_t archived_day_steps;   // 已淘汰日桶中的累计数，仅保留总量
} motion_algorithm_totals_t;

/*
 * 一个有限日桶。
 *
 * 同一日期在保留期间被再次访问时继续累加，不重复创建。
 * complete=false表示该日期历史可能已被淘汰，当前只有部分记录。
 */
typedef struct
{
    uint32_t local_day_key;                  // YYYYMMDD
    uint64_t steps[MOTION_ALGORITHM_COUNT];  // B0、B1分别记录

    bool used;                              // 当前槽位是否有效
    bool complete;                          // 是否完整保留本次启动内该日的已归日记录
} motion_day_bucket_t;

/*
 * 外部可取得的运动业务快照。
 *
 * 由运动worker组织并发布，读取者获得值副本。
 * 不暴露算法context、队列或内部可修改指针。
 */
typedef struct
{
    uint32_t format_version;
    uint32_t publish_seq;       // 运动摘要发布序号，可自然回绕
    uint32_t published_tick;    // 本次摘要发布时的单调tick

    motion_service_state_t state;
    motion_algorithm_id_t selected_algorithm; // 初始候选B1，B0作为对照

    uint32_t session_id;        // 0表示尚未建立会话
    uint32_t run_generation;    // 控制样本接纳的运行代际，与日历代际独立

    uint32_t quality_flags;          // 当前存在的问题
    uint32_t latched_quality_flags;  // 本次启动曾发生的问题

    /*
     * 以下日期来自worker最近检查的时间服务快照。
     * 用于展示“现在是哪一天”，不能被旧队列帧中的日期覆盖。
     */
    uint32_t current_day_key;
    uint32_t calendar_generation;
    bool current_day_valid;

    /*
     * 当前日期在保留日桶中的计数副本。
     * current_day_valid=false时不得显示成可信的“今日步数”。
     * current_day_complete=false时应明确是部分记录。
     */
    uint64_t current_day_steps[MOTION_ALGORITHM_COUNT];
    bool current_day_complete;

    motion_algorithm_totals_t totals[MOTION_ALGORITHM_COUNT];
    motion_day_bucket_t day_history[MOTION_DAY_HISTORY_CAPACITY];

    /*
     * 已消费与已送入算法是不同进展。
     * 丢弃过期帧也算消费，但不能算有效算法进展。
     * 对应has_*为false时，tick/seq字段不得单独解释。
     */
    uint32_t last_dequeued_seq;
    uint32_t last_dequeued_tick;
    bool has_dequeued;

    uint32_t last_processed_seq;
    uint32_t last_processed_tick; // 最近实际送入算法的样本sample_tick
    uint32_t processed_age_ms;    // 取得快照时计算，无样本时结合has_processed判断
    bool has_processed;

    uint32_t worker_progress_tick; // 最近完成一次有限工作单元，不单独代表健康
    int32_t last_error;            // 最近一次服务错误，0表示无错误
} motion_snapshot_t;

/*
 * 单个算法的服务级累计处理统计。
 *
 * 公共step_detector_reset()会清算法内部统计，
 * 因此长期统计在服务层累计，不直接把context中的计数当全生命周期计数。
 */
typedef struct
{
    uint64_t process_calls;
    uint64_t accepted_no_event;
    uint64_t events;
    uint64_t ignored;
    uint64_t errors;
    uint64_t reset_events;
} motion_algorithm_stats_t;

/*
 * 服务统计副本。
 *
 * 所有累计量从本次服务初始化开始保存，普通停启和算法reset不清零。
 * 计数达到上限时饱和并登记，不能静默回绕。
 */
typedef struct
{
    uint32_t captured_tick;       // 统计副本采集时刻
    bool accounting_base_ready; // 仅版本/在途基础条件就绪，不代替最终发布保证
    bool accounting_stable;      // 是否取得可直接用于跨组闭合检查的一致窗口
    bool counters_saturated;     // 任意累计统计是否达到饱和值

    uint32_t init_attempts;
    uint32_t init_failures;

    /* 控制请求：成功投递不代表请求已执行。 */
    uint64_t control_post_attempts;
    uint64_t control_post_accepted;
    uint64_t control_post_rejected;
    uint64_t control_completed;

    /*
     * 生产侧由sensor调用路径更新。
     * produced只统计资源就绪后、参数有效的完整提交尝试。
     * 空指针等接口误用单独计数，不进入produced闭合式。
     */
    uint64_t produced;
    uint64_t disabled_skipped;       // 已停止或接纳门关闭，未入样本队列
    uint64_t enqueue_ok;
    uint64_t enqueue_reject_full;
    uint64_t enqueue_reject_other;
    uint64_t invalid_submit_calls;

    /*
     * 消费侧分类必须互斥：每份dequeued样本只能落入其中一类。
     * passed_to_alg表示该帧分别交给B0、B1处理，不表示一定产生事件。
     */
    uint64_t dequeued;
    uint64_t discard_old_generation;
    uint64_t discard_stopped;
    uint64_t discard_invalid;
    uint64_t discard_stale;
    uint64_t discard_config;
    uint64_t discard_duplicate;
    uint64_t discard_internal; // 已出队，但核心调用在完成分类前报内部错误
    uint64_t discard_injected; // 有限诊断丢弃，已出队但未调用核心
    uint64_t passed_to_alg;

    /* 放在服务诊断统计附近。 */
    uint64_t wake_failures;               // 队列操作后唤醒事件发送失败
    uint64_t publish_failures;            // 未能发布本轮一致副本
    uint64_t sensor_status_read_failures; // 获取sensor状态副本失败，不等同硬件离线

    /* 连续性与恢复统计，不与上面的消费分类相加。 */
    uint64_t seq_gap_events;          // 观察到前向序号缺口的次数
    uint64_t seq_missing_samples;     // 按有效前向序号差推算的缺失帧数
    uint64_t seq_rollback_events;     // 非正常序号倒退次数，不含正常回绕
    uint64_t time_gap_events;         // 相邻样本时间缺口超过算法门限的次数
    uint64_t source_offline_events;
    uint64_t source_recovery_events;
    uint64_t calendar_change_events;
    uint64_t day_bucket_evictions;

    motion_algorithm_stats_t algorithm[MOTION_ALGORITHM_COUNT];

    /*
     * outstanding_frames为已入队但尚未计入dequeued的逻辑帧数，
     * 可能包括worker已经取出、尚未完成消费记账的帧。
     * 它不直接等于RT消息队列当前entry。
     */
    uint32_t outstanding_frames;
    uint32_t queue_high_water;        // 实际消息队列驻留帧数的历史峰值
    uint32_t submit_inflight;         // 正在完成投递及记账的调用数
    uint32_t receive_inflight;        // 正在完成出队记账的帧数

    /* 最大值用于验收，不能只记录平均耗时。单位均为系统tick。 */
    uint32_t max_submit_ticks;
    uint32_t max_queue_wait_ticks;
    uint32_t max_sample_age_ticks;
    uint32_t max_algorithm_ticks;

    uint32_t worker_progress_tick;
    uint32_t effective_progress_tick; // 最近完成一份有效算法输入的时刻
} motion_service_stats_t;

/* 异步业务停启与有限诊断控制；不开放运行时修改冻结算法参数。 */
typedef enum
{
    MOTION_REQUEST_START = 0,
    MOTION_REQUEST_STOP,
    MOTION_REQUEST_TEST, // 有限诊断配置，与普通停启共用请求队列
    MOTION_REQUEST_TRACE_ARM,
    MOTION_REQUEST_TRACE_CANCEL,
    MOTION_REQUEST_TRACE_CLEAR
} motion_request_kind_t;

typedef enum
{
    MOTION_RESULT_PENDING = 0,
    MOTION_RESULT_RUNNING,
    MOTION_RESULT_OK,
    MOTION_RESULT_FAILED
} motion_result_state_t;

/*
 * 有限结果槽中的值副本。
 * request_id=0保留为无效ID；PENDING/RUNNING记录不得被回收。
 */
typedef struct
{
    uint32_t request_id;
    motion_request_kind_t kind;
    motion_result_state_t state;
    int32_t error;

    uint32_t session_id;
    uint32_t run_generation;
    uint32_t finished_tick;     // 仅终态有效
} motion_request_result_t;


/* 诊断默认关闭；不修改硬件、采样参数或算法冻结配置。 */
#define MOTION_TEST_MAX_PAUSE_MS       500U
#define MOTION_TEST_MAX_DROP_FRAMES     16U
#define MOTION_TEST_ARM_TTL_MS        2000U

typedef enum
{
    MOTION_TEST_NONE = 0,
    MOTION_TEST_PAUSE_CONSUMER,
    MOTION_TEST_DROP_FRAMES,
    MOTION_TEST_FINAL_PUBLISH
} motion_test_fault_t;

typedef enum
{
    MOTION_TEST_END_NONE = 0,
    MOTION_TEST_END_COMPLETED,
    MOTION_TEST_END_EXPIRED,
    MOTION_TEST_END_CANCELLED,
    MOTION_TEST_END_FAULT
} motion_test_end_t;

/* worker维护并复制到发布区；clear不清累计证据。 */
typedef struct
{
    motion_test_fault_t kind;
    bool active;
    motion_test_end_t end_reason;
    uint32_t request_id;
    uint32_t argument;
    uint32_t remaining;       // PAUSE为剩余ms，DROP为剩余帧，FINAL为剩余次数
    uint32_t started_tick;
    uint32_t deadline_tick;
    uint32_t published_tick;
    uint64_t pause_activations;
    uint64_t dropped_frames;
    uint64_t final_publish_failures;
    uint64_t expirations;
    uint64_t cancellations;
    bool witness_valid;
    int32_t witness_read_error;
    bool witness_accounting_stable;
    bool witness_other_conditions; // 来自同一次真实getter的基础条件
    bool witness_same_revision;
    uint64_t witness_dequeued_delta;
    bool counters_saturated;
} motion_test_status_t;

#ifdef __cplusplus
}
#endif

#endif /* SMARTWATCH_MOTION_TYPES_H */
