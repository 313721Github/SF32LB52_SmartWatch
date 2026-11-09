#ifndef SMARTWATCH_MOTION_CORE_H
#define SMARTWATCH_MOTION_CORE_H

#include "motion_types.h"
#include "step_detector.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 纯C返回值，不依赖RT-Thread错误码。 */
typedef enum
{
    MOTION_CORE_OK = 0,                     // 核心操作成功
    MOTION_CORE_ERR_ARGUMENT = -1,          // 输入参数无效
    MOTION_CORE_ERR_STATE = -2,             // 当前状态不允许执行该操作
    MOTION_CORE_ERR_CLOCK = -3,             // 时钟参数或时间关系不满足约定
    MOTION_CORE_ERR_NOT_IMPLEMENTED = -4    // 当前功能尚未实现
} motion_core_result_t;

/*
 * 数据源状态由服务层根据sensor状态转换。
 * 核心不直接读取sensor快照，也不能把“没有步数”判断为离线。
 */
typedef enum
{
    MOTION_SOURCE_UNKNOWN = 0,  // 数据源状态尚未确定
    MOTION_SOURCE_AVAILABLE,    // 数据源可用
    MOTION_SOURCE_STALE,        // 数据源的最近样本已过期
    MOTION_SOURCE_OFFLINE       // 数据源已离线
} motion_source_state_t;

/*
 * 一份出队帧的最终分类。
 * 正常调用中只能选择一类，用于服务层消费统计闭合。
 */
typedef enum
{
    MOTION_FRAME_DISCARD_OLD_GENERATION = 0,    // 丢弃停启前遗留的旧运行代际帧
    MOTION_FRAME_DISCARD_STOPPED,               // 服务已停止，丢弃本帧
    MOTION_FRAME_DISCARD_INVALID,               // 样本无效，丢弃本帧
    MOTION_FRAME_DISCARD_STALE,                 // 样本已过期，丢弃本帧
    MOTION_FRAME_DISCARD_CONFIG,                // 样本配置不符合要求，丢弃本帧
    MOTION_FRAME_DISCARD_DUPLICATE,             // 样本序号重复，丢弃本帧
    MOTION_FRAME_PASSED_TO_ALGORITHM            // 已送入算法，不代表产生步数或被有效接受
} motion_frame_disposition_t;

/* 本次处理触发的连续性重建原因，可按位组合。 */
#define MOTION_RESET_NONE          UINT32_C(0)         // 无连续性重建原因
#define MOTION_RESET_SEQUENCE_GAP  (UINT32_C(1) << 0)  // 样本序号存在正向缺口
#define MOTION_RESET_TIME_GAP      (UINT32_C(1) << 1)  // 样本时间间隔超过允许范围
#define MOTION_RESET_ROLLBACK      (UINT32_C(1) << 2)  // 样本序号或单调时间异常倒退
#define MOTION_RESET_INVALID       (UINT32_C(1) << 3)  // 遇到无效样本
#define MOTION_RESET_STALE         (UINT32_C(1) << 4)  // 遇到过期样本
#define MOTION_RESET_CONFIG        (UINT32_C(1) << 5)  // 样本配置不符合算法要求
#define MOTION_RESET_SOURCE        (UINT32_C(1) << 6)  // 数据源状态变化，需要重建连续性
#define MOTION_RESET_STOP          (UINT32_C(1) << 7)  // 运动处理停止，中断当前连续性

/*
 * 单帧处理结果，供服务层诊断/有限数据导出使用。
 * 未调用算法时，algorithm_called=false，对应算法输出不能解释为有效。
 */
typedef struct
{
    motion_frame_disposition_t disposition; // 本帧最终处理分类
    uint32_t reset_reasons;                 // 本次连续性重建原因的位组合

    bool algorithm_called[MOTION_ALGORITHM_COUNT]; // 各算法本次是否实际被调用
    step_detector_process_result_t algorithm_result[MOTION_ALGORITHM_COUNT]; // 各算法的处理返回结果
    step_detector_output_t algorithm_output[MOTION_ALGORITHM_COUNT];         // 各算法的本次输出，结合调用标志和返回结果解释

    bool fatal_error; // 内部算法错误等；服务层应关闭样本接纳并发布故障
} motion_core_frame_result_t;

/*
 * 纯核心拥有的累计统计。
 * 不包含生产入队、控制队列、RT线程和真实执行耗时统计。
 */
typedef struct
{
    uint64_t dequeued;               // 交给核心处理的出队帧总数
    uint64_t discard_old_generation; // 因运行代际过旧而丢弃的帧数
    uint64_t discard_stopped;        // 因运动处理已停止而丢弃的帧数
    uint64_t discard_invalid;        // 因样本无效而丢弃的帧数
    uint64_t discard_stale;          // 因样本过期而丢弃的帧数
    uint64_t discard_config;         // 因配置不匹配而丢弃的帧数
    uint64_t discard_duplicate;      // 因样本重复而丢弃的帧数
    uint64_t passed_to_alg;          // 通过前置检查并送入算法阶段的帧数

    uint64_t seq_gap_events;         // 检测到正向序号缺口的次数
    uint64_t seq_missing_samples;    // 根据正向序号缺口累计的缺失样本数
    uint64_t seq_rollback_events;    // 检测到样本序号异常倒退的次数
    uint64_t time_gap_events;        // 检测到样本时间间隔过大的次数
    uint64_t source_offline_events;  // 数据源进入离线状态的次数
    uint64_t source_recovery_events; // 数据源恢复可用的次数
    uint64_t calendar_change_events; // 按核心约定识别到的日历变化次数
    uint64_t day_bucket_evictions;   // 日桶容量不足时淘汰旧记录的次数

    motion_algorithm_stats_t algorithm[MOTION_ALGORITHM_COUNT]; // 各算法的独立累计统计
    bool counters_saturated; // 统计计数曾达到上限的锁存标志
} motion_core_stats_t;

/*
 * 核心实例由调用者分配。
 *
 * 正式业务只有运动worker能够修改本实例；
 * PC测试/诊断使用另外的局部或静态实例，不能注入正式业务实例。
 * 结构体公开是为了静态分配和确定性测试，不授权UI/BLE直接修改。
 */
typedef struct
{
    bool initialized; // 核心是否已完成初始化
    uint32_t tick_hz; // 输入单调tick的频率，单位：次/秒

    step_detector_t detector[MOTION_ALGORITHM_COUNT]; // 各算法独立的内部上下文

    motion_snapshot_t snapshot; // 核心维护的业务结果，供服务层复制发布
    motion_core_stats_t stats;  // 核心维护的累计统计

    motion_source_state_t source_state; // 最近一次由服务层提供的数据源状态

    /* 当前运行代际内的输入连续性基准。 */
    bool has_previous_seq;       // 是否已建立有效的前序样本序号基准
    uint32_t previous_imu_seq;   // 上一次用于连续性检查的IMU序号

    bool has_previous_tick;         // 是否已建立有效的前序采样tick基准
    uint32_t previous_sample_tick;  // 上一次用于连续性检查的采样tick

    /* 日桶按有限轮转顺序淘汰，不通过日历大小判断“最旧”。 */
    uint32_t next_evict_index; // 下一次轮转淘汰的日桶索引
    bool history_evicted;     // 本次启动是否曾淘汰日桶记录

    /* 缺样等恢复期，持续到新输入完成预热。 */
    bool recovering_from_gap; // 是否处于输入中断后的连续性恢复期
    bool input_timed_out;     // 当前是否已判定输入超时

    /* 各算法本阶段是否已通过真实样本完成预热；reset后清除。 */
    bool warmup_completed[MOTION_ALGORITHM_COUNT];

    /* 达到业务计数上限后锁存，防止回绕或破坏累计关系。 */
    bool counting_blocked[MOTION_ALGORITHM_COUNT]; // 各算法是否已禁止继续累加业务步数

    /*
     * 由调用者提供的32位now_tick扩展成内部64位相对单调时间。
     * 不读系统时钟，不使用日历时间。
     * 相邻推进间隔必须小于32位tick半区间。
     */
    uint32_t previous_now_tick; // 上一次推进核心时间时传入的32位tick
    uint64_t monotonic_ticks;   // 内部累计的64位相对单调tick

    uint64_t session_started_at_ticks; // 当前会话开始时的内部64位单调tick

    /*
     * 用处理完成时的扩展时间加初始样本年龄计算长期age，
     * 避免长期无输入时，32位tick整圈回绕后age突然变小。
     */
    uint64_t last_processed_at_ticks;           // 最近一次算法处理完成时的内部64位单调tick
    uint32_t last_processed_initial_age_ticks;  // 该次处理完成时，输入样本已有的年龄，单位：tick
} motion_core_t;

//公开函数声明
int motion_core_init(motion_core_t *core,uint32_t tick_hz,
                     motion_algorithm_id_t selected,uint32_t now_tick);     //建立确定的核心初始状态

int motion_core_start(motion_core_t *core,uint32_t run_generation,
                      uint32_t session_id,uint32_t now_tick);               //开始新会话，保留本次启动的长期累计

int motion_core_stop(motion_core_t *core, uint32_t now_tick);               //停止算法消费并保留最终会话计数

int motion_core_update_calendar(motion_core_t *core,const motion_calendar_tag_t *calendar,
                                uint32_t now_tick);                         //更新当前日历视图，不改变样本流和算法阶段

int motion_core_update_source(motion_core_t *core,motion_source_state_t source,
                              uint32_t now_tick);                           //处理数据源可用性变化

int motion_core_process(motion_core_t *core,const motion_sample_frame_t *frame,
                        uint32_t now_tick,motion_core_frame_result_t *result);  //对一份出队帧完成唯一分类和双算法处理

int motion_core_tick(motion_core_t *core, uint32_t now_tick);               //无新样本时检测输入超时并维护恢复状态

int motion_core_get_snapshot(const motion_core_t *core,uint32_t now_tick,
                             motion_snapshot_t *snapshot);                  //复制核心摘要并计算当前样本年龄

int motion_core_get_stats(const motion_core_t *core,motion_core_stats_t *stats);//复制纯核心累计统计

#ifdef __cplusplus
}
#endif

#endif /* SMARTWATCH_MOTION_CORE_H */