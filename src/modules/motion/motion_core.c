#include "motion_core.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#define MOTION_CORE_INVALID_DAY_INDEX UINT32_MAX
#define MOTION_CORE_TICK_HALF_RANGE    UINT32_C(0x80000000)

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

//私有函数声明
static int motion_core_advance_clock(motion_core_t *core,uint32_t now_tick);    //把连续提供的32位tick推进为内部64位相对时间

static uint32_t motion_core_ticks_to_ms(uint64_t ticks,uint32_t tick_hz);       //把内部tick数量换算为饱和毫秒值

static void motion_core_add_saturated(motion_core_t *core,uint64_t *counter,uint64_t increment);    //对一个核心累计统计执行饱和加法

static void motion_core_reset_algorithms(motion_core_t *core,uint32_t reasons); //清算法阶段，同时保留业务累计与长期统计

static bool motion_core_day_key_is_valid(uint32_t day_key);                     //校验显示日期键的年月日合法性

static uint32_t motion_core_find_or_allocate_day(motion_core_t *core,uint32_t day_key); //找到指定日期的日桶，必要时分配或淘汰

static void motion_core_record_event(motion_core_t *core,motion_algorithm_id_t algorithm,
                                                    const motion_calendar_tag_t *calendar);//将一个算法事件加入业务累计和对应日期

static void motion_core_refresh_summary(motion_core_t *core);                   //从核心状态刷新当前日期计数及质量摘要

/* ------------------------------ 公开函数 ------------------------------ */

/*
 * 简介：建立确定的核心初始状态。
 * 输入：core非NULL；tick_hz/selected合法；now_tick为初始化时刻。
 * 返回值：0成功；负值表示参数或算法初始化失败。
 * 说明：仅首次初始化清全实例；不能用本函数代替普通停启。
 */
int motion_core_init(motion_core_t *core,uint32_t tick_hz,
                     motion_algorithm_id_t selected,uint32_t now_tick)
{
    int b0_result;
    int b1_result;

    /* 所有外部参数先检查，检查通过前不能清空实例。 */
    if (core == NULL)
    {
        return MOTION_CORE_ERR_ARGUMENT;
    }

    if ((selected != MOTION_ALGORITHM_B0) && (selected != MOTION_ALGORITHM_B1))
    {
        return MOTION_CORE_ERR_ARGUMENT;
    }

    /*
     * 时间单位必须与两份冻结配置一致。
     * 不通过修改冻结配置来适配其他tick频率。
     */
    if ((tick_hz == 0U) ||
        (tick_hz != STEP_DETECTOR_CONFIG_B0_TASK10_V1.tick_hz) ||
        (tick_hz != STEP_DETECTOR_CONFIG_B1_TASK10_V1.tick_hz))
    {
        return MOTION_CORE_ERR_ARGUMENT;
    }

    /*
     * 参数合法后建立全零状态：
     * initialized=false，所有累计为0，日桶未使用，
     * 连续性基准无效，恢复和饱和标志均未设置。
     */
    memset(core, 0, sizeof(*core));

    /* 两个算法使用独立上下文，显式对应各自冻结配置。 */
    b0_result = step_detector_init(&core->detector[MOTION_ALGORITHM_B0],&STEP_DETECTOR_CONFIG_B0_TASK10_V1);

    b1_result = step_detector_init(&core->detector[MOTION_ALGORITHM_B1],&STEP_DETECTOR_CONFIG_B1_TASK10_V1);

    if ((b0_result != 0) || (b1_result != 0))
    {
        /*
         * initialized仍为false，调用者不得继续运行该实例。
         * 当前错误枚举没有独立的算法初始化错误，使用状态错误。
         */
        core->snapshot.state = MOTION_SERVICE_FAULT;
        core->snapshot.last_error = MOTION_CORE_ERR_STATE;

        return MOTION_CORE_ERR_STATE;
    }

    core->tick_hz = tick_hz;

    /* 初始化只建立时间原点，不累计初始化前的系统运行时间。 */
    core->previous_now_tick = now_tick;
    core->monotonic_ticks = UINT64_C(0);

    core->source_state = MOTION_SOURCE_UNKNOWN;

    core->snapshot.format_version = MOTION_FORMAT_VERSION;
    core->snapshot.selected_algorithm = selected;
    core->snapshot.state = MOTION_SERVICE_STOPPED;

    /* 尚未取得可信日期，不建立任何日期桶。 */
    core->snapshot.current_day_key = 0U;
    core->snapshot.current_day_valid = false;
    core->snapshot.current_day_complete = false;

    /*
     * 尚未向算法送入样本。
     * 年龄为不可用值，读取者仍必须检查has_processed。
     */
    core->snapshot.has_processed = false;
    core->snapshot.processed_age_ms = UINT32_MAX;

    /* 日桶满后，首次从第0槽开始轮转淘汰。 */
    core->next_evict_index = 0U;

    /*
     * 整理初始日期和质量摘要。
     * 当前为STOPPED，不报告运行中的预热或输入超时；
     * 日期未知，因此会设置TIME_UNASSIGNED。
     */
    motion_core_refresh_summary(core);

    /* 所有初始化工作完成后，最后允许后续核心操作。 */
    core->initialized = true;

    return MOTION_CORE_OK;
}

/*
 * 简介：开始新会话，保留本次启动的长期累计。
 * 输入：core已初始化；两个ID非0；now_tick符合时间推进约定。
 * 返回值：0成功；参数、状态或时钟异常时返回对应错误。
 * 说明：只允许从STOPPED开始；不清启动累计、日桶及长期统计。
 */
int motion_core_start(motion_core_t *core, uint32_t run_generation, uint32_t session_id, uint32_t now_tick)
{
    uint32_t index;
    int clock_result;

    if ((core == NULL) || (run_generation == 0U) || (session_id == 0U))
    {
        return MOTION_CORE_ERR_ARGUMENT;
    }

    if (!core->initialized)
    {
        return MOTION_CORE_ERR_STATE;
    }

    /* 正在运行或处于故障状态时，不能隐式开启新会话。 */
    if (core->snapshot.state != MOTION_SERVICE_STOPPED)
    {
        return MOTION_CORE_ERR_STATE;
    }

    /* 时间检查失败时，不修改会话状态。 */
    clock_result = motion_core_advance_clock(core, now_tick);

    if (clock_result != MOTION_CORE_OK)
    {
        return clock_result;
    }

    core->snapshot.session_id = session_id;
    core->snapshot.run_generation = run_generation;

    /* 只清本次会话步数，不清启动累计及日期记录。 */
    for (index = 0U; index < MOTION_ALGORITHM_COUNT; index++)
    {
        core->snapshot.totals[index].session_steps = UINT64_C(0);
    }

    /*
     * 新会话重新建立算法阶段。
     * NONE表示正常启动，不设置异常缺样恢复标志。
     */
    motion_core_reset_algorithms(core, MOTION_RESET_NONE);

    /* 新会话重新建立序号和采样时间基准。 */
    core->has_previous_seq = false;
    core->previous_imu_seq = 0U;

    core->has_previous_tick = false;
    core->previous_sample_tick = 0U;

    core->input_timed_out = false;
    core->recovering_from_gap = false;

    /*
     * 清本会话的消费/处理进展。
     * 不能把上一会话的最后样本当成本会话已经收到的输入。
     * 累计消费和处理统计仍保留在stats中。
     */
    core->snapshot.has_dequeued = false;
    core->snapshot.last_dequeued_seq = 0U;
    core->snapshot.last_dequeued_tick = 0U;

    core->snapshot.has_processed = false;
    core->snapshot.last_processed_seq = 0U;
    core->snapshot.last_processed_tick = 0U;
    core->snapshot.processed_age_ms = UINT32_MAX;

    core->last_processed_at_ticks = UINT64_C(0);
    core->last_processed_initial_age_ticks = 0U;

    core->session_started_at_ticks = core->monotonic_ticks;

    /*
     * 新会话尚未收到输入，撤销上一会话的当前配置不匹配状态。
     * 新帧到达后重新检查；历史锁存标志仍保留。
     */
    core->snapshot.quality_flags &= ~MOTION_QUALITY_CONFIG_MISMATCH;

    core->snapshot.state = MOTION_SERVICE_RUNNING;

    /* 算法尚未建立样本阶段，摘要会报告WARMUP。 */
    motion_core_refresh_summary(core);

    return MOTION_CORE_OK;
}

/*
 * 简介：停止算法消费并保留最终会话计数。
 * 输入：core已初始化；now_tick符合时间推进约定。
 * 返回值：0成功；参数、状态或时钟异常时返回对应错误。
 * 说明：重复STOP不重复reset；服务层负责关闭生产接纳门。
 *       不通过STOP解除FAULT。
 */
int motion_core_stop(motion_core_t *core, uint32_t now_tick)
{
    int clock_result;

    if (core == NULL)
    {
        return MOTION_CORE_ERR_ARGUMENT;
    }

    if (!core->initialized)
    {
        return MOTION_CORE_ERR_STATE;
    }

    /* 只处理正常运行或已经停止的状态，不隐式解除故障。 */
    if ((core->snapshot.state != MOTION_SERVICE_RUNNING) &&
        (core->snapshot.state != MOTION_SERVICE_STOPPED))
    {
        return MOTION_CORE_ERR_STATE;
    }

    clock_result = motion_core_advance_clock(core, now_tick);

    if (clock_result != MOTION_CORE_OK)
    {
        return clock_result;
    }

    /* 幂等：已经停止时，不重复重置或增加reset统计。 */
    if (core->snapshot.state == MOTION_SERVICE_STOPPED)
    {
        return MOTION_CORE_OK;
    }

    /* 结束当前算法阶段，但保留核心业务累计和长期统计。 */
    motion_core_reset_algorithms(core, MOTION_RESET_STOP);

    core->snapshot.state = MOTION_SERVICE_STOPPED;

    /* 停止后，不再处于运行中的输入超时或缺样恢复期。 */
    core->input_timed_out = false;
    core->recovering_from_gap = false;

    /*
     * 撤销当前运行输入的配置不匹配状态。
     * 历史锁存不清，下一次START后的输入重新检查。
     */
    core->snapshot.quality_flags &= ~MOTION_QUALITY_CONFIG_MISMATCH;

    /*
     * 保留：
     * - session_id、run_generation及最终session_steps；
     * - 启动累计、未归日计数、日桶和归档；
     * - 累计统计、历史质量和最近处理样本信息。
     *
     * 刷新后不再报告WARMUP等运行期质量，
     * 但仍可报告日期未知、数据源离线或计数饱和。
     */
    motion_core_refresh_summary(core);

    return MOTION_CORE_OK;
}

/*
 * 简介：更新当前日历视图，不改变样本流和算法阶段。
 * 输入：core/calendar非NULL；可信日期须合法；now_tick合法。
 * 返回值：0成功；负值表示参数或时钟错误。
 * 说明：旧帧归日与当前显示日期分别维护，不回填未归日步数。
 */
int motion_core_update_calendar(motion_core_t *core,const motion_calendar_tag_t *calendar,uint32_t now_tick)
{
    bool changed;
    int clock_result;

    if ((core == NULL) || (calendar == NULL))
    {
        return MOTION_CORE_ERR_ARGUMENT;
    }

    if (!core->initialized)
    {
        return MOTION_CORE_ERR_STATE;
    }

    /* 仅在声明日期可信时，要求日期键合法。 */
    if (calendar->day_valid && !motion_core_day_key_is_valid(calendar->local_day_key))
    {
        return MOTION_CORE_ERR_ARGUMENT;
    }

    clock_result = motion_core_advance_clock(core, now_tick);

    if (clock_result != MOTION_CORE_OK)
    {
        return clock_result;
    }

    /*
     * 比较可信性、日历代际，以及可信日期本身。
     * 日期无效时，不将原始日期键的变化视为有效日期变化。
     */
    changed = (core->snapshot.current_day_valid != calendar->day_valid) ||
              (core->snapshot.calendar_generation !=calendar->calendar_generation) ||
              (calendar->day_valid &&(core->snapshot.current_day_key != calendar->local_day_key));

    if (changed)
    {
        motion_core_add_saturated(core,&core->stats.calendar_change_events,UINT64_C(1));
    }

    core->snapshot.current_day_valid = calendar->day_valid;
    core->snapshot.calendar_generation = calendar->calendar_generation;

    core->snapshot.current_day_key = calendar->day_valid ? calendar->local_day_key : 0U;

    /* 只查已有日桶，重新整理今日计数和质量。 */
    motion_core_refresh_summary(core);

    return MOTION_CORE_OK;
}

/*
 * 简介：处理数据源可用性变化。
 * 输入：core非NULL；source为有效枚举；now_tick合法。
 * 返回值：0成功；负值表示参数或时钟错误。
 * 说明：离线恢复计数按状态转换累计，不按轮询次数累计。
 */
int motion_core_update_source(motion_core_t *core, motion_source_state_t source, uint32_t now_tick)
{
    motion_source_state_t previous_source;
    int clock_result;

    if (core == NULL)
    {
        return MOTION_CORE_ERR_ARGUMENT;
    }

    if (!core->initialized)
    {
        return MOTION_CORE_ERR_STATE;
    }

    /* 显式检查枚举，不依赖其数值连续性。 */
    if ((source != MOTION_SOURCE_UNKNOWN) &&
        (source != MOTION_SOURCE_AVAILABLE) &&
        (source != MOTION_SOURCE_STALE) &&
        (source != MOTION_SOURCE_OFFLINE))
    {
        return MOTION_CORE_ERR_ARGUMENT;
    }

    clock_result = motion_core_advance_clock(core, now_tick);

    if (clock_result != MOTION_CORE_OK)
    {
        return clock_result;
    }

    previous_source = core->source_state;

    /* 相同状态不重复累计事件，也不重复重置算法。 */
    if (previous_source == source)
    {
        motion_core_refresh_summary(core);
        return MOTION_CORE_OK;
    }

    /* 每次进入OFFLINE，累计一次离线事件。 */
    if (source == MOTION_SOURCE_OFFLINE)
    {
        motion_core_add_saturated(core,&core->stats.source_offline_events,UINT64_C(1));
    }

    /*
     * 从过期或离线直接恢复可用，累计一次恢复事件。
     * UNKNOWN到AVAILABLE属于首次确认，不算故障恢复。
     */
    if (((previous_source == MOTION_SOURCE_STALE) ||(previous_source == MOTION_SOURCE_OFFLINE))
        && (source == MOTION_SOURCE_AVAILABLE))
    {
        motion_core_add_saturated(core, &core->stats.source_recovery_events, UINT64_C(1));
    }

    /*
     * 运行期间，数据源状态边界中断原有连续处理阶段。
     * 恢复后也从新有效样本重新建立预热，不接续旧滤波状态。
     */
    if (core->snapshot.state == MOTION_SERVICE_RUNNING)
    {
        motion_core_reset_algorithms(core, MOTION_RESET_SOURCE);
    }

    core->source_state = source;

    /*
     * source恢复AVAILABLE不等于已经处理了新样本。
     * 不在这里清input_timed_out，交给有效输入处理路径解除。
     */
    motion_core_refresh_summary(core);

    return MOTION_CORE_OK;
}

/*
 * 简介：对一份出队帧完成唯一分类和双算法处理。
 * 输入：core/frame/result非NULL；core已初始化；now_tick合法。
 * 返回值：0表示本帧分类完成；负值表示调用契约或单调时间错误。
 * 说明：每帧只计一次dequeued和一个消费分类。
 *       返回0不代表计步成功，内部故障通过fatal_error和FAULT表达。
 */
int motion_core_process(motion_core_t *core, const motion_sample_frame_t *frame, uint32_t now_tick, motion_core_frame_result_t *result)
{
    uint32_t sequence_delta;
    uint32_t sample_tick_delta;
    uint32_t sample_age_ticks;
    uint32_t sample_age_ms;
    uint32_t reset_reasons;
    uint32_t index;

    bool duplicate;
    bool sequence_gap;
    bool sequence_rollback;
    bool time_gap;
    bool all_algorithms_accepted;

    int clock_result;

    uint64_t *discard_counter;

    step_detector_input_t input;
    step_detector_process_result_t algorithm_result;
    step_detector_output_t output;

    /* ---------- A：调用检查，尚未开始消费记账 ---------- */

    if ((core == NULL) || (frame == NULL) || (result == NULL))
    {
        return MOTION_CORE_ERR_ARGUMENT;
    }

    if (!core->initialized)
    {
        return MOTION_CORE_ERR_STATE;
    }

    clock_result = motion_core_advance_clock(core, now_tick);

    if (clock_result != MOTION_CORE_OK)
    {
        return clock_result;
    }

    /*
     * 参数和时钟检查通过后，才认为本帧交给核心处理。
     * memset保证未调用算法的called标志为false。
     */
    memset(result, 0, sizeof(*result));

    duplicate = false;
    sequence_gap = false;
    sequence_rollback = false;
    time_gap = false;
    all_algorithms_accepted = true;

    reset_reasons = MOTION_RESET_NONE;
    discard_counter = NULL;

    motion_core_add_saturated(core, &core->stats.dequeued, UINT64_C(1));

    core->snapshot.has_dequeued = true;
    core->snapshot.last_dequeued_seq = frame->imu.imu_seq;
    core->snapshot.last_dequeued_tick = now_tick;

    /* ---------- B：先隔离旧代际，再检查运行状态 ---------- */

    if (frame->run_generation != core->snapshot.run_generation)
    {
        result->disposition = MOTION_FRAME_DISCARD_OLD_GENERATION;
        discard_counter = &core->stats.discard_old_generation;

        /*
         * 旧帧只登记消费分类。
         * 不得改变当前会话的序号、滤波状态或计数。
         */
        goto discard_frame;
    }

    if (core->snapshot.state != MOTION_SERVICE_RUNNING)
    {
        result->disposition = MOTION_FRAME_DISCARD_STOPPED;
        discard_counter = &core->stats.discard_stopped;
        goto discard_frame;
    }

    /* ---------- C：观察本代际提交序号 ---------- */

    if (core->has_previous_seq)
    {
        sequence_delta =(uint32_t)(frame->imu.imu_seq - core->previous_imu_seq);

        if (sequence_delta == 0U)
        {
            duplicate = true;
        }
        else if (sequence_delta < MOTION_CORE_TICK_HALF_RANGE)
        {
            if (sequence_delta > 1U)
            {
                sequence_gap = true;

                motion_core_add_saturated(core,&core->stats.seq_gap_events,UINT64_C(1));

                motion_core_add_saturated(core,&core->stats.seq_missing_samples,(uint64_t)(sequence_delta - 1U));
            }
        }
        else
        {
            sequence_rollback = true;

            motion_core_add_saturated(core,&core->stats.seq_rollback_events,UINT64_C(1));
        }
    }

    /*
     * “观察到序号”不等于“接受了样本”。
     * 即使稍后因过期等原因拒绝，也已经明确看到该帧，
     * 下一次不能再把它重复算成未观察到的缺失样本。
     */
    if (!duplicate)
    {
        core->previous_imu_seq = frame->imu.imu_seq;
        core->has_previous_seq = true;
    }

    if (sequence_gap)
    {
        reset_reasons |= MOTION_RESET_SEQUENCE_GAP;
    }

    if (sequence_rollback)
    {
        reset_reasons |= MOTION_RESET_ROLLBACK;
    }

    /* ---------- D：按固定优先级检查样本门禁 ---------- */

    if (!frame->imu.valid)
    {
        result->disposition = MOTION_FRAME_DISCARD_INVALID;
        discard_counter = &core->stats.discard_invalid;
        reset_reasons |= MOTION_RESET_INVALID;
        goto reject_input;
    }

    /*
     * 样本年龄必须使用采样tick。
     * enqueue_tick只用于服务层计算排队时间。
     */
    sample_age_ticks = (uint32_t)(now_tick - frame->imu.sample_tick);

    if (sample_age_ticks >= MOTION_CORE_TICK_HALF_RANGE)
    {
        /*
         * 无法解释为正常年龄，可能是未来时间或异常跨度。
         * 拒绝该帧，不拿一个错误年龄继续计步。
         */
        result->disposition = MOTION_FRAME_DISCARD_INVALID;
        discard_counter = &core->stats.discard_invalid;

        reset_reasons |= MOTION_RESET_INVALID | MOTION_RESET_ROLLBACK;

        core->snapshot.last_error = MOTION_CORE_ERR_CLOCK;
        goto reject_input;
    }

    sample_age_ms = motion_core_ticks_to_ms(sample_age_ticks, core->tick_hz);

    if (sample_age_ms > MOTION_INPUT_STALE_MS)
    {
        result->disposition = MOTION_FRAME_DISCARD_STALE;
        discard_counter = &core->stats.discard_stale;
        reset_reasons |= MOTION_RESET_STALE;

        /* 保持到后续有效新输入被接受，不能靠等待自动解除。 */
        core->input_timed_out = true;
        goto reject_input;
    }

    if (core->source_state != MOTION_SOURCE_AVAILABLE)
    {
        /*
         * 本分类也包含数据源不可用。
         * 具体是UNKNOWN、STALE还是OFFLINE，由source_state解释。
         * 不在这里伪造离线或恢复事件。
         */
        result->disposition = MOTION_FRAME_DISCARD_STALE;
        discard_counter = &core->stats.discard_stale;
        reset_reasons |= MOTION_RESET_SOURCE;
        goto reject_input;
    }

    if ((frame->imu.config_version !=
         STEP_DETECTOR_CONFIG_B0_TASK10_V1.expected_config_version) ||
        (frame->imu.config_version !=
         STEP_DETECTOR_CONFIG_B1_TASK10_V1.expected_config_version))
    {
        result->disposition = MOTION_FRAME_DISCARD_CONFIG;
        discard_counter = &core->stats.discard_config;
        reset_reasons |= MOTION_RESET_CONFIG;

        core->snapshot.quality_flags |= MOTION_QUALITY_CONFIG_MISMATCH;

        goto reject_input;
    }

    /* 当前新鲜输入的配置已经确认匹配；历史锁存不清。 */
    core->snapshot.quality_flags &= ~MOTION_QUALITY_CONFIG_MISMATCH;

    if (duplicate)
    {
        result->disposition = MOTION_FRAME_DISCARD_DUPLICATE;
        discard_counter = &core->stats.discard_duplicate;

        /* 普通重复不重置算法，也不刷新有效输入进展。 */
        goto discard_frame;
    }

    /* ---------- E：对新有效帧检查采样时间连续性 ---------- */

    if (core->has_previous_tick)
    {
        sample_tick_delta = (uint32_t)(frame->imu.sample_tick - core->previous_sample_tick);

        if (sample_tick_delta >= MOTION_CORE_TICK_HALF_RANGE)
        {
            /* 采样时间异常倒退或跨度不明确，重建阶段。 */
            time_gap = true;
            reset_reasons |= MOTION_RESET_ROLLBACK;
        }
        else
        {
            /*
             * 两个算法都要检查。
             * 任一个算法的连续性门限被超过，就统一重建阶段。
             */
            for (index = 0U; index < MOTION_ALGORITHM_COUNT; index++)
            {
                if (motion_core_ticks_to_ms(sample_tick_delta, core->tick_hz) >
                    core->detector[index].config.reset_gap_ms)
                {
                    time_gap = true;
                }
            }
        }

        if (time_gap)
        {
            reset_reasons |= MOTION_RESET_TIME_GAP;

            /* 一帧只计一次时间缺口，不按算法数量重复计数。 */
            motion_core_add_saturated(core,&core->stats.time_gap_events,UINT64_C(1));
        }
    }

    if (reset_reasons != MOTION_RESET_NONE)
    {
        /*
         * 多个原因合并后，只执行一次阶段重置。
         * 当前帧随后成为新预热阶段的第一份输入。
         */
        motion_core_reset_algorithms(core, reset_reasons);
        result->reset_reasons = reset_reasons;
    }

    /* ---------- F：转换输入，分别调用两个算法 ---------- */

    memset(&input, 0, sizeof(input));

    input.tick = frame->imu.sample_tick;
    input.imu_seq = frame->imu.imu_seq;

    input.ax_ug = frame->imu.accel_ug[0];
    input.ay_ug = frame->imu.accel_ug[1];
    input.az_ug = frame->imu.accel_ug[2];

    input.config_version = frame->imu.config_version;
    input.valid = true;
    input.stale = false;

    /*
     * 进入算法阶段后，本帧固定归类为passed_to_alg。
     * 即使算法随后报错，也不能再重复归入discard分类。
     */
    result->disposition = MOTION_FRAME_PASSED_TO_ALGORITHM;

    motion_core_add_saturated( core, &core->stats.passed_to_alg, UINT64_C(1));

    for (index = 0U; index < MOTION_ALGORITHM_COUNT; index++)
    {
        /* 防止算法错误返回时，输出中残留上一算法的数据。 */
        memset(&output, 0, sizeof(output));

        motion_core_add_saturated(core,&core->stats.algorithm[index].process_calls,UINT64_C(1));

        result->algorithm_called[index] = true;

        algorithm_result = step_detector_process( &core->detector[index], &input, &output);

        result->algorithm_result[index] = algorithm_result;
        result->algorithm_output[index] = output;

        switch (algorithm_result)
        {
        case STEP_DETECTOR_PROCESS_ACCEPTED:
            motion_core_add_saturated(
                core,
                &core->stats.algorithm[index].accepted_no_event,
                UINT64_C(1));
            break;

        case STEP_DETECTOR_PROCESS_EVENT:
            motion_core_add_saturated(
                core,
                &core->stats.algorithm[index].events,
                UINT64_C(1));
            break;

        case STEP_DETECTOR_PROCESS_IGNORED:
            all_algorithms_accepted = false;

            motion_core_add_saturated(
                core,
                &core->stats.algorithm[index].ignored,
                UINT64_C(1));
            break;

        case STEP_DETECTOR_PROCESS_ERROR:
        default:
            all_algorithms_accepted = false;

            motion_core_add_saturated(
                core,
                &core->stats.algorithm[index].errors,
                UINT64_C(1));

            core->snapshot.state = MOTION_SERVICE_FAULT;
            core->snapshot.last_error = MOTION_CORE_ERR_STATE;
            result->fatal_error = true;
            break;
        }

        /*
         * 记录算法调用内部实际发生的阶段重建。
         * 核心此前调用的是完整reset，新算法阶段尚未建立，
         * 正常情况下不会对同一个旧阶段再次报告state_reset。
         */
        if (output.state_reset)
        {
            /* 算法内部也可能自行重建阶段，旧的预热完成标志必须失效。 */
            core->warmup_completed[index] = false;

            motion_core_add_saturated(core,&core->stats.algorithm[index].reset_events,UINT64_C(1));

            if (output.reason == STEP_DETECTOR_REASON_TIME_GAP_RESET)
            {
                result->reset_reasons |= MOTION_RESET_TIME_GAP;
                core->recovering_from_gap = true;
            }
            else if (output.reason ==STEP_DETECTOR_REASON_SEQUENCE_ROLLBACK_RESET)
            {
                result->reset_reasons |= MOTION_RESET_ROLLBACK;
                core->recovering_from_gap = true;
            }
        }

        if (result->fatal_error)
        {
            /* 故障后不继续调用后面的算法或增加业务步数。 */
            break;
        }

        if (output.event_generated)
        {
            /*
             * 显式选择业务算法标识，不把不同枚举强制转换。
             * 日期取自本帧，不能使用消费时的当前日期替代。
             */
            motion_core_record_event(
                core,
                (index == MOTION_ALGORITHM_B0) ? MOTION_ALGORITHM_B0 : MOTION_ALGORITHM_B1,
                &frame->calendar);

            if (core->snapshot.state == MOTION_SERVICE_FAULT)
            {
                all_algorithms_accepted = false;
                result->fatal_error = true;
                break;
            }
        }
    }

    /* ---------- G：记录实际调用进展，整理摘要 ---------- */

    /*
     * has_processed表示确实送入过算法，不等于产生了步数，
     * 也不单独代表算法有效接受或服务健康。
     */
    core->snapshot.has_processed = true;
    core->snapshot.last_processed_seq = frame->imu.imu_seq;
    core->snapshot.last_processed_tick = frame->imu.sample_tick;

    core->last_processed_at_ticks = core->monotonic_ticks;
    core->last_processed_initial_age_ticks = sample_age_ticks;

    core->has_previous_tick = true;
    core->previous_sample_tick = frame->imu.sample_tick;

    if (!result->fatal_error && all_algorithms_accepted)
    {
        core->input_timed_out = false;
        core->snapshot.last_error = MOTION_CORE_OK;
    }

    motion_core_refresh_summary(core);

    /*
     * 只有有效新输入被接受，且两个算法均完成预热，
     * 才解除缺样恢复期；单纯经过墙钟时间不能解除。
     */
    if (!result->fatal_error && all_algorithms_accepted &&
        ((core->snapshot.quality_flags & MOTION_QUALITY_WARMUP) == UINT32_C(0)))
    {
        core->recovering_from_gap = false;
        motion_core_refresh_summary(core);
    }

    return MOTION_CORE_OK;

reject_input:
    /*
     * 无效、过期、配置不匹配等情况统一在这里重建阶段。
     * 已合并序号缺口等原因，不在每个分支重复reset。
     */
    result->reset_reasons = reset_reasons;
    motion_core_reset_algorithms(core, reset_reasons);

discard_frame:
    /*
     * 每条丢弃路径只增加一个分类。
     * discard_counter由上面的固定优先级分支设置。
     */
    motion_core_add_saturated(core, discard_counter, UINT64_C(1));

    if (core->snapshot.state == MOTION_SERVICE_FAULT)
    {
        result->fatal_error = true;
    }

    motion_core_refresh_summary(core);

    return MOTION_CORE_OK;
}

/*
 * 简介：无新样本时检测输入超时并维护恢复状态。
 * 输入：core已初始化；now_tick为周期调用时刻。
 * 返回值：0成功；负值表示参数或时钟错误。
 * 说明：超时锁存到有效输入恢复，防止tick回绕后旧输入重新变新鲜。
 */
int motion_core_tick(motion_core_t *core, uint32_t now_tick)
{
    uint64_t age_ticks;
    uint32_t age_ms;
    int clock_result;

    if (core == NULL)
    {
        return MOTION_CORE_ERR_ARGUMENT;
    }

    if (!core->initialized)
    {
        return MOTION_CORE_ERR_STATE;
    }

    clock_result = motion_core_advance_clock(core, now_tick);

    if (clock_result != MOTION_CORE_OK)
    {
        return clock_result;
    }

    /* 停止或故障状态不启动新的运行期输入超时。 */
    if (core->snapshot.state != MOTION_SERVICE_RUNNING)
    {
        motion_core_refresh_summary(core);
        return MOTION_CORE_OK;
    }

    /*
     * 已经判定超时后，保持该状态。
     * 只有后续有效输入处理路径才能解除，不能靠继续等待解除。
     */
    if (core->input_timed_out)
    {
        motion_core_refresh_summary(core);
        return MOTION_CORE_OK;
    }

    if (!core->snapshot.has_processed)
    {
        /* 本会话尚无输入，从START时刻计算等待时间。 */
        if (core->monotonic_ticks < core->session_started_at_ticks)
        {
            return MOTION_CORE_ERR_CLOCK;
        }

        age_ticks = core->monotonic_ticks - core->session_started_at_ticks;
    }
    else
    {
        /* 已有输入，计算最近处理样本到当前时刻的完整年龄。 */
        if (core->monotonic_ticks < core->last_processed_at_ticks)
        {
            return MOTION_CORE_ERR_CLOCK;
        }

        age_ticks =core->monotonic_ticks - core->last_processed_at_ticks;

        /* 加上样本在处理完成时已经存在的年龄。 */
        if (age_ticks >
            (UINT64_MAX -(uint64_t)core->last_processed_initial_age_ticks))
        {
            age_ticks = UINT64_MAX;
        }
        else
        {
            age_ticks +=
                (uint64_t)core->last_processed_initial_age_ticks;
        }
    }

    age_ms = motion_core_ticks_to_ms(age_ticks, core->tick_hz);

    /* 与当前契约一致：超过250ms才判定超时。 */
    if (age_ms > MOTION_INPUT_STALE_MS)
    {
        core->input_timed_out = true;

        /*
         * 超时边界只重建一次阶段。
         * 已建立阶段的reset_events由辅助函数登记。
         * 不清业务累计，不修改数据源离线/恢复统计。
         */
        motion_core_reset_algorithms(core, MOTION_RESET_STALE);
    }

    /* 根据input_timed_out等状态生成当前及历史质量标志。 */
    motion_core_refresh_summary(core);

    return MOTION_CORE_OK;
}

/*
 * 简介：复制核心摘要并计算当前样本年龄。
 * 输入：core/snapshot非NULL；now_tick合法。
 * 返回值：0成功；负值表示参数、状态或时钟错误。
 * 说明：const读取，不推进core；服务发布字段由服务层另行填写。
 */
int motion_core_get_snapshot(const motion_core_t *core, uint32_t now_tick, motion_snapshot_t *snapshot)
{
    uint32_t now_delta;
    uint64_t read_ticks;
    uint64_t age_ticks;

    if ((core == NULL) || (snapshot == NULL))
    {
        return MOTION_CORE_ERR_ARGUMENT;
    }

    if (!core->initialized)
    {
        return MOTION_CORE_ERR_STATE;
    }

    /* 计算距核心上一次推进又经过了多少tick。 */
    now_delta = (uint32_t)(now_tick - core->previous_now_tick);

    if (now_delta >= MOTION_CORE_TICK_HALF_RANGE)
    {
        return MOTION_CORE_ERR_CLOCK;
    }

    /* 计算读取时刻的扩展时间，先检查加法上限。 */
    if (core->monotonic_ticks > (UINT64_MAX - (uint64_t)now_delta))
    {
        return MOTION_CORE_ERR_CLOCK;
    }

    read_ticks = core->monotonic_ticks + (uint64_t)now_delta;

    age_ticks = UINT64_C(0);

    if (core->snapshot.has_processed)
    {
        /*
         * 处理完成时刻不可能晚于当前读取时刻。
         * 异常时拒绝，防止无符号减法产生错误年龄。
         */
        if (read_ticks < core->last_processed_at_ticks)
        {
            return MOTION_CORE_ERR_CLOCK;
        }

        /* 距最近一次处理完成，又经过了多久。 */
        age_ticks = read_ticks - core->last_processed_at_ticks;

        /*
         * 样本在处理完成时已经有一定年龄，需要加回来。
         * 极端情况下加法溢出，按最大年龄处理。
         */
        if (age_ticks > (UINT64_MAX -(uint64_t)core->last_processed_initial_age_ticks))
        {
            age_ticks = UINT64_MAX;
        }
        else
        {
            age_ticks += (uint64_t)core->last_processed_initial_age_ticks;
        }
    }

    /* 检查完成后，按值复制核心摘要。 */
    *snapshot = core->snapshot;

    if (snapshot->has_processed)
    {
        snapshot->processed_age_ms = motion_core_ticks_to_ms(age_ticks, core->tick_hz);
    }
    else
    {
        /* 没有处理过样本，不能把年龄解释成0毫秒。 */
        snapshot->processed_age_ms = UINT32_MAX;
    }

    return MOTION_CORE_OK;
}

/*
 * 简介：复制纯核心累计统计。
 * 输入：core/stats非NULL；core已初始化。
 * 返回值：0成功；参数或状态非法返回负值。
 * 说明：仅值复制，不清统计，不承担跨线程保护。
 */
int motion_core_get_stats(const motion_core_t *core,
                          motion_core_stats_t *stats)
{
    if ((core == NULL) || (stats == NULL))
    {
        return MOTION_CORE_ERR_ARGUMENT;
    }

    if (!core->initialized)
    {
        return MOTION_CORE_ERR_STATE;
    }

    *stats = core->stats;

    return MOTION_CORE_OK;
}

//---------------------------------------------------------------私有函数---------------------------------------------------------------
/*
 * 简介：把连续提供的32位tick推进为内部64位相对时间。
 * 输入：core已初始化；now_tick为调用者提供的单调tick。
 * 返回值：0成功；跨度不明确或扩展值溢出时返回时钟错误。
 * 说明：只处理无符号差，不比较now_tick大小；正常回绕不报错。
 */
static int motion_core_advance_clock(motion_core_t *core,uint32_t now_tick)
{
    uint32_t delta;

    if (core == NULL)
    {
        return MOTION_CORE_ERR_ARGUMENT;
    }

    if (!core->initialized)
    {
        return MOTION_CORE_ERR_STATE;
    }

    /* 无符号减法支持32位tick正常回绕。 */
    delta = (uint32_t)(now_tick - core->previous_now_tick);

    /*
     * 相邻调用跨度必须小于半区间。
     * 达到半区间后，无法可靠区分向前推进与异常倒退。
     */
    if (delta >= MOTION_CORE_TICK_HALF_RANGE)
    {
        return MOTION_CORE_ERR_CLOCK;
    }

    /* 先检查64位累计值是否会溢出，再执行加法。 */
    if (core->monotonic_ticks > (UINT64_MAX - (uint64_t)delta))
    {
        return MOTION_CORE_ERR_CLOCK;
    }

    /* 所有检查通过后，一次完成本次时间推进。 */
    core->monotonic_ticks += (uint64_t)delta;
    core->previous_now_tick = now_tick;

    return MOTION_CORE_OK;
}

/*
 * 简介：把内部tick数量换算为饱和毫秒值。
 * 输入：ticks为非负tick数量；tick_hz必须非0。
 * 返回值：毫秒值，超过UINT32_MAX时返回UINT32_MAX。
 * 说明：使用先商后余数等方式避免ticks乘1000发生64位溢出。
 */
static uint32_t motion_core_ticks_to_ms(uint64_t ticks,uint32_t tick_hz)
{
    uint64_t seconds;
    uint64_t remainder;
    uint64_t milliseconds;

    /*
     * 防止除零。
     * 返回饱和值，避免把非法频率换算成看似新鲜的0毫秒。
     */
    if (tick_hz == 0U)
    {
        return UINT32_MAX;
    }

    seconds = ticks / (uint64_t)tick_hz;
    remainder = ticks % (uint64_t)tick_hz;

    /*
     * 完整秒数已经超过输出范围时，直接饱和。
     * 此检查也保证后面的seconds乘1000安全。
     */
    if (seconds > ((uint64_t)UINT32_MAX / UINT64_C(1000)))
    {
        return UINT32_MAX;
    }

    /*
     * remainder小于32位tick_hz，
     * 因此提升到64位后乘1000不会溢出。
     */
    milliseconds =seconds * UINT64_C(1000) +
                (remainder * UINT64_C(1000)) / (uint64_t)tick_hz;

    /* 加上不足一秒的部分后，还需要检查一次输出上限。 */
    if (milliseconds > (uint64_t)UINT32_MAX)
    {
        return UINT32_MAX;
    }

    return (uint32_t)milliseconds;
}

/*
 * 简介：对一个核心累计统计执行饱和加法。
 * 输入：core/counter非NULL，counter指向core拥有的累计量。
 * 返回值：无。
 * 说明：溢出时饱和并锁存统计/快照的饱和标志，不静默回绕。
 */
static void motion_core_add_saturated(motion_core_t *core,uint64_t *counter,uint64_t increment)
{
    if ((core == NULL) || (counter == NULL))
    {
        return;
    }

    /* 没有增量时，不改变计数或标志。 */
    if (increment == UINT64_C(0))
    {
        return;
    }

    /*
     * UINT64_MAX - *counter表示剩余容量。
     * 先比较，避免先相加导致无符号回绕。
     */
    if (increment >= (UINT64_MAX - *counter))
    {
        *counter = UINT64_MAX;

        /* 直接设置标志，不能递归调用本函数登记饱和。 */
        core->stats.counters_saturated = true;
        core->snapshot.quality_flags |= MOTION_QUALITY_COUNTER_SATURATED;
        core->snapshot.latched_quality_flags |= MOTION_QUALITY_COUNTER_SATURATED;

        return;
    }

    *counter += increment;
}

/*
 * 简介：清算法阶段，同时保留业务累计与长期统计。
 * 输入：core非NULL；reasons说明本次阶段重建原因。
 * 返回值：无。
 * 说明：公共step_detector_reset会清内部计数，核心必须独立累计。
 *       不清已观察序号；START需要清序号时由START显式处理。
 */
static void motion_core_reset_algorithms(motion_core_t *core,uint32_t reasons)
{
    uint32_t index;
    bool had_phase;

    if (core == NULL)
    {
        return;
    }

    for (index = 0U; index < MOTION_ALGORITHM_COUNT; index++)
    {
        /* 未配置的算法无法执行有效重置，由初始化流程保证配置。 */
        if (!core->detector[index].configured)
        {
            continue;
        }

        /*
         * 与现有算法内部的阶段判断保持一致：
         * 已建立预热/滤波阶段、存在事件基准，或处于未重新武装状态。
         */
        had_phase =core->detector[index].initialized ||
                    core->detector[index].has_last_event ||
                    !core->detector[index].armed;

        /*
         * 清算法内部状态，不清核心维护的业务累计和长期统计。
         * 必须在reset之前保存had_phase。
         */
        step_detector_reset(&core->detector[index]);

        /* 原阶段结束，新阶段必须重新通过有效样本完成预热。 */
        core->warmup_completed[index] = false;

        /* 重复失效报告不会反复增加阶段重置次数。 */
        if (had_phase)
        {
            motion_core_add_saturated(core,
                                    &core->stats.algorithm[index].reset_events,
                                    UINT64_C(1));
        }
    }

    /* 下一份有效样本重新建立连续处理的时间基准。 */
    core->has_previous_tick = false;

    /*
     * 缺样或时序异常后进入恢复期。
     * 不因后续其他原因的reset而提前清除此标志。
     */
    if ((reasons & (MOTION_RESET_SEQUENCE_GAP |
                    MOTION_RESET_TIME_GAP |
                    MOTION_RESET_ROLLBACK)) != UINT32_C(0))
    {
        core->recovering_from_gap = true;
    }

    /*
     * 保留以下内容：
     * - 业务累计、日桶和核心长期统计；
     * - has_previous_seq及previous_imu_seq；
     * - 运行代际、会话信息和内部64位单调时间。
     */
}

/*
 * 简介：校验显示日期键的年月日合法性。
 * 输入：day_key为YYYYMMDD。
 * 返回值：合法返回true，否则false。
 * 说明：使用公历闰年规则，不访问RTC或mktime。
 *       当前UTC范围2000～2099、显示UTC+8，
 *       显示日期允许到2100-01-01，不能错误拒绝这一边界。
 */
static bool motion_core_day_key_is_valid(uint32_t day_key)
{
    static const uint8_t month_days[12] =
    {
        31U, 28U, 31U, 30U, 31U, 30U,
        31U, 31U, 30U, 31U, 30U, 31U
    };

    uint32_t year;
    uint32_t month;
    uint32_t day;
    uint32_t days_in_month;
    bool leap_year;

    /*
     * UTC上界为2099年末，UTC+8的显示日期可以进入2100-01-01。
     * 这里只检查日期，因此允许这一天，不代表允许其全部时刻。
     */
    if ((day_key < UINT32_C(20000101)) || (day_key > UINT32_C(21000101)))
    {
        return false;
    }

    /* 将YYYYMMDD拆成年、月、日。 */
    year = day_key / UINT32_C(10000);
    month = (day_key / UINT32_C(100)) % UINT32_C(100);
    day = day_key % UINT32_C(100);

    /* 必须先检查月份，才能安全访问月份天数数组。 */
    if ((month < 1U) || (month > 12U))
    {
        return false;
    }

    /* 公历：四年一闰，百年不闰，四百年再闰。 */
    leap_year = ((year % 4U) == 0U) &&
        (((year % 100U) != 0U) || ((year % 400U) == 0U));

    days_in_month = month_days[month - 1U];

    if ((month == 2U) && leap_year)
    {
        days_in_month = 29U;
    }

    if ((day < 1U) || (day > days_in_month))
    {
        return false;
    }

    return true;
}

/*
 * 简介：找到指定日期的日桶，必要时分配或淘汰。
 * 输入：core非NULL；day_key已通过合法性检查。
 * 返回值：有效槽下标；失败返回MOTION_CORE_INVALID_DAY_INDEX。
 * 说明：先查已有，再找空槽，最后有限轮转淘汰；
 *       不能通过日期大小推断最近访问顺序，因为日历允许回拨。
 */
static uint32_t motion_core_find_or_allocate_day(motion_core_t *core,uint32_t day_key)
{
    uint32_t index;
    uint32_t selected_index;
    uint32_t algorithm;

    if ((core == NULL) || !motion_core_day_key_is_valid(day_key))
    {
        return MOTION_CORE_INVALID_DAY_INDEX;
    }

    /*
     * 先查已有日期。
     * 日期回拨后再次访问原桶，应保留并继续累计原步数。
     */
    for (index = 0U; index < MOTION_DAY_HISTORY_CAPACITY; index++)
    {
        if (core->snapshot.day_history[index].used &&
            (core->snapshot.day_history[index].local_day_key == day_key))
        {
            return index;
        }
    }

    /* 没有对应日期，优先寻找空槽。 */
    selected_index = MOTION_CORE_INVALID_DAY_INDEX;

    for (index = 0U; index < MOTION_DAY_HISTORY_CAPACITY; index++)
    {
        if (!core->snapshot.day_history[index].used)
        {
            selected_index = index;
            break;
        }
    }

    /* 所有槽位均已使用，按有限轮转顺序淘汰。 */
    if (selected_index == MOTION_CORE_INVALID_DAY_INDEX)
    {
        /*
         * 下标异常时直接失败，不通过取模掩盖内部状态错误。
         * 此检查必须在修改计数之前完成。
         */
        if (core->next_evict_index >= MOTION_DAY_HISTORY_CAPACITY)
        {
            return MOTION_CORE_INVALID_DAY_INDEX;
        }

        selected_index = core->next_evict_index;

        /*
         * 清桶之前，分别归档B0、B1的旧步数。
         * 这里只转移存放位置，不增加boot_steps或session_steps。
         * 将对应日期(selected_index)的步数累加到totals.archived_day_steps。
         */
        for (algorithm = 0U;algorithm < MOTION_ALGORITHM_COUNT;algorithm++)
        {
            motion_core_add_saturated(core,
                &core->snapshot.totals[algorithm].archived_day_steps,
                core->snapshot.day_history[selected_index].steps[algorithm]);
        }

        motion_core_add_saturated(core,&core->stats.day_bucket_evictions,UINT64_C(1));

        core->history_evicted = true;

        /* 下一次淘汰下一个槽位，到末尾后回到第0槽。 */
        core->next_evict_index = selected_index + 1U;

        if (core->next_evict_index >= MOTION_DAY_HISTORY_CAPACITY)
        {
            core->next_evict_index = 0U;
        }
    }

    /* 只清理被选中的桶，其他日期记录保持不变。 */
    memset(&core->snapshot.day_history[selected_index],0,
           sizeof(core->snapshot.day_history[selected_index]));

    core->snapshot.day_history[selected_index].used = true;
    core->snapshot.day_history[selected_index].local_day_key = day_key;

    /*
     * 未发生淘汰时，可以证明该日期没有被删除过。
     * 曾发生淘汰后，无法判断新建日期是否曾被删除，保守标为不完整。
     */
    core->snapshot.day_history[selected_index].complete =!core->history_evicted;

    return selected_index;
}

/*
 * 简介：将一个算法事件加入业务累计和对应日期。
 * 输入：core/calendar非NULL；algorithm为有效B0/B1。
 * 返回值：无。
 * 说明：每个event_generated只调用一次；日历标签取自该帧。
 *       没有可信日期进入unassigned，不在后续校时回填。
 */
static void motion_core_record_event(motion_core_t *core,motion_algorithm_id_t algorithm,const motion_calendar_tag_t *calendar)
{
    uint32_t day_index;
    motion_algorithm_totals_t *totals;

    if (core == NULL)
    {
        return;
    }

    if ((calendar == NULL) ||
        ((algorithm != MOTION_ALGORITHM_B0) && (algorithm != MOTION_ALGORITHM_B1)))
    {
        core->snapshot.state = MOTION_SERVICE_FAULT;
        core->snapshot.last_error = MOTION_CORE_ERR_ARGUMENT;
        return;
    }

    /* 达到上限后保持阻断，不能通过后续事件使计数回绕。 */
    if (core->counting_blocked[algorithm])
    {
        return;
    }

    /*
     * 声明可信却携带非法日期，属于调用契约错误。
     * 不静默当成未归日，也不增加任何业务计数。
     */
    if (calendar->day_valid && !motion_core_day_key_is_valid(calendar->local_day_key))
    {
        core->snapshot.state = MOTION_SERVICE_FAULT;
        core->snapshot.last_error = MOTION_CORE_ERR_ARGUMENT;
        return;
    }

    totals = &core->snapshot.totals[algorithm];
    day_index = MOTION_CORE_INVALID_DAY_INDEX;

    /*
     * 先检查两项共同计数，再允许分配日桶。
     * 避免已经不能记账时，仍无意义地淘汰历史。
     */
    if ((totals->boot_steps == UINT64_MAX) ||
        (totals->session_steps == UINT64_MAX))
    {
        goto counter_saturated;
    }

    if (calendar->day_valid)
    {
        day_index = motion_core_find_or_allocate_day(core,calendar->local_day_key);

        if (day_index == MOTION_CORE_INVALID_DAY_INDEX)
        {
            core->snapshot.state = MOTION_SERVICE_FAULT;
            core->snapshot.last_error = MOTION_CORE_ERR_STATE;
            return;
        }

        /* 找到已有桶时，其计数也可能已经达到上限。 */
        if (core->snapshot.day_history[day_index].steps[algorithm] == UINT64_MAX)
        {
            goto counter_saturated;
        }
    }
    else
    {
        if (totals->unassigned_steps == UINT64_MAX)
        {
            goto counter_saturated;
        }
    }

    /*
     * 所有涉及的计数均有空间。
     * 从这里开始一起提交本次事件，不再执行可能失败的操作。
     */
    totals->boot_steps++;
    totals->session_steps++;

    if (calendar->day_valid)
    {
        core->snapshot.day_history[day_index].steps[algorithm]++;

        if (core->snapshot.day_history[day_index].steps[algorithm] == UINT64_MAX)
        {
            goto counter_saturated;
        }
    }
    else
    {
        totals->unassigned_steps++;

        if (totals->unassigned_steps == UINT64_MAX)
        {
            goto counter_saturated;
        }
    }

    /* 恰好达到上限时，本次事件已完整记账，之后禁止继续增加。 */
    if ((totals->boot_steps == UINT64_MAX) ||
        (totals->session_steps == UINT64_MAX))
    {
        goto counter_saturated;
    }

    return;

counter_saturated:
    core->counting_blocked[algorithm] = true;

    core->snapshot.quality_flags |= MOTION_QUALITY_COUNTER_SATURATED;

    core->snapshot.latched_quality_flags |= MOTION_QUALITY_COUNTER_SATURATED;
}

/*
 * 简介：从核心状态刷新当前日期计数及质量摘要。
 * 输入：core非NULL。
 * 返回值：无。
 * 说明：只查日桶，不分配；当前质量可解除，历史锁存不解除。
 *       不递增发布序号，不修改线程进展，不增加业务计数。
 */
static void motion_core_refresh_summary(motion_core_t *core)
{
    uint32_t index;
    uint32_t algorithm;
    uint32_t quality_flags;
    uint32_t warmup_ticks; // 本阶段实际样本覆盖的tick跨度
    bool warmup;
    bool found_current_day;

    if (core == NULL)
    {
        return;
    }

    /*
     * 配置不匹配由process根据实际输入设置和解除。
     * 其他质量位在下面根据当前状态重新计算。
     */
    quality_flags =core->snapshot.quality_flags & MOTION_QUALITY_CONFIG_MISMATCH;

    warmup = false;
    found_current_day = false;

    /* 每次重新整理当前日期计数，避免残留上一次日期的数据。 */
    for (algorithm = 0U;algorithm < MOTION_ALGORITHM_COUNT;algorithm++)
    {
        core->snapshot.current_day_steps[algorithm] = UINT64_C(0);
    }

    core->snapshot.current_day_complete = false;

    /*
     * day_key合法不等于日历可信。
     * 原本无效的日期不能仅因格式合法就恢复为有效。
     */
    if (core->snapshot.current_day_valid && !motion_core_day_key_is_valid(core->snapshot.current_day_key))
    {
        core->snapshot.current_day_valid = false;
    }

    if (core->snapshot.current_day_valid)
    {
        /* 只查已有桶，不因刷新显示而分配桶。 */
        for (index = 0U; index < MOTION_DAY_HISTORY_CAPACITY; index++)
        {
            if (core->snapshot.day_history[index].used &&
                (core->snapshot.day_history[index].local_day_key == core->snapshot.current_day_key))
            {
                for (algorithm = 0U; algorithm < MOTION_ALGORITHM_COUNT; algorithm++)
                {
                    core->snapshot.current_day_steps[algorithm] =
                        core->snapshot.day_history[index].steps[algorithm];
                }

                core->snapshot.current_day_complete =
                    core->snapshot.day_history[index].complete;

                found_current_day = true;
                break;
            }
        }

        /*
         * 没有对应桶：
         * 未淘汰过历史时，表示当前已归日计数确实为0；
         * 淘汰过历史后，不能排除该日期以前有记录。
         */
        if (!found_current_day)
        {
            core->snapshot.current_day_complete =!core->history_evicted;
        }

        if (!core->snapshot.current_day_complete)
        {
            quality_flags |= MOTION_QUALITY_HISTORY_PARTIAL;
        }
    }
    else
    {
        /*
         * 数组虽然清零，但此时不能显示为可信“今日0步”。
         * 读取者必须同时检查current_day_valid。
         */
        quality_flags |= MOTION_QUALITY_TIME_UNASSIGNED;
    }

    /*
     * 仅在运行期间报告预热。
     * 使用算法最近实际处理的样本tick判断，不使用now_tick或monotonic_ticks代替样本进展。
     */
    if (core->snapshot.state == MOTION_SERVICE_RUNNING)
    {
        for (algorithm = 0U; algorithm < MOTION_ALGORITHM_COUNT; algorithm++)
        {
            if (!core->detector[algorithm].configured ||
                !core->detector[algorithm].initialized)
            {
                core->warmup_completed[algorithm] = false;
                warmup = true;
                continue;
            }

            /*
            * 只在本阶段尚未完成预热时检查时间覆盖。
            * 一旦完成，不再因长期tick差进入半区间而重新判为预热。
            */
            if (!core->warmup_completed[algorithm])
            {
                warmup_ticks =(uint32_t)(
                                core->detector[algorithm].previous_tick -
                                core->detector[algorithm].warmup_start_tick);

                /*
                * 必须是可解释的样本时间跨度，并且tick频率有效。
                * 使用实际处理的样本时间，不使用当前墙钟等待时间。
                */
                if ((warmup_ticks < MOTION_CORE_TICK_HALF_RANGE) &&(core->detector[algorithm].config.tick_hz != 0U))
                {
                    if (motion_core_ticks_to_ms(warmup_ticks,
                        core->detector[algorithm].config.tick_hz) >=
                        core->detector[algorithm].config.warmup_ms)
                    {
                        core->warmup_completed[algorithm] = true;
                    }
                }
            }

            /* 任意一个算法尚未完成，摘要仍报告预热。 */
            if (!core->warmup_completed[algorithm])
            {
                warmup = true;
            }
        }

        if (warmup)
        {
            quality_flags |= MOTION_QUALITY_WARMUP;
        }

        if (core->recovering_from_gap)
        {
            quality_flags |= MOTION_QUALITY_SAMPLE_GAP;
        }

        if (core->input_timed_out ||
            (core->source_state == MOTION_SOURCE_UNKNOWN))
        {
            quality_flags |= MOTION_QUALITY_INPUT_STALE;
        }
    }

    /* 数据源质量与服务生命周期分别表达。 */
    if (core->source_state == MOTION_SOURCE_STALE)
    {
        quality_flags |= MOTION_QUALITY_INPUT_STALE;
    }

    if (core->source_state == MOTION_SOURCE_OFFLINE)
    {
        quality_flags |= MOTION_QUALITY_SOURCE_OFFLINE;
    }

    /* 累计统计饱和或业务计数被阻断，都需要对外报告。 */
    if (core->stats.counters_saturated)
    {
        quality_flags |= MOTION_QUALITY_COUNTER_SATURATED;
    }

    for (algorithm = 0U;
         algorithm < MOTION_ALGORITHM_COUNT;
         algorithm++)
    {
        if (core->counting_blocked[algorithm])
        {
            quality_flags |= MOTION_QUALITY_COUNTER_SATURATED;
        }
    }

    core->snapshot.quality_flags = quality_flags;

    /* 当前问题可以解除，历史记录只能增加，不能随之清除。 */
    core->snapshot.latched_quality_flags |= quality_flags;
}