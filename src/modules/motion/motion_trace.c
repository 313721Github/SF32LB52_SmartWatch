#include "motion_trace.h"
#if MOTION_TRACE_ENABLED
#include "rthw.h"
#include <limits.h>
#include <string.h>

#define TRACE_BAD_QUALITY (MOTION_QUALITY_WARMUP | MOTION_QUALITY_SAMPLE_GAP | \
    MOTION_QUALITY_INPUT_STALE | MOTION_QUALITY_SOURCE_OFFLINE | \
    MOTION_QUALITY_CONFIG_MISMATCH | MOTION_QUALITY_COUNTER_SATURATED)

typedef struct
{
    motion_trace_meta_t meta;
    motion_trace_row_t rows[MOTION_TRACE_ROW_CAPACITY];
    bool initialized, frame_pending;
    motion_trace_end_t deferred_end;
    int32_t deferred_error;
} trace_worker_t;

typedef struct
{
    motion_trace_status_t status;
    uint32_t next_lease, active_lease;
    rt_thread_t lease_owner;
} trace_shared_t;

static trace_worker_t g_trace;
static trace_shared_t g_trace_shared;
typedef char trace_worker_budget[(sizeof(trace_worker_t) + sizeof(trace_shared_t) <= 80U * 1024U) ? 1 : -1];

/* 私有函数集中声明；只在motion worker或明确标注的短临界区内访问共享状态。 */
static bool trace_due(uint32_t now, uint32_t deadline);
static uint32_t trace_ticks(uint32_t ms);
static motion_trace_state_t trace_state(void);
static void trace_publish(motion_trace_state_t state);
static bool trace_healthy(const motion_core_t *core);
static void trace_checkpoint(motion_trace_checkpoint_t *dst, const motion_core_t *core);
static void trace_finish(const motion_core_t *core, motion_trace_end_t reason, int32_t error, uint32_t now);
static void trace_measure(uint32_t began);
static bool trace_changed(const motion_core_t *core);

/* 简介：判断有限期限是否到达。输入：now/deadline为32位单调tick。
 * 返回值：到期true。说明：所有期限小于tick半区间，支持一次自然回绕。 */
static bool trace_due(uint32_t now, uint32_t deadline)
{ return (uint32_t)(now - deadline) < UINT32_C(0x80000000); }

/* 简介：向上换算毫秒。输入：有限且非负的ms。
 * 返回值：tick数。说明：0保持0，乘法使用64位。 */
static uint32_t trace_ticks(uint32_t ms)
{ return (uint32_t)(((uint64_t)ms * RT_TICK_PER_SECOND + 999U) / 1000U); }

/* 简介：读取采集小状态。输入：无。
 * 返回值：当前状态。说明：短临界区同步，不读取记录数组。 */
static motion_trace_state_t trace_state(void)
{
    rt_base_t level = rt_hw_interrupt_disable();
    motion_trace_state_t s = g_trace_shared.status.state;
    rt_hw_interrupt_enable(level);
    return s;
}

/* 简介：发布worker已完成的小状态。输入：state为新状态。
 * 返回值：无。说明：大数据先写完，再短临界区发布；导出租约期间worker不发布。 */
static void trace_publish(motion_trace_state_t state)
{
    rt_base_t level = rt_hw_interrupt_disable();
    motion_trace_status_t *s = &g_trace_shared.status;
    const motion_trace_meta_t *m = &g_trace.meta;
    s->ready = true; s->state = state; s->capture_id = m->capture_id;
    s->rows_stored = m->rows_stored; s->capacity = MOTION_TRACE_ROW_CAPACITY;
    s->scheduled_start_tick = m->scheduled_start_tick;
    s->start_process_tick = m->start_process_tick; s->stop_deadline_tick = m->stop_deadline_tick;
    s->end_reason = m->end_reason; s->error = m->last_core_error;
    rt_hw_interrupt_enable(level);
}

/* 简介：检查诊断所需业务基础。输入：core为worker独占实例，只读非空。
 * 返回值：运行、源可用且数据质量合格返回true。说明：未校日历允许，不能据此拒绝采样。 */
static bool trace_healthy(const motion_core_t *core)
{
    return core != RT_NULL && core->initialized && core->snapshot.state == MOTION_SERVICE_RUNNING &&
        core->source_state == MOTION_SOURCE_AVAILABLE && core->snapshot.has_processed &&
        !core->input_timed_out && !core->stats.counters_saturated &&
        (core->snapshot.quality_flags & TRACE_BAD_QUALITY) == 0U;
}

/* 简介：复制回放检查点。输入：dst/core非空，仅本次借用。
 * 返回值：无。说明：worker上下文、锁外复制；不保存内部指针。 */
static void trace_checkpoint(motion_trace_checkpoint_t *dst, const motion_core_t *core)
{
    uint32_t a;
    for (a = 0U; a < MOTION_ALGORITHM_COUNT; a++)
    {
        dst->detector[a] = core->detector[a];
        dst->boot_steps[a] = core->snapshot.totals[a].boot_steps;
        dst->session_steps[a] = core->snapshot.totals[a].session_steps;
    }
    dst->core_stats = core->stats;
    dst->session_id = core->snapshot.session_id; dst->run_generation = core->snapshot.run_generation;
    dst->source_state = (uint32_t)core->source_state; dst->quality_flags = core->snapshot.quality_flags;
    dst->previous_imu_seq = core->previous_imu_seq; dst->previous_sample_tick = core->previous_sample_tick;
    dst->has_previous_seq = core->has_previous_seq; dst->has_previous_tick = core->has_previous_tick;
}

/* 简介：识别帧外重置或会话切换。输入：core只读非空。
 * 返回值：采集起点以来发生重置/代际变化时true。说明：异常片段保留，不能静默当连续数据。 */
static bool trace_changed(const motion_core_t *core)
{
    uint32_t a;
    if (core->snapshot.session_id != g_trace.meta.before.session_id ||
        core->snapshot.run_generation != g_trace.meta.before.run_generation) return true;
    for (a = 0U; a < MOTION_ALGORITHM_COUNT; a++)
        if (core->stats.algorithm[a].reset_events != g_trace.meta.before.core_stats.algorithm[a].reset_events)
            return true;
    return false;
}

/* 简介：冻结元数据与数组。输入：core非空只读，reason/error为实际结束原因。
 * 返回值：无。说明：先完成after检查点再发布FROZEN；无起点时不伪造完整采集。 */
static void trace_finish(const motion_core_t *core, motion_trace_end_t reason, int32_t error, uint32_t now)
{
    motion_trace_meta_t *m = &g_trace.meta;
    uint32_t a;
    bool closed;
    motion_trace_state_t s = trace_state();
    if (s != MOTION_TRACE_ARMED && s != MOTION_TRACE_RECORDING) return;
    if (g_trace.frame_pending)
    {
        /* 帧内先保留首个异常，完成该行后再冻结，后续异常不得改写根因。 */
        if (g_trace.deferred_end == MOTION_TRACE_END_NONE)
        { g_trace.deferred_end = reason; g_trace.deferred_error = error; }
        return;
    }
    trace_checkpoint(&m->after, core);
    m->end_process_tick = now; m->end_reason = reason; m->last_core_error = error;
    closed = m->start_valid && m->rows_not_stored == 0U && m->rows_stored > 0U &&
        core->stats.dequeued >= m->before.core_stats.dequeued &&
        core->stats.dequeued - m->before.core_stats.dequeued == m->rows_stored &&
        core->stats.passed_to_alg >= m->before.core_stats.passed_to_alg &&
        core->stats.passed_to_alg - m->before.core_stats.passed_to_alg == m->rows_stored;
    for (a = 0U; a < MOTION_ALGORITHM_COUNT; a++)
        closed = closed && core->stats.algorithm[a].process_calls >= m->before.core_stats.algorithm[a].process_calls &&
            core->stats.algorithm[a].process_calls - m->before.core_stats.algorithm[a].process_calls == m->rows_stored;
    m->continuous = closed && trace_healthy(core) && !trace_changed(core);
    m->completed = reason == MOTION_TRACE_END_DURATION && m->continuous && trace_due(now, m->stop_deadline_tick);
    trace_publish(MOTION_TRACE_FROZEN);
}

/* 简介：保存诊断钩子耗时高水位。输入：began为进入钩子的tick。
 * 返回值：无。说明：仅worker；0tick表示低于当前时钟分辨率。 */
static void trace_measure(uint32_t began)
{
    uint32_t d = (uint32_t)rt_tick_get() - began;
    if (d > g_trace.meta.max_hook_ticks) g_trace.meta.max_hook_ticks = d;
}

/* 简介：建立采集器初始状态。输入：无。
 * 返回值：无。说明：worker启动前调用；重复调用不清已有证据，不擦除大数组。 */
void motion_trace_worker_init(void)
{
    if (g_trace.initialized) return;
    g_trace.initialized = true;
    trace_publish(MOTION_TRACE_EMPTY);
}

/* 简介：执行原控制队列中的采集请求。输入：core非空只读，参数仅本次有效。
 * 返回值：OK成功，EINVAL参数错，EBUSY状态冲突。说明：worker唯一写者，不修改业务算法。 */
rt_err_t motion_trace_worker_control(motion_request_kind_t kind, uint32_t request_id,
    uint32_t argument, const motion_core_t *core, bool injection_active, bool stop_pending, uint32_t now)
{
    uint32_t duration = argument & UINT32_C(0xFFFF), delay = argument >> 16;
    rt_base_t level;
    motion_trace_state_t s;
    if (!g_trace.initialized || core == RT_NULL || request_id == 0U) return -RT_EINVAL;
    if (kind == MOTION_REQUEST_TRACE_ARM)
    {
        if (duration < 1U || duration > MOTION_TRACE_MAX_SECONDS || delay > MOTION_TRACE_MAX_DELAY_SECONDS) return -RT_EINVAL;
        if (!trace_healthy(core) || injection_active || stop_pending) return -RT_EBUSY;
        level = rt_hw_interrupt_disable();
        if (g_trace_shared.status.state != MOTION_TRACE_EMPTY || g_trace_shared.active_lease != 0U)
        { rt_hw_interrupt_enable(level); return -RT_EBUSY; }
        g_trace_shared.status.state = MOTION_TRACE_CONTROL_BUSY;
        rt_hw_interrupt_enable(level);
        memset(&g_trace.meta, 0, sizeof(g_trace.meta));
        g_trace.frame_pending = false; g_trace.deferred_end = MOTION_TRACE_END_NONE;
        g_trace.deferred_error = 0;
        g_trace.meta.format_version = MOTION_TRACE_FORMAT_VERSION;
        g_trace.meta.capture_id = request_id; g_trace.meta.tick_hz = RT_TICK_PER_SECOND;
        g_trace.meta.row_size = sizeof(motion_trace_row_t); g_trace.meta.capacity = MOTION_TRACE_ROW_CAPACITY;
        g_trace.meta.duration_ms = duration * 1000U; g_trace.meta.delay_ms = delay * 1000U;
        g_trace.meta.armed_tick = now; g_trace.meta.scheduled_start_tick = now + trace_ticks(delay * 1000U);
        trace_publish(MOTION_TRACE_ARMED);
        return RT_EOK;
    }
    if (kind != MOTION_REQUEST_TRACE_CANCEL && kind != MOTION_REQUEST_TRACE_CLEAR) return -RT_EINVAL;
    level = rt_hw_interrupt_disable();
    s = g_trace_shared.status.state;
    if (argument == 0U || argument != g_trace_shared.status.capture_id)
    { rt_hw_interrupt_enable(level); return -RT_EINVAL; }
    if (kind == MOTION_REQUEST_TRACE_CLEAR)
    {
        if (s != MOTION_TRACE_FROZEN || g_trace_shared.active_lease != 0U)
        { rt_hw_interrupt_enable(level); return -RT_EBUSY; }
        g_trace_shared.status.state = MOTION_TRACE_CONTROL_BUSY;
        rt_hw_interrupt_enable(level);
        memset(&g_trace.meta, 0, sizeof(g_trace.meta));
        trace_publish(MOTION_TRACE_EMPTY);
        return RT_EOK;
    }
    rt_hw_interrupt_enable(level);
    if (s != MOTION_TRACE_ARMED && s != MOTION_TRACE_RECORDING) return -RT_EBUSY;
    motion_trace_worker_abort(core, MOTION_TRACE_END_USER_CANCEL, 0, now);
    return RT_EOK;
}

/* 简介：推进采集期限和健康检查。输入：core非空只读，其余为当前worker状态。
 * 返回值：无。说明：无帧时也必须调用，诊断结束不引起业务FAULT。 */
void motion_trace_worker_poll(const motion_core_t *core, bool injection_active, bool stop_pending, uint32_t now)
{
    motion_trace_state_t s = trace_state();
    if (core == RT_NULL || (s != MOTION_TRACE_ARMED && s != MOTION_TRACE_RECORDING)) return;
    if (injection_active) { motion_trace_worker_abort(core, MOTION_TRACE_END_TEST_CONFLICT, -RT_EBUSY, now); return; }
    if (stop_pending || core->snapshot.state == MOTION_SERVICE_STOPPED)
    { motion_trace_worker_abort(core, MOTION_TRACE_END_MOTION_STOP, 0, now); return; }
    if (!trace_healthy(core) || (s == MOTION_TRACE_RECORDING && trace_changed(core)))
    { motion_trace_worker_abort(core, MOTION_TRACE_END_DATA_QUALITY, -RT_ERROR, now); return; }
    if (s == MOTION_TRACE_ARMED && trace_due(now, g_trace.meta.scheduled_start_tick + trace_ticks(MOTION_TRACE_FIRST_FRAME_WAIT_MS)))
        trace_finish(core, MOTION_TRACE_END_NO_INPUT, -RT_ETIMEOUT, now);
    else if (s == MOTION_TRACE_RECORDING && trace_due(now, g_trace.meta.stop_deadline_tick))
        trace_finish(core, MOTION_TRACE_END_DURATION, 0, now);
}

/* 简介：为真实出队帧准备一行。输入：core/frame非空只读，地址仅本次有效。
 * 返回值：无。说明：在核心调用前执行，锁外复制；只记录正式消费流。 */
void motion_trace_worker_before_frame(const motion_core_t *core, const motion_sample_frame_t *frame, uint32_t now)
{
    motion_trace_state_t s = trace_state();
    motion_trace_meta_t *m = &g_trace.meta;
    motion_trace_row_t *row;
    const step_detector_t *b1;
    uint32_t began = (uint32_t)rt_tick_get();
    if (core == RT_NULL || frame == RT_NULL || (s != MOTION_TRACE_ARMED && s != MOTION_TRACE_RECORDING)) return;
    if (s == MOTION_TRACE_ARMED)
    {
        if (!trace_due(now, m->scheduled_start_tick)) return;
        if (!trace_healthy(core)) { trace_finish(core, MOTION_TRACE_END_DATA_QUALITY, -RT_ERROR, now); return; }
        trace_checkpoint(&m->before, core);
        m->start_valid = true; m->start_process_tick = now;
        m->stop_deadline_tick = now + trace_ticks(m->duration_ms);
        m->first_sample_tick = frame->imu.sample_tick; m->initial_imu_config = frame->imu.config_version;
        trace_publish(MOTION_TRACE_RECORDING);
    }
    else if (!trace_healthy(core) || trace_changed(core))
    { trace_finish(core, MOTION_TRACE_END_DATA_QUALITY, -RT_ERROR, now); return; }
    if (trace_due(now, m->stop_deadline_tick)) { trace_finish(core, MOTION_TRACE_END_DURATION, 0, now); return; }
    if (g_trace.frame_pending) { motion_trace_worker_abort(core, MOTION_TRACE_END_DATA_QUALITY, -RT_ERROR, now); return; }
    m->rows_seen++;
    if (m->rows_stored >= MOTION_TRACE_ROW_CAPACITY)
    {
        m->rows_not_stored++; m->bad_input_seq = frame->imu.imu_seq;
        m->bad_input_config = frame->imu.config_version; m->bad_input_generation = frame->run_generation;
        trace_finish(core, MOTION_TRACE_END_OVERFLOW, -RT_EFULL, now); return;
    }
    row = &g_trace.rows[m->rows_stored]; memset(row, 0, sizeof(*row));
    row->sample_tick = frame->imu.sample_tick; row->imu_seq = frame->imu.imu_seq;
    row->enqueue_tick = frame->enqueue_tick; row->process_tick = now;
    memcpy(row->accel_ug, frame->imu.accel_ug, sizeof(row->accel_ug));
    if (frame->imu.valid) row->flags |= MTR_F_IMU_VALID;
    b1 = &core->detector[MOTION_ALGORITHM_B1];
    row->b1_last_event_before = b1->last_event_tick;
    if (b1->armed) row->flags |= MTR_F_B1_ARMED_BEFORE;
    if (b1->has_last_event) row->flags |= MTR_F_B1_HAS_LAST_BEFORE;
    if (b1->initialized) row->flags |= MTR_F_B1_INIT_BEFORE;
    g_trace.frame_pending = true;
    trace_measure(began);
}

/* 简介：提交同一帧实际输出。输入：core/frame非空，result只在核心成功分类时非空。
 * 返回值：无。说明：异常帧先保留再冻结，不改正式分类或错误处理。 */
void motion_trace_worker_after_frame(const motion_core_t *core, const motion_sample_frame_t *frame,
    bool called, int core_result, const motion_core_frame_result_t *result, bool injected, uint32_t now)
{
    motion_trace_meta_t *m = &g_trace.meta;
    motion_trace_row_t *row;
    const step_detector_t *b1;
    uint32_t a, detail = injected ? 7U : 8U, flags, span;
    uint32_t began = (uint32_t)rt_tick_get();
    bool good = called && core_result == MOTION_CORE_OK && result != RT_NULL;
    if (!g_trace.frame_pending || core == RT_NULL || frame == RT_NULL) return;
    row = &g_trace.rows[m->rows_stored]; flags = row->flags;
    b1 = &core->detector[MOTION_ALGORITHM_B1]; row->filtered_ug = b1->filtered_dynamic_ug;
    if (b1->armed) flags |= MTR_F_B1_ARMED_AFTER;
    if (b1->has_last_event) flags |= MTR_F_B1_HAS_LAST_AFTER;
    if (b1->initialized) flags |= MTR_F_B1_INIT_AFTER;
    if (injected) flags |= MTR_F_INJECTED;
    if (good)
    {
        flags |= MTR_F_CORE_OK; detail = (uint32_t)result->disposition & 15U;
        detail |= (result->reset_reasons & UINT32_C(0xFFFF)) << 16;
        if (result->fatal_error) flags |= MTR_F_FATAL;
        for (a = 0U; a < MOTION_ALGORITHM_COUNT; a++)
        {
            const step_detector_output_t *o = &result->algorithm_output[a];
            if (!result->algorithm_called[a]) { good = false; continue; }
            flags |= MTR_F_B0_CALLED << a;
            if (o->sample_accepted) flags |= MTR_F_B0_ACCEPTED << a;
            if (o->event_generated) flags |= MTR_F_B0_EVENT << a;
            if (o->state_reset) flags |= MTR_F_B0_RESET << a;
            detail |= ((uint32_t)o->reason & 15U) << (4U + 4U * a);
            detail |= ((uint32_t)(result->algorithm_result[a] + 1) & 3U) << (12U + 2U * a);
            if (!o->sample_accepted || o->state_reset ||
                (result->algorithm_result[a] != STEP_DETECTOR_PROCESS_ACCEPTED && result->algorithm_result[a] != STEP_DETECTOR_PROCESS_EVENT)) good = false;
        }
        if (result->algorithm_called[MOTION_ALGORITHM_B1] && result->algorithm_output[MOTION_ALGORITHM_B1].sample_accepted)
        { row->dynamic_ug = result->algorithm_output[MOTION_ALGORITHM_B1].dynamic_ug; flags |= MTR_F_DYNAMIC_VALID; }
        if (result->disposition != MOTION_FRAME_PASSED_TO_ALGORITHM || result->reset_reasons != 0U || result->fatal_error) good = false;
    }
    if (!trace_healthy(core) || trace_changed(core) || frame->run_generation != m->before.run_generation || frame->imu.config_version != m->initial_imu_config) good = false;
    row->flags = flags; row->detail = detail;
    m->rows_stored++; m->last_sample_tick = frame->imu.sample_tick;
    g_trace.frame_pending = false;
    span = (uint32_t)rt_tick_get() - row->process_tick;
    if (span > m->max_frame_ticks) m->max_frame_ticks = span;
    trace_measure(began);
    if (!good)
    {
        m->bad_input_seq = frame->imu.imu_seq; m->bad_input_config = frame->imu.config_version;
        m->bad_input_generation = frame->run_generation;
    }
    if (g_trace.deferred_end != MOTION_TRACE_END_NONE)
    { trace_finish(core, g_trace.deferred_end, g_trace.deferred_error, now); return; }
    if (!good) { trace_finish(core, MOTION_TRACE_END_DATA_QUALITY, called ? core_result : -RT_ERROR, now); return; }
    trace_publish(MOTION_TRACE_RECORDING);
}

/* 简介：结束活动采集。输入：core只读，reason/error为真实原因。
 * 返回值：无。说明：帧内故障延迟冻结；冻结/导出状态幂等，不改已保存数据。 */
void motion_trace_worker_abort(const motion_core_t *core, motion_trace_end_t reason, int32_t error, uint32_t now)
{ if (core != RT_NULL) trace_finish(core, reason, error, now); }

/* 简介：判断活动采集。输入：无。
 * 返回值：ARMED/RECORDING为true。说明：worker用来拒绝新故障注入。 */
bool motion_trace_worker_is_active(void)
{ motion_trace_state_t s = trace_state(); return s == MOTION_TRACE_ARMED || s == MOTION_TRACE_RECORDING; }

/* 简介：复制小状态。输入：status非空，调用期间有效。
 * 返回值：OK成功或EINVAL/ERROR。说明：线程调用，短临界区，不读取活core。 */
rt_err_t motion_trace_get_status(motion_trace_status_t *status)
{
    rt_base_t level;
    if (status == RT_NULL) return -RT_EINVAL;
    level = rt_hw_interrupt_disable(); *status = g_trace_shared.status; rt_hw_interrupt_enable(level);
    return status->ready ? RT_EOK : -RT_ERROR;
}

/* 简介：取得冻结数据独占读取租约。输入：id匹配，lease/meta非空，失败lease=0。
 * 返回值：OK或EINVAL/EBUSY/EFULL。说明：仅线程；大元数据复制在锁外，禁止CLEAR并发覆盖。 */
rt_err_t motion_trace_export_begin(uint32_t id, uint32_t *lease, motion_trace_meta_t *meta)
{
    rt_base_t level;
    if (lease == RT_NULL || meta == RT_NULL) return -RT_EINVAL;
    *lease = 0U;
    level = rt_hw_interrupt_disable();
    if (id == 0U || id != g_trace_shared.status.capture_id)
    { rt_hw_interrupt_enable(level); return -RT_EINVAL; }
    if (g_trace_shared.status.state != MOTION_TRACE_FROZEN || g_trace_shared.active_lease != 0U)
    { rt_hw_interrupt_enable(level); return -RT_EBUSY; }
    if (g_trace_shared.next_lease == UINT32_MAX)
    { rt_hw_interrupt_enable(level); return -RT_EFULL; }
    g_trace_shared.active_lease = ++g_trace_shared.next_lease;
    g_trace_shared.lease_owner = rt_thread_self();
    *lease = g_trace_shared.active_lease;
    g_trace_shared.status.state = MOTION_TRACE_EXPORTING;
    rt_hw_interrupt_enable(level);
    *meta = g_trace.meta;
    return RT_EOK;
}

/* 简介：复制租约内的一行。输入：lease归当前线程，index合法，row非空。
 * 返回值：OK或EINVAL。说明：锁外复制不可变数据；其他线程不能结束本租约。 */
rt_err_t motion_trace_export_get_row(uint32_t lease, uint32_t index, motion_trace_row_t *row)
{
    rt_base_t level;
    bool valid;
    if (row == RT_NULL) return -RT_EINVAL;
    level = rt_hw_interrupt_disable();
    valid = lease != 0U && g_trace_shared.active_lease == lease &&
        g_trace_shared.lease_owner == rt_thread_self() && g_trace_shared.status.state == MOTION_TRACE_EXPORTING;
    rt_hw_interrupt_enable(level);
    if (!valid || index >= g_trace.meta.rows_stored) return -RT_EINVAL;
    *row = g_trace.rows[index];
    return RT_EOK;
}

/* 简介：释放当前线程租约。输入：lease为当前有效编号。
 * 返回值：OK或EINVAL。说明：恢复FROZEN且保留数据，失败导出也必须调用。 */
rt_err_t motion_trace_export_end(uint32_t lease)
{
    rt_base_t level = rt_hw_interrupt_disable();
    if (lease == 0U || g_trace_shared.active_lease != lease || g_trace_shared.lease_owner != rt_thread_self())
    { rt_hw_interrupt_enable(level); return -RT_EINVAL; }
    g_trace_shared.active_lease = 0U; g_trace_shared.lease_owner = RT_NULL;
    g_trace_shared.status.state = MOTION_TRACE_FROZEN;
    rt_hw_interrupt_enable(level);
    return RT_EOK;
}
#endif
