#include "motion_trace.h"
#if MOTION_TRACE_ENABLED && defined(RT_USING_FINSH)
#include <finsh.h>
#include "rthw.h"
#include "motion_service.h"
#include <stdarg.h>
#include <string.h>

typedef struct
{
    bool busy, crc_active;
    uint32_t crc;
    motion_trace_status_t status;
    motion_trace_meta_t meta;
    motion_trace_row_t row;
    motion_request_result_t result;
    char line[512], decimal[24];
} trace_shell_t;
static trace_shell_t g_trace_shell;
typedef char trace_shell_budget[(sizeof(trace_shell_t) <= 2U * 1024U) ? 1 : -1];

static rt_err_t trace_parse(const char *text, uint32_t max, uint32_t *value);
static uint32_t trace_crc(uint32_t crc, const void *bytes, rt_size_t length);
static rt_err_t trace_out(const char *format, ...);
static const char *trace_decimal(uint64_t value);
static rt_err_t trace_u64(uint32_t id, const char *phase, const char *owner,
    int algorithm, const char *name, uint64_t value);
static rt_err_t trace_emit_checkpoint(uint32_t id, const char *phase, const motion_trace_checkpoint_t *cp);
static rt_err_t trace_meta(uint32_t id);
static rt_err_t trace_dump(uint32_t id, uint32_t first, uint32_t count);
static int motion_trace(int argc, char **argv);

/* 简介：严格解析十进制无符号整数。输入：text/value非空，max为上限。
 * 返回值：OK或EINVAL。说明：拒绝符号、空串、尾字符及溢出，失败不改输出。 */
static rt_err_t trace_parse(const char *text, uint32_t max, uint32_t *value)
{
    uint32_t n = 0U, digit;
    const char *p;
    if (text == RT_NULL || value == RT_NULL || *text == '\0') return -RT_EINVAL;
    for (p = text; *p != '\0'; p++)
    {
        if (*p < '0' || *p > '9') return -RT_EINVAL;
        digit = (uint32_t)(*p - '0');
        if (digit > max || n > (max - digit) / 10U) return -RT_EINVAL;
        n = n * 10U + digit;
    }
    *value = n; return RT_EOK;
}

/* 简介：更新IEEE CRC32。输入：bytes在调用期间有效，length为长度。
 * 返回值：内部CRC。说明：初值FFFFFFFF，结束异或FFFFFFFF；仅Shell计算。 */
static uint32_t trace_crc(uint32_t crc, const void *bytes, rt_size_t length)
{
    const unsigned char *p = (const unsigned char *)bytes;
    rt_size_t i;
    uint32_t bit;
    for (i = 0U; i < length; i++)
    {
        crc ^= p[i];
        for (bit = 0U; bit < 8U; bit++) crc = (crc >> 1) ^ ((crc & 1U) ? UINT32_C(0xEDB88320) : 0U);
    }
    return crc;
}

/* 简介：提交一条完整协议行。输入：format非空，参数仅本次借用。
 * 返回值：OK、EFULL截断或实际I/O错误。说明：Shell上下文、不持采集锁，检查短写，不重设UART。 */
static rt_err_t trace_out(const char *format, ...)
{
    va_list args;
    int length;
    rt_device_t dev;
    rt_size_t written;
    va_start(args, format);
    length = rt_vsnprintf(g_trace_shell.line, sizeof(g_trace_shell.line), format, args);
    va_end(args);
    if (length < 0 || (rt_size_t)length >= sizeof(g_trace_shell.line)) return -RT_EFULL;
    dev = rt_console_get_device();
    if (dev == RT_NULL) return -RT_ERROR;
    written = rt_device_write(dev, 0, g_trace_shell.line, (rt_size_t)length);
    if (written != (rt_size_t)length) return -RT_EIO;
    if (g_trace_shell.crc_active) g_trace_shell.crc = trace_crc(g_trace_shell.crc, g_trace_shell.line, (rt_size_t)length);
    return RT_EOK;
}

/* 简介：把64位计数转换为十进制。输入：value为任意uint64值。
 * 返回值：Shell静态字符串，下次转换前有效。说明：不依赖SDK printf的long long支持，不可重入。 */
static const char *trace_decimal(uint64_t value)
{
    uint32_t used = 0U, i;
    char ch;
    do { g_trace_shell.decimal[used++] = (char)('0' + value % 10U); value /= 10U; } while (value != 0U);
    for (i = 0U; i < used / 2U; i++)
    { ch = g_trace_shell.decimal[i]; g_trace_shell.decimal[i] = g_trace_shell.decimal[used - i - 1U]; g_trace_shell.decimal[used - i - 1U] = ch; }
    g_trace_shell.decimal[used] = '\0'; return g_trace_shell.decimal;
}

/* 简介：输出带名称的64位检查点计数。输入：字符串为调用期有效常量。
 * 返回值：实际输出结果。说明：只在Shell独占工作区和导出租约内调用。 */
static rt_err_t trace_u64(uint32_t id, const char *phase, const char *owner,
    int algorithm, const char *name, uint64_t value)
{ return trace_out("#TRACE_U64,%lu,%s,%s,%d,%s,%s\n", (unsigned long)id, phase, owner, algorithm, name, trace_decimal(value)); }

#define OUT(...) do { r = trace_out(__VA_ARGS__); if (r != RT_EOK) goto done; } while (0)
#define U32(v) ((unsigned long)(v))

/* 简介：逐字段导出完整回放检查点。输入：cp非空只读，phase为before/after。
 * 返回值：实际输出结果。说明：不用结构体裸字节，完整保留两个算法的参数、阶段和诊断计数。 */
static rt_err_t trace_emit_checkpoint(uint32_t id, const char *phase, const motion_trace_checkpoint_t *cp)
{
    uint32_t a;
    rt_err_t r = RT_EOK;
    OUT("#TRACE_CP,%lu,%s,%lu,%lu,%lu,%lu,%u,%lu,%u,%lu,%u\n", U32(id), phase,
        U32(cp->session_id), U32(cp->run_generation), U32(cp->source_state), U32(cp->quality_flags),
        (unsigned)cp->has_previous_seq, U32(cp->previous_imu_seq), (unsigned)cp->has_previous_tick,
        U32(cp->previous_sample_tick), (unsigned)cp->core_stats.counters_saturated);
    for (a = 0U; a < MOTION_ALGORITHM_COUNT; a++)
    {
        const step_detector_t *d = &cp->detector[a];
        const step_detector_config_t *c = &d->config;
        OUT("#TRACE_CONFIG,%lu,%s,%lu,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n", U32(id), phase, U32(a),
            (unsigned)c->algorithm, U32(c->high_threshold_ug), U32(c->low_threshold_ug), U32(c->refractory_ms),
            U32(c->ewma_tau_ms), U32(c->warmup_ms), U32(c->reset_gap_ms), U32(c->tick_hz), U32(c->expected_config_version));
        OUT("#TRACE_STATE,%lu,%s,%lu,%u,%u,%u,%u,%u,%lu,%lu,%lu,%lu,%lu,%lu\n", U32(id), phase, U32(a),
            (unsigned)d->configured, (unsigned)d->initialized, (unsigned)d->armed,
            (unsigned)d->has_previous_seq, (unsigned)d->has_last_event, U32(d->previous_tick),
            U32(d->previous_imu_seq), U32(d->warmup_start_tick), U32(d->last_event_tick),
            U32(d->filtered_dynamic_ug), U32(d->step_count));
        OUT("#TRACE_DETECTOR_COUNTERS,%lu,%s,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n", U32(id), phase, U32(a),
            U32(d->accepted_count), U32(d->duplicate_count), U32(d->seq_gap_count), U32(d->reset_count),
            U32(d->invalid_count), U32(d->stale_count), U32(d->config_mismatch_count), U32(d->sequence_rollback_count));
        r = trace_u64(id, phase, "business", (int)a, "boot_steps", cp->boot_steps[a]); if (r != RT_EOK) goto done;
        r = trace_u64(id, phase, "business", (int)a, "session_steps", cp->session_steps[a]); if (r != RT_EOK) goto done;
        r = trace_u64(id, phase, "algorithm", (int)a, "process_calls", cp->core_stats.algorithm[a].process_calls); if (r != RT_EOK) goto done;
        r = trace_u64(id, phase, "algorithm", (int)a, "accepted_no_event", cp->core_stats.algorithm[a].accepted_no_event); if (r != RT_EOK) goto done;
        r = trace_u64(id, phase, "algorithm", (int)a, "events", cp->core_stats.algorithm[a].events); if (r != RT_EOK) goto done;
        r = trace_u64(id, phase, "algorithm", (int)a, "ignored", cp->core_stats.algorithm[a].ignored); if (r != RT_EOK) goto done;
        r = trace_u64(id, phase, "algorithm", (int)a, "errors", cp->core_stats.algorithm[a].errors); if (r != RT_EOK) goto done;
        r = trace_u64(id, phase, "algorithm", (int)a, "reset_events", cp->core_stats.algorithm[a].reset_events); if (r != RT_EOK) goto done;

    }
    r = trace_u64(id, phase, "core", -1, "dequeued", cp->core_stats.dequeued); if (r != RT_EOK) goto done;
    r = trace_u64(id, phase, "core", -1, "discard_old_generation", cp->core_stats.discard_old_generation); if (r != RT_EOK) goto done;
    r = trace_u64(id, phase, "core", -1, "discard_stopped", cp->core_stats.discard_stopped); if (r != RT_EOK) goto done;
    r = trace_u64(id, phase, "core", -1, "discard_invalid", cp->core_stats.discard_invalid); if (r != RT_EOK) goto done;
    r = trace_u64(id, phase, "core", -1, "discard_stale", cp->core_stats.discard_stale); if (r != RT_EOK) goto done;
    r = trace_u64(id, phase, "core", -1, "discard_config", cp->core_stats.discard_config); if (r != RT_EOK) goto done;
    r = trace_u64(id, phase, "core", -1, "discard_duplicate", cp->core_stats.discard_duplicate); if (r != RT_EOK) goto done;
    r = trace_u64(id, phase, "core", -1, "passed_to_alg", cp->core_stats.passed_to_alg); if (r != RT_EOK) goto done;
    r = trace_u64(id, phase, "core", -1, "seq_gap_events", cp->core_stats.seq_gap_events); if (r != RT_EOK) goto done;
    r = trace_u64(id, phase, "core", -1, "seq_missing_samples", cp->core_stats.seq_missing_samples); if (r != RT_EOK) goto done;
    r = trace_u64(id, phase, "core", -1, "seq_rollback_events", cp->core_stats.seq_rollback_events); if (r != RT_EOK) goto done;
    r = trace_u64(id, phase, "core", -1, "time_gap_events", cp->core_stats.time_gap_events); if (r != RT_EOK) goto done;
    r = trace_u64(id, phase, "core", -1, "source_offline_events", cp->core_stats.source_offline_events); if (r != RT_EOK) goto done;
    r = trace_u64(id, phase, "core", -1, "source_recovery_events", cp->core_stats.source_recovery_events); if (r != RT_EOK) goto done;
    r = trace_u64(id, phase, "core", -1, "calendar_change_events", cp->core_stats.calendar_change_events); if (r != RT_EOK) goto done;
    r = trace_u64(id, phase, "core", -1, "day_bucket_evictions", cp->core_stats.day_bucket_evictions); if (r != RT_EOK) goto done;

done:
    return r;
}

/* 简介：导出冻结元数据。输入：id为目标采集ID。
 * 返回值：OK或实际错误。说明：任何失败均释放租约，不清数据；CRC覆盖BEGIN至最后数据行的LF字节。 */
static rt_err_t trace_meta(uint32_t id)
{
    uint32_t lease = 0U;
    rt_err_t r, cleanup;
    motion_trace_meta_t *m = &g_trace_shell.meta;
    r = motion_trace_export_begin(id, &lease, m); if (r != RT_EOK) return r;
    g_trace_shell.crc = UINT32_C(0xFFFFFFFF); g_trace_shell.crc_active = true;
    OUT("#TRACE_META_BEGIN,%lu,%lu,%lu,%lu\n", U32(id), U32(m->format_version), U32(m->row_size), U32(m->rows_stored));
    OUT("#TRACE_META,%lu,tick_hz,%lu\n", U32(id), U32(m->tick_hz));
    OUT("#TRACE_META,%lu,capacity,%lu\n", U32(id), U32(m->capacity));
    OUT("#TRACE_META,%lu,duration_ms,%lu\n", U32(id), U32(m->duration_ms));
    OUT("#TRACE_META,%lu,delay_ms,%lu\n", U32(id), U32(m->delay_ms));
    OUT("#TRACE_META,%lu,armed_tick,%lu\n", U32(id), U32(m->armed_tick));
    OUT("#TRACE_META,%lu,scheduled_start_tick,%lu\n", U32(id), U32(m->scheduled_start_tick));
    OUT("#TRACE_META,%lu,start_process_tick,%lu\n", U32(id), U32(m->start_process_tick));
    OUT("#TRACE_META,%lu,stop_deadline_tick,%lu\n", U32(id), U32(m->stop_deadline_tick));
    OUT("#TRACE_META,%lu,end_process_tick,%lu\n", U32(id), U32(m->end_process_tick));
    OUT("#TRACE_META,%lu,first_sample_tick,%lu\n", U32(id), U32(m->first_sample_tick));
    OUT("#TRACE_META,%lu,last_sample_tick,%lu\n", U32(id), U32(m->last_sample_tick));
    OUT("#TRACE_META,%lu,initial_imu_config,%lu\n", U32(id), U32(m->initial_imu_config));
    OUT("#TRACE_META,%lu,rows_seen,%lu\n", U32(id), U32(m->rows_seen));
    OUT("#TRACE_META,%lu,rows_stored,%lu\n", U32(id), U32(m->rows_stored));
    OUT("#TRACE_META,%lu,rows_not_stored,%lu\n", U32(id), U32(m->rows_not_stored));
    OUT("#TRACE_META,%lu,max_hook_ticks,%lu\n", U32(id), U32(m->max_hook_ticks));
    OUT("#TRACE_META,%lu,max_frame_ticks,%lu\n", U32(id), U32(m->max_frame_ticks));
    OUT("#TRACE_META,%lu,end_reason,%lu\n", U32(id), U32(m->end_reason));
    OUT("#TRACE_META,%lu,start_valid,%lu\n", U32(id), U32(m->start_valid));
    OUT("#TRACE_META,%lu,continuous,%lu\n", U32(id), U32(m->continuous));
    OUT("#TRACE_META,%lu,completed,%lu\n", U32(id), U32(m->completed));
    OUT("#TRACE_META,%lu,bad_input_seq,%lu\n", U32(id), U32(m->bad_input_seq));
    OUT("#TRACE_META,%lu,bad_input_config,%lu\n", U32(id), U32(m->bad_input_config));
    OUT("#TRACE_META,%lu,bad_input_generation,%lu\n", U32(id), U32(m->bad_input_generation));

    OUT("#TRACE_META,%lu,last_core_error,%ld\n", U32(id), (long)m->last_core_error);
    OUT("#TRACE_META,%lu,algorithm_version,%lu\n", U32(id), U32(STEP_DETECTOR_ALGORITHM_VERSION));
    OUT("#TRACE_META,%lu,config_version,%lu\n", U32(id), U32(STEP_DETECTOR_TASK10_CONFIG_VERSION));
    r = trace_emit_checkpoint(id, "before", &m->before); if (r != RT_EOK) goto done;
    r = trace_emit_checkpoint(id, "after", &m->after); if (r != RT_EOK) goto done;
    g_trace_shell.crc_active = false;
    OUT("#TRACE_META_END,%lu,%08lx\n", U32(id), U32(g_trace_shell.crc ^ UINT32_C(0xFFFFFFFF)));
done:
    g_trace_shell.crc_active = false;
    cleanup = motion_trace_export_end(lease);
    return r == RT_EOK ? cleanup : r;
}

/* 简介：导出有限记录块。输入：id匹配，count为1～64且范围合法。
 * 返回值：OK或实际错误。说明：每行复制后输出，失败保留数据，可重导同一块；Shell定期让出CPU。 */
static rt_err_t trace_dump(uint32_t id, uint32_t first, uint32_t count)
{
    uint32_t lease = 0U, i;
    rt_err_t r, cleanup;
    motion_trace_row_t *q = &g_trace_shell.row;
    if (count == 0U || count > MOTION_TRACE_EXPORT_MAX_ROWS) return -RT_EINVAL;
    r = motion_trace_export_begin(id, &lease, &g_trace_shell.meta); if (r != RT_EOK) return r;
    if (first > g_trace_shell.meta.rows_stored || count > g_trace_shell.meta.rows_stored - first)
    { r = -RT_EINVAL; goto done; }
    g_trace_shell.crc = UINT32_C(0xFFFFFFFF); g_trace_shell.crc_active = true;
    OUT("#TRACE_CHUNK_BEGIN,%lu,%lu,%lu,%lu\n", U32(id), U32(first), U32(count), U32(MOTION_TRACE_FORMAT_VERSION));
    for (i = 0U; i < count; i++)
    {
        r = motion_trace_export_get_row(lease, first + i, q); if (r != RT_EOK) goto done;
        OUT("R,%lu,%lu,%lu,%lu,%lu,%lu,%ld,%ld,%ld,%lu,%lu,%lu,%lu,%lu\n", U32(id), U32(first+i),
            U32(q->sample_tick), U32(q->imu_seq), U32(q->enqueue_tick), U32(q->process_tick),
            (long)q->accel_ug[0], (long)q->accel_ug[1], (long)q->accel_ug[2], U32(q->dynamic_ug),
            U32(q->filtered_ug), U32(q->b1_last_event_before), U32(q->flags), U32(q->detail));
        if ((i & 7U) == 7U) rt_thread_delay(1);
    }
    g_trace_shell.crc_active = false;
    OUT("#TRACE_CHUNK_END,%lu,%lu,%lu,%08lx\n", U32(id), U32(first), U32(count), U32(g_trace_shell.crc ^ UINT32_C(0xFFFFFFFF)));
done:
    g_trace_shell.crc_active = false;
    cleanup = motion_trace_export_end(lease);
    return r == RT_EOK ? cleanup : r;
}

/* 简介：分派RAM诊断命令。输入：FinSH参数，地址只在调用期有效。
 * 返回值：0或实际负错误。说明：独占Shell工作区，受理不等于执行，所有退出路径清busy。 */
static int motion_trace(int argc, char **argv)
{
    rt_base_t level;
    rt_err_t r = -RT_EINVAL;
    uint32_t id = 0U, a = 0U, b = 0U, request = 0U;
    level = rt_hw_interrupt_disable();
    if (g_trace_shell.busy) { rt_hw_interrupt_enable(level); return -RT_EBUSY; }
    g_trace_shell.busy = true; rt_hw_interrupt_enable(level);
    if (argc == 2 && !strcmp(argv[1], "status"))
    {
        r = motion_trace_get_status(&g_trace_shell.status);
        if (r == RT_EOK)
        {
            motion_trace_status_t *s = &g_trace_shell.status;
            rt_kprintf("[TRACE] state=%u id=%lu rows=%lu capacity=%lu reason=%u error=%ld\n", (unsigned)s->state,
                U32(s->capture_id), U32(s->rows_stored), U32(s->capacity), (unsigned)s->end_reason, (long)s->error);
            rt_kprintf("[TRACE] scheduled=%lu start=%lu stop=%lu tick_hz=%lu\n", U32(s->scheduled_start_tick),
                U32(s->start_process_tick), U32(s->stop_deadline_tick), U32(RT_TICK_PER_SECOND));
        }
    }
    else if (argc == 4 && !strcmp(argv[1], "arm"))
    {
        r = trace_parse(argv[2], MOTION_TRACE_MAX_SECONDS, &a);
        if (r == RT_EOK) r = trace_parse(argv[3], MOTION_TRACE_MAX_DELAY_SECONDS, &b);
        if (r == RT_EOK) r = motion_service_request_trace_arm(a, b, &request);
    }
    else if (argc == 3)
    {
        r = trace_parse(argv[2], UINT32_MAX, &id);
        if (r == RT_EOK && id == 0U) r = -RT_EINVAL;
        if (r == RT_EOK)
        {
            if (!strcmp(argv[1], "meta")) r = trace_meta(id);
            else if (!strcmp(argv[1], "cancel")) r = motion_service_request_trace_cancel(id, &request);
            else if (!strcmp(argv[1], "clear")) r = motion_service_request_trace_clear(id, &request);
            else if (!strcmp(argv[1], "result"))
            {
                r = motion_service_get_result(id, &g_trace_shell.result);
                if (r == RT_EOK) rt_kprintf("[TRACE][RESULT] id=%lu kind=%u state=%u error=%ld\n", U32(id),
                    (unsigned)g_trace_shell.result.kind, (unsigned)g_trace_shell.result.state, (long)g_trace_shell.result.error);
            }
            else r = -RT_EINVAL;
        }
    }
    else if (argc == 5 && !strcmp(argv[1], "dump"))
    {
        r = trace_parse(argv[2], UINT32_MAX, &id);
        if (r == RT_EOK) r = trace_parse(argv[3], UINT32_MAX, &a);
        if (r == RT_EOK) r = trace_parse(argv[4], MOTION_TRACE_EXPORT_MAX_ROWS, &b);
        if (r == RT_EOK) r = trace_dump(id, a, b);
    }
    if (request != 0U) rt_kprintf("[TRACE][REQUEST] accepted=%lu; query result then status\n", U32(request));
    if (r == -RT_EINVAL)
    {
        rt_kprintf("motion_trace status|arm <1..30s> <0..60s delay>|result <request>\n");
        rt_kprintf("motion_trace cancel|clear|meta <capture>; dump <capture> <first> <1..64>\n");
    }
    if (r != RT_EOK) rt_kprintf("[TRACE][ERROR] code=%d\n", (int)r);
    level = rt_hw_interrupt_disable(); g_trace_shell.busy = false; rt_hw_interrupt_enable(level);
    return (int)r;
}
MSH_CMD_EXPORT(motion_trace, bounded motion RAM trace);
#endif
