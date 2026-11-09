#include <rtthread.h>
#ifdef RT_USING_FINSH
#include <finsh.h>
#include "rthw.h"
#include "motion_service.h"
#include "motion_core_test.h"
#include <stdint.h>
#include <string.h>

#define MOTION_DIAG_WAIT_MS 2000U

typedef struct
{
    bool busy;
    motion_snapshot_t snapshot;
    motion_snapshot_t stopped;
    motion_service_stats_t stats;
    motion_service_stats_t baseline;
    motion_test_status_t test;
    motion_request_result_t result;
    motion_core_test_summary_t summary;
    motion_sample_frame_t queue_frame;
} motion_diag_workspace_t;

/* 大缓冲不放在4096B FinSH栈上；入口串行保护不等同于正式服务锁。 */
static motion_diag_workspace_t g_diag;

static rt_tick_t motion_diag_ticks(uint32_t ms);
static rt_err_t motion_diag_parse_u32(const char *text, uint32_t max, uint32_t *value);
static void motion_diag_print_u64(const char *name, uint64_t value);
static bool motion_diag_sum(uint64_t *sum, uint64_t value);
static bool motion_diag_closed(const motion_service_stats_t *stats);
static void motion_diag_usage(void);
static rt_err_t motion_diag_status(void);
static rt_err_t motion_diag_stats(void);
static rt_err_t motion_diag_result(uint32_t id);
static rt_err_t motion_diag_fault_status(void);
static rt_err_t motion_diag_wait_result(uint32_t id, uint32_t timeout_ms);
static rt_err_t motion_diag_wait_stable(uint32_t timeout_ms);
static void motion_diag_report(const char *name, bool passed, void *user_data);
static rt_err_t motion_diag_selftest(void);
static rt_err_t motion_diag_queue_test(void);
static rt_err_t motion_diag_stop_test(void);
static int motion_diag(int argc, char **argv);

/*
 * 简介：把有限毫秒间隔向上换算成非零tick。
 * 输入：ms为内部限定的小毫秒值。
 * 返回值：至少1tick。
 * 说明：只用于Shell主动测试等待，不修改系统tick。
 */
static rt_tick_t motion_diag_ticks(uint32_t ms)
{
    uint64_t ticks = ((uint64_t)ms * RT_TICK_PER_SECOND + 999U) / 1000U;
    return (rt_tick_t)(ticks == 0U ? 1U : ticks);
}

/*
 * 简介：严格解析无符号十进制参数。
 * 输入：text/value非RT_NULL；max为上限，输入地址只在调用期间使用。
 * 返回值：成功RT_EOK，非法字符、空串、超限或溢出-EINVAL。
 * 说明：不接受符号、空白或部分解析；失败不改输出。
 */
static rt_err_t motion_diag_parse_u32(const char *text, uint32_t max, uint32_t *value)
{
    uint32_t n = 0U, digit;
    const char *p;
    if (text == RT_NULL || value == RT_NULL || *text == '\0') return -RT_EINVAL;
    for (p = text; *p != '\0'; p++)
    {
        if (*p < '0' || *p > '9') return -RT_EINVAL;
        digit = (uint32_t)(*p - '0');
        if (n > max / 10U || (n == max / 10U && digit > max % 10U)) return -RT_EINVAL;
        n = n * 10U + digit;
    }
    *value = n;
    return RT_EOK;
}

/*
 * 简介：完整输出一个64位无符号字段。
 * 输入：name非RT_NULL且调用期间有效，value为值参数。
 * 返回值：无。
 * 说明：最多20位手工转换后以%s输出，不依赖RT_PRINTF_LONGLONG。
 */
static void motion_diag_print_u64(const char *name, uint64_t value)
{
    char text[21];
    uint32_t pos = 20U;
    text[pos] = '\0';
    do
    {
        text[--pos] = (char)('0' + (uint32_t)(value % 10U));
        value /= 10U;
    } while (value != 0U);
    rt_kprintf("[MOTION][U64] %s=%s\n", name, &text[pos]);
}

/*
 * 简介：检查累计求和是否溢出。
 * 输入：sum非RT_NULL且由调用者持有，value为待加值。
 * 返回值：可以安全相加true，否则false且保留原值。
 * 说明：避免诊断闭合式自身回绕而误报PASS。
 */
static bool motion_diag_sum(uint64_t *sum, uint64_t value)
{
    if (value > UINT64_MAX - *sum) return false;
    *sum += value;
    return true;
}

/*
 * 简介：验证已复制的稳定统计是否全部闭合。
 * 输入：s非RT_NULL，只读副本，不保存地址。
 * 返回值：所有闭合式满足true，否则false。
 * 说明：只做有溢出保护的计算，不再次查询服务，避免换成另一个采集窗口。
 */
static bool motion_diag_closed(const motion_service_stats_t *s)
{
    uint64_t n;
    uint32_t a;
    if (!s->accounting_stable || s->counters_saturated) return false;
    n = s->disabled_skipped;
    if (!motion_diag_sum(&n, s->enqueue_ok) || !motion_diag_sum(&n, s->enqueue_reject_full) ||
        !motion_diag_sum(&n, s->enqueue_reject_other) || n != s->produced) return false;
    n = s->dequeued;
    if (!motion_diag_sum(&n, s->outstanding_frames) || n != s->enqueue_ok) return false;
    n = s->discard_old_generation;
    if (!motion_diag_sum(&n, s->discard_stopped) || !motion_diag_sum(&n, s->discard_invalid) ||
        !motion_diag_sum(&n, s->discard_stale) || !motion_diag_sum(&n, s->discard_config) ||
        !motion_diag_sum(&n, s->discard_duplicate) || !motion_diag_sum(&n, s->discard_internal) ||
        !motion_diag_sum(&n, s->discard_injected) || !motion_diag_sum(&n, s->passed_to_alg) || n != s->dequeued) return false;
    for (a = 0U; a < MOTION_ALGORITHM_COUNT; a++)
    {
        n = s->algorithm[a].accepted_no_event;
        if (!motion_diag_sum(&n, s->algorithm[a].events) || !motion_diag_sum(&n, s->algorithm[a].ignored) ||
            !motion_diag_sum(&n, s->algorithm[a].errors) || n != s->algorithm[a].process_calls) return false;
    }
    return true;
}

/*
 * 简介：显示诊断语法与调用影响。
 * 输入：无。
 * 返回值：无。
 * 说明：本命令不自动下载；只有显式stop_test会主动编排正式停启。
 */
static void motion_diag_usage(void)
{
    rt_kprintf("motion_diag status|stats|start|stop|result <id>\n");
    rt_kprintf("motion_diag selftest|queue_test|stop_test\n");
    rt_kprintf("motion_diag fault status|clear\n");
    rt_kprintf("motion_diag fault pause <1..500>|drop <1..16>|final 1\n");
    rt_kprintf("[MOTION] accepted != completed; selftest is isolated; stop_test changes session\n");
}

/*
 * 简介：显示正式服务的已发布摘要。
 * 输入：无。
 * 返回值：实际getter结果。
 * 说明：只读值副本，可信日期和完整性必须与今日计数一起解释。
 */
static rt_err_t motion_diag_status(void)
{
    rt_err_t r = motion_service_get_snapshot(&g_diag.snapshot);
    uint32_t a;
    motion_snapshot_t *s = &g_diag.snapshot;
    if (r != RT_EOK) return r;
    rt_kprintf("[MOTION][STATUS] state=%u selected=%u session=%u run_gen=%u pub=%u\n",
        (unsigned)s->state, (unsigned)s->selected_algorithm, (unsigned)s->session_id,
        (unsigned)s->run_generation, (unsigned)s->publish_seq);
    rt_kprintf("[MOTION][STATUS] quality=0x%x latched=0x%x has=%u seq=%u age_ms=%u error=%d\n",
        (unsigned)s->quality_flags, (unsigned)s->latched_quality_flags, (unsigned)s->has_processed,
        (unsigned)s->last_processed_seq, (unsigned)s->processed_age_ms, (int)s->last_error);
    rt_kprintf("[MOTION][DAY] valid=%u key=%u complete=%u calendar_gen=%u\n",
        (unsigned)s->current_day_valid, (unsigned)s->current_day_key,
        (unsigned)s->current_day_complete, (unsigned)s->calendar_generation);
    for (a = 0U; a < MOTION_ALGORITHM_COUNT; a++)
    {
        rt_kprintf("[MOTION][ALGO] id=%u reboot_resets_counts=1\n", (unsigned)a);
        motion_diag_print_u64("boot", s->totals[a].boot_steps);
        motion_diag_print_u64("session", s->totals[a].session_steps);
        motion_diag_print_u64("unassigned", s->totals[a].unassigned_steps);
        motion_diag_print_u64("archived", s->totals[a].archived_day_steps);
        if (s->current_day_valid) motion_diag_print_u64("day", s->current_day_steps[a]);
        else rt_kprintf("[MOTION][DAY] count=UNASSIGNED\n");
    }
    return RT_EOK;
}

/*
 * 简介：显示累计统计并判断真实一致窗口内的闭合。
 * 输入：无。
 * 返回值：读取成功且未发现闭合错误RT_EOK，否则实际错误。
 * 说明：不稳定或饱和明确SKIP，不能把SKIP计为通过。
 */
static rt_err_t motion_diag_stats(void)
{
    motion_service_stats_t *s = &g_diag.stats;
    rt_err_t r = motion_service_get_stats(s);
    uint64_t n;
    uint32_t a;
    bool ok = true, closed;
    if (r != RT_EOK) return r;
#define MOTION_PRINT_FIELD(field) motion_diag_print_u64(#field, s->field)
    MOTION_PRINT_FIELD(produced); MOTION_PRINT_FIELD(disabled_skipped);
    MOTION_PRINT_FIELD(enqueue_ok); MOTION_PRINT_FIELD(enqueue_reject_full); MOTION_PRINT_FIELD(enqueue_reject_other);
    MOTION_PRINT_FIELD(dequeued); MOTION_PRINT_FIELD(discard_old_generation); MOTION_PRINT_FIELD(discard_stopped);
    MOTION_PRINT_FIELD(discard_invalid); MOTION_PRINT_FIELD(discard_stale); MOTION_PRINT_FIELD(discard_config);
    MOTION_PRINT_FIELD(discard_duplicate); MOTION_PRINT_FIELD(discard_internal); MOTION_PRINT_FIELD(discard_injected);
    MOTION_PRINT_FIELD(passed_to_alg); MOTION_PRINT_FIELD(seq_gap_events); MOTION_PRINT_FIELD(seq_missing_samples);
    MOTION_PRINT_FIELD(time_gap_events); MOTION_PRINT_FIELD(publish_failures); MOTION_PRINT_FIELD(wake_failures);
    MOTION_PRINT_FIELD(sensor_status_read_failures);
#undef MOTION_PRINT_FIELD
    rt_kprintf("[MOTION][STATS] stable=%u saturated=%u submit=%u receive=%u outstanding=%u high=%u\n",
        (unsigned)s->accounting_stable, (unsigned)s->counters_saturated, (unsigned)s->submit_inflight,
        (unsigned)s->receive_inflight, (unsigned)s->outstanding_frames, (unsigned)s->queue_high_water);
    rt_kprintf("[MOTION][MAX_TICKS] submit=%u queue=%u sample_age=%u core=%u hz=%u\n",
        (unsigned)s->max_submit_ticks, (unsigned)s->max_queue_wait_ticks, (unsigned)s->max_sample_age_ticks,
        (unsigned)s->max_algorithm_ticks, (unsigned)RT_TICK_PER_SECOND);
    rt_kprintf("[MOTION][PROGRESS] worker=%u effective=%u\n", (unsigned)s->worker_progress_tick, (unsigned)s->effective_progress_tick);
    for (a = 0U; a < MOTION_ALGORITHM_COUNT; a++)
    {
        rt_kprintf("[MOTION][ALGO_STATS] id=%u\n", (unsigned)a);
        motion_diag_print_u64("calls", s->algorithm[a].process_calls);
        motion_diag_print_u64("accepted_no_event", s->algorithm[a].accepted_no_event);
        motion_diag_print_u64("events", s->algorithm[a].events);
        motion_diag_print_u64("ignored", s->algorithm[a].ignored);
        motion_diag_print_u64("errors", s->algorithm[a].errors);
        motion_diag_print_u64("resets", s->algorithm[a].reset_events);
    }
    if (!s->accounting_stable || s->counters_saturated)
    {
        rt_kprintf("[MOTION][CLOSURE][SKIP] inconsistent window or saturated; retry later\n");
        return RT_EOK;
    }
    n = s->disabled_skipped;
    closed = motion_diag_sum(&n, s->enqueue_ok) && motion_diag_sum(&n, s->enqueue_reject_full) &&
             motion_diag_sum(&n, s->enqueue_reject_other) && n == s->produced;
    ok = ok && closed;
    rt_kprintf("[MOTION][CLOSURE] producer=%s\n", closed ? "PASS" : "FAIL");
    n = s->dequeued;
    closed = motion_diag_sum(&n, s->outstanding_frames) && n == s->enqueue_ok;
    ok = ok && closed;
    rt_kprintf("[MOTION][CLOSURE] channel=%s\n", closed ? "PASS" : "FAIL");
    n = s->discard_old_generation;
    closed = motion_diag_sum(&n, s->discard_stopped) && motion_diag_sum(&n, s->discard_invalid) &&
        motion_diag_sum(&n, s->discard_stale) && motion_diag_sum(&n, s->discard_config) &&
        motion_diag_sum(&n, s->discard_duplicate) && motion_diag_sum(&n, s->discard_internal) &&
        motion_diag_sum(&n, s->discard_injected) && motion_diag_sum(&n, s->passed_to_alg) && n == s->dequeued;
    ok = ok && closed;
    rt_kprintf("[MOTION][CLOSURE] consumer=%s\n", closed ? "PASS" : "FAIL");
    for (a = 0U; a < MOTION_ALGORITHM_COUNT; a++)
    {
        n = s->algorithm[a].accepted_no_event;
        closed = motion_diag_sum(&n, s->algorithm[a].events) && motion_diag_sum(&n, s->algorithm[a].ignored) &&
            motion_diag_sum(&n, s->algorithm[a].errors) && n == s->algorithm[a].process_calls;
        ok = ok && closed;
        rt_kprintf("[MOTION][CLOSURE] algo=%u %s\n", (unsigned)a, closed ? "PASS" : "FAIL");
    }
    return ok ? RT_EOK : -RT_ERROR;
}

/*
 * 简介：显示一条异步请求记录。
 * 输入：id非0。
 * 返回值：实际查询结果。
 * 说明：PENDING/RUNNING不代表完成，记录不存在也不能推断成功。
 */
static rt_err_t motion_diag_result(uint32_t id)
{
    rt_err_t r = motion_service_get_result(id, &g_diag.result);
    if (r == RT_EOK)
        rt_kprintf("[MOTION][RESULT] id=%u kind=%u state=%u error=%d session=%u gen=%u tick=%u\n",
            (unsigned)g_diag.result.request_id, (unsigned)g_diag.result.kind, (unsigned)g_diag.result.state,
            (int)g_diag.result.error, (unsigned)g_diag.result.session_id,
            (unsigned)g_diag.result.run_generation, (unsigned)g_diag.result.finished_tick);
    return r;
}

/*
 * 简介：显示注入状态和真实窗口观测。
 * 输入：无。
 * 返回值：实际getter结果，已完成FINAL验证失败时-RT_ERROR。
 * 说明：过期、未触发或witness无效均不判PASS；显示发布年龄避免看旧记录。
 */
static rt_err_t motion_diag_fault_status(void)
{
    motion_test_status_t *s = &g_diag.test;
    rt_err_t r = motion_service_get_test_status(s);
    uint32_t age;
    bool passed;
    if (r != RT_EOK) return r;
    age = (uint32_t)(rt_tick_get() - s->published_tick);
    rt_kprintf("[MOTION][TEST] id=%u kind=%u active=%u end=%u arg=%u remaining=%u pub_age_ticks=%u\n",
        (unsigned)s->request_id, (unsigned)s->kind, (unsigned)s->active, (unsigned)s->end_reason,
        (unsigned)s->argument, (unsigned)s->remaining, (unsigned)age);
    motion_diag_print_u64("pause_activations", s->pause_activations);
    motion_diag_print_u64("dropped_frames", s->dropped_frames);
    motion_diag_print_u64("final_publish_failures", s->final_publish_failures);
    motion_diag_print_u64("expirations", s->expirations);
    motion_diag_print_u64("cancellations", s->cancellations);
    rt_kprintf("[MOTION][WITNESS] valid=%u read_error=%d same_revision=%u stable=%u\n",
        (unsigned)s->witness_valid, (int)s->witness_read_error,
        (unsigned)s->witness_same_revision, (unsigned)s->witness_accounting_stable);
    rt_kprintf("[MOTION][WITNESS] other_conditions=%u\n", (unsigned)s->witness_other_conditions);
    motion_diag_print_u64("witness_dequeued_delta", s->witness_dequeued_delta);
    if (age > motion_diag_ticks(250U))
    {
        rt_kprintf("[MOTION][TEST][SKIP] stale publication\n");
        return -RT_EBUSY;
    }
    if (s->kind == MOTION_TEST_FINAL_PUBLISH && !s->active && s->end_reason == MOTION_TEST_END_COMPLETED)
    {
        passed = s->witness_valid && s->witness_read_error == 0 && s->witness_other_conditions && s->witness_same_revision &&
                 s->witness_dequeued_delta != 0U && !s->witness_accounting_stable;
        rt_kprintf("[MOTION][FINAL_WINDOW] %s\n", passed ? "PASS" : "FAIL");
        return passed ? RT_EOK : -RT_ERROR;
    }
    return RT_EOK;
}

/*
 * 简介：有界等待控制请求终态。
 * 输入：id非0，timeout_ms为1～2000。
 * 返回值：成功终态RT_EOK，失败为终态错误，观察超时-ETIMEOUT。
 * 说明：每次轮询阻塞10ms，超时不等于请求已撤销，不自动重复提交。
 */
static rt_err_t motion_diag_wait_result(uint32_t id, uint32_t timeout_ms)
{
    rt_tick_t start = rt_tick_get();
    rt_err_t r;
    if (id == 0U || timeout_ms == 0U || timeout_ms > MOTION_DIAG_WAIT_MS) return -RT_EINVAL;
    do
    {
        r = motion_service_get_result(id, &g_diag.result);
        if (r != RT_EOK) return r;
        if (g_diag.result.state == MOTION_RESULT_OK) return RT_EOK;
        if (g_diag.result.state == MOTION_RESULT_FAILED)
            return g_diag.result.error < 0 ? (rt_err_t)g_diag.result.error : -RT_ERROR;
        if ((rt_tick_t)(rt_tick_get() - start) >= motion_diag_ticks(timeout_ms)) break;
        rt_thread_delay(motion_diag_ticks(10U));
    } while (1);
    return -RT_ETIMEOUT;
}

/*
 * 简介：有界等待可用于闭合判断的统计副本。
 * 输入：timeout_ms为1～2000。
 * 返回值：取得稳定未饱和副本RT_EOK，否则实际错误或-ETIMEOUT。
 * 说明：不暂停生产，仅重试getter，结果存放Shell静态工作区。
 */
static rt_err_t motion_diag_wait_stable(uint32_t timeout_ms)
{
    rt_tick_t start = rt_tick_get();
    rt_err_t r;
    if (timeout_ms == 0U || timeout_ms > MOTION_DIAG_WAIT_MS) return -RT_EINVAL;
    do
    {
        r = motion_service_get_stats(&g_diag.stats);
        if (r == RT_EOK && g_diag.stats.accounting_stable && !g_diag.stats.counters_saturated) return RT_EOK;
        if (r != RT_EOK && r != -RT_EBUSY) return r;
        if ((rt_tick_t)(rt_tick_get() - start) >= motion_diag_ticks(timeout_ms)) break;
        rt_thread_delay(motion_diag_ticks(10U));
    } while (1);
    return -RT_ETIMEOUT;
}

/*
 * 简介：输出独立核心实际断言。
 * 输入：name为调用期间有效字符串，user_data本方案不用。
 * 返回值：无。
 * 说明：不重入测试或访问正式服务。
 */
static void motion_diag_report(const char *name, bool passed, void *user_data)
{
    (void)user_data;
    rt_kprintf("[MOTION][CORETEST][%s] %s\n", passed ? "PASS" : "FAIL", name);
}

/*
 * 简介：运行独立核心测试并打印汇总。
 * 输入：无。
 * 返回值：全通过RT_EOK，否则-RT_ERROR。
 * 说明：不暂停正式业务，不把无效/跳过的测试算通过。
 */
static rt_err_t motion_diag_selftest(void)
{
    int r = motion_core_test_run(&g_diag.summary, motion_diag_report, RT_NULL);
    rt_kprintf("[MOTION][CORETEST][SUMMARY] groups=%u checks=%u pass=%u fail=%u setup_errors=%u result=%d\n",
        (unsigned)g_diag.summary.groups_run, (unsigned)g_diag.summary.checks,
        (unsigned)g_diag.summary.passed, (unsigned)g_diag.summary.failed,
        (unsigned)g_diag.summary.setup_errors, r);
    return r == 0 ? RT_EOK : -RT_ERROR;
}

/*
 * 简介：在私有队列验证容量、拒绝新帧和FIFO。
 * 输入：无。
 * 返回值：全部满足RT_EOK，否则实际错误。
 * 说明：只创建/删除mot_tq，不操作正式队列；所有失败统一清理。
 */
static rt_err_t motion_diag_queue_test(void)
{
    rt_mq_t queue;
    rt_err_t r = RT_EOK;
    rt_err_t cleanup;
    uint32_t i, accepted = 0U, rejected = 0U, received = 0U;
    queue = rt_mq_create("mot_tq", sizeof(motion_sample_frame_t), MOTION_SAMPLE_QUEUE_CAPACITY, RT_IPC_FLAG_FIFO);
    if (queue == RT_NULL) return -RT_ENOMEM;
    memset(&g_diag.queue_frame, 0, sizeof(g_diag.queue_frame));
    for (i = 0U; i < MOTION_SAMPLE_QUEUE_CAPACITY; i++)
    {
        g_diag.queue_frame.imu.imu_seq = i;
        r = rt_mq_send(queue, &g_diag.queue_frame, sizeof(g_diag.queue_frame));
        if (r != RT_EOK) goto cleanup;
        accepted++;
    }
    g_diag.queue_frame.imu.imu_seq = UINT32_MAX;
    r = rt_mq_send(queue, &g_diag.queue_frame, sizeof(g_diag.queue_frame));
    if (r != -RT_EFULL) { r = -RT_ERROR; goto cleanup; }
    rejected++;
    for (i = 0U; i < MOTION_SAMPLE_QUEUE_CAPACITY; i++)
    {
        r = rt_mq_recv(queue, &g_diag.queue_frame, sizeof(g_diag.queue_frame), 0);
        if (r != RT_EOK) goto cleanup;
        if (g_diag.queue_frame.imu.imu_seq != i) { r = -RT_ERROR; goto cleanup; }
        received++;
    }
    r = rt_mq_recv(queue, &g_diag.queue_frame, sizeof(g_diag.queue_frame), 0);
    r = (r == -RT_ETIMEOUT && accepted == received && rejected == 1U) ? RT_EOK : -RT_ERROR;
cleanup:
    cleanup = rt_mq_delete(queue);
    if (cleanup != RT_EOK) r = cleanup;
    rt_kprintf("[MOTION][QUEUE_TEST] accepted=%u rejected=%u received=%u %s\n",
        (unsigned)accepted, (unsigned)rejected, (unsigned)received, r == RT_EOK ? "PASS" : "FAIL");
    return r;
}

/*
 * 简介：自动建立积压并验证STOP排空和新会话恢复。
 * 输入：无，调用前服务须RUNNING且没有活动注入。
 * 返回值：全部完成RT_EOK；未覆盖或失败为负值并保留现场。
 * 说明：会改变正式会话，启动累计不清；失败只撤销仍活动注入，
 *       不自动清FAULT，不在超时后重复提交STOP或START。
 */
static rt_err_t motion_diag_stop_test(void)
{
    uint32_t id, fault_id = 0U, a;
    rt_err_t r, cleanup;
    rt_tick_t start;
    bool covered = false;
    r = motion_service_get_snapshot(&g_diag.snapshot);
    if (r != RT_EOK) return r;
    if (g_diag.snapshot.state != MOTION_SERVICE_RUNNING) return -RT_EBUSY;
    r = motion_service_get_test_status(&g_diag.test);
    if (r != RT_EOK || g_diag.test.active) return r != RT_EOK ? r : -RT_EBUSY;
    r = motion_diag_wait_stable(1000U);
    if (r != RT_EOK) return r;
    g_diag.baseline = g_diag.stats;
    r = motion_service_request_test(MOTION_TEST_PAUSE_CONSUMER, 500U, &fault_id);
    if (r != RT_EOK) return r;
    r = motion_diag_wait_result(fault_id, MOTION_DIAG_WAIT_MS);
    if (r != RT_EOK) goto done;
    start = rt_tick_get();
    while ((rt_tick_t)(rt_tick_get() - start) < motion_diag_ticks(500U))
    {
        r = motion_service_get_test_status(&g_diag.test);
        if (r != RT_EOK) goto done;
        /* 配置完成记录可能先于本轮状态发布，旧ID不能当成本次已结束。 */
        if (g_diag.test.request_id != fault_id)
        {
            rt_thread_delay(motion_diag_ticks(10U));
            continue;
        }
        if (!g_diag.test.active) break;
        r = motion_service_get_stats(&g_diag.stats);
        if (r == RT_EOK && g_diag.stats.enqueue_reject_full > g_diag.baseline.enqueue_reject_full)
        { covered = true; break; }
        if (r != RT_EOK && r != -RT_EBUSY) goto done;
        rt_thread_delay(motion_diag_ticks(10U));
    }
    if (!covered)
    {
        rt_kprintf("[MOTION][STOP_TEST][NOT_COVERED] no full queue during this pause\n");
        r = -RT_ETIMEOUT;
        goto done;
    }
    r = motion_service_request_stop(&id);
    if (r != RT_EOK) goto done;
    r = motion_diag_wait_result(id, MOTION_DIAG_WAIT_MS);
    if (r != RT_EOK) goto done;
    r = motion_service_get_test_status(&g_diag.test);
    if (r != RT_EOK) goto done;
    if (g_diag.test.request_id != fault_id || g_diag.test.active ||
        g_diag.test.end_reason != MOTION_TEST_END_CANCELLED)
    {
        covered = false;
        rt_kprintf("[MOTION][STOP_TEST][NOT_COVERED] pause ended before STOP cancelled it\n");
        r = -RT_ETIMEOUT;
        goto done;
    }
    r = motion_service_get_snapshot(&g_diag.stopped);
    if (r != RT_EOK) goto done;
    if (g_diag.stopped.state != MOTION_SERVICE_STOPPED) { r = -RT_ERROR; goto done; }
    r = motion_diag_wait_stable(1000U);
    if (r != RT_EOK) goto done;
    if (!motion_diag_closed(&g_diag.stats) || g_diag.stats.submit_inflight != 0U || g_diag.stats.receive_inflight != 0U ||
        g_diag.stats.outstanding_frames != 0U || g_diag.stats.enqueue_ok != g_diag.stats.dequeued)
    { r = -RT_ERROR; goto done; }
    g_diag.baseline = g_diag.stats;
    /* 确认停止后没有继续追加事件，同时真实sensor回调仍继续到达。 */
    rt_thread_delay(motion_diag_ticks(200U));
    r = motion_service_get_snapshot(&g_diag.snapshot);
    if (r != RT_EOK) goto done;
    if (g_diag.snapshot.state != MOTION_SERVICE_STOPPED ||
        g_diag.snapshot.session_id != g_diag.stopped.session_id ||
        g_diag.snapshot.run_generation != g_diag.stopped.run_generation)
    { r = -RT_ERROR; goto done; }
    r = motion_diag_wait_stable(1000U);
    if (r != RT_EOK) goto done;
    if (!motion_diag_closed(&g_diag.stats) ||
        g_diag.stats.disabled_skipped <= g_diag.baseline.disabled_skipped ||
        g_diag.stats.enqueue_ok != g_diag.baseline.enqueue_ok ||
        g_diag.stats.dequeued != g_diag.baseline.dequeued)
    { r = -RT_ERROR; goto done; }
    for (a = 0U; a < 2U; a++)
        if (g_diag.snapshot.totals[a].boot_steps != g_diag.stopped.totals[a].boot_steps ||
            g_diag.snapshot.totals[a].session_steps != g_diag.stopped.totals[a].session_steps)
        { r = -RT_ERROR; goto done; }
    r = motion_service_request_start(&id);
    if (r != RT_EOK) goto done;
    r = motion_diag_wait_result(id, MOTION_DIAG_WAIT_MS);
    if (r != RT_EOK) goto done;
    r = motion_service_get_snapshot(&g_diag.snapshot);
    if (r != RT_EOK) goto done;
    if (g_diag.snapshot.state != MOTION_SERVICE_RUNNING ||
        g_diag.snapshot.session_id == g_diag.stopped.session_id ||
        g_diag.snapshot.run_generation == g_diag.stopped.run_generation)
    { r = -RT_ERROR; goto done; }
    for (a = 0U; a < 2U; a++)
        if (g_diag.snapshot.totals[a].boot_steps < g_diag.stopped.totals[a].boot_steps)
        { r = -RT_ERROR; goto done; }
    /* RUNNING本身不足以证明接纳门恢复，等待新会话真实输入完成预热。 */
    start = rt_tick_get();
    for (;;)
    {
        r = motion_service_get_snapshot(&g_diag.snapshot);
        if (r != RT_EOK && r != -RT_EBUSY) goto done;
        if (r == RT_EOK && g_diag.snapshot.state == MOTION_SERVICE_RUNNING &&
            g_diag.snapshot.has_processed && g_diag.snapshot.processed_age_ms <= MOTION_INPUT_STALE_MS &&
            (g_diag.snapshot.quality_flags & (MOTION_QUALITY_WARMUP | MOTION_QUALITY_INPUT_STALE |
             MOTION_QUALITY_SOURCE_OFFLINE | MOTION_QUALITY_CONFIG_MISMATCH)) == 0U)
        {
            r = motion_service_get_stats(&g_diag.stats);
            if (r != RT_EOK && r != -RT_EBUSY) goto done;
            if (r == RT_EOK && motion_diag_closed(&g_diag.stats) &&
                g_diag.stats.passed_to_alg > g_diag.baseline.passed_to_alg)
            {
                rt_kprintf("[MOTION][STOP_TEST] recovery_stream=PASS\n");
                break;
            }
        }
        if ((rt_tick_t)(rt_tick_get() - start) >= motion_diag_ticks(1000U))
        { r = -RT_ETIMEOUT; goto done; }
        rt_thread_delay(motion_diag_ticks(10U));
    }
    r = RT_EOK;
done:
    /* 仅撤销本次仍活动的注入；失败不自动反复开关正式会话。 */
    cleanup = motion_service_get_test_status(&g_diag.test);
    if (cleanup == RT_EOK && g_diag.test.active && g_diag.test.request_id == fault_id)
    {
        cleanup = motion_service_request_test(MOTION_TEST_NONE, 0U, &id);
        if (cleanup == RT_EOK) cleanup = motion_diag_wait_result(id, MOTION_DIAG_WAIT_MS);
        if (cleanup != RT_EOK) rt_kprintf("[MOTION][STOP_TEST] cleanup_error=%d\n", (int)cleanup);
        if (r == RT_EOK && cleanup != RT_EOK) r = cleanup;
    }
    rt_kprintf("[MOTION][STOP_TEST] covered=%u result=%d %s\n", (unsigned)covered, (int)r, r == RT_EOK ? "PASS" : "FAIL_OR_NOT_COVERED");
    return r;
}

/*
 * 简介：严格分派诊断命令并串行保护静态工作区。
 * 输入：argc/argv为FinSH提供的参数，仅本次调用期间有效。
 * 返回值：0成功或负错误码；异步受理不表示完成。
 * 说明：所有退出路径释放busy；不在解析过程中访问活的core。
 */
static int motion_diag(int argc, char **argv)
{
    rt_base_t level;
    rt_err_t r = -RT_EINVAL;
    uint32_t value = 0U, id = 0U, max = 0U;
    motion_test_fault_t fault = MOTION_TEST_NONE;
    bool accepted = false;
    level = rt_hw_interrupt_disable();
    if (g_diag.busy) { rt_hw_interrupt_enable(level); return -RT_EBUSY; }
    g_diag.busy = true;
    rt_hw_interrupt_enable(level);
    if (argc == 2)
    {
        if (!strcmp(argv[1], "status")) r = motion_diag_status();
        else if (!strcmp(argv[1], "stats")) r = motion_diag_stats();
        else if (!strcmp(argv[1], "selftest")) r = motion_diag_selftest();
        else if (!strcmp(argv[1], "queue_test")) r = motion_diag_queue_test();
        else if (!strcmp(argv[1], "stop_test")) r = motion_diag_stop_test();
        else if (!strcmp(argv[1], "start")) { r = motion_service_request_start(&id); accepted = r == RT_EOK; }
        else if (!strcmp(argv[1], "stop")) { r = motion_service_request_stop(&id); accepted = r == RT_EOK; }
    }
    else if (argc == 3 && !strcmp(argv[1], "result"))
    {
        r = motion_diag_parse_u32(argv[2], UINT32_MAX, &id);
        if (r == RT_EOK) r = id != 0U ? motion_diag_result(id) : -RT_EINVAL;
    }
    else if (argc == 3 && !strcmp(argv[1], "fault"))
    {
        if (!strcmp(argv[2], "status")) r = motion_diag_fault_status();
        else if (!strcmp(argv[2], "clear"))
        { r = motion_service_request_test(MOTION_TEST_NONE, 0U, &id); accepted = r == RT_EOK; }
    }
    else if (argc == 4 && !strcmp(argv[1], "fault"))
    {
        if (!strcmp(argv[2], "pause")) { fault = MOTION_TEST_PAUSE_CONSUMER; max = MOTION_TEST_MAX_PAUSE_MS; }
        else if (!strcmp(argv[2], "drop")) { fault = MOTION_TEST_DROP_FRAMES; max = MOTION_TEST_MAX_DROP_FRAMES; }
        else if (!strcmp(argv[2], "final")) { fault = MOTION_TEST_FINAL_PUBLISH; max = 1U; }
        if (fault != MOTION_TEST_NONE)
        {
            r = motion_diag_parse_u32(argv[3], max, &value);
            if (r == RT_EOK) r = motion_service_request_test(fault, value, &id);
            accepted = r == RT_EOK;
        }
    }
    if (accepted) rt_kprintf("[MOTION][REQUEST] accepted id=%u; query result, then fault status if applicable\n", (unsigned)id);
    if (r == -RT_EINVAL) motion_diag_usage();
    if (r != RT_EOK) rt_kprintf("[MOTION][ERROR] code=%d\n", (int)r);
    level = rt_hw_interrupt_disable();
    g_diag.busy = false;
    rt_hw_interrupt_enable(level);
    return (int)r;
}

MSH_CMD_EXPORT(motion_diag, bounded motion diagnostics and isolated tests);
#endif /* RT_USING_FINSH */
