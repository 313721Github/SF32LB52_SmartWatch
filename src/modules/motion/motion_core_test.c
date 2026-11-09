#include "motion_core_test.h"
#include "motion_core.h"
#include "step_detector_golden_vectors.h"
#include <limits.h>
#include <stddef.h>
#include <string.h>

#define MOTION_TEST_GROUPS 9U

typedef struct
{
    motion_core_t core;
    motion_sample_frame_t frame;
    motion_sample_frame_t saved[3];
    motion_core_frame_result_t result;
    motion_snapshot_t snapshot;
    motion_core_stats_t stats;
    motion_calendar_tag_t calendar;
    uint32_t now;
    uint32_t next_seq;
    uint32_t generation;
    motion_core_test_summary_t summary;
    motion_core_test_report_t report;
    void *user_data;
} motion_test_workspace_t;

/* 只属于串行测试，不与正式worker共享地址。 */
static motion_test_workspace_t g_test;

int motion_core_test_run(motion_core_test_summary_t *summary,
                         motion_core_test_report_t report, void *user_data);
static void motion_test_expect(const char *name, bool condition);
static bool motion_test_begin(uint32_t now, uint32_t first_seq);
static void motion_test_make_frame(motion_sample_frame_t *frame, uint32_t tick,
                                   uint32_t seq, int32_t az, const motion_calendar_tag_t *tag);
static bool motion_test_process(void);
static bool motion_test_feed(int32_t az, const motion_calendar_tag_t *tag);
static bool motion_test_low(uint32_t count);
static bool motion_test_pulse(const motion_calendar_tag_t *tag);
static bool motion_test_calendar(bool valid, uint32_t key, uint32_t generation);
static void motion_test_accounting(void);
static void motion_test_lifecycle(void);
static void motion_test_sequence(void);
static void motion_test_guards(void);
static void motion_test_clock(void);
static void motion_test_source(void);
static void motion_test_days(void);
static void motion_test_epochs(void);
static void motion_test_history(void);
static void motion_test_golden(void);

/*
 * 简介：登记实际断言并调用可选报告回调。
 * 输入：name为调用期间有效字符串；condition为实际判断。
 * 返回值：无。
 * 说明：不打印、不重入正式服务；失败仍保留整轮统计。
 */
static void motion_test_expect(const char *name, bool condition)
{
    g_test.summary.checks++;
    if (condition) g_test.summary.passed++;
    else g_test.summary.failed++;
    if (g_test.report != NULL) g_test.report(name, condition, g_test.user_data);
}

/*
 * 简介：建立一组虚拟时间、独立核心和正常数据源。
 * 输入：now为虚拟起点，first_seq为首样本序号。
 * 返回值：全部准备成功true，否则false。
 * 说明：不清整轮汇总；任何准备错误单独登记，不能把跳过判为通过。
 */
static bool motion_test_begin(uint32_t now, uint32_t first_seq)
{
    int result;
    g_test.now = now;
    g_test.next_seq = first_seq;
    g_test.generation = 1U;
    memset(&g_test.calendar, 0, sizeof(g_test.calendar));
    result = motion_core_init(&g_test.core, 1000U, MOTION_ALGORITHM_B1, now);
    if (result == 0) result = motion_core_update_source(&g_test.core, MOTION_SOURCE_AVAILABLE, now);
    if (result == 0) result = motion_core_update_calendar(&g_test.core, &g_test.calendar, now);
    if (result == 0) result = motion_core_start(&g_test.core, 1U, 1U, now);
    if (result != 0)
    {
        g_test.summary.setup_errors++;
        motion_test_expect("setup succeeded", false);
        return false;
    }
    return true;
}

/*
 * 简介：创建明确标为软件构造值的输入帧。
 * 输入：frame/tag非NULL，其他为值参数；函数只复制，不保存指针。
 * 返回值：无。
 * 说明：raw字段为0，converted字段用于核心测试，不代表实采寄存器数据。
 */
static void motion_test_make_frame(motion_sample_frame_t *frame, uint32_t tick,
                                   uint32_t seq, int32_t az, const motion_calendar_tag_t *tag)
{
    memset(frame, 0, sizeof(*frame));
    frame->imu.sample_tick = tick;
    frame->imu.imu_seq = seq;
    frame->imu.config_version = STEP_DETECTOR_TASK10_CONFIG_VERSION;
    frame->imu.accel_ug[2] = az;
    frame->imu.valid = true;
    frame->calendar = *tag;
    frame->enqueue_tick = tick;
    frame->run_generation = g_test.generation;
}

/*
 * 简介：执行一次预期可分类的独立帧处理。
 * 输入：无，使用测试工作区。
 * 返回值：调用成功true；调用错误false并登记失败。
 * 说明：只调用独立core，调用失败不能继续把旧result当新结果。
 */
static bool motion_test_process(void)
{
    int result = motion_core_process(&g_test.core, &g_test.frame, g_test.now, &g_test.result);
    if (result != 0)
    {
        g_test.summary.setup_errors++;
        motion_test_expect("frame call completed", false);
        return false;
    }
    return true;
}

/*
 * 简介：按20ms周期推进虚拟输入。
 * 输入：az为微g构造值；tag非NULL，仅复制。
 * 返回值：核心调用成功true，否则false。
 * 说明：不延时、不读系统tick，不投递正式消息队列。
 */
static bool motion_test_feed(int32_t az, const motion_calendar_tag_t *tag)
{
    g_test.now += 20U;
    motion_test_make_frame(&g_test.frame, g_test.now, g_test.next_seq++, az, tag);
    return motion_test_process();
}

/*
 * 简介：提供有限数量1g低动态样本。
 * 输入：count为内部固定有限循环次数。
 * 返回值：全部输入正常true，否则false。
 * 说明：预热来自实际样本时间覆盖，不靠墙钟等待。
 */
static bool motion_test_low(uint32_t count)
{
    uint32_t i;
    for (i = 0U; i < count; i++)
        if (!motion_test_feed(1000000, &g_test.calendar)) return false;
    return true;
}

/*
 * 简介：生成300ms低动态、400ms高动态及400ms低动态波形。
 * 输入：tag非NULL，整段复制该日历标签。
 * 返回值：全部输入成功true，否则false。
 * 说明：正常冻结配置每段应各产生一个B0/B1事件；是否计入业务由用例断言。
 */
static bool motion_test_pulse(const motion_calendar_tag_t *tag)
{
    uint32_t i;
    for (i = 0U; i < 15U; i++) if (!motion_test_feed(1000000, tag)) return false;
    for (i = 0U; i < 20U; i++) if (!motion_test_feed(1800000, tag)) return false;
    for (i = 0U; i < 20U; i++) if (!motion_test_feed(1000000, tag)) return false;
    return true;
}

/*
 * 简介：更新测试实例的当前日期。
 * 输入：valid/key/generation为值参数。
 * 返回值：更新成功true，否则false并登记准备错误。
 * 说明：不调用真实时间服务，也不改变既有帧标签。
 */
static bool motion_test_calendar(bool valid, uint32_t key, uint32_t generation)
{
    int result;
    g_test.calendar.day_valid = valid;
    g_test.calendar.local_day_key = valid ? key : 0U;
    g_test.calendar.calendar_generation = generation;
    g_test.calendar.day_seq++;
    result = motion_core_update_calendar(&g_test.core, &g_test.calendar, g_test.now);
    if (result != 0)
    {
        g_test.summary.setup_errors++;
        motion_test_expect("calendar setup succeeded", false);
    }
    return result == 0;
}

/*
 * 简介：验证核心的业务与消费累计闭合。
 * 输入：无，只从独立核心复制结果。
 * 返回值：无。
 * 说明：加法逐项检查上限，禁止靠溢出求和误判通过。
 */
static void motion_test_accounting(void)
{
    uint32_t a, i;
    uint64_t sum, term;
    bool safe;
    motion_test_expect("snapshot copy", motion_core_get_snapshot(&g_test.core, g_test.now, &g_test.snapshot) == 0);
    motion_test_expect("stats copy", motion_core_get_stats(&g_test.core, &g_test.stats) == 0);
    for (a = 0U; a < MOTION_ALGORITHM_COUNT; a++)
    {
        sum = g_test.snapshot.totals[a].unassigned_steps;
        term = g_test.snapshot.totals[a].archived_day_steps;
        safe = term <= UINT64_MAX - sum;
        if (safe) sum += term;
        for (i = 0U; i < MOTION_DAY_HISTORY_CAPACITY; i++)
        {
            if (!g_test.snapshot.day_history[i].used) continue;
            term = g_test.snapshot.day_history[i].steps[a];
            if (term > UINT64_MAX - sum) safe = false;
            else if (safe) sum += term;
        }
        motion_test_expect("boot equals unassigned + archive + days", safe && sum == g_test.snapshot.totals[a].boot_steps);
        sum = g_test.stats.algorithm[a].accepted_no_event + g_test.stats.algorithm[a].events +
              g_test.stats.algorithm[a].ignored + g_test.stats.algorithm[a].errors;
        motion_test_expect("algorithm call classification", sum == g_test.stats.algorithm[a].process_calls);
    }
    sum = g_test.stats.discard_old_generation + g_test.stats.discard_stopped +
          g_test.stats.discard_invalid + g_test.stats.discard_stale + g_test.stats.discard_config +
          g_test.stats.discard_duplicate + g_test.stats.passed_to_alg;
    motion_test_expect("core dequeue classification", sum == g_test.stats.dequeued);
}

/*
 * 简介：验证普通生命周期与业务累计分离。
 * 输入：无，使用独立实例。
 * 返回值：无。
 * 说明：重复START不得抹掉会话，正常停启只清新会话累计。
 */
static void motion_test_lifecycle(void)
{
    uint32_t a;
    if (!motion_test_begin(1000U, 1U) || !motion_test_pulse(&g_test.calendar)) return;
    for (a = 0U; a < 2U; a++) motion_test_expect("constructed pulse is one event", g_test.core.snapshot.totals[a].boot_steps == 1U);
    motion_test_expect("repeated start rejected", motion_core_start(&g_test.core, 2U, 2U, g_test.now) == MOTION_CORE_ERR_STATE);
    motion_test_expect("stop accepted", motion_core_stop(&g_test.core, g_test.now) == 0);
    motion_test_expect("repeated stop accepted", motion_core_stop(&g_test.core, g_test.now) == 0);
    motion_test_expect("start new session", motion_core_start(&g_test.core, 2U, 2U, g_test.now) == 0);
    for (a = 0U; a < 2U; a++)
    {
        motion_test_expect("boot preserved across restart", g_test.core.snapshot.totals[a].boot_steps == 1U);
        motion_test_expect("new session starts zero", g_test.core.snapshot.totals[a].session_steps == 0U);
    }
    motion_test_accounting();
}

/*
 * 简介：验证重复、缺样、倒退及序号回绕。
 * 输入：无，使用独立实例。
 * 返回值：无。
 * 说明：每份帧唯一分类；缺口后不补造步数。
 */
static void motion_test_sequence(void)
{
    if (!motion_test_begin(1000U, 1U) || !motion_test_low(15U)) return;
    if (!motion_test_process()) return;
    motion_test_expect("duplicate discarded", g_test.result.disposition == MOTION_FRAME_DISCARD_DUPLICATE);
    g_test.next_seq += 2U;
    if (!motion_test_feed(1000000, &g_test.calendar)) return;
    motion_test_expect("two missing frames counted", g_test.core.stats.seq_missing_samples == 2U);
    motion_test_expect("gap rebuild recorded", (g_test.result.reset_reasons & MOTION_RESET_SEQUENCE_GAP) != 0U);
    g_test.next_seq = g_test.frame.imu.imu_seq - 10U;
    if (!motion_test_feed(1000000, &g_test.calendar)) return;
    motion_test_expect("sequence rollback recorded", g_test.core.stats.seq_rollback_events == 1U);
    motion_test_accounting();
    if (!motion_test_begin(1000U, UINT32_MAX - 3U) || !motion_test_low(9U)) return;
    motion_test_expect("natural seq wrap is continuous", g_test.core.stats.seq_gap_events == 0U && g_test.core.stats.seq_rollback_events == 0U);
    motion_test_accounting();
}

/*
 * 简介：验证输入门禁及非法调用不增加消费。
 * 输入：无，使用独立实例。
 * 返回值：无。
 * 说明：每项拒绝后保持不计步，正常配置版本来自冻结常量。
 */
static void motion_test_guards(void)
{
    uint64_t before;
    if (!motion_test_begin(1000U, 1U) || !motion_test_low(15U)) return;
    before = g_test.core.stats.dequeued;
    motion_test_expect("null frame rejected", motion_core_process(&g_test.core, NULL, g_test.now, &g_test.result) == MOTION_CORE_ERR_ARGUMENT);
    motion_test_expect("bad call does not consume", g_test.core.stats.dequeued == before);
    g_test.now += 20U;
    motion_test_make_frame(&g_test.frame, g_test.now, g_test.next_seq++, 1800000, &g_test.calendar);
    g_test.frame.imu.valid = false;
    if (!motion_test_process()) return;
    motion_test_expect("invalid classified", g_test.result.disposition == MOTION_FRAME_DISCARD_INVALID);
    g_test.now += 20U;
    motion_test_make_frame(&g_test.frame, g_test.now - 251U, g_test.next_seq++, 1800000, &g_test.calendar);
    if (!motion_test_process()) return;
    motion_test_expect("251ms sample stale", g_test.result.disposition == MOTION_FRAME_DISCARD_STALE);
    g_test.now += 20U;
    motion_test_make_frame(&g_test.frame, g_test.now, g_test.next_seq++, 1800000, &g_test.calendar);
    g_test.frame.imu.config_version++;
    if (!motion_test_process()) return;
    motion_test_expect("config mismatch classified", g_test.result.disposition == MOTION_FRAME_DISCARD_CONFIG);
    g_test.now += 20U;
    motion_test_make_frame(&g_test.frame, g_test.now + 1U, g_test.next_seq++, 1800000, &g_test.calendar);
    if (!motion_test_process()) return;
    motion_test_expect("future sample rejected", g_test.result.disposition == MOTION_FRAME_DISCARD_INVALID);
    motion_test_expect("rejected inputs do not count", g_test.core.snapshot.totals[0].boot_steps == 0U && g_test.core.snapshot.totals[1].boot_steps == 0U);
    motion_test_accounting();
}

/*
 * 简介：构造单调时间回绕、倒退和跨整圈年龄。
 * 输入：无，全部时间为函数参数。
 * 返回值：无。
 * 说明：长期场景由五次小于半区间的推进构成，不是实机长期运行证据。
 */
static void motion_test_clock(void)
{
    uint32_t i;
    if (!motion_test_begin(UINT32_MAX - 120U, 1U) || !motion_test_low(15U)) return;
    motion_test_expect("natural tick wrap no gap", g_test.core.stats.time_gap_events == 0U);
    motion_test_expect("backward now rejected", motion_core_tick(&g_test.core, g_test.now - 1U) == MOTION_CORE_ERR_CLOCK);
    motion_test_accounting();
    if (!motion_test_begin(1000U, 1U) || !motion_test_low(1U)) return;
    for (i = 0U; i < 5U; i++)
    {
        g_test.now += UINT32_C(0x40000000);
        motion_test_expect("bounded long clock advance", motion_core_tick(&g_test.core, g_test.now) == 0);
    }
    motion_test_expect("internal clock exceeds 32 bits", g_test.core.monotonic_ticks > UINT32_MAX);
    motion_test_expect("long age copy", motion_core_get_snapshot(&g_test.core, g_test.now, &g_test.snapshot) == 0);
    motion_test_expect("long age saturated, not fresh", g_test.snapshot.processed_age_ms == UINT32_MAX && g_test.core.input_timed_out);
}

/*
 * 简介：验证无输入超时和离线恢复的边界。
 * 输入：无，使用虚拟时间和状态。
 * 返回值：无。
 * 说明：STALE不伪造OFFLINE，重复超时不反复重置。
 */
static void motion_test_source(void)
{
    uint64_t resets;
    if (!motion_test_begin(1000U, 1U) || !motion_test_low(15U)) return;
    g_test.now += 250U;
    motion_test_expect("250ms tick accepted", motion_core_tick(&g_test.core, g_test.now) == 0);
    motion_test_expect("250ms not expired", !g_test.core.input_timed_out);
    g_test.now++;
    motion_test_expect("251ms tick accepted", motion_core_tick(&g_test.core, g_test.now) == 0);
    resets = g_test.core.stats.algorithm[0].reset_events;
    motion_test_expect("timeout preserves RUNNING", g_test.core.input_timed_out && g_test.core.snapshot.state == MOTION_SERVICE_RUNNING);
    g_test.now += 20U;
    motion_test_expect("repeated timeout call", motion_core_tick(&g_test.core, g_test.now) == 0);
    motion_test_expect("timeout reset once", g_test.core.stats.algorithm[0].reset_events == resets);
    if (!motion_test_low(11U)) return;
    motion_test_expect("new input recovers after warmup", !g_test.core.input_timed_out && (g_test.core.snapshot.quality_flags & MOTION_QUALITY_WARMUP) == 0U);
    motion_test_expect("offline transition", motion_core_update_source(&g_test.core, MOTION_SOURCE_OFFLINE, g_test.now) == 0);
    motion_test_expect("repeat offline transition", motion_core_update_source(&g_test.core, MOTION_SOURCE_OFFLINE, g_test.now) == 0);
    motion_test_expect("offline event once", g_test.core.stats.source_offline_events == 1U);
    motion_test_expect("source recovery", motion_core_update_source(&g_test.core, MOTION_SOURCE_AVAILABLE, g_test.now) == 0);
    if (!motion_test_low(11U)) return;
    motion_test_expect("recovery event once", g_test.core.stats.source_recovery_events == 1U);
    motion_test_accounting();
}

/*
 * 简介：验证未归日、回拨及三帧缓存的旧标签归属。
 * 输入：无，使用构造日期和独立缓存。
 * 返回值：无。
 * 说明：旧帧年龄分别为40/20/0ms，不用过期样本伪造计步。
 */
static void motion_test_days(void)
{
    uint32_t i, a;
    motion_calendar_tag_t old_tag;
    if (!motion_test_begin(1000U, 1U) || !motion_test_pulse(&g_test.calendar)) return;
    if (!motion_test_calendar(true, 20261107U, 1U) || !motion_test_pulse(&g_test.calendar)) return;
    old_tag = g_test.calendar;
    for (i = 0U; i < 3U; i++)
        motion_test_make_frame(&g_test.saved[i], g_test.now + (i + 1U) * 20U, g_test.next_seq++, 1800000, &old_tag);
    g_test.now += 60U;
    if (!motion_test_calendar(true, 20261108U, 2U)) return;
    for (i = 0U; i < 3U; i++)
    {
        g_test.frame = g_test.saved[i];
        if (!motion_test_process()) return;
    }
    /* 完成这一段高动态及低动态尾部，保持后续输入连续。 */
    for (i = 0U; i < 17U; i++) if (!motion_test_feed(1800000, &old_tag)) return;
    for (i = 0U; i < 20U; i++) if (!motion_test_feed(1000000, &old_tag)) return;
    motion_test_expect("old tags do not reverse current day", g_test.core.snapshot.current_day_key == 20261108U);
    for (a = 0U; a < 2U; a++)
        motion_test_expect("new day excludes old queued events", g_test.core.snapshot.current_day_steps[a] == 0U);
    if (!motion_test_pulse(&g_test.calendar) || !motion_test_calendar(true, 20261107U, 3U) || !motion_test_pulse(&g_test.calendar)) return;
    for (a = 0U; a < 2U; a++)
    {
        motion_test_expect("calendar changes preserve boot events", g_test.core.snapshot.totals[a].boot_steps == 5U);
        motion_test_expect("unassigned not backfilled", g_test.core.snapshot.totals[a].unassigned_steps == 1U);
        motion_test_expect("revisited date accumulates", g_test.core.snapshot.current_day_steps[a] == 3U);
    }
    motion_test_accounting();
}

/*
 * 简介：验证停止帧、旧运行代际和预热完成锁存。
 * 输入：无，使用独立实例。
 * 返回值：无。
 * 说明：半区间状态预置仅用于质量分支，不声称实际运行了24天。
 */
static void motion_test_epochs(void)
{
    uint32_t a;
    if (!motion_test_begin(1000U, 1U) || !motion_test_pulse(&g_test.calendar)) return;
    motion_test_expect("stop before epoch test", motion_core_stop(&g_test.core, g_test.now) == 0);
    if (!motion_test_feed(1800000, &g_test.calendar)) return;
    motion_test_expect("stopped frame discarded", g_test.result.disposition == MOTION_FRAME_DISCARD_STOPPED);
    motion_test_expect("new generation starts", motion_core_start(&g_test.core, 2U, 2U, g_test.now) == 0);
    if (!motion_test_feed(1800000, &g_test.calendar)) return; /* 工作区仍使用旧generation=1。 */
    motion_test_expect("old generation discarded", g_test.result.disposition == MOTION_FRAME_DISCARD_OLD_GENERATION);
    motion_test_expect("old frame did not establish filter", !g_test.core.detector[0].initialized && !g_test.core.detector[1].initialized);
    g_test.generation = 2U;
    if (!motion_test_low(15U)) return;
    for (a = 0U; a < 2U; a++)
    {
        motion_test_expect("warmup latched complete", g_test.core.warmup_completed[a]);
        g_test.core.detector[a].previous_tick = g_test.core.detector[a].warmup_start_tick + UINT32_C(0x80000000);
    }
    if (!motion_test_calendar(false, 0U, 1U)) return;
    motion_test_expect("long phase does not reenter warmup", (g_test.core.snapshot.quality_flags & MOTION_QUALITY_WARMUP) == 0U);
    motion_test_expect("reset after constructed state", motion_core_stop(&g_test.core, g_test.now) == 0);
    motion_test_expect("reset clears warmup latches", !g_test.core.warmup_completed[0] && !g_test.core.warmup_completed[1]);
    motion_test_accounting();
}

/*
 * 简介：验证日桶淘汰及业务计数饱和。
 * 输入：无，使用独立实例。
 * 返回值：无。
 * 说明：临界计数直接构造且保持初始闭合，不冒充真实长期计步。
 */
static void motion_test_history(void)
{
    uint32_t i, a;
    if (!motion_test_begin(1000U, 1U)) return;
    for (i = 0U; i < 9U; i++)
        if (!motion_test_calendar(true, 20261101U + i, i + 1U) || !motion_test_pulse(&g_test.calendar)) return;
    motion_test_expect("ninth day evicts once", g_test.core.stats.day_bucket_evictions == 1U);
    for (a = 0U; a < 2U; a++) motion_test_expect("evicted events archived", g_test.core.snapshot.totals[a].archived_day_steps == 1U);
    if (!motion_test_calendar(true, 20261101U, 10U) || !motion_test_pulse(&g_test.calendar)) return;
    motion_test_expect("revisited evicted day partial", !g_test.core.snapshot.current_day_complete);
    motion_test_accounting();
    if (!motion_test_begin(1000U, 1U)) return;
    for (a = 0U; a < 2U; a++)
    {
        g_test.core.snapshot.totals[a].boot_steps = UINT64_MAX - 1U;
        g_test.core.snapshot.totals[a].session_steps = UINT64_MAX - 1U;
        g_test.core.snapshot.totals[a].unassigned_steps = UINT64_MAX - 1U;
    }
    if (!motion_test_pulse(&g_test.calendar) || !motion_test_pulse(&g_test.calendar)) return;
    for (a = 0U; a < 2U; a++)
        motion_test_expect("saturation freezes complete accounting", g_test.core.counting_blocked[a] && g_test.core.snapshot.totals[a].boot_steps == UINT64_MAX && g_test.core.snapshot.totals[a].unassigned_steps == UINT64_MAX);
    motion_test_expect("saturation quality visible", (g_test.core.snapshot.quality_flags & MOTION_QUALITY_COUNTER_SATURATED) != 0U);
}

/*
 * 简介：通过新核心回放冻结黄金向量并逐事件比较。
 * 输入：无，黄金数组只读。
 * 返回值：无。
 * 说明：复用231帧、两套3事件，不修改任务10数据；不代表产品精度。
 */
static void motion_test_golden(void)
{
    uint32_t i, a, event_index[2] = {0U, 0U};
    uint32_t relative_ms;
    const step_detector_golden_sample_t *sample;
    const step_detector_golden_event_t *event;
    if (!motion_test_begin(g_step_detector_golden_samples[0].sample_tick, g_step_detector_golden_samples[0].imu_seq)) return;
    for (i = 0U; i < STEP_GOLDEN_SAMPLE_COUNT; i++)
    {
        sample = &g_step_detector_golden_samples[i];
        g_test.now = sample->sample_tick;
        motion_test_make_frame(&g_test.frame, sample->sample_tick, sample->imu_seq, sample->az_ug, &g_test.calendar);
        g_test.frame.imu.accel_ug[0] = sample->ax_ug;
        g_test.frame.imu.accel_ug[1] = sample->ay_ug;
        if (!motion_test_process()) return;
        for (a = 0U; a < 2U; a++)
        {
            if (!g_test.result.algorithm_called[a] || !g_test.result.algorithm_output[a].sample_accepted ||
                g_test.result.algorithm_output[a].magnitude_ug != sample->expected_magnitude_ug ||
                g_test.result.algorithm_output[a].dynamic_ug != sample->expected_dynamic_ug)
                motion_test_expect("golden per-frame acceptance/features", false);
            if (!g_test.result.algorithm_output[a].event_generated) continue;
            if (event_index[a] >= 3U) { motion_test_expect("golden extra event", false); continue; }
            event = (a == 0U) ? &g_step_detector_golden_b0_events[event_index[a]] : &g_step_detector_golden_b1_events[event_index[a]];
            relative_ms = STEP_GOLDEN_START_MS + (g_test.result.algorithm_output[a].event_tick - g_step_detector_golden_samples[0].sample_tick);
            motion_test_expect("golden event time/features", relative_ms == event->time_ms &&
                g_test.result.algorithm_output[a].dynamic_ug == event->dynamic_ug &&
                g_test.result.algorithm_output[a].filtered_dynamic_ug == event->filtered_dynamic_ug);
            event_index[a]++;
        }
    }
    motion_test_expect("golden 231 frames processed", g_test.core.stats.passed_to_alg == STEP_GOLDEN_SAMPLE_COUNT);
    for (a = 0U; a < 2U; a++)
        motion_test_expect("golden three unassigned events", event_index[a] == 3U && g_test.core.snapshot.totals[a].boot_steps == 3U && g_test.core.snapshot.totals[a].unassigned_steps == 3U);
    motion_test_accounting();
}

/*
 * 简介：串行运行九组独立测试并返回真实汇总。
 * 输入：summary非NULL；report可为空，上下文不保留到返回后。
 * 返回值：0全通过，-1测试/准备失败，-2参数非法。
 * 说明：只操作静态测试工作区；调用方负责防止并发和回调重入。
 */
int motion_core_test_run(motion_core_test_summary_t *summary,
                         motion_core_test_report_t report, void *user_data)
{
    static void (*const groups[MOTION_TEST_GROUPS])(void) =
    {
        motion_test_lifecycle, motion_test_sequence, motion_test_guards,
        motion_test_clock, motion_test_source, motion_test_days,
        motion_test_epochs, motion_test_history, motion_test_golden
    };
    uint32_t i;
    if (summary == NULL) return -2;
    memset(&g_test, 0, sizeof(g_test));
    g_test.report = report;
    g_test.user_data = user_data;
    for (i = 0U; i < MOTION_TEST_GROUPS; i++)
    {
        g_test.summary.groups_run++;
        groups[i]();
    }
    *summary = g_test.summary;
    g_test.report = NULL;
    g_test.user_data = NULL;
    return (summary->groups_run == MOTION_TEST_GROUPS && summary->checks != 0U &&
            summary->failed == 0U && summary->setup_errors == 0U) ? 0 : -1;
}
