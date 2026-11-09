#ifndef SMARTWATCH_MOTION_TRACE_H
#define SMARTWATCH_MOTION_TRACE_H

#include <rtthread.h>
#include "motion_core.h"

/* 项目侧诊断开关；置0可裁掉记录数组、Shell和业务钩子，不改SDK。 */
#ifndef MOTION_TRACE_ENABLED
#define MOTION_TRACE_ENABLED 1
#endif

#define MOTION_TRACE_FORMAT_VERSION 1U
#ifndef MOTION_TRACE_ROW_CAPACITY
#define MOTION_TRACE_ROW_CAPACITY 1600U
#endif
#define MOTION_TRACE_MAX_SECONDS 30U
#define MOTION_TRACE_MAX_DELAY_SECONDS 60U
#define MOTION_TRACE_FIRST_FRAME_WAIT_MS 500U
#define MOTION_TRACE_EXPORT_MAX_ROWS 64U

typedef enum
{
    MOTION_TRACE_EMPTY = 0, MOTION_TRACE_CONTROL_BUSY,
    MOTION_TRACE_ARMED, MOTION_TRACE_RECORDING,
    MOTION_TRACE_FROZEN, MOTION_TRACE_EXPORTING
} motion_trace_state_t;

typedef enum
{
    MOTION_TRACE_END_NONE = 0, MOTION_TRACE_END_DURATION,
    MOTION_TRACE_END_USER_CANCEL, MOTION_TRACE_END_OVERFLOW,
    MOTION_TRACE_END_NO_INPUT, MOTION_TRACE_END_DATA_QUALITY,
    MOTION_TRACE_END_MOTION_STOP, MOTION_TRACE_END_MOTION_FAULT,
    MOTION_TRACE_END_TEST_CONFLICT
} motion_trace_end_t;

#define MTR_F_IMU_VALID          (UINT32_C(1) << 0)
#define MTR_F_B0_CALLED          (UINT32_C(1) << 1)
#define MTR_F_B1_CALLED          (UINT32_C(1) << 2)
#define MTR_F_B0_ACCEPTED        (UINT32_C(1) << 3)
#define MTR_F_B1_ACCEPTED        (UINT32_C(1) << 4)
#define MTR_F_B0_EVENT           (UINT32_C(1) << 5)
#define MTR_F_B1_EVENT           (UINT32_C(1) << 6)
#define MTR_F_B0_RESET           (UINT32_C(1) << 7)
#define MTR_F_B1_RESET           (UINT32_C(1) << 8)
#define MTR_F_B1_ARMED_BEFORE    (UINT32_C(1) << 9)
#define MTR_F_B1_ARMED_AFTER     (UINT32_C(1) << 10)
#define MTR_F_B1_HAS_LAST_BEFORE (UINT32_C(1) << 11)
#define MTR_F_B1_HAS_LAST_AFTER  (UINT32_C(1) << 12)
#define MTR_F_B1_INIT_BEFORE     (UINT32_C(1) << 13)
#define MTR_F_B1_INIT_AFTER      (UINT32_C(1) << 14)
#define MTR_F_FATAL             (UINT32_C(1) << 15)
#define MTR_F_DYNAMIC_VALID     (UINT32_C(1) << 16)
#define MTR_F_CORE_OK           (UINT32_C(1) << 17)
#define MTR_F_INJECTED          (UINT32_C(1) << 18)

/* detail: [3:0]分类(7注入/8内部失败)，[7:4]/[11:8]B0/B1 reason，
 * [13:12]/[15:14]算法返回值+1，[31:16]reset_reasons；CALLED区分未调用。 */
typedef struct
{
    uint32_t sample_tick, imu_seq, enqueue_tick, process_tick;
    int32_t accel_ug[3];
    uint32_t dynamic_ug, filtered_ug, b1_last_event_before;
    uint32_t flags, detail;
} motion_trace_row_t;

typedef char motion_trace_row_size_check[(sizeof(motion_trace_row_t) == 48U) ? 1 : -1];

/* 仅值副本；CSV逐字段输出，不把ABI/padding作为协议。 */
typedef struct
{
    step_detector_t detector[MOTION_ALGORITHM_COUNT];
    motion_core_stats_t core_stats;
    uint64_t boot_steps[MOTION_ALGORITHM_COUNT];
    uint64_t session_steps[MOTION_ALGORITHM_COUNT];
    uint32_t session_id, run_generation, source_state, quality_flags;
    uint32_t previous_imu_seq, previous_sample_tick;
    bool has_previous_seq, has_previous_tick;
} motion_trace_checkpoint_t;

typedef struct
{
    uint32_t format_version, capture_id, tick_hz, row_size, capacity;
    uint32_t duration_ms, delay_ms, armed_tick, scheduled_start_tick;
    uint32_t start_process_tick, stop_deadline_tick, end_process_tick;
    uint32_t first_sample_tick, last_sample_tick, initial_imu_config;
    uint32_t rows_seen, rows_stored, rows_not_stored;
    uint32_t max_hook_ticks, max_frame_ticks;
    motion_trace_end_t end_reason;
    int32_t last_core_error;
    bool start_valid, continuous, completed;
    uint32_t bad_input_seq, bad_input_config, bad_input_generation;
    motion_trace_checkpoint_t before, after;
} motion_trace_meta_t;

typedef struct
{
    bool ready;
    motion_trace_state_t state;
    uint32_t capture_id, rows_stored, capacity;
    uint32_t scheduled_start_tick, start_process_tick, stop_deadline_tick;
    motion_trace_end_t end_reason;
    int32_t error;
} motion_trace_status_t;

#if MOTION_TRACE_ENABLED
void motion_trace_worker_init(void);
rt_err_t motion_trace_worker_control(motion_request_kind_t kind, uint32_t request_id,
    uint32_t argument, const motion_core_t *core, bool injection_active,
    bool stop_pending, uint32_t now_tick);
void motion_trace_worker_poll(const motion_core_t *core, bool injection_active,
    bool stop_pending, uint32_t now_tick);
void motion_trace_worker_before_frame(const motion_core_t *core,
    const motion_sample_frame_t *frame, uint32_t now_tick);
void motion_trace_worker_after_frame(const motion_core_t *core,
    const motion_sample_frame_t *frame, bool core_called, int core_result,
    const motion_core_frame_result_t *frame_result, bool injected, uint32_t now_tick);
void motion_trace_worker_abort(const motion_core_t *core, motion_trace_end_t reason,
    int32_t error, uint32_t now_tick);
bool motion_trace_worker_is_active(void);
rt_err_t motion_trace_get_status(motion_trace_status_t *status);
rt_err_t motion_trace_export_begin(uint32_t capture_id, uint32_t *lease,
    motion_trace_meta_t *meta);
rt_err_t motion_trace_export_get_row(uint32_t lease, uint32_t index, motion_trace_row_t *row);
rt_err_t motion_trace_export_end(uint32_t lease);
#else
/* 简介：裁剪模式的无副作用内联入口。输入：与启用接口一致。
 * 返回值：控制返回-ENOSYS，活动判断false；void无返回。
 * 说明：业务钩子被编译器消除，无记录数组或Shell。 */
static inline void motion_trace_worker_init(void) {}
static inline bool motion_trace_worker_is_active(void) { return false; }
static inline rt_err_t motion_trace_worker_control(motion_request_kind_t k, uint32_t id,
    uint32_t a, const motion_core_t *c, bool f, bool s, uint32_t n)
{ (void)k; (void)id; (void)a; (void)c; (void)f; (void)s; (void)n; return -RT_ENOSYS; }
static inline void motion_trace_worker_poll(const motion_core_t *c, bool f, bool s, uint32_t n)
{ (void)c; (void)f; (void)s; (void)n; }
static inline void motion_trace_worker_before_frame(const motion_core_t *c,
    const motion_sample_frame_t *f, uint32_t n) { (void)c; (void)f; (void)n; }
static inline void motion_trace_worker_after_frame(const motion_core_t *c,
    const motion_sample_frame_t *f, bool called, int r,
    const motion_core_frame_result_t *o, bool i, uint32_t n)
{ (void)c; (void)f; (void)called; (void)r; (void)o; (void)i; (void)n; }
static inline void motion_trace_worker_abort(const motion_core_t *c,
    motion_trace_end_t r, int32_t e, uint32_t n) { (void)c; (void)r; (void)e; (void)n; }
#endif
#endif
