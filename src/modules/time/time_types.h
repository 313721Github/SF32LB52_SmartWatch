#ifndef SMARTWATCH_TIME_TYPES_H
#define SMARTWATCH_TIME_TYPES_H
#include <stdint.h>
#include <stdbool.h>

#define TIME_UTC_MIN_SECONDS          UINT32_C(946684800)  // 最小UTC时间：2000-01-01 00:00:00
#define TIME_UTC_MAX_SECONDS          UINT32_C(4102444799) // 最大UTC时间：2099-12-31 23:59:59
#define TIME_DISPLAY_OFFSET_MINUTES   480                   // 显示时区偏移：UTC+8，共480分钟

// UTC校时日期，允许年份为2000～2099年。
typedef struct
{
    uint16_t year;    // 年：2000～2099
    uint8_t month;    // 月：1～12
    uint8_t day;      // 日：1～当月最大天数
    uint8_t hour;     // 时：0～23
    uint8_t minute;   // 分：0～59
    uint8_t second;   // 秒：0～59，不支持闰秒
} time_calendar_t;

// 校时来源。
typedef enum
{
    TIME_SOURCE_NONE = 0, // 尚无可信校时来源
    TIME_SOURCE_SHELL,    // 通过串口Shell提交校时
    TIME_SOURCE_BLE,      // 通过BLE提交校时，当前仅预留
    TIME_SOURCE_COUNT     // 来源数量及范围检查上限，不是有效来源
} time_source_t;

// 时间服务当前状态。
typedef enum
{
    TIME_CLOCK_BOOT = 0,   // 服务尚未完成启动
    TIME_CLOCK_NO_DEVICE,  // RTC设备不可用
    TIME_CLOCK_UNSET,      // 尚未完成可信校时
    TIME_CLOCK_SYNCING,    // 正在写入并确认校时时间
    TIME_CLOCK_READY,      // 时间可信且可正常使用
    TIME_CLOCK_READ_ERROR, // 最近一次RTC读取失败
    TIME_CLOCK_STALE,      // 距离上次有效读取过久，时间已过期
    TIME_CLOCK_INVALID,    // RTC候选时间超出允许范围
    TIME_CLOCK_UNCERTAIN   // 写入或确认失败，当前日历不能继续信任
} time_clock_state_t;

// 最近一次日历变更的原因。
typedef enum
{
    TIME_CHANGE_NONE = 0,      // 尚无日历变更事件
    TIME_CHANGE_SYNC_BEGIN,    // 开始校时，暂时撤销时间信任
    TIME_CHANGE_SYNC_OK,       // 校时及读回确认成功
    TIME_CHANGE_SYNC_FAILED,   // 校时写入或读回确认失败
    TIME_CHANGE_DAY,           // 正常走时导致显示时区的日期变化
    TIME_CHANGE_EXTERNAL_JUMP, // 检测到非本服务校时引起的异常时间跳变
    TIME_CHANGE_INVALID        // RTC候选时间超出允许范围
} time_change_reason_t;

// 最近一次时间错误发生的阶段。
typedef enum
{
    TIME_ERROR_NONE = 0, // 当前无错误
    TIME_ERROR_INIT,     // RTC适配环境或设备初始化失败
    TIME_ERROR_READ,     // RTC读取失败
    TIME_ERROR_WRITE,    // RTC写入失败
    TIME_ERROR_VERIFY,   // 校时后的读回确认失败
    TIME_ERROR_RANGE     // 时间范围检查失败
} time_error_stage_t;

// 线程间传递的时间副本，仅包含值，不包含设备句柄或内部指针。
typedef struct
{
    uint32_t utc_seconds;   // 最近一次范围合法的RTC读值，单位：Unix秒
    uint32_t candidate_utc; // 最近一次设备成功返回的原始UTC候选值，可能超范围
    uint32_t sample_tick;   // utc_seconds对应的读取完成tick
    uint32_t age_ms;        // 获取副本时计算的数据年龄，单位：毫秒

    uint32_t publish_seq;         // 快照发布序号，时间或状态发布时更新
    uint32_t calendar_generation; // 日历代际，用于区分校时及异常时间跳变
    uint32_t change_seq;          // 日历变更事件序号
    uint32_t day_seq;             // 显示日期变化序号，结合日历代际判断变化原因
    uint32_t local_day_key;       // 显示时区的日期标识，格式：YYYYMMDD

    bool service_running; // 时间服务是否已经启动
    bool has_value;       // 是否曾取得范围合法的RTC读值
    bool trusted;         // 当前日历是否具有已确认的可信校时来源
    bool fresh;           // 最近有效读值是否仍在新鲜度期限内

    time_source_t source;             // 可信校时来源
    time_clock_state_t state;         // 当前时间服务状态
    time_change_reason_t last_change; // 最近一次日历变更原因
    time_error_stage_t error_stage;   // 最近一次时间错误发生阶段
    int32_t last_error;               // 最近一次错误码，0表示无错误
} time_snapshot_t;

// 时间服务控制请求的类型。
typedef enum
{
    TIME_REQUEST_SET_UTC = 0, // 设置UTC时间并执行读回确认
    TIME_REQUEST_FAULT,       // 配置或清除软件故障注入
    TIME_REQUEST_TRACE        // 启动或停止后台时间跟踪
} time_request_kind_t;

// 一条请求的处理状态。
typedef enum
{
    TIME_RESULT_PENDING = 0, // 已登记，等待worker处理
    TIME_RESULT_RUNNING,     // worker正在处理
    TIME_RESULT_OK,          // 请求执行成功
    TIME_RESULT_FAILED       // 请求执行失败
} time_result_state_t;

// 有限容量的请求记录；ID为0表示空槽，等待或处理中记录不得回收。
typedef struct
{
    uint32_t request_id;        // 请求唯一序号，0表示槽位未使用
    time_request_kind_t kind;   // 请求类型
    time_result_state_t state;  // 请求处理状态
    int32_t error;              // 执行错误码，0表示无错误；仍须结合state判断是否完成

    uint32_t calendar_generation; // 请求完成时的日历代际
    uint32_t finished_tick;       // 请求完成时的单调tick，未完成时无效
    bool readback_available;      // 是否实际取得校时读回值，不代表校验通过
    uint32_t readback_utc;        // 校时读回的UTC候选值，仅在readback_available为真时有效
} time_request_result_t;

// 软件故障注入类型，默认关闭，按有限次数自动消耗。
typedef enum
{
    TIME_FAULT_NONE = 0, // 配合count=0清除全部故障注入
    TIME_FAULT_READ,     // 模拟普通RTC读取失败
    TIME_FAULT_WRITE,    // 模拟RTC写入失败
    TIME_FAULT_READBACK, // 模拟校时后的读回确认失败
    TIME_FAULT_MISSING,  // 模拟本轮RTC设备不可用，不删除真实设备
    TIME_FAULT_COUNT     // 枚举数量及范围检查上限，不是有效故障类型
} time_test_fault_t;

#endif
