#ifndef SMARTWATCH_TIME_SERVICE_H
#define SMARTWATCH_TIME_SERVICE_H

#include "rtthread.h"
#include "time_types.h"

// 时间服务统计副本，不包含内部资源指针。
typedef struct
{
    uint32_t init_attempts;     // 服务初始化尝试次数
    uint32_t init_failures;     // 服务初始化失败次数

    uint32_t post_attempts;      // 控制请求投递尝试次数
    uint32_t post_accepted;      // 成功入队的请求次数，不代表执行成功
    uint32_t post_rejected;      // 投递阶段拒绝的请求次数
    uint32_t invalid_requests;   // 参数或类型非法的请求次数
    uint32_t completed_requests; // 已进入成功或失败终态的请求总数

    uint32_t read_attempts;     // 读取尝试次数，包含软件注入的失败尝试
    uint32_t read_success;      // 读取接口返回成功的次数，不代表时间可信
    uint32_t read_failures;     // 读取失败次数

    uint32_t write_attempts;    // 写入尝试次数，包含软件注入的失败尝试
    uint32_t write_success;     // SET接口返回成功的次数，不代表读回确认成功
    uint32_t write_failures;    // 写入失败次数
    uint32_t verify_failures;   // 校时读回失败或读回值不符合预期的次数
    uint32_t range_failures;    // RTC候选值超出业务时间范围的次数

    uint32_t external_jumps;       // 检测到外部异常日历跳变的次数
    uint32_t skipped_poll_periods; // 调度延误导致跳过的轮询周期数
    uint32_t worker_completed_tick;// worker最近完成一项工作的单调tick，不单独代表健康

    uint32_t last_io_ticks;     // 最近一次设备访问尝试耗时，单位：tick
    uint32_t max_io_ticks;      // 设备访问尝试的历史最大耗时，单位：tick
} time_service_stats_t;

rt_err_t time_service_init(void);// 创建服务资源并启动唯一time线程；成功不代表RTC已校时。

rt_err_t time_service_request_set_utc(uint32_t utc_seconds,
                                    time_source_t source,
                                    uint32_t *request_id);      // 非阻塞提交UTC校时请求；RT_EOK仅表示受理，执行结果按ID查询。
rt_err_t time_service_get_snapshot(time_snapshot_t *snapshot);  // 复制时间快照并计算当前age；输出不可为空，可信性须检查快照字段。
rt_err_t time_service_get_stats(time_service_stats_t *stats);   // 复制累计统计；输出不可为空，不返回内部结构指针。

rt_err_t time_service_get_result(uint32_t request_id,
                            time_request_result_t *result);     // 查询仍被保留的请求记录；RT_EOK表示找到记录，还须检查state和error。

rt_err_t time_service_test_inject(time_test_fault_t fault,
                                    uint32_t count,
                                    uint32_t *request_id);      // 提交有限次数的软件故障注入配置；默认关闭，执行结果按ID查询

rt_err_t time_service_request_trace(uint32_t duration_seconds,
                                    uint32_t interval_seconds,
                                    uint32_t *request_id);      // 提交后台跟踪配置，立即返回；不占用Shell等待跟踪结束

#endif
