#ifndef SMARTWATCH_MOTION_SERVICE_H
#define SMARTWATCH_MOTION_SERVICE_H

#include <rtthread.h>
#include "motion_types.h"
#ifdef __cplusplus
extern "C" {
#endif

/*
 * 初始资源预算，正式验收前结合sizeof、MAP和实测确认。
 * 16槽在约50 Hz时对应约320 ms容量，不是正常允许的排队时间。
 */
#define MOTION_SAMPLE_QUEUE_CAPACITY       16U
#define MOTION_CONTROL_QUEUE_CAPACITY      4U
#define MOTION_RESULT_SLOT_COUNT           8U

/* 包含完整motion_sample_frame_t的初始单帧预算。 */
#define MOTION_SAMPLE_FRAME_BUDGET_BYTES    128U

//公开函数声明
rt_err_t motion_service_init(void);
rt_err_t motion_service_request_start(uint32_t *request_id);            //非阻塞提交START请求
rt_err_t motion_service_request_stop(uint32_t *request_id);             //非阻塞提交STOP请求
rt_err_t motion_service_get_result(uint32_t request_id,
                                  motion_request_result_t *result);     //查询有限结果槽中的一条控制结果
rt_err_t motion_service_get_snapshot(motion_snapshot_t *snapshot);      //取得线程间发布的运动摘要副本
rt_err_t motion_service_get_stats(motion_service_stats_t *stats);       //合并已发布消费统计与短临界区生产/控制统计
rt_err_t motion_service_submit_imu(const motion_imu_sample_t *sample);  //从唯一sensor生产者复制并非阻塞投递一帧

/*
 * 简介：提交有限诊断配置或撤销请求。
 * 输入：fault/argument符合上限；request_id非RT_NULL，失败输出0。
 * 返回值：受理RT_EOK；参数-EINVAL；资源-EFULL；不可用-ERROR。
 * 说明：仅线程调用，经worker执行；完成OK表示已配置，不代表测试通过。
 */
rt_err_t motion_service_request_test(motion_test_fault_t fault, uint32_t argument,
                                     uint32_t *request_id);
/*
 * 简介：取得最近发布的诊断状态值副本。
 * 输入：status非RT_NULL，仅本次写入，不保存指针。
 * 返回值：成功RT_EOK；参数-EINVAL；繁忙-EBUSY；不可用-ERROR。
 * 说明：线程上下文，有界取发布锁；还须检查published_tick的新鲜度。
 */
rt_err_t motion_service_get_test_status(motion_test_status_t *status);

/* 仅诊断：受理不代表采集已开始，使用原get_result查询执行结果。 */
rt_err_t motion_service_request_trace_arm(uint32_t duration_s, uint32_t delay_s, uint32_t *request_id);
rt_err_t motion_service_request_trace_cancel(uint32_t capture_id, uint32_t *request_id);
rt_err_t motion_service_request_trace_clear(uint32_t capture_id, uint32_t *request_id);

#ifdef __cplusplus
}
#endif

#endif /* SMARTWATCH_MOTION_SERVICE_H */
