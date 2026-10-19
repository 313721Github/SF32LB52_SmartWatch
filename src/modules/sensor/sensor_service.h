#ifndef SENSOR_SERVICE_H
#define SENSOR_SERVICE_H

#include "rtthread.h"
#include "sensor_types.h"
#ifdef __cplusplus
extern "C" {
#endif

//设备位图
#define SENSOR_DEVICE_MASK_IMU    (1UL << SENSOR_DEVICE_IMU)
#define SENSOR_DEVICE_MASK_MMC    (1UL << SENSOR_DEVICE_MMC)
#define SENSOR_DEVICE_MASK_ALS    (1UL << SENSOR_DEVICE_ALS)

#define SENSOR_DEVICE_MASK_ALL    (SENSOR_DEVICE_MASK_IMU | SENSOR_DEVICE_MASK_MMC | SENSOR_DEVICE_MASK_ALS)


typedef rt_err_t (*sensor_notify_handler_t)(rt_uint32_t changed_mask,
                            rt_uint32_t publish_seq,void *user_data);                       //快照或状态变化通知

rt_err_t sensor_service_register_notify(sensor_notify_handler_t handler,void *user_data);   //注册服务通知回调

rt_err_t sensor_service_init(void);                                                         //创建资源并启动唯一sensor线程

rt_bool_t sensor_service_is_running(void);                                                  //查询统一服务是否已经运行

rt_err_t sensor_service_get_snapshot(sensor_snapshot_t *snapshot);                          //把最新一致快照复制到调用者缓冲区

rt_err_t sensor_service_get_stats(sensor_service_stats_t *stats);                           //复制统一服务累计统计

rt_err_t sensor_service_request_probe(rt_uint32_t device_mask);                             //请求sensor线程重探指定器件

rt_err_t sensor_service_test_inject(sensor_device_id_t device,sensor_test_fault_t fault,
                                                                        rt_uint32_t count); //配置固定次数的软件故障注入

#ifdef __cplusplus
}
#endif
#endif /* SENSOR_SERVICE_H */