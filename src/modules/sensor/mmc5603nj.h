#ifndef MMC5603NJ_H
#define MMC5603NJ_H

#include "rtthread.h"
#include "sensor_types.h"

#define MMC5603NJ_ADDRESS           0x30U
#define MMC5603NJ_PRODUCT_ID_REG    0x39U
#define MMC5603NJ_PRODUCT_ID_VALUE  0x10U

/**
 * @brief 显式SET/RESET配对测量结果
 *
 * SET和RESET样本保留完整原始证据。field_centered_raw和
 * offset_centered_raw由两份样本组合得到，field_nt是配对磁场结果。
 */
typedef struct
{
    sensor_magnetometer_sample_t set_sample;
    sensor_magnetometer_sample_t reset_sample;

    /* SET/RESET抵消后的磁场结果：(SET - RESET) / 2。 */
    sensor_vec3_i32_t field_centered_raw;
    sensor_vec3_i32_t field_nt;

    /* SET和RESET共有的偏置估计：(SET + RESET) / 2。 */
    sensor_vec3_i32_t offset_centered_raw;
} mmc5603nj_set_reset_result_t;

rt_err_t mmc5603nj_read_id(rt_uint8_t *product_id);     //读取MMC5603NJ的PRODUCT_ID寄存器值
rt_err_t mmc5603nj_init(void);                          //初始化MMC5603NJ
rt_err_t mmc5603nj_read_sample(sensor_magnetometer_sample_t *sample);           //读取MMC5603NJ的三轴磁场采样数据
rt_err_t mmc5603nj_run_set_reset_pair(mmc5603nj_set_reset_result_t *result);    //执行一次SET/RESET配对测量，获取磁场和偏置结果
rt_err_t mmc5603nj_run_ready_timeout_test(rt_uint32_t *timeout_elapsed_ms,
                            sensor_magnetometer_sample_t *recovered_sample);    //执行一次安全的ready超时与恢复测试
rt_err_t mmc5603nj_start_measurement(void);                                     //启动一次普通磁场测量,只写测量命令，不等待ready
rt_err_t mmc5603nj_try_read_sample(sensor_magnetometer_sample_t *sample);       //单次检查ready并在就绪时读取完整样本
rt_err_t mmc5603nj_reinitialize(void);                                          //强制重新验证身份并恢复冻结配置,只能由唯一sensor线程调用，用于离线重探
#endif
