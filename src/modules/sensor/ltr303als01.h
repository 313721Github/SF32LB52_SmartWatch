#ifndef LTR303ALS01_H
#define LTR303ALS01_H

#include "rtthread.h"
#include "sensor_types.h"

#define LTR303ALS01_ADDRESS            0x29U
#define LTR303ALS01_PART_ID_REG        0x86U
#define LTR303ALS01_MANUFAC_ID_REG     0x87U
#define LTR303ALS01_PART_ID_VALUE      0xA0U
#define LTR303ALS01_MANUFAC_ID_VALUE   0x05U

/**
 * @brief LTR303ALS01关键配置寄存器读回值
 *
 * 保存初始化完成后从器件实际读回的寄存器值。
 * 本结构只记录硬件读回事实，不保存配置是否匹配的派生判断。
 */
typedef struct
{
    rt_uint8_t als_contr;
    /**< ALS_CONTR（地址0x80）读回值。
         当前冻结配置期望为0x01，即ALS处于Active模式、
         软件复位位清零、增益为1X。 */

    rt_uint8_t als_meas_rate;
    /**< ALS_MEAS_RATE（地址0x85）读回值。
         当前冻结配置期望为0x02，即积分时间100 ms、
         测量更新周期200 ms。 */
} ltr303als01_config_readback_t;

rt_err_t ltr303als01_read_ids(rt_uint8_t *part_id,
                              rt_uint8_t *manufacturer_id);     //读取LTR303ALS01的PART_ID和MANUFACTURER_ID寄存器值
rt_err_t ltr303als01_init(
                    ltr303als01_config_readback_t *readback);   //验证身份并写入LTR303ALS01冻结配置
rt_err_t ltr303als01_estimate_lux(
                            rt_uint16_t ch0_raw,
                            rt_uint16_t ch1_raw,
                            rt_uint16_t gain,
                            rt_uint16_t integration_ms,
                            sensor_als_estimate_t *estimate);   //根据LTR303ALS01双通道原始值估算照度

rt_err_t ltr303als01_read_sample(
                                sensor_als_sample_t *sample);   //等待新数据并读取一份LTR303ALS01双通道样本
#endif
