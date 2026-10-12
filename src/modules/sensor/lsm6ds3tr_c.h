#ifndef LSM6DS3TR_C_H
#define LSM6DS3TR_C_H

#include "rtthread.h"
#include "sensor_types.h"
/*
 * SCH_V1.2实板于2026-10-09通过WHO_AM_I实测确认SA0为低，
 * 正常访问固定使用7位地址0x6A。
 */
#define LSM6DS3TR_C_ADDRESS            LSM6DS3TR_C_ADDRESS_SA0_LOW
#define LSM6DS3TR_C_ADDRESS_SA0_LOW   0x6AU
#define LSM6DS3TR_C_ADDRESS_SA0_HIGH  0x6BU
#define LSM6DS3TR_C_WHO_AM_I_REG      0x0FU
#define LSM6DS3TR_C_WHO_AM_I_VALUE    0x6AU


/**
 * @brief LSM6DS3TR-C关键配置寄存器读回值
 *
 * 保存初始化完成后从器件实际读回的原始寄存器值。
 * 本结构只记录硬件事实，不保存“是否匹配”的派生判断。
 */
typedef struct
{
    rt_uint8_t ctrl1_xl;
    /**< CTRL1_XL（地址0x10）读回值；
         bit[7:4]：加速度计输出数据率ODR_XL；
         bit[3:2]：加速度计量程FS_XL；
         当前冻结配置期望值：0x30，即52 Hz、±2 g */

    rt_uint8_t ctrl2_g;
    /**< CTRL2_G（地址0x11）读回值；
         bit[7:4]：陀螺仪输出数据率ODR_G；
         bit[3:2]：陀螺仪量程FS_G；
         bit[1]：FS_125；
         当前冻结配置期望值：0x30，即52 Hz、FS_G=00、FS_125=0 */

    rt_uint8_t ctrl3_c;
    /**< CTRL3_C（地址0x12）读回值；
         bit[6]：BDU；
         bit[2]：IF_INC；
         bit[1]：BLE；
         bit[0]：SW_RESET；
         当前冻结配置期望值：0x44，即BDU=1、IF_INC=1、
         BLE=0且SW_RESET已经自动清零 */
} lsm6ds3tr_c_config_readback_t;
rt_err_t lsm6ds3tr_c_read_id(rt_uint8_t device_address,rt_uint8_t *who_am_i);     //读取LSM6DS3TR-C的WHO_AM_I寄存器值
rt_err_t lsm6ds3tr_c_init(lsm6ds3tr_c_config_readback_t *readback);     //初始化LSM6DS3TR-C，读取关键寄存器值并保存到readback结构体

rt_err_t lsm6ds3tr_c_read_sample(sensor_imu_sample_t *sample);          //读取LSM6DS3TR-C的陀螺仪和加速度计采样数据
#endif
