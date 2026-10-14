#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 三轴16位原始数据
 *
 * 用于保存传感器寄存器输出的有符号原始值。
 * 数据尚未进行灵敏度换算。
 */
typedef struct
{
    rt_int16_t x;    /**< X轴原始值，单位：LSB */
    rt_int16_t y;    /**< Y轴原始值，单位：LSB */
    rt_int16_t z;    /**< Z轴原始值，单位：LSB */
} sensor_vec3_i16_t;

/**
 * @brief 三轴32位物理量数据
 *
 * 用于保存经过灵敏度换算后的定点物理量。
 * 具体单位由使用该结构体的字段决定。
 */
typedef struct
{
    rt_int32_t x;    /**< X轴换算值，单位由所属字段定义 */
    rt_int32_t y;    /**< Y轴换算值，单位由所属字段定义 */
    rt_int32_t z;    /**< Z轴换算值，单位由所属字段定义 */
} sensor_vec3_i32_t;

/**
 * @brief 三轴32位无符号原始数据
 *
 * 用于保存MMC5603NJ拼接后的20位无符号偏置编码。
 * 每个轴只有低20位有效。
 */
typedef struct
{
    rt_uint32_t x;
    rt_uint32_t y;
    rt_uint32_t z;
} sensor_vec3_u32_t;

/**
 * @brief LSM6DS3TR-C一次完整IMU采样结果
 *
 * 该结构体由调用者持有。驱动必须先在局部变量中完成12字节
 * burst读取、字节拼接和单位换算，全部成功后再整体复制到调用者
 * 缓冲区，避免向外暴露半新半旧的数据。
 */
typedef struct
{
    sensor_vec3_i16_t gyro_raw;
    /**< 陀螺仪三轴原始值，单位：LSB */

    sensor_vec3_i16_t accel_raw;
    /**< 加速度计三轴原始值，单位：LSB */

    sensor_vec3_i32_t gyro_udps;
    /**< 陀螺仪三轴角速度，单位：微度每秒（µdps）；
         当前配置下换算公式：gyro_udps = gyro_raw × 8750 */

    sensor_vec3_i32_t accel_ug;
    /**< 加速度计三轴加速度，单位：微重力加速度（µg）；
         当前配置下换算公式：accel_ug = accel_raw × 61 */

    rt_tick_t sample_tick;
    /**< 采样完成时间戳；
         在12字节burst成功完成、最后一个字节进入MCU后调用
         rt_tick_get()取得，单位为RT-Thread系统tick */

    rt_uint32_t config_version;
    /**< 产生该样本时使用的传感器配置版本；
         当前冻结配置版本为1，ODR、量程或换算规则改变时必须更新 */

    rt_uint8_t status_reg;
    /**< 本次burst读取前获得的LSM6DS3TR-C STATUS_REG原始值；
         bit0=XLDA表示新的加速度数据可用；
         bit1=GDA表示新的角速度数据可用；
         当前任务要求两位均有效后再读取完整样本 */

    rt_bool_t valid;
    /**< 整份样本是否有效；
         只有状态检查、12字节完整读取、解码和换算全部成功时才为RT_TRUE；
         任一步失败时不得把不完整样本发布为有效数据 */
} sensor_imu_sample_t;

/**
 * @brief MMC5603NJ一次完整三轴磁场样本
 *
 * 驱动先在局部变量中完成9字节读取、20位拼接、中心偏置
 * 处理和单位换算，全部成功后才复制到调用者缓冲区。
 */
typedef struct
{
    sensor_vec3_u32_t code_u20;
    /**< 三轴20位无符号输出编码，合法范围0～0xFFFFF */

    sensor_vec3_i32_t centered_raw;
    /**< 减去中心偏置524288后的有符号原始值 */

    sensor_vec3_i32_t field_nt;
    /**< 三轴磁场值，单位nT；
         换算公式为centered_raw × 100000 / 16384 */

    rt_tick_t sample_tick;
    /**< 9字节burst完整读取后的系统tick */

    rt_uint32_t config_version;
    /**< 当前单次测量、BW=00配置版本为1 */

    rt_uint8_t status_reg;
    /**< 数据读取前的STATUS1值；bit6为Meas_m_done */

    rt_bool_t valid;
    /**< 只有完整读取和换算全部成功后才为RT_TRUE */
} sensor_magnetometer_sample_t;

#ifdef __cplusplus
}
#endif

#endif