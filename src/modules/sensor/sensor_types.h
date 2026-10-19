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

/**
 * @brief ALS照度估算结果分类
 *
 * 用于解释照度值是否可信以及失效原因。调用者不能只检查
 * lux_mlux，还必须同时检查lux_valid和reason。
 */
typedef enum
{
    SENSOR_ALS_REASON_OK = 0,           //双通道数据正常，照度估算结果有效。

    SENSOR_ALS_REASON_ZERO_COUNTS,
    /**< CH0和CH1均为0。
         此时比例没有数学意义，ratio_x10000约定保存为0；
         lux_mlux为0且lux_valid为RT_TRUE。 */

    SENSOR_ALS_REASON_IR_DOMINANT,
    /**< CH1/(CH0+CH1)大于等于0.85，红外成分占优。
         公式结果约定为0，但不能将其解释为黑暗环境，
         因此lux_valid为RT_FALSE。 */

    SENSOR_ALS_REASON_SATURATED,
    /**< CH0或CH1达到0xFFFF，传感器输出饱和。
         原始双通道数据仍可用于诊断，但照度估算不可信。 */

    SENSOR_ALS_REASON_DEVICE_INVALID,
    /**< ALS_STATUS的Invalid Data位表明当前器件数据无效。 */

    SENSOR_ALS_REASON_GAIN_MISMATCH,
    /**< ALS_STATUS报告的实际增益与任务7冻结配置不一致。
         不能使用错误的增益参数完成照度换算。 */
    SENSOR_ALS_REASON_MATH_ERROR
    /**< 照度计算出现负分子、结果越界等数学异常。
         不允许把异常结果静默截断或强制改成0。 */
} sensor_als_estimate_reason_t;

/**
 * @brief 根据一对CH0/CH1原始值生成的照度估算结果
 *
 * 本结构只描述纯数学换算，不包含I2C状态、时间戳或器件配置
 * 读回信息，因此可以同时用于构造值测试和实机样本换算。
 */
typedef struct
{
    rt_uint32_t ratio_x10000;
    /**< 通道比例CH1/(CH0+CH1)放大10000倍后的整数值。
         仅用于记录和诊断，公式分段必须使用交叉乘法判断，
         不能使用本字段的舍入结果决定分段。
         当CH0和CH1均为0时，本字段约定为0。 */

    rt_uint32_t lux_mlux;
    /**< 估算照度，单位为毫勒克斯（mlux）。
         1000 mlux等于1 lux。
         只有lux_valid为RT_TRUE时才能作为有效照度使用。 */

    rt_bool_t lux_valid;
    /**< 照度估算是否可信。
         RT_TRUE表示lux_mlux可以使用；
         RT_FALSE表示应根据reason识别饱和、红外占优或数学异常。 */

    rt_bool_t saturated;
    /**< 双通道是否存在饱和。
         CH0或CH1等于0xFFFF时置为RT_TRUE。
         饱和时保留原始值和比例诊断，但不得执行或采用照度公式。 */

    sensor_als_estimate_reason_t reason;
    /**< 当前估算结果的分类原因。
         调用者应将本字段与lux_valid一起检查。 */
} sensor_als_estimate_t;

/**
 * @brief LTR303ALS01一次完整双通道采样结果
 *
 * 驱动先在局部变量中完成STATUS检查、4字节burst读取、
 * 字节拼接和照度估算，全部处理完成后再整体复制到调用者
 * 缓冲区，避免向外暴露半新半旧的数据。
 *
 * 本结构由调用者持有，驱动不保存调用者传入的结构体指针。
 */
typedef struct
{
    rt_uint16_t ch0_raw;
    /**< CH0可见光加红外通道原始值。
         由寄存器0x8A低字节和0x8B高字节拼接得到。 */

    rt_uint16_t ch1_raw;
    /**< CH1红外通道原始值。
         由寄存器0x88低字节和0x89高字节拼接得到。
         驱动必须从0x88开始一次连续读取4字节，使CH0和CH1
         属于同一个测量周期。 */

    sensor_als_estimate_t estimate;
    /**< 根据本次CH0和CH1生成的照度估算与分类结果。 */

    rt_tick_t sample_tick;
    /**< 采样完成时间戳。
         在0x88～0x8B四字节burst完整读取、最后一个字节进入MCU后，
         通过rt_tick_get()取得，单位为RT-Thread系统tick。
         它表示软件完成读取的时刻，不等同于器件积分开始时刻。 */

    rt_uint32_t config_version;
    /**< 产生本样本时使用的ALS配置版本。
         当前冻结配置版本为1；增益、积分时间、更新周期或换算规则
         发生变化时必须递增，防止不同配置的数据被混为同一语义。 */

    rt_uint8_t status_reg;
    /**< 本次读取数据前获得的ALS_STATUS原始寄存器值。
         bit7为数据无效标志，值为0才表示有效；
         bits[6:4]报告实际增益；
         bit2为新数据标志。 */

    rt_uint16_t gain;
    /**< 本样本使用的实际增益倍数。
         当前冻结配置期望为1X，若状态寄存器报告其他增益，
         样本valid必须置为RT_FALSE并标记GAIN_MISMATCH。 */

    rt_uint16_t integration_ms;
    /**< ALS积分时间，单位毫秒。
         当前冻结值为100 ms，用于照度公式的积分时间归一化。 */

    rt_uint16_t repeat_ms;
    /**< 器件测量更新周期，单位毫秒。
         当前冻结值为200 ms，必须大于等于积分时间。 */

    rt_bool_t valid;
    /**< 原始双通道样本是否完整且符合器件状态和配置。
         完整burst、STATUS有效且实际增益匹配时为RT_TRUE。
         本字段为RT_TRUE仍不代表照度一定可信，还必须检查
         estimate.lux_valid。 */

    rt_bool_t calibrated;
    /**< 照度结果是否经过参考照度计标定。
         当前项目没有参考照度计，本任务固定为RT_FALSE；
         因此lux_mlux只能称为未标定照度估算值。 */
} sensor_als_sample_t;


/**
 * @brief 统一服务中的器件编号
 */
typedef enum
{
    SENSOR_DEVICE_IMU = 0,
    SENSOR_DEVICE_MMC,
    SENSOR_DEVICE_ALS,
    SENSOR_DEVICE_COUNT
} sensor_device_id_t;

/**
 * @brief 单个器件运行状态
 */
typedef enum
{
    SENSOR_DEVICE_STATE_UNINITIALIZED = 0,
    SENSOR_DEVICE_STATE_READY,
    SENSOR_DEVICE_STATE_TEMP_FAILED,
    SENSOR_DEVICE_STATE_OFFLINE
} sensor_device_state_t;

/**
 * @brief 单个器件随快照发布的状态
 *
 * valid表示服务曾经取得过完整有效样本。
 * state和stale决定这份last-good数据现在是否仍适合业务使用。
 */
typedef struct
{
    sensor_device_state_t state;//器件当前状态：未初始化、就绪、暂时失败或离线

    rt_bool_t valid;          //是否已经取得过完整有效样本
    rt_bool_t stale;          //last-good样本是否过期

    rt_uint32_t seq;          //器件样本序号
    rt_uint32_t config_version;//last-good样本使用的配置版本
    rt_tick_t sample_tick;    //last-good样本的采样完成tick
    rt_tick_t last_success_tick;//last-good样本的采样完成时间
    rt_tick_t age_ticks;      //调用者读取快照时，距离last_success_tick经过的tick数

    rt_err_t last_error;      //最近一次器件操作结果
} sensor_device_status_t;

/**
 * @brief 单个器件累计统计
 */
typedef struct
{
    rt_uint32_t attempt_count;     //实际发起器件操作的总次数
    rt_uint32_t success_count;     //完整有效样本成功读取、换算并提交的次数
    rt_uint32_t failure_count;     //实际失败次数，不包含正常的-RT_EBUSY未就绪结果
    rt_uint32_t timeout_count;     //测量、ready等待或底层操作超过规定上界的次数
    rt_uint32_t not_ready_count;   //单次检查返回-RT_EBUSY的次数，表示数据暂未就绪，不应直接等同于器件故障
    rt_uint32_t retry_count;       //器件处于TEMP_FAILED后执行受控重试的次数。不包含同一轮中的无限或紧密重试


    rt_uint32_t consecutive_failure_count;   //当前连续失败次数
    rt_uint32_t recovery_count;              //器件从TEMP_FAILED或OFFLINE恢复并成功提交第一份，完整有效样本的次数。首次正常启动不计为恢复

    rt_uint32_t late_count;                  //调度函数开始执行时已经晚于目标期限的次数
    rt_uint32_t missed_count;                //因执行时间过长或系统调度延迟而主动跳过的采样周期数

    rt_tick_t last_operation_ticks;          //最近一次实际驱动调用占用sensor线程的时间
    rt_tick_t max_operation_ticks;           //本次服务启动以来，单次实际驱动调用占用sensor线程的最大tick数，用于检查慢操作是否破坏最快采样周期
} sensor_device_stats_t;

/**
 * @brief 三器件最新一致快照
 *
 * 本结构始终复制给调用者，服务不得返回内部对象指针。
 */
typedef struct
{
    rt_uint32_t publish_seq;                 //服务级发布序号
    rt_tick_t snapshot_tick;                 //用者取得本快照副本的系统tick

    sensor_device_status_t imu_status;       // IMU当前状态、样本序号、年龄和最近错误
    sensor_device_status_t mmc_status;       //磁力计当前状态、样本序号、年龄和最近错误
    sensor_device_status_t als_status;       //环境光传感器当前状态、样本序号、年龄和最近错误

    sensor_imu_sample_t imu;                 //IMU最近一份完整有效的last-good样本
    sensor_magnetometer_sample_t mmc;        //MMC5603NJ最近一份完整有效的last-good样本
    sensor_als_sample_t als;                 //LTR303ALS01最近一份完整有效的last-good样本
} sensor_snapshot_t;

/**
 * @brief 统一服务统计
 */
typedef struct
{
    rt_uint32_t loop_count;        //sensor调度循环执行总次数
    rt_uint32_t wake_count;        //sensor线程从event等待中返回的总次数
    rt_uint32_t publish_count;     //快照内容或对外状态成功提交的总次数

    rt_uint32_t notify_count;      //通知回调成功执行并返回RT_EOK的次数
    rt_uint32_t notify_drop_count; //通知回调返回错误或下游拒绝通知的次数

    rt_uint32_t snapshot_lock_timeout_count;//外部读取快照或统计时，未能在规定上界内取得snapshot mutex的次
    rt_uint32_t probe_request_count;//外部成功提交重探请求的次数，多个请求可能通过器件位图合并，因此不等同于实际重探次数

    rt_tick_t last_loop_ticks;      //最近一轮sensor调度循环占用线程的tick数
    rt_tick_t max_loop_ticks;       //本次服务启动以来，单轮调度循环占用线程的最大tick数，用于判断整轮处理是否可能破坏最快采样周期

    sensor_device_stats_t imu;     //IMU累计采样、未就绪、失败、恢复和调度统计
    sensor_device_stats_t mmc;     //MMC5603NJ累计触发、检查、失败、恢复和调度统计
    sensor_device_stats_t als;     //LTR303ALS01累计检查、采样、失败、恢复和调度统计
} sensor_service_stats_t;

/**
 * @brief 诊断故障注入类型
 *
 * 默认必须为NONE。该类型只验证服务失败分支，不代表真实总线波形。
 */
typedef enum
{
    SENSOR_TEST_FAULT_NONE = 0,
    SENSOR_TEST_FAULT_BUS_IO,
    SENSOR_TEST_FAULT_SHORT_TRANSFER,
    SENSOR_TEST_FAULT_TIMEOUT
} sensor_test_fault_t;


#ifdef __cplusplus
}
#endif

#endif