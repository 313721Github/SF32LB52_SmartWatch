#include "lsm6ds3tr_c.h"
#include "sensor_bus_port.h"


/* LSM6DS3TR-C关键寄存器地址。 */
#define LSM6DS3TR_C_CTRL1_XL_REG             0x10U  /* 加速度ODR与量程 */
#define LSM6DS3TR_C_CTRL2_G_REG              0x11U  /* 陀螺仪ODR与量程 */
#define LSM6DS3TR_C_CTRL3_C_REG              0x12U  /* 复位、BDU和地址自增 */
#define LSM6DS3TR_C_STATUS_REG               0x1EU  /* 数据就绪状态 */
#define LSM6DS3TR_C_OUTX_L_G_REG             0x22U  /* 12字节burst起始地址 */

/* 任务5冻结的寄存器配置值。 */
#define LSM6DS3TR_C_SW_RESET_VALUE           0x01U  /* 触发软件复位 */
#define LSM6DS3TR_C_CTRL1_XL_VALUE           0x30U  /* 加速度：52 Hz、±2 g */
#define LSM6DS3TR_C_CTRL2_G_VALUE            0x30U  /* 陀螺仪：52 Hz、FS_G=00、FS_125=0、灵敏度8750 µdps/LSB */
#define LSM6DS3TR_C_CTRL3_C_VALUE            0x44U  /* BDU=1、IF_INC=1、BLE=0 */

/* 状态位掩码。 */
#define LSM6DS3TR_C_SW_RESET_MASK            0x01U  /* SW_RESET位 */
#define LSM6DS3TR_C_DATA_READY_MASK          0x03U  /* GDA和XLDA均就绪 */

/* 有限轮询参数：每2 ms检查一次，最多检查50次。 */
#define LSM6DS3TR_C_POLL_INTERVAL_MS         2U
#define LSM6DS3TR_C_RESET_POLL_COUNT         50U
#define LSM6DS3TR_C_READY_POLL_COUNT         50U

/*
 * CTRL2_G从Power-Down进入52 Hz高性能模式后，
 * 资料规定陀螺仪最长需要约70 ms启动稳定时间。
 */
#define LSM6DS3TR_C_GYRO_TURN_ON_MS          70U

/*
 * 52 Hz对应周期约19.23 ms。
 * 使用20 ms保证预热丢弃跨过不同的ODR输出周期。
 * 修改ODR时必须同步检查此常量和配置版本。
 */
#define LSM6DS3TR_C_SAMPLE_PERIOD_MS          20U

/* 样本格式、配置版本和定点换算系数。 */
#define LSM6DS3TR_C_BURST_LENGTH             12U    /* 陀螺仪和加速度共12字节 */
#define LSM6DS3TR_C_CONFIG_VERSION           1U     /* 当前冻结配置版本 */
#define LSM6DS3TR_C_ACCEL_UG_PER_LSB         61L    /* ±2 g：61 µg/LSB */
#define LSM6DS3TR_C_GYRO_UDPS_PER_LSB        8750L  /* 8.75 mdps/LSB */
#define LSM6DS3TR_C_WARMUP_SAMPLE_COUNT      3U     // 采样前丢弃的样本数，避免刚上电时的异常值
static rt_bool_t g_lsm6ds3tr_c_initialized = RT_FALSE;  //初始化状态仅由本驱动维护





//公开函数声明
rt_err_t lsm6ds3tr_c_read_id(rt_uint8_t device_address,
                             rt_uint8_t *who_am_i);         // 读取LSM6DS3TR-C的WHO_AM_I寄存器值
rt_err_t lsm6ds3tr_c_init(lsm6ds3tr_c_config_readback_t *readback);     //初始化LSM6DS3TR-C，读取关键寄存器值并保存到readback结构体
rt_err_t lsm6ds3tr_c_read_sample(sensor_imu_sample_t *sample);          //读取LSM6DS3TR-C的陀螺仪和加速度计采样数据
//私有函数声明
static rt_int16_t lsm6ds3tr_c_decode_i16(rt_uint8_t low,rt_uint8_t high); /* 将低字节和高字节解码为16位有符号原始值。 */
static rt_err_t lsm6ds3tr_c_wait_reset_complete(void);                    /* 有限等待软件复位位自动清零。 */
static rt_err_t lsm6ds3tr_c_wait_data_ready(void);                        /* 有限等待加速度和陀螺仪数据同时就绪。 */
static rt_err_t lsm6ds3tr_c_discard_warmup_samples(void);                 /* 丢弃上电后的异常样本。 */
//---------------------------------------------------------------公开函数---------------------------------------------------------------
/*
 * 简介：复位LSM6DS3TR-C，写入任务5冻结配置并验证实际读回值。
 * 输入：readback为可选输出缓冲区；允许为RT_NULL；缓冲区由调用者拥有。
 * 返回值：成功返回RT_EOK；身份或配置不匹配返回-RT_EIO；
 *         等待超时返回-RT_ETIMEOUT；总线失败返回底层错误。
 * 说明：只能在线程上下文调用；失败时保持驱动未初始化，
 *       且不修改调用者提供的readback。
 */
rt_err_t lsm6ds3tr_c_init(lsm6ds3tr_c_config_readback_t *readback)
{
    lsm6ds3tr_c_config_readback_t actual = {0};
    rt_uint8_t who_am_i = 0U;
    rt_uint8_t value;
    rt_err_t result;

    g_lsm6ds3tr_c_initialized = RT_FALSE;

    /* 确认器件身份。 */
    result = lsm6ds3tr_c_read_id(LSM6DS3TR_C_ADDRESS, &who_am_i);
    if (result != RT_EOK)
    {
        return result;
    }

    if (who_am_i != LSM6DS3TR_C_WHO_AM_I_VALUE)
    {
        return -RT_EIO;
    }
    /* 软件复位并有限等待复位完成。 */
    value = LSM6DS3TR_C_SW_RESET_VALUE;
    result = sensor_bus_port_write_reg(LSM6DS3TR_C_ADDRESS,
                                       LSM6DS3TR_C_CTRL3_C_REG,
                                       &value,
                                       1U);
    if (result != RT_EOK)
    {
        return result;
    }

    result = lsm6ds3tr_c_wait_reset_complete();
    if (result != RT_EOK)
    {
        return result;
    }
    /* 配置BDU、地址自增和字节序。 */
    value = LSM6DS3TR_C_CTRL3_C_VALUE;
    result = sensor_bus_port_write_reg(LSM6DS3TR_C_ADDRESS,
                                       LSM6DS3TR_C_CTRL3_C_REG,
                                       &value,
                                       1U);
    if (result != RT_EOK)
    {
        return result;
    }
    /* 配置加速度计：52 Hz、±2 g。 */
    value = LSM6DS3TR_C_CTRL1_XL_VALUE;
    result = sensor_bus_port_write_reg(LSM6DS3TR_C_ADDRESS,
                                       LSM6DS3TR_C_CTRL1_XL_REG,
                                       &value,
                                       1U);
    if (result != RT_EOK)
    {
        return result;
    }

    /* 配置陀螺仪：52 Hz、FS_G=00、FS_125=0、灵敏度8750 µdps/LSB。 */
    value = LSM6DS3TR_C_CTRL2_G_VALUE;
    result = sensor_bus_port_write_reg(LSM6DS3TR_C_ADDRESS,
                                       LSM6DS3TR_C_CTRL2_G_REG,
                                       &value,
                                       1U);
    if (result != RT_EOK)
    {
        return result;
    }

    /*
     * 分别读回三个寄存器，避免验证IF_INC时反过来依赖地址自增。
     */
    result = sensor_bus_port_read_reg(LSM6DS3TR_C_ADDRESS,
                                      LSM6DS3TR_C_CTRL1_XL_REG,
                                      &actual.ctrl1_xl,
                                      1U);
    if (result != RT_EOK)
    {
        return result;
    }

    result = sensor_bus_port_read_reg(LSM6DS3TR_C_ADDRESS,
                                      LSM6DS3TR_C_CTRL2_G_REG,
                                      &actual.ctrl2_g,
                                      1U);
    if (result != RT_EOK)
    {
        return result;
    }

    result = sensor_bus_port_read_reg(LSM6DS3TR_C_ADDRESS,
                                      LSM6DS3TR_C_CTRL3_C_REG,
                                      &actual.ctrl3_c,
                                      1U);
    if (result != RT_EOK)
    {
        return result;
    }

    /* 任一配置不匹配都不能宣布初始化成功。 */
    if ((actual.ctrl1_xl != LSM6DS3TR_C_CTRL1_XL_VALUE) ||
        (actual.ctrl2_g  != LSM6DS3TR_C_CTRL2_G_VALUE) ||
        (actual.ctrl3_c  != LSM6DS3TR_C_CTRL3_C_VALUE))
    {
        return -RT_EIO;
    }

    /*
    * 陀螺仪从Power-Down进入52 Hz高性能模式后，必须先等待模拟前端和数字滤波链完成启动稳定。
    * 该延时只允许在线程上下文执行。
    */
    rt_thread_mdelay(LSM6DS3TR_C_GYRO_TURN_ON_MS);

    /*
    * 丢弃传感器刚启动后的前三份完整六轴样本。
    * 每次都必须等待XLDA和GDA同时就绪，并通过12字节burst真正消费样本。
    */
    result = lsm6ds3tr_c_discard_warmup_samples();
    if (result != RT_EOK)
    {
        return result;
    }

    /*
    * 丢弃三份启动样本后，再等待下一份数据。
    * 这样read_sample()的首次调用能够读取第四份稳定样本。
    */
    result = lsm6ds3tr_c_wait_data_ready();
    if (result != RT_EOK)
    {
        return result;
    }

    /* 所有步骤通过后一次性提交结果。 */
    if (readback != RT_NULL)
    {
        *readback = actual;
    }

    g_lsm6ds3tr_c_initialized = RT_TRUE;

    return RT_EOK;
}


/*简介：读取LSM6DS3TR-C的WHO_AM_I寄存器值
  输入：device_address: LSM6DS3TR-C的I2C设备地址
        who_am_i: 指向存储WHO_AM_I寄存器值的缓冲区指针
  返回值：rt_err_t:错误类型
*/
rt_err_t lsm6ds3tr_c_read_id(rt_uint8_t device_address,
                             rt_uint8_t *who_am_i)
{
    if (who_am_i == RT_NULL)
    {
        return -RT_EINVAL;
    }

    *who_am_i = 0U;

    return sensor_bus_port_read_reg(device_address,
                                    LSM6DS3TR_C_WHO_AM_I_REG,
                                    who_am_i,
                                    1U);
}

/*
 * 简介：检查六轴数据就绪状态并通过一次12字节burst取得完整样本。
 * 输入：sample为调用者拥有的输出缓冲区，不允许为RT_NULL；
 *       函数不保存该指针，返回后由调用者继续拥有。
 * 返回值：完整采样成功返回RT_EOK；数据未同时就绪返回-RT_EBUSY；
 *         参数无效返回-RT_EINVAL；未初始化返回-RT_ENOSYS；
 *         总线失败返回底层错误。
 * 说明：只能在线程上下文调用；失败时不修改调用者原有sample。
 */
rt_err_t lsm6ds3tr_c_read_sample(sensor_imu_sample_t *sample)
{
    sensor_imu_sample_t candidate = {0};
    rt_uint8_t buffer[LSM6DS3TR_C_BURST_LENGTH] = {0};
    rt_uint8_t status = 0U;
    rt_tick_t completion_tick;
    rt_err_t result;

    if (sample == RT_NULL)
    {
        return -RT_EINVAL;
    }

    if (g_lsm6ds3tr_c_initialized == RT_FALSE)
    {
        return -RT_ENOSYS;
    }

    /* 检查加速度和陀螺仪是否同时就绪。 */
    result = sensor_bus_port_read_reg(LSM6DS3TR_C_ADDRESS,
                                      LSM6DS3TR_C_STATUS_REG,
                                      &status,
                                      1U);
    if (result != RT_EOK)
    {
        return result;
    }

    if ((status & LSM6DS3TR_C_DATA_READY_MASK) !=
        LSM6DS3TR_C_DATA_READY_MASK)
    {
        return -RT_EBUSY;
    }

    /* 一次读取陀螺仪和加速度计的六轴输出。 */
    result = sensor_bus_port_read_reg(LSM6DS3TR_C_ADDRESS,
                                      LSM6DS3TR_C_OUTX_L_G_REG,
                                      buffer,
                                      LSM6DS3TR_C_BURST_LENGTH);
    if (result != RT_EOK)
    {
        return result;
    }

    /* 时间戳表示burst最后一个字节已经进入MCU。 */
    completion_tick = rt_tick_get();

    /* 解析陀螺仪原始值。 */
    candidate.gyro_raw.x = lsm6ds3tr_c_decode_i16(buffer[0], buffer[1]);
    candidate.gyro_raw.y = lsm6ds3tr_c_decode_i16(buffer[2], buffer[3]);
    candidate.gyro_raw.z = lsm6ds3tr_c_decode_i16(buffer[4], buffer[5]);

    /* 解析加速度计原始值。 */
    candidate.accel_raw.x = lsm6ds3tr_c_decode_i16(buffer[6], buffer[7]);
    candidate.accel_raw.y = lsm6ds3tr_c_decode_i16(buffer[8], buffer[9]);
    candidate.accel_raw.z = lsm6ds3tr_c_decode_i16(buffer[10], buffer[11]);

    /* ±2 g配置：61 µg/LSB。 */
    candidate.accel_ug.x =
        (rt_int32_t)candidate.accel_raw.x *
        (rt_int32_t)LSM6DS3TR_C_ACCEL_UG_PER_LSB;

    candidate.accel_ug.y =
        (rt_int32_t)candidate.accel_raw.y *
        (rt_int32_t)LSM6DS3TR_C_ACCEL_UG_PER_LSB;

    candidate.accel_ug.z =
        (rt_int32_t)candidate.accel_raw.z *
        (rt_int32_t)LSM6DS3TR_C_ACCEL_UG_PER_LSB;

    /* FS_G=00、FS_125=0、灵敏度8750 µdps/LSB配置：8750 µdps/LSB。 */
    candidate.gyro_udps.x =
        (rt_int32_t)candidate.gyro_raw.x *
        (rt_int32_t)LSM6DS3TR_C_GYRO_UDPS_PER_LSB;

    candidate.gyro_udps.y =
        (rt_int32_t)candidate.gyro_raw.y *
        (rt_int32_t)LSM6DS3TR_C_GYRO_UDPS_PER_LSB;

    candidate.gyro_udps.z =
        (rt_int32_t)candidate.gyro_raw.z *
        (rt_int32_t)LSM6DS3TR_C_GYRO_UDPS_PER_LSB;

    candidate.sample_tick = completion_tick;
    candidate.config_version = LSM6DS3TR_C_CONFIG_VERSION;
    candidate.status_reg = status;
    candidate.valid = RT_TRUE;

    /* 完整构造成功后，一次性提交给调用者。 */
    *sample = candidate;

    return RT_EOK;
}
//---------------------------------------------------------------私有函数---------------------------------------------------------------
/*
 * 简介：把低字节在前的两个无符号字节解析为16位有符号补码。
 * 输入：low为低8位；high为高8位，参数均按原始寄存器值传入。
 * 返回值：解析后的rt_int16_t有符号原始值。
 * 说明：先使用无符号类型拼接，再通过32位中间值显式完成补码转换。
 */
static rt_int16_t lsm6ds3tr_c_decode_i16(rt_uint8_t low,
                                         rt_uint8_t high)
{
    rt_uint16_t unsigned_value;
    rt_int32_t signed_value;

    unsigned_value = ((rt_uint16_t)high << 8) | (rt_uint16_t)low;

    /*先把unsigned_value赋给signed_value。 */
    signed_value = (rt_int32_t)unsigned_value;

    /*如果最高位为1，说明值为负数，从signed_value中减去65536。 */
    if (signed_value & 0x8000)
    {
        signed_value -= 65536;
    }

    //返回显式转换后的rt_int16_t
    return (rt_int16_t)signed_value;
}

/*
 * 简介：有限轮询CTRL3_C，等待SW_RESET位自动清零。
 * 输入：无。
 * 返回值：复位完成返回RT_EOK；寄存器读取失败返回底层错误；
 *         超过约100 ms仍未清零返回-RT_ETIMEOUT。
 * 说明：只能在线程上下文调用；函数内部包含2 ms延时，禁止ISR调用。
 */
static rt_err_t lsm6ds3tr_c_wait_reset_complete(void)
{
    rt_uint32_t attempt;
    rt_uint8_t ctrl3_c;
    rt_err_t result;

    for (attempt = 0U;
         attempt < LSM6DS3TR_C_RESET_POLL_COUNT;
         attempt++)
    {
        ctrl3_c = 0U;

        /*读取CTRL3_C。 */
        result = sensor_bus_port_read_reg(LSM6DS3TR_C_ADDRESS,
                                          LSM6DS3TR_C_CTRL3_C_REG,
                                          &ctrl3_c,
                                          1U);

        /*读取失败时立即返回result。 */
        if (result != RT_EOK)
        {
            return result;
        }
        /*SW_RESET位为0时返回RT_EOK。 */
        if (!(ctrl3_c & LSM6DS3TR_C_SW_RESET_MASK))
        {
            return RT_EOK;
        }

        rt_thread_mdelay(LSM6DS3TR_C_POLL_INTERVAL_MS);
    }

    return -RT_ETIMEOUT;
}

/*
 * 简介：有限等待加速度和角速度数据同时就绪。
 * 输入：无。
 * 返回值：XLDA和GDA同时置位返回RT_EOK；读取失败返回底层错误；
 *         超过约100 ms仍未同时就绪返回-RT_ETIMEOUT。
 * 说明：只能在线程上下文调用；仅检查状态，不读取或清除输出数据。
 */
static rt_err_t lsm6ds3tr_c_wait_data_ready(void)
{
    rt_uint32_t attempt;
    rt_uint8_t status;
    rt_err_t result;

    for (attempt = 0U;
         attempt < LSM6DS3TR_C_READY_POLL_COUNT;
         attempt++)
    {
        status = 0U;

        /*读取STATUS_REG。 */
        result = sensor_bus_port_read_reg(LSM6DS3TR_C_ADDRESS,
                                          LSM6DS3TR_C_STATUS_REG,
                                          &status,
                                          1U);

        /*读取失败时立即返回result。 */
        if (result != RT_EOK)
        {
            return result;
        }

        /*(status & 0x03U)等于0x03U时返回RT_EOK。 */
        if ((status & LSM6DS3TR_C_DATA_READY_MASK) == LSM6DS3TR_C_DATA_READY_MASK)
        {
            return RT_EOK;
        }

        rt_thread_mdelay(LSM6DS3TR_C_POLL_INTERVAL_MS);
    }

    return -RT_ETIMEOUT;
}
/*
 * 简介：丢弃LSM6DS3TR-C上电后可能产生的异常样本。
 * 输入：无。
 * 返回值：丢弃成功返回RT_EOK；读取失败返回底层错误。
 * 说明：
 * 1. 只能在线程上下文调用；
 * 2. 每次都等待XLDA和GDA同时就绪；
 * 3. 每次通过12字节burst完整消费一份六轴样本；
 * 4. 相邻丢弃之间跨过一个52 Hz周期，避免重复消费同一输出窗口；
 * 5. 本函数不打印日志，错误交给上层测试入口统一记录。
 */
static rt_err_t lsm6ds3tr_c_discard_warmup_samples(void)
{
    rt_uint8_t buffer[LSM6DS3TR_C_BURST_LENGTH];
    rt_uint32_t index;
    rt_err_t result;

    for (index = 0U;
         index < LSM6DS3TR_C_WARMUP_SAMPLE_COUNT;
         index++)
    {
        result = lsm6ds3tr_c_wait_data_ready();
        if (result != RT_EOK)
        {
            return result;
        }

        result = sensor_bus_port_read_reg(
            LSM6DS3TR_C_ADDRESS,
            LSM6DS3TR_C_OUTX_L_G_REG,
            buffer,
            LSM6DS3TR_C_BURST_LENGTH);

        if (result != RT_EOK)
        {
            return result;
        }
        /*
         * 前两份丢弃样本后等待下一个ODR周期。
         * 第三份之后由init中的wait_data_ready()
         * 等待第四份样本，不在这里重复延时。
         */
        if ((index + 1U) < LSM6DS3TR_C_WARMUP_SAMPLE_COUNT)
        {
            rt_thread_mdelay(LSM6DS3TR_C_SAMPLE_PERIOD_MS);
        }
    }

    return RT_EOK;
}
