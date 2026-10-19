#include "ltr303als01.h"
#include "sensor_bus_port.h"

/* LTR303ALS01关键寄存器地址。 */
#define LTR303ALS01_ALS_CONTR_REG             0x80U  /* 工作模式、软件复位和增益控制 */
#define LTR303ALS01_ALS_MEAS_RATE_REG         0x85U  /* 积分时间和测量更新周期 */
#define LTR303ALS01_DATA_CH1_0_REG            0x88U  /* 4字节burst读取起点：CH1低字节 */
#define LTR303ALS01_ALS_STATUS_REG            0x8CU  /* 数据有效性、实际增益和NEW状态 */

/* 任务7冻结配置值。 */
#define LTR303ALS01_ALS_CONTR_STANDBY          0x00U  /* Standby、1X增益 */
#define LTR303ALS01_ALS_CONTR_ACTIVE_1X        0x01U  /* Active、1X增益 */
#define LTR303ALS01_ALS_MEAS_RATE_VALUE        0x02U  /* 100 ms积分、200 ms更新 */

/* ALS_STATUS寄存器位定义。 */
#define LTR303ALS01_STATUS_INVALID_MASK        0x80U  /* bit7：1表示当前数据无效 */
#define LTR303ALS01_STATUS_GAIN_MASK           0x70U  /* bits[6:4]：实际增益编码 */
#define LTR303ALS01_STATUS_GAIN_SHIFT          4U     /* 实际增益编码右移位数 */
#define LTR303ALS01_STATUS_NEW_MASK            0x04U  /* bit2：1表示存在新数据 */

/* 双通道读取和有限等待参数。 */
#define LTR303ALS01_DATA_LENGTH                4U     /* CH1_L/H和CH0_L/H共4字节 */
#define LTR303ALS01_READY_POLL_COUNT           25U    /* 最多读取25次STATUS */
#define LTR303ALS01_READY_POLL_INTERVAL_MS     10U    /* 未就绪时等待10 ms */
#define LTR303ALS01_ACTIVE_WAKE_MS             10U    /* Standby进入Active后的最短等待 */

/* 当前样本携带的冻结配置语义。 */
#define LTR303ALS01_GAIN                       1U     /* 实际增益倍数：1X */
#define LTR303ALS01_INTEGRATION_MS             100U   /* 积分时间，单位ms */
#define LTR303ALS01_REPEAT_MS                  200U   /* 测量更新周期，单位ms */
#define LTR303ALS01_CONFIG_VERSION             1U     /* 配置或换算规则改变时递增 */
/*
 * 驱动初始化状态，仅由LTR驱动在线程上下文中维护。
 * 它不代表当前一定有新数据，只表示身份和冻结配置已经验证通过。
 */
static rt_bool_t g_ltr303als01_initialized = RT_FALSE;

//公开函数声明
rt_err_t ltr303als01_init(
                    ltr303als01_config_readback_t *readback);   //验证身份并写入LTR303ALS01冻结配置

rt_err_t ltr303als01_read_ids(rt_uint8_t *part_id,
                              rt_uint8_t *manufacturer_id);         // 读取LTR303ALS01的PART_ID和MANUFACTURER_ID寄存器值

rt_err_t ltr303als01_estimate_lux(
                            rt_uint16_t ch0_raw,
                            rt_uint16_t ch1_raw,
                            rt_uint16_t gain,
                            rt_uint16_t integration_ms,
                            sensor_als_estimate_t *estimate);   //根据LTR303ALS01双通道原始值估算照度

rt_err_t ltr303als01_read_sample(sensor_als_sample_t *sample);      //阻塞等待NEW并读取一份完整双通道样本
rt_err_t ltr303als01_try_read_sample(sensor_als_sample_t *sample);  //单次检查NEW并在就绪时读取完整双通道样本 没有NEW返回-RT_EBUSY，不延时、不循环
rt_err_t ltr303als01_reinitialize(ltr303als01_config_readback_t *readback); //强制重新验证LTR303身份并恢复冻结配置
//私有函数声明

static rt_err_t ltr303als01_read_config(
                    ltr303als01_config_readback_t *readback);   //读取ALS_CONTR和ALS_MEAS_RATE；成功后一次性更新readback

static rt_err_t ltr303als01_wait_new_data(
                                    rt_uint8_t *status_reg);    //有限轮询ALS_STATUS.NEW，并返回检测到新数据时的完整STATUS值

static rt_uint16_t ltr303als01_decode_u16(
                                            rt_uint8_t low,
                                            rt_uint8_t high);   //按小端顺序把低、高字节拼接为16位无符号原始值

static rt_uint16_t ltr303als01_decode_status_gain(
                                    rt_uint8_t status_reg);     //将ALS_STATUS bits[6:4]的增益编码转换为实际增益倍数，未定义或保留编码返回0，由调用者判定为配置不匹配。
static rt_err_t ltr303als01_read_sample_with_status(rt_uint8_t status,
                                    sensor_als_sample_t *sample);//使用已经确认存在NEW的STATUS读取并解释完整双通道样本
//---------------------------------------------------------------公开函数---------------------------------------------------------------
/*
 * 简介：验证LTR303身份，写入冻结配置并核对实际读回值。
 * 输入：readback为可选输出，允许为RT_NULL。
 * 返回值：完整初始化成功返回RT_EOK；
 *         身份或配置不匹配返回-RT_EIO；
 *         总线失败返回底层错误。
 * 说明：调用前必须完成sensor_bus_port_init()；
 *       失败时保持驱动未初始化，不修改调用者readback。
 */
rt_err_t ltr303als01_init(ltr303als01_config_readback_t *readback)
{
    ltr303als01_config_readback_t actual = {0};

    rt_uint8_t part_id = 0U;
    rt_uint8_t manufacturer_id = 0U;
    rt_uint8_t value;

    rt_err_t result;

    /*
     * 重复初始化时先核对器件当前配置。
     * 配置仍然匹配时不重复切换Standby和Active。
     */
    if (g_ltr303als01_initialized == RT_TRUE)
    {
        result = ltr303als01_read_config(&actual);
        if (result != RT_EOK)
        {
            g_ltr303als01_initialized = RT_FALSE;
            return result;
        }

        if ((actual.als_contr ==LTR303ALS01_ALS_CONTR_ACTIVE_1X) &&
            (actual.als_meas_rate ==LTR303ALS01_ALS_MEAS_RATE_VALUE))
        {
            if (readback != RT_NULL)
            {
                *readback = actual;
            }

            return RT_EOK;
        }

        /*
         * 配置被外部改变时清除初始化状态，并重新执行完整初始化。
         */
        g_ltr303als01_initialized = RT_FALSE;
    }

    /* 读取并核对器件身份。 */
    result = ltr303als01_read_ids(
                &part_id,
                &manufacturer_id);
    if (result != RT_EOK)
    {
        return result;
    }

    if ((part_id != LTR303ALS01_PART_ID_VALUE) ||
        (manufacturer_id != LTR303ALS01_MANUFAC_ID_VALUE))
    {
        return -RT_EIO;
    }

    /* 进入Standby，避免在修改测量配置时继续产生样本。 */
    value = LTR303ALS01_ALS_CONTR_STANDBY;
    result = sensor_bus_port_write_reg(
        LTR303ALS01_ADDRESS,
        LTR303ALS01_ALS_CONTR_REG,
        &value,
        1U);
    if (result != RT_EOK)
    {
        return result;
    }

    /* 配置100 ms积分时间和200 ms测量更新周期。 */
    value = LTR303ALS01_ALS_MEAS_RATE_VALUE;
    result = sensor_bus_port_write_reg(
        LTR303ALS01_ADDRESS,
        LTR303ALS01_ALS_MEAS_RATE_REG,
        &value,
        1U);
    if (result != RT_EOK)
    {
        return result;
    }
    /* 以1X增益进入Active模式。 */
    value = LTR303ALS01_ALS_CONTR_ACTIVE_1X;
    result = sensor_bus_port_write_reg(
        LTR303ALS01_ADDRESS,
        LTR303ALS01_ALS_CONTR_REG,
        &value,
        1U);
    if (result != RT_EOK)
    {
        return result;
    }

    /*
     * 满足Standby进入Active的最短唤醒时间。
     * 首份完整积分数据仍由read_sample()有限轮询NEW。
     */
    rt_thread_mdelay(LTR303ALS01_ACTIVE_WAKE_MS);

    /* 读取器件实际配置。 */
    result = ltr303als01_read_config(&actual);
    if (result != RT_EOK)
    {
        return result;
    }

    /* 配置读回必须与任务7冻结值完全一致。 */
    if ((actual.als_contr !=LTR303ALS01_ALS_CONTR_ACTIVE_1X) ||
        (actual.als_meas_rate !=LTR303ALS01_ALS_MEAS_RATE_VALUE))
    {
        return -RT_EIO;
    }

    /*
     * 只有身份、写入和读回全部通过后，才发布配置和初始化状态。
     */
    if (readback != RT_NULL)
    {
        *readback = actual;
    }

    g_ltr303als01_initialized = RT_TRUE;

    return RT_EOK;
}


/*简介：读取LTR303ALS01的PART_ID和MANUFACTURER_ID寄存器值
  输入：part_id: 指向存储PART_ID寄存器值的缓冲区指针
        manufacturer_id: 指向存储MANUFACTURER_ID寄存器值的缓冲区指针
  返回值：rt_err_t:错误类型
*/
rt_err_t ltr303als01_read_ids(rt_uint8_t *part_id,
                              rt_uint8_t *manufacturer_id)
{
    rt_err_t result;

    if ((part_id == RT_NULL) || (manufacturer_id == RT_NULL))
    {
        return -RT_EINVAL;
    }

    *part_id = 0U;
    *manufacturer_id = 0U;

    result = sensor_bus_port_read_reg(LTR303ALS01_ADDRESS,
                                      LTR303ALS01_PART_ID_REG,
                                      part_id,
                                      1U);
    if (result != RT_EOK)
    {
        return result;
    }

    result = sensor_bus_port_read_reg(LTR303ALS01_ADDRESS,
                                      LTR303ALS01_MANUFAC_ID_REG,
                                      manufacturer_id,
                                      1U);
    if (result != RT_EOK)
    {
        return result;
    }

    return RT_EOK;
}
/*
 * 简介：根据同一测量周期的CH0/CH1估算照度。
 * 返回值：参数错误返回-RT_EINVAL；
 *         成功生成分类结果返回RT_EOK。
 * 说明：ZERO_COUNTS、SATURATED、IR_DOMINANT和MATH_ERROR
 *       都属于已经成功生成的分类结果，因此返回RT_EOK；
 *       调用者通过lux_valid和reason判断能否使用照度值。
 */
rt_err_t ltr303als01_estimate_lux(rt_uint16_t ch0_raw,
                                    rt_uint16_t ch1_raw,
                                    rt_uint16_t gain,
                                    rt_uint16_t integration_ms,
                                    sensor_als_estimate_t *estimate)
{
    sensor_als_estimate_t candidate = {0};

    rt_uint64_t sum;
    rt_uint64_t ratio_left;
    rt_uint64_t denominator;
    rt_uint64_t rounded_numerator;
    rt_uint64_t lux_result;

    rt_int64_t coefficient_numerator = 0;

    if ((estimate == RT_NULL) ||(gain == 0U) ||(integration_ms == 0U))
    {
        return -RT_EINVAL;
    }

    sum = (rt_uint64_t)ch0_raw +(rt_uint64_t)ch1_raw;

    /* 两个通道均为0时，比例没有数学意义，项目约定保存为0。 */
    if (sum == 0ULL)
    {
        candidate.ratio_x10000 = 0U;
        candidate.lux_mlux = 0U;
        candidate.lux_valid = RT_TRUE;
        candidate.saturated = RT_FALSE;
        candidate.reason = SENSOR_ALS_REASON_ZERO_COUNTS;
        goto publish;
    }
    /*
     * ratio_x10000： ((uint64)ch1 × 10000 + sum/2(四舍五入，防止截断)) / sum
     * 比例仅用于日志和诊断。公式分段仍使用后面的交叉乘法，
     * 避免舍入后的比例改变边界归属。
     */
    candidate.ratio_x10000 = (rt_uint32_t)
        (((rt_uint64_t)ch1_raw * 10000ULL +sum / 2ULL) /sum);


    /* 饱和时保留比例诊断，但跳过照度公式。 */
    if ((ch0_raw == 0xFFFFU) ||
        (ch1_raw == 0xFFFFU))
    {
        candidate.lux_mlux = 0U;
        candidate.lux_valid = RT_FALSE;
        candidate.saturated = RT_TRUE;
        candidate.reason = SENSOR_ALS_REASON_SATURATED;
        goto publish;
    }
    /*
    分段公式为：
    ratio < 0.45:
        lux = (1.7743×CH0 + 1.1059×CH1) / gain / ALS_INT

    0.45 ≤ ratio < 0.64:
        lux = (4.2785×CH0 - 1.9548×CH1) / gain / ALS_INT

    0.64 ≤ ratio < 0.85:
        lux = (0.5926×CH0 + 0.1185×CH1) / gain / ALS_INT

    ratio ≥ 0.85:
        公式结果为0，但项目标记IR_DOMINANT
    */
    ratio_left = (rt_uint64_t)ch1_raw * 100ULL;

    if (ratio_left < (45ULL * sum))
    {
        /*
         * coefficient_numerator =
         *     17743 × CH0 + 11059 × CH1
         */
        coefficient_numerator =(17743LL * (rt_int64_t)ch0_raw) + (11059LL * (rt_int64_t)ch1_raw);
    }
    else if (ratio_left < (64ULL * sum))
    {
        /*
         * coefficient_numerator =
         *     42785 × CH0 - 19548 × CH1
         *
         * 两个乘法都必须在rt_int64_t中完成。
         */
        coefficient_numerator =(42785LL * (rt_int64_t)ch0_raw) - (19548LL * (rt_int64_t)ch1_raw);
    }
    else if (ratio_left < (85ULL * sum))
    {
        /*
         * coefficient_numerator =
         *     5926 × CH0 + 1185 × CH1
         */
        coefficient_numerator =(5926LL * (rt_int64_t)ch0_raw) + (1185LL * (rt_int64_t)ch1_raw);
    }
    else
    {
        /*
         * Appendix公式在该区间返回0，但红外占优不能解释为
         * 黑暗，因此照度结果标记为不可信。
         */
        candidate.lux_mlux = 0U;
        candidate.lux_valid = RT_FALSE;
        candidate.saturated = RT_FALSE;
        candidate.reason = SENSOR_ALS_REASON_IR_DOMINANT;
        goto publish;
    }

    /* 正确分段下不应出现负分子，出现时作为数学异常保留。 */
    if (coefficient_numerator < 0)
    {
        candidate.lux_mlux = 0U;
        candidate.lux_valid = RT_FALSE;
        candidate.saturated = RT_FALSE;
        candidate.reason = SENSOR_ALS_REASON_MATH_ERROR;
        goto publish;
    }

    denominator =(rt_uint64_t)gain *(rt_uint64_t)integration_ms;

    /*
     * coefficient_numerator已经包含放大10000倍的公式系数。
     * 换算为mlux时乘10，并加入denominator/2实现正数四舍五入。
     */
    rounded_numerator =(rt_uint64_t)coefficient_numerator * 10ULL +denominator / 2ULL;

    lux_result = rounded_numerator / denominator;

    if (lux_result > 0xFFFFFFFFULL)
    {
        candidate.lux_mlux = 0U;
        candidate.lux_valid = RT_FALSE;
        candidate.saturated = RT_FALSE;
        candidate.reason = SENSOR_ALS_REASON_MATH_ERROR;
        goto publish;
    }

    candidate.lux_mlux = (rt_uint32_t)lux_result;
    candidate.lux_valid = RT_TRUE;
    candidate.saturated = RT_FALSE;
    candidate.reason = SENSOR_ALS_REASON_OK;

publish:
    *estimate = candidate;
    return RT_EOK;
}

/*
 * 简介：阻塞等待NEW并读取一份完整双通道样本
 * 输入：sample为调用者持有的输出缓冲区。
 * 返回值：成功形成完整诊断样本返回RT_EOK；
 *         空指针返回-RT_EINVAL；
 *         未初始化返回-RT_ENOSYS；
 *         NEW超时或总线错误返回对应错误。
 * 说明：超时和总线失败时不修改调用者原有sample。
 */
rt_err_t ltr303als01_read_sample(sensor_als_sample_t *sample)
{
    rt_uint8_t status = 0U;
    rt_err_t result;

    if (sample == RT_NULL)
    {
        return -RT_EINVAL;
    }

    if (g_ltr303als01_initialized == RT_FALSE)
    {
        return -RT_ENOSYS;
    }

    result = ltr303als01_wait_new_data(&status);
    if (result != RT_EOK)
    {
        return result;
    }

    return ltr303als01_read_sample_with_status(status, sample);
}


/*
 * 简介：单次检查NEW并读取一份完整双通道样本。
 * 返回值：NEW未置位返回-RT_EBUSY，不延时、不轮询。
 */
rt_err_t ltr303als01_try_read_sample(sensor_als_sample_t *sample)
{
    rt_uint8_t status = 0U;
    rt_err_t result;

    if (sample == RT_NULL)
    {
        return -RT_EINVAL;
    }

    if (g_ltr303als01_initialized == RT_FALSE)
    {
        return -RT_ENOSYS;
    }

    result = sensor_bus_port_read_reg(LTR303ALS01_ADDRESS,
                                LTR303ALS01_ALS_STATUS_REG,
                                &status,
                                1U);
    if (result != RT_EOK)
    {
        return result;
    }

    if ((status & LTR303ALS01_STATUS_NEW_MASK) == 0U)
    {
        return -RT_EBUSY;
    }

    return ltr303als01_read_sample_with_status(status, sample);
}

/*
 * 简介：强制重新验证LTR303身份并恢复冻结配置。
 * 说明：离线重探不能只检查旧配置，必须重新确认ID。
 */
rt_err_t ltr303als01_reinitialize(ltr303als01_config_readback_t *readback)
{
    g_ltr303als01_initialized = RT_FALSE;

    return ltr303als01_init(readback);
}
//---------------------------------------------------------------私有函数---------------------------------------------------------------

/*
 * 简介：读取LTR303ALS01的实际配置。
 * 输入：readback为调用者提供的输出缓冲区。
 * 返回值：成功返回RT_EOK；空指针返回-RT_EINVAL；
 *         总线失败返回底层错误。
 * 说明：先写入局部变量，两个寄存器都读取成功后才更新调用者缓冲区。
 */
static rt_err_t ltr303als01_read_config(ltr303als01_config_readback_t *readback)
{
    ltr303als01_config_readback_t actual = {0};
    rt_err_t result;

    if (readback == RT_NULL)
    {
        return -RT_EINVAL;
    }

    /* 读取工作模式和增益配置。 */
    result = sensor_bus_port_read_reg(
        LTR303ALS01_ADDRESS,
        LTR303ALS01_ALS_CONTR_REG,
        &actual.als_contr,
        1U);
    if (result != RT_EOK)
    {
        return result;
    }

    /* 读取积分时间和测量更新周期配置。 */
    result = sensor_bus_port_read_reg(
        LTR303ALS01_ADDRESS,
        LTR303ALS01_ALS_MEAS_RATE_REG,
        &actual.als_meas_rate,
        1U);
    if (result != RT_EOK)
    {
        return result;
    }

    /*
     * 两项配置均读取成功后才整体提交，防止调用者看到
     * 一项为新值、另一项仍为旧值的部分更新结果。
     */
    *readback = actual;
    return RT_EOK;
}

/*
 * 简介：有限轮询ALS_STATUS.NEW，等待一份新数据。
 * 输入：status_reg用于返回检测到NEW时的完整状态寄存器。
 * 返回值：检测到NEW返回RT_EOK；空指针返回-RT_EINVAL；
 *         总线失败返回底层错误；25次仍无NEW返回-RT_ETIMEOUT。
 * 说明：总共最多读取25次STATUS，只在还要进行下一次读取时延时。
 */
static rt_err_t ltr303als01_wait_new_data(rt_uint8_t *status_reg)
{
    rt_uint32_t attempt;
    rt_uint8_t status;
    rt_err_t result;

    if (status_reg == RT_NULL)
    {
        return -RT_EINVAL;
    }

    for (attempt = 0U;attempt < LTR303ALS01_READY_POLL_COUNT;attempt++)
    {
        status = 0U;

        /* 读取当前数据状态。 */
        result = sensor_bus_port_read_reg(
                LTR303ALS01_ADDRESS,
                LTR303ALS01_ALS_STATUS_REG,
                &status,
                1U);
        if (result != RT_EOK)
        {
            return result;
        }

        /* NEW置位时返回本次完整状态值。 */
        if ((status & LTR303ALS01_STATUS_NEW_MASK) != 0U)
        {
            *status_reg = status;
            return RT_EOK;
        }

        /*
         * 最后一次检查失败后直接退出，避免产生一次无意义的
         * 额外延时。25次读取之间最多存在24次10 ms延时。
         */
        if ((attempt + 1U) < LTR303ALS01_READY_POLL_COUNT)
        {
            rt_thread_mdelay(LTR303ALS01_READY_POLL_INTERVAL_MS);
        }
    }

    return -RT_ETIMEOUT;
}

/*
 * 简介：按LTR303小端顺序拼接16位无符号通道值。
 */
static rt_uint16_t ltr303als01_decode_u16(
    rt_uint8_t low,
    rt_uint8_t high)
{
    rt_uint16_t value;

    /*
     * 先扩展到16位再移位，避免依赖整型提升规则，
     * 最后与低字节合并。
     */
    value =(rt_uint16_t)low |(rt_uint16_t)((rt_uint16_t)high << 8);
    return value;
}
/*
 * 简介：把ALS_STATUS中的增益编码转换为实际倍数。
 *  status_reg: ALS_STATUS寄存器值
 * 返回值：1、2、4、8、48或96；保留编码返回0。
 */
 static rt_uint16_t ltr303als01_decode_status_gain(rt_uint8_t status_reg)
{
    rt_uint8_t gain_code;// ALS_STATUS bits[6:4]的增益编码

    gain_code =(rt_uint8_t)((status_reg & LTR303ALS01_STATUS_GAIN_MASK) >>
                                        LTR303ALS01_STATUS_GAIN_SHIFT);
    switch (gain_code)
    {
    case 0U:
        return 1U;

    case 1U:
        return 2U;

    case 2U:
        return 4U;

    case 3U:
        return 8U;

    case 6U:
        return 48U;

    case 7U:
        return 96U;

    default:
        return 0U;
    }
}
/*
 * 简介：使用已经确认存在NEW的STATUS读取并解释完整双通道样本。
 * 说明：失败时不修改调用者sample。
 */
static rt_err_t ltr303als01_read_sample_with_status(
    rt_uint8_t status,
    sensor_als_sample_t *sample)
{
    sensor_als_sample_t candidate = {0};
    rt_uint8_t data[LTR303ALS01_DATA_LENGTH] = {0};
    rt_err_t result;

    result = sensor_bus_port_read_reg(LTR303ALS01_ADDRESS,
                                LTR303ALS01_DATA_CH1_0_REG,
                                data,
                                LTR303ALS01_DATA_LENGTH);
    if (result != RT_EOK)
    {
        return result;
    }

    candidate.sample_tick = rt_tick_get();

    /* 寄存器顺序固定为CH1_L、CH1_H、CH0_L、CH0_H。 */
    candidate.ch1_raw =
        ltr303als01_decode_u16(data[0], data[1]);

    candidate.ch0_raw =
        ltr303als01_decode_u16(data[2], data[3]);

    candidate.status_reg = status;
    candidate.config_version = LTR303ALS01_CONFIG_VERSION;
    candidate.integration_ms = LTR303ALS01_INTEGRATION_MS;
    candidate.repeat_ms = LTR303ALS01_REPEAT_MS;
    candidate.calibrated = RT_FALSE;

    candidate.gain =
        ltr303als01_decode_status_gain(status);

    if ((status & LTR303ALS01_STATUS_INVALID_MASK) != 0U)
    {
        candidate.valid = RT_FALSE;
        candidate.estimate.lux_valid = RT_FALSE;
        candidate.estimate.saturated = RT_FALSE;
        candidate.estimate.reason =SENSOR_ALS_REASON_DEVICE_INVALID;

        *sample = candidate;
        return RT_EOK;
    }

    if (candidate.gain != LTR303ALS01_GAIN)
    {
        candidate.valid = RT_FALSE;
        candidate.estimate.lux_valid = RT_FALSE;
        candidate.estimate.saturated = RT_FALSE;
        candidate.estimate.reason =SENSOR_ALS_REASON_GAIN_MISMATCH;

        *sample = candidate;
        return RT_EOK;
    }

    result = ltr303als01_estimate_lux(candidate.ch0_raw,
                                        candidate.ch1_raw,
                                        candidate.gain,
                                        candidate.integration_ms,
                                        &candidate.estimate);
    if (result != RT_EOK)
    {
        return result;
    }

    candidate.valid = RT_TRUE;
    *sample = candidate;

    return RT_EOK;
}
