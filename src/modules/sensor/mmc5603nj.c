#include "mmc5603nj.h"
#include "sensor_bus_port.h"


/* 输出、状态和控制寄存器。 */
/*
 * MMC5603NJ寄存器地址。
 *
 * XOUT0～ZOUT2分布在0x00～0x08，读取时从XOUT0开始一次连续读取
 * 9字节，可得到三轴完整20位无符号输出。
 *
 * STATUS1用于查询磁场测量是否完成。
 * CONTROL0用于触发磁场测量以及执行SET/RESET。
 * CONTROL1用于选择测量带宽和执行软件复位。
 */
#define MMC5603NJ_XOUT0_REG                 0x00U
#define MMC5603NJ_STATUS1_REG               0x18U
#define MMC5603NJ_CONTROL0_REG              0x1BU
#define MMC5603NJ_CONTROL1_REG              0x1CU
/* CONTROL0命令位。 */
/*
 * CONTROL0（0x1B）命令位。
 *
 * TAKE_MEAS_M：
 * 写1触发一次磁场测量，该位在测量完成后由器件自动清零。
 *
 * DO_SET / DO_RESET：
 * 分别产生SET和RESET脉冲，用于恢复AMR磁性材料状态，
 * 并可通过配对测量分离外部磁场和桥路偏置。
 * 两项操作完成后相应命令位自动清零。
 *
 * AUTO_SR：
 * 开启自动SET/RESET功能，资料建议正常应用中启用。
 *
 * NORMAL_MEASURE_VALUE=0x21：
 * 同时设置AUTO_SR和TAKE_MEAS_M，作为普通单次测量命令。
 */
#define MMC5603NJ_TAKE_MEAS_M_VALUE         0x01U
#define MMC5603NJ_DO_SET_VALUE              0x08U
#define MMC5603NJ_DO_RESET_VALUE            0x10U
#define MMC5603NJ_AUTO_SR_VALUE             0x20U
#define MMC5603NJ_NORMAL_MEASURE_VALUE      \
    (MMC5603NJ_AUTO_SR_VALUE | MMC5603NJ_TAKE_MEAS_M_VALUE)
/* CONTROL1配置。 */
/*
 * CONTROL1（0x1C）配置值。
 *
 * BW=00：
 * 三轴全部启用时，单次磁场测量时间最长约6.6 ms，
 * 同时具有当前四档带宽中最低的典型噪声。
 *
 * SW_RESET：
 * bit7写1触发软件复位，器件会清除控制寄存器并重新读取OTP。
 * CONTROL1为只写寄存器，不能通过读回确认配置。
 */
#define MMC5603NJ_BW00_VALUE                0x00U
#define MMC5603NJ_SW_RESET_VALUE            0x80U

/* STATUS1.Meas_m_done位。 */
/*
 * STATUS1（0x18）的Meas_m_done标志。
 *
 * bit6=1表示一次磁场测量已经完成，0x00～0x08中的数据可以读取。
 * 读取任意磁场输出寄存器后，该标志由器件清零。
 */
#define MMC5603NJ_MEAS_M_DONE_MASK          0x40U

/* 复位、SET/RESET和ready等待策略。 */
/*
 * 复位、SET/RESET和ready等待策略。
 *
 * 软件复位后资料给出的启动时间为20 ms，项目使用25 ms，
 * 为RTOS tick量化和线程调度留出余量。
 *
 * SET或RESET结束后，距离下一项器件操作至少等待1 ms。
 *
 * ready每1 ms检查一次，最多读取12次STATUS1。
 * 该策略限制的是应用层轮询次数；包含11次轮询间隔，
 * 加上I2C事务时间后，墙钟等待约为11～12 ms。
 * 它不能证明SDK底层总线锁和HAL等待在所有故障下严格有界。
 */
#define MMC5603NJ_RESET_WAIT_MS             25U
#define MMC5603NJ_SET_RESET_WAIT_MS         1U
#define MMC5603NJ_READY_POLL_INTERVAL_MS    1U
/*
 * BW=00并启用自动SET/RESET时，完整测量可能超过单次6.6 ms。
 * 保留约25 ms有界等待，覆盖自动SET/RESET测量和调度余量。
 */
#define MMC5603NJ_READY_POLL_COUNT          25U

/* 20位输出和固定点换算参数。 */
/*
 * 20位输出布局和固定点换算参数。
 *
 * 一次连续读取9字节：
 *   data[0..5]：X/Y/Z各轴的bit[19:4]
 *   data[6..8]：X/Y/Z各轴的bit[3:0]，位于字节高4位
 *
 * MMC5603NJ采用20位无符号偏置编码：
 *   0x80000 = 2^19 = 524288，表示零磁场附近。
 *
 * 中心化原始值：
 *   centered_raw = code_u20 - 524288
 *
 * 20位模式灵敏度：
 *   16384 count/G
 *
 * 单位关系：
 *   1 G = 100000 nT
 *
 * 换算公式：
 *   field_nt = centered_raw × 100000 / 16384
 *
 * 乘法必须使用64位中间值，以防32位溢出。
 */
#define MMC5603NJ_DATA_LENGTH               9U
#define MMC5603NJ_CENTER_OFFSET             524288L
#define MMC5603NJ_COUNTS_PER_G              16384LL
#define MMC5603NJ_NT_PER_G                  100000LL
#define MMC5603NJ_CONFIG_VERSION            1U

static rt_bool_t g_mmc5603nj_initialized = RT_FALSE;

//公开函数声明
rt_err_t mmc5603nj_read_id(rt_uint8_t *product_id);
rt_err_t mmc5603nj_init(void);                                                  //验证身份、软件复位、冻结BW=00并执行一次初始SET
rt_err_t mmc5603nj_read_sample(sensor_magnetometer_sample_t *sample);           //读取MMC5603NJ的三轴磁场采样数据
rt_err_t mmc5603nj_run_set_reset_pair(mmc5603nj_set_reset_result_t *result);    //执行一次SET/RESET配对测量，获取磁场和偏置结果
rt_err_t mmc5603nj_run_ready_timeout_test(rt_uint32_t *timeout_elapsed_ms,
                            sensor_magnetometer_sample_t *recovered_sample);    //执行一次安全的ready超时与恢复测试

//私有函数声明
static rt_uint32_t mmc5603nj_decode_u20(rt_uint8_t high,
                                       rt_uint8_t middle,
                                       rt_uint8_t low_nibble_byte);             //拼接单轴20位无符号输出

static rt_int32_t mmc5603nj_center_code(rt_uint32_t code_u20);                  //把20位无符号偏置编码转换为中心化有符号原始值

static rt_int32_t mmc5603nj_convert_raw_to_nt(rt_int32_t centered_raw);         //把中心化原始值换算成nT，并执行对称四舍五入

static rt_err_t mmc5603nj_wait_data_ready(rt_uint8_t *ready_status);            //有限轮询STATUS1，等待Meas_m_done置位

static rt_err_t mmc5603nj_read_output(rt_uint8_t ready_status,
                                sensor_magnetometer_sample_t *sample);          //一次读取9字节磁场输出并生成完整样本

static rt_err_t mmc5603nj_measure_with_command(rt_uint8_t command,
                            sensor_magnetometer_sample_t *sample);              //写入指定测量命令，有限等待ready并读取完整数据。

static void mmc5603nj_calculate_pair_axis(
    rt_int32_t set_raw,
    rt_int32_t reset_raw,
    rt_int32_t *field_raw,
    rt_int32_t *offset_raw,
    rt_int32_t *field_nt);

//---------------------------------------------------------------公开函数---------------------------------------------------------------
/*
 * 简介：验证身份、软件复位、冻结BW=00并执行一次初始SET。
 * 返回值：完整初始化成功返回RT_EOK；身份不符返回-RT_EIO；
 *         总线失败返回底层错误。
 * 说明：只能在线程上下文调用；重复初始化不会重复复位器件。
 */
rt_err_t mmc5603nj_init(void)
{
    rt_uint8_t product_id = 0U;
    rt_uint8_t value;
    rt_err_t result;

    if (g_mmc5603nj_initialized == RT_TRUE)
    {
        return RT_EOK;
    }

    g_mmc5603nj_initialized = RT_FALSE;

    result = mmc5603nj_read_id(&product_id);
    if (result != RT_EOK)
    {
        return result;
    }

    if (product_id != MMC5603NJ_PRODUCT_ID_VALUE)
    {
        return -RT_EIO;
    }

    /* 软件复位会清除寄存器并重新加载OTP。 */
    value = MMC5603NJ_SW_RESET_VALUE;
    result = sensor_bus_port_write_reg(MMC5603NJ_ADDRESS,
                                       MMC5603NJ_CONTROL1_REG,
                                       &value,
                                       1U);
    if (result != RT_EOK)
    {
        return result;
    }

    /*
     * 资料给出软件复位后的power-on time为20 ms。
     * 项目使用25 ms留出线程调度和tick量化余量。
     */
    rt_thread_mdelay(MMC5603NJ_RESET_WAIT_MS);

    /* 复位后再次确认器件可以正常应答且身份未改变。 */
    result = mmc5603nj_read_id(&product_id);
    if (result != RT_EOK)
    {
        return result;
    }

    if (product_id != MMC5603NJ_PRODUCT_ID_VALUE)
    {
        return -RT_EIO;
    }

    /*
     * CONTROL1是只写寄存器，无法读回验证。
     * 显式写0x00表示BW=00且X/Y/Z均启用。
     */
    value = MMC5603NJ_BW00_VALUE;
    result = sensor_bus_port_write_reg(MMC5603NJ_ADDRESS,
                                       MMC5603NJ_CONTROL1_REG,
                                       &value,
                                       1U);
    if (result != RT_EOK)
    {
        return result;
    }

    /* 初始化末尾执行一次SET，使AMR敏感元件进入已知磁化状态。 */
    value = MMC5603NJ_DO_SET_VALUE;
    result = sensor_bus_port_write_reg(MMC5603NJ_ADDRESS,
                                       MMC5603NJ_CONTROL0_REG,
                                       &value,
                                       1U);
    if (result != RT_EOK)
    {
        return result;
    }

    /*
     * SET到下一项操作之间至少等待1 ms。
     * 该延时只能在线程上下文执行。
     */
    rt_thread_mdelay(MMC5603NJ_SET_RESET_WAIT_MS);

    g_mmc5603nj_initialized = RT_TRUE;

    return RT_EOK;
}


/*简介：读取MMC5603NJ的PRODUCT_ID寄存器值
  输入：product_id: 指向存储PRODUCT_ID寄存器值的缓冲区指针
  返回值：rt_err_t:错误类型
*/
rt_err_t mmc5603nj_read_id(rt_uint8_t *product_id)
{
    if (product_id == RT_NULL)
    {
        return -RT_EINVAL;
    }

    *product_id = 0U;

    return sensor_bus_port_read_reg(MMC5603NJ_ADDRESS,
                                    MMC5603NJ_PRODUCT_ID_REG,
                                    product_id,
                                    1U);
}

/*
 * 简介：执行一次带自动SET/RESET的单次磁场测量。
 * 输入：sample为调用者拥有的输出缓冲区。
 * 返回值：完整测量成功返回RT_EOK；ready超时返回-RT_ETIMEOUT；
 *         参数、初始化或总线错误返回相应错误。
 * 说明：失败时不修改调用者原有sample。
 */
rt_err_t mmc5603nj_read_sample(
    sensor_magnetometer_sample_t *sample)
{
    if (sample == RT_NULL)
    {
        return -RT_EINVAL;
    }

    if (g_mmc5603nj_initialized == RT_FALSE)
    {
        return -RT_ENOSYS;
    }

    return mmc5603nj_measure_with_command(
        MMC5603NJ_NORMAL_MEASURE_VALUE,
        sample);
}

/*
 * 简介：执行显式SET测量和RESET测量，并计算磁场与桥路偏置。
 * 输入：result为调用者拥有的输出缓冲区。
 * 返回值：完整配对成功返回RT_EOK；任一步失败返回对应错误。
 * 说明：本接口用于受控诊断，不作为普通周期采样路径；
 *       失败时不修改调用者原有result。
 */
rt_err_t mmc5603nj_run_set_reset_pair(
    mmc5603nj_set_reset_result_t *result)
{
    mmc5603nj_set_reset_result_t candidate = {0};
    rt_uint8_t value;
    rt_err_t status;

    if (result == RT_NULL)
    {
        return -RT_EINVAL;
    }

    if (g_mmc5603nj_initialized == RT_FALSE)
    {
        return -RT_ENOSYS;
    }

    /* SET后等待至少1 ms，再执行不带Auto_SR的测量。 */
    value = MMC5603NJ_DO_SET_VALUE;
    status = sensor_bus_port_write_reg(MMC5603NJ_ADDRESS,
                                       MMC5603NJ_CONTROL0_REG,
                                       &value,
                                       1U);
    if (status != RT_EOK)
    {
        return status;
    }

    rt_thread_mdelay(MMC5603NJ_SET_RESET_WAIT_MS);
    //触发一次磁场测量，并读取完整样本
    status = mmc5603nj_measure_with_command(
        MMC5603NJ_TAKE_MEAS_M_VALUE,
        &candidate.set_sample);
    if (status != RT_EOK)
    {
        return status;
    }

    /* RESET后等待至少1 ms，再执行第二次测量。 */
    value = MMC5603NJ_DO_RESET_VALUE;
    status = sensor_bus_port_write_reg(MMC5603NJ_ADDRESS,
                                       MMC5603NJ_CONTROL0_REG,
                                       &value,
                                       1U);
    if (status != RT_EOK)
    {
        return status;
    }

    rt_thread_mdelay(MMC5603NJ_SET_RESET_WAIT_MS);
    //触发一次磁场测量，并读取完整样本
    status = mmc5603nj_measure_with_command(
        MMC5603NJ_TAKE_MEAS_M_VALUE,
        &candidate.reset_sample);
    if (status != RT_EOK)
    {
        return status;
    }

    mmc5603nj_calculate_pair_axis(
        candidate.set_sample.centered_raw.x,
        candidate.reset_sample.centered_raw.x,
        &candidate.field_centered_raw.x,
        &candidate.offset_centered_raw.x,
        &candidate.field_nt.x);

    mmc5603nj_calculate_pair_axis(
        candidate.set_sample.centered_raw.y,
        candidate.reset_sample.centered_raw.y,
        &candidate.field_centered_raw.y,
        &candidate.offset_centered_raw.y,
        &candidate.field_nt.y);

    mmc5603nj_calculate_pair_axis(
        candidate.set_sample.centered_raw.z,
        candidate.reset_sample.centered_raw.z,
        &candidate.field_centered_raw.z,
        &candidate.offset_centered_raw.z,
        &candidate.field_nt.z);

    /*
     * 配对结束后再执行一次SET，使器件回到确定状态。
     * 后续普通测量仍会通过0x21重新启用Auto_SR。
     */
    value = MMC5603NJ_DO_SET_VALUE;
    status = sensor_bus_port_write_reg(MMC5603NJ_ADDRESS,
                                       MMC5603NJ_CONTROL0_REG,
                                       &value,
                                       1U);
    if (status != RT_EOK)
    {
        return status;
    }

    rt_thread_mdelay(MMC5603NJ_SET_RESET_WAIT_MS);

    *result = candidate;

    return RT_EOK;
}

/*
 * 简介：安全验证ready有限超时，并确认超时后可以正常恢复。
 * 输入：timeout_elapsed_ms保存项目侧ready等待耗时；
 *       recovered_sample保存超时后的第一份正常样本。
 * 返回值：观察到预期超时并恢复成功时返回RT_EOK。
 * 说明：先读取一份正常数据清除ready，再故意不触发新测量；
 *       不使用错误地址、不拉低总线，也不修改未知寄存器。
 */
rt_err_t mmc5603nj_run_ready_timeout_test(
    rt_uint32_t *timeout_elapsed_ms,
    sensor_magnetometer_sample_t *recovered_sample)
{
    sensor_magnetometer_sample_t consumed_sample = {0};
    sensor_magnetometer_sample_t recovered_candidate = {0};
    rt_tick_t start_ms;
    rt_tick_t elapsed_ms;
    rt_uint8_t unexpected_status = 0U;
    rt_err_t result;

    if ((timeout_elapsed_ms == RT_NULL) ||
        (recovered_sample == RT_NULL))
    {
        return -RT_EINVAL;
    }

    if (g_mmc5603nj_initialized == RT_FALSE)
    {
        return -RT_ENOSYS;
    }

    /*
     * 先完成一份正常单次测量。读取任一磁场数据寄存器后，
     * Meas_m_done应被器件清除。
     */
    result = mmc5603nj_read_sample(&consumed_sample);
    if (result != RT_EOK)
    {
        return result;
    }

    /*
     * 不发送新的Take_meas_M，直接有限等待ready。
     * 单次模式下预期返回-RT_ETIMEOUT。
     */
    start_ms = rt_tick_get_millisecond();
    result = mmc5603nj_wait_data_ready(&unexpected_status);
    elapsed_ms =
        (rt_tick_t)(rt_tick_get_millisecond() - start_ms);

    if (result == RT_EOK)
    {
        return -RT_EIO;
    }

    if (result != -RT_ETIMEOUT)
    {
        return result;
    }

    /* 超时后重新走一次正常测量，证明状态机和总线仍可使用。 */
    result = mmc5603nj_read_sample(&recovered_candidate);
    if (result != RT_EOK)
    {
        return result;
    }

    *timeout_elapsed_ms = (rt_uint32_t)elapsed_ms;
    *recovered_sample = recovered_candidate;

    return RT_EOK;
}



//---------------------------------------------------------------私有函数---------------------------------------------------------------
/*
 * 简介：拼接单轴20位无符号输出。
 * 输入：high为bit[19:12]，middle为bit[11:4]，
 * low_nibble_byte为bit[3:0]位于字节高4位。
 * 返回值：code_u20为20位无符号偏置编码，合法范围0～0xFFFFF。
 * 说明：out2的高4位保存数据低4位，低4位为0。
 */
static rt_uint32_t mmc5603nj_decode_u20(
    rt_uint8_t high,
    rt_uint8_t middle,
    rt_uint8_t low_nibble_byte)
{
    return ((rt_uint32_t)high << 12) |
           ((rt_uint32_t)middle << 4) |
           ((rt_uint32_t)low_nibble_byte >> 4);
}

/*
 * 简介：把20位无符号偏置编码转换为中心化有符号原始值。
 * 输入：code_u20为20位无符号偏置编码，合法范围0～0xFFFFF。
 * 返回值：centered_raw为中心化有符号原始值，合法范围-524288～524287。
 * 说明：中心化原始值的零点位于0x80000，
 */
static rt_int32_t mmc5603nj_center_code(rt_uint32_t code_u20)
{
    return (rt_int32_t)code_u20 -
           (rt_int32_t)MMC5603NJ_CENTER_OFFSET;
}

/*
 * 简介：把中心化原始值换算成nT，并执行对称四舍五入
 * 输入：centered_raw为中心化有符号原始值，合法范围-524288～524287。
 * 说明：乘法必须使用64位中间值，避免32位溢出。
 */
static rt_int32_t mmc5603nj_convert_raw_to_nt(
    rt_int32_t centered_raw)
{
    rt_int64_t scaled;

    scaled = (rt_int64_t)centered_raw *
             (rt_int64_t)MMC5603NJ_NT_PER_G;
    // 对称四舍五入，避免32位溢出
    if (scaled >= 0)
    {
        scaled += MMC5603NJ_COUNTS_PER_G / 2LL;
    }
    else
    {
        scaled -= MMC5603NJ_COUNTS_PER_G / 2LL;
    }

    return (rt_int32_t)(scaled / MMC5603NJ_COUNTS_PER_G);
}
/*
 * 简介：有限轮询STATUS1，等待Meas_m_done置位。
 * 输入：ready_status保存成功时的STATUS1原始值。
 * 返回值：ready返回RT_EOK；约25 ms仍未ready返回-RT_ETIMEOUT；
 *         总线错误原样返回。
 * 说明：只读取状态，不读取磁场输出寄存器。
 */
static rt_err_t mmc5603nj_wait_data_ready(
    rt_uint8_t *ready_status)
{
    rt_uint32_t attempt;
    rt_uint8_t status;
    rt_err_t result;

    if (ready_status == RT_NULL)
    {
        return -RT_EINVAL;
    }

    for (attempt = 0U;attempt < MMC5603NJ_READY_POLL_COUNT;attempt++)
    {
        status = 0U;

        result = sensor_bus_port_read_reg(
            MMC5603NJ_ADDRESS,
            MMC5603NJ_STATUS1_REG,
            &status,
            1U);
        if (result != RT_EOK)
        {
            return result;
        }

        if ((status & MMC5603NJ_MEAS_M_DONE_MASK) != 0U)
        {
            *ready_status = status;
            return RT_EOK;
        }

        if ((attempt + 1U) < MMC5603NJ_READY_POLL_COUNT)
        {
            rt_thread_mdelay(
                MMC5603NJ_READY_POLL_INTERVAL_MS);
        }
    }

    return -RT_ETIMEOUT;
}

/*
 * 简介：一次读取9字节磁场输出并生成完整样本。
 * 输入：ready_status为读取数据前确认的STATUS1值；
 *       sample为调用者输出缓冲区。
 * 说明：读取磁场数据会清除Meas_m_done；失败时不修改sample。
 */
static rt_err_t mmc5603nj_read_output(
    rt_uint8_t ready_status,
    sensor_magnetometer_sample_t *sample)
{
    sensor_magnetometer_sample_t candidate = {0};
    rt_uint8_t data[MMC5603NJ_DATA_LENGTH] = {0};
    rt_err_t result;

    result = sensor_bus_port_read_reg(
        MMC5603NJ_ADDRESS,
        MMC5603NJ_XOUT0_REG,
        data,
        MMC5603NJ_DATA_LENGTH);
    if (result != RT_EOK)
    {
        return result;
    }

    candidate.sample_tick = rt_tick_get();

    candidate.code_u20.x =
        mmc5603nj_decode_u20(data[0], data[1], data[6]);
    candidate.code_u20.y =
        mmc5603nj_decode_u20(data[2], data[3], data[7]);
    candidate.code_u20.z =
        mmc5603nj_decode_u20(data[4], data[5], data[8]);

    candidate.centered_raw.x =
        mmc5603nj_center_code(candidate.code_u20.x);
    candidate.centered_raw.y =
        mmc5603nj_center_code(candidate.code_u20.y);
    candidate.centered_raw.z =
        mmc5603nj_center_code(candidate.code_u20.z);

    candidate.field_nt.x =
        mmc5603nj_convert_raw_to_nt(candidate.centered_raw.x);
    candidate.field_nt.y =
        mmc5603nj_convert_raw_to_nt(candidate.centered_raw.y);
    candidate.field_nt.z =
        mmc5603nj_convert_raw_to_nt(candidate.centered_raw.z);

    candidate.config_version = MMC5603NJ_CONFIG_VERSION;
    candidate.status_reg = ready_status;
    candidate.valid = RT_TRUE;

    *sample = candidate;

    return RT_EOK;
}

/*
 * 简介：写入指定测量命令，有限等待ready并读取完整数据。
 * 说明：command由本驱动内部提供，调用者不能传入任意寄存器值。
 */
static rt_err_t mmc5603nj_measure_with_command(
    rt_uint8_t command,
    sensor_magnetometer_sample_t *sample)
{
    rt_uint8_t status = 0U;
    rt_err_t result;

    result = sensor_bus_port_write_reg(
        MMC5603NJ_ADDRESS,
        MMC5603NJ_CONTROL0_REG,
        &command,
        1U);
    if (result != RT_EOK)
    {
        return result;
    }

    result = mmc5603nj_wait_data_ready(&status);
    if (result != RT_EOK)
    {
        return result;
    }

    return mmc5603nj_read_output(status, sample);
}

/*
 * 简介：计算一个轴的SET/RESET配对磁场和桥路偏置。
    * 输入：set_raw为SET测量的中心化原始值；
    *       reset_raw为RESET测量的中心化原始值；
    *       field_raw为输出的中心化原始值；
    *       offset_raw为输出的桥路偏置原始值；
    *       field_nt为输出的磁场换算值，单位nT。
    * 说明：SET/RESET配对测量可以分离外部磁场和桥路偏置。
 */
static void mmc5603nj_calculate_pair_axis(
    rt_int32_t set_raw,
    rt_int32_t reset_raw,
    rt_int32_t *field_raw,
    rt_int32_t *offset_raw,
    rt_int32_t *field_nt)
{
    rt_int64_t difference;
    rt_int64_t sum;

    difference = (rt_int64_t)set_raw -
                 (rt_int64_t)reset_raw;
    sum = (rt_int64_t)set_raw +
          (rt_int64_t)reset_raw;

    *field_raw = (rt_int32_t)(difference / 2LL);
    *offset_raw = (rt_int32_t)(sum / 2LL);
    *field_nt = mmc5603nj_convert_raw_to_nt(*field_raw);
}
