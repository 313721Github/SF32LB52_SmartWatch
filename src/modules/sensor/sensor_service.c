#include "rtthread.h"
#include "sensor_bus_port.h"
#include "lsm6ds3tr_c.h"
#include "mmc5603nj.h"
#include "ltr303als01.h"

#ifdef RT_USING_FINSH
#include "finsh.h"
#include <string.h>
#include <errno.h>
#include <stdlib.h>

//私有函数
static rt_err_t task4_probe_lsm(rt_uint8_t *active_address);            //探测LSM6DS3TR-C传感器并返回其I2C地址
static rt_err_t task4_probe_mmc5603nj(void);                            //探测MMC5603NJ传感器
static rt_err_t task4_probe_ltr303(void);                               //探测LTR303ALS01传感器
static rt_err_t task4_probe_all(rt_uint8_t *lsm_address);               //探测所有传感器并返回LSM6DS3TR-C的I2C地址
static rt_err_t task4_failure_recovery_test(rt_uint8_t active_address); //探测LSM6DS3TR-C传感器并进行故障恢复测试
static rt_err_t sensor_test_parse_uint_arg(const char *text,
                                     rt_uint32_t minimum,
                                     rt_uint32_t maximum,
                                     rt_uint32_t *value);               //解析FINSH命令行参数为无符号整数
//shell测试
//传感器id测试
static int sensor_id_test(int argc, char **argv);                       //通过FINSH命令行探测所有传感器并返回LSM6DS3TR-C的I2C地址
//LSM6DS3TR-C采样测试
static int imu_sample_test(int argc, char **argv);                      //通过FINSH命令行读取LSM6DS3TR-C的陀螺仪和加速度计采样数据
//MMC5603NJ采样测试
static int mmc_sample_test(int argc, char **argv);                      //通过FINSH命令行读取MMC5603NJ的磁场采样数据
static int mmc_set_reset_test(int argc, char **argv);                   //通过FINSH命令行执行MMC5603NJ的SET/RESET配对测量并计算磁场与桥路偏置
static int mmc_ready_timeout_test(int argc, char **argv);               //通过FINSH命令行执行MMC5603NJ的ready超时测试并验证恢复能力
//---------------------------------------------------------------私有函数---------------------------------------------------------------
/*简介：读取实板LSM6DS3TR-C身份并输出已确认的I2C地址
  输入：active_address: 输出已确认的LSM6DS3TR-C有效7位I2C地址
  返回值：rt_err_t:错误类型
*/
static rt_err_t task4_probe_lsm(rt_uint8_t *active_address)
{
    rt_uint8_t id;
    rt_err_t result;
    rt_tick_t start_ms;
    rt_tick_t elapsed_ms;

    if (active_address == RT_NULL)
    {
        return -RT_EINVAL;
    }

    *active_address = 0U;

    start_ms = rt_tick_get_millisecond();
    result = lsm6ds3tr_c_read_id(LSM6DS3TR_C_ADDRESS, &id);
    elapsed_ms = rt_tick_get_millisecond() - start_ms;

    if (result != RT_EOK)
    {
        rt_kprintf("[SENSOR][ID][ERR] LSM6DS3TR-C addr=0x%02X "
                   "read failed result=%d elapsed_ms=%lu\n",
                   LSM6DS3TR_C_ADDRESS,
                   (int)result,
                   (unsigned long)elapsed_ms);
        return result;
    }

    if (id != LSM6DS3TR_C_WHO_AM_I_VALUE)
    {
        rt_kprintf("[SENSOR][ID][ERR] LSM6DS3TR-C addr=0x%02X "
                   "reg=0x%02X value=0x%02X expected=0x%02X "
                   "elapsed_ms=%lu\n",
                   LSM6DS3TR_C_ADDRESS,
                   LSM6DS3TR_C_WHO_AM_I_REG,
                   id,
                   LSM6DS3TR_C_WHO_AM_I_VALUE,
                   (unsigned long)elapsed_ms);
        return -RT_EIO;
    }

    *active_address = LSM6DS3TR_C_ADDRESS;

    rt_kprintf("[SENSOR][ID][OK] LSM6DS3TR-C addr=0x%02X "
               "reg=0x%02X value=0x%02X elapsed_ms=%lu\n",
               *active_address,
               LSM6DS3TR_C_WHO_AM_I_REG,
               id,
               (unsigned long)elapsed_ms);

    return RT_EOK;
}

/*简介：探测MMC5603NJ传感器
  输入：无
  返回值：rt_err_t:错误类型
*/
static rt_err_t task4_probe_mmc5603nj(void)
{
    rt_uint8_t id;
    rt_err_t result;
    rt_tick_t start_ms;
    rt_tick_t elapsed_ms;

    start_ms = rt_tick_get_millisecond();
    result = mmc5603nj_read_id(&id);
    elapsed_ms = rt_tick_get_millisecond() - start_ms;

    if ((result != RT_EOK) || (id != MMC5603NJ_PRODUCT_ID_VALUE))
    {
        rt_kprintf("[SENSOR][ID][ERR] MMC5603NJ addr=0x%02X "
                   "reg=0x%02X value=0x%02X expected=0x%02X "
                   "result=%d elapsed_ms=%lu\n",
                   MMC5603NJ_ADDRESS,
                   MMC5603NJ_PRODUCT_ID_REG,
                   id,
                   MMC5603NJ_PRODUCT_ID_VALUE,
                   (int)result,
                   (unsigned long)elapsed_ms);
        return (result == RT_EOK) ? -RT_EIO : result;
    }

    rt_kprintf("[SENSOR][ID][OK] MMC5603NJ addr=0x%02X "
               "reg=0x%02X value=0x%02X elapsed_ms=%lu\n",
               MMC5603NJ_ADDRESS,
               MMC5603NJ_PRODUCT_ID_REG,
               id,
               (unsigned long)elapsed_ms);

    return RT_EOK;
}

/*简介：探测LTR303ALS01传感器
  输入：无
  返回值：rt_err_t:错误类型
*/
static rt_err_t task4_probe_ltr303(void)
{
    rt_uint8_t part_id;
    rt_uint8_t manufacturer_id;
    rt_err_t result;
    rt_tick_t start_ms;
    rt_tick_t elapsed_ms;

    start_ms = rt_tick_get_millisecond();
    result = ltr303als01_read_ids(&part_id, &manufacturer_id);
    elapsed_ms = rt_tick_get_millisecond() - start_ms;

    if ((result != RT_EOK) ||
        (part_id != LTR303ALS01_PART_ID_VALUE) ||
        (manufacturer_id != LTR303ALS01_MANUFAC_ID_VALUE))
    {
        rt_kprintf("[SENSOR][ID][ERR] LTR303 addr=0x%02X "
                   "part=0x%02X expected_part=0x%02X "
                   "manufacturer=0x%02X expected_manufacturer=0x%02X "
                   "result=%d elapsed_ms=%lu\n",
                   LTR303ALS01_ADDRESS,
                   part_id,
                   LTR303ALS01_PART_ID_VALUE,
                   manufacturer_id,
                   LTR303ALS01_MANUFAC_ID_VALUE,
                   (int)result,
                   (unsigned long)elapsed_ms);
        return (result == RT_EOK) ? -RT_EIO : result;
    }

    rt_kprintf("[SENSOR][ID][OK] LTR303 addr=0x%02X "
               "part_reg=0x%02X part=0x%02X "
               "manufacturer_reg=0x%02X manufacturer=0x%02X "
               "elapsed_ms=%lu\n",
               LTR303ALS01_ADDRESS,
               LTR303ALS01_PART_ID_REG,
               part_id,
               LTR303ALS01_MANUFAC_ID_REG,
               manufacturer_id,
               (unsigned long)elapsed_ms);

    return RT_EOK;
}

/*简介：探测所有传感器并返回LSM6DS3TR-C的I2C地址
  输入：active_address: 指向存储LSM6DS3TR-C有效I2C地址的缓冲区指针
  返回值：rt_err_t:错误类型
*/
static rt_err_t task4_probe_all(rt_uint8_t *lsm_address)
{
    rt_uint8_t passed = 0U;
    rt_err_t result;

    result = sensor_bus_port_init();
    if (result != RT_EOK)
    {
        rt_kprintf("[SENSOR][ID][SUMMARY] bus initialization failed "
                   "result=%d\n",
                   (int)result);
        return result;
    }

    if (task4_probe_lsm(lsm_address) == RT_EOK)
    {
        passed++;
    }

    if (task4_probe_mmc5603nj() == RT_EOK)
    {
        passed++;
    }

    if (task4_probe_ltr303() == RT_EOK)
    {
        passed++;
    }

    rt_kprintf("[SENSOR][ID][SUMMARY] passed=%u total=3 result=%s\n",
               passed,
               (passed == 3U) ? "PASS" : "FAIL");

    return (passed == 3U) ? RT_EOK : -RT_EIO;
}

/*简介：探测LSM6DS3TR-C传感器并进行故障恢复测试
  输入：active_address: 已确认的LSM6DS3TR-C有效7位I2C地址
  返回值：rt_err_t:错误类型
*/
static rt_err_t task4_failure_recovery_test(rt_uint8_t active_address)
{
    rt_uint8_t inactive_address;    //LSM6DS3TR-C的另一个可能的I2C地址
    rt_uint8_t id;
    rt_err_t result;
    rt_tick_t start_ms;
    rt_tick_t elapsed_ms;

    inactive_address =
        (active_address == LSM6DS3TR_C_ADDRESS_SA0_HIGH)
        ? LSM6DS3TR_C_ADDRESS_SA0_LOW
        : LSM6DS3TR_C_ADDRESS_SA0_HIGH;

    /*
     * 只读取LSM6DS3TR-C另一个合法SA0候选地址。
     * 不扫描未知地址，不写寄存器，也不制造电气短路。
     */
    start_ms = rt_tick_get_millisecond();
    result = lsm6ds3tr_c_read_id(inactive_address, &id);
    elapsed_ms = rt_tick_get_millisecond() - start_ms;

    if (result == RT_EOK)
    {
        rt_kprintf("[SENSOR][RECOVERY][ERR] expected NACK at addr=0x%02X "
                   "but read value=0x%02X elapsed_ms=%lu\n",
                   inactive_address,
                   id,
                   (unsigned long)elapsed_ms);
        return -RT_EIO;
    }

    rt_kprintf("[SENSOR][RECOVERY][INFO] expected failure addr=0x%02X "
               "result=%d elapsed_ms=%lu\n",
               inactive_address,
               (int)result,
               (unsigned long)elapsed_ms);

    start_ms = rt_tick_get_millisecond();
    result = lsm6ds3tr_c_read_id(active_address, &id);
    elapsed_ms = rt_tick_get_millisecond() - start_ms;

    if ((result != RT_EOK) || (id != LSM6DS3TR_C_WHO_AM_I_VALUE))
    {
        rt_kprintf("[SENSOR][RECOVERY][ERR] real address did not recover "
                   "addr=0x%02X value=0x%02X result=%d elapsed_ms=%lu\n",
                   active_address,
                   id,
                   (int)result,
                   (unsigned long)elapsed_ms);
        return (result == RT_EOK) ? -RT_EIO : result;
    }

    rt_kprintf("[SENSOR][RECOVERY][OK] real address recovered "
               "addr=0x%02X value=0x%02X elapsed_ms=%lu\n",
               active_address,
               id,
               (unsigned long)elapsed_ms);

    return RT_EOK;
}

//-----------------------------------------------------shell测试-----------------------------------------------------

/*简介：通过FINSH命令行探测所有传感器并返回LSM6DS3TR-C的I2C地址
  输入：argc: 参数个数
        argv: 参数数组
  返回值：rt_err_t:错误类型
*/
static int sensor_id_test(int argc, char **argv)
{
    rt_uint8_t lsm_address;
    rt_err_t result;
    rt_bool_t run_recovery = RT_FALSE;

    if (argc > 2)
    {
        rt_kprintf("usage: sensor_id_test [recover]\n");
        return -RT_EINVAL;
    }

    if (argc == 2)
    {
        if (strcmp(argv[1], "recover") != 0)
        {
            rt_kprintf("usage: sensor_id_test [recover]\n");
            return -RT_EINVAL;
        }

        run_recovery = RT_TRUE;
    }

    result = task4_probe_all(&lsm_address);
    if (result != RT_EOK)
    {
        return (int)result;
    }

    if (run_recovery == RT_TRUE)
    {
        result = task4_failure_recovery_test(lsm_address);
    }

    return (int)result;
}

/*
 * 简介：把Shell参数解析为指定范围内的无符号整数。
 * 输入：text为以'\0'结束的参数字符串，不允许为RT_NULL；
 *       minimum和maximum为闭区间边界；
 *       value为调用者拥有的输出缓冲区，不允许为RT_NULL。
 * 返回值：解析成功返回RT_EOK；参数为空、含非法字符或越界返回-RT_EINVAL。
 * 说明：只解析参数，不访问传感器、共享状态或UI对象。
 */
static rt_err_t sensor_test_parse_uint_arg(const char *text,
                                     rt_uint32_t minimum,
                                     rt_uint32_t maximum,
                                     rt_uint32_t *value)
{
    char *end;
    unsigned long parsed;

    if ((text == RT_NULL) || (value == RT_NULL)|| (minimum > maximum))
    {
        return -RT_EINVAL;
    }

    end = RT_NULL;
    errno = 0;
    /*
     * strtoul允许空白和正负号，这里要求Shell参数必须直接以数字开头。
     */
    if ((*text < '0') || (*text > '9'))
    {
        return -RT_EINVAL;
    }
    errno = 0;      //清除errno，确保后续检查的准确性
    parsed = strtoul(text, &end, 10);//以10进制解析字符串为无符号长整数

    if ((errno == ERANGE) ||
        (end == RT_NULL) ||
        (end == text) ||
        (*end != '\0') ||
        (parsed < (unsigned long)minimum) ||
        (parsed > (unsigned long)maximum))
    {
        return -RT_EINVAL;
    }

    /* 所有检查通过后才更新调用者的输出。 */
    *value = (rt_uint32_t)parsed;

    return RT_EOK;
}

/*
 * 简介：初始化IMU并按指定次数和周期采集、输出六轴样本和间隔统计。
 * 输入：argc和argv来自FinSH线程；
 *       可用格式为imu_sample_test [count] [period_ms]。
 * 返回值：测试完成返回RT_EOK；参数、初始化或采样错误返回相应错误。
 * 说明：运行在FinSH线程；执行期间由本命令独占任务5的传感器I2C访问，
 *       不创建后台线程，不修改UI对象。
 */
static int imu_sample_test(int argc, char **argv)
{
    lsm6ds3tr_c_config_readback_t config = {0};
    sensor_imu_sample_t sample = {0};

    rt_uint32_t count = 10U;
    rt_uint32_t period_ms = 20U;
    rt_uint32_t index;
    rt_tick_t iteration_start_tick;
    rt_tick_t current_tick;
    rt_tick_t elapsed_tick;
    rt_tick_t period_tick;

    rt_uint32_t attempt_count = 0U;
    rt_uint32_t success_count = 0U;
    rt_uint32_t not_ready_count = 0U;
    rt_uint32_t failure_count = 0U;

    rt_bool_t have_previous_tick = RT_FALSE;        //标记是否有上一个样本的时间戳
    rt_bool_t have_delta = RT_FALSE;                //标记是否有有效的时间间隔数据

    rt_tick_t previous_tick = 0U;
    rt_tick_t delta;
    rt_tick_t minimum_delta = 0U;
    rt_tick_t maximum_delta = 0U;

    rt_uint64_t delta_sum = 0U;
    rt_uint32_t delta_count = 0U;

    rt_err_t result;

    if (argc > 3)
    {
        rt_kprintf("usage: imu_sample_test [count] [period_ms]\n");
        return -RT_EINVAL;
    }

    /* 解析采样次数。 */
    if (argc >= 2)
    {
        result = sensor_test_parse_uint_arg(argv[1],
                                     1U,
                                     3000U,
                                     &count);
        if (result != RT_EOK)
        {
            rt_kprintf("[IMU][ARG_ERROR] count=%s range=1..3000\n",
                       argv[1]);
            return result;
        }
    }

   /* 解析采样周期。 */
    if (argc >= 3)
    {
        result = sensor_test_parse_uint_arg(argv[2],
                                     20U,
                                     1000U,
                                     &period_ms);
        if (result != RT_EOK)
        {
            rt_kprintf("[IMU][ARG_ERROR] period_ms=%s range=20..1000\n",
                       argv[2]);
            return result;
        }
    }

    /* 初始化项目侧传感器总线。 */
    result = sensor_bus_port_init();
    if (result != RT_EOK)
    {
        rt_kprintf("[IMU][BUS_INIT_ERROR] result=%d\n", (int)result);
        return result;
    }

    /* 复位、配置并验证IMU。 */
    result = lsm6ds3tr_c_init(&config);
    if (result != RT_EOK)
    {
        rt_kprintf("[IMU][INIT_ERROR] result=%d\n", (int)result);
        return result;
    }

    //打印实际配置读回值
    rt_kprintf("[IMU][CONFIG][OK] "
               "ctrl1_xl=0x%02X ctrl2_g=0x%02X ctrl3_c=0x%02X "
               "count=%lu period_ms=%lu\n",
               (unsigned int)config.ctrl1_xl,
               (unsigned int)config.ctrl2_g,
               (unsigned int)config.ctrl3_c,
               (unsigned long)count,
               (unsigned long)period_ms);

    period_tick = rt_tick_from_millisecond((rt_int32_t)period_ms);  //将采样周期从毫秒转换为系统节拍数
    for (index = 0U; index < count; index++)
    {
        iteration_start_tick = rt_tick_get();   //记录每次采样迭代的起始节拍数
        attempt_count++;

        result = lsm6ds3tr_c_read_sample(&sample);

        if (result == RT_EOK)
        {
            success_count++;

            if (have_previous_tick == RT_TRUE)
            {
                /*
                 * rt_tick_t为无符号类型时，该减法可以处理一次正常回绕。
                 */
                delta =(rt_tick_t)(sample.sample_tick - previous_tick);

                if (have_delta == RT_FALSE)
                {
                    minimum_delta = delta;
                    maximum_delta = delta;
                    have_delta = RT_TRUE;
                }
                else
                {
                   if (delta < minimum_delta)
                    {
                        minimum_delta = delta;
                    }

                    if (delta > maximum_delta)
                    {
                        maximum_delta = delta;
                    }
                }

                delta_sum += (rt_uint64_t)delta;
                delta_count++;
            }

            previous_tick = sample.sample_tick;
            have_previous_tick = RT_TRUE;


        rt_kprintf("[IMU][RAW] "
                "i=%lu t=%lu st=0x%02X cfg=%lu "
                "g=%d,%d,%d a=%d,%d,%d\n",
                (unsigned long)index,
                (unsigned long)sample.sample_tick,
                (unsigned int)sample.status_reg,
                (unsigned long)sample.config_version,
                (int)sample.gyro_raw.x,
                (int)sample.gyro_raw.y,
                (int)sample.gyro_raw.z,
                (int)sample.accel_raw.x,
                (int)sample.accel_raw.y,
                (int)sample.accel_raw.z);

        rt_kprintf("[IMU][UNIT] "
                "i=%lu gd=%ld,%ld,%ld au=%ld,%ld,%ld\n",
                (unsigned long)index,
                (long)sample.gyro_udps.x,
                (long)sample.gyro_udps.y,
                (long)sample.gyro_udps.z,
                (long)sample.accel_ug.x,
                (long)sample.accel_ug.y,
                (long)sample.accel_ug.z);
        }
        else if (result == -RT_EBUSY)
        {
           not_ready_count++;

            rt_kprintf("[IMU][NOT_READY] index=%lu result=%d\n",
                       (unsigned long)index,
                       (int)result);
        }
        else
        {
            failure_count++;

           rt_kprintf("[IMU][READ_ERROR] index=%lu result=%d\n",
                       (unsigned long)index,
                       (int)result);
        }

        if ((index + 1U) < count)
        {
            current_tick = rt_tick_get();
            elapsed_tick =(rt_tick_t)(current_tick - iteration_start_tick);

            if (elapsed_tick < period_tick)
            {
                rt_thread_delay(period_tick - elapsed_tick);
            }
        }
    }

    rt_kprintf("[IMU][SUMMARY] "
               "attempt=%lu success=%lu not_ready=%lu failure=%lu\n",
               (unsigned long)attempt_count,
               (unsigned long)success_count,
               (unsigned long)not_ready_count,
               (unsigned long)failure_count);

    if (delta_count > 0U)
    {
        rt_kprintf("[IMU][INTERVAL] "
                   "samples=%lu min_tick=%lu max_tick=%lu avg_tick=%lu\n",
                   (unsigned long)delta_count,
                   (unsigned long)minimum_delta,
                   (unsigned long)maximum_delta,
                   (unsigned long)(delta_sum /(rt_uint64_t)delta_count));
    }
    else
    {
        rt_kprintf("[IMU][INTERVAL] samples=0 result=N/A\n");
    }
    if ((failure_count != 0U) || (success_count == 0U))
    {
        return -RT_EIO;
    }

    return RT_EOK;
}

/*
 * 简介：按指定次数和周期执行MMC5603NJ单次磁场测量。
 * 输入：mmc_sample_test [count] [period_ms]
 * 说明：运行在FinSH线程，不创建后台线程，不修改UI对象。
 */
static int mmc_sample_test(int argc, char **argv)
{
    sensor_magnetometer_sample_t sample = {0};
    rt_uint32_t count = 10U;
    rt_uint32_t period_ms = 50U;
    rt_uint32_t index;
    rt_uint32_t attempt_count = 0U;
    rt_uint32_t success_count = 0U;
    rt_uint32_t timeout_count = 0U;
    rt_uint32_t failure_count = 0U;

    rt_tick_t period_tick;
    rt_tick_t iteration_start_tick;
    rt_tick_t current_tick;
    rt_tick_t elapsed_tick;

    rt_bool_t have_previous_tick = RT_FALSE;
    rt_bool_t have_delta = RT_FALSE;
    rt_tick_t previous_tick = 0U;
    rt_tick_t delta;
    rt_tick_t minimum_delta = 0U;
    rt_tick_t maximum_delta = 0U;
    rt_uint64_t delta_sum = 0U;
    rt_uint32_t delta_count = 0U;

    rt_err_t result;

    if (argc > 3)
    {
        rt_kprintf(
            "usage: mmc_sample_test [count] [period_ms]\n");
        return -RT_EINVAL;
    }

    if (argc >= 2)
    {
        result = sensor_test_parse_uint_arg(
            argv[1], 1U, 2000U, &count);
        if (result != RT_EOK)
        {
            rt_kprintf(
                "[MMC][ARG_ERROR] count=%s range=1..2000\n",
                argv[1]);
            return result;
        }
    }

    if (argc >= 3)
    {
        result = sensor_test_parse_uint_arg(
            argv[2], 10U, 1000U, &period_ms);
        if (result != RT_EOK)
        {
            rt_kprintf(
                "[MMC][ARG_ERROR] period_ms=%s range=10..1000\n",
                argv[2]);
            return result;
        }
    }

    result = sensor_bus_port_init();
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[MMC][BUS_INIT_ERROR] result=%d\n",
            (int)result);
        return result;
    }

    result = mmc5603nj_init();
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[MMC][INIT_ERROR] result=%d\n",
            (int)result);
        return result;
    }

    rt_kprintf(
        "[MMC][CONFIG][OK] id=0x10 mode=single "
        "bw=00 auto_sr=1 cfg=1 count=%lu period_ms=%lu\n",
        (unsigned long)count,
        (unsigned long)period_ms);

    period_tick =
        rt_tick_from_millisecond((rt_int32_t)period_ms);

    for (index = 0U; index < count; index++)
    {
        iteration_start_tick = rt_tick_get();
        attempt_count++;

        result = mmc5603nj_read_sample(&sample);

        if (result == RT_EOK)
        {
            success_count++;

            if (have_previous_tick == RT_TRUE)
            {
                delta = (rt_tick_t)
                    (sample.sample_tick - previous_tick);

                if (have_delta == RT_FALSE)
                {
                    minimum_delta = delta;
                    maximum_delta = delta;
                    have_delta = RT_TRUE;
                }
                else
                {
                    if (delta < minimum_delta)
                    {
                        minimum_delta = delta;
                    }

                    if (delta > maximum_delta)
                    {
                        maximum_delta = delta;
                    }
                }

                delta_sum += (rt_uint64_t)delta;
                delta_count++;
            }

            previous_tick = sample.sample_tick;
            have_previous_tick = RT_TRUE;

            rt_kprintf(
                "[MMC][RAW] i=%lu t=%lu st=0x%02X cfg=%lu "
                "code=%lu,%lu,%lu centered=%ld,%ld,%ld\n",
                (unsigned long)index,
                (unsigned long)sample.sample_tick,
                (unsigned int)sample.status_reg,
                (unsigned long)sample.config_version,
                (unsigned long)sample.code_u20.x,
                (unsigned long)sample.code_u20.y,
                (unsigned long)sample.code_u20.z,
                (long)sample.centered_raw.x,
                (long)sample.centered_raw.y,
                (long)sample.centered_raw.z);

            rt_kprintf(
                "[MMC][UNIT] i=%lu nt=%ld,%ld,%ld\n",
                (unsigned long)index,
                (long)sample.field_nt.x,
                (long)sample.field_nt.y,
                (long)sample.field_nt.z);
        }
        else if (result == -RT_ETIMEOUT)
        {
            timeout_count++;

            rt_kprintf(
                "[MMC][READY_TIMEOUT] index=%lu result=%d\n",
                (unsigned long)index,
                (int)result);
        }
        else
        {
            failure_count++;

            rt_kprintf(
                "[MMC][READ_ERROR] index=%lu result=%d\n",
                (unsigned long)index,
                (int)result);
        }

        if ((index + 1U) < count)
        {
            current_tick = rt_tick_get();
            elapsed_tick = (rt_tick_t)
                (current_tick - iteration_start_tick);

            if (elapsed_tick < period_tick)
            {
                rt_thread_delay(period_tick - elapsed_tick);
            }
        }
    }

    rt_kprintf(
        "[MMC][SUMMARY] attempt=%lu success=%lu "
        "timeout=%lu failure=%lu\n",
        (unsigned long)attempt_count,
        (unsigned long)success_count,
        (unsigned long)timeout_count,
        (unsigned long)failure_count);

    if (delta_count > 0U)
    {
        rt_kprintf(
            "[MMC][INTERVAL] samples=%lu min_tick=%lu "
            "max_tick=%lu avg_tick=%lu\n",
            (unsigned long)delta_count,
            (unsigned long)minimum_delta,
            (unsigned long)maximum_delta,
            (unsigned long)
                (delta_sum / (rt_uint64_t)delta_count));
    }
    else
    {
        rt_kprintf(
            "[MMC][INTERVAL] samples=0 result=N/A\n");
    }

    if ((success_count != attempt_count) ||
        (timeout_count != 0U) ||
        (failure_count != 0U))
    {
        return -RT_EIO;
    }

    return RT_EOK;
}

/*
 * 简介：执行一次显式SET/RESET配对测量。
 */
static int mmc_set_reset_test(int argc, char **argv)
{
    mmc5603nj_set_reset_result_t pair = {0};
    rt_err_t result;

    (void)argv;

    if (argc != 1)
    {
        rt_kprintf("usage: mmc_set_reset_test\n");
        return -RT_EINVAL;
    }

    result = sensor_bus_port_init();
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[MMC][SR][BUS_INIT_ERROR] result=%d\n",
            (int)result);
        return result;
    }

    result = mmc5603nj_init();
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[MMC][SR][INIT_ERROR] result=%d\n",
            (int)result);
        return result;
    }

    result = mmc5603nj_run_set_reset_pair(&pair);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[MMC][SR][ERROR] result=%d\n",
            (int)result);
        return result;
    }

    rt_kprintf(
        "[MMC][SR][SET] code=%lu,%lu,%lu "
        "centered=%ld,%ld,%ld\n",
        (unsigned long)pair.set_sample.code_u20.x,
        (unsigned long)pair.set_sample.code_u20.y,
        (unsigned long)pair.set_sample.code_u20.z,
        (long)pair.set_sample.centered_raw.x,
        (long)pair.set_sample.centered_raw.y,
        (long)pair.set_sample.centered_raw.z);

    rt_kprintf(
        "[MMC][SR][RESET] code=%lu,%lu,%lu "
        "centered=%ld,%ld,%ld\n",
        (unsigned long)pair.reset_sample.code_u20.x,
        (unsigned long)pair.reset_sample.code_u20.y,
        (unsigned long)pair.reset_sample.code_u20.z,
        (long)pair.reset_sample.centered_raw.x,
        (long)pair.reset_sample.centered_raw.y,
        (long)pair.reset_sample.centered_raw.z);

    rt_kprintf(
        "[MMC][SR][RESULT] field_raw=%ld,%ld,%ld "
        "field_nt=%ld,%ld,%ld offset_raw=%ld,%ld,%ld\n",
        (long)pair.field_centered_raw.x,
        (long)pair.field_centered_raw.y,
        (long)pair.field_centered_raw.z,
        (long)pair.field_nt.x,
        (long)pair.field_nt.y,
        (long)pair.field_nt.z,
        (long)pair.offset_centered_raw.x,
        (long)pair.offset_centered_raw.y,
        (long)pair.offset_centered_raw.z);

    return RT_EOK;
}

/*
 * 简介：验证未触发测量时ready等待会有限超时，
 *       并证明超时后正常测量能够恢复。
 */
static int mmc_ready_timeout_test(int argc, char **argv)
{
    sensor_magnetometer_sample_t recovered = {0};
    rt_uint32_t elapsed_ms = 0U;
    rt_err_t result;

    (void)argv;

    if (argc != 1)
    {
        rt_kprintf("usage: mmc_ready_timeout_test\n");
        return -RT_EINVAL;
    }

    result = sensor_bus_port_init();
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[MMC][TIMEOUT][BUS_INIT_ERROR] result=%d\n",
            (int)result);
        return result;
    }

    result = mmc5603nj_init();
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[MMC][TIMEOUT][INIT_ERROR] result=%d\n",
            (int)result);
        return result;
    }

    result = mmc5603nj_run_ready_timeout_test(
        &elapsed_ms,
        &recovered);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[MMC][TIMEOUT][ERROR] result=%d\n",
            (int)result);
        return result;
    }

    rt_kprintf(
        "[MMC][TIMEOUT][OK] expected_timeout=1 "
        "elapsed_ms=%lu recovery_valid=%u "
        "recovery_nt=%ld,%ld,%ld\n",
        (unsigned long)elapsed_ms,
        (unsigned int)recovered.valid,
        (long)recovered.field_nt.x,
        (long)recovered.field_nt.y,
        (long)recovered.field_nt.z);

    return RT_EOK;
}


MSH_CMD_EXPORT(sensor_id_test,
                read three sensor IDs; optional recover runs safe NACK test);
MSH_CMD_EXPORT(imu_sample_test,
                sample LSM6DS3TR-C: imu_sample_test [count] [period_ms]);
MSH_CMD_EXPORT(mmc_sample_test,
               sample MMC5603NJ: mmc_sample_test [count] [period_ms]);

MSH_CMD_EXPORT(mmc_set_reset_test,
               run MMC5603NJ explicit SET and RESET pair);

MSH_CMD_EXPORT(mmc_ready_timeout_test,
               verify MMC5603NJ ready timeout and recovery);
#endif
