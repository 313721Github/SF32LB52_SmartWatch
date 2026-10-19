#include "rtthread.h"
#include "sensor_bus_port.h"
#include "lsm6ds3tr_c.h"
#include "mmc5603nj.h"
#include "ltr303als01.h"
#include "sensor_service.h"

#ifdef RT_USING_FINSH
#include "finsh.h"
#include <string.h>
#include <errno.h>
#include <stdlib.h>

/**
 * @brief ALS照度公式单个构造测试用例
 *
 * 保存一组输入参数和对应的预期结果，由als_formula_test()
 * 调用纯计算接口并逐字段比较。该结构只用于FinSH测试，
 * 不参与实机样本发布。
 */
typedef struct
{
    const char *name;
    /**< 测试用例名称，必须指向生命周期覆盖整个测试的只读字符串。 */

    rt_uint16_t ch0_raw;
    /**< 输入的CH0原始值。 */

    rt_uint16_t ch1_raw;
    /**< 输入的CH1原始值。 */

    rt_uint16_t gain;
    /**< 输入的实际增益倍数。 */

    rt_uint16_t integration_ms;
    /**< 输入的积分时间，单位ms。 */

    rt_uint32_t expected_ratio_x10000;
    /**< 预期通道比例，CH1/(CH0+CH1)放大10000倍并四舍五入。 */

    rt_uint32_t expected_lux_mlux;
    /**< 预期照度估算值，单位mlux。
         当expected_lux_valid为RT_FALSE时通常约定为0。 */

    rt_bool_t expected_lux_valid;
    /**< 预期照度是否可信。 */

    rt_bool_t expected_saturated;
    /**< 预期是否检测到任一通道饱和。 */

    sensor_als_estimate_reason_t expected_reason;
    /**< 预期估算分类原因。 */
} als_formula_case_t;

static const als_formula_case_t g_als_formula_cases[] =
{
    {
        "zero",
        0U, 0U, 1U, 100U,
        0U, 0U,
        RT_TRUE, RT_FALSE,
        SENSOR_ALS_REASON_ZERO_COUNTS
    },
    {
        "below_045",
        5501U, 4499U, 1U, 100U,
        4499U, 14735868U,
        RT_TRUE, RT_FALSE,
        SENSOR_ALS_REASON_OK
    },
    {
        "equal_045",
        5500U, 4500U, 1U, 100U,
        4500U, 14735150U,
        RT_TRUE, RT_FALSE,
        SENSOR_ALS_REASON_OK
    },
    {
        "below_064",
        3601U, 6399U, 1U, 100U,
        6399U, 2898113U,
        RT_TRUE, RT_FALSE,
        SENSOR_ALS_REASON_OK
    },
    {
        "equal_064",
        3600U, 6400U, 1U, 100U,
        6400U, 2891760U,
        RT_TRUE, RT_FALSE,
        SENSOR_ALS_REASON_OK
    },
    {
        "below_085",
        1501U, 8499U, 1U, 100U,
        8499U, 1896624U,
        RT_TRUE, RT_FALSE,
        SENSOR_ALS_REASON_OK
    },
    {
        "equal_085",
        1500U, 8500U, 1U, 100U,
        8500U, 0U,
        RT_FALSE, RT_FALSE,
        SENSOR_ALS_REASON_IR_DOMINANT
    },
    {
        "wrong_branch_guard",
        100U, 300U, 1U, 100U,
        7500U, 94810U,
        RT_TRUE, RT_FALSE,
        SENSOR_ALS_REASON_OK
    },
    {
        "int32_overflow_guard",
        60000U, 49091U, 1U, 100U,
        4500U, 160746913U,
        RT_TRUE, RT_FALSE,
        SENSOR_ALS_REASON_OK
    },
    {
        "saturated",
        65535U, 1000U, 1U, 100U,
        150U, 0U,
        RT_FALSE, RT_TRUE,
        SENSOR_ALS_REASON_SATURATED
    },
    {
        "normalize_1x_100ms",
        1000U, 200U, 1U, 100U,
        1667U, 1995480U,
        RT_TRUE, RT_FALSE,
        SENSOR_ALS_REASON_OK
    },
    {
        "normalize_1x_200ms",
        1000U, 200U, 1U, 200U,
        1667U, 997740U,
        RT_TRUE, RT_FALSE,
        SENSOR_ALS_REASON_OK
    },
    {
        "normalize_2x_100ms",
        1000U, 200U, 2U, 100U,
        1667U, 997740U,
        RT_TRUE, RT_FALSE,
        SENSOR_ALS_REASON_OK
    },
    {
        "normalize_2x_200ms",
        1000U, 200U, 2U, 200U,
        1667U, 498870U,
        RT_TRUE, RT_FALSE,
        SENSOR_ALS_REASON_OK
    }
};

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
//ltr303als01测试
static const char *als_reason_name(sensor_als_estimate_reason_t reason);//把ALS估算原因枚举转换为固定字符串，供FinSH日志使用
static int als_formula_test(int argc, char **argv);                     //运行14组构造值测试。
static int als_sample_test(int argc, char **argv);                      //初始化LTR303ALS01并按指定次数和周期读取双通道样本
static rt_err_t sensor_diag_require_service_stopped(void);

/* 统一sensor服务诊断辅助函数。 */
static const char *sensor_service_device_state_name(
                                sensor_device_state_t state);           //将器件状态枚举转换为诊断字符串
static rt_err_t sensor_service_parse_device(
                                const char *text,
                                sensor_device_id_t *device);            //解析器件名称并返回器件编号
static rt_err_t sensor_service_parse_probe_mask(
                                const char *text,
                                rt_uint32_t *device_mask);               //解析重探目标并生成器件位图
static rt_err_t sensor_service_parse_fault(
                                const char *text,
                                sensor_test_fault_t *fault);            //解析故障名称并返回故障类型
static void sensor_service_print_status(
                                const char *name,
                                const sensor_device_status_t *status);  //输出单颗器件的状态和样本时序
static void sensor_service_print_device_stats(
                                const char *name,
                                const sensor_device_stats_t *stats);    //输出单颗器件的运行统计

/* 统一sensor服务FinSH命令。 */
static int sensor_service_start(int argc, char **argv);                 //启动统一sensor服务
static int sensor_service_show(int argc, char **argv);                  //显示最新快照和三颗器件状态
static int sensor_service_stats(int argc, char **argv);                 //显示服务与各器件累计统计
static int sensor_service_probe(int argc, char **argv);                 //提交指定器件的异步重探请求
static int sensor_service_inject(int argc, char **argv);                //配置指定器件的有限故障注入

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

    if (sensor_diag_require_service_stopped() != RT_EOK)
    {
        return -RT_EBUSY;
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

    if (sensor_diag_require_service_stopped() != RT_EOK)
    {
        return -RT_EBUSY;
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
    if (sensor_diag_require_service_stopped() != RT_EOK)
    {
        return -RT_EBUSY;
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
    if (sensor_diag_require_service_stopped() != RT_EOK)
    {
        return -RT_EBUSY;
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
    if (sensor_diag_require_service_stopped() != RT_EOK)
    {
        return -RT_EBUSY;
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


static const char *als_reason_name(
    sensor_als_estimate_reason_t reason)
{
    switch (reason)
    {
    case SENSOR_ALS_REASON_OK:
        return "OK";

    case SENSOR_ALS_REASON_ZERO_COUNTS:
        return "ZERO_COUNTS";

    case SENSOR_ALS_REASON_IR_DOMINANT:
        return "IR_DOMINANT";

    case SENSOR_ALS_REASON_SATURATED:
        return "SATURATED";

    case SENSOR_ALS_REASON_DEVICE_INVALID:
        return "DEVICE_INVALID";

    case SENSOR_ALS_REASON_GAIN_MISMATCH:
        return "GAIN_MISMATCH";

    case SENSOR_ALS_REASON_MATH_ERROR:
        return "MATH_ERROR";

    default:
        return "UNKNOWN";
    }
}

/*
 * 简介：使用14组构造值验证LTR303照度公式。
 * 说明：不访问I2C，不初始化硬件，不修改UI对象。
 */
static int als_formula_test(int argc, char **argv)
{
    sensor_als_estimate_t actual;
    const als_formula_case_t *test_case;

    rt_uint32_t index;
    rt_uint32_t passed = 0U;
    rt_uint32_t total;

    rt_bool_t matched;
    rt_err_t result;

    (void)argv;

    if (argc != 1)
    {
        rt_kprintf("usage: als_formula_test\n");
        return -RT_EINVAL;
    }

    total = (rt_uint32_t)
        (sizeof(g_als_formula_cases) /
         sizeof(g_als_formula_cases[0]));

    for (index = 0U; index < total; index++)
    {
        test_case = &g_als_formula_cases[index];

        /*
         * 使用哨兵值覆盖actual，证明估算函数完整写入所有字段。
         */
        memset(&actual, 0xA5, sizeof(actual));

        result = ltr303als01_estimate_lux(
            test_case->ch0_raw,
            test_case->ch1_raw,
            test_case->gain,
            test_case->integration_ms,
            &actual);

        /*
         * 同时比较返回值和五个结果字段。
         */
        matched =
            (result == RT_EOK) &&
            (actual.ratio_x10000 ==
             test_case->expected_ratio_x10000) &&
            (actual.lux_mlux ==
             test_case->expected_lux_mlux) &&
            (actual.lux_valid ==
             test_case->expected_lux_valid) &&
            (actual.saturated ==
             test_case->expected_saturated) &&
            (actual.reason ==
             test_case->expected_reason);

        if (matched == RT_TRUE)
        {
            passed++;
        }

    rt_kprintf(
        "[ALS][FORMULA][%s] "
        "i=%lu name=%s result=%d\n",
        (matched == RT_TRUE) ? "PASS" : "FAIL",
        (unsigned long)index,
        test_case->name,
        (int)result);

    rt_kprintf(
        "[ALS][FORMULA][DATA] "
        "ch0=%u ch1=%u g=%u im=%u "
        "ratio=%lu mlux=%lu lv=%u sat=%u reason=%s\n",
        (unsigned int)test_case->ch0_raw,
        (unsigned int)test_case->ch1_raw,
        (unsigned int)test_case->gain,
        (unsigned int)test_case->integration_ms,
        (unsigned long)actual.ratio_x10000,
        (unsigned long)actual.lux_mlux,
        (unsigned int)actual.lux_valid,
        (unsigned int)actual.saturated,
        als_reason_name(actual.reason));
    }

    rt_kprintf(
        "[ALS][FORMULA][SUMMARY] "
        "passed=%lu total=%lu result=%s\n",
        (unsigned long)passed,
        (unsigned long)total,
        (passed == total) ? "PASS" : "FAIL");

    return (passed == total) ? RT_EOK : -RT_EIO;
}
/*
 * 简介：初始化LTR303并按指定次数和周期采集双通道样本。
 * 输入：als_sample_test [count] [period_ms]
 * 默认：count=10，period_ms=200。
 * 返回值：测试通过返回RT_EOK；参数、初始化、超时、总线或
 *         关键样本状态异常时返回对应错误。
 * 说明：运行在FinSH线程，不创建后台线程，不修改UI对象。
 */
static int als_sample_test(int argc, char **argv)
{
    ltr303als01_config_readback_t config = {0};
    sensor_als_sample_t sample = {0};

    rt_uint32_t count = 10U;
    rt_uint32_t period_ms = 200U;
    rt_uint32_t index;

    rt_uint32_t attempt_count = 0U;
    rt_uint32_t raw_valid_count = 0U;
    rt_uint32_t lux_valid_count = 0U;

    rt_uint32_t zero_count = 0U;
    rt_uint32_t ir_count = 0U;
    rt_uint32_t saturated_count = 0U;
    rt_uint32_t device_invalid_count = 0U;
    rt_uint32_t gain_mismatch_count = 0U;
    rt_uint32_t math_error_count = 0U;

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
            "usage: als_sample_test [count] [period_ms]\n");
        return -RT_EINVAL;
    }

    /* 解析采样次数。 */
    if (argc >= 2)
    {
        result = sensor_test_parse_uint_arg(
            argv[1],
            1U,
            2000U,
            &count);
        if (result != RT_EOK)
        {
            rt_kprintf(
                "[ALS][ARG_ERROR] count=%s range=1..2000\n",
                argv[1]);
            return result;
        }
    }

    /*
     * 解析软件采样周期。
     * 当前器件更新周期为200 ms，因此不允许配置得更短。
     */
    if (argc >= 3)
    {
        result = sensor_test_parse_uint_arg(
            argv[2],
            200U,
            2000U,
            &period_ms);
        if (result != RT_EOK)
        {
            rt_kprintf(
                "[ALS][ARG_ERROR] "
                "period_ms=%s range=200..2000\n",
                argv[2]);
            return result;
        }
    }
    if (sensor_diag_require_service_stopped() != RT_EOK)
    {
        return -RT_EBUSY;
    }
    /* 初始化项目侧传感器总线。 */
    result = sensor_bus_port_init();
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[ALS][BUS_INIT_ERROR] result=%d\n",
            (int)result);
        return result;
    }

    /* 验证LTR303身份、写入冻结配置并核对读回值。 */
    result = ltr303als01_init(&config);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[ALS][INIT_ERROR] result=%d\n",
            (int)result);
        return result;
    }

    rt_kprintf(
        "[ALS][CONFIG][OK] "
        "contr=0x%02X rate=0x%02X "
        "gain=1 int_ms=100 repeat_ms=200 cfg=1 "
        "count=%lu period_ms=%lu\n",
        (unsigned int)config.als_contr,
        (unsigned int)config.als_meas_rate,
        (unsigned long)count,
        (unsigned long)period_ms);

    period_tick =rt_tick_from_millisecond((rt_int32_t)period_ms);

    for (index = 0U; index < count; index++)
    {
        iteration_start_tick = rt_tick_get();
        attempt_count++;

        result = ltr303als01_read_sample(&sample);

        if (result == RT_EOK)
        {
            /*
             * raw有效和照度有效是两层语义。
             * 饱和或红外占优时raw仍可能有效，但lux无效。
             */
            if (sample.valid == RT_TRUE)
            {
                raw_valid_count++;
            }

            if (sample.estimate.lux_valid == RT_TRUE)
            {
                lux_valid_count++;
            }

            /* 统计本次样本的估算分类。 */
            switch (sample.estimate.reason)
            {
            case SENSOR_ALS_REASON_OK:
                break;

            case SENSOR_ALS_REASON_ZERO_COUNTS:
                zero_count++;
                break;

            case SENSOR_ALS_REASON_IR_DOMINANT:
                ir_count++;
                break;

            case SENSOR_ALS_REASON_SATURATED:
                saturated_count++;
                break;

            case SENSOR_ALS_REASON_DEVICE_INVALID:
                device_invalid_count++;
                break;

            case SENSOR_ALS_REASON_GAIN_MISMATCH:
                gain_mismatch_count++;
                break;

            case SENSOR_ALS_REASON_MATH_ERROR:
                math_error_count++;
                break;

            default:
                /*
                 * 未知枚举表示驱动与测试入口的契约不一致，
                 * 计入普通失败。
                 */
                failure_count++;
                break;
            }

            /*
             * 仅对raw有效样本统计采样完成时间间隔。
             * rt_tick_t为无符号类型时，该减法可以处理一次正常回绕。
             */
            if (sample.valid == RT_TRUE)
            {
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
            }

            rt_kprintf(
                "[ALS][RAW] "
                "i=%lu t=%lu st=0x%02X cfg=%lu "
                "ch0=%u ch1=%u gain=%u valid=%u\n",
                (unsigned long)index,
                (unsigned long)sample.sample_tick,
                (unsigned int)sample.status_reg,
                (unsigned long)sample.config_version,
                (unsigned int)sample.ch0_raw,
                (unsigned int)sample.ch1_raw,
                (unsigned int)sample.gain,
                (unsigned int)sample.valid);

            rt_kprintf(
                "[ALS][LUX] "
                "i=%lu ratio=%lu mlux=%lu "
                "lux_valid=%u saturated=%u "
                "calibrated=%u reason=%s\n",
                (unsigned long)index,
                (unsigned long)
                    sample.estimate.ratio_x10000,
                (unsigned long)
                    sample.estimate.lux_mlux,
                (unsigned int)
                    sample.estimate.lux_valid,
                (unsigned int)
                    sample.estimate.saturated,
                (unsigned int)sample.calibrated,
                als_reason_name(
                    sample.estimate.reason));
        }
        else if (result == -RT_ETIMEOUT)
        {
            timeout_count++;

            rt_kprintf(
                "[ALS][READY_TIMEOUT] "
                "index=%lu result=%d\n",
                (unsigned long)index,
                (int)result);
        }
        else
        {
            failure_count++;

            rt_kprintf(
                "[ALS][READ_ERROR] "
                "index=%lu result=%d\n",
                (unsigned long)index,
                (int)result);
        }

        /*
         * 从本轮开始时刻补足软件周期，避免在读取耗时后再固定
         * 延时period_ms而使实际周期持续变长。
         */
        if ((index + 1U) < count)
        {
            current_tick = rt_tick_get();
            elapsed_tick = (rt_tick_t)
                (current_tick - iteration_start_tick);

            if (elapsed_tick < period_tick)
            {
                rt_thread_delay(
                    period_tick - elapsed_tick);
            }
        }
    }

    rt_kprintf(
        "[ALS][SUMMARY] "
        "attempt=%lu raw_valid=%lu lux_valid=%lu "
        "timeout=%lu failure=%lu\n",
        (unsigned long)attempt_count,
        (unsigned long)raw_valid_count,
        (unsigned long)lux_valid_count,
        (unsigned long)timeout_count,
        (unsigned long)failure_count);

    rt_kprintf(
        "[ALS][CLASS1] "
        "zero=%lu ir=%lu saturated=%lu\n",
        (unsigned long)zero_count,
        (unsigned long)ir_count,
        (unsigned long)saturated_count);

    rt_kprintf(
        "[ALS][CLASS2] "
        "device_invalid=%lu gain_mismatch=%lu math_error=%lu\n",
        (unsigned long)device_invalid_count,
        (unsigned long)gain_mismatch_count,
        (unsigned long)math_error_count);

    if (delta_count > 0U)
    {
        rt_kprintf(
            "[ALS][INTERVAL] "
            "samples=%lu min_tick=%lu "
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
            "[ALS][INTERVAL] samples=0 result=N/A\n");
    }

    /*
     * IR_DOMINANT和SATURATED是有效环境分类，不直接使测试失败。
     * 器件状态、配置、数学、总线或等待异常均使测试失败。
     */
    if ((raw_valid_count == 0U) ||
        (timeout_count != 0U) ||
        (failure_count != 0U) ||
        (device_invalid_count != 0U) ||
        (gain_mismatch_count != 0U) ||
        (math_error_count != 0U))
    {
        return -RT_EIO;
    }

    return RT_EOK;
}
/*
 * 旧测试命令会直接访问传感器驱动和I2C。
 * 统一服务运行后，I2C只能由sensor线程访问，
 * 因此必须拒绝这些命令，避免并发事务和配置竞争。
 */
static rt_err_t sensor_diag_require_service_stopped(void)
{
    if (sensor_service_is_running() == RT_TRUE)
    {
        rt_kprintf(
            "[SENSOR][TEST][ERR] unified service is running; "
            "direct driver test is disabled\n");

        return -RT_EBUSY;
    }

    return RT_EOK;
}

/*
 * 简介：把器件运行状态枚举转换为固定只读字符串。
 * 输入：state为快照中的器件状态。
 * 返回值：状态对应的静态字符串；未知值返回"unknown"。
 * 说明：返回值只用于诊断打印，调用者不得修改或释放。
 */
static const char *sensor_service_device_state_name(sensor_device_state_t state)
{
    switch (state)
    {
    case SENSOR_DEVICE_STATE_UNINITIALIZED:
        return "uninitialized";

    case SENSOR_DEVICE_STATE_READY:
        return "ready";

    case SENSOR_DEVICE_STATE_TEMP_FAILED:
        return "temp_failed";

    case SENSOR_DEVICE_STATE_OFFLINE:
        return "offline";

    default:
        return "unknown";
    }
}

/*
 * 简介：把FinSH器件名称转换为统一服务器件编号。
 * 输入：text为"imu"、"mmc"或"als"；
 *       device为调用者持有的输出缓冲区。
 * 返回值：解析成功返回RT_EOK；参数无效返回-RT_EINVAL。
 * 说明：全部检查完成后才更新输出，避免失败时留下部分结果。
 */
static rt_err_t sensor_service_parse_device(const char *text,sensor_device_id_t *device)
{
    sensor_device_id_t candidate;

    if ((text == RT_NULL) || (device == RT_NULL))
    {
        return -RT_EINVAL;
    }

    if (strcmp(text, "imu") == 0)
    {
        candidate = SENSOR_DEVICE_IMU;
    }
    else if (strcmp(text, "mmc") == 0)
    {
        candidate = SENSOR_DEVICE_MMC;
    }
    else if (strcmp(text, "als") == 0)
    {
        candidate = SENSOR_DEVICE_ALS;
    }
    else
    {
        return -RT_EINVAL;
    }

    *device = candidate;
    return RT_EOK;
}

/*
 * 简介：把FinSH重探目标转换为器件位图。
 * 输入：text为"imu"、"mmc"、"als"或"all"；
 *       device_mask为调用者持有的输出缓冲区。
 * 返回值：解析成功返回RT_EOK；参数无效返回-RT_EINVAL。
 * 说明：位图只提交给sensor线程，调用FinSH命令的线程不执行I2C。
 */
static rt_err_t sensor_service_parse_probe_mask(const char *text, rt_uint32_t *device_mask)
{
    rt_uint32_t candidate;

    if ((text == RT_NULL) || (device_mask == RT_NULL))
    {
        return -RT_EINVAL;
    }

    if (strcmp(text, "imu") == 0)
    {
        candidate = SENSOR_DEVICE_MASK_IMU;
    }
    else if (strcmp(text, "mmc") == 0)
    {
        candidate = SENSOR_DEVICE_MASK_MMC;
    }
    else if (strcmp(text, "als") == 0)
    {
        candidate = SENSOR_DEVICE_MASK_ALS;
    }
    else if (strcmp(text, "all") == 0)
    {
        candidate = SENSOR_DEVICE_MASK_ALL;
    }
    else
    {
        return -RT_EINVAL;
    }

    *device_mask = candidate;
    return RT_EOK;
}

/*
 * 简介：把FinSH故障名称转换为统一服务故障类型。
 * 输入：text为"none"、"io"、"short"或"timeout"；
 *       fault为调用者持有的输出缓冲区。
 * 返回值：解析成功返回RT_EOK；参数无效返回-RT_EINVAL。
 * 说明：软件故障只验证服务失败分支，不代表真实总线产生相同波形。
 */
static rt_err_t sensor_service_parse_fault(const char *text,sensor_test_fault_t *fault)
{
    sensor_test_fault_t candidate;

    if ((text == RT_NULL) || (fault == RT_NULL))
    {
        return -RT_EINVAL;
    }

    if (strcmp(text, "none") == 0)
    {
        candidate = SENSOR_TEST_FAULT_NONE;
    }
    else if (strcmp(text, "io") == 0)
    {
        candidate = SENSOR_TEST_FAULT_BUS_IO;
    }
    else if (strcmp(text, "short") == 0)
    {
        candidate = SENSOR_TEST_FAULT_SHORT_TRANSFER;
    }
    else if (strcmp(text, "timeout") == 0)
    {
        candidate = SENSOR_TEST_FAULT_TIMEOUT;
    }
    else
    {
        return -RT_EINVAL;
    }

    *fault = candidate;
    return RT_EOK;
}

/*
 * 简介：打印单颗器件随快照发布的状态。
 * 输入：name为器件显示名称；
 *       status为调用者已经取得的快照状态副本。
 * 返回值：无。
 * 说明：本函数只读取调用者副本，不访问服务内部对象，也不持锁。
 */
static void sensor_service_print_status(const char *name,const sensor_device_status_t *status)
{
    if ((name == RT_NULL) || (status == RT_NULL))
    {
        return;
    }

    rt_kprintf(
        "[SENSOR][SERVICE][%s] state=%s valid=%d stale=%d "
        "seq=%lu config=%lu sample_tick=%lu age=%lu error=%d\n",
        name,
        sensor_service_device_state_name(status->state),
        (int)status->valid,
        (int)status->stale,
        (unsigned long)status->seq,
        (unsigned long)status->config_version,
        (unsigned long)status->sample_tick,
        (unsigned long)status->age_ticks,
        (int)status->last_error);
}

/*
 * 简介：打印单颗器件的累计运行统计。
 * 输入：name为器件显示名称；
 *       stats为调用者已经取得的统计副本。
 * 返回值：无。
 * 说明：拆成两行打印，便于观察失败、恢复、调度和耗时关系。
 */
static void sensor_service_print_device_stats(
                                const char *name,
                                const sensor_device_stats_t *stats)
{
    if ((name == RT_NULL) || (stats == RT_NULL))
    {
        return;
    }

    rt_kprintf(
        "[SENSOR][STATS][%s] attempt=%lu success=%lu failure=%lu "
        "timeout=%lu not_ready=%lu retry=%lu consecutive=%lu recovery=%lu\n",
        name,
        (unsigned long)stats->attempt_count,
        (unsigned long)stats->success_count,
        (unsigned long)stats->failure_count,
        (unsigned long)stats->timeout_count,
        (unsigned long)stats->not_ready_count,
        (unsigned long)stats->retry_count,
        (unsigned long)stats->consecutive_failure_count,
        (unsigned long)stats->recovery_count);

    rt_kprintf(
        "[SENSOR][STATS][%s] late=%lu missed=%lu "
        "last_operation_ticks=%lu max_operation_ticks=%lu\n",
        name,
        (unsigned long)stats->late_count,
        (unsigned long)stats->missed_count,
        (unsigned long)stats->last_operation_ticks,
        (unsigned long)stats->max_operation_ticks);
}

/*
 * 简介：创建统一sensor服务资源并启动唯一sensor线程。
 * 输入：命令格式为sensor_service_start，不接受额外参数。
 * 返回值：启动成功或服务已经运行返回RT_EOK；
 *         参数错误或服务初始化失败返回对应错误。
 * 说明：RT_EOK只表示线程和服务资源已经启动，
 *       不表示三颗器件都已就绪，也不表示已经取得有效样本。
 */
static int sensor_service_start(int argc, char **argv)
{
    rt_bool_t already_running;
    rt_err_t result;

    (void)argv;

    if (argc != 1)
    {
        rt_kprintf("usage: sensor_service_start\n");
        return -RT_EINVAL;
    }

    already_running = sensor_service_is_running();

    result = sensor_service_init();
    if (result != RT_EOK)
    {
        rt_kprintf("[SENSOR][SERVICE][ERR] start failed result=%d\n",
            (int)result);
        return result;
    }

    if (already_running == RT_TRUE)
    {
        rt_kprintf("[SENSOR][SERVICE] already running\n");
    }
    else
    {
        rt_kprintf(
            "[SENSOR][SERVICE][OK] thread started; "
            "device readiness is asynchronous\n");
    }

    return RT_EOK;
}

/*
 * 简介：取得并打印统一服务的最新一致快照。
 * 输入：命令格式为sensor_service_show，不接受额外参数。
 * 返回值：快照复制和打印成功返回RT_EOK；
 *         参数错误、服务未启动或取锁失败返回对应错误。
 * 说明：快照被复制到FinSH线程的局部变量后再打印，
 *       打印期间不持有服务mutex，也不阻塞sensor线程发布新样本。
 */
static int sensor_service_show(int argc, char **argv)
{
    sensor_snapshot_t snapshot = {0};
    rt_err_t result;

    (void)argv;

    if (argc != 1)
    {
        rt_kprintf("usage: sensor_service_show\n");
        return -RT_EINVAL;
    }

    result = sensor_service_get_snapshot(&snapshot);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[SENSOR][SERVICE][ERR] get snapshot failed result=%d\n",
            (int)result);
        return result;
    }

    rt_kprintf(
        "[SENSOR][SERVICE] publish_seq=%lu snapshot_tick=%lu\n",
        (unsigned long)snapshot.publish_seq,
        (unsigned long)snapshot.snapshot_tick);

    sensor_service_print_status("IMU", &snapshot.imu_status);
    sensor_service_print_status("MMC", &snapshot.mmc_status);
    sensor_service_print_status("ALS", &snapshot.als_status);

    /*
     * valid表示至少存在一份完整last-good样本。
     * 即使stale为真，也保留该样本用于故障诊断，但业务层不能把它
     * 伪装成当前新值。
     */
    if (snapshot.imu_status.valid == RT_TRUE)
    {
        rt_kprintf(
            "[SENSOR][SERVICE][IMU] "
            "accel_ug=%ld,%ld,%ld gyro_udps=%ld,%ld,%ld status=0x%02X\n",
            (long)snapshot.imu.accel_ug.x,
            (long)snapshot.imu.accel_ug.y,
            (long)snapshot.imu.accel_ug.z,
            (long)snapshot.imu.gyro_udps.x,
            (long)snapshot.imu.gyro_udps.y,
            (long)snapshot.imu.gyro_udps.z,
            snapshot.imu.status_reg);
    }

    if (snapshot.mmc_status.valid == RT_TRUE)
    {
        rt_kprintf(
            "[SENSOR][SERVICE][MMC] "
            "field_nt=%ld,%ld,%ld status=0x%02X\n",
            (long)snapshot.mmc.field_nt.x,
            (long)snapshot.mmc.field_nt.y,
            (long)snapshot.mmc.field_nt.z,
            snapshot.mmc.status_reg);
    }

    if (snapshot.als_status.valid == RT_TRUE)
    {
        rt_kprintf(
            "[SENSOR][SERVICE][ALS] "
            "ch0=%u ch1=%u ratio_x10000=%lu lux_mlux=%lu "
            "lux_valid=%d saturated=%d reason=%s status=0x%02X\n",
            snapshot.als.ch0_raw,
            snapshot.als.ch1_raw,
            (unsigned long)snapshot.als.estimate.ratio_x10000,
            (unsigned long)snapshot.als.estimate.lux_mlux,
            (int)snapshot.als.estimate.lux_valid,
            (int)snapshot.als.estimate.saturated,
            als_reason_name(snapshot.als.estimate.reason),
            snapshot.als.status_reg);
    }

    return RT_EOK;
}

/*
 * 简介：取得并打印统一sensor服务累计统计。
 * 输入：命令格式为sensor_service_stats，不接受额外参数。
 * 返回值：统计复制和打印成功返回RT_EOK；
 *         参数错误、服务未启动或取锁失败返回对应错误。
 * 说明：统计先复制到局部变量，日志输出不在snapshot mutex内执行。
 */
static int sensor_service_stats(int argc, char **argv)
{
    sensor_service_stats_t stats = {0};
    rt_err_t result;

    (void)argv;

    if (argc != 1)
    {
        rt_kprintf("usage: sensor_service_stats\n");
        return -RT_EINVAL;
    }

    result = sensor_service_get_stats(&stats);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[SENSOR][STATS][ERR] get stats failed result=%d\n",
            (int)result);
        return result;
    }

    rt_kprintf(
        "[SENSOR][STATS] loop=%lu wake=%lu publish=%lu "
        "notify=%lu notify_drop=%lu\n",
        (unsigned long)stats.loop_count,
        (unsigned long)stats.wake_count,
        (unsigned long)stats.publish_count,
        (unsigned long)stats.notify_count,
        (unsigned long)stats.notify_drop_count);

    rt_kprintf(
        "[SENSOR][STATS] lock_timeout=%lu probe_request=%lu "
        "last_loop_ticks=%lu max_loop_ticks=%lu\n",
        (unsigned long)stats.snapshot_lock_timeout_count,
        (unsigned long)stats.probe_request_count,
        (unsigned long)stats.last_loop_ticks,
        (unsigned long)stats.max_loop_ticks);

    sensor_service_print_device_stats("IMU", &stats.imu);
    sensor_service_print_device_stats("MMC", &stats.mmc);
    sensor_service_print_device_stats("ALS", &stats.als);

    return RT_EOK;
}

/*
 * 简介：向唯一sensor线程提交指定器件的异步重探请求。
 * 输入：命令格式为sensor_service_probe <imu|mmc|als|all>。
 * 返回值：请求成功提交返回RT_EOK；
 *         参数、服务状态或event发送错误返回对应错误。
 * 说明：命令返回成功只表示请求已进入合并位图；
 *       实际ID检查、配置和首份有效样本确认由sensor线程完成。
 */
static int sensor_service_probe(int argc, char **argv)
{
    rt_uint32_t device_mask;
    rt_err_t result;

    if (argc != 2)
    {
        rt_kprintf(
            "usage: sensor_service_probe <imu|mmc|als|all>\n");
        return -RT_EINVAL;
    }

    result = sensor_service_parse_probe_mask(argv[1], &device_mask);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[SENSOR][PROBE][ERR] invalid device=%s\n",
            argv[1]);
        return result;
    }

    result = sensor_service_request_probe(device_mask);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[SENSOR][PROBE][ERR] submit failed mask=0x%02lX result=%d\n",
            (unsigned long)device_mask,
            (int)result);
        return result;
    }

    rt_kprintf(
        "[SENSOR][PROBE][OK] request queued mask=0x%02lX; "
        "check snapshot and stats for completion\n",
        (unsigned long)device_mask);

    return RT_EOK;
}

/*
 * 简介：为指定器件配置固定次数的软件故障注入，或清除注入。
 * 输入：启用格式：
 *       sensor_service_inject <imu|mmc|als> <io|short|timeout> <count>
 *       清除格式：
 *       sensor_service_inject <imu|mmc|als> none
 * 返回值：配置成功返回RT_EOK；
 *         参数、服务状态或event发送错误返回对应错误。
 * 说明：count限制为1～1000；次数耗尽后服务自动恢复NONE。
 *       软件注入不改变真实I2C波形，只用于验证失败、退避和恢复分支。
 */
static int sensor_service_inject(int argc, char **argv)
{
    sensor_device_id_t device;
    sensor_test_fault_t fault;
    rt_uint32_t count = 0U;
    rt_err_t result;

    if ((argc != 3) && (argc != 4))
    {
        rt_kprintf(
            "usage:\n"
            "  sensor_service_inject <imu|mmc|als> "
            "<io|short|timeout> <count>\n"
            "  sensor_service_inject <imu|mmc|als> none\n");
        return -RT_EINVAL;
    }

    result = sensor_service_parse_device(argv[1], &device);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[SENSOR][INJECT][ERR] invalid device=%s\n",
            argv[1]);
        return result;
    }

    result = sensor_service_parse_fault(argv[2], &fault);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[SENSOR][INJECT][ERR] invalid fault=%s\n",
            argv[2]);
        return result;
    }

    if (fault == SENSOR_TEST_FAULT_NONE)
    {
        if (argc != 3)
        {
            rt_kprintf(
                "usage: sensor_service_inject "
                "<imu|mmc|als> none\n");
            return -RT_EINVAL;
        }

        count = 0U;
    }
    else
    {
        if (argc != 4)
        {
            rt_kprintf(
                "usage: sensor_service_inject "
                "<imu|mmc|als> <io|short|timeout> <count>\n");
            return -RT_EINVAL;
        }

        result = sensor_test_parse_uint_arg(
            argv[3],
            1U,
            1000U,
            &count);
        if (result != RT_EOK)
        {
            rt_kprintf(
                "[SENSOR][INJECT][ERR] invalid count=%s range=1..1000\n",
                argv[3]);
            return result;
        }
    }

    result = sensor_service_test_inject(device, fault, count);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[SENSOR][INJECT][ERR] configure failed "
            "device=%s fault=%s count=%lu result=%d\n",
            argv[1],
            argv[2],
            (unsigned long)count,
            (int)result);
        return result;
    }

    rt_kprintf(
        "[SENSOR][INJECT][OK] device=%s fault=%s count=%lu\n",
        argv[1],
        argv[2],
        (unsigned long)count);

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

MSH_CMD_EXPORT(als_formula_test,
               verify LTR303 lux formula boundaries);

MSH_CMD_EXPORT(als_sample_test,
               sample LTR303: als_sample_test [count] [period_ms]);

MSH_CMD_EXPORT(sensor_service_start,
               start unified sensor service);

MSH_CMD_EXPORT(sensor_service_show,
               show latest consistent sensor snapshot);

MSH_CMD_EXPORT(sensor_service_stats,
               show unified sensor service statistics);

MSH_CMD_EXPORT(sensor_service_probe,
               request probe: sensor_service_probe imu|mmc|als|all);

MSH_CMD_EXPORT(sensor_service_inject,
               inject fault: sensor_service_inject device fault count);
#endif
