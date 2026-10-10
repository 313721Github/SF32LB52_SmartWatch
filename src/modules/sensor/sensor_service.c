#include "rtthread.h"
#include "sensor_bus_port.h"
#include "lsm6ds3tr_c.h"
#include "mmc5603nj.h"
#include "ltr303als01.h"

#ifdef RT_USING_FINSH
#include "finsh.h"
#include <string.h>

//私有函数
static rt_err_t task4_probe_lsm(rt_uint8_t *active_address);            //探测LSM6DS3TR-C传感器并返回其I2C地址
static rt_err_t task4_probe_mmc5603nj(void);                            //探测MMC5603NJ传感器
static rt_err_t task4_probe_ltr303(void);                               //探测LTR303ALS01传感器
static rt_err_t task4_probe_all(rt_uint8_t *lsm_address);               //探测所有传感器并返回LSM6DS3TR-C的I2C地址
static rt_err_t task4_failure_recovery_test(rt_uint8_t active_address); //探测LSM6DS3TR-C传感器并进行故障恢复测试
static int sensor_id_test(int argc, char **argv);                       //通过FINSH命令行探测所有传感器并返回LSM6DS3TR-C的I2C地址

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

MSH_CMD_EXPORT(sensor_id_test,
               read three sensor IDs; optional recover runs safe NACK test);
#endif
