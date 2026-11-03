#include "rtc_port.h"

#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
//rtc设备句柄
static rt_device_t g_rtc_device = RT_NULL;
//rtc设备环境是否初始化完成
static rt_bool_t g_utc_environment_ready = RT_FALSE;
//RTC时区是否配置
static rt_bool_t g_utc_environment_configured = RT_FALSE;
//公开函数声明
rt_err_t rtc_port_init(void);                       // 初始化UTC转换环境并查找RTC设备，不重设RTC时间。
rt_err_t rtc_port_read_utc(uint32_t *utc_seconds);  // 读取RTC的UTC候选秒值；输出指针不可为空，失败时不修改输出。
rt_err_t rtc_port_write_utc(uint32_t utc_seconds);  // 写入已校验的UTC秒值；成功仅表示SET调用成功，仍需时间服务读回确认。

//---------------------------------------------------------------公开函数---------------------------------------------------------------
/*
 * 简介：初始化项目RTC适配环境并查找设备。
 * 输入：无。
 * 返回值：成功返回RT_EOK；时区环境或设备检查失败返回负错误码。
 * 说明：仅由time线程调用；不重设RTC、不借备份寄存器、不重复open/close设备。
 */
rt_err_t rtc_port_init(void)
{
    static const uint32_t probe_values[] =
    {
        UINT32_C(946684800),   // 2000-01-01 00:00:00 UTC
        UINT32_C(4102444799)   // 2099-12-31 23:59:59 UTC
    };

    struct timezone utc_zone = {0};
    struct tm utc_calendar = {0};
    struct tm local_calendar = {0};
    time_t probe;
    time_t restored;
    rt_device_t device;
    size_t index;
    int result;

    if(g_utc_environment_ready == RT_FALSE)
    {
        // 检查time_t能否完整表示本项目支持的UTC上限。
        if ((int64_t)(time_t)UINT32_C(4102444799) != INT64_C(4102444799))
        {
            return -RT_ENOSYS;
        }

        // 全局时区只配置一次，后续重试仅重复验证或查找设备。
        if (g_utc_environment_configured == RT_FALSE)
        {
            // 配置newlib使用的时区，检查环境变量设置是否成功。
            result = setenv("TZ", "UTC0", 1);   //设置进程中的时区环境变量
            if (result != 0)
            {
                return -RT_ERROR;
            }

            tzset();

            // 配置SDK自定义localtime使用的时区，保持UTC零偏移。
            tz_set(&utc_zone);  //让newlib时间库读取`TZ`，更新内部时区规则

            g_utc_environment_configured = RT_TRUE;
        }

        // 验证支持范围首尾，避免只验证2000年而漏掉2038年后的问题。

        for (index = 0U;index < sizeof(probe_values) / sizeof(probe_values[0]);index++)
        {
            probe = (time_t)probe_values[index];

            // SDK转换函数未必填写struct tm的所有成员，先完整清零。
            memset(&utc_calendar, 0, sizeof(utc_calendar));
            memset(&local_calendar, 0, sizeof(local_calendar));

            if (gmtime_r(&probe, &utc_calendar) == RT_NULL)
            {
                return -RT_ERROR;
            }

            if (localtime_r(&probe, &local_calendar) == RT_NULL)
            {
                return -RT_ERROR;
            }

            // UTC环境下，两条转换路径的年月日时分秒应完全一致。
            if ((utc_calendar.tm_year != local_calendar.tm_year) ||
                (utc_calendar.tm_mon != local_calendar.tm_mon) ||
                (utc_calendar.tm_mday != local_calendar.tm_mday) ||
                (utc_calendar.tm_hour != local_calendar.tm_hour) ||
                (utc_calendar.tm_min != local_calendar.tm_min) ||
                (utc_calendar.tm_sec != local_calendar.tm_sec))
            {
                return -RT_ERROR;
            }

            // 明确关闭夏令时，再检查newlib的反向转换。
            utc_calendar.tm_isdst = 0;
            restored = mktime(&utc_calendar);
            if (restored != probe)
            {
                return -RT_ERROR;
            }

            local_calendar.tm_isdst = 0;
            restored = mktime(&local_calendar);
            if (restored != probe)
            {
                return -RT_ERROR;
            }
        }

        // 全部转换验证成功后，才标记UTC环境可用。
        g_utc_environment_ready = RT_TRUE;
    }

    // 获取SDK注册的设备句柄，不创建第二个RTC设备。
    device = rt_device_find("rtc");
    if (device == RT_NULL)
    {
        g_rtc_device = RT_NULL;
        return -RT_ERROR;
    }

    // 防止同名设备类型错误。
    if (device->type != RT_Device_Class_RTC)
    {
        g_rtc_device = RT_NULL;
        return -RT_EINVAL;
    }

    g_rtc_device = device;

    return RT_EOK;
}
/*
 * 简介：通过RT设备控制接口读取UTC候选值。
 * 输入：utc_seconds不可为空，输出缓冲归调用者所有。
 * 返回值：设备调用成功返回RT_EOK；失败返回负错误码且不修改输出。
 * 说明：仅time线程调用；使用rt_uint32_t临时值，不能直接传time_t指针。
 */
rt_err_t rtc_port_read_utc(uint32_t *utc_seconds)
{
    rt_uint32_t raw_seconds = 0U;
    rt_err_t result;

    if (utc_seconds == RT_NULL)
    {
        return -RT_EINVAL;
    }

    if(g_rtc_device==RT_NULL)
    {
        return -RT_ERROR;

    }
    // SDK控制接口按rt_uint32_t指针写入UTC秒值。
    result = rt_device_control(
        g_rtc_device,
        RT_DEVICE_CTRL_RTC_GET_TIME,
        &raw_seconds);

    if (result != RT_EOK)
    {
        return result;
    }

    // 成功后才提交候选值，年份范围和可信性由时间服务检查。
    *utc_seconds = (uint32_t)raw_seconds;

    return RT_EOK;
}

/*
 * 简介：通过RT设备控制接口写入UTC。
 * 输入：utc_seconds为已经由时间服务验证的UTC秒值。
 * 返回值：设备SET调用成功返回RT_EOK；
 *         设备未就绪返回-RT_ERROR；调用失败返回实际错误码。
 * 说明：仅time线程调用；成功不等于校时完成，读回确认由时间服务负责。
 */
rt_err_t rtc_port_write_utc(uint32_t utc_seconds)
{
    rt_uint32_t raw_seconds;
    rt_err_t result;

    // 未取得设备句柄时，不执行写入。
    if (g_rtc_device == RT_NULL)
    {
        return -RT_ERROR;
    }

    // 使用SDK要求的明确32位参数类型。
    raw_seconds = (rt_uint32_t)utc_seconds;

    result = rt_device_control(
        g_rtc_device,
        RT_DEVICE_CTRL_RTC_SET_TIME,
        &raw_seconds);

    // 原样返回设备结果，不在适配层修改trusted或系统tick。
    return result;
}
