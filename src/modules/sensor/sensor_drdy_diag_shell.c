#include "rtthread.h"
#include "sensor_service.h"
#include "sensor_drdy_port.h"

#ifdef RT_USING_FINSH

#include "finsh.h"
#include <string.h>

/*
 * 模式切换命令最多等待1秒。
 * 命令线程只等待sensor线程处理结果，不执行I2C或GPIO配置。
 */
#define SENSOR_IMU_MODE_WAIT_TIMEOUT_MS        1000U
#define SENSOR_IMU_MODE_WAIT_INTERVAL_MS         10U


static const char *sensor_imu_trigger_mode_name(sensor_imu_trigger_mode_t mode);

static void sensor_imu_trigger_print(const sensor_imu_trigger_status_t *status,const sensor_drdy_port_stats_t *port_stats,rt_err_t port_result);

static int sensor_imu_trigger_show(int argc,char **argv);

static int sensor_imu_mode(int argc,char **argv);
/*
 * 简介：将IMU触发模式转换为固定诊断字符串。
 * 输入：mode为触发模式。
 * 返回值：POLLING、DRDY或UNKNOWN只读字符串。
 * 说明：只用于FinSH输出，不参与模式判断。
 */
static const char *sensor_imu_trigger_mode_name(sensor_imu_trigger_mode_t mode)
{
    switch (mode)
    {
    case SENSOR_IMU_TRIGGER_POLLING:
        return "POLLING";

    case SENSOR_IMU_TRIGGER_DRDY:
        return "DRDY";

    default:
        return "UNKNOWN";
    }
}

/*
 * 简介：打印一份完整的IMU触发状态。
 * 输入：status为服务状态；port_stats为PA31端口统计；
 *       port_result为端口统计查询结果。
 * 返回值：无。
 * 说明：服务统计和GPIO统计来自相邻但不完全相同的观察时刻，
 *       不能将它们解释成原子一致快照。
 */
static void sensor_imu_trigger_print(const sensor_imu_trigger_status_t *status,const sensor_drdy_port_stats_t *port_stats,rt_err_t port_result)
{
    if (status == RT_NULL)
    {
        return;
    }

    rt_kprintf(
        "[IMU][TRIGGER] requested=%s active=%s "
        "armed=%u pending=%u request_seq=%u "
        "applied_seq=%u last_error=%d\n",
        sensor_imu_trigger_mode_name(
            status->requested_mode),
        sensor_imu_trigger_mode_name(
            status->active_mode),
        (unsigned int)status->drdy_armed,
        (unsigned int)status->drdy_pending,
        (unsigned int)status->request_seq,
        (unsigned int)status->applied_seq,
        (int)status->last_switch_error);

    rt_kprintf(
        "[IMU][TRIGGER] drdy_samples=%u "
        "fallback=%u coalesced=%u\n",
        (unsigned int)status->drdy_sample_count,
        (unsigned int)status->fallback_count,
        (unsigned int)status->coalesced_count);

    rt_kprintf(
        "[IMU][TRIGGER] last_assert=%u "
        "last_read=%u max_latency=%u\n",
        (unsigned int)status->last_assert_tick,
        (unsigned int)status->last_read_start_tick,
        (unsigned int)status->max_assert_to_read_ticks);

    if ((port_result == RT_EOK) &&
        (port_stats != RT_NULL))
    {
        rt_kprintf(
            "[IMU][DRDY_PORT] irq=%u assert=%u "
            "release=%u notify_ok=%u notify_fail=%u\n",
            (unsigned int)port_stats->irq_count,
            (unsigned int)port_stats->assert_count,
            (unsigned int)port_stats->release_count,
            (unsigned int)port_stats->notify_success_count,
            (unsigned int)port_stats->notify_failure_count);

        rt_kprintf(
            "[IMU][DRDY_PORT] last_assert=%u "
            "last_release=%u last_level=%u "
            "enabled=%u\n",
            (unsigned int)port_stats->last_assert_tick,
            (unsigned int)port_stats->last_release_tick,
            (unsigned int)port_stats->last_level,
            (unsigned int)sensor_drdy_port_is_enabled());
    }
    else
    {
        /*
         * 默认POLLING启动时PA31端口可能尚未初始化。
         * 这是正常状态，不能误判为传感器故障。
         */
        rt_kprintf(
            "[IMU][DRDY_PORT] unavailable result=%d "
            "enabled=%u\n",
            (int)port_result,
            (unsigned int)sensor_drdy_port_is_enabled());
    }
}

/*
 * 简介：显示当前IMU触发模式和PA31中断统计。
 * 用法：sensor_imu_trigger_show
 * 返回值：成功返回0，服务状态读取失败返回-1。
 * 说明：只读取复制出来的状态，不访问I2C或修改模式。
 */
static int sensor_imu_trigger_show(int argc,char **argv)
{
    sensor_imu_trigger_status_t status = {0};
    sensor_drdy_port_stats_t port_stats = {0};
    rt_err_t result;
    rt_err_t port_result;

    (void)argv;

    if (argc != 1)
    {
        rt_kprintf(
            "usage: sensor_imu_trigger_show\n");

        return -1;
    }

    result =
        sensor_service_get_imu_trigger_status(
            &status);

    if (result != RT_EOK)
    {
        rt_kprintf(
            "[IMU][TRIGGER] show failed result=%d\n",
            (int)result);

        return -1;
    }

    port_result =
        sensor_drdy_port_get_stats(&port_stats);

    sensor_imu_trigger_print(
        &status,
        &port_stats,
        port_result);

    return 0;
}

/*
 * 简介：请求统一服务切换IMU触发模式并有限等待处理结果。
 * 用法：sensor_imu_mode polling
 *       sensor_imu_mode drdy
 * 返回值：切换成功返回0；参数、提交、执行或等待失败返回-1。
 * 说明：
 * 1. 本命令只提交模式请求；
 * 2. I2C、INT1和PA31配置仍由唯一sensor线程完成；
 * 3. 等待期间每10 ms查询一次状态，最多等待1秒；
 * 4. applied_seq匹配后才能依据last_switch_error判断结果。
 */
static int sensor_imu_mode(int argc,char **argv)
{
    sensor_imu_trigger_status_t status = {0};
    sensor_drdy_port_stats_t port_stats = {0};
    sensor_imu_trigger_mode_t requested_mode;

    rt_uint32_t target_seq;
    rt_tick_t start_tick;
    rt_tick_t timeout_ticks;

    rt_err_t result;
    rt_err_t port_result;

    if (argc != 2)
    {
        rt_kprintf(
            "usage: sensor_imu_mode polling|drdy\n");

        return -1;
    }

    if ((strcmp(argv[1], "polling") == 0) ||
        (strcmp(argv[1], "poll") == 0))
    {
        requested_mode =
            SENSOR_IMU_TRIGGER_POLLING;
    }
    else if (strcmp(argv[1], "drdy") == 0)
    {
        requested_mode =
            SENSOR_IMU_TRIGGER_DRDY;
    }
    else
    {
        rt_kprintf(
            "invalid mode: %s\n",
            argv[1]);

        rt_kprintf(
            "usage: sensor_imu_mode polling|drdy\n");

        return -1;
    }

    result = sensor_service_request_imu_trigger_mode(requested_mode);

    if (result != RT_EOK)
    {
        rt_kprintf(
            "[IMU][MODE] request failed "
            "mode=%s result=%d\n",
            sensor_imu_trigger_mode_name(
                requested_mode),
            (int)result);

        return -1;
    }

    /*
     * 请求提交后读取本次request_seq。
     * 即使sensor线程已经快速处理完成，applied_seq也会匹配。
     */
    result =sensor_service_get_imu_trigger_status(&status);

    if (result != RT_EOK)
    {
        rt_kprintf(
            "[IMU][MODE] status failed result=%d\n",
            (int)result);

        return -1;
    }

    target_seq = status.request_seq;
    start_tick = rt_tick_get();

    timeout_ticks = rt_tick_from_millisecond(
        SENSOR_IMU_MODE_WAIT_TIMEOUT_MS);

    if (timeout_ticks == 0U)
    {
        timeout_ticks = 1U;
    }

    while ((rt_tick_t)(rt_tick_get() - start_tick) <
           timeout_ticks)
    {
        result =
            sensor_service_get_imu_trigger_status(
                &status);

        if (result != RT_EOK)
        {
            rt_kprintf(
                "[IMU][MODE] status failed "
                "result=%d\n",
                (int)result);

            return -1;
        }

        if (status.applied_seq == target_seq)
        {
            port_result =
                sensor_drdy_port_get_stats(
                    &port_stats);

            sensor_imu_trigger_print(
                &status,
                &port_stats,
                port_result);

            if (status.last_switch_error != RT_EOK)
            {
                rt_kprintf(
                    "[IMU][MODE] apply failed "
                    "seq=%u result=%d\n",
                    (unsigned int)target_seq,
                    (int)status.last_switch_error);

                return -1;
            }

            rt_kprintf(
                "[IMU][MODE] apply success "
                "seq=%u active=%s\n",
                (unsigned int)target_seq,
                sensor_imu_trigger_mode_name(
                    status.active_mode));

            return 0;
        }

        rt_thread_mdelay(
            SENSOR_IMU_MODE_WAIT_INTERVAL_MS);
    }

    /*
     * 超时后再读取一次，用于留下最终状态证据。
     */
    result =
        sensor_service_get_imu_trigger_status(
            &status);

    if (result == RT_EOK)
    {
        port_result =
            sensor_drdy_port_get_stats(
                &port_stats);

        sensor_imu_trigger_print(
            &status,
            &port_stats,
            port_result);
    }

    rt_kprintf(
        "[IMU][MODE] apply timeout "
        "target_seq=%u\n",
        (unsigned int)target_seq);

    return -1;
}

MSH_CMD_EXPORT(
    sensor_imu_trigger_show,
    show IMU polling DRDY mode and PA31 statistics);

MSH_CMD_EXPORT(
    sensor_imu_mode,
    switch IMU mode: sensor_imu_mode polling or drdy);

#endif /* RT_USING_FINSH */
