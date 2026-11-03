#include "time_service.h"
#include "time_calendar.h"

#include <finsh.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <time.h>

/* 集中声明。 */
static rt_err_t time_diag_parse_u32(const char *text, uint32_t maximum, uint32_t *value);   //严格解析一项无符号十进制参数
static rt_bool_t time_diag_calendar_equal( const time_calendar_t *left,const time_calendar_t *right);   //逐字段比较两份日历
static void time_diag_check( const char *name,rt_bool_t condition,uint32_t *passed,uint32_t *failed);   //记录并打印一个构造值检查结果
static void time_diag_print_usage(void);        //打印时间诊断命令用法
static rt_err_t time_diag_print_status(void);   //打印时间服务快照与统计
static rt_err_t time_diag_selftest(void);       //运行日历转换和参数解析的构造值测试
static int time_diag(int argc, char **argv);    //解析时间诊断命令并调用时间服务接口


/*
 * 简介：严格解析一项无符号十进制参数。
 * 输入：text和value不可为空；maximum为允许上限；不保存输入指针。
 * 返回值：成功返回RT_EOK；空串、非法字符、溢出或超限返回-EINVAL。
 * 说明：允许前导零，不接受正负号、空白或部分解析；失败不修改输出。
 */
static rt_err_t time_diag_parse_u32(const char *text, uint32_t maximum,uint32_t *value)
{
    uint32_t accumulated = 0U;
    uint32_t digit;
    const char *cursor;

    if ((text == RT_NULL) ||(value == RT_NULL) || (text[0] == '\0'))
    {
        return -RT_EINVAL;
    }

    for (cursor = text; *cursor != '\0'; cursor++)
    {
        if ((*cursor < '0') || (*cursor > '9'))
        {
            return -RT_EINVAL;
        }

        digit = (uint32_t)(*cursor - '0');

        /*
         * 必须在乘10加digit之前检查。
         * 这种写法也正确处理maximum为0或小于digit的情况。
         */
        if ((accumulated > maximum / 10U) ||
            ((accumulated == maximum / 10U) &&(digit > maximum % 10U)))
        {
            return -RT_EINVAL;
        }

        accumulated = accumulated * 10U + digit;
    }

    *value = accumulated;
    return RT_EOK;
}


/*
 * 简介：逐字段比较两份日历。
 * 输入：left和right不可为空，均仅在调用期间读取。
 * 返回值：六个字段均相同返回RT_TRUE，否则RT_FALSE。
 * 说明：不使用memcmp，避免结构体填充字节影响比较。
 */
static rt_bool_t time_diag_calendar_equal(
    const time_calendar_t *left,
    const time_calendar_t *right)
{
    if ((left == RT_NULL) || (right == RT_NULL))
    {
        return RT_FALSE;
    }

    return ((left->year == right->year) &&
            (left->month == right->month) &&
            (left->day == right->day) &&
            (left->hour == right->hour) &&
            (left->minute == right->minute) &&
            (left->second == right->second))
           ? RT_TRUE : RT_FALSE;
}


/*
 * 简介：记录并打印一个构造值检查结果。
 * 输入：name和两个计数器指针不可为空；计数器归自检调用者所有。
 * 返回值：无。
 * 说明：FinSH上下文，仅更新测试计数，不访问服务、RTC或LVGL。
 */
static void time_diag_check( const char *name, rt_bool_t condition, uint32_t *passed, uint32_t *failed)
{
    if (condition == RT_TRUE)
    {
        (*passed)++;
    }
    else
    {
        (*failed)++;
    }

    rt_kprintf("[TIME][TEST] %s %s\n",
               (condition == RT_TRUE) ? "PASS" : "FAIL", name);
}


/*
 * 简介：打印时间诊断命令用法。
 * 输入：无。
 * 返回值：无。
 * 说明：FinSH上下文，分行输出，避免控制台格式化缓冲截断。
 */
static void time_diag_print_usage(void)
{
    rt_kprintf("time_diag status\n");
    rt_kprintf("time_diag set_utc YEAR MONTH DAY HOUR MINUTE SECOND\n");
    rt_kprintf("time_diag result ID\n");
    rt_kprintf("time_diag selftest\n");
    rt_kprintf("time_diag trace DURATION_SECONDS INTERVAL_SECONDS\n");
    rt_kprintf("time_diag trace 0 0\n");
    rt_kprintf("time_diag fault read|write|readback|missing COUNT\n");
    rt_kprintf("time_diag fault clear\n");
    rt_kprintf("set_utc uses UTC; local display uses UTC+08:00.\n");
    rt_kprintf("Queued requests must be checked with result ID.\n");
}


/*
 * 简介：打印时间服务快照与统计。
 * 输入：无。
 * 返回值：成功返回RT_EOK；获取副本或日历转换失败返回负错误码。
 * 说明：FinSH上下文，只读服务副本；不直接访问RTC或LVGL。
 *       last-good日历只作诊断，正常显示资格另行判断。
 */
static rt_err_t time_diag_print_status(void)
{
    static const char * const state_names[] =
    {
        "BOOT",
        "NO_DEVICE",
        "UNSET",
        "SYNCING",
        "READY",
        "READ_ERROR",
        "STALE",
        "INVALID",
        "UNCERTAIN"
    };

    time_snapshot_t snapshot;
    time_service_stats_t stats;
    time_calendar_t utc_calendar = {0};
    time_calendar_t local_calendar = {0};

    const char *state_name = "UNKNOWN";
    rt_bool_t normal_display;
    rt_err_t result;
    int calendar_result;

    result = time_service_get_snapshot(&snapshot);
    if (result != RT_EOK)
    {
        rt_kprintf("[TIME][STATUS][ERROR] snapshot=%d\n", (int)result);
        return result;
    }

    result = time_service_get_stats(&stats);
    if (result != RT_EOK)
    {
        rt_kprintf("[TIME][STATUS][ERROR] stats=%d\n", (int)result);
        return result;
    }

    if ((uint32_t)snapshot.state < (uint32_t)(sizeof(state_names) / sizeof(state_names[0])))
    {
        state_name = state_names[(uint32_t)snapshot.state];
    }

    normal_display =
        (snapshot.service_running &&
         snapshot.has_value &&
         snapshot.trusted &&
         snapshot.fresh &&
         (snapshot.state == TIME_CLOCK_READY))
        ? RT_TRUE : RT_FALSE;

    rt_kprintf("[TIME][STATUS] time_t_bytes=%u tick_hz=%u offset_min=%d\n",
               (unsigned int)sizeof(time_t),
               (unsigned int)RT_TICK_PER_SECOND,
               (int)TIME_DISPLAY_OFFSET_MINUTES);

    rt_kprintf("[TIME][STATUS] utc_range=%u..%u\n",
               (unsigned int)TIME_UTC_MIN_SECONDS,
               (unsigned int)TIME_UTC_MAX_SECONDS);

    rt_kprintf("[TIME][STATUS] state=%s run=%u has=%u trusted=%u fresh=%u\n",
               state_name,
               (unsigned int)snapshot.service_running,
               (unsigned int)snapshot.has_value,
               (unsigned int)snapshot.trusted,
               (unsigned int)snapshot.fresh);

    rt_kprintf("[TIME][STATUS] normal_display=%u source=%u utc=%u raw=%u\n",
               (unsigned int)normal_display,
               (unsigned int)snapshot.source,
               (unsigned int)snapshot.utc_seconds,
               (unsigned int)snapshot.candidate_utc);

    rt_kprintf("[TIME][STATUS] sample_tick=%u age_ms=%u publish_seq=%u\n",
               (unsigned int)snapshot.sample_tick,
               (unsigned int)snapshot.age_ms,
               (unsigned int)snapshot.publish_seq);

    rt_kprintf("[TIME][STATUS] gen=%u day=%u day_seq=%u change=%u/%u\n",
               (unsigned int)snapshot.calendar_generation,
               (unsigned int)snapshot.local_day_key,
               (unsigned int)snapshot.day_seq,
               (unsigned int)snapshot.change_seq,
               (unsigned int)snapshot.last_change);

    rt_kprintf("[TIME][STATUS] error_stage=%u error=%d\n",
               (unsigned int)snapshot.error_stage,
               (int)snapshot.last_error);

    rt_kprintf("[TIME][STATS] resource_init=%u/%u\n",
               (unsigned int)stats.init_attempts,
               (unsigned int)stats.init_failures);

    rt_kprintf("[TIME][STATS] post=%u accepted=%u rejected=%u invalid=%u\n",
               (unsigned int)stats.post_attempts,
               (unsigned int)stats.post_accepted,
               (unsigned int)stats.post_rejected,
               (unsigned int)stats.invalid_requests);

    rt_kprintf("[TIME][STATS] completed=%u worker_tick=%u\n",
               (unsigned int)stats.completed_requests,
               (unsigned int)stats.worker_completed_tick);

    rt_kprintf("[TIME][STATS] read=%u/%u/%u write=%u/%u/%u\n",
               (unsigned int)stats.read_attempts,
               (unsigned int)stats.read_success,
               (unsigned int)stats.read_failures,
               (unsigned int)stats.write_attempts,
               (unsigned int)stats.write_success,
               (unsigned int)stats.write_failures);

    rt_kprintf("[TIME][STATS] verify_fail=%u range_fail=%u jump=%u\n",
               (unsigned int)stats.verify_failures,
               (unsigned int)stats.range_failures,
               (unsigned int)stats.external_jumps);

    rt_kprintf("[TIME][STATS] skipped_poll=%u io_ticks=%u/%u\n",
               (unsigned int)stats.skipped_poll_periods,
               (unsigned int)stats.last_io_ticks,
               (unsigned int)stats.max_io_ticks);

    if (!snapshot.has_value)
    {
        rt_kprintf("[TIME][STATUS] no valid last-good calendar\n");
        return RT_EOK;
    }

    calendar_result = time_calendar_from_utc(
        snapshot.utc_seconds, 0, &utc_calendar);

    if (calendar_result != TIME_CAL_OK)
    {
        rt_kprintf("[TIME][STATUS][ERROR] UTC conversion=%d\n",
                   calendar_result);

        return (calendar_result == TIME_CAL_NOT_IMPLEMENTED)
               ? -RT_ENOSYS : -RT_ERROR;
    }

    calendar_result = time_calendar_from_utc(
        snapshot.utc_seconds,
        TIME_DISPLAY_OFFSET_MINUTES,
        &local_calendar);

    if (calendar_result != TIME_CAL_OK)
    {
        rt_kprintf("[TIME][STATUS][ERROR] local conversion=%d\n",
                   calendar_result);

        return (calendar_result == TIME_CAL_NOT_IMPLEMENTED)
               ? -RT_ENOSYS : -RT_ERROR;
    }

    rt_kprintf("[TIME][LAST_GOOD] UTC=%04u-%02u-%02u %02u:%02u:%02u\n",
               (unsigned int)utc_calendar.year,
               (unsigned int)utc_calendar.month,
               (unsigned int)utc_calendar.day,
               (unsigned int)utc_calendar.hour,
               (unsigned int)utc_calendar.minute,
               (unsigned int)utc_calendar.second);

    rt_kprintf("[TIME][LAST_GOOD] LOCAL=%04u-%02u-%02u %02u:%02u:%02u\n",
               (unsigned int)local_calendar.year,
               (unsigned int)local_calendar.month,
               (unsigned int)local_calendar.day,
               (unsigned int)local_calendar.hour,
               (unsigned int)local_calendar.minute,
               (unsigned int)local_calendar.second);

    if (normal_display == RT_FALSE)
    {
        rt_kprintf("[TIME][STATUS] last-good is diagnostic, not normal display\n");
    }

    return RT_EOK;
}


/*
 * 简介：运行日历转换和参数解析的构造值测试。
 * 输入：无。
 * 返回值：全部符合预期返回RT_EOK；任一失败返回-RT_ERROR。
 * 说明：FinSH上下文；本函数不访问RTC、不校时、不改tick。
 *       使用静态用例表和独立期望值；后台时间线程照常运行。
 */
static rt_err_t time_diag_selftest(void)
{
    typedef struct
    {
        const char *name;
        time_calendar_t calendar;
        int expected_result;
        uint32_t expected_utc;
    } calendar_case_t;

    typedef struct
    {
        const char *name;
        uint32_t utc;
        int16_t offset;
        int expected_result;
        time_calendar_t expected_calendar;
    } display_case_t;

    typedef struct
    {
        const char *name;
        const char *text;
        uint32_t maximum;
        rt_err_t expected_result;
        uint32_t expected_value;
    } parse_case_t;

    static const calendar_case_t calendar_cases[] =
    {
        { "utc_min",
          {2000, 1, 1, 0, 0, 0},
          TIME_CAL_OK, UINT32_C(946684800) },

        { "leap_2000",
          {2000, 2, 29, 12, 34, 56},
          TIME_CAL_OK, UINT32_C(951827696) },

        { "leap_2028",
          {2028, 2, 29, 23, 59, 59},
          TIME_CAL_OK, UINT32_C(1835481599) },

        { "before_2038_boundary",
          {2038, 1, 19, 3, 14, 7},
          TIME_CAL_OK, UINT32_C(2147483647) },

        { "after_2038_boundary",
          {2038, 1, 19, 3, 14, 8},
          TIME_CAL_OK, UINT32_C(2147483648) },

        { "utc_max",
          {2099, 12, 31, 23, 59, 59},
          TIME_CAL_OK, UINT32_C(4102444799) },

        { "invalid_leap_2027",
          {2027, 2, 29, 0, 0, 0},
          TIME_CAL_INVALID, 0U },

        { "invalid_april_31",
          {2026, 4, 31, 0, 0, 0},
          TIME_CAL_INVALID, 0U },

        { "month_zero",
          {2026, 0, 1, 0, 0, 0},
          TIME_CAL_INVALID, 0U },

        { "month_13",
          {2026, 13, 1, 0, 0, 0},
          TIME_CAL_INVALID, 0U },

        { "day_zero",
          {2026, 1, 0, 0, 0, 0},
          TIME_CAL_INVALID, 0U },

        { "hour_24",
          {2026, 1, 1, 24, 0, 0},
          TIME_CAL_INVALID, 0U },

        { "minute_60",
          {2026, 1, 1, 0, 60, 0},
          TIME_CAL_INVALID, 0U },

        { "second_60",
          {2026, 1, 1, 0, 0, 60},
          TIME_CAL_INVALID, 0U },

        { "year_1999",
          {1999, 12, 31, 23, 59, 59},
          TIME_CAL_RANGE, 0U },

        { "year_2100",
          {2100, 1, 1, 0, 0, 0},
          TIME_CAL_RANGE, 0U }
    };

    static const display_case_t display_cases[] =
    {
        { "display_next_month",
          UINT32_C(1793462400), 480, TIME_CAL_OK,
          {2026, 11, 1, 0, 0, 0} },

        { "display_leap_day",
          UINT32_C(1835366400), 480, TIME_CAL_OK,
          {2028, 2, 29, 0, 0, 0} },

        { "display_after_leap_day",
          UINT32_C(1835452800), 480, TIME_CAL_OK,
          {2028, 3, 1, 0, 0, 0} },

        { "display_next_year",
          UINT32_C(1798732800), 480, TIME_CAL_OK,
          {2027, 1, 1, 0, 0, 0} },

        { "display_2100",
          UINT32_C(4102444799), 480, TIME_CAL_OK,
          {2100, 1, 1, 7, 59, 59} },

        { "display_negative_offset",
          UINT32_C(946684800), -480, TIME_CAL_OK,
          {1999, 12, 31, 16, 0, 0} },

        { "offset_too_large",
          UINT32_C(946684800), 841, TIME_CAL_RANGE,
          {0} },

        { "offset_too_small",
          UINT32_C(946684800), -841, TIME_CAL_RANGE,
          {0} },

        { "display_before_epoch",
          0U, -1, TIME_CAL_RANGE,
          {0} },

        { "display_seconds_overflow",
          UINT32_MAX, 1, TIME_CAL_RANGE,
          {0} }
    };

    static const parse_case_t parse_cases[] =
    {
        { "parse_zero", "0", 0U, RT_EOK, 0U },
        { "parse_leading_zero", "00012", 12U, RT_EOK, 12U },
        { "parse_u32_max", "4294967295", UINT32_MAX,
          RT_EOK, UINT32_MAX },

        { "parse_empty", "", UINT32_MAX, -RT_EINVAL, 0U },
        { "parse_null", NULL, UINT32_MAX, -RT_EINVAL, 0U },
        { "parse_negative", "-1", UINT32_MAX, -RT_EINVAL, 0U },
        { "parse_positive_sign", "+1", UINT32_MAX, -RT_EINVAL, 0U },
        { "parse_space", "1 ", UINT32_MAX, -RT_EINVAL, 0U },
        { "parse_suffix", "12x", UINT32_MAX, -RT_EINVAL, 0U },
        { "parse_hex", "0x10", UINT32_MAX, -RT_EINVAL, 0U },
        { "parse_overflow", "4294967296", UINT32_MAX,
          -RT_EINVAL, 0U },
        { "parse_above_max", "12", 11U, -RT_EINVAL, 0U },
        { "parse_small_max", "9", 5U, -RT_EINVAL, 0U },
        { "parse_zero_max", "1", 0U, -RT_EINVAL, 0U }
    };

    static const time_calendar_t marker =
        {5555, 55, 55, 55, 55, 55};

    const uint32_t value_marker = UINT32_C(0xA5A5A5A5);

    time_calendar_t calendar;
    time_calendar_t restored;
    uint32_t utc_seconds;
    uint32_t parsed_value;
    uint32_t passed = 0U;
    uint32_t failed = 0U;

    size_t index;
    int validate_result;
    int convert_result;
    int roundtrip_result;
    rt_err_t parse_result;
    rt_bool_t ok;

    /*
     * 日历测试：
     * 不仅验证来回一致，还比较预先给定的UTC秒值，
     * 防止正反转换存在相同错误时相互抵消。
     */
    for (index = 0U;
         index < sizeof(calendar_cases) / sizeof(calendar_cases[0]);
         index++)
    {
        calendar = calendar_cases[index].calendar;
        utc_seconds = value_marker;
        restored = marker;
        roundtrip_result = TIME_CAL_NOT_IMPLEMENTED;

        validate_result = time_calendar_validate(&calendar);
        convert_result = time_calendar_to_utc(&calendar, &utc_seconds);

        ok = ((validate_result ==
               calendar_cases[index].expected_result) &&
              (convert_result ==
               calendar_cases[index].expected_result))
             ? RT_TRUE : RT_FALSE;

        if (calendar_cases[index].expected_result == TIME_CAL_OK)
        {
            if ((convert_result != TIME_CAL_OK) ||
                (utc_seconds != calendar_cases[index].expected_utc))
            {
                ok = RT_FALSE;
            }

            if (convert_result == TIME_CAL_OK)
            {
                roundtrip_result =
                    time_calendar_from_utc(utc_seconds, 0, &restored);

                if ((roundtrip_result != TIME_CAL_OK) ||
                    (time_diag_calendar_equal(
                        &calendar, &restored) == RT_FALSE))
                {
                    ok = RT_FALSE;
                }
            }
        }
        else if (utc_seconds != value_marker)
        {
            /* 转换失败必须保持输出不变。 */
            ok = RT_FALSE;
        }

        time_diag_check(calendar_cases[index].name,
                        ok, &passed, &failed);

        if (ok == RT_FALSE)
        {
            rt_kprintf("[TIME][TEST] validate=%d convert=%d roundtrip=%d\n",
                       validate_result,
                       convert_result,
                       roundtrip_result);

            rt_kprintf("[TIME][TEST] expected_rc=%d utc=%u expected_utc=%u\n",
                       calendar_cases[index].expected_result,
                       (unsigned int)utc_seconds,
                       (unsigned int)calendar_cases[index].expected_utc);
        }
    }

    /* 显示偏移、跨日/月/年，以及失败不修改输出。 */
    for (index = 0U;
         index < sizeof(display_cases) / sizeof(display_cases[0]);
         index++)
    {
        restored = marker;

        convert_result = time_calendar_from_utc(
            display_cases[index].utc,
            display_cases[index].offset,
            &restored);

        ok = (convert_result == display_cases[index].expected_result)
             ? RT_TRUE : RT_FALSE;

        if (display_cases[index].expected_result == TIME_CAL_OK)
        {
            if (time_diag_calendar_equal(
                    &restored,
                    &display_cases[index].expected_calendar) == RT_FALSE)
            {
                ok = RT_FALSE;
            }
        }
        else if (time_diag_calendar_equal(
                     &restored, &marker) == RT_FALSE)
        {
            ok = RT_FALSE;
        }

        time_diag_check(display_cases[index].name,
                        ok, &passed, &failed);

        if (ok == RT_FALSE)
        {
            rt_kprintf("[TIME][TEST] display_rc=%d expected_rc=%d\n",
                       convert_result,
                       display_cases[index].expected_result);

            rt_kprintf("[TIME][TEST] got=%04u-%02u-%02u %02u:%02u:%02u\n",
                       (unsigned int)restored.year,
                       (unsigned int)restored.month,
                       (unsigned int)restored.day,
                       (unsigned int)restored.hour,
                       (unsigned int)restored.minute,
                       (unsigned int)restored.second);
        }
    }

    /* 严格解析与溢出检查。 */
    for (index = 0U;
         index < sizeof(parse_cases) / sizeof(parse_cases[0]);
         index++)
    {
        parsed_value = value_marker;

        parse_result = time_diag_parse_u32(
            parse_cases[index].text,
            parse_cases[index].maximum,
            &parsed_value);

        ok = (parse_result == parse_cases[index].expected_result)
             ? RT_TRUE : RT_FALSE;

        if (parse_cases[index].expected_result == RT_EOK)
        {
            if (parsed_value != parse_cases[index].expected_value)
            {
                ok = RT_FALSE;
            }
        }
        else if (parsed_value != value_marker)
        {
            ok = RT_FALSE;
        }

        time_diag_check(parse_cases[index].name,
                        ok, &passed, &failed);
    }

    /* 空指针接口约定。 */
    time_diag_check(
        "validate_null",
        (time_calendar_validate(NULL) == TIME_CAL_INVALID)
        ? RT_TRUE : RT_FALSE,
        &passed, &failed);

    utc_seconds = value_marker;

    convert_result = time_calendar_to_utc(NULL, &utc_seconds);

    time_diag_check(
        "to_utc_null_input",
        ((convert_result == TIME_CAL_INVALID) &&
         (utc_seconds == value_marker))
        ? RT_TRUE : RT_FALSE,
        &passed, &failed);

    calendar = calendar_cases[0].calendar;

    time_diag_check(
        "to_utc_null_output",
        (time_calendar_to_utc(&calendar, NULL) == TIME_CAL_INVALID)
        ? RT_TRUE : RT_FALSE,
        &passed, &failed);

    time_diag_check(
        "from_utc_null_output",
        (time_calendar_from_utc(TIME_UTC_MIN_SECONDS, 0, NULL) ==
         TIME_CAL_INVALID)
        ? RT_TRUE : RT_FALSE,
        &passed, &failed);

    time_diag_check(
        "parse_null_output",
        (time_diag_parse_u32("1", 1U, NULL) == -RT_EINVAL)
        ? RT_TRUE : RT_FALSE,
        &passed, &failed);

    rt_kprintf("[TIME][SELFTEST] passed=%u failed=%u\n",
               (unsigned int)passed,
               (unsigned int)failed);

    return (failed == 0U) ? RT_EOK : -RT_ERROR;
}


/*
 * 简介：解析时间诊断命令并调用时间服务接口。
 * 输入：argc/argv由FinSH提供，字符串仅在调用期间有效，不保存指针。
 * 返回值：命令处理成功返回0；参数、受理或查询失败返回负错误码。
 * 说明：不直接读写RTC；异步请求只报告已受理和ID，结果须另行查询。
 */
static int time_diag(int argc, char **argv)
{
    static const uint32_t field_maximum[6] =
    {
        2099U, 12U, 31U, 23U, 59U, 59U
    };

    static const char * const field_names[6] =
    {
        "year", "month", "day", "hour", "minute", "second"
    };

    static const char * const result_names[] =
    {
        "PENDING", "RUNNING", "OK", "FAILED"
    };

    time_calendar_t calendar = {0};
    time_request_result_t request_result;

    uint32_t values[6] = {0};
    uint32_t utc_seconds;
    uint32_t request_id = 0U;
    uint32_t duration;
    uint32_t interval;
    uint32_t count = 0U;

    time_test_fault_t fault = TIME_FAULT_NONE;
    const char *result_name;
    int index;
    int calendar_result;
    rt_err_t result;

    if ((argc < 1) || (argv == RT_NULL))
    {
        return -RT_EINVAL;
    }

    for (index = 0; index < argc; index++)
    {
        if (argv[index] == RT_NULL)
        {
            return -RT_EINVAL;
        }
    }

    if ((argc == 1) ||
        ((argc == 2) && (strcmp(argv[1], "help") == 0)))
    {
        time_diag_print_usage();
        return 0;
    }

    if (strcmp(argv[1], "status") == 0)
    {
        if (argc != 2)
        {
            goto invalid_arguments;
        }

        return (int)time_diag_print_status();
    }

    if (strcmp(argv[1], "selftest") == 0)
    {
        if (argc != 2)
        {
            goto invalid_arguments;
        }

        return (int)time_diag_selftest();
    }

    if (strcmp(argv[1], "set_utc") == 0)
    {
        if (argc != 8)
        {
            goto invalid_arguments;
        }

        /*
         * 先解析到uint32_t并检查各字段上限，
         * 再缩窄到uint16_t/uint8_t，避免截断后变成合法小数值。
         */
        for (index = 0; index < 6; index++)
        {
            result = time_diag_parse_u32(
                argv[index + 2],
                field_maximum[index],
                &values[index]);

            if (result != RT_EOK)
            {
                rt_kprintf("[TIME][CMD][ERROR] invalid %s\n",
                           field_names[index]);
                return (int)result;
            }
        }

        calendar.year = (uint16_t)values[0];
        calendar.month = (uint8_t)values[1];
        calendar.day = (uint8_t)values[2];
        calendar.hour = (uint8_t)values[3];
        calendar.minute = (uint8_t)values[4];
        calendar.second = (uint8_t)values[5];

        /*
         * to_utc内部执行严格日历校验：
         * 包括年份下限、月/日下限和当月合法天数。
         */
        calendar_result = time_calendar_to_utc(
            &calendar, &utc_seconds);

        if (calendar_result != TIME_CAL_OK)
        {
            rt_kprintf("[TIME][CMD][ERROR] invalid UTC calendar, code=%d\n",
                       calendar_result);

            return (calendar_result == TIME_CAL_NOT_IMPLEMENTED)
                   ? -RT_ENOSYS : -RT_EINVAL;
        }

        result = time_service_request_set_utc(
            utc_seconds,
            TIME_SOURCE_SHELL,
            &request_id);

        if (result != RT_EOK)
        {
            goto request_rejected;
        }

        rt_kprintf("[TIME][CMD] SET_UTC queued id=%u utc=%u\n",
                   (unsigned int)request_id,
                   (unsigned int)utc_seconds);

        return 0;
    }

    if (strcmp(argv[1], "result") == 0)
    {
        if (argc != 3)
        {
            goto invalid_arguments;
        }

        result = time_diag_parse_u32(
            argv[2], UINT32_MAX, &request_id);

        if ((result != RT_EOK) || (request_id == 0U))
        {
            rt_kprintf("[TIME][CMD][ERROR] invalid request ID\n");
            return -RT_EINVAL;
        }

        result = time_service_get_result(
            request_id, &request_result);

        if (result != RT_EOK)
        {
            rt_kprintf("[TIME][RESULT][ERROR] id=%u lookup=%d\n",
                       (unsigned int)request_id,
                       (int)result);
            return (int)result;
        }

        result_name = "UNKNOWN";

        if ((uint32_t)request_result.state <
            (uint32_t)(sizeof(result_names) / sizeof(result_names[0])))
        {
            result_name = result_names[(uint32_t)request_result.state];
        }

        rt_kprintf("[TIME][RESULT] id=%u kind=%u state=%s error=%d\n",
                   (unsigned int)request_result.request_id,
                   (unsigned int)request_result.kind,
                   result_name,
                   (int)request_result.error);

        if ((request_result.state == TIME_RESULT_OK) ||
            (request_result.state == TIME_RESULT_FAILED))
        {
            rt_kprintf("[TIME][RESULT] gen=%u finished_tick=%u readback=%u\n",
                       (unsigned int)request_result.calendar_generation,
                       (unsigned int)request_result.finished_tick,
                       (unsigned int)request_result.readback_available);

            if (request_result.readback_available)
            {
                rt_kprintf("[TIME][RESULT] readback_utc=%u\n",
                           (unsigned int)request_result.readback_utc);
            }
        }

        /*
         * 返回0表示查询命令执行成功。
         * 被查询请求是否成功，由打印的state/error表达。
         */
        return 0;
    }

    if (strcmp(argv[1], "trace") == 0)
    {
        if (argc != 4)
        {
            goto invalid_arguments;
        }

        result = time_diag_parse_u32(
            argv[2], UINT32_MAX, &duration);

        if (result != RT_EOK)
        {
            goto invalid_arguments;
        }

        result = time_diag_parse_u32(
            argv[3], UINT32_MAX, &interval);

        if (result != RT_EOK)
        {
            goto invalid_arguments;
        }

        /*
         * 时长、间隔及tick跨度的业务限制由服务统一检查，
         * 不在Shell重复维护服务私有宏。
         */
        result = time_service_request_trace(
            duration, interval, &request_id);

        if (result != RT_EOK)
        {
            goto request_rejected;
        }

        rt_kprintf("[TIME][CMD] TRACE queued id=%u duration=%u interval=%u\n",
                   (unsigned int)request_id,
                   (unsigned int)duration,
                   (unsigned int)interval);

        return 0;
    }

    if (strcmp(argv[1], "fault") == 0)
    {
        if ((argc == 3) && (strcmp(argv[2], "clear") == 0))
        {
            fault = TIME_FAULT_NONE;
            count = 0U;
        }
        else
        {
            if (argc != 4)
            {
                goto invalid_arguments;
            }

            if (strcmp(argv[2], "read") == 0)
            {
                fault = TIME_FAULT_READ;
            }
            else if (strcmp(argv[2], "write") == 0)
            {
                fault = TIME_FAULT_WRITE;
            }
            else if (strcmp(argv[2], "readback") == 0)
            {
                fault = TIME_FAULT_READBACK;
            }
            else if (strcmp(argv[2], "missing") == 0)
            {
                fault = TIME_FAULT_MISSING;
            }
            else
            {
                goto invalid_arguments;
            }

            result = time_diag_parse_u32(
                argv[3], UINT32_MAX, &count);

            if (result != RT_EOK)
            {
                goto invalid_arguments;
            }
        }

        result = time_service_test_inject(
            fault, count, &request_id);

        if (result != RT_EOK)
        {
            goto request_rejected;
        }

        rt_kprintf("[TIME][CMD] SOFTWARE_FAULT queued id=%u type=%u count=%u\n",
                   (unsigned int)request_id,
                   (unsigned int)fault,
                   (unsigned int)count);

        return 0;
    }

invalid_arguments:

    rt_kprintf("[TIME][CMD][ERROR] invalid command or arguments\n");
    time_diag_print_usage();
    return -RT_EINVAL;

request_rejected:

    rt_kprintf("[TIME][CMD][REJECT] error=%d\n", (int)result);
    return (int)result;
}

MSH_CMD_EXPORT(time_diag, RTC time service diagnostics);