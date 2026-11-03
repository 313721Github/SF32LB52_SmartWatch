#include "time_calendar.h"
#include <stddef.h>

//公开函数声明
int time_calendar_validate(const time_calendar_t *calendar);    // 检查UTC校时日期是否合法，包含年份范围、闰年和当月天数。

int time_calendar_to_utc(const time_calendar_t *calendar,uint32_t *utc_seconds);// 将合法UTC日期转换为Unix秒；失败时不修改utc_seconds。

int time_calendar_from_utc(uint32_t utc_seconds,int16_t offset_minutes,time_calendar_t *calendar);// 将UTC秒按指定时区偏移转换为日历；失败时不修改calendar。

//私有函数声明
static uint8_t time_calendar_month_days(uint16_t year, uint8_t month);  //取得指定年月的合法天数

//---------------------------------------------------------------公开函数---------------------------------------------------------------
/*
 * 简介：严格检查一份UTC校时日历。
 * 输入：calendar不可为空，归调用者所有，仅在调用期间读取。
 * 返回值：合法返回TIME_CAL_OK；字段非法返回INVALID；年份超范围返回RANGE。
 * 说明：接受2000～2099年；秒为0～59，不支持闰秒；失败不修改输入。
 */
int time_calendar_validate(const time_calendar_t *calendar)
{
    uint8_t month_days;

    if (calendar == NULL)
    {
        return TIME_CAL_INVALID;
    }

    // 年份超出业务支持范围，单独返回范围错误。
    if ((calendar->year < 2000U) || (calendar->year > 2099U))
    {
        return TIME_CAL_RANGE;
    }

    // 检查月份及日内时间，不接受24点或闰秒。
    if ((calendar->month < 1U) || (calendar->month > 12U) ||
        (calendar->hour > 23U) ||
        (calendar->minute > 59U) ||
        (calendar->second > 59U))
    {
        return TIME_CAL_INVALID;
    }
    // 根据年份和月份取得实际天数，包含闰年二月。
    month_days = time_calendar_month_days(calendar->year,calendar->month);
    // 日期从1开始，不能超过当月天数。
    if ((month_days == 0U) ||(calendar->day < 1U) ||(calendar->day > month_days))
    {
        return TIME_CAL_INVALID;
    }

    return TIME_CAL_OK;
}

/*
 * 简介：将合法UTC日历转换为Unix秒。
 * 输入：calendar与utc_seconds均不可为空；输出缓冲由调用者持有。
 * 返回值：成功返回TIME_CAL_OK；失败返回日历错误码且不修改输出。
 * 说明：先校验，再用64位中间值累计自1970年以来的天数和秒数。
 */
int time_calendar_to_utc(const time_calendar_t *calendar,uint32_t *utc_seconds)
{
    uint64_t days = 0U;
    uint64_t seconds;
    uint16_t year;
    uint8_t month;
    bool leap;
    int result;

    if ((calendar == NULL) || (utc_seconds == NULL))
    {
        return TIME_CAL_INVALID;
    }

    // 先拒绝非法日期，避免后续计算把错误输入变成正常时间。
    result = time_calendar_validate(calendar);
    if (result != TIME_CAL_OK)
    {
        return result;
    }

    // 累计1970年至目标年前一年的完整天数。
    for (year = 1970U; year < calendar->year; year++)
    {
        leap = (((year % 4U) == 0U) && ((year % 100U) != 0U)) ||
               ((year % 400U) == 0U);

        if (leap)
        {
            days += 366U;
        }
        else
        {
            days += 365U;
        }
    }

    // 累计目标年中，目标月份之前已经完整经过的月份。
    for (month = 1U; month < calendar->month; month++)
    {
        days += time_calendar_month_days(calendar->year, month);
    }

    // 每月1日零点尚未经过当天，因此需要减1。
    days += (uint64_t)(calendar->day - 1U);

    // 使用64位运算，避免在检查范围之前发生中间值溢出。
    seconds = days * UINT64_C(86400) +  (uint64_t)calendar->hour * UINT64_C(3600) +
              (uint64_t)calendar->minute * UINT64_C(60) +(uint64_t)calendar->second;

    // 检查业务时间范围，确认后才能缩窄为uint32_t。
    if ((seconds < (uint64_t)TIME_UTC_MIN_SECONDS) ||
        (seconds > (uint64_t)TIME_UTC_MAX_SECONDS))
    {
        return TIME_CAL_RANGE;
    }

    *utc_seconds=(uint32_t)seconds;

    return TIME_CAL_OK;
}

/*
 * 简介：将UTC秒和固定显示偏移转换为日历。
 * 输入：calendar不可为空；offset_minutes限定为-840～840分钟。
 * 返回值：成功返回TIME_CAL_OK；参数或结果超范围返回错误且不修改输出。
 * 说明：纯计算；偏移后的秒值限定为0～UINT32_MAX，显示年份不限制为2099年。
 */
int time_calendar_from_utc(uint32_t utc_seconds,int16_t offset_minutes, time_calendar_t *calendar)
{
    int64_t shifted_seconds;
    uint64_t remaining_days;
    uint32_t seconds_of_day;
    uint16_t year = 1970U;
    uint8_t month = 1U;
    uint16_t days_in_year;
    uint8_t days_in_month;
    bool leap;
    time_calendar_t value = {0};

    if (calendar == NULL)
    {
        return TIME_CAL_INVALID;
    }

    // 检查显示时区偏移，最大允许UTC±14小时。
    if ((offset_minutes < -840) || (offset_minutes > 840))
    {
        return TIME_CAL_RANGE;
    }

    // 先提升为有符号64位，再进行偏移运算，避免负偏移变成无符号大数。
    shifted_seconds = (int64_t)utc_seconds + (int64_t)offset_minutes * INT64_C(60);

    // 本函数不处理1970年之前或超过约定秒值范围的显示结果。
    if ((shifted_seconds < 0) || (shifted_seconds > (int64_t)UINT32_MAX))
    {
        return TIME_CAL_RANGE;
    }

    // 分离完整天数和当天已经经过的秒数。
    remaining_days = (uint64_t)shifted_seconds / UINT64_C(86400);
    seconds_of_day = (uint32_t)((uint64_t)shifted_seconds % UINT64_C(86400));

    // 逐年扣除完整天数，剩余不足一年时即找到目标年份。
    for (;;)
    {
        leap = (((year % 4U) == 0U) && ((year % 100U) != 0U)) ||
               ((year % 400U) == 0U);

        days_in_year = leap ? 366U : 365U;

        if (remaining_days < days_in_year)
        {
            break;
        }

        remaining_days -= days_in_year;
        year++;
    }

    // 逐月扣除完整天数，剩余不足一个月时即找到目标月份。
    while (month <= 12U)
    {
        days_in_month = time_calendar_month_days(year, month);
        if (days_in_month == 0U)
        {
            return TIME_CAL_INVALID;
        }

        if (remaining_days < days_in_month)
        {
            break;
        }

        remaining_days -= days_in_month;
        month++;
    }

    if (month > 12U)
    {
        return TIME_CAL_RANGE;
    }

    // 剩余天数从0开始，日历日期从1开始。
    value.year = year;
    value.month = month;
    value.day = (uint8_t)(remaining_days + 1U);

    // 将日内秒数拆成时、分、秒。
    value.hour = (uint8_t)(seconds_of_day / 3600U);
    value.minute = (uint8_t)((seconds_of_day % 3600U) / 60U);
    value.second = (uint8_t)(seconds_of_day % 60U);

    // 所有计算成功后一次性提交，失败时保持调用者输出不变。
    *calendar = value;

    return TIME_CAL_OK;
}
//---------------------------------------------------------------私有函数---------------------------------------------------------------
/*
 * 简介：取得指定年月的合法天数。
 * 输入：year为公历年份；month为1～12；均为值参数。
 * 返回值：合法月份返回28～31；非法或尚未实现返回0。
 * 说明：纯计算，不访问设备、不使用共享状态；包含公历闰年规则。
 */
static uint8_t time_calendar_month_days(uint16_t year, uint8_t month)
{
    static const uint8_t normal_days[12] =
    {
        31, 28, 31, 30, 31, 30,
        31, 31, 30, 31, 30, 31
    };
    bool leap;

    if(month<1U||month>12U)
    {
        return 0U;
    }
    // 能被4整除但不能被100整除，或者能被400整除。
    leap = (((year % 4U) == 0U) && ((year % 100U) != 0U)) ||
           ((year % 400U) == 0U);

    if(month==2&&leap)
    {
        return 29U;
    }

    return normal_days[month-1U];
}