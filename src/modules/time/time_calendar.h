#ifndef SMARTWATCH_TIME_CALENDAR_H
#define SMARTWATCH_TIME_CALENDAR_H
/*
 * 模块：time_calendar
 *
 * 简介：
 *     提供UTC日历校验、Unix时间戳转换和固定时区显示转换，
 *     为时间服务、表盘及后续BLE校时提供统一的纯计算接口。
 *
 * 主要功能：
 *     1. 严格检查年月日时分秒，包含公历闰年和当月天数。
 *     2. 将2000～2099年的合法UTC日历转换为Unix秒。
 *     3. 将UTC秒按指定偏移转换为显示日历，支持UTC±14小时。
 *
 * 设计说明：
 *     1. 不依赖RTOS、RTC设备或LVGL，不维护可变共享状态。
 *     2. 不调用localtime或mktime，不自动修正非法日期。
 *     3. 使用64位中间值计算，检查范围后再缩窄类型。
 *     4. 显示日期允许跨到2100年，不受校时输入年份上限限制。
 *     5. 输出缓冲由调用者持有，转换失败时不修改输出。
 *
 * 返回约定：
 *     成功返回TIME_CAL_OK，失败返回对应的日历错误码。
 */


#include "time_types.h"

// 纯日历转换模块的返回码。
typedef enum
{
    TIME_CAL_OK = 0,             // 校验或转换成功
    TIME_CAL_INVALID = -1,       // 空指针或日期字段非法
    TIME_CAL_RANGE = -2,         // 年份、偏移或转换结果超出支持范围
    TIME_CAL_NOT_IMPLEMENTED = -3 // 骨架尚未实现，不可当作成功
} time_calendar_error_t;

int time_calendar_validate(const time_calendar_t *calendar);    // 检查UTC校时日期是否合法，包含年份范围、闰年和当月天数。

int time_calendar_to_utc(const time_calendar_t *calendar,uint32_t *utc_seconds);// 将合法UTC日期转换为Unix秒；失败时不修改utc_seconds。

int time_calendar_from_utc(uint32_t utc_seconds,int16_t offset_minutes,time_calendar_t *calendar);// 将UTC秒按指定时区偏移转换为日历；失败时不修改calendar。

#endif
