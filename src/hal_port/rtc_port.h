#ifndef SMARTWATCH_RTC_PORT_H
#define SMARTWATCH_RTC_PORT_H

#include <stdint.h>
#include "rtthread.h"

rt_err_t rtc_port_init(void);                       // 初始化UTC转换环境并查找RTC设备，不重设RTC时间。
rt_err_t rtc_port_read_utc(uint32_t *utc_seconds);  // 读取RTC的UTC候选秒值；输出指针不可为空，失败时不修改输出。
rt_err_t rtc_port_write_utc(uint32_t utc_seconds);  // 写入已校验的UTC秒值；成功仅表示SET调用成功，仍需时间服务读回确认。

#endif
