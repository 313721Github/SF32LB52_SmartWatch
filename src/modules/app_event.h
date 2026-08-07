#ifndef APP_EVENT_H
#define APP_EVENT_H

#include "rtthread.h"

//应用事件来源
typedef enum
{
    APP_EVENT_SOURCE_KEY1=0,
    APP_EVENT_SOURCE_KEY2,
    APP_EVENT_SOURCE_SYSTEM,
    APP_EVENT_SOURCE_TEST,
    APP_EVENT_SOURCE_COUNT

} app_event_source_t;

//应用事件类型
typedef enum
{
    APP_EVENT_NONE = 0,
    APP_EVENT_KEY_DOWN,
    APP_EVENT_KEY_UP,
    APP_EVENT_KEY_SHORT,
    APP_EVENT_KEY_LONG,
    APP_EVENT_DIAG_REQUEST,
    APP_EVENT_INDICATOR_SELF_TEST,
    APP_EVENT_TYPE_COUNT

}app_event_type_t;

//事件结构
typedef struct
{
    app_event_type_t type;      //发生了什么；
    app_event_source_t source;  //来自KEY1、KEY2还是系统；
    rt_tick_t  tick;            //发生时间；
    rt_uint32_t value;          //按键事件中保存duration_ms。

}app_event_t;



#endif