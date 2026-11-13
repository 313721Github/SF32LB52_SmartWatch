#ifndef UI_EVENT_H
#define UI_EVENT_H

#include "rtthread.h"

//ui事件类型
typedef enum
{
    UI_ACTION_NONE = 0,
    UI_ACTION_HOME,
    UI_ACTION_BACK = 3,
    UI_ACTION_REFRESH_STATUS,
    UI_ACTION_TOUCH_DIAG_ENABLE,
    UI_ACTION_TOUCH_DIAG_DISABLE,
    UI_ACTION_OPEN_MENU,
    UI_ACTION_SWIPE_LEFT,
    UI_ACTION_SWIPE_RIGHT,
    UI_ACTION_OPEN_DETAIL,
    UI_ACTION_NOTICE_READY,  /* value为非零请求ID */
    UI_ACTION_NOTICE_CLOSE,  /* value为按下关闭按钮时捕获的显示ID */
    UI_ACTION_COUNT
} ui_action_t;

//ui事件来源
typedef enum
{
    UI_EVENT_SOURCE_APP = 0,
    UI_EVENT_SOURCE_KEY,
    UI_EVENT_SOURCE_SYSTEM,
    UI_EVENT_SOURCE_TEST,
    UI_EVENT_SOURCE_TOUCH,
    UI_EVENT_SOURCE_COUNT
} ui_event_source_t;

//ui事件
typedef struct
{
    ui_action_t type;
    ui_event_source_t source;
    rt_tick_t tick;
    rt_uint32_t value;
} ui_event_t;


#endif
