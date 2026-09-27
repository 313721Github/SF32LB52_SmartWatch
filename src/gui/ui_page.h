#ifndef UI_PAGE_H
#define UI_PAGE_H
#include "rtthread.h"
#include "ui_event.h"

//创建ui界面
rt_err_t ui_page_create(void);
//ui事件处理函数
rt_err_t ui_page_handle_event(const ui_event_t *event);
#endif
