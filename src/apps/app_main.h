#ifndef APP_MAIN_H
#define APP_MAIN_H
#include "rtthread.h"
#include "app_event.h"

#ifdef RT_USING_FINSH
#include <finsh.h>
#endif
rt_err_t app_main_init(void);
rt_err_t app_event_post(const app_event_t *event);


#endif