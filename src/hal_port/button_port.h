#ifndef BUTTON_PORT_H
#define BUTTON_PORT_H

#include "rtthread.h"

//按键编号
typedef enum
{
    BUTTON_PORT_KEY1=0,
    BUTTON_PORT_KEY2, 
    BUTTON_PORT_KEY_COUNT
}button_port_key_t;


//按键事件定义
typedef enum
{
    BUTTON_EVENT_DOWN=0,
    BUTTON_EVENT_UP,
    BUTTON_EVENT_SHORT,
    BUTTON_EVENT_LONG,
    BUTTON_EVENT_COUNT
}  button_event_type_t;

/*
哪个按键：KEY1/KEY2
发生了什么：DOWN/UP/SHORT/LONG
何时发生：tick
持续多久：duration_ms
*/ 
typedef struct 
{
    /* data */
    button_port_key_t key;
    button_event_type_t type;
    rt_tick_t tick;
    rt_uint32_t duration_ms;
}button_event_t;


/* 配置KEY1/KEY2为输入，并记录启动时原始电平 */
rt_err_t button_port_init(void);

/* 读取未消抖的原始电平 */
rt_err_t button_port_read_raw(button_port_key_t key,rt_uint8_t *level);

typedef rt_err_t (*button_event_handler_t)(const button_event_t *event,void* user_data);
rt_err_t button_port_register_handler(button_event_handler_t handler, void* userdata);

#endif
