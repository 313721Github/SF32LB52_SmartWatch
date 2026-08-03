#ifndef BUTTON_PORT_H
#define BUTTON_PORT_H

#include "rtthread.h"

typedef enum
{
    BUTTON_PORT_KEY1=0,
    BUTTON_PORT_KEY2, 
    BUTTON_PORT_KEY_COUNT
}button_port_key_t;

/* 配置KEY1/KEY2为输入，并记录启动时原始电平 */
rt_err_t button_port_init(void);

/* 读取未消抖的原始电平 */
rt_err_t button_port_read_raw(button_port_key_t key,rt_uint8_t *level);

#endif
