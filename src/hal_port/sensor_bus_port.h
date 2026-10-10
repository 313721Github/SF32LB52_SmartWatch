#ifndef SENSOR_BUS_PORT_H
#define SENSOR_BUS_PORT_H

#include "rtthread.h"
#ifdef __cplusplus
extern "C" {
#endif
/*
 * 只能在线程上下文调用。
 * 当前总线句柄由本适配层借用并保存，app、UI和器件驱动不能自行查找i2c3。
 */
rt_err_t sensor_bus_port_init(void);                            // 初始化传感器总线端口
rt_err_t sensor_bus_port_read_reg(rt_uint8_t device_address,
                                  rt_uint8_t register_address,
                                  void *buffer,
                                  rt_size_t length);            // 读取传感器寄存器
rt_err_t sensor_bus_port_write_reg(rt_uint8_t device_address,
                                   rt_uint8_t register_address,
                                   const void *buffer,
                                   rt_size_t length);           // 写入传感器寄存器

#ifdef __cplusplus
}
#endif

#endif
