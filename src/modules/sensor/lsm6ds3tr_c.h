#ifndef LSM6DS3TR_C_H
#define LSM6DS3TR_C_H

#include "rtthread.h"
/*
 * SCH_V1.2实板于2026-10-09通过WHO_AM_I实测确认SA0为低，
 * 正常访问固定使用7位地址0x6A。
 */
#define LSM6DS3TR_C_ADDRESS            LSM6DS3TR_C_ADDRESS_SA0_LOW
#define LSM6DS3TR_C_ADDRESS_SA0_LOW   0x6AU
#define LSM6DS3TR_C_ADDRESS_SA0_HIGH  0x6BU
#define LSM6DS3TR_C_WHO_AM_I_REG      0x0FU
#define LSM6DS3TR_C_WHO_AM_I_VALUE    0x6AU

rt_err_t lsm6ds3tr_c_read_id(rt_uint8_t device_address,
                             rt_uint8_t *who_am_i);     //读取LSM6DS3TR-C的WHO_AM_I寄存器值

#endif
