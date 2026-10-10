#ifndef MMC5603NJ_H
#define MMC5603NJ_H

#include "rtthread.h"

#define MMC5603NJ_ADDRESS           0x30U
#define MMC5603NJ_PRODUCT_ID_REG    0x39U
#define MMC5603NJ_PRODUCT_ID_VALUE  0x10U

rt_err_t mmc5603nj_read_id(rt_uint8_t *product_id);     //读取MMC5603NJ的PRODUCT_ID寄存器值

#endif
