#ifndef LTR303ALS01_H
#define LTR303ALS01_H

#include "rtthread.h"

#define LTR303ALS01_ADDRESS            0x29U
#define LTR303ALS01_PART_ID_REG        0x86U
#define LTR303ALS01_MANUFAC_ID_REG     0x87U
#define LTR303ALS01_PART_ID_VALUE      0xA0U
#define LTR303ALS01_MANUFAC_ID_VALUE   0x05U

rt_err_t ltr303als01_read_ids(rt_uint8_t *part_id,
                              rt_uint8_t *manufacturer_id);     //读取LTR303ALS01的PART_ID和MANUFACTURER_ID寄存器值

#endif
