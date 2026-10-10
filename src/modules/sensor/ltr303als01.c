#include "ltr303als01.h"
#include "sensor_bus_port.h"

//公开函数声明
rt_err_t ltr303als01_read_ids(rt_uint8_t *part_id,
                              rt_uint8_t *manufacturer_id);         // 读取LTR303ALS01的PART_ID和MANUFACTURER_ID寄存器值

/*简介：读取LTR303ALS01的PART_ID和MANUFACTURER_ID寄存器值
  输入：part_id: 指向存储PART_ID寄存器值的缓冲区指针
        manufacturer_id: 指向存储MANUFACTURER_ID寄存器值的缓冲区指针
  返回值：rt_err_t:错误类型
*/
rt_err_t ltr303als01_read_ids(rt_uint8_t *part_id,
                              rt_uint8_t *manufacturer_id)
{
    rt_err_t result;

    if ((part_id == RT_NULL) || (manufacturer_id == RT_NULL))
    {
        return -RT_EINVAL;
    }

    *part_id = 0U;
    *manufacturer_id = 0U;

    result = sensor_bus_port_read_reg(LTR303ALS01_ADDRESS,
                                      LTR303ALS01_PART_ID_REG,
                                      part_id,
                                      1U);
    if (result != RT_EOK)
    {
        return result;
    }

    result = sensor_bus_port_read_reg(LTR303ALS01_ADDRESS,
                                      LTR303ALS01_MANUFAC_ID_REG,
                                      manufacturer_id,
                                      1U);
    if (result != RT_EOK)
    {
        return result;
    }

    return RT_EOK;
}
