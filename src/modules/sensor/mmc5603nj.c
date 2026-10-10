#include "mmc5603nj.h"
#include "sensor_bus_port.h"

//公开函数声明
rt_err_t mmc5603nj_read_id(rt_uint8_t *product_id);


//---------------------------------------------------------------公开函数---------------------------------------------------------------
/*简介：读取MMC5603NJ的PRODUCT_ID寄存器值
  输入：product_id: 指向存储PRODUCT_ID寄存器值的缓冲区指针
  返回值：rt_err_t:错误类型
*/
rt_err_t mmc5603nj_read_id(rt_uint8_t *product_id)
{
    if (product_id == RT_NULL)
    {
        return -RT_EINVAL;
    }

    *product_id = 0U;

    return sensor_bus_port_read_reg(MMC5603NJ_ADDRESS,
                                    MMC5603NJ_PRODUCT_ID_REG,
                                    product_id,
                                    1U);
}
