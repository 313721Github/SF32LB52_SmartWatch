#include "lsm6ds3tr_c.h"
#include "sensor_bus_port.h"


//公开函数声明
rt_err_t lsm6ds3tr_c_read_id(rt_uint8_t device_address,
                             rt_uint8_t *who_am_i);         // 读取LSM6DS3TR-C的WHO_AM_I寄存器值

//---------------------------------------------------------------公开函数---------------------------------------------------------------
/*简介：读取LSM6DS3TR-C的WHO_AM_I寄存器值
  输入：device_address: LSM6DS3TR-C的I2C设备地址
        who_am_i: 指向存储WHO_AM_I寄存器值的缓冲区指针
  返回值：rt_err_t:错误类型
*/
rt_err_t lsm6ds3tr_c_read_id(rt_uint8_t device_address,
                             rt_uint8_t *who_am_i)
{
    if (who_am_i == RT_NULL)
    {
        return -RT_EINVAL;
    }

    *who_am_i = 0U;

    return sensor_bus_port_read_reg(device_address,
                                    LSM6DS3TR_C_WHO_AM_I_REG,
                                    who_am_i,
                                    1U);
}
