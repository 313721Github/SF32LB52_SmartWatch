#include "sensor_bus_port.h"

#include "bf0_hal.h"
#include "rtdevice.h"

#define SENSOR_I2C_DEVICE_NAME      "i2c3"      // 传感器I2C设备名称
#define SENSOR_POWER_PIN            30          // 传感器电源引脚
#define SENSOR_POWER_STABLE_MS      100U        // 传感器电源稳定时间
#define SENSOR_REGISTER_ADDR_BITS   8U          // 传感器寄存器地址位数

static struct rt_i2c_bus_device *g_sensor_i2c_bus = RT_NULL;    // 传感器I2C总线句柄
static rt_bool_t g_sensor_bus_initialized = RT_FALSE;           // 传感器总线是否已初始化



//公开函数声明
rt_err_t sensor_bus_port_init(void);                            // 初始化传感器总线端口
rt_err_t sensor_bus_port_read_reg(rt_uint8_t device_address,
                                  rt_uint8_t register_address,
                                  void *buffer,
                                  rt_size_t length);            // 读取传感器寄存器

rt_err_t sensor_bus_port_write_reg(rt_uint8_t device_address,
                                   rt_uint8_t register_address,
                                   const void *buffer,
                                   rt_size_t length);           // 写入传感器寄存器
//私有函数声明


//---------------------------------------------------------------公开函数---------------------------------------------------------------

/*简介：初始化传感器总线端口
  输入：无
  返回值：rt_err_t:错误类型
*/
rt_err_t sensor_bus_port_init(void)
{
    rt_device_t pin_device; // pin设备句柄

    // 只能在线程上下文调用；任务4由FinSH线程执行，后续由sensor服务线程调用。
    if (g_sensor_bus_initialized == RT_TRUE)
    {
        return RT_EOK;
    }


    //在尝试通过RT-Thread控制PA30之前，确认系统中存在PIN驱动
    pin_device = rt_device_find("pin");         // 查找pin设备
    if (pin_device == RT_NULL)
    {
        rt_kprintf("[SENSOR][BUS][ERR] pin device not found\n");
        return -RT_ENOSYS;
    }

    /*
     * i2c3由SDK在INIT_BOARD阶段注册。
     * 本模块只借用设备，不负责创建、关闭或释放SDK设备。
     */
    g_sensor_i2c_bus =
        (struct rt_i2c_bus_device *)rt_device_find(SENSOR_I2C_DEVICE_NAME);

    if (g_sensor_i2c_bus == RT_NULL)
    {
        rt_kprintf("[SENSOR][BUS][ERR] i2c3 device not found\n");
        return -RT_ENOSYS;
    }
    /*
     * PA39、PA40在当前QADSPI显示配置下未被LCD占用。
     * 板上已有2.2 kΩ上拉到GS_3V3，因此使用PIN_NOPULL。
     */
    HAL_PIN_Set(PAD_PA39, I2C3_SDA, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA40, I2C3_SCL, PIN_NOPULL, 1);

    /*
     * PA30经R54连接U15.EN，拉高后开启GS_3V3传感器电源域。
     * 100 ms覆盖LTR303首次访问要求，也覆盖另外两颗器件的启动时间。
     */
    HAL_PIN_Set(PAD_PA30, GPIO_A30, PIN_PULLDOWN, 1);
    rt_pin_mode(SENSOR_POWER_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(SENSOR_POWER_PIN, PIN_HIGH);
    rt_thread_mdelay(SENSOR_POWER_STABLE_MS);

    g_sensor_bus_initialized = RT_TRUE;

    rt_kprintf("[SENSOR][BUS][OK] device=i2c3 sda=PA39 scl=PA40 "
               "power=PA30 wait_ms=%u\n",
               SENSOR_POWER_STABLE_MS);

    return RT_EOK;
}

/*简介：读取传感器寄存器
  输入：device_address - 设备地址
        register_address - 寄存器地址
        buffer - 缓冲区指针
        length - 读取长度
  返回值：rt_err_t:错误类型
*/
rt_err_t sensor_bus_port_read_reg(rt_uint8_t device_address,
                                  rt_uint8_t register_address,
                                  void *buffer,
                                  rt_size_t length)
{
    rt_size_t transferred;  // 实际传输的字节数

    // 参数检查
    if ((buffer == RT_NULL) || (length == 0U) ||
        (device_address > 0x7FU))
    {
        return -RT_EINVAL;
    }

    // 检查总线是否已初始化
    if ((g_sensor_bus_initialized == RT_FALSE) ||
        (g_sensor_i2c_bus == RT_NULL))
    {
        return -RT_ENOSYS;
    }

    // 调用RT-Thread I2C API读取寄存器
    transferred = rt_i2c_mem_read(g_sensor_i2c_bus,
                                  device_address,
                                  register_address,
                                  SENSOR_REGISTER_ADDR_BITS,
                                  buffer,
                                  length);

    // 检查实际传输的字节数是否与预期一致
    if (transferred != length)
    {
        rt_kprintf("[SENSOR][BUS][ERR] read addr=0x%02X reg=0x%02X "
                   "actual=%u expected=%u\n",
                   device_address,
                   register_address,
                   (unsigned int)transferred,
                   (unsigned int)length);
        return -RT_EIO;
    }

    return RT_EOK;
}

/*简介：写入传感器寄存器
  输入：device_address - 设备地址
        register_address - 寄存器地址
        buffer - 缓冲区指针
        length - 写入长度
  返回值：rt_err_t:错误类型
*/
rt_err_t sensor_bus_port_write_reg(rt_uint8_t device_address,
                                   rt_uint8_t register_address,
                                   const void *buffer,
                                   rt_size_t length)
{
    rt_size_t transferred;  // 实际传输的字节数
    // 参数检查
    if ((buffer == RT_NULL) || (length == 0U) ||
        (device_address > 0x7FU))
    {
        return -RT_EINVAL;
    }
    // 检查总线是否已初始化
    if ((g_sensor_bus_initialized == RT_FALSE) ||
        (g_sensor_i2c_bus == RT_NULL))
    {
        return -RT_ENOSYS;
    }
    // 调用RT-Thread I2C API写入寄存器
    transferred = rt_i2c_mem_write(g_sensor_i2c_bus,
                                   device_address,
                                   register_address,
                                   SENSOR_REGISTER_ADDR_BITS,
                                   (void *)buffer,
                                   length);

    if (transferred != length)
    {
        rt_kprintf("[SENSOR][BUS][ERR] write addr=0x%02X reg=0x%02X "
                   "actual=%u expected=%u\n",
                   device_address,
                   register_address,
                   (unsigned int)transferred,
                   (unsigned int)length);
        return -RT_EIO;
    }

    return RT_EOK;
}
