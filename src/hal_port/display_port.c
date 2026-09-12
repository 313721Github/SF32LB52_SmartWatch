#include "display_port.h"
#include "rtthread.h"
#include "rtdevice.h"
#include "drv_lcd.h"
#include "finsh.h"
#include "mem_section.h"

#define DISPLAY_DEVICE_NAME          "lcd"

#define DISPLAY_EXPECTED_WIDTH       390U
#define DISPLAY_EXPECTED_HEIGHT      450U

#define DISPLAY_STRIP_HEIGHT         10U
#define DISPLAY_STRIP_PIXEL_COUNT    \
    (DISPLAY_EXPECTED_WIDTH * DISPLAY_STRIP_HEIGHT)

#define DISPLAY_COLOR_BLACK          0x0000U
#define DISPLAY_COLOR_DARK_GRAY      0x4208U
#define DISPLAY_COLOR_RED            0xF800U
#define DISPLAY_COLOR_GREEN          0x07E0U
#define DISPLAY_COLOR_BLUE           0x001FU
#define DISPLAY_COLOR_WHITE          0xFFFFU
#define DISPLAY_COLOR_YELLOW         0xFFE0U
#define DISPLAY_COLOR_MAGENTA        0xF81FU

#define DISPLAY_POWER_CYCLE_COUNT 5U
#define DISPLAY_POWER_OFF_HOLD_MS 2000
#define DISPLAY_POWER_ON_HOLD_MS  2000
//错误字段：错误发生在哪一步
typedef enum
{
    DISPLAY_ERR_STAGE_NONE = 0,         //尚未发生错误
    DISPLAY_ERR_STAGE_FIND,             //查找 "lcd" 设备失败
    DISPLAY_ERR_STAGE_OPEN,             //打开LCD设备失败
    DISPLAY_ERR_STAGE_GET_INFO,         //获取分辨率、像素格式等设备信息失败。
    DISPLAY_ERR_STAGE_SET_FORMAT,       //设置像素格式
    DISPLAY_ERR_STAGE_CHECK_INFO,       //获取到的信息INFO不符合预期
    DISPLAY_ERR_STAGE_GET_OPS,          //未取得图形设备操作表，或必要操作函数为空
    DISPLAY_ERR_STAGE_SET_WINDOW_READ,  //绘制窗口未就绪
    DISPLAY_ERR_STAGE_DRAW_WHILE_OFF,    //在显示关闭状态下尝试绘制
    DISPLAY_ERR_STAGE_SET_WINDOW,       //设置绘制窗口失败
    DISPLAY_ERR_STAGE_DRAW_RECT,        //提交矩形绘制失败
    DISPLAY_ERR_STAGE_GET_DRAW_ERR,     //查询底层绘制错误状态失败
    DISPLAY_ERR_STAGE_CLOSE,            //关闭LCD设备失败
    DISPLAY_ERR_STAGE_SET_BRIGHTNESS,   //设置亮度失败
    DISPLAY_ERR_STAGE_GET_BRIGHTNESS,   //获取亮度失败
    DISPLAY_ERR_STAGE_POWER_ON,         //开屏失败
    DISPLAY_ERR_STAGE_POWER_OFF,        //关屏失败
    DISPLAY_ERR_STAGE_GET_STATE,        //获取状态失败
    DISPLAY_ERR_STAGE_STATE_VERIFY      //状态验证失败
} display_error_stage_t;

//内部私有变量

//LCD设备上下文：集中保存设备资源、运行状态和统计信息
typedef struct
{
    //lcd设备上下文
    rt_device_t device;                         //LCD设备句柄
    const struct rt_device_graphic_ops* ops;    //LCD图形操作表指针
    struct rt_device_graphic_info info;         //LCD设备信息的本地副本

    //状态字段
    rt_bool_t opened;                           //表示设备是否已经打开
    rt_bool_t ready;                            //表示是否已经完成全部初始化检查
    rt_bool_t test_running;                      //表示当前是否正在执行显示测试

    rt_err_t first_error;                       //保存本轮初始化或测试遇到的第一处具体错误码
    display_error_stage_t first_error_stage;    //保存 first_error 是在哪个操作阶段产生的
    display_port_stats_t stats;                 //保存显示模块累计运行统计

    display_port_state_t display_state;         //项目当前状态
    rt_uint8_t  last_brightness;                //最近一次设置的亮度值，范围0～100
    rt_bool_t brightness_valid;                 //亮度值是否有效
}display_port_context_t;
static display_port_context_t g_display_context;


/*
 * 复用SDK lcd_fb示例的L1非保留区和64字节对齐方式。
 * 数组类型为rt_uint16_t，每个元素表示一个RGB565像素。
 */
L1_NON_RET_BSS_SECT_BEGIN(display_port)
L1_NON_RET_BSS_SECT(
    display_port,
    ALIGN(64)
    static rt_uint16_t
    g_display_strip_buffer[DISPLAY_STRIP_PIXEL_COUNT]);
L1_NON_RET_BSS_SECT_END






//公开函数
//查找、打开并验证lcd设备，设置RGB565格式。
rt_err_t  display_port_init(void);
//矩形同步填充函数
rt_err_t display_port_fill_rect(rt_int16_t x0,rt_int16_t y0,rt_int16_t x1,rt_int16_t y1,rt_uint16_t color);
//冒烟测试
rt_err_t display_port_run_smoke_test(void);
//关闭函数
rt_err_t display_port_deinit(void);
//信息与统计查询
rt_err_t display_port_get_info(display_port_info_t *info);
rt_err_t display_port_get_stats(display_port_stats_t *stats);

//设置屏幕亮度值
rt_err_t display_port_set_brightness(rt_uint8_t percent);
//关屏
rt_err_t display_port_power_off(void);
//开屏
rt_err_t display_port_power_on(void);
//查询当前状态
rt_err_t display_port_get_control_state(display_port_control_state_t *state);
//执行控制测试
rt_err_t display_port_run_control_test(void);
//私有函数
static void display_record_first_error(display_error_stage_t stage, rt_err_t error);    //保存第一次错误
static rt_err_t display_port_release_device(void);                                       //关闭LCD设备
static void display_port_reset_runtime_state(void);                                     //清除“依赖设备正常工作的派生状态”
static rt_err_t display_port_validate_rect(rt_int16_t x0,                               //矩形合法性检查
                                           rt_int16_t y0,
                                           rt_int16_t x1,
                                           rt_int16_t y1);
static rt_err_t display_port_fill_strip(rt_uint16_t color,                              //填充分条缓冲
                                        rt_size_t pixel_count);
static rt_err_t display_port_check_draw_error(void);                                    //检查绘制错误
static rt_err_t display_port_show_color(const char *name,                               //单色全屏辅助函数
                                        rt_uint16_t color,
                                        rt_int32_t keep_ms);
static void display_port_print_info(void);                                              //打印辅助函数
static void display_port_print_stats(void);

static void display_stats(void);                                                        //打印统计信息shell入口包装
static void display_smoke_test(void);                                                   //冒烟测试shell入口包装
static void display_info(void);                                                         //打印设备信息shell入口包装

static rt_err_t display_port_query_driver_state(LCD_DrvStatusTypeDef *state);           //查询驱动状态骨架
static rt_err_t display_port_query_cached_brightness(rt_uint8_t *brightness);           //查询亮度缓存函数
static rt_err_t display_port_verify_driver_state(LCD_DrvStatusTypeDef expected);        //验证驱动状态骨架

static rt_err_t display_port_draw_control_pattern(void);                                //控制图案函数

static rt_err_t display_port_test_brightness(void);                                     //亮度测试函数

static rt_err_t display_port_test_power_cycles(void);                                   //电源循环函数

static rt_err_t display_port_test_boundaries(void);

static rt_err_t display_port_restore_after_test(void);                                  //统一恢复函数

//打印辅助函数
static const char *display_port_state_name(display_port_state_t state);

static const char *display_error_stage_name(display_error_stage_t stage);

static void display_port_print_control_state(void);

static void display_control_state(void);

static void display_control_test(void);


//---------------------------------------------------------------公开函数---------------------------------------------------------------

/*简介：LCD显示模块初始化
  输入：无            
  返回值：rt_err_t错误类型
*/
rt_err_t display_port_init(void)
{
    rt_device_t device;
    rt_err_t result;
    rt_uint16_t buffer_format;

    struct rt_device_graphic_ops* ops;
    
    result=RT_EOK;
    //初始化开始
    g_display_context.stats.init_attempt++;

    //已经初始化完成了
    if(g_display_context.opened||g_display_context.ready)
    {
        rt_kprintf("[LCD][WARN] LCD device is already opened!!!\n");
        return -RT_EBUSY;
    }

    //清理本轮临时字段
    g_display_context.device=RT_NULL;
    g_display_context.ops=RT_NULL;
    rt_memset( &g_display_context.info,0,sizeof( g_display_context.info));
    g_display_context.opened=RT_FALSE;
    g_display_context.ready=RT_FALSE;
    g_display_context.first_error=0;
    g_display_context.first_error_stage=DISPLAY_ERR_STAGE_NONE;
    g_display_context.display_state=DISPLAY_PORT_STATE_CLOSED;

    device=rt_device_find(DISPLAY_DEVICE_NAME);
    if(!device)
    {

        rt_kprintf("[LCD][ERR] LCD device is null\n");
        g_display_context.stats.init_fail++;
        display_record_first_error(DISPLAY_ERR_STAGE_FIND,-RT_ENOSYS);//保存第一处错误
        result=-RT_ENOSYS;
        goto fail;
    }


    result=rt_device_open(device,RT_DEVICE_OFLAG_RDWR);
    if(result!=RT_EOK)
    {
        rt_kprintf("[LCD][ERR] LCD device open failed\n");
        g_display_context.stats.init_fail++;
        display_record_first_error(DISPLAY_ERR_STAGE_OPEN,result);
        goto fail;
    }

    //设备存在且可以正常打开
    g_display_context.device=device;
    g_display_context.opened=RT_TRUE;

    //获取设备信息
    result=rt_device_control(device,RTGRAPHIC_CTRL_GET_INFO,&g_display_context.info);
    if (result != RT_EOK)
    {
        rt_kprintf("[LCD][ERR] LCD get info failed\n");
        g_display_context.stats.init_fail++;
        display_record_first_error(DISPLAY_ERR_STAGE_GET_INFO,result);
        goto fail;
    }

    rt_kprintf("[LCD][INFO] LCD width:%d\n",g_display_context.info.width);
    rt_kprintf("[LCD][INFO] LCD height:%d\n",g_display_context.info.height);

    rt_kprintf("[LCD][INFO] LCD bits_per_pixel:%d\n",g_display_context.info.bits_per_pixel);
    rt_kprintf("[LCD][INFO] LCD pixel_format:%d\n",g_display_context.info.pixel_format);

    rt_kprintf("[LCD][INFO] LCD draw_align:%d\n",g_display_context.info.draw_align);
    rt_kprintf("[LCD][INFO] LCD is_round:%d\n",g_display_context.info.is_round);
    rt_kprintf("[LCD][INFO] LCD framebuffer:%p\n",g_display_context.info.framebuffer);
    rt_kprintf("[LCD][INFO] LCD bandwidth:%d\n",g_display_context.info.bandwidth);

    if( g_display_context.info.width!=DISPLAY_EXPECTED_WIDTH||
        g_display_context.info.height!=DISPLAY_EXPECTED_HEIGHT||
        g_display_context.info.bits_per_pixel!=16||
        g_display_context.info.pixel_format!=RTGRAPHIC_PIXEL_FORMAT_RGB565||
        g_display_context.info.draw_align<=0)
    {
        rt_kprintf("[LCD][ERR] LCD info error\n");
        result=-RT_ERROR;
        g_display_context.stats.init_fail++;
        display_record_first_error(DISPLAY_ERR_STAGE_CHECK_INFO,result);
        
        goto fail;
    }


    ops = rt_graphix_ops(device);  
    if (ops == RT_NULL)
    {
        result = -RT_ENOSYS;
        rt_kprintf("[LCD][ERR] graphic ops is null\n");
        g_display_context.stats.init_fail++;
        display_record_first_error(DISPLAY_ERR_STAGE_GET_OPS, result);
        goto fail;
    }

    if (ops->set_window == RT_NULL)
    {
        result = -RT_ENOSYS;
        rt_kprintf("[LCD][ERR] set_window operation is not supported\n");
        g_display_context.stats.init_fail++;
        display_record_first_error(DISPLAY_ERR_STAGE_GET_OPS, result);
        goto fail;
    }

    if (ops->draw_rect == RT_NULL)
    {
        result = -RT_ENOSYS;
        rt_kprintf("[LCD][ERR] draw_rect operation is not supported\n");
        g_display_context.stats.init_fail++;
        display_record_first_error(DISPLAY_ERR_STAGE_GET_OPS, result);
        goto fail;
    }
    g_display_context.ops=ops;


    //设置像素格式
    buffer_format=RTGRAPHIC_PIXEL_FORMAT_RGB565;
    result=rt_device_control(device,RTGRAPHIC_CTRL_SET_BUF_FORMAT,&buffer_format);
    if (result != RT_EOK)
    {
        rt_kprintf("[LCD][ERR] LCD set format failed\n");
        g_display_context.stats.init_fail++;
        display_record_first_error(DISPLAY_ERR_STAGE_SET_FORMAT,result);
        goto fail;
    }

    g_display_context.ready=RT_TRUE;
    g_display_context.stats.init_success++;

    g_display_context.display_state=DISPLAY_PORT_STATE_INITIALIZED;
    g_display_context.brightness_valid=RT_FALSE;

    rt_kprintf("[LCD][EOK] LCD init success!\n");
    
    display_port_print_info();
    return result;

fail:
    display_port_release_device();
    /*
     * close_result只用于日志和统计。
     * 不覆盖first_error。
     */

    display_port_reset_runtime_state();
    return result;


}


/*简介：矩形同步填充函数
  输入： x0,y0,x1,y1:填充矩形坐标
        color:填充的颜色RGB565格式的代码         
  返回值：rt_err_t错误类型
*/
rt_err_t display_port_fill_rect(rt_int16_t x0,rt_int16_t y0,rt_int16_t x1,rt_int16_t y1,rt_uint16_t color)
{
    rt_err_t result;
    rt_int32_t rect_width;
    rt_int32_t current_y;
    rt_int32_t remaining_rows;  //剩余的高度
    rt_int32_t rows;            //行数
    rt_int32_t strip_y1;
    rt_size_t pixel_count;

    LCD_DrvStatusTypeDef driver_state;
    result=RT_EOK;



    result=display_port_validate_rect(x0,y0,x1,y1);
    if(result!=RT_EOK)
    {
        return result;
    }

    //绘制尝试次数++
    g_display_context.stats.draw_attempt++;

    rect_width=(rt_uint32_t)x1-x0+1;
    current_y=y0;
    while (current_y<=y1)
    {
        remaining_rows=(rt_int32_t)y1-current_y+1;
        /*
         * TODO 1：
         * rows取remaining_rows和DISPLAY_STRIP_HEIGHT中的较小值。
         */
        rows=remaining_rows<=DISPLAY_STRIP_HEIGHT?remaining_rows:DISPLAY_STRIP_HEIGHT;
        strip_y1=current_y+rows-1;
        pixel_count =(rt_size_t)rect_width*rows;
       
        /*
         * TODO 2：
         * 调用display_port_fill_strip()；
         * 失败立即停止。
         */
        result=display_port_fill_strip(color,pixel_count);
        if(result!=RT_EOK)
        {
            rt_kprintf("[LCD][ERR] LCD draw_rect pixel_count error! \n");
            g_display_context.stats.draw_fail++;
            display_record_first_error(DISPLAY_ERR_STAGE_DRAW_RECT,result);
            return result;
        }
       
        /*
         * TODO 3：
         * 调用ops->set_window：
         * x0, current_y, x1, strip_y1
         */
        g_display_context.ops->set_window(x0,current_y,x1,strip_y1);
       
        /*
         * TODO 4：
         * 调用ops->draw_rect：
         * buffer转换成const char *
         * 区域仍为x0, current_y, x1, strip_y1
         *
         * draw_rect返回void，不接收返回值。
         */
        g_display_context.ops->draw_rect((const char *)g_display_strip_buffer,x0,current_y,x1,strip_y1);
        
        result = display_port_check_draw_error();
        if (result != RT_EOK)
        {
            /*
             * TODO：
             * draw_fail增加；
             * 保存第一处错误；
             * 打印本条y范围；
             * 返回result。
             */
            g_display_context.stats.draw_fail++;
            display_record_first_error(DISPLAY_ERR_STAGE_GET_DRAW_ERR,result);
            rt_kprintf("[LCD][ERR] LCD check_draw_erro X0:%d, y0:%d,X1:%d,y1:%d,\n",x0,current_y,x1,strip_y1);
            return result;
        }

        /*
         * TODO 5：
         * current_y移动到下一条。
         */
        g_display_context.stats.strip_count++;
        current_y+=rows;

    }
    
   //更新设备驱动状态缓存
   //绘制到最后一行时驱动应该会将设备状态更新为DISPLAY_ON
    if(g_display_context.display_state==DISPLAY_PORT_STATE_INITIALIZED&&y1==(rt_int16_t)(g_display_context.info.height-1U))
    {
        result=display_port_query_driver_state(&driver_state);
        if(result!=RT_EOK)
        {
            rt_kprintf("[LCD][ERR] LCD query driver state failed!\n");
            g_display_context.stats.draw_fail++;
            return result;

        }
        else if(driver_state!=LCD_STATUS_DISPLAY_ON)
        {
            rt_kprintf("[LCD][ERR] LCD driver state is not DISPLAY_ON after full screen draw!\n");
            g_display_context.stats.draw_fail++;
            display_record_first_error(DISPLAY_ERR_STAGE_STATE_VERIFY,-RT_EIO);
            return -RT_EIO;
        }
        else
        {
            g_display_context.display_state=DISPLAY_PORT_STATE_ON;
        }

    }
    g_display_context.stats.draw_success++;

    return result;
}



//冒烟测试
rt_err_t display_port_run_smoke_test(void)
{
    rt_err_t result;

    rt_err_t close_result;

    result = RT_EOK; 
    if (g_display_context.test_running == RT_TRUE)
    {
        return -RT_EBUSY;
    }

    g_display_context.test_running=RT_TRUE;
    g_display_context.first_error=RT_EOK;

    rt_kprintf("[DISPLAY][TEST] start\n");

    result = display_port_init();
    if (result != RT_EOK)
    {
        /*
         * TODO：
         * 记录错误；
         * 跳到统一结束位置。
         */
        rt_kprintf("[DISPLAY][ERR] display_port_init failed!\n");
        goto end;     
    }


    /*
     * TODO 1：依次测试全屏颜色
     *
     * black      1000 ms
     * dark gray  1000 ms
     * red        1000 ms
     * green      1000 ms
     * blue       1000 ms
     * white       300～500 ms
     *
     * 每次都检查display_port_show_color返回值。
     * 任意失败停止后续测试。
     */
    result=display_port_show_color("black",DISPLAY_COLOR_BLACK,1000);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_show_color failed!\n");
        goto end;     
    }
    result=display_port_show_color("dark gray",DISPLAY_COLOR_DARK_GRAY,1000);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_show_color failed!\n");
        goto end;     
    }
    result=display_port_show_color("red",DISPLAY_COLOR_RED,1000);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_show_color failed!\n");
        goto end;     
    }
    result=display_port_show_color("green",DISPLAY_COLOR_GREEN,1000);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_show_color failed!\n");
        goto end;     
    }
    result=display_port_show_color("blue",DISPLAY_COLOR_BLUE,1000);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_show_color failed!\n");
        goto end;     
    }
    result=display_port_show_color("white",DISPLAY_COLOR_WHITE,500);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_show_color failed!\n");
        goto end;     
    }

    /*
     * TODO 2：不对称方向图
     *
     * 全屏深灰
     * 左上红色：(0,0)～(15,15)
     * 右上绿色：(358,0)～(389,15)
     * 左下蓝色：(0,418)～(15,449)
     * 右下黄色：(358,418)～(389,449)
     * 中央品红：(160,208)～(223,239)
     *
     * 保持约10秒。
     */
    result=display_port_show_color("dark gray",DISPLAY_COLOR_DARK_GRAY,1000);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_show_color failed!\n");
        goto end;     
    }

    /*逻辑左上红色：
    (48,48)～(79,79)
    尺寸32×32*/  
    result=display_port_fill_rect(48,48,79,79,DISPLAY_COLOR_RED);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_fill_rect failed!\n");
        goto end;     
    } 
    
    /*逻辑右上绿色：
    (310,48)～(357,79)
    尺寸48×32*/
    result=display_port_fill_rect(310,48,357,79,DISPLAY_COLOR_GREEN);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_fill_rect failed!\n");
        goto end;     
    } 
    /*逻辑左下蓝色：
    (48,370)～(79,417)
    尺寸32×48*/
    result=display_port_fill_rect(48,370,79,417,DISPLAY_COLOR_BLUE);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_fill_rect failed!\n");
        goto end;     
    }  
    /*逻辑右下黄色：
    (310,370)～(357,417)
    尺寸48×48*/
    result=display_port_fill_rect(310,370,357,417,DISPLAY_COLOR_YELLOW);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_fill_rect failed!\n");
        goto end;     
    } 
    /*中央品红：
    继续使用(160,208)～(223,239)
    尺寸64×32*/
    result=display_port_fill_rect(160,208,223,239,DISPLAY_COLOR_MAGENTA);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_fill_rect failed!\n");
        goto end;     
    }  
    rt_thread_mdelay(10000);

        /*
     * TODO 3：局部刷新
     *
     * 全屏深灰
     * 中央区域依次显示红、绿、蓝
     * 每次保持约2秒
     * 检查每一步返回值。
     */
    result=display_port_show_color("dark gray",DISPLAY_COLOR_DARK_GRAY,1000);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_show_color failed!\n");
        goto end;     
    }

    result=display_port_fill_rect(160,208,223,239,DISPLAY_COLOR_RED);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_Partial refresh failed!\n");
        goto end;     
    }  
    rt_thread_mdelay(2000);
    result=display_port_fill_rect(160,208,223,239,DISPLAY_COLOR_GREEN);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_Partial refresh failed!\n");
        goto end;     
    }  
    rt_thread_mdelay(2000);
    result=display_port_fill_rect(160,208,223,239,DISPLAY_COLOR_BLUE);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_Partial refresh failed!\n");
        goto end;     
    } 
    rt_thread_mdelay(2000); 

    /*
     * TODO 4：安全非法输入
     *
     * 调用fill_rect时让x1 == device_info.width。
     * 预期返回-RT_EINVAL。
     *
     * 这是预期错误：
     * - 不应覆盖正常测试first_error；
     * - 不应被当作硬件绘制失败；
     * - invalid_count应增加。
     */
    result = display_port_fill_rect(
    0,
    0,
    (rt_int16_t)g_display_context.info.width,
    15,
    DISPLAY_COLOR_BLUE);
    if (result != -RT_EINVAL)
    {
        rt_kprintf("[DISPLAY][ERR] invalid-coordinate guard failed, result=%d\n",result);
        result = -RT_ERROR;  
        goto end;
    } 
    result = RT_EOK;
    rt_kprintf("[DISPLAY][TEST] invalid-coordinate guard PASS\n");

end:
    close_result=display_port_deinit();
    if ((result == RT_EOK) && (close_result != RT_EOK))
    {
        result = close_result;
    }

    g_display_context.test_running = RT_FALSE;

    rt_kprintf("[DISPLAY][TEST] end result=%d first_error=%d stage=%d\n",
               result,
               g_display_context.first_error,
               (int)g_display_context.first_error_stage);
    return result;

}


/*简介：关闭函数
  输入：无            
  返回值：rt_err_t错误类型
*/
rt_err_t display_port_deinit(void)
{
    rt_err_t close_result;

    close_result=RT_EOK;
    /*
     * 先禁止后续绘制。
     * 如果未来存在并发访问，这一步还需要锁保护。
     */
    g_display_context.ready = RT_FALSE;

    close_result = display_port_release_device();
    display_port_reset_runtime_state();

    return close_result;
}

/*简介：信息查询函数
  输入：info：收集信息参数            
  返回值：rt_err_t错误类型
*/
rt_err_t display_port_get_info(display_port_info_t *info)
{
    if (info == RT_NULL)
    {
        return -RT_EINVAL;
    }

    if ((g_display_context.ready != RT_TRUE) ||
        (g_display_context.device == RT_NULL))
    {
        return -RT_ENOSYS;
    }

    info->width = g_display_context.info.width;
    info->height = g_display_context.info.height;
    info->pixel_format = g_display_context.info.pixel_format;
    info->bits_per_pixel = g_display_context.info.bits_per_pixel;
    info->draw_align = g_display_context.info.draw_align;
    info->is_round = g_display_context.info.is_round;
    info->bandwidth = g_display_context.info.bandwidth;

    return RT_EOK;
}

/*简介：统计信息查询
  输入：stats：收集统计信息参数           
  返回值：rt_err_t错误类型
*/
rt_err_t display_port_get_stats(display_port_stats_t *stats)
{
    if (stats == RT_NULL)
    {
        return -RT_EINVAL;
    }

    rt_memcpy(stats,
              &g_display_context.stats,
              sizeof(display_port_stats_t));

    return RT_EOK;
}

/*简介：设置屏幕亮度值
  输入：percent：要设置的亮度值，范围0～100
  返回值：rt_err_t错误类型
*/
rt_err_t display_port_set_brightness(rt_uint8_t percent)
{
    rt_err_t result;
    rt_uint8_t cached_brightness;
    LCD_DrvStatusTypeDef driver_state;

    /*
     * 1. percent > 100：
     *    invalid_count++;
     *    返回-RT_EINVAL；
     *    不保存first_error。
     */
    if(percent>100)
    {
        g_display_context.stats.invalid_count++;
        return -RT_EINVAL;
    }

    /*
     * 2. 检查ready、opened和device：
     *    不满足时invalid_count++;
     *    返回-RT_ENOSYS。
     */

    if ((g_display_context.ready != RT_TRUE) ||
        (g_display_context.device == RT_NULL)||
        (g_display_context.opened != RT_TRUE))
    {
        g_display_context.stats.invalid_count++;
        result=-RT_ENOSYS;
        return result;
    }


    /*
     * 3. 如果display_state == OFF：
     *    invalid_count++;
     *    返回-RT_EBUSY；
     *    不进入SDK。
     */
    if((display_port_state_t)g_display_context.display_state==DISPLAY_PORT_STATE_OFF||
       (display_port_state_t)g_display_context.display_state==DISPLAY_PORT_STATE_CLOSED)
    {
        g_display_context.stats.invalid_count++;
        return -RT_EBUSY;
    }

    /*
     * 4. 只允许INITIALIZED或ON状态。
     */

    /* 5. brightness_attempt++。 */
    g_display_context.stats.brightness_attempt++;

    /*
     * 6. 调用：
     * rt_device_control(
     *     device,
     *     RTGRAPHIC_CTRL_SET_BRIGHTNESS,
     *     &percent);
     */
    result=rt_device_control(
        g_display_context.device,
        RTGRAPHIC_CTRL_SET_BRIGHTNESS,
        &percent);

    /*
     * 7. SET失败：
     *    brightness_fail++;
     *    保存SET_BRIGHTNESS阶段；
     *    返回result。
     */
    if (result != RT_EOK)
    {
        g_display_context.stats.brightness_fail++;
        display_record_first_error(DISPLAY_ERR_STAGE_SET_BRIGHTNESS,result);
        rt_kprintf("[DISPLAY][ERR] set_brightness failed, result=%d\n",result);
        return result;
    }

    /*
     * 8. 调用query_cached_brightness()。
     *    失败时brightness_fail++并返回。
     */
    result=display_port_query_cached_brightness(&cached_brightness);
    if (result != RT_EOK)
    {
        g_display_context.stats.brightness_fail++;
        rt_kprintf("[DISPLAY][ERR] query_cached_brightness failed, result=%d\n",result);
        return result;
    }

    /*
     * 9. cached_brightness != percent：
     *    brightness_fail++;
     *    保存STATE_VERIFY；
     *    返回-RT_EIO。
     */
    if(cached_brightness!=percent)
    {
        g_display_context.stats.brightness_fail++;
        display_record_first_error(DISPLAY_ERR_STAGE_STATE_VERIFY,-RT_EIO);
        rt_kprintf("[DISPLAY][ERR] cached_brightness != percent, cached=%d, percent=%d\n",cached_brightness,percent);
        return -RT_EIO;
    }

    /*
     * 10. 查询driver_state。
     *     预期LCD_STATUS_DISPLAY_ON。
     *
     * 当前drv_lcd处理SET_BRIGHTNESS后会调用
     * lcd_display_on()，所以0%时状态也应为ON。
     */
    result=display_port_query_driver_state(&driver_state);
    if (result != RT_EOK)
    {
        g_display_context.stats.brightness_fail++;
        rt_kprintf("[DISPLAY][ERR] query_driver_state failed, result=%d\n",result);
        return result;
    }

    /*
     * 11. 状态不符合：
     *     brightness_fail++;
     *     保存STATE_VERIFY；
     *     返回-RT_EIO。
     */
    if(driver_state!=LCD_STATUS_DISPLAY_ON)
    {
        g_display_context.stats.brightness_fail++;
        display_record_first_error(DISPLAY_ERR_STAGE_STATE_VERIFY,-RT_EIO);
        rt_kprintf("[DISPLAY][ERR] driver_state != LCD_STATUS_DISPLAY_ON, state=%d\n",driver_state);
        return -RT_EIO;
    }

    /*
     * 12. 全部成功后：
     *     last_brightness = percent;
     *     brightness_valid = RT_TRUE;
     *     display_state = DISPLAY_PORT_STATE_ON;
     *     brightness_success++;
     */
    g_display_context.last_brightness=percent;
    g_display_context.brightness_valid=RT_TRUE;
    g_display_context.display_state=DISPLAY_PORT_STATE_ON;
    g_display_context.stats.brightness_success++;

    /*
     * 13. 打印：
     * request、cached、driver_state、result。
     */
    rt_kprintf("[DISPLAY][EOK] set_brightness request=%d, cached=%d, driver_state=%d, result=%d\n",
               percent,
               cached_brightness,
               driver_state,
               result);

    return RT_EOK;
}

/*简介：关屏
  输入：无
  返回值：rt_err_t错误类型
*/
rt_err_t display_port_power_off(void)
{
    rt_err_t result;

    /*
     * 1. 检查ready、opened、device。
     *    不满足返回-RT_ENOSYS。
     */
    if ((g_display_context.ready != RT_TRUE) ||
        (g_display_context.device == RT_NULL)||
        (g_display_context.opened != RT_TRUE))
    {
        g_display_context.stats.invalid_count++;
        result=-RT_ENOSYS;
        return result;
    }

    /*
     * 2. 如果已经OFF：
     *    打印already off；
     *    直接返回RT_EOK；
     *    不重复调用SDK。
     */
    if((display_port_state_t)g_display_context.display_state==DISPLAY_PORT_STATE_OFF)
    {
        rt_kprintf("[DISPLAY][WARN] already off\n");
        return RT_EOK;
    }

    /*
     * 3. 只允许INITIALIZED或ON进入POWEROFF。
     *    其他状态返回-RT_EBUSY。
     */
    if((display_port_state_t)g_display_context.display_state!=DISPLAY_PORT_STATE_INITIALIZED&&
       (display_port_state_t)g_display_context.display_state!=DISPLAY_PORT_STATE_ON)
    {
        g_display_context.stats.invalid_count++;
        return -RT_EBUSY;
    }


    /* 4. power_off_attempt++。 */
    g_display_context.stats.power_off_attempt++;
    /*
     * 5. 调用：
     * rt_device_control(
     *     device,
     *     RTGRAPHIC_CTRL_POWEROFF,
     *     RT_NULL);
     */
    result=rt_device_control(
        g_display_context.device,
        RTGRAPHIC_CTRL_POWEROFF,
        RT_NULL);

    /*
     * 6. 调用失败：
     *    power_off_fail++;
     *    保存POWER_OFF首错；
     *    返回result。
     */
    if(result!=RT_EOK)
    {
        g_display_context.stats.power_off_fail++;
        display_record_first_error(DISPLAY_ERR_STAGE_POWER_OFF,result);
        rt_kprintf("[DISPLAY][ERR] power_off failed, result=%d\n",result);
        return result;
    }
    //保守将项目状态设置为OFF，避免后续绘制尝试
    g_display_context.display_state=DISPLAY_PORT_STATE_OFF;

    /*
     * 7. 调用verify_driver_state(
     *        LCD_STATUS_DISPLAY_OFF)。
     */
    result=display_port_verify_driver_state(LCD_STATUS_DISPLAY_OFF);

    /*
     * 8. 验证失败：
     *    power_off_fail++;
     *    验证失败时项目状态继续保守保持 OFF
     *    返回错误。
     */
    if(result!=RT_EOK)
    {
        g_display_context.stats.power_off_fail++;
        rt_kprintf("[DISPLAY][ERR] verify_driver_state failed, result=%d\n",result);
        return result;
    }

    /*
     * 9. 成功后：
     *    display_state = OFF;
     *    power_off_success++;
     */
    g_display_context.display_state=DISPLAY_PORT_STATE_OFF;
    g_display_context.stats.power_off_success++;

    return RT_EOK;
}

/*简介：开屏
  输入：无
  返回值：rt_err_t错误类型
*/
rt_err_t display_port_power_on(void)
{
    rt_err_t result;
    LCD_DrvStatusTypeDef driver_state;

    /*
     * 1. 检查ready、opened和device。
     */
    if ((g_display_context.ready != RT_TRUE) ||
        (g_display_context.device == RT_NULL)||
        (g_display_context.opened != RT_TRUE))
    {
        g_display_context.stats.invalid_count++;
        result=-RT_ENOSYS;
        return result;
    }

    /*
     * 2. 如果状态为ON或INITIALIZED：
     *    打印already powered；
     *    返回RT_EOK；
     *    不重复进入SDK。
     */
    if((display_port_state_t)g_display_context.display_state==DISPLAY_PORT_STATE_ON||
       (display_port_state_t)g_display_context.display_state==DISPLAY_PORT_STATE_INITIALIZED)
    {
        rt_kprintf("[DISPLAY][WARN] already on\n");
        return RT_EOK;
    }

    /*
     * 3. 只有OFF状态允许继续。
     */
    if((display_port_state_t)g_display_context.display_state!=DISPLAY_PORT_STATE_OFF)
    {
        g_display_context.stats.invalid_count++;
        return -RT_EBUSY;
    }

    /* 4. power_on_attempt++。 */
    g_display_context.stats.power_on_attempt++;
    /*
     * 5. 调用：
     * rt_device_control(
     *     device,
     *     RTGRAPHIC_CTRL_POWERON,
     *     RT_NULL);
     *
     * 注意：这里只证明POWERON消息成功进入队列。
     */
    result=rt_device_control(
        g_display_context.device,
        RTGRAPHIC_CTRL_POWERON,
        RT_NULL);

    /*
     * 6. POWERON请求失败：
     *    power_on_fail++;
     *    保存POWER_ON首错；
     *    返回result。
     */
    if(result!=RT_EOK)
    {
        g_display_context.stats.power_on_fail++;
        display_record_first_error(DISPLAY_ERR_STAGE_POWER_ON,result);
        rt_kprintf("[DISPLAY][ERR] power_on failed, result=%d\n",result);
        return result;
    }

    /*
     * 7. 立即调用query_driver_state(&driver_state)。
     *
     * GET_STATE通过同步消息执行；
     * 队列顺序保证它排在POWERON后面。
     */
    result=display_port_query_driver_state(&driver_state);
    if(result!=RT_EOK)
    {
        g_display_context.stats.power_on_fail++;
        display_record_first_error(DISPLAY_ERR_STAGE_GET_STATE,result);
        rt_kprintf("[DISPLAY][ERR] query_driver_state failed, result=%d\n",result);
        return result;
    }

    /*
     * 8. 预期：
     * driver_state == LCD_STATUS_INITIALIZED
     */

    /*
     * 9. 状态不符合：
     *    power_on_fail++;
     *    保存STATE_VERIFY；
     *    项目状态仍保持OFF；
     *    返回-RT_EIO。
     */
    if(driver_state != LCD_STATUS_INITIALIZED)
    {
        g_display_context.stats.power_on_fail++;
        display_record_first_error(DISPLAY_ERR_STAGE_STATE_VERIFY,-RT_EIO);
        rt_kprintf("[DISPLAY][ERR] driver_state != LCD_STATUS_INITIALIZED, state=%d\n",driver_state);
        return -RT_EIO;
    }
    /*
     * 10. 成功：
     *     display_state = INITIALIZED;
     *     power_on_success++;
     */
    g_display_context.display_state=DISPLAY_PORT_STATE_INITIALIZED;
    g_display_context.stats.power_on_success++;
    /*
     * 11. 不在这里设置ON。
     *     不在这里偷偷重绘。
     */

    return RT_EOK;
}

/*简介：获取显示控制状态
  输入：state:显示控制状态结构体指针
  返回值：rt_err_t错误类型
*/
rt_err_t display_port_get_control_state(display_port_control_state_t *state)
{
    /* 1. 检查state是否为空。 */
    if(state==RT_NULL)
    {
        return -RT_EINVAL;
    }
    /*
     * 2. 复制：
     *    display_state
     *    last_brightness
     *    brightness_valid
     */
    state->display_state=(display_port_state_t)g_display_context.display_state;
    state->last_brightness=g_display_context.last_brightness;
    state->brightness_valid=g_display_context.brightness_valid;

    /* 3. 返回RT_EOK。 */
    return RT_EOK;
}


//执行控制测试
rt_err_t display_port_run_control_test(void)
{
    rt_err_t result = RT_EOK;
    rt_err_t restore_result = RT_EOK;
    rt_err_t close_result = RT_EOK;
    rt_bool_t initialized_here = RT_FALSE;

    if (g_display_context.test_running == RT_TRUE)
    {
        return -RT_EBUSY;
    }

    g_display_context.test_running = RT_TRUE;
    g_display_context.first_error = RT_EOK;
    g_display_context.first_error_stage = DISPLAY_ERR_STAGE_NONE;

    /* 原测试流程 */

    result = display_port_init();
    if (result != RT_EOK)
    {
        goto end;
    }

    initialized_here = RT_TRUE;

    result=display_port_set_brightness(50);
    if(result!=RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_set_brightness failed!\n");
        goto end;
    }

    result=display_port_draw_control_pattern();
    if(result!=RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_draw_control_pattern failed!\n");
        goto end;
    }

    result=display_port_test_brightness();                                     //亮度测试函数
    if(result!=RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_test_brightness failed!\n");
        goto end;
    }
    result=display_port_test_power_cycles();                                   //电源循环函数
    if(result!=RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_test_power_cycles failed!\n");
        goto end;
    }
    result=display_port_test_boundaries();                                   //边界测试函数
    if(result!=RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_test_boundaries failed!\n");
        goto end;
    }


end:
    if (initialized_here == RT_TRUE)
    {
        if ((g_display_context.ready == RT_TRUE) &&
            (g_display_context.opened == RT_TRUE) &&
            (g_display_context.device != RT_NULL))
        {
            /*
             * 不按ON/INITIALIZED/OFF过滤。
             * restore_after_test()内部负责处理OFF和INITIALIZED。
             */
            restore_result = display_port_restore_after_test();

            if ((result == RT_EOK) &&
                (restore_result != RT_EOK))
            {
                result = restore_result;
            }
        }

        close_result = display_port_deinit();

        if ((result == RT_EOK) &&
            (close_result != RT_EOK))
        {
            result = close_result;
        }
    }

    g_display_context.test_running = RT_FALSE;

    /* 保留现有结束日志 */
    return result;
}

//---------------------------------------------------------------私有函数---------------------------------------------------------------

/*简介：保存测试的第一次错误
  输入：display_error_stage_t：错误字段
        rt_err_t：错误类型
  返回值：无
*/
static void display_record_first_error(display_error_stage_t stage, rt_err_t error)
{
    if((g_display_context.first_error==RT_EOK)&&(error!=RT_EOK))
    {
        g_display_context.first_error=error;
        g_display_context.first_error_stage=stage;
    }
    return;
}


/*简介：关闭已经打开的LCD设备
  输入：无
  返回值：rt_err_t：错误类型
*/
static rt_err_t display_port_release_device(void)
{

    rt_err_t result;
    result=RT_EOK;
    if(g_display_context.opened==RT_FALSE)
    {
        g_display_context.device=RT_NULL;
        return RT_EOK;
    }

    if(g_display_context.device==RT_NULL)
    {
         /* opened和device出现矛盾。
         * 记录内部状态错误，但不能调用close。
         */
        return -RT_ERROR;
    }


    result=rt_device_close(g_display_context.device);
    if(result==RT_EOK)
    {
        g_display_context.opened = RT_FALSE;
        g_display_context.device = RT_NULL;
    }
    else
    {
        g_display_context.stats.close_fail++;
        display_record_first_error(DISPLAY_ERR_STAGE_CLOSE,result);
        rt_kprintf("[DISPLAY][ERR] close failed, result=%d\n",result);
    }

    return result;
}


/*简介：清除“依赖设备正常工作的派生状态”
  输入：无
  返回值：rt_err_t：错误类型
*/
static void display_port_reset_runtime_state(void)
{
    g_display_context.ready = RT_FALSE;
    g_display_context.ops = RT_NULL;
    g_display_context.display_state = DISPLAY_PORT_STATE_CLOSED;
    g_display_context.brightness_valid = RT_FALSE;

    /* 清除或标记设备信息无效 */
    rt_memset(&g_display_context.info, 0, sizeof(g_display_context.info));
}


/*简介：矩形合法性检查
  输入： x0,y0,x1,y1:填充矩形坐标
  返回值：rt_err_t：错误类型
*/
static rt_err_t display_port_validate_rect(rt_int16_t x0,
                                           rt_int16_t y0,
                                           rt_int16_t x1,
                                           rt_int16_t y1)
{
    rt_err_t result;
    rt_int32_t width;
    rt_int32_t height;
    rt_uint8_t align;



    result=RT_EOK;

    //检查设备是否就绪
    if(g_display_context.ready!=RT_TRUE||
        g_display_context.device==RT_NULL||
        g_display_context.ops==RT_NULL)
    {
        result=-RT_ENOSYS;
        rt_kprintf("[LCD][ERR] LCD draw_rect is not ready\n");
        g_display_context.stats.invalid_count++;
        display_record_first_error(DISPLAY_ERR_STAGE_SET_WINDOW_READ,result);

        return result;
    }

    if(g_display_context.display_state!=DISPLAY_PORT_STATE_INITIALIZED&&
       g_display_context.display_state!=DISPLAY_PORT_STATE_ON)
    {
        result=-RT_EBUSY;
        rt_kprintf("[LCD][ERR] LCD draw_rect is not initialized or on\n");
        g_display_context.stats.invalid_count++;
        display_record_first_error(DISPLAY_ERR_STAGE_DRAW_WHILE_OFF,result);

        return result;
    }
    //检查参数是否合法
    if(x0<0||y0<0||x0>x1||y0>y1||x1>=g_display_context.info.width||y1>=g_display_context.info.height)
    {

            result=-RT_EINVAL;
                    rt_kprintf(
            "[DISPLAY][REJECT] invalid rect "
            "(%d,%d)-(%d,%d), screen=%ux%u\n",
            x0,
            y0,
            x1,
            y1,
            (unsigned int)g_display_context.info.width,
            (unsigned int)g_display_context.info.height);
            g_display_context.stats.invalid_count++;

            return result;

    }

    align = g_display_context.info.draw_align;
    if (align == 0U)
    {
        rt_kprintf("[DISPLAY][ERR] invalid draw_align=0\n");
        g_display_context.stats.invalid_count++;
        display_record_first_error(
            DISPLAY_ERR_STAGE_SET_WINDOW,
            -RT_EIO);
        return -RT_EIO;
    }


    width = (rt_int32_t)x1 - x0 + 1;
    height = (rt_int32_t)y1 - y0 + 1;

    if (((x0 % align) != 0) ||
        ((y0 % align) != 0) ||
        ((width % align) != 0) ||
        ((height % align) != 0))
    {
        rt_kprintf(
            "[DISPLAY][REJECT] unaligned rect "
            "(%d,%d)-(%d,%d), size=%dx%d align=%u\n",
            x0,
            y0,
            x1,
            y1,
            width,
            height,
            (unsigned int)align);

        g_display_context.stats.invalid_count++;
        return -RT_EINVAL;
    }

    return result;
}



/*简介：填充分条缓冲
  输入： RGB565格式颜色数据
        pixel_count像素数量
  返回值：rt_err_t：错误类型
*/
static rt_err_t display_port_fill_strip(rt_uint16_t color,
                                        rt_size_t pixel_count)
{
    rt_err_t result;
    rt_size_t index;

    result=RT_EOK;
    if((pixel_count == 0U) ||(pixel_count>DISPLAY_STRIP_PIXEL_COUNT))
    {

        result= -RT_EINVAL;
        return result;
    }

    for(index=0;index<pixel_count;index++)
    {
        g_display_strip_buffer[index]=color;
    }

    return result;
}

/*简介：检查绘制错误
  输入： 无
  返回值：rt_err_t：错误类型
*/
static rt_err_t display_port_check_draw_error(void)
{

    rt_err_t result;
    rt_uint8_t draw_error = 0U;
    //rt_bool_t busy = RT_FALSE;
    result =RT_EOK;

    result=rt_device_control(g_display_context.device,SF_GRAPHIC_CTRL_GET_DRAW_ERR,&draw_error);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] get draw error failed: %d\n",
                   result);
        return result;
    }
    if(draw_error!=0)
    {
        rt_kprintf("[DISPLAY][ERR] driver draw_error=%u\n",
                   (unsigned int)draw_error);
        return -RT_EIO;
    }
/*
//当前配置编入了framebuffer支持，
//但任务5仅打开原始lcd设备。
//GET_BUSY仍无条件检查framebuffer状态，
//导致访问未初始化的drv_lcd_fb.event。


    result=rt_device_control(g_display_context.device,RTGRAPHIC_CTRL_GET_BUSY,&busy);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] get busy failed: %d\n",
                   result);
        return result;
    }
    if (busy == RT_TRUE)
    {
        rt_kprintf("[DISPLAY][ERR] lcd remains busy after sync draw\n");
        return -RT_EBUSY;
    }
*/
    return result;
}


/*简介：单色全屏辅助函数
  输入： name：颜色名称
        color：颜色编号
        keep_ms：持续时间(ms)
  返回值：rt_err_t：错误类型
*/
static rt_err_t display_port_show_color(const char *name,
                                        rt_uint16_t color,
                                        rt_int32_t keep_ms)
{
    rt_err_t result;

    if (name == RT_NULL)
    {
        return -RT_EINVAL;
    }

    result=RT_EOK;
    rt_kprintf("[DISPLAY][COLOR] %s start color=0x%04x\n",
               name,
               (unsigned int)color);
    result=display_port_fill_rect(0,0,g_display_context.info.width-1,g_display_context.info.height-1,color);
    if(result!=RT_EOK)
    {
        return result;
    }

    if(keep_ms>0)
    {
        rt_thread_mdelay(keep_ms);
    }

    rt_kprintf("[DISPLAY][COLOR] %s done\n", name);
    return result;
}


/*简介：打印辅助函数
  输入：无
  返回值：无
*/
static void display_port_print_info(void)
{
    rt_err_t result;
    display_port_info_t info;

    rt_memset(&info, 0, sizeof(info));

    result = display_port_get_info(&info);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][INFO] unavailable, result=%d ready=%d opened=%d\n",
                   result,
                   g_display_context.ready,
                   g_display_context.opened);
        return;
    }

    rt_kprintf("[DISPLAY][INFO] width=%u height=%u\n",
               (unsigned int)info.width,
               (unsigned int)info.height);

    rt_kprintf("[DISPLAY][INFO] pixel_format=%u bpp=%u align=%u\n",
               (unsigned int)info.pixel_format,
               (unsigned int)info.bits_per_pixel,
               (unsigned int)info.draw_align);

    rt_kprintf("[DISPLAY][INFO] is_round=%u bandwidth=%u\n",
               (unsigned int)info.is_round,
               (unsigned int)info.bandwidth);

    rt_kprintf("[DISPLAY][INFO] framebuffer=%p opened=%d ready=%d\n",
               g_display_context.info.framebuffer,
               g_display_context.opened,
               g_display_context.ready);
}

/*简介：查询驱动状态辅助函数
  输入：state：指向LCD_DrvStatusTypeDef结构体的指针
        注：    这里输出参数类型必须是LCD_DrvStatusTypeDef *
                不要用rt_uint8_t *代替，因为驱动内部按LCD_DrvStatusTypeDef *写入
  返回值：rt_err_t：错误类型
*/
static rt_err_t display_port_query_driver_state(LCD_DrvStatusTypeDef *state)
{
    rt_err_t result;

    result= RT_EOK;
    if (state == RT_NULL)
    {
        result=-RT_EINVAL;
        return result;
    }

    if ((g_display_context.ready != RT_TRUE) ||
        (g_display_context.device == RT_NULL))
    {
        result=-RT_ENOSYS;
        return result;
    }

    result = rt_device_control(g_display_context.device,
                               RTGRAPHIC_CTRL_GET_STATE,
                               state);
    if (result != RT_EOK)
    {
        display_record_first_error(DISPLAY_ERR_STAGE_GET_STATE,result);
        rt_kprintf("[DISPLAY][ERR] get driver status failed, result=%d\n",result);
    }

    return result;
}


/*简介：查询亮度缓存函数
  输入：brightness：指向亮度值的指针，范围0～100
  返回值：rt_err_t：错误类型
*/
static rt_err_t display_port_query_cached_brightness(rt_uint8_t *brightness)
{
    rt_err_t result;

    result= RT_EOK;
    if (brightness == RT_NULL)
    {
        result=-RT_EINVAL;
        return result;
    }

    if ((g_display_context.ready != RT_TRUE) ||
        (g_display_context.device == RT_NULL))
    {
        result=-RT_ENOSYS;
        return result;
    }

    result=rt_device_control(g_display_context.device,
                               RTGRAPHIC_CTRL_GET_BRIGHTNESS,
                               brightness);
    if (result != RT_EOK)
    {
        display_record_first_error(DISPLAY_ERR_STAGE_GET_BRIGHTNESS,result);
        rt_kprintf("[DISPLAY][ERR] get brightness failed, result=%d\n",result);
    }

    return result;
}

/*简介：验证驱动状态函数
  输入：expected：期望的驱动状态
  返回值：rt_err_t：错误类型
*/
static rt_err_t display_port_verify_driver_state(LCD_DrvStatusTypeDef expected)
{
    rt_err_t result;
    LCD_DrvStatusTypeDef actual;

    /* 1. 调用display_port_query_driver_state(&actual)。 */
    result=display_port_query_driver_state(&actual);
    /* 2. 查询失败则直接返回。 */
    if (result != RT_EOK)
    {
        return result;
    }

    /*
     * 3. actual != expected时：
     *    - 打印expected和actual；
     *    - 保存STATE_VERIFY阶段；
     *    - 返回-RT_EIO。
     */
    if (actual != expected)
    {

        rt_kprintf("[DISPLAY][ERR] driver state mismatch, expected=%d actual=%d\n",
                   (int)expected,
                   (int)actual);
        display_record_first_error(DISPLAY_ERR_STAGE_STATE_VERIFY,-RT_EIO);
        return -RT_EIO;
    }
    /* 4. 状态一致，返回RT_EOK。 */
    return RT_EOK;
}
/*简介：控制图案函数
  输入：无
  返回值：rt_err_t：错误类型
*/
static rt_err_t display_port_draw_control_pattern(void)
{
    rt_err_t result;

    result=RT_EOK;
    rt_kprintf("[DISPLAY][TEST] draw control pattern start\n");

    result=display_port_show_color("dark gray",DISPLAY_COLOR_DARK_GRAY,1000);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_show_color failed!\n");
        return result;
    }

    /*逻辑左上红色：
    (48,48)～(79,79)
    尺寸32×32*/
    result=display_port_fill_rect(48,48,79,79,DISPLAY_COLOR_RED);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_fill_rect failed!\n");
        return result;
    }

    /*逻辑右上绿色：
    (310,48)～(357,79)
    尺寸48×32*/
    result=display_port_fill_rect(310,48,357,79,DISPLAY_COLOR_GREEN);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_fill_rect failed!\n");
        return result;
    }
    /*逻辑左下蓝色：
    (48,370)～(79,417)
    尺寸32×48*/
    result=display_port_fill_rect(48,370,79,417,DISPLAY_COLOR_BLUE);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_fill_rect failed!\n");
        return result;
    }
    /*逻辑右下黄色：
    (310,370)～(357,417)
    尺寸48×48*/
    result=display_port_fill_rect(310,370,357,417,DISPLAY_COLOR_YELLOW);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_fill_rect failed!\n");
        return result;
    }
    /*中央品红：
    继续使用(160,208)～(223,239)
    尺寸64×32*/
    result=display_port_fill_rect(160,208,223,239,DISPLAY_COLOR_MAGENTA);
    if (result != RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] display_port_fill_rect failed!\n");
        return result;
    }
    return result;
}


/*简介：亮度测试函数
  输入：无
  返回值：rt_err_t：错误类型
*/
static rt_err_t display_port_test_brightness(void)
{
    rt_err_t result;
    rt_size_t index;
    //亮度测试值数组
    static const rt_uint8_t brightness_levels[] =
    {
        20U,
        50U,
        100U,
        0U,
        50U
    };
    static rt_int32_t times[]={3000,3000,2000,2000,3000};
    result=RT_EOK;

    for(index=0;index<sizeof(brightness_levels)/sizeof(brightness_levels[0]);index++)
    {
        rt_uint8_t level=brightness_levels[index];
        rt_kprintf("[DISPLAY][TEST] set brightness=%u\n",(unsigned int)level);
        result=display_port_set_brightness(level);
        if(result!=RT_EOK)
        {
            rt_kprintf("[DISPLAY][ERR] display_port_set_brightness failed, result=%d\n",result);
            return result;
        }
        rt_thread_mdelay(times[index]);
    }
    return result;
}

/*简介：电源循环函数
  输入：无
  返回值：rt_err_t：错误类型
*/
static rt_err_t display_port_test_power_cycles(void)
{
    rt_err_t result;
    rt_uint32_t cycle;
    display_port_control_state_t state;
    LCD_DrvStatusTypeDef driver_state;

    result=RT_EOK;
    for(cycle=0;cycle<DISPLAY_POWER_CYCLE_COUNT;cycle++)
    {
        rt_kprintf("[DISPLAY][TEST] power cycle %u start\n",(unsigned int)cycle);

        //OFF
        result=display_port_power_off();
        if(result!=RT_EOK)
        {
            rt_kprintf("[DISPLAY][ERR] display_port_power_off failed, result=%d\n",result);
            return result;
        }
        result=display_port_query_driver_state(&driver_state);
        if (result != RT_EOK)
        {
            rt_kprintf("[DISPLAY][ERR] display_port_query_driver_state failed, result=%d\n",result);
            return result;
        }
        if(driver_state!=LCD_STATUS_DISPLAY_OFF)
        {
            rt_kprintf("[DISPLAY][ERR] driver state not OFF after power_off, state=%d\n",driver_state);
            return -RT_EIO;
        }
        rt_thread_mdelay(DISPLAY_POWER_OFF_HOLD_MS);

        //ON
        result=display_port_power_on();
        if(result!=RT_EOK)
        {
            rt_kprintf("[DISPLAY][ERR] display_port_power_on failed, result=%d\n",result);
            return result;
        }
        result=display_port_query_driver_state(&driver_state);
        if (result != RT_EOK)
        {
            rt_kprintf("[DISPLAY][ERR] display_port_query_driver_state failed, result=%d\n",result);
            return result;
        }
        if(driver_state!=LCD_STATUS_INITIALIZED)
        {
            rt_kprintf("[DISPLAY][ERR] driver state not initialized after power_on, state=%d\n",driver_state);
            return -RT_EIO;
        }
        //绘图测试
        result=display_port_draw_control_pattern();
        if(result!=RT_EOK)
        {
            rt_kprintf("[DISPLAY][ERR] display_port_draw_control_pattern failed, result=%d\n",result);
            return result;
        }
        result=display_port_query_driver_state(&driver_state);
        if (result != RT_EOK)
        {
            rt_kprintf("[DISPLAY][ERR] display_port_query_driver_state failed, result=%d\n",result);
            return result;
        }
        if(driver_state!=LCD_STATUS_DISPLAY_ON)
        {
            rt_kprintf("[DISPLAY][ERR] driver state not ON after power_on, state=%d\n",driver_state);
            return -RT_EIO;
        }
        rt_thread_mdelay(DISPLAY_POWER_ON_HOLD_MS);


        result=display_port_get_control_state(&state);
        if(result!=RT_EOK)
        {
            rt_kprintf("[DISPLAY][ERR] display_port_get_control_state failed, result=%d\n",result);
            return result;
        }
        rt_kprintf("[DISPLAY][TEST] power cycle %u done, state=%d brightness=%u valid=%d\n",
                   (unsigned int)cycle,
                   (int)state.display_state,
                   (unsigned int)state.last_brightness,
                   (int)state.brightness_valid);
    }

    return result;
}

/*简介：边界测试函数
  输入：无
  返回值：rt_err_t：错误类型
*/
static rt_err_t display_port_test_boundaries(void)
{
    typedef struct
    {
        const char *name;
        rt_int16_t x0;
        rt_int16_t y0;
        rt_int16_t x1;
        rt_int16_t y1;
        rt_uint16_t color;
    } display_boundary_case_t;

    //合法区域测试用例
    static const display_boundary_case_t valid_cases[] =
    {
        {
            "minimum-2x2",
            194, 224, 195, 225,
            DISPLAY_COLOR_WHITE
        },
        {
            "top-edge",
            100, 0, 289, 7,
            DISPLAY_COLOR_RED
        },
        {
            "bottom-edge",
            100, 442, 289, 449,
            DISPLAY_COLOR_GREEN
        },
        {
            "left-edge",
            0, 100, 7, 349,
            DISPLAY_COLOR_BLUE
        },
        {
            "right-edge",
            382, 100, 389, 349,
            DISPLAY_COLOR_YELLOW
        }
    };

    //非法区域测试用例
    static const display_boundary_case_t invalid_cases[] =
    {
        {
            "single-pixel",
            194, 224, 194, 224,
            DISPLAY_COLOR_MAGENTA
        },
        {
            "single-row",
            194, 224, 195, 224,
            DISPLAY_COLOR_MAGENTA
        },
        {
            "single-column",
            194, 224, 194, 225,
            DISPLAY_COLOR_MAGENTA
        },
        {
            "negative-coordinate",
            -1, 0, 1, 1,
            DISPLAY_COLOR_MAGENTA
        },
        {
            "right-overflow",
            388, 0, 390, 1,
            DISPLAY_COLOR_MAGENTA
        },
        {
            "bottom-overflow",
            0, 448, 1, 450,
            DISPLAY_COLOR_MAGENTA
        },
        {
            "reversed-endpoints",
            10, 10, 8, 8,
            DISPLAY_COLOR_MAGENTA
        }
    };

    rt_err_t result;
    rt_err_t first_error_before;
    display_error_stage_t first_stage_before;
    display_port_stats_t before;
    display_port_stats_t after;
    rt_size_t index;
    rt_size_t valid_count;
    rt_size_t invalid_count;
    rt_uint32_t invalid_delta;

    valid_count =
        sizeof(valid_cases) / sizeof(valid_cases[0]);

    invalid_count =
        sizeof(invalid_cases) / sizeof(invalid_cases[0]);

    rt_kprintf(
        "[DISPLAY][BOUNDARY] test start, "
        "valid=%u invalid=%u\n",
        (unsigned int)valid_count,
        (unsigned int)invalid_count);

    /*
     * 合法区域测试。
     * 每个用例都应该真正进入绘制路径并返回RT_EOK。
     */
    for (index = 0U; index < valid_count; index++)
    {
        rt_kprintf(
            "[DISPLAY][BOUNDARY] valid %s "
            "(%d,%d)-(%d,%d) start\n",
            valid_cases[index].name,
            (int)valid_cases[index].x0,
            (int)valid_cases[index].y0,
            (int)valid_cases[index].x1,
            (int)valid_cases[index].y1);

        result = display_port_fill_rect(
            valid_cases[index].x0,
            valid_cases[index].y0,
            valid_cases[index].x1,
            valid_cases[index].y1,
            valid_cases[index].color);

        if (result != RT_EOK)
        {
            rt_kprintf(
                "[DISPLAY][BOUNDARY][ERR] "
                "valid case %s failed, result=%d\n",
                valid_cases[index].name,
                result);

            return result;
        }

        rt_kprintf(
            "[DISPLAY][BOUNDARY] valid %s PASS\n",
            valid_cases[index].name);
    }

    /*
     * 在执行预期非法输入前保存统计和首错。
     * 后续7次请求只能增加invalid_count。
     */
    rt_memset(&before, 0, sizeof(before));

    result = display_port_get_stats(&before);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[DISPLAY][BOUNDARY][ERR] "
            "get before stats failed, result=%d\n",
            result);

        return result;
    }

    first_error_before =
        g_display_context.first_error;

    first_stage_before =
        g_display_context.first_error_stage;

    /*
     * 非法区域测试。
     * 这些输入必须由项目参数检查层返回-RT_EINVAL，
     * 不能进入set_window或draw_rect。
     */
    for (index = 0U; index < invalid_count; index++)
    {
        rt_kprintf(
            "[DISPLAY][BOUNDARY] reject %s "
            "(%d,%d)-(%d,%d) start\n",
            invalid_cases[index].name,
            (int)invalid_cases[index].x0,
            (int)invalid_cases[index].y0,
            (int)invalid_cases[index].x1,
            (int)invalid_cases[index].y1);

        result = display_port_fill_rect(
            invalid_cases[index].x0,
            invalid_cases[index].y0,
            invalid_cases[index].x1,
            invalid_cases[index].y1,
            invalid_cases[index].color);

        if (result != -RT_EINVAL)
        {
            rt_kprintf(
                "[DISPLAY][BOUNDARY][ERR] "
                "case %s returned %d, expected=%d\n",
                invalid_cases[index].name,
                result,
                -RT_EINVAL);

            display_record_first_error(
                DISPLAY_ERR_STAGE_STATE_VERIFY,
                -RT_EIO);

            return -RT_EIO;
        }

        rt_kprintf(
            "[DISPLAY][BOUNDARY] reject %s PASS, "
            "result=%d\n",
            invalid_cases[index].name,
            result);
    }

    rt_memset(&after, 0, sizeof(after));

    result = display_port_get_stats(&after);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[DISPLAY][BOUNDARY][ERR] "
            "get after stats failed, result=%d\n",
            result);

        return result;
    }

    invalid_delta =
        after.invalid_count - before.invalid_count;

    /*
     * 预期非法输入不得进入硬件绘制，因此：
     * - invalid_count增加7；
     * - draw_attempt/draw_success/draw_fail不变；
     * - strip_count不变；
     * - first_error和first_stage不变。
     */
    if ((invalid_delta != (rt_uint32_t)invalid_count) ||
        (after.draw_attempt != before.draw_attempt) ||
        (after.draw_success != before.draw_success) ||
        (after.draw_fail != before.draw_fail) ||
        (after.strip_count != before.strip_count) ||
        (g_display_context.first_error !=
         first_error_before) ||
        (g_display_context.first_error_stage !=
         first_stage_before))
    {
        rt_kprintf(
            "[DISPLAY][BOUNDARY][ERR] "
            "guard statistics mismatch\n");

        rt_kprintf(
            "[DISPLAY][BOUNDARY][ERR] "
            "invalid delta=%u expected=%u\n",
            (unsigned int)invalid_delta,
            (unsigned int)invalid_count);

        rt_kprintf(
            "[DISPLAY][BOUNDARY][ERR] "
            "draw before=%u/%u/%u/%u "
            "after=%u/%u/%u/%u\n",
            (unsigned int)before.draw_attempt,
            (unsigned int)before.draw_success,
            (unsigned int)before.draw_fail,
            (unsigned int)before.strip_count,
            (unsigned int)after.draw_attempt,
            (unsigned int)after.draw_success,
            (unsigned int)after.draw_fail,
            (unsigned int)after.strip_count);

        rt_kprintf(
            "[DISPLAY][BOUNDARY][ERR] "
            "first before=%d/%d after=%d/%d\n",
            first_error_before,
            (int)first_stage_before,
            g_display_context.first_error,
            (int)g_display_context.first_error_stage);

        display_record_first_error(
            DISPLAY_ERR_STAGE_STATE_VERIFY,
            -RT_EIO);

        return -RT_EIO;
    }

    rt_kprintf(
        "[DISPLAY][BOUNDARY] test PASS, "
        "valid=%u rejected=%u invalid_delta=%u\n",
        (unsigned int)valid_count,
        (unsigned int)invalid_count,
        (unsigned int)invalid_delta);

    return RT_EOK;
}



/*简介：统一恢复函数
  输入：无
  返回值：rt_err_t：错误类型
*/
static rt_err_t display_port_restore_after_test(void)
{

    rt_err_t result;
    display_port_control_state_t state;

    result=RT_EOK;

    if(g_display_context.ready!=RT_TRUE||
       g_display_context.device==RT_NULL||
       g_display_context.opened!=RT_TRUE)
    {
        rt_kprintf("[DISPLAY] no need to restore\n");
        return result;
    }

    //恢复电源状态
    if(g_display_context.display_state==DISPLAY_PORT_STATE_OFF)
    {
        result=display_port_power_on();
        if(result!=RT_EOK)
        {
            rt_kprintf("[DISPLAY][ERR] restore power on failed, result=%d\n",result);
            return result;
        }
    }
    if(g_display_context.display_state==DISPLAY_PORT_STATE_INITIALIZED)
    {
        //全屏绘制深灰
        result=display_port_show_color("dark gray",DISPLAY_COLOR_DARK_GRAY,1000);
        if(result!=RT_EOK)
        {
            rt_kprintf("[DISPLAY][ERR] restore show color failed, result=%d\n",result);
            return result;
        }
        result=display_port_get_control_state(&state);
        if(result!=RT_EOK)
        {
            rt_kprintf("[DISPLAY][ERR] display_port_get_control_state failed, result=%d\n",result);
            return result;
        }
    }

    //恢复亮度
    result=display_port_set_brightness(50);
    if(result!=RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] restore brightness failed, result=%d\n",result);
        return result;
    }

    //绘制稳定控制图案
    result=display_port_draw_control_pattern();
    if(result!=RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] restore draw control pattern failed, result=%d\n",result);
        return result;
    }

    //查询最终状态
    result=display_port_get_control_state(&state);
    if(result!=RT_EOK)
    {
        rt_kprintf("[DISPLAY][ERR] get control state failed, result=%d\n",result);
        return result;
    }

    rt_kprintf("[DISPLAY][RESTORE] done, state=%d brightness=%u valid=%d\n",
               (int)state.display_state,
               (unsigned int)state.last_brightness,
               (int)state.brightness_valid);
    return result;
}
 /*简介：返回项目状态名称
  输入：state：display_port_state_t类型的状态值
  返回值：char *：状态名称字符串
*/
static const char *display_port_state_name(
    display_port_state_t state)
{
    switch (state)
    {
    case DISPLAY_PORT_STATE_CLOSED:
        return "CLOSED";

    case DISPLAY_PORT_STATE_INITIALIZED:
        return "INITIALIZED";

    case DISPLAY_PORT_STATE_ON:
        return "ON";

    case DISPLAY_PORT_STATE_OFF:
        return "OFF";

    default:
        return "UNKNOWN";
    }
}
 /*简介：返回错误阶段名称
  输入：stage：display_error_stage_t类型的状态值
  返回值：char *：状态名称字符串
*/
static const char *display_error_stage_name(
    display_error_stage_t stage)
{
    switch (stage)
    {
    case DISPLAY_ERR_STAGE_NONE:
        return "NONE";

    case DISPLAY_ERR_STAGE_FIND:
        return "FIND";

    case DISPLAY_ERR_STAGE_OPEN:
        return "OPEN";

    case DISPLAY_ERR_STAGE_GET_INFO:
        return "GET_INFO";

    case DISPLAY_ERR_STAGE_SET_FORMAT:
        return "SET_FORMAT";

    case DISPLAY_ERR_STAGE_CHECK_INFO:
        return "CHECK_INFO";

    case DISPLAY_ERR_STAGE_GET_OPS:
        return "GET_OPS";

    case DISPLAY_ERR_STAGE_SET_WINDOW_READ:
        return "NOT_READY";

    case DISPLAY_ERR_STAGE_DRAW_WHILE_OFF:
        return "DRAW_WHILE_OFF";

    case DISPLAY_ERR_STAGE_SET_WINDOW:
        return "SET_WINDOW";

    case DISPLAY_ERR_STAGE_DRAW_RECT:
        return "DRAW_RECT";

    case DISPLAY_ERR_STAGE_GET_DRAW_ERR:
        return "GET_DRAW_ERR";

    case DISPLAY_ERR_STAGE_CLOSE:
        return "CLOSE";

    case DISPLAY_ERR_STAGE_SET_BRIGHTNESS:
        return "SET_BRIGHTNESS";

    case DISPLAY_ERR_STAGE_GET_BRIGHTNESS:
        return "GET_BRIGHTNESS";

    case DISPLAY_ERR_STAGE_POWER_ON:
        return "POWER_ON";

    case DISPLAY_ERR_STAGE_POWER_OFF:
        return "POWER_OFF";

    case DISPLAY_ERR_STAGE_GET_STATE:
        return "GET_STATE";

    case DISPLAY_ERR_STAGE_STATE_VERIFY:
        return "STATE_VERIFY";

    default:
        return "UNKNOWN";
    }
}

 /*简介：打印控制状态
  输入：无
  返回值：无
*/
static void display_port_print_control_state(void)
{
    rt_err_t result;
    display_port_control_state_t state;

    rt_memset(&state, 0, sizeof(state));

    result = display_port_get_control_state(&state);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[DISPLAY][STATE] query failed, result=%d\n",
            result);
        return;
    }

    rt_kprintf(
        "[DISPLAY][STATE] project=%s(%d) "
        "opened=%d ready=%d\n",
        display_port_state_name(state.display_state),
        (int)state.display_state,
        g_display_context.opened,
        g_display_context.ready);

    if (state.brightness_valid == RT_TRUE)
    {
        rt_kprintf(
            "[DISPLAY][STATE] brightness=%u valid=1\n",
            (unsigned int)state.last_brightness);
    }
    else
    {
        rt_kprintf(
            "[DISPLAY][STATE] brightness=unknown valid=0\n");
    }
}

 /*简介：打印统计信息
  输入：无
  返回值：无
*/
static void display_port_print_stats(void)
{
    rt_err_t result;
    display_port_stats_t stats;

    rt_memset(&stats, 0, sizeof(stats));

    result = display_port_get_stats(&stats);
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[DISPLAY][STATS] query failed, result=%d\n",
            result);
        return;
    }

    rt_kprintf(
        "[DISPLAY][STATS] init "
        "attempt=%u success=%u fail=%u\n",
        (unsigned int)stats.init_attempt,
        (unsigned int)stats.init_success,
        (unsigned int)stats.init_fail);

    rt_kprintf(
        "[DISPLAY][STATS] draw "
        "attempt=%u success=%u fail=%u strips=%u\n",
        (unsigned int)stats.draw_attempt,
        (unsigned int)stats.draw_success,
        (unsigned int)stats.draw_fail,
        (unsigned int)stats.strip_count);

    rt_kprintf(
        "[DISPLAY][STATS] brightness "
        "attempt=%u success=%u fail=%u\n",
        (unsigned int)stats.brightness_attempt,
        (unsigned int)stats.brightness_success,
        (unsigned int)stats.brightness_fail);

    rt_kprintf(
        "[DISPLAY][STATS] power_on "
        "attempt=%u success=%u fail=%u\n",
        (unsigned int)stats.power_on_attempt,
        (unsigned int)stats.power_on_success,
        (unsigned int)stats.power_on_fail);

    rt_kprintf(
        "[DISPLAY][STATS] power_off "
        "attempt=%u success=%u fail=%u\n",
        (unsigned int)stats.power_off_attempt,
        (unsigned int)stats.power_off_success,
        (unsigned int)stats.power_off_fail);

    rt_kprintf(
        "[DISPLAY][STATS] invalid=%u close_fail=%u\n",
        (unsigned int)stats.invalid_count,
        (unsigned int)stats.close_fail);

    rt_kprintf(
        "[DISPLAY][STATS] first_error=%d "
        "first_stage=%s(%d)\n",
        g_display_context.first_error,
        display_error_stage_name(
            g_display_context.first_error_stage),
        (int)g_display_context.first_error_stage);
}

//========================================shell===============================
static void display_smoke_test(void)
{
    rt_err_t result;

    rt_kprintf("[DISPLAY][SHELL] smoke test requested\n");

    result = display_port_run_smoke_test();

    if (result == RT_EOK)
    {
        rt_kprintf("[DISPLAY][SHELL] smoke test PASS\n");
    }
    else
    {
        rt_kprintf("[DISPLAY][SHELL] smoke test FAIL, result=%d\n",
                   result);
    }

    display_port_print_stats();
}
MSH_CMD_EXPORT(display_smoke_test,
               run raw LCD full-screen and partial-draw smoke test);

static void display_stats(void)
{
    display_port_print_stats();
}
MSH_CMD_EXPORT(display_stats,
               show display port accumulated statistics);

static void display_info(void)
{
    rt_err_t result;
    rt_err_t close_result;

    if (g_display_context.ready == RT_TRUE)
    {
        display_port_print_info();
        return;
    }

    result = display_port_init();
    if (result != RT_EOK)
    {
        rt_kprintf(
            "[DISPLAY][SHELL] info init failed, result=%d\n",
            result);
        return;
    }

    display_port_print_info();

    close_result = display_port_deinit();
    if (close_result != RT_EOK)
    {
        rt_kprintf(
            "[DISPLAY][SHELL] info close failed, result=%d\n",
            close_result);
    }
}
MSH_CMD_EXPORT(display_info,
               open lcd temporarily and show device information);

//控制打印shell包装
static void display_control_state(void)
{
    display_port_print_control_state();
    display_port_print_stats();
}
MSH_CMD_EXPORT(
    display_control_state,
    show display control state and statistics);

//控制测试shell包装
static void display_control_test(void)
{
    rt_err_t result;

    rt_kprintf(
        "[DISPLAY][SHELL] control test requested\n");

    result = display_port_run_control_test();

    if (result == RT_EOK)
    {
        rt_kprintf(
            "[DISPLAY][SHELL] control test PASS\n");
    }
    else
    {
        rt_kprintf(
            "[DISPLAY][SHELL] control test FAIL, "
            "result=%d\n",
            result);
    }

    display_port_print_control_state();
    display_port_print_stats();
}
MSH_CMD_EXPORT(
    display_control_test,
    run brightness power and boundary control test);
