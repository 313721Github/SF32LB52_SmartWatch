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
    DISPLAY_ERR_STAGE_SET_WINDOW,       //设置绘制窗口失败
    DISPLAY_ERR_STAGE_DRAW_RECT,        //提交矩形绘制失败
    DISPLAY_ERR_STAGE_GET_DRAW_ERR,     //查询底层绘制错误状态失败
    DISPLAY_ERR_STAGE_CLOSE,            //关闭LCD设备失败
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

/*简介：打印辅助函数
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
        rt_kprintf("[DISPLAY][STATS] query failed, result=%d\n",
                   result);
        return;
    }

    rt_kprintf("[DISPLAY][STATS] init attempt=%u success=%u fail=%u\n",
               (unsigned int)stats.init_attempt,
               (unsigned int)stats.init_success,
               (unsigned int)stats.init_fail);

    rt_kprintf("[DISPLAY][STATS] draw attempt=%u success=%u fail=%u\n",
               (unsigned int)stats.draw_attempt,
               (unsigned int)stats.draw_success,
               (unsigned int)stats.draw_fail);

    rt_kprintf("[DISPLAY][STATS] strips=%u invalid=%u close_fail=%u\n",
               (unsigned int)stats.strip_count,
               (unsigned int)stats.invalid_count,
               (unsigned int)stats.close_fail);

    rt_kprintf("[DISPLAY][STATS] first_error=%d first_stage=%d\n",
               g_display_context.first_error,
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