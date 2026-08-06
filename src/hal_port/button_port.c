#include "button_port.h"
#include "board.h"
#include "rtdevice.h"


#ifdef RT_USING_FINSH
#include "finsh.h"
#endif

#define BUTTON_POLL_INTERVAL_MS  2U
#define BUTTON_POLL_DURATION_MS  20000U
#define BUTTON_POLL_SAMPLE_COUNT \
    (BUTTON_POLL_DURATION_MS / BUTTON_POLL_INTERVAL_MS)
#define BUTTON_DEBOUNCE_MS      30U         //按键消抖时间，单位毫秒
#define BUTTON_LONG_PRESS_MS  1000U         //按键长按时间，单位毫秒



//按键上下文
typedef struct
{
    button_port_key_t key;                      //按键编号
    rt_base_t pin;                              //按键对应的GPIO引脚编号

    struct rt_timer debounce_timer;             //消抖定时器

    volatile rt_uint8_t last_irq_level;         //上次中断触发时的电平
    volatile rt_tick_t last_irq_tick;           //上次中断触发时的系统节拍

    volatile rt_uint32_t irq_count;             //GPIO原始中断次数
    volatile rt_uint32_t notify_fail_count;     //软定时器启动失败次数
    rt_uint32_t debounce_timeout_count;         //timer线程执行确认回调次数

    rt_bool_t suppress_until_release;           //当前按键是否因为上电按住而需要等待释放；
    rt_uint32_t startup_release_count;          //记录被抑制的上电释放次数，方便诊断。
    rt_uint8_t stable_level;                    //最近确认的稳定电平
    rt_tick_t press_tick;                       //按键按下时的系统节拍
    rt_bool_t long_reported;                    //本次按下是否已经产生长按
    struct rt_timer long_timer;                 //长按一次性定时器

    rt_uint32_t press_count;                        //按下次数
    rt_uint32_t release_count;                      //释放次数
    rt_uint32_t short_count;                        //短按次数
    rt_uint32_t long_count;                         //长按次数
    rt_uint32_t event_fail_count;                   //事件失败次数
    rt_uint32_t long_timer_stop_fail_count;         //定时器状态出错

}button_port_context_t;


//KEY1 KEY2轮询验证

static rt_bool_t g_button_initialized = RT_FALSE; //标记是否已初始化
static const rt_base_t g_button_pin_table[BUTTON_PORT_KEY_COUNT] = 
{   [BUTTON_PORT_KEY1] = BSP_KEY1_PIN,
    [BUTTON_PORT_KEY2] = BSP_KEY2_PIN
}; //KEY1/KEY2对应的GPIO引脚号

static button_port_context_t g_button_context[BUTTON_PORT_KEY_COUNT]   //两个按键上下文
={
    [BUTTON_PORT_KEY1] = {
        .key = BUTTON_PORT_KEY1,
        .pin =  g_button_pin_table[BUTTON_PORT_KEY1],
        .last_irq_level = 0U,
        .last_irq_tick = 0U,
        .irq_count = 0U,
        .notify_fail_count = 0U,
        .debounce_timeout_count = 0U,

        .press_tick=0U,
        .long_reported=RT_FALSE,

        .press_count=0U,                        //按下次数
        .release_count=0U,                      //释放次数
        .short_count=0U,                        //短按次数
        .long_count=0U                         //长按次数
    },

    [BUTTON_PORT_KEY2] = {
        .key = BUTTON_PORT_KEY2,
        .pin =  g_button_pin_table[BUTTON_PORT_KEY2],
        .last_irq_level = 0U,
        .last_irq_tick = 0U,
        .irq_count = 0U,
        .notify_fail_count = 0U,
        .debounce_timeout_count = 0U,

        .press_tick=0U,
        .long_reported=RT_FALSE,

        .press_count=0U,                        //按下次数
        .release_count=0U,                      //释放次数
        .short_count=0U,                        //短按次数
        .long_count=0U                         //长按次数

    },


};


rt_err_t button_port_init(void);    //初始化配置KEY1/KEY2为输入，并记录启动时原始电平
rt_err_t button_port_read_raw(button_port_key_t key,rt_uint8_t *level); //读取未消抖的原始电平

rt_err_t button_port_register_handler(button_event_handler_t handler, void* userdata);

/* 私有函数声明 */
static int button_poll_test(int argc, char **argv);     //轮询测试按键原始电平，打印到串口
static void button_gpio_isr(void *args);                //GPIO二级中断回调；
static void button_debounce_timeout(void *parameter);   //30 ms后由timer线程运行
static void button_long_timeout(void *parameter);       //长按定时器回调
static button_event_handler_t g_event_handler = RT_NULL;
static void *g_event_user_data = RT_NULL;
static void button_emit_event(button_port_context_t *context,button_event_type_t type,rt_uint32_t duration_ms);//按键事件发送函数
    //
//初始化按键输入
rt_err_t button_port_init(void)
{
    rt_device_t pin_device;
    rt_uint8_t key1_level;
    rt_uint8_t key2_level;
    rt_err_t result;
//-------------------------------------------------按键GPIO初始化-------------------------------------------------
    if(g_button_initialized==RT_TRUE)
    {
        return RT_EOK; //已经初始化过了
    }

    pin_device = rt_device_find("pin");
    if(pin_device==RT_NULL)
    {
        rt_kprintf("[KEY][ERR] pin device not found\n");
        return -RT_ERROR;
    }
    /*
     * BSP已经将PA34/PA43复用为GPIO。
     * 此处只配置GPIO方向为输入、无内部上下拉。
    */
    rt_pin_mode(g_button_pin_table[BUTTON_PORT_KEY1], PIN_MODE_INPUT);
    rt_pin_mode(g_button_pin_table[BUTTON_PORT_KEY2], PIN_MODE_INPUT);


    //读取启动时电平
    key1_level =
        (rt_pin_read(g_button_pin_table[BUTTON_PORT_KEY1]) == PIN_LOW)
        ? 0U : 1U;

    key2_level =
        (rt_pin_read(g_button_pin_table[BUTTON_PORT_KEY2]) == PIN_LOW)
        ? 0U : 1U;
    rt_kprintf("[KEY][INIT] KEY1 pin=%d raw=%d active=high\n",
               BSP_KEY1_PIN, key1_level);
    rt_kprintf("[KEY][INIT] KEY2 pin=%d raw=%d active=high\n",
               BSP_KEY2_PIN, key2_level);

    //初始化稳定电平
    g_button_context[BUTTON_PORT_KEY1].stable_level = key1_level;
    g_button_context[BUTTON_PORT_KEY1].suppress_until_release =(key1_level == 1U) ? RT_TRUE : RT_FALSE;
    g_button_context[BUTTON_PORT_KEY2].stable_level = key2_level;
    g_button_context[BUTTON_PORT_KEY2].suppress_until_release =(key2_level == 1U) ? RT_TRUE : RT_FALSE;
    if (key1_level == 1U)
    {
        rt_kprintf(
            "[KEY][INIT] KEY1 held at startup, "
            "events suppressed until release\n");
    }

    if (key2_level == 1U)
    {
        rt_kprintf(
            "[KEY][INIT] KEY2 held at startup, "
            "events suppressed until release\n");
    }


//-------------------------------------------------按键中断及定时器初始化-------------------------------------------------
    //初始化按键消抖定时器
    rt_timer_init(&g_button_context[BUTTON_PORT_KEY1].debounce_timer,
                  "key1_debounce",
                  button_debounce_timeout,
                   &g_button_context[BUTTON_PORT_KEY1],
                  rt_tick_from_millisecond(BUTTON_DEBOUNCE_MS),
                  RT_TIMER_FLAG_ONE_SHOT |RT_TIMER_FLAG_SOFT_TIMER);
    rt_timer_init(&g_button_context[BUTTON_PORT_KEY2].debounce_timer,
                  "key2_debounce",
                  button_debounce_timeout,
                   &g_button_context[BUTTON_PORT_KEY2],
                  rt_tick_from_millisecond(BUTTON_DEBOUNCE_MS),
                  RT_TIMER_FLAG_ONE_SHOT |RT_TIMER_FLAG_SOFT_TIMER);
    rt_timer_init(&g_button_context[BUTTON_PORT_KEY1].long_timer,
                  "key1_long_timer",
                  button_long_timeout,
                   &g_button_context[BUTTON_PORT_KEY1],
                  rt_tick_from_millisecond(BUTTON_LONG_PRESS_MS),
                  RT_TIMER_FLAG_ONE_SHOT |RT_TIMER_FLAG_SOFT_TIMER);
    rt_timer_init(&g_button_context[BUTTON_PORT_KEY2].long_timer,
                  "key2_long_timer",
                  button_long_timeout,
                   &g_button_context[BUTTON_PORT_KEY2],
                  rt_tick_from_millisecond(BUTTON_LONG_PRESS_MS),
                  RT_TIMER_FLAG_ONE_SHOT |RT_TIMER_FLAG_SOFT_TIMER);

    

    //注册KEY1GPIO中断回调
    result = rt_pin_attach_irq(g_button_pin_table[BUTTON_PORT_KEY1],
                                PIN_IRQ_MODE_RISING_FALLING,
                                button_gpio_isr,
                                 &g_button_context[BUTTON_PORT_KEY1]);
    if(result != RT_EOK)
    {
        g_button_initialized = RT_FALSE;
        return result;
    }
    //注册KEY2GPIO中断回调
    result = rt_pin_attach_irq(g_button_pin_table[BUTTON_PORT_KEY2],
                                PIN_IRQ_MODE_RISING_FALLING,
                                button_gpio_isr,
                                 &g_button_context[BUTTON_PORT_KEY2]);
    if(result != RT_EOK)
    {
        g_button_initialized = RT_FALSE;
        return result;
    }  
    //使能KEY1/KEY2 GPIO中断
    result = rt_pin_irq_enable(g_button_pin_table[BUTTON_PORT_KEY1], PIN_IRQ_ENABLE);
    if(result != RT_EOK)
    {
        g_button_initialized = RT_FALSE;
        return result;
    }
    result = rt_pin_irq_enable(g_button_pin_table[BUTTON_PORT_KEY2], PIN_IRQ_ENABLE);
    if(result != RT_EOK)
    {
        g_button_initialized = RT_FALSE;
        return result;
    }

    g_button_initialized = RT_TRUE;

    if(result == RT_EOK)
    {
        rt_kprintf("[KEY][INIT] GPIO interrupt enabled\n");
    }
    return result;
}

rt_err_t button_port_register_handler(button_event_handler_t handler, void* userdata)
{
    if(handler==RT_NULL)
    {
        return -RT_EINVAL;
    }

    if(g_button_initialized==RT_TRUE)
    {
        return -RT_EBUSY;
    }

    g_event_handler=handler;
    g_event_user_data=userdata;

    return RT_EOK;

}



//读取按键原始电平
rt_err_t button_port_read_raw(button_port_key_t key,rt_uint8_t *level)
{
    rt_base_t raw_level;

    if(level==RT_NULL)
    {
        return -RT_EINVAL;
    }

    if(g_button_initialized==RT_FALSE)
    {
        return -RT_ERROR; //未初始化
    }

    if(key<BUTTON_PORT_KEY1 || key>=BUTTON_PORT_KEY_COUNT)
    {
        return -RT_EINVAL; //无效的按键编号
    }


    raw_level = rt_pin_read(g_button_pin_table[key]);
    if(raw_level==PIN_LOW)
    {
        *level = 0;
    }
    else
    {
        *level = 1;
    }

    return RT_EOK;
}


static int button_poll_test(int argc, char **argv)
{

    rt_uint8_t previous_key1;
    rt_uint8_t previous_key2;
    rt_uint8_t current_key1;
    rt_uint8_t current_key2;
    rt_uint32_t key1_transitions = 0;
    rt_uint32_t key2_transitions = 0;
    rt_uint32_t sample;
    rt_err_t result;


    //防止未初始化
    result = button_port_init();
    if (result != RT_EOK)
    {
        rt_kprintf("[KEY][TEST][ERR] init failed: %d\n",
                   (int)result);
        return (int)result;
    }

    //读取一次
    result = button_port_read_raw(BUTTON_PORT_KEY1,
                                  &previous_key1);
    if (result != RT_EOK)
    {
        return (int)result;
    }

    result = button_port_read_raw(BUTTON_PORT_KEY2,
                                  &previous_key2);
    if (result != RT_EOK)
    {
        return (int)result;
    }

    rt_kprintf(
        "[KEY][RAW] start, duration=%u ms, interval=%u ms\n",
        BUTTON_POLL_DURATION_MS,
        BUTTON_POLL_INTERVAL_MS);

    rt_kprintf("[KEY][RAW] initial KEY1=%d KEY2=%d\n",
               previous_key1,
               previous_key2);

    //开始轮询测试
    for (sample = 0;
         sample < BUTTON_POLL_SAMPLE_COUNT;
         sample++)
    {
        result = button_port_read_raw(BUTTON_PORT_KEY1,
                                      &current_key1);
        if (result != RT_EOK)
        {
            return (int)result;
        }

        result = button_port_read_raw(BUTTON_PORT_KEY2,
                                      &current_key2);
        if (result != RT_EOK)
        {
            return (int)result;
        }

        if (current_key1 != previous_key1)
        {
            rt_kprintf("[KEY][RAW] tick=%lu KEY1 %d->%d\n",
                       (unsigned long)rt_tick_get(),
                       previous_key1,
                       current_key1);

            previous_key1 = current_key1;
            key1_transitions++;
        }

        if (current_key2 != previous_key2)
        {
            rt_kprintf("[KEY][RAW] tick=%lu KEY2 %d->%d\n",
                       (unsigned long)rt_tick_get(),
                       previous_key2,
                       current_key2);

            previous_key2 = current_key2;
            key2_transitions++;
        }

        rt_thread_mdelay(BUTTON_POLL_INTERVAL_MS);
    }

    rt_kprintf(
        "[KEY][RAW] done, KEY1 transitions=%lu, "
        "KEY2 transitions=%lu\n",
        (unsigned long)key1_transitions,
        (unsigned long)key2_transitions);

    return 0;

}

//按键事件发送函数
static void button_emit_event(button_port_context_t *context,button_event_type_t type,rt_uint32_t duration_ms)
{
    button_event_t event;
    rt_err_t result;

    if(context==RT_NULL)
    {
        return;
    }

    event.key=context->key;
    event.type=type;
    event.tick=rt_tick_get();
    event.duration_ms=duration_ms;
    
    if(g_event_handler==RT_NULL)
    {
        return;

    }
    result = g_event_handler(&event,g_event_user_data);
    if (result != RT_EOK)
    {
        context->event_fail_count++;
    }

}

// GPIO中断回调函数
static void button_gpio_isr(void *args)
{
    button_port_context_t *context = (button_port_context_t *)args;
    rt_err_t result;

    if(context == RT_NULL)
    {
        return;
    }

    context->last_irq_level=(rt_pin_read(context->pin)==PIN_LOW)?0U:1U; //上次中断触发时的电平
    context->last_irq_tick=rt_tick_get();   //上次中断触发时的系统节拍
    context->irq_count++;                   //GPIO原始中断次数
   
    result = rt_timer_start(&context->debounce_timer);
    if (result != RT_EOK)
    {
        context->notify_fail_count++;   //软定时器启动失败次数
    }

}




// 消抖定时器回调函数
static void button_debounce_timeout(void *parameter)
{
    button_port_context_t *context = (button_port_context_t *)parameter;
    rt_uint8_t current_level;
    rt_err_t result;

    if(context == RT_NULL)
    {
        return;
    }

    current_level = (rt_pin_read(context->pin) == PIN_LOW) ? 0U : 1U;
    if(current_level==context->stable_level)
    {
        return; //电平未变化，无需处理
    }
    context->stable_level=current_level;//更新稳定电平
    context->debounce_timeout_count++;  //timer线程执行确认回调次数
    /*
    * 如果按键在系统启动时已经处于按下状态，
    * 则忽略这次按住过程，直到检测到稳定释放。
    */
    if (context->suppress_until_release == RT_TRUE)
    {
        if (current_level == 0U)
        {
            context->suppress_until_release = RT_FALSE;
            context->startup_release_count++;

            context->press_tick = 0U;
            context->long_reported = RT_FALSE;

            rt_kprintf(
                "[KEY][INIT] key=%d startup hold released, "
                "normal detection enabled\n",
                context->key);
        }

        return;
    }

    //稳定按下处理
    if(current_level==1U)
    {
        context->press_tick=rt_tick_get();  //按键按下时的系统节拍
        context->long_reported=RT_FALSE;    //本次按下还未产生长按
        context->press_count++;             //按下次数
        button_emit_event(context,BUTTON_EVENT_DOWN,0); //稳定按下事件

        result=rt_timer_start(&context->long_timer);
        if (result != RT_EOK)
        {
            rt_kprintf("[KEY][DEBOUNCE][ERR] key=%d long_timer start failed: %d\n",
                    context->key,
                    (int)result);
        }
        rt_kprintf("[KEY][DOWN] key=%d tick=%lu\n",
                   context->key,
                   (unsigned long)context->press_tick);
    }
    else if(current_level==0U)//稳定释放处理
    {
        rt_tick_t release_tick=rt_tick_get();
        rt_tick_t duration_ticks=release_tick-context->press_tick;
        rt_uint32_t duration_ms=duration_ticks*1000/RT_TICK_PER_SECOND;
        rt_tick_t long_press_ticks =rt_tick_from_millisecond(BUTTON_LONG_PRESS_MS);

        context->release_count++;
        
        if (context->long_reported == RT_FALSE)
        {
            result = rt_timer_stop(&context->long_timer);

            if (duration_ticks >= long_press_ticks)
            {
                /*
                * 已达到长按阈值，但长按定时器回调尚未报告，
                * 由释放路径补报一次LONG。
                *
                * 此时rt_timer_stop()失败不一定是异常，
                * 定时器可能已经到期并自动停止。
                */
                context->long_reported = RT_TRUE;
                context->long_count++;

                button_emit_event(context,BUTTON_EVENT_LONG,duration_ms);
            }
            else
            {
                /*
                * 短按在长按阈值到达前释放，此时long_time论上仍应处于活动状态。停止失败说明内部定时器状态与预期不一致，需要记录。
                */
                if (result != RT_EOK)
                {
                    context->long_timer_stop_fail_count++;

                    rt_kprintf(
                        "[KEY][ERR] key=%d short release, "
                        "long timer stop failed: %d\n",
                        context->key,
                        (int)result);
                }

                /*
                * 短按由稳定电平和持续时间共同确认。
                * 即使停止定时器失败，也不能丢弃该短按事件。
                */
                context->short_count++;

                button_emit_event(context,BUTTON_EVENT_SHORT,duration_ms);
            }
        }
        /*
        * 保持项目当前事件顺序：
        *
        * 短按：DOWN -> SHORT -> UP
        * 长按：DOWN -> LONG  -> UP
        */
        button_emit_event(context,BUTTON_EVENT_UP,duration_ms);
        context->press_tick = 0;
        context->long_reported = RT_FALSE;
        rt_kprintf("[KEY][UP] key=%d tick=%lu duration=%lu ms\n",
                   context->key,
                   (unsigned long)release_tick,
                   (unsigned long)duration_ms);

    }

}

//长按定时器回调函数
static void button_long_timeout(void *parameter)
{

    button_port_context_t *context = (button_port_context_t *)parameter;
    rt_uint8_t current_level;

    if(context == RT_NULL)
    {
        return;
    }
    
    current_level = (rt_pin_read(context->pin) == PIN_LOW) ? 0U : 1U;

    //判断是否发生长按
    if(current_level==context->stable_level && current_level==1U&&context->long_reported==RT_FALSE)
    {
        context->long_reported=RT_TRUE;  //本次按下已经产生长按
        context->long_count++;            //长按次数
        button_emit_event(context,BUTTON_EVENT_LONG,BUTTON_LONG_PRESS_MS);
        rt_kprintf("[KEY][LONG] key=%d long_press tick=%lu duration=%u\n",
                   context->key,
                   (unsigned long)rt_tick_get(),
                   BUTTON_LONG_PRESS_MS
                );
    }


}


//-------------------------------------------------测试-------------------------------------------------
static int button_stats(int argc, char **argv);

typedef struct
{
    rt_uint8_t stable_level;

    rt_uint32_t irq_count;
    rt_uint32_t notify_fail_count;
    rt_uint32_t stable_transition_count;

    rt_uint32_t press_count;
    rt_uint32_t release_count;
    rt_uint32_t short_count;
    rt_uint32_t long_count;
    rt_uint32_t event_fail_count;
    rt_uint32_t long_timer_stop_fail_count;
} button_stats_snapshot_t;


static int button_stats(int argc, char **argv)
{
    button_stats_snapshot_t snapshot[BUTTON_PORT_KEY_COUNT];
    rt_base_t interrupt_level;
    rt_uint32_t key;

    (void)argc;
    (void)argv;

    if (g_button_initialized == RT_FALSE)
    {
        rt_kprintf("[KEY][STATS][ERR] button not initialized\n");
        return -RT_ERROR;
    }

    /*
     * 暂时关闭本核中断，只复制少量计数。
     * 不可以在关中断期间打印日志。
     */
    interrupt_level = rt_hw_interrupt_disable();

    for (key = 0; key < BUTTON_PORT_KEY_COUNT; key++)
    {
        snapshot[key].stable_level =
            g_button_context[key].stable_level;

        snapshot[key].irq_count =
            g_button_context[key].irq_count;

        snapshot[key].notify_fail_count =
            g_button_context[key].notify_fail_count;

        snapshot[key].stable_transition_count =
            g_button_context[key].debounce_timeout_count;

        snapshot[key].press_count =
            g_button_context[key].press_count;

        snapshot[key].release_count =
            g_button_context[key].release_count;

        snapshot[key].short_count =
            g_button_context[key].short_count;

        snapshot[key].long_count =
            g_button_context[key].long_count;

        snapshot[key].event_fail_count=
            g_button_context[key].event_fail_count;

        snapshot[key].long_timer_stop_fail_count=
            g_button_context[key].long_timer_stop_fail_count;
    }

    rt_hw_interrupt_enable(interrupt_level);

    for (key = 0; key < BUTTON_PORT_KEY_COUNT; key++)
    {
        rt_kprintf
        (
            "[KEY][STATS] KEY%u level=%u irq=%lu stable=%lu "
            "timer_fail=%lu event_fail=%lu stop_fail=%lu "
            "down=%lu up=%lu short=%lu long=%lu\n",

            (unsigned int)(key + 1U),
            (unsigned int)snapshot[key].stable_level,
            (unsigned long)snapshot[key].irq_count,
            (unsigned long)snapshot[key].stable_transition_count,

            (unsigned long)snapshot[key].notify_fail_count,
            (unsigned long)snapshot[key].event_fail_count,
            (unsigned long)snapshot[key].long_timer_stop_fail_count,

            (unsigned long)snapshot[key].press_count,
            (unsigned long)snapshot[key].release_count,
            (unsigned long)snapshot[key].short_count,
            (unsigned long)snapshot[key].long_count
        );
    }

    return RT_EOK;
}


#ifdef RT_USING_FINSH
MSH_CMD_EXPORT(button_poll_test,
               poll raw KEY1 and KEY2 levels for 20 seconds);
MSH_CMD_EXPORT(button_stats,
               show button interrupt and event statistics);
#endif
