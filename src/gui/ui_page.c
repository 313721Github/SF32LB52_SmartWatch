#include "ui_page.h"
#include "rthw.h"
#include "lvgl.h"
#include "lvsf_font.h"
#include "system_diag.h"
#include "sensor_service.h"

#define UI_SENSOR_DIAG_REFRESH_MS       500U        //诊断页面初始刷新频率固定为2 Hz
#define UI_SENSOR_DIAG_RECOVERY_MS      1000U       //从失败状态恢复到READY后显示1秒RECOVERED提示


/*
 * UI最终显示状态。
 * 该状态只用于把sensor服务状态映射为文字和颜色，不是第二套sensor状态机。
 */
typedef enum
{
    UI_SENSOR_DISPLAY_WAIT = 0,
    UI_SENSOR_DISPLAY_READY,
    UI_SENSOR_DISPLAY_RECOVERED,
    UI_SENSOR_DISPLAY_WARN,
    UI_SENSOR_DISPLAY_STALE,
    UI_SENSOR_DISPLAY_OFFLINE
} ui_sensor_display_state_t;
typedef struct
{
    lv_obj_t *root_screen;

    lv_obj_t *watchface_page;
    lv_obj_t *watchface_title;
    lv_obj_t *watchface_time;
    lv_obj_t *watchface_status;

    lv_obj_t *menu_page;
    lv_obj_t *menu_title;
    lv_obj_t *menu_items;

    lv_obj_t *system_info_page;
    lv_obj_t *system_info_title;
    lv_obj_t *system_info_text;

    /*
     * 传感器诊断页面及其LVGL对象。
     * 这些指针只能由UI线程创建、使用和清理。
     * sensor线程不得获取或保存其中任何一个对象指针。
     */
    lv_obj_t *sensor_diag_page;
    lv_obj_t *sensor_diag_title;
    lv_obj_t *sensor_diag_imu_text;
    lv_obj_t *sensor_diag_mmc_text;
    lv_obj_t *sensor_diag_als_text;
    lv_obj_t *sensor_diag_footer;

    lv_timer_t *sensor_diag_timer;              //诊断刷新timer由UI页面上下文拥有
    ui_sensor_diag_stats_t sensor_diag_stats;   //诊断刷新统计由UI线程写入
    sensor_device_state_t sensor_diag_previous_state[SENSOR_DEVICE_COUNT];  // previous_state只用于识别“失败状态进入READY”的恢复边沿
    rt_bool_t sensor_diag_previous_valid[SENSOR_DEVICE_COUNT];              //UI是否已经观察过该器件的状态
    rt_tick_t sensor_diag_recovery_until[SENSOR_DEVICE_COUNT];              //保存每颗器件的RECOVERED提示截止时间
    rt_bool_t sensor_diag_has_publish_seq;                                  //第一次取得快照前没有可比较的publish_seq

    ui_page_action_handler_t action_handler;
    void *action_user_data;

    ui_page_stats_t stats;

} ui_page_context_t;
static ui_page_context_t g_ui_page_context =
{
    .stats =
    {
        .created = RT_FALSE,
        .current_page = UI_PAGE_COUNT
    }
};

static const ui_action_t g_ui_nav_home = UI_ACTION_HOME;
static const ui_action_t g_ui_nav_back = UI_ACTION_BACK;
static const ui_action_t g_ui_nav_next = UI_ACTION_NEXT;

//公开函数声明
rt_err_t ui_page_create(void);                              //创建ui界面
rt_err_t ui_page_handle_event(const ui_event_t *event);     //ui事件处理函数
rt_err_t ui_page_get_stats(ui_page_stats_t *stats);         //获取ui页面统计信息
rt_err_t ui_page_register_action_handler(ui_page_action_handler_t handler,void *user_data); //注册页面动作处理函数
rt_err_t ui_page_get_sensor_diag_stats(ui_sensor_diag_stats_t *stats);  //获取传感器诊断页面刷新统计
//私有函数声明
static rt_err_t ui_page_create_watchface(lv_obj_t *parent);         //创建表盘页面
static rt_err_t ui_page_create_menu(lv_obj_t *parent);              //创建菜单页面
static rt_err_t ui_page_create_system_info(lv_obj_t *parent);       //创建系统信息页面
static rt_err_t ui_page_create_sensor_diag(lv_obj_t *parent);       //创建传感器诊断页面
static lv_obj_t *ui_page_create_sensor_diag_label(lv_obj_t *parent,const char *initial_text,
                                                lv_coord_t y_offset,lv_color_t text_color);// 创建诊断页使用的多行文本标签
static lv_obj_t *ui_page_create_nav_button(lv_obj_t *parent,const char *text,
    lv_align_t align,lv_coord_t x_offset,const ui_action_t *action);//创建最小导航按钮

static rt_err_t ui_page_apply_visibility(ui_page_id_t target);      //ui界面可见性函数
static rt_err_t ui_page_show(ui_page_id_t target);                  //切换到指定页面

static rt_bool_t ui_page_id_is_valid(ui_page_id_t page);            //检查页面id是否有效
static ui_page_id_t ui_page_next_id(ui_page_id_t current);          //获取下一个页面ID
static ui_page_id_t ui_page_back_id(ui_page_id_t current);          //获取返回页面ID
static void ui_page_nav_event_cb(lv_event_t *event);                //LVGL点击回调
static void ui_page_cleanup(void);                                  //ui界面统一清理函数
static void ui_page_stat_increment(rt_uint32_t *counter);           //线程安全地增加一个32位统计计数

//传感器诊断页面相关函数
static void ui_page_sensor_diag_timer_cb(lv_timer_t *timer);  //500ms LVGL定时器回调，仅在诊断页可见时触发快照刷新
static rt_err_t ui_page_refresh_sensor_diag(void);            //复制一致快照，完成三器件渲染并记录刷新统计
static void ui_page_render_imu(const sensor_snapshot_t *snapshot,
                                rt_tick_t now_tick);          //将IMU状态、age、错误和六轴物理量更新到IMU标签
static void ui_page_render_mmc(const sensor_snapshot_t *snapshot,
                                rt_tick_t now_tick);          //将MMC状态、age、错误和三轴磁场值更新到MMC标签
static void ui_page_render_als(const sensor_snapshot_t *snapshot,
                                rt_tick_t now_tick);          //将ALS状态、age、错误、双通道和照度估算更新到ALS标签
static ui_sensor_display_state_t ui_page_sensor_display_state(sensor_device_id_t device,
    const sensor_device_status_t *status,rt_tick_t now_tick); //把服务状态映射为UI状态，并识别失败到READY的恢复边沿

static const char *ui_page_sensor_state_text(ui_sensor_display_state_t state,
                                            rt_bool_t has_last_good);  //根据UI状态和last-good有效性返回状态显示文字

static lv_color_t ui_page_sensor_state_color(ui_sensor_display_state_t state);  //根据最终UI状态返回对应的诊断文字颜色

static rt_uint32_t ui_page_ticks_to_ms(rt_tick_t ticks);     //把系统tick安全换算为用于诊断显示的毫秒数

static rt_tick_t ui_page_ms_to_ticks(rt_uint32_t milliseconds); //把毫秒安全换算为非零系统tick数

static rt_bool_t ui_page_tick_reached(rt_tick_t now_tick,rt_tick_t deadline_tick); //使用回绕安全的tick差值判断截止时间是否到达

//---------------------------------------------------------------公开函数---------------------------------------------------------------

/*
 * 简介：创建四页最小UI
 * 输入：无
 * 返回值：
 *     RT_EOK：创建成功
 *     -RT_EBUSY：已经创建，或上下文中存在未清理对象
 *     -RT_ENOMEM：LVGL对象创建失败
 *     其他负错误码：子页面创建或可见性设置失败
 *
 * 约束：
 * 1. 只能由UI线程调用；
 * 2. 所有页面都以root_screen为共同父对象；
 * 3. 任一创建步骤失败，都删除整个对象树；
 * 4. 最后才提交created和current_page状态。
 */
rt_err_t ui_page_create(void)
{

    rt_base_t level;
    rt_err_t result;

    ui_page_stat_increment(&g_ui_page_context.stats.create_attempt);

    if(g_ui_page_context.stats.created==RT_TRUE)
    {
        ui_page_stat_increment(&g_ui_page_context.stats.create_fail);
        return -RT_EBUSY;
    }

    /*
     * created为false但仍存在对象句柄，说明上下文不一致。
     * 此时停止创建，不覆盖原来的根对象指针。
     */
    if ((g_ui_page_context.root_screen != RT_NULL) ||
        (g_ui_page_context.watchface_page != RT_NULL) ||
        (g_ui_page_context.watchface_title != RT_NULL) ||
        (g_ui_page_context.watchface_time != RT_NULL) ||
        (g_ui_page_context.watchface_status != RT_NULL) ||
        (g_ui_page_context.menu_page != RT_NULL) ||
        (g_ui_page_context.menu_title != RT_NULL) ||
        (g_ui_page_context.menu_items != RT_NULL) ||
        (g_ui_page_context.system_info_page != RT_NULL) ||
        (g_ui_page_context.system_info_title != RT_NULL) ||
        (g_ui_page_context.system_info_text != RT_NULL) ||
        (g_ui_page_context.sensor_diag_page != RT_NULL) ||
        (g_ui_page_context.sensor_diag_title != RT_NULL) ||
        (g_ui_page_context.sensor_diag_imu_text != RT_NULL) ||
        (g_ui_page_context.sensor_diag_mmc_text != RT_NULL) ||
        (g_ui_page_context.sensor_diag_als_text != RT_NULL) ||
        (g_ui_page_context.sensor_diag_footer != RT_NULL) ||
        (g_ui_page_context.sensor_diag_timer != RT_NULL))
    {
        ui_page_stat_increment(
            &g_ui_page_context.stats.create_fail);

        return -RT_EBUSY;
    }

    /*
     * 创建所有页面的共同根screen。
     * 后续清理时只需删除该对象，即可递归删除所有子对象。
     */
    g_ui_page_context.root_screen=lv_obj_create(NULL);
    if(g_ui_page_context.root_screen==NULL)
    {
        result = -RT_ENOMEM;
        goto create_failed;
    }

    result=ui_page_create_watchface(g_ui_page_context.root_screen);
    if(result!=RT_EOK)
    {
        goto create_failed;
    }

    result=ui_page_create_menu(g_ui_page_context.root_screen);
    if(result!=RT_EOK)
    {
        goto create_failed;
    }

    result=ui_page_create_system_info(g_ui_page_context.root_screen);
    if(result!=RT_EOK)
    {
        goto create_failed;
    }

    /*
     * 诊断页面进入与其他页面相同的一次创建、隐藏复用生命周期。
     * 这里暂时只创建静态占位符，不访问sensor服务。
     */
    result = ui_page_create_sensor_diag(g_ui_page_context.root_screen);
    if (result != RT_EOK)
    {
        goto create_failed;
    }
    /*
     * 在装载screen之前设置初始可见性。
     * 四个页面已经完整创建，因此不会出现半切换状态。
     */
    result = ui_page_apply_visibility(UI_PAGE_WATCHFACE);
    if(result!=RT_EOK)
    {
        goto create_failed;
    }

    /*
     * 页面树构造完成后，再装载为活动screen，
     * 防止用户看到创建过程中的中间画面。
     */

    lv_scr_load(g_ui_page_context.root_screen);

    level = rt_hw_interrupt_disable();
    g_ui_page_context.stats.current_page=UI_PAGE_WATCHFACE;
    g_ui_page_context.stats.created = RT_TRUE;
    g_ui_page_context.stats.create_success++;
    rt_hw_interrupt_enable(level);

    //打印：[UI][PAGE] initialized current=WATCHFACE
    rt_kprintf("[UI][PAGE] initialized current=WATCHFACE\n");
    return RT_EOK;
create_failed:

    /*
     * 保存的result不能被cleanup覆盖。
     * cleanup删除root_screen及其所有子对象，但保留累计统计。
     */
    ui_page_cleanup();

    ui_page_stat_increment(
        &g_ui_page_context.stats.create_fail);

    return result;
}

/*
 * 简介：处理UI线程收到的页面事件
 * 输入：
 *     event：UI事件
 * 返回值：
 *     RT_EOK：事件处理成功
 *     -RT_EINVAL：事件为空、事件类型非法或页面状态非法
 *     -RT_EBUSY：页面尚未完成创建
 *     -RT_ERROR：目标LVGL对象异常
 *     其他负错误码：页面切换函数返回的原始错误
 *
 * 约束：
 * 1. 本函数只能由UI线程调用；
 * 2. 其他线程只能向UI队列发布事件；
 * 3. 本函数不得隐藏ui_page_show()的错误。
 */
rt_err_t ui_page_handle_event(const ui_event_t *event)
{
    ui_page_id_t target_page;
    if(event==RT_NULL)
    {
        return -RT_EINVAL;
    }

    //检查ui界面是否创建
    if(g_ui_page_context.stats.created==RT_FALSE)
    {

        return -RT_EBUSY;
    }

    //处理ui事件
    switch (event->type)
    {
    case UI_ACTION_HOME:
        /* 任意页面返回表盘；已在表盘时属于合法noop。 */
        return ui_page_show(UI_PAGE_WATCHFACE);
    case UI_ACTION_NEXT:
        target_page = ui_page_next_id(
            g_ui_page_context.stats.current_page);

        if (ui_page_id_is_valid(target_page) != RT_TRUE)
        {
            return -RT_EINVAL;
        }

        return ui_page_show(target_page);
    case UI_ACTION_BACK:
        /*
         * 这里必须使用ui_page_back_id()：
         * SENSOR_DIAG -> SYSTEM_INFO
         * SYSTEM_INFO -> MENU
         * MENU        -> WATCHFACE
         * WATCHFACE   -> WATCHFACE
         */
        target_page = ui_page_back_id(
            g_ui_page_context.stats.current_page);

        if (ui_page_id_is_valid(target_page) != RT_TRUE)
        {
            return -RT_EINVAL;
        }

        return ui_page_show(target_page);
    case UI_ACTION_REFRESH_STATUS:

        /*
         * 状态刷新只修改表盘状态标签，
         * 不切换页面，也不增加switch统计。
         */
        if (g_ui_page_context.watchface_status == RT_NULL)
        {
            return -RT_ERROR;
        }

        lv_label_set_text_fmt(
            g_ui_page_context.watchface_status,
            "STATUS: %u",
            (unsigned int)event->value);

        return RT_EOK;

    case UI_ACTION_NONE:
    default:
        return -RT_EINVAL;
    }

    return RT_EOK;
}


/*
 * 简介：获取UI页面统计快照
 * 输入：
 *     stats：用于接收统计信息的输出缓冲区
 * 返回值：
 *     RT_EOK：获取成功
 *     -RT_EINVAL：输出指针为空
 *
 * 说明：
 * 1. 只复制值类型的统计信息；
 * 2. 不返回任何LVGL对象指针；
 * 3. 使用短临界区保证关联字段来自同一个状态快照；
 * 4. 临界区内不执行日志、内存申请或LVGL操作。
 */
rt_err_t ui_page_get_stats(ui_page_stats_t *stats)
{
    rt_base_t level;

    if (stats == RT_NULL)
    {
        return -RT_EINVAL;
    }

    level = rt_hw_interrupt_disable();

    *stats = g_ui_page_context.stats;

    rt_hw_interrupt_enable(level);

    return RT_EOK;
}
/*
 * 简介：复制传感器诊断页面刷新统计
 * 输入：stats：调用者持有的输出缓冲区
 * 返回值：
 *     RT_EOK：复制成功
 *     -RT_EINVAL：输出指针为空
 *
 * 并发边界：
 * 1. 只复制值类型统计；
 * 2. 不返回LVGL对象、timer或sensor快照指针；
 * 3. 临界区内不执行日志、LVGL、I2C或内存申请。
 */
rt_err_t ui_page_get_sensor_diag_stats(ui_sensor_diag_stats_t *stats)
{
    rt_base_t level;

    if (stats == RT_NULL)
    {
        return -RT_EINVAL;
    }

    level = rt_hw_interrupt_disable();

    *stats =g_ui_page_context.sensor_diag_stats;

    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

/*
 * 简介：注册页面动作处理函数
 * 输入：
 *     handler：页面动作处理函数
 *     user_data：传递给动作处理函数的用户数据
 * 返回值：
 *     RT_EOK：注册成功
 *     -RT_EINVAL：handler为空
 *     -RT_EBUSY：页面已经创建，不允许替换回调
 */
rt_err_t ui_page_register_action_handler(
    ui_page_action_handler_t handler,
    void *user_data)
{
    rt_base_t level;

    if (handler == RT_NULL)
    {
        return -RT_EINVAL;
    }

    /*
     * 页面已经创建后不允许替换回调，
     * 防止LVGL对象运行期间改变行为所有者。
     */
    if (g_ui_page_context.stats.created == RT_TRUE)
    {
        return -RT_EBUSY;
    }

    level = rt_hw_interrupt_disable();

    g_ui_page_context.action_handler = handler;
    g_ui_page_context.action_user_data = user_data;

    rt_hw_interrupt_enable(level);

    return RT_EOK;
}
//---------------------------------------------------------------私有函数---------------------------------------------------------------


/*
 * 简介：创建最小表盘页面
 * 输入：
 *     parent：页面的父对象，当前应为根screen
 * 返回值：
 *     RT_EOK：创建成功
 *     -RT_EINVAL：父对象无效
 *     -RT_EBUSY：表盘页面已经创建
 *     -RT_ENOMEM：LVGL对象创建失败
 *
 * 说明：
 * 1. 本函数只能由UI线程调用；
 * 2. 本函数只负责创建表盘对象，不负责切换当前页面；
 * 3. 创建失败时删除page容器，LVGL会递归删除其子对象；
 * 4. 全部创建成功后，才把局部对象句柄写入全局上下文。
 */
static rt_err_t ui_page_create_watchface(lv_obj_t *parent)
{
    lv_obj_t *page;
    lv_obj_t *title;
    lv_obj_t *time_label;
    lv_obj_t *status_label;
    lv_obj_t *nav_button;

    /* 父对象必须是已经创建成功的根screen。 */
    if (parent == RT_NULL)
    {
        return -RT_EINVAL;
    }

    /*
     * 防止重复创建。
     * 页面采用“一次创建、隐藏保留”策略，不能在每次切换时重新创建。
     */
    if ((g_ui_page_context.watchface_page != RT_NULL) ||
        (g_ui_page_context.watchface_title != RT_NULL) ||
        (g_ui_page_context.watchface_time != RT_NULL) ||
        (g_ui_page_context.watchface_status != RT_NULL))
    {
        return -RT_EBUSY;
    }

    page = RT_NULL;
    title = RT_NULL;
    time_label = RT_NULL;
    status_label = RT_NULL;
    nav_button = RT_NULL;
    /*
     * 1. 创建表盘页面容器。
     * 三个页面都应是root screen下相互独立的container。
     */
    page = lv_obj_create(parent);
    if (page == RT_NULL)
    {
        return -RT_ENOMEM;
    }
    /*
     * 2. 页面铺满根screen并居中。
     * 使用百分比尺寸，避免在页面模块中重复硬编码390×450。
     */
    lv_obj_set_size(page, LV_PCT(100), LV_PCT(100));
    lv_obj_center(page);

    /*
     * 3. 页面只作静态显示，关闭默认滚动能力。
     * 同时去掉container默认边框、圆角和内边距。
     */
    lv_obj_clear_flag(page, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_style_border_width(
        page,
        0,
        LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_radius(
        page,
        0,
        LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_pad_all(
        page,
        0,
        LV_PART_MAIN | LV_STATE_DEFAULT);
    /*
     * 4. 设置深色不透明背景。
     * 当前不使用图片、渐变或复杂表盘资源。
     */
    lv_obj_set_style_bg_color(
        page,
        lv_color_hex(0x10151C),
        LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_opa(
        page,
        LV_OPA_COVER,
        LV_PART_MAIN | LV_STATE_DEFAULT);
    /*
     * 5. 创建顶部标题。
     */
    title = lv_label_create(page);
    if (title == RT_NULL)
    {
        goto create_failed;
    }

    lv_label_set_text(title, "WATCH");
   /*
     * 继续使用已经实机验证过的SiFli字体接口。
     * 不恢复直接使用lv_font_montserrat_xx的旧路径。
     */
    lv_ext_set_local_font(
        title,
        FONT_TITLE,
        LV_COLOR_WHITE);
    lv_obj_align(
        title,
        LV_ALIGN_TOP_MID,
        0,
        36);
    /*
     * 6. 创建模拟时间。
     * 10:08只是静态占位值，不表示已经接入RTC。
     */
    time_label = lv_label_create(page);
    if (time_label == RT_NULL)
    {
        goto create_failed;
    }

    lv_label_set_text(time_label, "10:08");

    lv_ext_set_local_font(
        time_label,
        FONT_HUGE,
        LV_COLOR_WHITE);

    lv_obj_align(
        time_label,
        LV_ALIGN_CENTER,
        0,
        -10);
 /*
     * 7. 创建状态标签。
     * 初始明确显示SIMULATED，防止用户误以为当前时间来自RTC。
     * 任务8的REFRESH_STATUS事件后续可以更新该标签。
     */
    status_label = lv_label_create(page);
    if (status_label == RT_NULL)
    {
        goto create_failed;
    }

    lv_label_set_text(status_label, "SIMULATED");

    lv_ext_set_local_font(
        status_label,
        FONT_NORMAL,
        lv_color_hex(0xA8B0BA));

    lv_obj_align(
        status_label,
        LV_ALIGN_CENTER,
        0,
        58);

    /*
    * 8. 创建表盘页NEXT按钮。
    *
    * 点击回调只上报UI_ACTION_NEXT，
    * 不允许在LVGL回调中直接调用ui_page_show()。
    */
    nav_button = ui_page_create_nav_button(
        page,
        "NEXT",
        LV_ALIGN_BOTTOM_RIGHT,
        -18,
        &g_ui_nav_next);

    if (nav_button == RT_NULL)
    {
        goto create_failed;
    }
    /*
     * 9. 全部对象创建成功后再写入全局上下文。
     * 这样其他函数不会观察到一个只创建了一半的页面。
     */
    g_ui_page_context.watchface_page = page;
    g_ui_page_context.watchface_title = title;
    g_ui_page_context.watchface_time = time_label;
    g_ui_page_context.watchface_status = status_label;

    return RT_EOK;


create_failed:

    /*
     * page是所有表盘对象的父对象。
     * 删除page会递归删除已经创建的title、time和status，
     * 因此不能再逐个删除子对象。
     */
    if (page != RT_NULL)
    {
        lv_obj_del(page);
    }

    /*
     * 全局上下文尚未写入这些局部对象，
     * 这里仍显式保持空值，便于后续检查和统一清理。
     */
    g_ui_page_context.watchface_page = RT_NULL;
    g_ui_page_context.watchface_title = RT_NULL;
    g_ui_page_context.watchface_time = RT_NULL;
    g_ui_page_context.watchface_status = RT_NULL;

    return -RT_ENOMEM;
}

/*
 * 简介：创建最小菜单页面
 * 输入：
 *     parent：页面的父对象，当前应为根screen
 * 返回值：
 *     RT_EOK：创建成功
 *     -RT_EINVAL：父对象无效
 *     -RT_EBUSY：菜单页面已经创建
 *     -RT_ENOMEM：LVGL对象创建失败
 *
 * 说明：
 * 1. 菜单内容仍为静态展示，不执行真实应用业务；
 * 2. 只创建任务11要求的HOME、BACK和NEXT导航按钮；
 * 3. 不加入动画、图片或完整应用入口。
 */
static rt_err_t ui_page_create_menu(lv_obj_t *parent)
{
    lv_obj_t *page;
    lv_obj_t *title;
    lv_obj_t *items;
    lv_obj_t *nav_button;

    /* 父对象必须是有效的根screen。 */
    if (parent == RT_NULL)
    {
        return -RT_EINVAL;
    }

    /*
     * 页面采用“一次创建、隐藏保留”策略。
     * 如果任意菜单对象已经存在，就拒绝重复创建。
     */
    if ((g_ui_page_context.menu_page != RT_NULL) ||
        (g_ui_page_context.menu_title != RT_NULL) ||
        (g_ui_page_context.menu_items != RT_NULL))
    {
        return -RT_EBUSY;
    }

    page = RT_NULL;
    title = RT_NULL;
    items = RT_NULL;
    nav_button = RT_NULL;
    /*
     * 1. 创建菜单页面container。
     * 该container与表盘页、系统信息页同为root screen的直接子对象。
     */
    page = lv_obj_create(parent);
    if (page == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    /*
     * 2. 页面铺满根screen。
     */
    lv_obj_set_size(page, LV_PCT(100), LV_PCT(100));
    lv_obj_center(page);

    /*
     * 3. 当前菜单不支持滚动和点击。
     * 后续真正实现应用菜单时，再单独设计输入和焦点行为。
     */
    lv_obj_clear_flag(
        page,
        LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    /*
     * 去掉container默认边框、圆角和内边距，
     * 使它表现为一个完整页面。
     */
    lv_obj_set_style_border_width(
        page,
        0,
        LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_radius(
        page,
        0,
        LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_pad_all(
        page,
        0,
        LV_PART_MAIN | LV_STATE_DEFAULT);

    /*
     * 4. 设置菜单页背景。
     * 使用纯色背景，不加入图片、渐变或动画。
     */
    lv_obj_set_style_bg_color(
        page,
        lv_color_hex(0x18202A),
        LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_opa(
        page,
        LV_OPA_COVER,
        LV_PART_MAIN | LV_STATE_DEFAULT);
    /*
     * 5. 创建菜单标题。
     */
    title = lv_label_create(page);
    if (title == RT_NULL)
    {
        goto create_failed;
    }

    lv_label_set_text(title, "MENU");
    /*
     * 使用已经验证过的SiFli字体接口，
     * 不直接使用lv_font_montserrat_xx。
     */
    lv_ext_set_local_font(
        title,
        FONT_TITLE,
        LV_COLOR_WHITE);

    lv_obj_align(
        title,
        LV_ALIGN_TOP_MID,
        0,
        36);

    /*
     * 6. 使用一个多行label显示三个静态入口。
     * 当前这些文字不对应按钮，也不会触发真实业务。
     */
    items = lv_label_create(page);
    if (items == RT_NULL)
    {
        goto create_failed;
    }

    lv_label_set_text(
        items,
        "System Info\n"
        "Settings\n"
        "Diagnostics");
 /*
     * 设置label宽度和换行方式。
     * 即使后续字符串稍微变长，也不会直接越过页面边界。
     */
    lv_obj_set_width(items, LV_PCT(85));
    lv_label_set_long_mode(items, LV_LABEL_LONG_WRAP);
    /*
     * 菜单文字居中，并增加行间距，
     * 让三个静态入口在视觉 上容易区分。
     */
    lv_obj_set_style_text_align(
        items,
        LV_TEXT_ALIGN_CENTER,
        LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_text_line_space(
        items,
        18,
        LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_ext_set_local_font(
        items,
        FONT_NORMAL,
        lv_color_hex(0xD8DEE8));

    lv_obj_align(
        items,
        LV_ALIGN_CENTER,
        0,
        20);

    /*
    * 7. 创建菜单页HOME按钮。
    * HOME固定在左侧，保持三个页面的触摸位置一致。
    */
    nav_button = ui_page_create_nav_button(
        page,
        "HOME",
        LV_ALIGN_BOTTOM_LEFT,
        18,
        &g_ui_nav_home);

    if (nav_button == RT_NULL)
    {
        goto create_failed;
    }

    /*
    * 8. 创建菜单页BACK按钮。
    */
    nav_button = ui_page_create_nav_button(
        page,
        "BACK",
        LV_ALIGN_BOTTOM_MID,
        0,
        &g_ui_nav_back);

    if (nav_button == RT_NULL)
    {
        goto create_failed;
    }

    /*
    * 9. 创建菜单页NEXT按钮。
    */
    nav_button = ui_page_create_nav_button(
        page,
        "NEXT",
        LV_ALIGN_BOTTOM_RIGHT,
        -18,
        &g_ui_nav_next);

    if (nav_button == RT_NULL)
    {
        goto create_failed;
    }
    /*
     * 10. 全部创建成功后再提交到全局上下文。
     * 避免其他函数看到只创建了一部分的菜单页面。
     */
    g_ui_page_context.menu_page = page;
    g_ui_page_context.menu_title = title;
    g_ui_page_context.menu_items = items;

    return RT_EOK;

create_failed:

    /*
     * page拥有title和items。
     * 删除page会递归删除已创建的子对象。
     */
    if (page != RT_NULL)
    {
        lv_obj_del(page);
    }

    g_ui_page_context.menu_page = RT_NULL;
    g_ui_page_context.menu_title = RT_NULL;
    g_ui_page_context.menu_items = RT_NULL;

    return -RT_ENOMEM;

}


/*
 * 简介：创建最小系统信息页面
 * 输入：
 *     parent：页面的父对象，当前应为根screen
 * 返回值：
 *     RT_EOK：创建成功
 *     -RT_EINVAL：父对象或诊断字符串无效
 *     -RT_EBUSY：系统信息页面已经创建
 *     -RT_ERROR：无法取得系统诊断信息
 *     -RT_ENOMEM：LVGL对象创建失败
 *
 * 说明：
 * 1. 本函数只能由UI线程调用；
 * 2. 项目、板型、版本和提交信息全部来自system_diag；
 * 3. 不在UI模块中维护第二套版本字符串；
 * 4. lv_label_set_text_fmt()会把格式化结果保存到label文本中，
 *    页面不需要长期持有diag中的字符串指针；
 * 5. 全部对象创建成功后才写入全局上下文。
 */
static rt_err_t ui_page_create_system_info(lv_obj_t *parent)
{
    const system_diag_info_t *diag;
    lv_obj_t *page;
    lv_obj_t *title;
    lv_obj_t *info_text;
    lv_obj_t *nav_button;

    /* 父对象必须是有效的根screen。 */
    if (parent == RT_NULL)
    {
        return -RT_EINVAL;
    }

    /*
     * 页面采用“一次创建、隐藏保留”策略，
     * 因此不能重复创建系统信息页。
     */
    if ((g_ui_page_context.system_info_page != RT_NULL) ||
        (g_ui_page_context.system_info_title != RT_NULL) ||
        (g_ui_page_context.system_info_text != RT_NULL))
    {
        return -RT_EBUSY;
    }

    /*
     * 1. 从system_diag取得唯一的系统信息来源。
     * 不在本函数中重新定义项目版本、Git提交或SDK提交。
     */
    diag = system_diag_get_info();
    if (diag == RT_NULL)
    {
        return -RT_ERROR;
    }

    /*
     * 2. 检查本页面需要使用的字符串字段。
     * 防止后续格式化字符串时访问空指针。
     */
    if ((diag->project_name == RT_NULL) ||
        (diag->board_name == RT_NULL) ||
        (diag->project_version == RT_NULL) ||
        (diag->project_commit == RT_NULL) ||
        (diag->sdk_commit == RT_NULL))
    {
        return -RT_EINVAL;
    }

    page = RT_NULL;
    title = RT_NULL;
    info_text = RT_NULL;
    nav_button = RT_NULL;
    /*
     * 3. 创建系统信息页container。
     */
    page = lv_obj_create(parent);
    if (page == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    /*
     * 4. 页面铺满根screen并居中。
     */
    lv_obj_set_size(page, LV_PCT(100), LV_PCT(100));
    lv_obj_center(page);

    /*
     * 当前页面只显示静态信息，不实现滚动和点击。
     * 如果未来信息量超过一屏，应作为独立任务设计滚动行为。
     */
    lv_obj_clear_flag(
        page,
        LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    /*
     * 去除container默认边框、圆角和内边距，
     * 使其成为完整页面背景。
     */
    lv_obj_set_style_border_width(
        page,
        0,
        LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_radius(
        page,
        0,
        LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_pad_all(
        page,
        0,
        LV_PART_MAIN | LV_STATE_DEFAULT);

    /*
     * 5. 设置系统信息页背景。
     * 当前只使用纯色，不加入图片或动画资源。
     */
    lv_obj_set_style_bg_color(
        page,
        lv_color_hex(0x121820),
        LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_opa(
        page,
        LV_OPA_COVER,
        LV_PART_MAIN | LV_STATE_DEFAULT);
    /*
     * 6. 创建页面标题。
     */
    title = lv_label_create(page);
    if (title == RT_NULL)
    {
        goto create_failed;
    }

    lv_label_set_text(title, "SYSTEM INFO");

    /*
     * 使用任务7已经实机验证过的SiFli字体接口，
     * 不直接使用lv_font_montserrat_xx。
     */
    lv_ext_set_local_font(
        title,
        FONT_TITLE,
        LV_COLOR_WHITE);

    lv_obj_align(
        title,
        LV_ALIGN_TOP_MID,
        0,
        30);

    /*
     * 7. 创建一个多行label承载全部系统信息。
     * 使用单个label可以减少页面对象数量。
     */
    info_text = lv_label_create(page);
    if (info_text == RT_NULL)
    {
        goto create_failed;
    }

    /*
     * 宽度约为屏幕的88%，在390像素屏幕上约为343像素。
     * 超出宽度的文本按照单词或字符边界自动换行。
     */
    lv_obj_set_width(info_text, LV_PCT(88));
    lv_label_set_long_mode(info_text, LV_LABEL_LONG_WRAP);

    lv_obj_set_style_text_align(
        info_text,
        LV_TEXT_ALIGN_LEFT,
        LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_text_line_space(
        info_text,
        8,
        LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_ext_set_local_font(
        info_text,
        FONT_NORMAL,
        lv_color_hex(0xD8DEE8));

    /*
     * 8. 使用system_diag提供的真实字段构造页面文本。
     * 当前project_version即使仍显示phase1，也必须忠实显示，
     * 不能在UI模块中另写一个“phase2”版本字符串。
     */
    lv_label_set_text_fmt(
        info_text,
        "Project: %s\n"
        "Board: %s\n"
        "Version: %s\n"
        "Git: %s\n"
        "SDK: %s",
        diag->project_name,
        diag->board_name,
        diag->project_version,
        diag->project_commit,
        diag->sdk_commit);

    /*
     * 标题占用顶部区域，信息正文从标题下方开始。
     * 使用TOP_MID对齐可以配合固定宽度保持左右边距对称。
     */
    lv_obj_align(
        info_text,
        LV_ALIGN_TOP_MID,
        0,
        88);

    /*
    * 9. 创建系统信息页HOME按钮。
    */
    nav_button = ui_page_create_nav_button(
        page,
        "HOME",
        LV_ALIGN_BOTTOM_LEFT,
        18,
        &g_ui_nav_home);

    if (nav_button == RT_NULL)
    {
        goto create_failed;
    }

    /*
    * 10. 创建系统信息页BACK按钮。
    *
    * BACK始终位于底部中间，
    * 避免相同语义在不同页面出现位置变化。
    */
    nav_button = ui_page_create_nav_button(
        page,
        "BACK",
        LV_ALIGN_BOTTOM_MID,
        0,
        &g_ui_nav_back);

    if (nav_button == RT_NULL)
    {
        goto create_failed;
    }
    /*
     * 创建NEXT按钮，使触摸导航可以从系统信息页进入传感器诊断页。
     * 按钮只产生统一UI_ACTION_NEXT语义，
     * 页面回调不直接修改current_page。
     */
    nav_button = ui_page_create_nav_button(
        page,
        "NEXT",
        LV_ALIGN_BOTTOM_RIGHT,
        -18,
        &g_ui_nav_next);

    if (nav_button == RT_NULL)
    {
        goto create_failed;
    }
    /*
     * 11. 全部对象创建成功后再提交到全局上下文，
     * 避免其他函数看到创建了一半的页面。
     */
    g_ui_page_context.system_info_page = page;
    g_ui_page_context.system_info_title = title;
    g_ui_page_context.system_info_text = info_text;

    return RT_EOK;

create_failed:

    /*
     * page是title和info_text的父对象。
     * 删除page会递归删除所有已经创建的子对象。
     */
    if (page != RT_NULL)
    {
        lv_obj_del(page);
    }

    g_ui_page_context.system_info_page = RT_NULL;
    g_ui_page_context.system_info_title = RT_NULL;
    g_ui_page_context.system_info_text = RT_NULL;

    return -RT_ENOMEM;
}
/*
 * 简介：创建传感器诊断页面使用的多行文本标签
 * 输入：
 *     parent：标签父对象，必须是诊断页面
 *     initial_text：初始占位文本
 *     y_offset：相对页面顶部的Y方向位置
 *     text_color：标签初始文字颜色
 * 返回值：
 *     成功：新建的LVGL label对象
 *     失败：RT_NULL
 *
 * 线程与所有权：
 * 1. 只能由UI线程调用；
 * 2. 返回对象由诊断页面拥有；
 * 3. 删除诊断页面时，label由LVGL递归删除；
 * 4. 本函数不访问sensor服务、I2C或消息队列。
 */
static lv_obj_t *ui_page_create_sensor_diag_label(lv_obj_t *parent,const char *initial_text,
                                                    lv_coord_t y_offset,lv_color_t text_color)
{
    lv_obj_t *label;

    if ((parent == RT_NULL) ||(initial_text == RT_NULL))
    {
        return RT_NULL;
    }

    label = lv_label_create(parent);
    if (label == RT_NULL)
    {
        return RT_NULL;
    }

    /*
     * 390像素宽屏幕使用92%宽度，左右保留基本边距。
     * 固定高度用于容纳两行诊断文本，避免三个区域互相覆盖。
     */
    lv_obj_set_size(label,LV_PCT(92),58);

    lv_label_set_long_mode(label,LV_LABEL_LONG_WRAP);

    lv_obj_set_style_text_align(label,LV_TEXT_ALIGN_LEFT,LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_text_line_space(label,3,LV_PART_MAIN | LV_STATE_DEFAULT);

    /*
     * 使用FONT_SMALL控制每个器件区域的宽度。
     * 当前屏幕需要同时显示状态、age、错误和物理量，
     * FONT_NORMAL可能导致长数值过早换行。
     */
    lv_ext_set_local_font(label,FONT_SMALL,text_color);

    lv_label_set_text(label,initial_text);

    lv_obj_align(label,LV_ALIGN_TOP_MID,0,y_offset);

    /*
     * 诊断文字本身不产生触摸动作。
     * 所有导航仍由底部按钮产生统一UI语义。
     */
    lv_obj_clear_flag(label,LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    return label;
}
/*
 * 简介：创建最小传感器诊断页面
 * 输入：
 *     parent：页面共同根screen
 * 返回值：
 *     RT_EOK：页面完整创建成功
 *     -RT_EINVAL：父对象为空
 *     -RT_EBUSY：诊断页面或其对象已经存在
 *     -RT_ENOMEM：页面、标签或导航按钮创建失败
 *
 * 线程与所有权：
 * 1. 本函数只能由UI线程调用；
 * 2. 所有LVGL对象都以page为父对象；
 * 3. 全部对象成功后才把指针提交到全局上下文；
 * 4. 创建失败时删除page即可递归清理所有子对象；
 * 5. 本函数创建页面对象和500ms刷新timer，
 *    实际快照读取只在UI线程的刷新函数中执行；
 * 6. sensor线程不得持有这里创建的任何对象指针。
 */
static rt_err_t ui_page_create_sensor_diag(lv_obj_t *parent)
{
    lv_obj_t *page;
    lv_obj_t *title;
    lv_obj_t *imu_text;
    lv_obj_t *mmc_text;
    lv_obj_t *als_text;
    lv_obj_t *footer;
    lv_obj_t *nav_button;
    lv_timer_t *refresh_timer;
    rt_uint32_t device_index;

    if (parent == RT_NULL)
    {
        return -RT_EINVAL;
    }

    /*
     * 页面采用一次创建、隐藏复用策略。
     * 任意一个对象指针已经存在，都说明不能再次创建。
     */
    if ((g_ui_page_context.sensor_diag_page != RT_NULL) ||
        (g_ui_page_context.sensor_diag_title != RT_NULL) ||
        (g_ui_page_context.sensor_diag_imu_text != RT_NULL) ||
        (g_ui_page_context.sensor_diag_mmc_text != RT_NULL) ||
        (g_ui_page_context.sensor_diag_als_text != RT_NULL) ||
        (g_ui_page_context.sensor_diag_footer != RT_NULL)   ||
        (g_ui_page_context.sensor_diag_timer != RT_NULL))
    {
        return -RT_EBUSY;
    }

    page = RT_NULL;
    title = RT_NULL;
    imu_text = RT_NULL;
    mmc_text = RT_NULL;
    als_text = RT_NULL;
    footer = RT_NULL;
    nav_button = RT_NULL;
    refresh_timer = RT_NULL;

    page = lv_obj_create(parent);
    if (page == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    lv_obj_set_size(page,LV_PCT(100),LV_PCT(100));

    lv_obj_center(page);

    lv_obj_clear_flag(
        page,
        LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    lv_obj_set_style_border_width(
        page,
        0,
        LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_radius(
        page,
        0,
        LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_pad_all(
        page,
        0,
        LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(
        page,
        lv_color_hex(0x10161E),
        LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_opa(
        page,
        LV_OPA_COVER,
        LV_PART_MAIN | LV_STATE_DEFAULT);

    title = lv_label_create(page);
    if (title == RT_NULL)
    {
        goto create_failed;
    }

    lv_label_set_text(
        title,
        "SENSOR DIAG");

    lv_ext_set_local_font(
        title,
        FONT_TITLE,
        LV_COLOR_WHITE);

    lv_obj_align(
        title,
        LV_ALIGN_TOP_MID,
        0,
        24);

    /*
    * 页面创建时先显示WAIT占位符。
    * 进入诊断页后的首次刷新会使用一致快照更新这些文本；
    * 若sensor服务尚未运行，则保留占位符并在footer显示错误。
    */
    imu_text = ui_page_create_sensor_diag_label(
        page,
        "IMU WAIT  age=--  err=0\n"
        "A=--,--,-- mg  G=--,--,-- mdps",
        72,
        lv_color_hex(0x8B949E));

    if (imu_text == RT_NULL)
    {
        goto create_failed;
    }

    mmc_text = ui_page_create_sensor_diag_label(
        page,
        "MMC WAIT  age=--  err=0\n"
        "B=--,--,-- uT",
        145,
        lv_color_hex(0x8B949E));

    if (mmc_text == RT_NULL)
    {
        goto create_failed;
    }

    als_text = ui_page_create_sensor_diag_label(
        page,
        "ALS WAIT  age=--  err=0\n"
        "L=-- lux  CH=--/--",
        218,
        lv_color_hex(0x8B949E));

    if (als_text == RT_NULL)
    {
        goto create_failed;
    }

    footer = ui_page_create_sensor_diag_label(
        page,
        "snapshot=--  refresh<=2Hz",
        305,
        lv_color_hex(0x70A5D8));

    if (footer == RT_NULL)
    {
        goto create_failed;
    }

    nav_button = ui_page_create_nav_button(
        page,
        "HOME",
        LV_ALIGN_BOTTOM_LEFT,
        18,
        &g_ui_nav_home);

    if (nav_button == RT_NULL)
    {
        goto create_failed;
    }

    nav_button = ui_page_create_nav_button(
        page,
        "BACK",
        LV_ALIGN_BOTTOM_MID,
        0,
        &g_ui_nav_back);

    if (nav_button == RT_NULL)
    {
        goto create_failed;
    }

    /*
     * 创建500ms诊断刷新timer。
     *
     * lv_timer回调由lv_task_handler()在UI线程执行，所以回调中可以更新LVGL对象。
     *
     * timer只读取sensor服务已经发布的内存快照，
     * 不执行I2C，也不向app_evt或ui_evt投递逐样本事件。
     */
    refresh_timer = lv_timer_create(
        ui_page_sensor_diag_timer_cb,
        UI_SENSOR_DIAG_REFRESH_MS,
        RT_NULL);

    if (refresh_timer == RT_NULL)
    {
        goto create_failed;
    }

    /*
     * 初始化UI侧恢复边沿记录。
     * previous_valid=false表示还没有观察过该器件状态，
     * 第一次看到READY不能误判为“故障恢复”。
     */
    for (device_index = 0U;device_index < (rt_uint32_t)SENSOR_DEVICE_COUNT;device_index++)
    {
        g_ui_page_context.sensor_diag_previous_state[device_index] =SENSOR_DEVICE_STATE_UNINITIALIZED;

        g_ui_page_context.sensor_diag_previous_valid[device_index] =RT_FALSE;

        g_ui_page_context.sensor_diag_recovery_until[device_index] =0U;
    }

    g_ui_page_context.sensor_diag_has_publish_seq = RT_FALSE;
    /*
     * 所有对象都已创建成功，此时才提交全局对象指针。
     * UI以外的线程不得读取或修改这些指针。
     */
    g_ui_page_context.sensor_diag_page = page;
    g_ui_page_context.sensor_diag_title = title;
    g_ui_page_context.sensor_diag_imu_text = imu_text;
    g_ui_page_context.sensor_diag_mmc_text = mmc_text;
    g_ui_page_context.sensor_diag_als_text = als_text;
    g_ui_page_context.sensor_diag_footer = footer;
    g_ui_page_context.sensor_diag_timer = refresh_timer;

    return RT_EOK;

create_failed:

    /*
     * LVGL timer不属于page对象树，必须单独删除。
     * 否则page删除后timer仍可能访问已经释放的label。
     */
    if (refresh_timer != RT_NULL)
    {
        lv_timer_del(refresh_timer);
        refresh_timer = RT_NULL;
    }
    /*
     * page拥有本函数创建的全部子对象。
     * 删除page即可一次性回收标题、标签和导航按钮。
     */
    if (page != RT_NULL)
    {
        lv_obj_del(page);
    }

    g_ui_page_context.sensor_diag_page = RT_NULL;
    g_ui_page_context.sensor_diag_title = RT_NULL;
    g_ui_page_context.sensor_diag_imu_text = RT_NULL;
    g_ui_page_context.sensor_diag_mmc_text = RT_NULL;
    g_ui_page_context.sensor_diag_als_text = RT_NULL;
    g_ui_page_context.sensor_diag_footer = RT_NULL;
    g_ui_page_context.sensor_diag_timer = RT_NULL;

    return -RT_ENOMEM;
}


/*简介：创建最小导航按钮
  输入: parent: 父对象
        text: 按钮文本
        align: 对齐方式
        x_offset: X方向偏移量
        action: 点击动作回调
  返回值：lv_obj_t * 创建的导航按钮对象，若创建失败则返回RT_NULL。
*/
static lv_obj_t *ui_page_create_nav_button(
    lv_obj_t *parent,
    const char *text,
    lv_align_t align,
    lv_coord_t x_offset,
    const ui_action_t *action)
{
    lv_obj_t *button;
    lv_obj_t *label;

    if ((parent == RT_NULL) ||(text == RT_NULL) ||(action == RT_NULL))
    {
        return RT_NULL;
    }

    button = lv_btn_create(parent);
    if (button == RT_NULL)
    {
        return RT_NULL;
    }

    lv_obj_set_size(button, 96, 44);
    lv_obj_align(button, align, x_offset, -12);

    lv_obj_add_event_cb(
        button,
        ui_page_nav_event_cb,
        LV_EVENT_CLICKED,
        (void *)action);

    label = lv_label_create(button);
    if (label == RT_NULL)
    {
        lv_obj_del(button);
        return RT_NULL;
    }

    lv_label_set_text(label, text);

    /*
     * 继续使用已经通过任务7验证的SiFli字体入口，
     * 避免恢复直接LVGL字体导致的对齐问题。
     */
    lv_ext_set_local_font(
        label,
        FONT_NORMAL,
        LV_COLOR_WHITE);

    lv_obj_center(label);

    return button;
}

/*简介：ui界面统一清理函数
  输入: 无
  返回值：无
*/
static void ui_page_cleanup(void)
{
    lv_obj_t *root_screen;
    lv_timer_t *sensor_diag_timer;
    rt_uint32_t device_index;
    /*
     * 先保存根对象。
     * 后续统一清空上下文，避免删除过程中的回调观察到悬空指针。
     */
    root_screen = g_ui_page_context.root_screen;
    sensor_diag_timer =g_ui_page_context.sensor_diag_timer;
    /*
     * 将所有lv_obj_t指针设为RT_NULL
     * created=false
     * current_page=UI_PAGE_COUNT
     */
    g_ui_page_context.root_screen = RT_NULL;

    g_ui_page_context.watchface_page = RT_NULL;
    g_ui_page_context.watchface_title = RT_NULL;
    g_ui_page_context.watchface_time = RT_NULL;
    g_ui_page_context.watchface_status = RT_NULL;

    g_ui_page_context.menu_page = RT_NULL;
    g_ui_page_context.menu_title = RT_NULL;
    g_ui_page_context.menu_items = RT_NULL;

    g_ui_page_context.system_info_page = RT_NULL;
    g_ui_page_context.system_info_title = RT_NULL;
    g_ui_page_context.system_info_text = RT_NULL;

    /*
    * timer不属于root_screen对象树，
    * 已在删除页面对象以前单独停止并清理。
    */
    g_ui_page_context.sensor_diag_page = RT_NULL;
    g_ui_page_context.sensor_diag_title = RT_NULL;
    g_ui_page_context.sensor_diag_imu_text = RT_NULL;
    g_ui_page_context.sensor_diag_mmc_text = RT_NULL;
    g_ui_page_context.sensor_diag_als_text = RT_NULL;
    g_ui_page_context.sensor_diag_footer = RT_NULL;
    g_ui_page_context.sensor_diag_timer = RT_NULL;
    g_ui_page_context.sensor_diag_has_publish_seq = RT_FALSE;

    for (device_index = 0U;device_index < (rt_uint32_t)SENSOR_DEVICE_COUNT;device_index++)
    {
        g_ui_page_context.sensor_diag_previous_state[device_index] =SENSOR_DEVICE_STATE_UNINITIALIZED;

        g_ui_page_context.sensor_diag_previous_valid[device_index] =RT_FALSE;

        g_ui_page_context.sensor_diag_recovery_until[device_index] =0U;
    }

    //必须先停止timer，再删除其可能访问的LVGL对象。
    if (sensor_diag_timer != RT_NULL)
    {
        lv_timer_del(sensor_diag_timer);
    }
    /*
     * root_screen是所有页面的共同父对象。
     * 删除它即可递归释放四个页面及其标签。
     */
    if (root_screen != RT_NULL)
    {
        lv_obj_del(root_screen);
    }

    /*
     * 只复位当前运行状态，保留累计统计，
     * 方便诊断此次创建失败或切换失败。
     */

    g_ui_page_context.stats.created = RT_FALSE;
    g_ui_page_context.stats.current_page = UI_PAGE_COUNT;

}

/*
 * 简介：设置唯一可见页面
 * 输入：
 *     target：目标页面ID
 * 返回值：
 *     RT_EOK：切换成功
 *     -RT_EINVAL：页面ID非法
 *     -RT_ERROR：页面对象尚未完整创建
 *
 * 约束：
 * 1. 只能由UI线程调用；
 * 2. 本函数只修改LVGL对象的可见性；
 * 3. 不负责更新current_page和切换统计；
 * 4. 校验失败时不改变当前页面显示。
 */
static rt_err_t ui_page_apply_visibility(ui_page_id_t target)
{

    lv_obj_t *target_page = RT_NULL;

    if(ui_page_id_is_valid(target)!=RT_TRUE)
    {
        return -RT_EINVAL;
    }

     /*
     * 本函数会同时操作四个页面，因此必须在任何修改前，
     * 确认根screen和四个页面对象都已经完整创建。
     */
    if ((g_ui_page_context.root_screen == RT_NULL) ||
        (g_ui_page_context.watchface_page == RT_NULL) ||
        (g_ui_page_context.menu_page == RT_NULL) ||
        (g_ui_page_context.system_info_page == RT_NULL) ||
        (g_ui_page_context.sensor_diag_page == RT_NULL) ||
        (g_ui_page_context.sensor_diag_timer == RT_NULL))
    {
        return -RT_ERROR;
    }

    /* 先把页面ID转换为具体对象。 */
    switch (target)
    {
    case UI_PAGE_WATCHFACE:
        target_page = g_ui_page_context.watchface_page;
        break;

    case UI_PAGE_MENU:
        target_page = g_ui_page_context.menu_page;
        break;

    case UI_PAGE_SYSTEM_INFO:
        target_page = g_ui_page_context.system_info_page;
        break;
    case UI_PAGE_SENSOR_DIAG:
        target_page = g_ui_page_context.sensor_diag_page;
        break;
    default:
        /* 防御性分支，正常情况下前面的ID检查已经拦截。 */
        return -RT_EINVAL;
    }

    /*
     * 所有可能失败的校验已经完成。
     * 从这里开始不会出现“隐藏一半后返回失败”的状态。
     */
    lv_obj_add_flag(
        g_ui_page_context.watchface_page,
        LV_OBJ_FLAG_HIDDEN);

    lv_obj_add_flag(
        g_ui_page_context.menu_page,
        LV_OBJ_FLAG_HIDDEN);

    lv_obj_add_flag(
        g_ui_page_context.system_info_page,
        LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(
        g_ui_page_context.sensor_diag_page,
        LV_OBJ_FLAG_HIDDEN);
    /* 唯一显示目标页面。 */
    lv_obj_clear_flag(target_page, LV_OBJ_FLAG_HIDDEN);


    return RT_EOK;

}

/*
 * 简介：切换到指定页面
 * 输入：
 *     target：目标页面ID
 * 返回值：
 *     RT_EOK：切换成功或目标就是当前页面
 *     -RT_EBUSY：页面系统尚未创建
 *     -RT_EINVAL：目标或当前页面ID非法
 *     其他负错误码：页面可见性修改失败
 *
 * 统计语义：
 * switch_attempt：每次调用本函数增加一次；
 * switch_success：成功切换和合法noop都增加；
 * switch_fail：本函数返回失败时增加；
 * switch_noop：目标页面等于当前页面时增加。
 */
static rt_err_t ui_page_show(ui_page_id_t target)
{
    rt_base_t level;
    rt_err_t result;
    ui_page_id_t old_page;

    ui_page_stat_increment(&g_ui_page_context.stats.switch_attempt);

    //检查created
    if(g_ui_page_context.stats.created!=RT_TRUE)
    {
        ui_page_stat_increment(&g_ui_page_context.stats.switch_fail);
        return -RT_EBUSY;
    }

    /*
     * 同时验证目标页面和当前状态。
     * current_page异常时不能继续推导NEXT或BACK。
     */
    if(ui_page_id_is_valid(target)!=RT_TRUE||
        ui_page_id_is_valid(g_ui_page_context.stats.current_page)!=RT_TRUE)
    {
        ui_page_stat_increment(&g_ui_page_context.stats.switch_fail);
        return -RT_EINVAL;
    }

    old_page = g_ui_page_context.stats.current_page;
    /*
     * 切换到当前页面是合法的幂等操作。
     * 不操作LVGL对象，但记作成功和noop。
     */
    if(target==g_ui_page_context.stats.current_page)
    {
        level = rt_hw_interrupt_disable();

        g_ui_page_context.stats.switch_success++;
        g_ui_page_context.stats.switch_noop++;

        rt_hw_interrupt_enable(level);
        return RT_EOK;
    }

    /*
     * apply_visibility成功前不能修改current_page。
     * 该函数失败时，原页面仍保持可见。
     */

    result=ui_page_apply_visibility(target);
    if(result!=RT_EOK)
    {
        ui_page_stat_increment(&g_ui_page_context.stats.switch_fail);
        return result;
    }

    /*
     * 视觉切换成功后，再提交权威页面状态和成功计数。
     * 临界区内部直接递增，避免再次嵌套关闭中断。
     */
    level = rt_hw_interrupt_disable();
    g_ui_page_context.stats.current_page=target;
    g_ui_page_context.stats.switch_success++;
    rt_hw_interrupt_enable(level);
    /*
     * 进入诊断页时立即读取一次最新快照，
     * 避免用户最多等待500ms才看到数据。
     *
     * 随后重置timer，保证下一次计划刷新至少在500ms以后，
     * 不会因timer刚好即将到期而短时间连续刷新两次。
     */
    if ((target == UI_PAGE_SENSOR_DIAG) &&(g_ui_page_context.sensor_diag_timer != RT_NULL))
    {
        ui_page_stat_increment(&g_ui_page_context.sensor_diag_stats.visible_refresh_count);

        (void)ui_page_refresh_sensor_diag();

        lv_timer_reset(g_ui_page_context.sensor_diag_timer);
    }
    rt_kprintf("[UI][PAGE] switch %d -> %d\n",
           (int)old_page,
           (int)target);

    return RT_EOK;

}



/*简介：获取下一个页面ID
  输入: current: 当前页面id
  返回值：ui_page_id_t：下一个页面ID
        合法时返回下一个页面ID；
        非法时返回UI_PAGE_COUNT作为无效哨兵值。
*/
static ui_page_id_t ui_page_next_id(ui_page_id_t current)
{
    if(ui_page_id_is_valid(current)!=RT_TRUE)
    {
        return UI_PAGE_COUNT;
    }

    /*
     * 页面循环顺序：
     * WATCHFACE   -> MENU
     * MENU        -> SYSTEM_INFO
     * SYSTEM_INFO -> SENSOR_DIAG
     * SENSOR_DIAG -> WATCHFACE
     */
    switch (current)
    {
    case UI_PAGE_WATCHFACE:
        return UI_PAGE_MENU;

    case UI_PAGE_MENU:
        return UI_PAGE_SYSTEM_INFO;

    case UI_PAGE_SYSTEM_INFO:
        return UI_PAGE_SENSOR_DIAG;

    case UI_PAGE_SENSOR_DIAG:
        return UI_PAGE_WATCHFACE;

    default:
        return UI_PAGE_COUNT;
    }


}

/*简介：获取返回页面ID
  输入: current: 当前页面id
  返回值：ui_page_id_t：上一个页面ID
        合法时返回上一个页面ID；
        非法时返回UI_PAGE_COUNT作为无效哨兵值。

*/
static ui_page_id_t ui_page_back_id(ui_page_id_t current)
{
    if(ui_page_id_is_valid(current)!=RT_TRUE)
    {
        return UI_PAGE_COUNT;
    }

     /*
     * 页面返回层级：
     * WATCHFACE   -> WATCHFACE
     * MENU        -> WATCHFACE
     * SYSTEM_INFO -> MENU
     * SENSOR_DIAG -> SYSTEM_INFO
     */
    switch (current)
    {
    case UI_PAGE_WATCHFACE:
        return UI_PAGE_WATCHFACE;

    case UI_PAGE_MENU:
        return UI_PAGE_WATCHFACE;

    case UI_PAGE_SYSTEM_INFO:
        return UI_PAGE_MENU;

    case UI_PAGE_SENSOR_DIAG:
        return UI_PAGE_SYSTEM_INFO;

    default:
        return UI_PAGE_COUNT;
    }

}

/*简介：LVGL点击回调
  输入: event: LVGL事件对象
  返回值：void: 无

*/
static void ui_page_nav_event_cb(lv_event_t *event)
{
    const ui_action_t *action;
    rt_err_t result;

    if (event == RT_NULL)
    {
        return;
    }

    if (lv_event_get_code(event) != LV_EVENT_CLICKED)
    {
        return;
    }

    action = (const ui_action_t *)lv_event_get_user_data(event);
    if ((action == RT_NULL) ||
        (*action <= UI_ACTION_NONE) ||
        (*action >= UI_ACTION_COUNT))
    {
        ui_page_stat_increment(
            &g_ui_page_context.stats.touch_action_fail);
        return;
    }

    ui_page_stat_increment(
        &g_ui_page_context.stats.touch_action_attempt);

    if (g_ui_page_context.action_handler == RT_NULL)
    {
        ui_page_stat_increment(
            &g_ui_page_context.stats.touch_action_fail);
        return;
    }

    /*
     * 重要：这里不能直接调用ui_page_show()。
     * 触摸和按键必须进入同一个UI事件队列。
     */
    result = g_ui_page_context.action_handler(
        *action,
        rt_tick_get(),
        g_ui_page_context.action_user_data);

    if (result == RT_EOK)
    {
        ui_page_stat_increment(
            &g_ui_page_context.stats.touch_action_success);
    }
    else
    {
        ui_page_stat_increment(
            &g_ui_page_context.stats.touch_action_fail);

        rt_kprintf(
            "[UI][TOUCH_NAV][WARN] action=%d post failed: %d\n",
            (int)*action,
            (int)result);
    }
}

/*简介：检查页面id是否有效
  输入：page：页面id
  返回值：rt_bool_t：页面id是否有效
        RT_TRUE：有效页面
        RT_FALSE：非法页面
*/
static rt_bool_t ui_page_id_is_valid(ui_page_id_t page)
{
    /*
     * 核心：
     * 显式转换为int，避免枚举底层为无符号类型时，
     * “page >= 0”产生恒真比较warning。
     *
     * 当前设计要求有效页面ID连续排列在：
     * UI_PAGE_WATCHFACE ～ UI_PAGE_COUNT - 1。
     */
    int page_value = (int)page;

    if ((page_value < (int)UI_PAGE_WATCHFACE) ||
        (page_value >= (int)UI_PAGE_COUNT))
    {
        return RT_FALSE;
    }

    return RT_TRUE;
}

/* 简介：线程安全地增加一个32位统计计数
 * 输入：counter：需要递增的统计字段
 * 返回值：无
 */
static void ui_page_stat_increment(rt_uint32_t *counter)
{
    rt_base_t level;

    if (counter == RT_NULL)
    {
        return;
    }

    level = rt_hw_interrupt_disable();
    (*counter)++;
    rt_hw_interrupt_enable(level);
}


/*
 * 简介：把毫秒转换为非零系统tick
 * 输入：milliseconds：需要换算的毫秒数
 * 返回值：
 *     0：输入为0 ms
 *     大于0：向上取整后的系统tick数
 *
 * 设计与约束：
 * 1. 使用64位中间值，避免毫秒数与RT_TICK_PER_SECOND相乘时溢出；
 * 2. 使用向上取整，避免非零毫秒被截断成0 tick；
 * 3. 本函数只进行单位换算，不延时、不读取系统时间；
 * 4. RT_TICK_PER_SECOND由当前RT-Thread配置提供，必须大于0。
 */
static rt_tick_t ui_page_ms_to_ticks(rt_uint32_t milliseconds)
{
    rt_uint64_t ticks;

    /*
     * 加999后再除以1000，实现正数毫秒到tick的向上取整。
     *
     * 例如tick频率为100 Hz时：
     * 1 ms会得到1 tick，而不是因整数截断得到0 tick。
     */
    ticks =((rt_uint64_t)milliseconds *(rt_uint64_t)RT_TICK_PER_SECOND +999ULL) /
            1000ULL;

    /*
     * 保留防御性检查：
     * 只要输入不是0 ms，返回值就不能是0 tick。
     */
    if ((milliseconds > 0U) &&(ticks == 0ULL))
    {
        ticks = 1ULL;
    }

    return (rt_tick_t)ticks;
}

/*
 * 简介：把系统tick转换为毫秒
 * 输入：ticks：需要换算的系统tick数
 * 返回值：
 *     0～UINT32_MAX：用于诊断显示的毫秒数；
 *     结果超过32位范围时返回UINT32_MAX。
 *
 * 设计与约束：
 * 1. 使用64位中间值，避免ticks乘以1000时发生32位溢出；
 * 2. 整数除法向下取整，因此结果只用于诊断显示；
 * 3. 本函数不改变sensor快照中原始tick字段；
 * 4. 返回UINT32_MAX表示显示值饱和，不代表真实时间恰好等于该值。
 */
static rt_uint32_t ui_page_ticks_to_ms(rt_tick_t ticks)
{
    rt_uint64_t milliseconds;

    /*
     * 先扩展到64位再执行乘法，
     * 防止较大的age_ticks在乘以1000时回绕。
     */
    milliseconds =((rt_uint64_t)ticks * 1000ULL) /(rt_uint64_t)RT_TICK_PER_SECOND;

    /*
     * UI统计字段只有32位。
     * 超出显示范围时饱和到UINT32_MAX，禁止直接截断低32位。
     */
    if (milliseconds > 0xFFFFFFFFULL)
    {
        return 0xFFFFFFFFU;
    }

    return (rt_uint32_t)milliseconds;
}

/*
 * 简介：判断当前tick是否已经到达指定截止tick
 * 输入：
 *     now_tick：当前系统tick
 *     deadline_tick：需要判断的截止tick
 * 返回值：
 *     RT_TRUE：当前时间已经到达或超过截止时间；
 *     RT_FALSE：当前时间尚未到达截止时间。
 *
 * 设计与约束：
 * 1. 使用有符号差值处理无符号tick自然回绕；
 * 2. now_tick与deadline_tick的实际距离必须小于tick范围的一半；
 * 3. 当前仅用于约1秒的RECOVERED显示窗口，满足半范围约束；
 * 4. 本函数只比较时间，不读取系统tick，也不修改任何状态。
 */
static rt_bool_t ui_page_tick_reached(rt_tick_t now_tick,rt_tick_t deadline_tick)
{
    /*
     * 即使tick发生以下回绕：
     *
     * deadline = 0xFFFFFFF0
     * now      = 0x00000010
     *
     * 无符号减法仍会得到正确的小正差值，
     * 转换为有符号数后可以判断deadline已经到达。
     */
    if ((rt_int32_t)(now_tick - deadline_tick) >= 0)
    {
        return RT_TRUE;
    }

    return RT_FALSE;
}

/*
 * 简介：把sensor服务状态映射为诊断页面最终显示状态
 * 输入：
 *     device：需要处理的器件编号；
 *     status：该器件在当前一致快照中的状态；
 *     now_tick：本次UI取得快照的时间基准。
 * 返回值：
 *     UI_SENSOR_DISPLAY_WAIT：尚未取得有效样本或状态不可用；
 *     UI_SENSOR_DISPLAY_READY：器件正常且数据未过期；
 *     UI_SENSOR_DISPLAY_RECOVERED：刚从失败或离线状态恢复；
 *     UI_SENSOR_DISPLAY_WARN：暂时失败；
 *     UI_SENSOR_DISPLAY_STALE：last-good已经过期；
 *     UI_SENSOR_DISPLAY_OFFLINE：器件已经进入离线状态。
 *
 * 线程与所有权：
 * 1. 本函数只能由UI线程调用；
 * 2. status指向调用者局部sensor_snapshot_t中的状态副本；
 * 3. 本函数不会保存status指针，也不访问sensor内部快照；
 * 4. previous_state和recovery_until只属于UI显示状态；
 * 5. 本函数不执行I2C、LVGL操作、延时、日志或队列投递。
 *
 * 状态优先级：
 * OFFLINE > STALE > TEMP_FAILED > RECOVERED > READY > WAIT。
 */
static ui_sensor_display_state_t ui_page_sensor_display_state(sensor_device_id_t device,
                                                                const sensor_device_status_t *status,
                                                                rt_tick_t now_tick)
{
    sensor_device_state_t previous_state;
    rt_bool_t previous_valid;
    rt_tick_t recovery_ticks;

    /*
     * 在使用device作为数组下标以前，必须同时检查status和范围。
     * 非法输入统一映射为WAIT，禁止访问数组边界之外的状态。
     */
    if ((status == RT_NULL) ||
        ((int)device < (int)SENSOR_DEVICE_IMU) ||
        ((int)device >= (int)SENSOR_DEVICE_COUNT))
    {
        return UI_SENSOR_DISPLAY_WAIT;
    }

    /*
     * previous_valid用于区分：
     * 1. UI确实观察过UNINITIALIZED；
     * 2. 数组只是静态初始化后的默认零值。
     */
    previous_state =g_ui_page_context.sensor_diag_previous_state[device];

    previous_valid =g_ui_page_context.sensor_diag_previous_valid[device];

    /*
     * 只有UI已经观察过失败状态，当前又进入READY并带有完整样本，
     * 才能认定发生了一次可显示的恢复。
     *
     * 第一次打开页面就看到READY不属于恢复。
     */
    if ((previous_valid == RT_TRUE) &&
        ((previous_state == SENSOR_DEVICE_STATE_TEMP_FAILED) ||(previous_state ==SENSOR_DEVICE_STATE_OFFLINE)) &&
        (status->state ==SENSOR_DEVICE_STATE_READY) &&(status->valid == RT_TRUE))
    {
        recovery_ticks =ui_page_ms_to_ticks(UI_SENSOR_DIAG_RECOVERY_MS);

        /*
         * tick允许自然回绕。
         * 后续通过ui_page_tick_reached()比较截止时间。
         */
        g_ui_page_context.sensor_diag_recovery_until[device] = now_tick + recovery_ticks;
    }

    /*
     * 保存UI本次观察到的服务状态，为下一次刷新判断状态边沿。
     *
     * 这里只保存枚举和“已经观察过”标志，
     * 不复制传感器样本，也不构造第二套sensor状态机。
     */
    g_ui_page_context.sensor_diag_previous_state[device] = status->state;

    g_ui_page_context.sensor_diag_previous_valid[device] = RT_TRUE;

    /*
     * OFFLINE优先级最高。
     *
     * 即使仍保留last-good，也必须显示OFFLINE；
     * 数值是否追加OLD由后续文字格式化决定。
     */
    if (status->state ==SENSOR_DEVICE_STATE_OFFLINE)
    {
        g_ui_page_context.sensor_diag_recovery_until[device] = 0U;

        return UI_SENSOR_DISPLAY_OFFLINE;
    }

    /*
     * TEMP_FAILED且没有last-good时仍显示WARN。
     *
     * 这样初始化或首次采样失败不会被普通WAIT掩盖，
     * UI仍能显示last_error用于诊断。
     */
    if ((status->state ==SENSOR_DEVICE_STATE_TEMP_FAILED) &&
        (status->valid != RT_TRUE))
    {
        g_ui_page_context.sensor_diag_recovery_until[device] = 0U;

        return UI_SENSOR_DISPLAY_WARN;
    }

    /*
     * 除上述暂时失败情况外，没有有效last-good时不能显示数值。
     */
    if (status->valid != RT_TRUE)
    {
        g_ui_page_context.sensor_diag_recovery_until[device] = 0U;

        return UI_SENSOR_DISPLAY_WAIT;
    }

    /*
     * 数据已经过期时优先显示STALE。
     *
     * 即使服务状态同时是TEMP_FAILED，
     * 用户更需要知道当前展示的是过期last-good。
     */
    if (status->stale == RT_TRUE)
    {
        g_ui_page_context.sensor_diag_recovery_until[device] = 0U;

        return UI_SENSOR_DISPLAY_STALE;
    }

    /*
     * 数据尚未过期，但当前采样已经暂时失败：
     * 保留last-good，并显示WARN OLD。
     */
    if (status->state ==SENSOR_DEVICE_STATE_TEMP_FAILED)
    {
        g_ui_page_context.sensor_diag_recovery_until[device] = 0U;

        return UI_SENSOR_DISPLAY_WARN;
    }

    /*
     * 处理未来新增状态或异常状态值。
     * 只有明确的READY才能进入正常或恢复显示。
     */
    if (status->state !=SENSOR_DEVICE_STATE_READY)
    {
        g_ui_page_context.sensor_diag_recovery_until[device] = 0U;

        return UI_SENSOR_DISPLAY_WAIT;
    }

    /*
     * READY状态下检查RECOVERED提示窗口。
     *
     * 截止时间未到：继续显示RECOVERED；
     * 截止时间已到：清除窗口并恢复显示READY。
     */
    if (g_ui_page_context.sensor_diag_recovery_until[device] != 0U)
    {
        if (ui_page_tick_reached(now_tick,g_ui_page_context.sensor_diag_recovery_until[device])!= RT_TRUE)
        {
            return UI_SENSOR_DISPLAY_RECOVERED;
        }

        g_ui_page_context.sensor_diag_recovery_until[device] = 0U;
    }

    return UI_SENSOR_DISPLAY_READY;
}

/*
 * 简介：取得诊断状态对应的显示文字
 * 输入：
 *     state：已经完成优先级判断的UI显示状态；
 *     has_last_good：sensor服务是否保留完整有效的last-good。
 * 返回值：
 *     指向只读静态字符串的指针；
 *     调用者不得修改或释放返回字符串。
 *
 * 设计与约束：
 * 1. WARN或OFFLINE存在last-good时追加OLD；
 * 2. STALE一定表示正在展示过期last-good，因此固定追加OLD；
 * 3. 未识别状态统一返回WAIT；
 * 4. 本函数不读取全局状态，也不修改LVGL对象。
 */
static const char *ui_page_sensor_state_text(ui_sensor_display_state_t state,
    rt_bool_t has_last_good)
{
    switch (state)
    {
    case UI_SENSOR_DISPLAY_READY:return "READY";

    case UI_SENSOR_DISPLAY_RECOVERED:return "RECOVERED";

    case UI_SENSOR_DISPLAY_WARN:
        /*
         * 有last-good时继续显示旧值，但必须明确标记OLD；
         * 没有last-good时只显示WARN和错误码。
         */
        return (has_last_good == RT_TRUE)? "WARN OLD" : "WARN";

    case UI_SENSOR_DISPLAY_STALE:
        /*
         * STALE成立的前提是存在已经过期的last-good。
         */
        return "STALE OLD";

    case UI_SENSOR_DISPLAY_OFFLINE:
        /*
         * 离线后可能仍保留最后一次成功数据，
         * 是否追加OLD由has_last_good决定。
         */
        return (has_last_good == RT_TRUE)? "OFFLINE OLD": "OFFLINE";

    case UI_SENSOR_DISPLAY_WAIT:
    default:
        /*
         * default作为未来枚举扩展或状态损坏的防御性兜底。
         */
        return "WAIT";
    }
}

/*
 * 简介：取得诊断状态对应的文字颜色
 * 输入：state：已经完成优先级判断的UI显示状态
 * 返回值：
 *     对应状态的LVGL颜色值。
 *
 * 颜色约定：
 *     READY：绿色；
 *     RECOVERED：蓝色；
 *     WARN：黄色；
 *     STALE：橙色；
 *     OFFLINE：红色；
 *     WAIT或未知状态：灰色。
 *
 * 线程与所有权：
 * 1. 本函数不访问或修改LVGL对象；
 * 2. 返回值由调用者用于UI线程中的样式更新；
 * 3. 未识别状态统一使用灰色，避免误显示为正常绿色。
 */
static lv_color_t ui_page_sensor_state_color(
    ui_sensor_display_state_t state)
{
    switch (state)
    {
    case UI_SENSOR_DISPLAY_READY:
        return lv_color_hex(0x4CAF50);

    case UI_SENSOR_DISPLAY_RECOVERED:
        return lv_color_hex(0x29B6F6);

    case UI_SENSOR_DISPLAY_WARN:
        return lv_color_hex(0xFFC107);

    case UI_SENSOR_DISPLAY_STALE:
        return lv_color_hex(0xFF9800);

    case UI_SENSOR_DISPLAY_OFFLINE:
        return lv_color_hex(0xF44336);

    case UI_SENSOR_DISPLAY_WAIT:
    default:
        /*
         * 未初始化和未知状态不得使用正常状态颜色。
         */
        return lv_color_hex(0x8B949E);
    }
}

/*
 * 简介：传感器诊断页面500ms定时刷新入口
 * 输入：
 *     timer：触发本次回调的LVGL timer对象。
 * 返回值：
 *     无。
 *
 * 调用流程：
 * 1. 记录timer触发次数；
 * 2. 检查页面系统是否创建完成；
 * 3. 检查当前可见页面是否为传感器诊断页；
 * 4. 页面隐藏时只记录跳过次数；
 * 5. 页面可见时记录刷新尝试并复制、渲染一次最新快照。
 *
 * 线程与所有权：
 * 1. 本回调由lv_task_handler()在UI线程中执行；
 * 2. timer由ui_page_context创建、持有和删除；
 * 3. 本函数不保存timer指针，也不修改timer的生命周期；
 * 4. 页面隐藏时不复制快照、不更新任何label；
 * 5. 页面可见时只读取sensor服务已经发布的内存快照；
 * 6. 禁止在本函数中访问I2C、延时或发布逐样本事件；
 * 7. 所有LVGL对象更新由ui_page_refresh_sensor_diag()在UI线程完成。
 *
 * 错误处理：
 * ui_page_refresh_sensor_diag()会记录快照失败并更新footer；
 * 本回调不删除timer，也不停止后续周期，下一次触发时自动重试。
 */
static void ui_page_sensor_diag_timer_cb(lv_timer_t *timer)
{
    /*
     * 当前timer不使用user_data，也不需要在回调内修改周期。
     * 显式转换为void，避免未使用参数编译警告。
     */
    (void)timer;

    /*
     * timer_fire_count记录LVGL实际执行该回调的次数。
     *
     * 它不等于成功刷新次数，因为页面隐藏、页面尚未创建
     * 或快照读取失败时也会进入本回调。
     */
    ui_page_stat_increment(&g_ui_page_context.sensor_diag_stats.timer_fire_count);

    /*
     * 只有同时满足以下条件才允许刷新：
     *
     * 1. 页面对象树已经完整创建；
     * 2. 权威current_page为SENSOR_DIAG。
     *
     * 这里使用唯一的current_page判断可见性，
     * 不另外维护第二份“诊断页是否显示”状态。
     */
    if ((g_ui_page_context.stats.created != RT_TRUE) ||
        (g_ui_page_context.stats.current_page !=
            UI_PAGE_SENSOR_DIAG))
    {
        /*
         * 页面隐藏时保持timer运行，只记录本次跳过。
         *
         * 不调用get_snapshot，不更新label，
         * 也不会影响sensor线程自身的采样周期。
         */
        ui_page_stat_increment(&g_ui_page_context.sensor_diag_stats.hidden_skip_count);

        return;
    }

    /*
     * 页面可见且即将发起一次快照刷新。
     *
     * visible_refresh_count表示刷新尝试次数；
     * 是否成功由snapshot_success_count和
     * snapshot_failure_count进一步区分。
     */
    ui_page_stat_increment( &g_ui_page_context.sensor_diag_stats.visible_refresh_count);

    /*
     * refresh函数完成以下工作：
     *
     * 1. 把sensor一致快照复制到局部变量；
     * 2. 统计publish_seq合并情况；
     * 3. 在UI线程更新三个器件label和footer；
     * 4. 记录复制、渲染耗时及失败原因。
     *
     * 返回值不在本回调中继续传播：
     * 错误已经由refresh函数记录并显示在footer，
     * timer继续存活，500ms后的下一周期自动重试。
     */
    (void)ui_page_refresh_sensor_diag();
}

/*
 * 简介：把IMU状态和最近一份完整样本渲染到诊断页面
 * 输入：
 *     snapshot：UI线程持有的一致sensor快照副本；
 *     now_tick：本次快照读取时使用的时间基准。
 * 返回值：
 *     无。
 *
 * 显示内容：
 * 1. 服务状态、数据年龄和最近错误；
 * 2. 三轴加速度，显示单位为mg；
 * 3. 三轴角速度，显示单位为mdps；
 * 4. 无有效样本时使用“--”占位，不读取样本字段。
 *
 * 单位缩放：
 *     accel_ug / 1000  -> mg；
 *     gyro_udps / 1000 -> mdps。
 *
 * 线程与所有权：
 * 1. 本函数只能由UI线程调用；
 * 2. snapshot是调用者持有的局部快照副本；
 * 3. 本函数不保存snapshot或status指针；
 * 4. sensor_diag_imu_text由UI页面上下文拥有；
 * 5. 本函数不执行I2C、延时、日志或消息投递；
 * 6. 单位缩放只影响显示，不修改快照中的原始物理量。
 */
static void ui_page_render_imu(const sensor_snapshot_t *snapshot,rt_tick_t now_tick)
{
    const sensor_device_status_t *status;
    ui_sensor_display_state_t display_state;
    const char *state_text;
    lv_color_t color;
    rt_uint32_t age_ms;

    /*
     * 快照或IMU标签不存在时不能继续渲染。
     * 这里直接返回，不访问空指针，也不修改其他器件区域。
     */
    if ((snapshot == RT_NULL) ||(g_ui_page_context.sensor_diag_imu_text == RT_NULL))
    {
        return;
    }

    /*
     * status指向本函数收到的局部快照成员。
     * 它只在本次调用期间使用，不保存到全局上下文。
     */
    status = &snapshot->imu_status;

    /*
     * 把服务状态、valid、stale和恢复边沿统一映射为UI状态。
     * now_tick只用于判断RECOVERED提示是否到期。
     */
    display_state =ui_page_sensor_display_state(SENSOR_DEVICE_IMU,status,now_tick);

    /*
     * 根据是否存在last-good决定失败状态是否追加OLD。
     */
    state_text =ui_page_sensor_state_text(display_state,status->valid);

    color =ui_page_sensor_state_color(display_state);

    /*
     * 状态颜色和文本必须在UI线程中更新。
     * sensor服务不接触该label对象。
     */
    lv_obj_set_style_text_color(g_ui_page_context.sensor_diag_imu_text,
                                color,
                                LV_PART_MAIN | LV_STATE_DEFAULT);

    /*
     * 同时检查服务状态valid和IMU样本自身valid。
     *
     * 两者都有效时才允许访问和显示last-good数值，
     * 防止内部状态不一致时把未完成样本显示为真实数据。
     */
    if ((status->valid == RT_TRUE) &&(snapshot->imu.valid == RT_TRUE))
    {
        age_ms =ui_page_ticks_to_ms(status->age_ticks);

        /*
         * C语言有符号整数除法向0截断。
         * 这里的整数mg和mdps只用于紧凑诊断显示，
         * 快照中的ug和udps原始精度保持不变。
         */
        lv_label_set_text_fmt(
            g_ui_page_context.sensor_diag_imu_text,
            "IMU %s age=%ums err=%d\n"
            "A=%ld,%ld,%ldmg G=%ld,%ld,%ldmdps",
            state_text,
            (unsigned int)age_ms,
            (int)status->last_error,
            (long)(snapshot->imu.accel_ug.x / 1000),
            (long)(snapshot->imu.accel_ug.y / 1000),
            (long)(snapshot->imu.accel_ug.z / 1000),
            (long)(snapshot->imu.gyro_udps.x / 1000),
            (long)(snapshot->imu.gyro_udps.y / 1000),
            (long)(snapshot->imu.gyro_udps.z / 1000));
    }
    else
    {
        /*
         * 没有完整last-good时禁止显示数值和0 ms年龄。
         *
         * “--”表示尚无可用样本；
         * last_error仍然保留，方便区分等待和真实读取失败。
         */
        lv_label_set_text_fmt(
            g_ui_page_context.sensor_diag_imu_text,
            "IMU %s age=-- err=%d\n"
            "A=--,--,--mg G=--,--,--mdps",
            state_text,
            (int)status->last_error);
    }
}

/*
 * 简介：把MMC5603NJ状态和最近一份完整磁场样本渲染到诊断页面
 * 输入：
 *     snapshot：UI线程持有的一致sensor快照副本；
 *     now_tick：本次快照读取时使用的时间基准。
 * 返回值：
 *     无。
 *
 * 显示内容：
 * 1. 服务状态、数据年龄和最近错误；
 * 2. 三轴磁场值；
 * 3. 无有效样本时使用“--”占位。
 *
 * 单位缩放：
 *     field_nt / 1000 -> 整数uT。
 *
 * 线程与所有权：
 * 1. 本函数只能由UI线程调用；
 * 2. snapshot和status指针都不跨出本函数生命周期；
 * 3. sensor_diag_mmc_text由UI页面上下文拥有；
 * 4. 本函数不触发新测量，也不访问I2C；
 * 5. 显示为整数uT只为节省页面空间，快照仍保留nT精度。
 */
static void ui_page_render_mmc(const sensor_snapshot_t *snapshot,rt_tick_t now_tick)
{
    const sensor_device_status_t *status;
    ui_sensor_display_state_t display_state;
    const char *state_text;
    lv_color_t color;
    rt_uint32_t age_ms;

    /*
     * 只有快照和MMC标签都存在时才能继续渲染。
     */
    if ((snapshot == RT_NULL) || (g_ui_page_context.sensor_diag_mmc_text == RT_NULL))
    {
        return;
    }

    /*
     * 引用当前一致快照中的MMC状态。
     * 不读取sensor服务内部对象。
     */
    status = &snapshot->mmc_status;

    display_state =ui_page_sensor_display_state(SENSOR_DEVICE_MMC,
                                                status,
                                                now_tick);

    state_text =ui_page_sensor_state_text(display_state,status->valid);

    color =ui_page_sensor_state_color(display_state);

    lv_obj_set_style_text_color(g_ui_page_context.sensor_diag_mmc_text,
                                color,
                                LV_PART_MAIN | LV_STATE_DEFAULT);

    /*
     * 服务状态和MMC样本都有效时，才能展示last-good磁场值。
     *
     * WARN、STALE或OFFLINE状态仍可能满足该条件，
     * 此时state_text会带OLD，明确说明数值不是新样本。
     */
    if ((status->valid == RT_TRUE) && (snapshot->mmc.valid == RT_TRUE))
    {
        age_ms =ui_page_ticks_to_ms(status->age_ticks);

        /*
         * nT除以1000得到整数uT。
         * 有符号数除法向0截断，仅影响UI显示精度。
         */
        lv_label_set_text_fmt(
            g_ui_page_context.sensor_diag_mmc_text,
            "MMC %s age=%ums err=%d\n"
            "B=%ld,%ld,%lduT",
            state_text,
            (unsigned int)age_ms,
            (int)status->last_error,
            (long)(snapshot->mmc.field_nt.x / 1000),
            (long)(snapshot->mmc.field_nt.y / 1000),
            (long)(snapshot->mmc.field_nt.z / 1000));
    }
    else
    {
        /*
         * 没有完整last-good时不显示中心偏置值或默认0，
         * 避免用户把占位数据误认为真实磁场。
         */
        lv_label_set_text_fmt(
            g_ui_page_context.sensor_diag_mmc_text,
            "MMC %s age=-- err=%d\n"
            "B=--,--,--uT",
            state_text,
            (int)status->last_error);
    }
}

/*
 * 简介：把LTR303ALS01状态、双通道数据和照度估算渲染到诊断页面
 * 输入：
 *     snapshot：UI线程持有的一致sensor快照副本；
 *     now_tick：本次快照读取时使用的时间基准。
 * 返回值：
 *     无。
 *
 * 显示内容：
 * 1. 服务状态、数据年龄和最近错误；
 * 2. CH0和CH1同周期原始值；
 * 3. 有效时显示未标定照度估算；
 * 4. 估算无效时显示INVALID和具体reason；
 * 5. 无完整样本时使用“--”占位。
 *
 * 照度显示：
 *     lux_mlux / 1000得到整数lux；
 *     lux_mlux余数的百位作为一位小数；
 *     “L~=”表示结果未经参考照度计标定。
 *
 * 线程与所有权：
 * 1. 本函数只能由UI线程调用；
 * 2. snapshot和status指针只在本次调用内使用；
 * 3. sensor_diag_als_text由UI页面上下文拥有；
 * 4. 本函数不访问I2C、不重新计算sensor服务状态；
 * 5. 服务READY与照度估算有效是两个不同层次；
 * 6. 本函数不会把无效公式结果静默显示为0 lux。
 */
static void ui_page_render_als(const sensor_snapshot_t *snapshot,rt_tick_t now_tick)
{
    const sensor_device_status_t *status;
    ui_sensor_display_state_t display_state;
    const char *state_text;
    lv_color_t color;
    rt_uint32_t age_ms;
    rt_uint32_t lux_whole;
    rt_uint32_t lux_tenth;

    /*
     * 快照或ALS标签不存在时直接返回。
     */
    if ((snapshot == RT_NULL) ||(g_ui_page_context.sensor_diag_als_text == RT_NULL))
    {
        return;
    }

    /*
     * 使用一致快照中的ALS服务状态。
     * 双通道值和estimate也来自同一份快照副本。
     */
    status = &snapshot->als_status;

    display_state = ui_page_sensor_display_state(SENSOR_DEVICE_ALS,status,now_tick);

    state_text = ui_page_sensor_state_text(display_state,status->valid);

    color = ui_page_sensor_state_color(display_state);

    /*
     * 服务READY只表示I2C读取、状态检查和样本发布成功。
     *
     * 照度估算还可能因为饱和、红外占优、增益不匹配
     * 或数学异常而无效。此时保留服务状态文字，
     * 但使用黄色提醒用户不要采用当前照度值。
     */
    if (((display_state ==UI_SENSOR_DISPLAY_READY) ||(display_state ==UI_SENSOR_DISPLAY_RECOVERED)) &&
        (snapshot->als.valid == RT_TRUE) &&
        (snapshot->als.estimate.lux_valid != RT_TRUE))
    {
        color = lv_color_hex(0xFFC107);
    }

    lv_obj_set_style_text_color(g_ui_page_context.sensor_diag_als_text,
                                color,
                                LV_PART_MAIN | LV_STATE_DEFAULT);

    /*
     * 服务状态和ALS原始样本都有效时，
     * 才允许显示CH0、CH1及对应的照度解释。
     */
    if ((status->valid == RT_TRUE) &&(snapshot->als.valid == RT_TRUE))
    {
        age_ms = ui_page_ticks_to_ms(status->age_ticks);

        if (snapshot->als.estimate.lux_valid == RT_TRUE)
        {
            /*
             * 把mlux拆成整数部分和一位小数。
             *
             * 例如186450 mlux显示为186.4 lux；
             * 这里只改变显示精度，不改写estimate.lux_mlux。
             */
            lux_whole =snapshot->als.estimate.lux_mlux /1000U;

            lux_tenth =(snapshot->als.estimate.lux_mlux %1000U) /100U;

            /*
             * L~=明确表示这是未经参考照度计标定的估算结果。
             * CH0/CH1同时保留，便于诊断公式分段和异常光源。
             */
            lv_label_set_text_fmt(
                g_ui_page_context.sensor_diag_als_text,
                "ALS %s age=%ums err=%d\n"
                "L~=%u.%ulux CH=%u/%u",
                state_text,
                (unsigned int)age_ms,
                (int)status->last_error,
                (unsigned int)lux_whole,
                (unsigned int)lux_tenth,
                (unsigned int)snapshot->als.ch0_raw,
                (unsigned int)snapshot->als.ch1_raw);
        }
        else
        {
            /*
             * 原始双通道样本完整，但照度公式结果不可采用。
             *
             * 保留reason和CH0/CH1，禁止把饱和、红外占优
             * 或数学异常统一显示为0 lux。
             */
            lv_label_set_text_fmt(
                g_ui_page_context.sensor_diag_als_text,
                "ALS %s age=%ums err=%d\n"
                "L=INVALID reason=%d CH=%u/%u",
                state_text,
                (unsigned int)age_ms,
                (int)status->last_error,
                (int)snapshot->als.estimate.reason,
                (unsigned int)snapshot->als.ch0_raw,
                (unsigned int)snapshot->als.ch1_raw);
        }
    }
    else
    {
        /*
         * 没有完整ALS last-good时不显示照度或双通道默认值。
         * last_error仍可帮助判断尚未初始化或读取失败。
         */
        lv_label_set_text_fmt(
            g_ui_page_context.sensor_diag_als_text,
            "ALS %s age=-- err=%d\n"
            "L=--lux CH=--/--",
            state_text,
            (int)status->last_error);
    }
}

/*
 * 简介：复制并渲染一次sensor服务一致快照
 * 输入：无。函数从sensor统一服务复制当前最新快照。
 * 返回值：
 *     RT_EOK：快照复制和页面更新成功；
 *     -RT_EBUSY：诊断页面或必要标签尚未完整创建；
 *     -RT_ENOSYS：sensor服务尚未运行或快照锁尚未建立；
 *     其他负错误码：sensor_service_get_snapshot()返回的同步错误。
 *
 * 完成的工作：
 * 1. 检查诊断页及其四个必要标签；
 * 2. 复制一份调用者持有的一致sensor快照；
 * 3. 记录快照复制成功、失败及耗时；
 * 4. 根据publish_seq统计重复刷新和合并发布；
 * 5. 分别渲染IMU、MMC和ALS；
 * 6. 更新footer以及本次渲染耗时。
 *
 * 线程与所有权：
 * 1. 本函数只能由UI线程调用；
 * 2. snapshot是栈上的局部副本，函数返回后不再使用；
 * 3. sensor_service_get_snapshot()在内部锁内只复制快照；
 * 4. sensor快照锁释放后才执行LVGL操作；
 * 5. 本函数不访问I2C、不延时、不投递逐样本事件；
 * 6. 本函数不返回或保存sensor服务内部对象指针；
 * 7. 快照失败时保留已有器件文本，只更新footer错误。
 *
 * 时间语义：
 * copy_ticks表示复制一致快照的调用耗时；
 * render_ticks表示更新三个器件标签和footer的代码耗时；
 * 两者都不表示AMOLED物理出屏延迟。
 */
static rt_err_t ui_page_refresh_sensor_diag(void)
{
    sensor_snapshot_t snapshot;
    rt_tick_t copy_start_tick;
    rt_tick_t copy_end_tick;
    rt_tick_t render_start_tick;
    rt_tick_t render_end_tick;
    rt_tick_t copy_ticks;
    rt_tick_t render_ticks;
    rt_uint32_t publish_delta;
    rt_base_t level;
    rt_err_t result;

    /*
     * 诊断页面及四个需要更新的标签必须全部存在。
     *
     * timer不是本函数直接操作的对象，因此不在这里检查；
     * timer完整性由页面创建和可见性切换路径保证。
     */
    if ((g_ui_page_context.sensor_diag_page == RT_NULL) ||
        (g_ui_page_context.sensor_diag_imu_text == RT_NULL) ||
        (g_ui_page_context.sensor_diag_mmc_text == RT_NULL) ||
        (g_ui_page_context.sensor_diag_als_text == RT_NULL) ||
        (g_ui_page_context.sensor_diag_footer == RT_NULL))
    {
        return -RT_EBUSY;
    }

    /*
     * 在调用前后分别取得tick，用于测量快照复制耗时。
     *
     * 该耗时包括：
     * 1. get_snapshot()的参数和服务状态检查；
     * 2. 最多约5ms的快照mutex等待；
     * 3. 固定长度结构体复制；
     * 4. 锁外age和stale计算。
     *
     * 它不包含后续LVGL渲染时间。
     */
    copy_start_tick = rt_tick_get();

    result =sensor_service_get_snapshot(&snapshot);

    copy_end_tick = rt_tick_get();

    /*
     * 无符号tick减法允许系统tick在调用期间自然回绕。
     */
    copy_ticks =(rt_tick_t)(copy_end_tick - copy_start_tick);

    /*
     * 统计可能由FinSH线程同时读取，因此在短临界区中
     * 一次更新本次耗时、最大耗时和快照结果。
     *
     * 临界区内禁止执行日志、LVGL操作和sensor接口调用。
     */
    level = rt_hw_interrupt_disable();

    g_ui_page_context.sensor_diag_stats.last_snapshot_copy_ticks = copy_ticks;

    if (copy_ticks > g_ui_page_context.sensor_diag_stats.max_snapshot_copy_ticks)
    {
        g_ui_page_context.sensor_diag_stats.max_snapshot_copy_ticks = copy_ticks;
    }

    if (result != RT_EOK)
    {
        g_ui_page_context.sensor_diag_stats.snapshot_failure_count++;

        g_ui_page_context.sensor_diag_stats .last_snapshot_error = result;
    }
    else
    {
        g_ui_page_context.sensor_diag_stats .snapshot_success_count++;

        g_ui_page_context.sensor_diag_stats.last_snapshot_error = RT_EOK;
    }

    rt_hw_interrupt_enable(level);

    /*
     * get_snapshot()保证失败时不修改调用者输出缓冲区。
     *
     * 因此失败后不能读取局部snapshot中的任何字段，
     * 只能保留之前成功显示的器件数据并更新footer。
     */
    if (result != RT_EOK)
    {
        /*
         * 保留三个器件标签的上一次成功显示。
         *
         * footer使用红色显示快照接口错误，
         * 下一次500ms timer触发时会自动重新尝试。
         */
        lv_label_set_text_fmt(
            g_ui_page_context.sensor_diag_footer,
            "snapshot err=%d UI<=2Hz ALS=uncal",
            (int)result);

        lv_obj_set_style_text_color(
            g_ui_page_context.sensor_diag_footer,
            lv_color_hex(0xF44336),
            LV_PART_MAIN | LV_STATE_DEFAULT);

        return result;
    }

    /*
     * 第一次成功取得快照以前，没有可比较的上一发布序号。
     *
     * 不能使用last_publish_seq == 0判断第一次读取，
     * 因为publish_seq的合法值和自然回绕结果都可能为0。
     */
    if (g_ui_page_context.sensor_diag_has_publish_seq == RT_TRUE)
    {
        /*
         * publish_seq使用无符号减法。
         * 即使序号从UINT32_MAX自然回绕到0，也能得到正确差值。
         */
        publish_delta =snapshot.publish_seq - g_ui_page_context.sensor_diag_stats.last_publish_seq;

        if (publish_delta == 0U)
        {
            /*
             * publish_seq没有变化，说明服务没有发布新内容。
             *
             * 仍然执行本次渲染，因为age会随时间增加，
             * RECOVERED显示窗口也可能在本次刷新中到期。
             *
             * unchanged_skip_count表示跳过重复发布处理，
             * 并不表示完全跳过LVGL渲染。
             */
            ui_page_stat_increment(
                &g_ui_page_context.sensor_diag_stats
                    .unchanged_skip_count);
        }
        else if (publish_delta > 1U)
        {
            /*
             * 两次UI刷新之间产生了多次sensor发布。
             *
             * UI只显示最新一致快照，不追赶中间历史样本，
             * 避免高频IMU数据占满8槽UI队列。
             *
             * 当前快照本身需要显示，因此合并数量为delta-1。
             */
            level = rt_hw_interrupt_disable();

            g_ui_page_context.sensor_diag_stats
                .coalesced_publish_count +=
                    publish_delta - 1U;

            rt_hw_interrupt_enable(level);
        }
    }
    else
    {
        /*
         * 第一次成功取得快照只建立比较基线，
         * 不把从服务启动到当前的序号计为UI合并数量。
         */
        g_ui_page_context.sensor_diag_has_publish_seq =RT_TRUE;
    }

    /*
     * 从这里开始，sensor快照mutex已经释放。
     *
     * 三个渲染函数只读取局部snapshot，
     * 并在当前UI线程中更新各自的LVGL标签。
     */
    render_start_tick = rt_tick_get();

    /*
     * 三颗器件使用同一个snapshot_tick作为状态显示时间基准，
     * 避免一次UI刷新中分别读取系统tick造成细小时间偏差。
     */
    ui_page_render_imu(
        &snapshot,
        snapshot.snapshot_tick);

    ui_page_render_mmc(
        &snapshot,
        snapshot.snapshot_tick);

    ui_page_render_als(
        &snapshot,
        snapshot.snapshot_tick);

    /*
     * footer显示本次实际渲染的服务发布序号。
     *
     * UI<=2Hz说明当前页面刷新上限；
     * ALS=uncal说明照度只是未经参考照度计标定的估算值。
     */
    lv_label_set_text_fmt(
        g_ui_page_context.sensor_diag_footer,
        "pub=%u UI<=2Hz ALS=uncal",
        (unsigned int)snapshot.publish_seq);

    lv_obj_set_style_text_color(
        g_ui_page_context.sensor_diag_footer,
        lv_color_hex(0x70A5D8),
        LV_PART_MAIN | LV_STATE_DEFAULT);

    render_end_tick = rt_tick_get();

    /*
     * 渲染耗时包含：
     * 1. 三颗器件状态映射；
     * 2. 数值缩放和字符串格式化；
     * 3. 三个器件标签及footer的LVGL API调用。
     *
     * 它不包含后续LCD传输和AMOLED物理扫描时间。
     */
    render_ticks =(rt_tick_t)(render_end_tick - render_start_tick);

    /*
     * 完整渲染成功后才提交render_count和last_publish_seq。
     *
     * 如果未来渲染函数改为可能失败的接口，
     * 应先汇总返回值，再决定是否提交这些统计。
     */
    level = rt_hw_interrupt_disable();

    g_ui_page_context.sensor_diag_stats
        .render_count++;

    g_ui_page_context.sensor_diag_stats
        .last_publish_seq =
            snapshot.publish_seq;

    g_ui_page_context.sensor_diag_stats
        .last_render_ticks =
            render_ticks;

    if (render_ticks >
        g_ui_page_context.sensor_diag_stats
            .max_render_ticks)
    {
        g_ui_page_context.sensor_diag_stats
            .max_render_ticks =
                render_ticks;
    }

    rt_hw_interrupt_enable(level);

    return RT_EOK;
}
