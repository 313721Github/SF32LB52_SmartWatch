#include "ui_page.h"
#include "rthw.h"
#include "lvgl.h"
#include "lvsf_font.h"
#include "system_diag.h"

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

//公开函数声明
rt_err_t ui_page_create(void);                              //创建ui界面
rt_err_t ui_page_handle_event(const ui_event_t *event);     //ui事件处理函数
rt_err_t ui_page_get_stats(ui_page_stats_t *stats);         //获取ui页面统计信息

//私有函数声明

static rt_err_t ui_page_create_watchface(lv_obj_t *parent);         //创建表盘页面
static rt_err_t ui_page_create_menu(lv_obj_t *parent);              //创建菜单页面
static rt_err_t ui_page_create_system_info(lv_obj_t *parent);       //创建系统信息页面

static rt_err_t ui_page_apply_visibility(ui_page_id_t target);      //ui界面可见性函数
static rt_err_t ui_page_show(ui_page_id_t target);                  //切换到指定页面

static rt_bool_t ui_page_id_is_valid(ui_page_id_t page);            //检查页面id是否有效
static ui_page_id_t ui_page_next_id(ui_page_id_t current);          //获取下一个页面ID
static ui_page_id_t ui_page_back_id(ui_page_id_t current);          //获取返回页面ID

static void ui_page_cleanup(void);                                  //ui界面统一清理函数

static void ui_page_stat_increment(rt_uint32_t *counter);           //线程安全地增加一个32位统计计数
//---------------------------------------------------------------公开函数---------------------------------------------------------------

/*
 * 简介：创建三页最小UI
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
        ui_page_stat_increment(
            &g_ui_page_context.stats.create_fail);
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
        (g_ui_page_context.system_info_text != RT_NULL))
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
     * 在装载screen之前设置初始可见性。
     * 三个页面已经完整创建，因此不会出现半切换状态。
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
     * 8. 全部对象创建成功后再写入全局上下文。
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
 * 1. 本函数只能由UI线程调用；
 * 2. 菜单内容当前只是静态文字；
 * 3. 不创建按钮、点击回调、动画或真实应用入口；
 * 4. 全部创建成功后才把对象句柄写入全局上下文。
 */
static rt_err_t ui_page_create_menu(lv_obj_t *parent)
{
    lv_obj_t *page;
    lv_obj_t *title;
    lv_obj_t *items;

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
     * 7. 全部创建成功后再提交到全局上下文。
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
     * 9. 全部对象创建成功后再提交到全局上下文，
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

/*简介：ui界面统一清理函数
  输入: 无
  返回值：无
*/
static void ui_page_cleanup(void)
{
    lv_obj_t *root_screen;

    /*
     * 先保存根对象。
     * 后续统一清空上下文，避免删除过程中的回调观察到悬空指针。
     */
    root_screen = g_ui_page_context.root_screen;

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
     * root_screen是所有页面的共同父对象。
     * 删除它即可递归释放三个页面及其标签。
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
     * 本函数会同时操作三个页面，因此必须在任何修改前，
     * 确认根screen和三个页面对象都已经完整创建。
     */
    if ((g_ui_page_context.root_screen == RT_NULL) ||
        (g_ui_page_context.watchface_page == RT_NULL) ||
        (g_ui_page_context.menu_page == RT_NULL) ||
        (g_ui_page_context.system_info_page == RT_NULL))
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
    页面循环顺序：
    WATCHFACE → MENU
    MENU → SYSTEM_INFO
    SYSTEM_INFO → WATCHFACE
    非法状态 → UI_PAGE_COUNT
    */
    switch (current)
    {
    case UI_PAGE_WATCHFACE:
        return UI_PAGE_MENU;
    case UI_PAGE_MENU:
        return UI_PAGE_SYSTEM_INFO;
    case UI_PAGE_SYSTEM_INFO:
        return UI_PAGE_WATCHFACE;
    default:
        /*
         * 核心：
         * 即使前面已经检查过范围，也必须保留兜底返回。
         * 这样未来枚举增加空洞、状态损坏或代码调整时，
         * 不会出现非void函数没有返回值。
         */
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
    页面返回层级：
    WATCHFACE → WATCHFACE
    MENU → WATCHFACE
    SYSTEM_INFO → MENU
    非法状态 → UI_PAGE_COUNT
    */
    switch (current)
    {
    case UI_PAGE_WATCHFACE:
        /*
         * 表盘是根页面。
         * 在根页面执行BACK属于幂等成功，不进入非法状态。
         */
        return UI_PAGE_WATCHFACE;
    case UI_PAGE_MENU:
        return UI_PAGE_WATCHFACE;
    case UI_PAGE_SYSTEM_INFO:
        return UI_PAGE_MENU;
    default:
        /* 核心：禁止非法值沿页面切换链继续传播。 */
        return UI_PAGE_COUNT;
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
