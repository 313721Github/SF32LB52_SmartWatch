#include "ui_touch_diag.h"
#include "rthw.h"
#include "lvsf_font.h"
#include <string.h>

typedef struct
{
    lv_obj_t *root;
    lv_obj_t *targets[UI_TOUCH_DIAG_TARGET_COUNT];
    lv_obj_t *target_labels[UI_TOUCH_DIAG_TARGET_COUNT];
    lv_obj_t *point_marker;     // 触摸点标记
    lv_obj_t *status_label;     // 状态信息标签

    rt_tick_t last_report_tick; // 上次报告触摸事件的时间戳

    ui_touch_diag_stats_t stats;
} ui_touch_diag_context_t;

static ui_touch_diag_context_t g_touch_diag;

// 触摸目标描述
typedef struct
{
    ui_touch_target_t id;   // 目标 ID
    const char *name;       // 目标名称
    lv_coord_t x;           // 目标 X 坐标
    lv_coord_t y;           // 目标 Y 坐标
    lv_coord_t width;       // 目标宽度 width  = x1 - x0 + 1
    lv_coord_t height;      // 目标高度 height = y1 - y0 + 1
    rt_uint32_t rgb;       // 目标颜色
} ui_touch_target_desc_t;

static const ui_touch_target_desc_t g_target_desc[UI_TOUCH_DIAG_TARGET_COUNT] =
{
    {UI_TOUCH_TARGET_CENTER,       "CENTER", 145, 175, 100, 100, 0x334155},
    {UI_TOUCH_TARGET_TOP_LEFT,     "TL",       0,   0, 100, 100, 0x713434},
    {UI_TOUCH_TARGET_TOP_RIGHT,    "TR",     290,   0, 100, 100, 0x346344},
    {UI_TOUCH_TARGET_BOTTOM_LEFT,  "BL",       0, 350, 100, 100, 0x344B71},
    {UI_TOUCH_TARGET_BOTTOM_RIGHT, "BR",     290, 350, 100, 100, 0x716334},
    {UI_TOUCH_TARGET_TOP_EDGE,     "TOP",    145,   0, 100,  90, 0x534571},
    {UI_TOUCH_TARGET_BOTTOM_EDGE,  "BOTTOM", 145, 360, 100,  90, 0x345F63},
    {UI_TOUCH_TARGET_LEFT_EDGE,    "LEFT",     0, 180,  90, 130, 0x634B45},
    {UI_TOUCH_TARGET_RIGHT_EDGE,   "RIGHT",  300, 180,  90, 130, 0x456353}
};



//公开函数声明
rt_err_t ui_touch_diag_create(lv_obj_t *parent);                // 创建触摸诊断界面
rt_err_t ui_touch_diag_set_enabled(rt_bool_t enabled);          // 设置触摸诊断界面启用状态
rt_err_t ui_touch_diag_get_stats(ui_touch_diag_stats_t *stats); // 获取触摸诊断统计信息

//私有函数声明
static void ui_touch_diag_event_cb(lv_event_t *event);          // 触摸诊断界面事件回调函数

static rt_err_t ui_touch_diag_create_target(rt_uint32_t index);             // 创建触摸目标对象

static void ui_touch_diag_move_marker(const lv_point_t *point);             // 移动触摸点标记到指定坐标

static const char *ui_touch_diag_target_name(ui_touch_target_t target);     // 获取触摸目标名称

static const char *ui_touch_diag_event_name(ui_touch_diag_event_t event);   // 获取触摸事件名称

//---------------------------------------------------------------公开函数---------------------------------------------------------------
/* 简介: 创建触摸诊断界面
 * 输入: parent: 父对象
 * 返回值: rt_err_t: 错误码
 */
rt_err_t ui_touch_diag_create(lv_obj_t *parent)
{
    rt_err_t result = -RT_ENOMEM;
    rt_uint32_t index;
    rt_base_t level;
    if (parent == RT_NULL)
    {
        return -RT_EINVAL;
    }

    if (g_touch_diag.stats.created)
    {
        return RT_EOK;
    }

    // 创建根对象
    g_touch_diag.root = lv_obj_create(parent);
    if (g_touch_diag.root == RT_NULL)
    {
        goto create_failed;
    }

    lv_obj_remove_style_all(g_touch_diag.root);
    lv_obj_set_pos(g_touch_diag.root, 0, 0);
    lv_obj_set_size(g_touch_diag.root, 390, 450);
    lv_obj_set_style_bg_color(g_touch_diag.root, lv_color_hex(0x101820), 0);
    lv_obj_set_style_bg_opa(g_touch_diag.root, LV_OPA_COVER, 0);

    lv_obj_clear_flag(g_touch_diag.root, LV_OBJ_FLAG_SCROLLABLE);   // 清除 root 的“可滚动”标志，防止 root 被滚动
    lv_obj_clear_flag(g_touch_diag.root, LV_OBJ_FLAG_EVENT_BUBBLE); // 清除 root 的“事件冒泡”标志，防止事件传递到父对象
    lv_obj_add_flag(
        g_touch_diag.root,
        LV_OBJ_FLAG_CLICKABLE |
        LV_OBJ_FLAG_PRESS_LOCK |
        LV_OBJ_FLAG_HIDDEN);        // 给 root 添加“可点击”、“按下锁定”和“隐藏”标志，允许点击并锁定按下状态，同时初始状态为隐藏

    lv_obj_add_event_cb(
        g_touch_diag.root,
        ui_touch_diag_event_cb,
        LV_EVENT_ALL,
        (void *)(rt_ubase_t)UI_TOUCH_TARGET_NONE);  // 给 root 添加事件回调函数 ui_touch_diag_event_cb，监听所有事件，并传递目标 ID 为 UI_TOUCH_TARGET_NONE 作为用户数据

    // 创建触摸目标对象
    for (index = 0; index < UI_TOUCH_DIAG_TARGET_COUNT; index++)
    {
        result = ui_touch_diag_create_target(index);
        if (result != RT_EOK)
        {
            goto create_failed;
        }
    }

    /*
     * TODO：
     * 创建8×8的point_marker。
     * 创建status_label，建议放在(95,110)，宽200、高60。
     * 两者均清除CLICKABLE和SCROLLABLE。
     * 状态标签使用现有LVSF字体接口。
     * 每个创建调用后检查空指针。
     * 创建失败时result=-RT_ENOMEM，再goto create_failed。
     */
    // 创建触摸点标记
    g_touch_diag.point_marker = lv_obj_create(g_touch_diag.root);
    if (g_touch_diag.point_marker == RT_NULL)
    {
        result = -RT_ENOMEM;
        goto create_failed;
    }
    // 设置触摸点标记的样式和位置
    lv_obj_remove_style_all(g_touch_diag.point_marker);
    lv_obj_set_size(g_touch_diag.point_marker, 8, 8);
    lv_obj_set_pos(g_touch_diag.point_marker, 0, 0);

    lv_obj_set_style_bg_color(
        g_touch_diag.point_marker, LV_COLOR_WHITE, 0);
    lv_obj_set_style_bg_opa(
        g_touch_diag.point_marker, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(
        g_touch_diag.point_marker, LV_RADIUS_CIRCLE, 0);

    lv_obj_clear_flag(g_touch_diag.point_marker, LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);

    /* 首次取得合法触摸坐标后再显示标记。 */
    lv_obj_add_flag(
        g_touch_diag.point_marker, LV_OBJ_FLAG_HIDDEN);

    // 创建状态标签
    g_touch_diag.status_label = lv_label_create(g_touch_diag.root);
    if (g_touch_diag.status_label == RT_NULL)
    {
        result = -RT_ENOMEM;
        goto create_failed;
    }
    //设置标签位置和大小
    lv_obj_set_pos(g_touch_diag.status_label, 95, 110);
    lv_obj_set_size(g_touch_diag.status_label, 200, 60);

    lv_ext_set_local_font(g_touch_diag.status_label, FONT_NORMAL, LV_COLOR_WHITE);
    lv_label_set_text(g_touch_diag.status_label, "Touch a target");
    //清除CLICKABLE和SCROLLABLE
    lv_obj_clear_flag(g_touch_diag.status_label, LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
    /* 标记置于标签和目标上方，但不参与点击。 */
    lv_obj_move_foreground(g_touch_diag.point_marker);

    g_touch_diag.last_report_tick = 0;

    /* 所有对象创建成功后，统一发布成功状态。 */
    level = rt_hw_interrupt_disable();

    memset(&g_touch_diag.stats, 0, sizeof(g_touch_diag.stats));
    g_touch_diag.stats.created = RT_TRUE;
    g_touch_diag.stats.enabled = RT_FALSE;
    g_touch_diag.stats.pressed_active = RT_FALSE;
    g_touch_diag.stats.last_target = UI_TOUCH_TARGET_NONE;
    g_touch_diag.stats.last_error = RT_EOK;

    rt_hw_interrupt_enable(level);

    /* 必须返回，避免继续进入失败清理。 */
    return RT_EOK;
create_failed:
    /*
     * TODO：
     * 1. 若root存在，lv_obj_del(root)。
     * 2. 将所有对象句柄清空。
     * 3. 短临界区内清空stats，设置last_error=result。
     * 4. 清零last_report_tick。
     * 不在关中断区调用lv_obj_del()。
     */
    if(g_touch_diag.root != RT_NULL)
    {
        lv_obj_del(g_touch_diag.root);
    }

    g_touch_diag.root = RT_NULL;
    g_touch_diag.point_marker = RT_NULL;
    g_touch_diag.status_label = RT_NULL;

    for (index = 0; index < UI_TOUCH_DIAG_TARGET_COUNT; index++)
    {
        g_touch_diag.targets[index] = RT_NULL;
        g_touch_diag.target_labels[index] = RT_NULL;
    }

    level = rt_hw_interrupt_disable();
    memset(&g_touch_diag.stats, 0, sizeof(g_touch_diag.stats));
    g_touch_diag.stats.last_target = UI_TOUCH_TARGET_NONE;
    g_touch_diag.stats.last_error = result;
    rt_hw_interrupt_enable(level);

    return result;
}

/* 简介: 设置触摸诊断界面启用状态
 * 输入: enabled: 启用状态
 * 返回值: rt_err_t: 错误码
 *  未创建 → 返回错误
    已经启用，又请求启用 → 成功返回，不清空正在进行的测试
    已经停用，又请求停用 → 成功返回
    正在按压时请求停用 → 返回-RT_EBUSY，提示先抬手
 */
rt_err_t ui_touch_diag_set_enabled(rt_bool_t enabled)
{
    rt_base_t level;

    if (!g_touch_diag.stats.created)
    {
        return -RT_ERROR;
    }

    enabled = enabled ? RT_TRUE : RT_FALSE;

    if (g_touch_diag.stats.enabled == enabled)
    {
        return RT_EOK;
    }

    //正在按压时请求停用
    if (!enabled && g_touch_diag.stats.pressed_active)
    {
        return -RT_EBUSY;
    }


    if (enabled)
    {
        level = rt_hw_interrupt_disable();

        memset(&g_touch_diag.stats, 0, sizeof(g_touch_diag.stats));
        g_touch_diag.stats.created = RT_TRUE;
        g_touch_diag.stats.enabled = RT_TRUE;
        g_touch_diag.stats.last_target = UI_TOUCH_TARGET_NONE;

        rt_hw_interrupt_enable(level);

        g_touch_diag.last_report_tick = rt_tick_get();

        lv_label_set_text(g_touch_diag.status_label, "Touch a target");
        lv_obj_add_flag(g_touch_diag.point_marker, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(g_touch_diag.root);
        lv_obj_clear_flag(g_touch_diag.root, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        level = rt_hw_interrupt_disable();
        g_touch_diag.stats.enabled = RT_FALSE;
        rt_hw_interrupt_enable(level);

        /* 隐藏并保留对象，下一次启用时复用。 */
        lv_obj_add_flag(g_touch_diag.root, LV_OBJ_FLAG_HIDDEN);
    }
    return RT_EOK;
}

/* 简介: 获取触摸诊断统计信息
 * 输入: stats: 指向统计信息结构的指针
 * 返回值: rt_err_t: 错误码
 */
rt_err_t ui_touch_diag_get_stats(ui_touch_diag_stats_t *stats)
{
    rt_base_t level;

    if (stats == RT_NULL)
    {
        return -RT_EINVAL;
    }

    level = rt_hw_interrupt_disable();
    *stats = g_touch_diag.stats;
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}
//---------------------------------------------------------------私有函数---------------------------------------------------------------


/* 简介: 触摸诊断界面事件回调函数
 * 输入: event: 事件指针
 * 返回值:void
 */
static void ui_touch_diag_event_cb(lv_event_t *event)
{
    lv_event_code_t code;
    lv_indev_t *indev;
    lv_point_t point;
    ui_touch_target_t target;
    ui_touch_diag_event_t diag_event;   // 触摸诊断事件类型
    ui_touch_diag_stats_t snapshot;     // 触摸诊断统计信息快照
    rt_tick_t now;
    rt_base_t level;
    rt_bool_t valid;                    // 触摸点是否在有效范围内
    rt_bool_t report;                   // 是否需要报告触摸事件

    // 检查是否启用触摸诊断
    if (!g_touch_diag.stats.enabled)
    {
        return;
    }

    // 获取事件代码
    code = lv_event_get_code(event);

    // 按下意外丢失事件
    if (code == LV_EVENT_PRESS_LOST)
    {
        level = rt_hw_interrupt_disable();
        g_touch_diag.stats.press_lost_count++;
        g_touch_diag.stats.pressed_active = RT_FALSE;
        rt_hw_interrupt_enable(level);

        rt_kprintf("[UI][TOUCH][WARN] PRESS_LOST\n");
        return;
    }

    switch (code)
    {
    case LV_EVENT_PRESSED:
        diag_event = UI_TOUCH_DIAG_EVENT_PRESSED;
        break;

    case LV_EVENT_PRESSING:
        diag_event = UI_TOUCH_DIAG_EVENT_PRESSING;
        break;

    case LV_EVENT_RELEASED:
        diag_event = UI_TOUCH_DIAG_EVENT_RELEASED;
        break;

    default:
        return;
    }

    // 获取当前活动的输入设备
    indev = lv_indev_get_act();
    if (indev == RT_NULL)
    {
        level = rt_hw_interrupt_disable();
        g_touch_diag.stats.last_error = -RT_ERROR;
        rt_hw_interrupt_enable(level);
        return;
    }

    // 获取当前触摸点的坐标
    lv_indev_get_point(indev, &point);

    target = (ui_touch_target_t)(rt_ubase_t)
        lv_event_get_user_data(event);

    now = rt_tick_get();

    // 检查触摸点是否在有效范围内
    valid = (point.x >= 0 && point.x < 390 &&
             point.y >= 0 && point.y < 450);

    if (!valid)
    {
        target = UI_TOUCH_TARGET_OUT_OF_RANGE;
    }

    /*
     * TODO：短临界区内完成下一段统计规则，
     * 最后snapshot=g_touch_diag.stats，
     * 随后立即恢复中断。
     */
    /* 更新统计，并为后续日志和界面更新取得一致的快照。 */
    level = rt_hw_interrupt_disable();

    g_touch_diag.stats.event_sequence++;
    g_touch_diag.stats.last_tick = now;
    g_touch_diag.stats.last_x = point.x;
    g_touch_diag.stats.last_y = point.y;
    g_touch_diag.stats.last_event = diag_event;
    g_touch_diag.stats.last_target = target;

    if (!valid)
    {
        g_touch_diag.stats.out_of_range_count++;
    }

    switch (diag_event)
    {
    case UI_TOUCH_DIAG_EVENT_PRESSED:
        g_touch_diag.stats.pressed_count++;

        if (g_touch_diag.stats.pressed_active)
        {
            g_touch_diag.stats.press_without_release_count++;
        }

        g_touch_diag.stats.pressed_active = RT_TRUE;

        /* 每次按下只统计一次命中，非法坐标不计入命中。 */
        if (valid)
        {
            if ((unsigned int)target < UI_TOUCH_DIAG_TARGET_COUNT)
            {
                g_touch_diag.stats.target_hit_count[target]++;
            }
            else if (target == UI_TOUCH_TARGET_NONE)
            {
                g_touch_diag.stats.none_hit_count++;
            }
            else
            {
                /* 防止非法目标ID被用作数组下标。 */
                g_touch_diag.stats.last_error = -RT_EINVAL;
            }
        }
        break;

    case UI_TOUCH_DIAG_EVENT_PRESSING:
        g_touch_diag.stats.pressing_count++;
        /* 持续按压不重复增加目标命中次数。 */
        break;

    case UI_TOUCH_DIAG_EVENT_RELEASED:
        g_touch_diag.stats.released_count++;

        if (g_touch_diag.stats.pressed_active)
        {
            g_touch_diag.stats.completed_sequence_count++;
        }
        else
        {
            g_touch_diag.stats.release_without_press_count++;
        }

        g_touch_diag.stats.pressed_active = RT_FALSE;
        break;

    default:
        /* 前面的LVGL事件过滤保证正常不会进入这里。 */
        break;
    }

    snapshot = g_touch_diag.stats;

    rt_hw_interrupt_enable(level);

    /* LVGL对象操作必须在恢复中断之后执行。 */
    if (valid)
    {
        ui_touch_diag_move_marker(&point);
    }

    /*
     * TODO：合法坐标时调用ui_touch_diag_move_marker(&point)。
     */

    report = (!valid ||
            (diag_event != UI_TOUCH_DIAG_EVENT_PRESSING)) ||
            ((rt_tick_t)(now - g_touch_diag.last_report_tick) >=
            rt_tick_from_millisecond(50));

    if (report)
    {
        g_touch_diag.last_report_tick = now;

        rt_kprintf(
            "[UI][TOUCH] seq=%u tick=%u event=%s "
            "x=%d y=%d target=%s valid=%d\n",
            (unsigned int)snapshot.event_sequence,
            (unsigned int)snapshot.last_tick,
            ui_touch_diag_event_name(snapshot.last_event),
            (int)snapshot.last_x,
            (int)snapshot.last_y,
            ui_touch_diag_target_name(snapshot.last_target),
            (int)valid);

        lv_label_set_text_fmt(
            g_touch_diag.status_label,
            "%s (%d,%d)\n%s",
            ui_touch_diag_event_name(snapshot.last_event),
            (int)snapshot.last_x,
            (int)snapshot.last_y,
            ui_touch_diag_target_name(snapshot.last_target));
    }
}

/* 简介: 实现触点标记移动
 * 输入: point: 指向触摸点坐标的指针
 * 返回值:void
 */
static void ui_touch_diag_move_marker(const lv_point_t *point)
{
    lv_coord_t marker_x = point->x - 4;
    lv_coord_t marker_y = point->y - 4;

    if (marker_x < 0) marker_x = 0;
    if (marker_y < 0) marker_y = 0;
    if (marker_x > 382) marker_x = 382;
    if (marker_y > 442) marker_y = 442;

    lv_obj_set_pos(g_touch_diag.point_marker, marker_x, marker_y);
    lv_obj_clear_flag(g_touch_diag.point_marker, LV_OBJ_FLAG_HIDDEN);
}

/* 简介: 创建触摸目标对象
 * 输入: index: 触摸目标索引
 * 返回值: rt_err_t: 错误码
 */
static rt_err_t ui_touch_diag_create_target(rt_uint32_t index)
{

    const ui_touch_target_desc_t *desc;
    lv_obj_t *target;
    lv_obj_t *label;

    // 检查索引是否有效
    if (index >= UI_TOUCH_DIAG_TARGET_COUNT)
    {
        return -RT_EINVAL;
    }
    // 获取目标描述
    desc = &g_target_desc[index];

    // 创建目标对象
    target = lv_obj_create(g_touch_diag.root);
    if (target == RT_NULL)
    {
        return -RT_ENOMEM;
    }


    g_touch_diag.targets[index] = target;
    /* 去掉主题默认padding、圆角等，保证实际命中框符合设计。 */
    lv_obj_remove_style_all(target);
    lv_obj_set_pos(target, desc->x, desc->y);
    lv_obj_set_size(target, desc->width, desc->height);

    lv_obj_set_style_bg_color(target, lv_color_hex(desc->rgb), 0);
    lv_obj_set_style_bg_opa(target, LV_OPA_COVER, 0);                   //把背景不透明度设置为完全不透明
    lv_obj_set_style_border_width(target, 1, 0);                        //给 target 设置 1 像素宽的边框
    lv_obj_set_style_border_color(target, lv_color_hex(0x8090A0), 0);   //把边框颜色设置为 0x8090A0，这是一种灰蓝色

    lv_obj_clear_flag(target, LV_OBJ_FLAG_SCROLLABLE);                  //清除 target 的“可滚动”标志
    lv_obj_clear_flag(target, LV_OBJ_FLAG_EVENT_BUBBLE);                //清除 target 的“事件冒泡”标志，防止事件传递到父对象
    lv_obj_add_flag(target,LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK); //给 target 添加“可点击”和“按下锁定”标志（当 target 被按下后，即使手指或指针滑出了对象边界，它仍然保持按下状态），允许点击并锁定按下状态

    //给 target 添加事件回调函数 ui_touch_diag_event_cb，监听所有事件，并传递目标 ID 作为用户数据
    lv_obj_add_event_cb(
        target,
        ui_touch_diag_event_cb,
        LV_EVENT_ALL,
        (void *)(rt_ubase_t)desc->id);
    /*
    注意：CENTER的枚举值为0，所以它转换成user_data后可能是空指针值。
    这是合法的ID编码，回调不能把user_data == NULL当成错误

    */

    // 创建目标标签
    label = lv_label_create(target);
    if (label == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    g_touch_diag.target_labels[index] = label;

    lv_obj_clear_flag(label, LV_OBJ_FLAG_CLICKABLE);
    lv_label_set_text(label, desc->name);
    lv_ext_set_local_font(label, FONT_NORMAL, LV_COLOR_WHITE);
    lv_obj_center(label);

    return RT_EOK;
}


/* 简介：获取触摸目标名称
 * 输入：ui_touch_target_t：触摸目标类型
 * 返回值:const char *：目标名称
 */
static const char *ui_touch_diag_target_name(ui_touch_target_t target)
{
    if ((unsigned int)target < UI_TOUCH_DIAG_TARGET_COUNT)
    {
        return g_target_desc[target].name;
    }

    if (target == UI_TOUCH_TARGET_NONE)
    {
        return "NONE";
    }

    if (target == UI_TOUCH_TARGET_OUT_OF_RANGE)
    {
        return "OUT_OF_RANGE";
    }

    return "INVALID";
}


/* 简介：获取触摸事件名称
 * 输入：ui_touch_diag_event_t：触摸事件类型
 * 返回值:const char *：事件名称
 */
static const char *ui_touch_diag_event_name(ui_touch_diag_event_t event)
{
    switch (event)
    {
    case UI_TOUCH_DIAG_EVENT_PRESSED:
        return "PRESSED";

    case UI_TOUCH_DIAG_EVENT_PRESSING:
        return "PRESSING";

    case UI_TOUCH_DIAG_EVENT_RELEASED:
        return "RELEASED";

    default:
        return "NONE";
    }
}
