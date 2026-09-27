#include "ui_page.h"
#include "lvgl.h"
#include "lvsf_font.h"


static lv_obj_t *g_ui_screen = RT_NULL;         //ui界面屏幕
static lv_obj_t *g_ui_label = RT_NULL;          //ui界面标签
static rt_bool_t g_ui_page_created = RT_FALSE;  //ui界面是否创造成功

//公开函数声明
rt_err_t ui_page_create(void);                              //创建ui界面
rt_err_t ui_page_handle_event(const ui_event_t *event);     //ui事件处理函数
//---------------------------------------------------------------公开函数---------------------------------------------------------------

/*简介：创建ui界面
  输入：无
  返回值：rt_err_t：错误类型
*/
rt_err_t ui_page_create(void)
{
    if(g_ui_page_created==RT_TRUE)
    {
        return -RT_EBUSY;
    }
      /*
     * TODO 1：
     * 使用lv_obj_create(NULL)创建独立screen。
     * 失败时返回-RT_ENOMEM。
     */
    g_ui_screen=lv_obj_create(NULL);
    if(g_ui_screen==NULL)
    {
        return -RT_ENOMEM;
    }

    /*
     * TODO 2：
     * 设置screen背景：
     * - 深灰或近黑色
     * - LV_PART_MAIN | LV_STATE_DEFAULT
     * - LV_OPA_COVER
     */
    lv_obj_set_style_bg_color(g_ui_screen,lv_color_hex(0x20242A),LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(g_ui_screen,LV_OPA_COVER,LV_PART_MAIN | LV_STATE_DEFAULT);
    /*
     * TODO 3：
     * 在screen下创建label。
     *
     * 如果失败：
     * - 删除刚创建的screen；
     * - 清空g_ui_screen；
     * - 返回-RT_ENOMEM。
     */

    g_ui_label = lv_label_create(g_ui_screen);
    if (g_ui_label == NULL)
    {
        lv_obj_del(g_ui_screen);

        g_ui_screen = RT_NULL;
        g_ui_label = RT_NULL;

        return -RT_ENOMEM;
    }

    /*
     * TODO 4：
     * label文字设置为"LVGL v8 READY"。
     * 字体使用LV_FONT_DEFAULT。
     * 文字颜色设置为白色。
     * 使用lv_obj_center()居中。
     */
    lv_label_set_text(g_ui_label, "LVGL v8 READY");

    //error：使用此字体发生字体偏移撕裂错误
    // lv_obj_set_style_text_font(
    // g_ui_label,
    // &lv_font_montserrat_28,
    // LV_PART_MAIN | LV_STATE_DEFAULT);

    // lv_obj_set_style_text_color(g_ui_label,lv_color_hex(0xFFFFFF),LV_PART_MAIN | LV_STATE_DEFAULT);

    //字体使用SiFli lvsf_font接口的FONT_TITLE，
    //确保FreeType字形经过EPIC A2/A4行对齐兼容处理。
    lv_ext_set_local_font(g_ui_label,FONT_TITLE,LV_COLOR_WHITE);


    lv_obj_center(g_ui_label);

    /*
     * 页面构建完成后，再一次性装载为活动screen。
     */
    lv_scr_load(g_ui_screen);

    g_ui_page_created = RT_TRUE;
    return RT_EOK;
}

/*简介：ui事件处理函数
  输入：event：ui事件
  返回值：rt_err_t：错误类型
*/
rt_err_t ui_page_handle_event(const ui_event_t *event)
{
    if(event==RT_NULL)
    {
        return -RT_EINVAL;
    }

    //检查ui界面是否创建
    if(g_ui_page_created==RT_FALSE)
    {

        return -RT_EBUSY;
    }

    //检查screen和label有效
    if(g_ui_screen==RT_NULL || g_ui_label==RT_NULL)
    {
        return -RT_ERROR;
    }

    //处理ui事件
    switch (event->type)
    {
    case UI_ACTION_HOME:
        lv_label_set_text(g_ui_label, "UI HOME");
        break;
    case UI_ACTION_NEXT:
        lv_label_set_text(g_ui_label, "UI NEXT");
        break;
    case UI_ACTION_BACK:
        lv_label_set_text(g_ui_label, "UI BACK");
        break;
        case UI_ACTION_REFRESH_STATUS:
            lv_label_set_text_fmt(
                g_ui_label,
                "STATUS: %u",
                (unsigned int)event->value);
            break;
    default:
        return -RT_EINVAL;
    }

    return RT_EOK;
}
