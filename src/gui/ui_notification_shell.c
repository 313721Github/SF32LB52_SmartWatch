#include <rtthread.h>

#ifdef RT_USING_FINSH

#include <finsh.h>
#include <stdint.h>
#include <string.h>

#include "ui_notification.h"
#include "ui_page.h"
#include "ui_runtime.h"
#include "ui_touch_diag.h"

#define NOTICE_DIAG_WAIT_MS       2000U
#define NOTICE_DIAG_POLL_MS        10U
#define NOTICE_DIAG_MAX_QUEUE      16U

/* 集中函数声明。 */
static rt_err_t notice_diag_parse_id(const char *text, uint32_t *value);
static rt_err_t notice_diag_status(void);
static rt_err_t notice_diag_submit_sample(const char *name,
                                         uint32_t *request_id);
static rt_err_t notice_diag_result(uint32_t request_id);
static rt_err_t notice_diag_close(uint32_t display_id);
static rt_err_t notice_diag_nav(const char *name);
static rt_err_t notice_diag_wait_idle(void);
static rt_err_t notice_diag_stress(rt_bool_t queue_full);
static void notice_diag_usage(void);
static int notice_diag(int argc, char **argv);

/*
 * 简介：解析非零十进制请求ID。
 * 输入：text为终止字符串，value为非空输出。
 * 返回值：RT_EOK或-RT_EINVAL。
 * 说明：拒绝符号、空白、尾随字符和32位溢出。
 */
static rt_err_t notice_diag_parse_id(const char *text, uint32_t *value)
{
    uint32_t number = 0U;
    uint32_t digit;
    const char *cursor;

    if (text == RT_NULL || value == RT_NULL || text[0] == '\0')
    {
        return -RT_EINVAL;
    }

    for (cursor = text; *cursor != '\0'; cursor++)
    {
        if (*cursor < '0' || *cursor > '9')
        {
            return -RT_EINVAL;
        }

        digit = (uint32_t)(*cursor - '0');

        if (number > (UINT32_MAX - digit) / 10U)
        {
            return -RT_EINVAL;
        }

        number = number * 10U + digit;
    }

    if (number == 0U)
    {
        return -RT_EINVAL;
    }

    *value = number;
    return RT_EOK;
}

/*
 * 简介：输出通知、页面和队列的诊断值。
 * 输入：无。
 * 返回值：RT_EOK或查询错误。
 * 说明：FinSH线程调用，只读取值副本。
 */
static rt_err_t notice_diag_status(void)
{
    ui_notice_stats_t notice;
    ui_notice_view_t view;
    ui_page_stats_t page;
    ui_runtime_stats_t runtime;
    rt_err_t result;

    result = ui_notification_get_stats(&notice);
    if (result != RT_EOK) return result;

    result = ui_page_get_notice_view(&view);
    if (result != RT_EOK) return result;

    result = ui_page_get_stats(&page);
    if (result != RT_EOK) return result;

    result = ui_runtime_get_stats(&runtime);
    if (result != RT_EOK) return result;

    rt_kprintf(
        "[NOTICE][VIEW] tick=%lu visible=%u display=%lu "
        "latest=%lu has_latest=%u return=%d deadline=%lu\n",
        (unsigned long)view.sample_tick,
        (unsigned int)view.visible,
        (unsigned long)view.display_id,
        (unsigned long)view.latest_id,
        (unsigned int)view.has_latest,
        (int)view.return_page,
        (unsigned long)view.deadline_tick);

    rt_kprintf(
        "[NOTICE][SERVICE] attempt=%lu invalid=%lu busy=%lu "
        "post_fail=%lu queued=%lu completed=%lu active=%lu error=%d\n",
        (unsigned long)notice.submit_attempt,
        (unsigned long)notice.invalid,
        (unsigned long)notice.busy,
        (unsigned long)notice.post_fail,
        (unsigned long)notice.queued,
        (unsigned long)notice.completed,
        (unsigned long)notice.active_request_id,
        (int)notice.last_error);

    rt_kprintf(
        "[NOTICE][PAGE] current=%d shown=%lu replaced=%lu merged=%lu "
        "manual=%lu timeout=%lu diag=%lu stale=%lu missing=%lu\n",
        (int)page.current_page,
        (unsigned long)page.notice_shown_count,
        (unsigned long)page.notice_replaced_count,
        (unsigned long)page.notice_merged_count,
        (unsigned long)page.notice_manual_close_count,
        (unsigned long)page.notice_timeout_close_count,
        (unsigned long)page.notice_diag_close_count,
        (unsigned long)page.notice_stale_close_count,
        (unsigned long)page.notice_missing_glyph_count);

    rt_kprintf(
        "[NOTICE][QUEUE] state=%d depth=%u posted=%lu "
        "consumed=%lu dropped=%lu handled=%lu failed=%lu\n",
        (int)runtime.state,
        (unsigned int)runtime.queue_depth,
        (unsigned long)runtime.posted,
        (unsigned long)runtime.consumed,
        (unsigned long)runtime.dropped,
        (unsigned long)runtime.handled,
        (unsigned long)runtime.handle_fail);

    return RT_EOK;
}

/*
 * 简介：生成一个有限测试样本并走正式提交入口。
 * 输入：name为样本名，request_id为非空输出。
 * 返回值：正式提交结果，未知样本返回-RT_EINVAL。
 * 说明：输入在函数返回前已被服务复制。
 *       非法样本保留原始字节长度，不能用strlen计算其有效长度。
 */
static rt_err_t notice_diag_submit_sample(const char *name,
                                         uint32_t *request_id)
{
    static const char bad_utf8[] = { (char)0xC0, (char)0xAF };
    static const char embedded_nul[] = { 'A', '\0', 'B' };

    char title_buffer[UI_NOTICE_TITLE_MAX_BYTES + 2U];
    char body_buffer[UI_NOTICE_BODY_MAX_BYTES + 2U];

    const char *title = "Local notice";
    const char *body = "Message A";
    size_t title_length;
    size_t body_length;

    if (name == RT_NULL || request_id == RT_NULL)
    {
        return -RT_EINVAL;
    }

    *request_id = 0U;

    title_length = strlen(title);
    body_length = strlen(body);

    if (strcmp(name, "basic") == 0)
    {
        /* 重复发送同一内容，用于验证合并且不续时。 */
    }
    else if (strcmp(name, "alt") == 0)
    {
        body = "Message B";
        body_length = strlen(body);
    }
    else if (strcmp(name, "empty") == 0)
    {
        body = RT_NULL;
        body_length = 0U;
    }
    else if (strcmp(name, "max") == 0)
    {
        memset(title_buffer, 'T', UI_NOTICE_TITLE_MAX_BYTES);
        title_buffer[UI_NOTICE_TITLE_MAX_BYTES] = '\0';

        memset(body_buffer, 'B', UI_NOTICE_BODY_MAX_BYTES);
        body_buffer[UI_NOTICE_BODY_MAX_BYTES] = '\0';

        title = title_buffer;
        body = body_buffer;
        title_length = UI_NOTICE_TITLE_MAX_BYTES;
        body_length = UI_NOTICE_BODY_MAX_BYTES;
    }
    else if (strcmp(name, "mixed") == 0)
    {
        title = "本地通知";
        body = "第一行中文\nSecond line\nEmoji: \xF0\x9F\x93\xA9";
        title_length = strlen(title);
        body_length = strlen(body);
    }
    else if (strcmp(name, "bad_utf8") == 0)
    {
        body = bad_utf8;
        body_length = sizeof(bad_utf8);
    }
    else if (strcmp(name, "nul") == 0)
    {
        body = embedded_nul;
        body_length = sizeof(embedded_nul);
    }
    else if (strcmp(name, "over_title") == 0)
    {
        memset(title_buffer, 'T', sizeof(title_buffer) - 1U);
        title_buffer[sizeof(title_buffer) - 1U] = '\0';

        title = title_buffer;
        title_length = UI_NOTICE_TITLE_MAX_BYTES + 1U;
    }
    else if (strcmp(name, "over_body") == 0)
    {
        memset(body_buffer, 'B', sizeof(body_buffer) - 1U);
        body_buffer[sizeof(body_buffer) - 1U] = '\0';

        body = body_buffer;
        body_length = UI_NOTICE_BODY_MAX_BYTES + 1U;
    }
    else
    {
        return -RT_EINVAL;
    }

    return ui_notification_submit(
        title, title_length, body, body_length,
        UI_EVENT_SOURCE_TEST, request_id);
}

/*
 * 简介：查询一个仍被保留的请求结果。
 * 输入：request_id为非零ID。
 * 返回值：RT_EOK或查询错误。
 * 说明：PENDING表示尚未完成；显示结果不代表物理扫描完成。
 */
static rt_err_t notice_diag_result(uint32_t request_id)
{
    ui_notice_result_t value;
    rt_err_t result;

    result = ui_notification_get_result(request_id, &value);
    if (result != RT_EOK)
    {
        rt_kprintf("[NOTICE][RESULT] id=%lu lookup=%d\n",
                   (unsigned long)request_id, (int)result);
        return result;
    }

    rt_kprintf(
        "[NOTICE][RESULT] id=%lu outcome=%u error=%d completed_tick=%lu\n",
        (unsigned long)value.request_id,
        (unsigned int)value.outcome,
        (int)value.error,
        (unsigned long)value.completed_tick);

    return RT_EOK;
}

/*
 * 简介：投递指定显示ID的关闭动作。
 * 输入：display_id为非零显示ID。
 * 返回值：UI事件投递结果。
 * 说明：只投递事件，实际是否关闭通过status及关闭计数核对。
 */
static rt_err_t notice_diag_close(uint32_t display_id)
{
    ui_event_t event = {0};
    rt_err_t result;

    event.type = UI_ACTION_NOTICE_CLOSE;
    event.source = UI_EVENT_SOURCE_TEST;
    event.tick = rt_tick_get();
    event.value = display_id;

    result = ui_event_post(&event);

    rt_kprintf("[NOTICE][CLOSE_POST] id=%lu result=%d\n",
               (unsigned long)display_id, (int)result);

    return result;
}

/*
 * 简介：通过正式UI事件入口投递有限导航动作。
 * 输入：name为导航名。
 * 返回值：投递结果或-RT_EINVAL。
 * 说明：Shell注入验证软件分发，真实滑动和点按仍需实物测试。
 */
static rt_err_t notice_diag_nav(const char *name)
{
    ui_event_t event = {0};
    ui_page_stats_t page;
    rt_err_t result;

    event.source = UI_EVENT_SOURCE_TEST;
    event.tick = rt_tick_get();

    if (strcmp(name, "home") == 0)
        event.type = UI_ACTION_HOME;
    else if (strcmp(name, "menu") == 0)
        event.type = UI_ACTION_OPEN_MENU;
    else if (strcmp(name, "back") == 0)
        event.type = UI_ACTION_BACK;
    else if (strcmp(name, "left") == 0 ||
             strcmp(name, "right") == 0)
    {
        result = ui_page_get_stats(&page);
        if (result != RT_EOK) return result;

        event.type = strcmp(name, "left") == 0
                   ? UI_ACTION_SWIPE_LEFT : UI_ACTION_SWIPE_RIGHT;
        event.value = (rt_uint32_t)page.current_page;
    }
    else
    {
        event.type = UI_ACTION_OPEN_DETAIL;

        if (strcmp(name, "sensor") == 0)
            event.value = UI_PAGE_SENSOR_DIAG;
        else if (strcmp(name, "settings") == 0)
            event.value = UI_PAGE_SETTINGS;
        else if (strcmp(name, "info") == 0)
            event.value = UI_PAGE_SYSTEM_INFO;
        else
            return -RT_EINVAL;
    }

    result = ui_event_post(&event);
    rt_kprintf("[NOTICE][NAV_POST] action=%u value=%lu result=%d\n",
               (unsigned int)event.type,
               (unsigned long)event.value,
               (int)result);

    return result;
}

/*
 * 简介：有界等待通知槽和UI事件队列排空。
 * 输入：无。
 * 返回值：RT_EOK、查询错误或-RT_ETIMEOUT。
 * 说明：FinSH线程短暂休眠让UI运行，不复位或强制清空队列。
 */
static rt_err_t notice_diag_wait_idle(void)
{
    ui_notice_stats_t notice;
    ui_runtime_stats_t runtime;
    rt_tick_t start = rt_tick_get();
    rt_tick_t limit = rt_tick_from_millisecond(NOTICE_DIAG_WAIT_MS);
    rt_err_t result;

    if (limit == 0U) limit = 1U;

    do
    {
        result = ui_notification_get_stats(&notice);
        if (result != RT_EOK) return result;

        result = ui_runtime_get_stats(&runtime);
        if (result != RT_EOK) return result;

        if (notice.active_request_id == 0U &&
            runtime.posted == runtime.consumed)
        {
            return RT_EOK;
        }

        rt_thread_mdelay(NOTICE_DIAG_POLL_MS);
    }
    while ((rt_tick_t)(rt_tick_get() - start) < limit);

    return -RT_ETIMEOUT;
}

/*
 * 简介：验证一次槽忙或满队列拒绝，并检查恢复。
 * 输入：queue_full为真测队列满，否则测等待槽忙。
 * 返回值：全部判据满足返回RT_EOK，否则返回错误。
 * 说明：仅显式调用时执行。调度锁内只有有限的非阻塞投递，
 *       无打印、等待或硬件动作，退出调度锁后有界等待恢复。
 */
static rt_err_t notice_diag_stress(rt_bool_t queue_full)
{
    static const char title[] = "NOTICE_TEST";
    static const char body[] = "Bounded pressure";

    ui_notice_stats_t before;
    ui_notice_stats_t after;
    ui_runtime_stats_t runtime;
    ui_touch_diag_stats_t touch;
    ui_notice_result_t terminal;
    ui_event_t event = {0};

    uint32_t first_id = 0U;
    uint32_t second_id = 0U;
    rt_uint32_t index;
    rt_err_t result;
    rt_err_t first_result = -RT_ERROR;
    rt_err_t second_result = -RT_ERROR;
    rt_err_t fill_result = RT_EOK;
    rt_bool_t passed;

    result = ui_notification_get_stats(&before);
    if (result != RT_EOK) return result;

    result = ui_runtime_get_stats(&runtime);
    if (result != RT_EOK) return result;

    result = ui_touch_diag_get_stats(&touch);
    if (result != RT_EOK) return result;

    if (runtime.state != UI_RUNTIME_STATE_READY ||
        runtime.queue_ready != RT_TRUE ||
        runtime.queue_depth == 0U ||
        runtime.queue_depth > NOTICE_DIAG_MAX_QUEUE ||
        runtime.posted != runtime.consumed ||
        before.active_request_id != 0U ||
        touch.enabled == RT_TRUE)
    {
        rt_kprintf("[NOTICE][STRESS] result=BUSY_PRECONDITION\n");
        return -RT_EBUSY;
    }

    event.type = UI_ACTION_REFRESH_STATUS;
    event.source = UI_EVENT_SOURCE_TEST;

    rt_enter_critical();

    if (queue_full == RT_TRUE)
    {
        for (index = 0U; index < runtime.queue_depth; index++)
        {
            event.tick = rt_tick_get();
            event.value = index;
            fill_result = ui_event_post(&event);

            if (fill_result != RT_EOK)
            {
                break;
            }
        }

        if (fill_result == RT_EOK)
        {
            first_result = ui_notification_submit(
                title, sizeof(title) - 1U,
                body, sizeof(body) - 1U,
                UI_EVENT_SOURCE_TEST, &first_id);
        }
    }
    else
    {
        first_result = ui_notification_submit(
            title, sizeof(title) - 1U,
            body, sizeof(body) - 1U,
            UI_EVENT_SOURCE_TEST, &first_id);

        second_result = ui_notification_submit(
            title, sizeof(title) - 1U,
            body, sizeof(body) - 1U,
            UI_EVENT_SOURCE_TEST, &second_id);
    }

    rt_exit_critical();

    result = notice_diag_wait_idle();
    if (result != RT_EOK)
    {
        rt_kprintf("[NOTICE][STRESS] idle_error=%d\n", (int)result);
        return result;
    }

    if (queue_full == RT_TRUE)
    {
        passed = fill_result == RT_EOK &&
                 first_result == -RT_EFULL &&
                 first_id == 0U;

        if (passed != RT_TRUE)
        {
            rt_kprintf(
                "[NOTICE][STRESS] mode=full fill=%d rejected=%d id=%lu\n",
                (int)fill_result, (int)first_result,
                (unsigned long)first_id);
            return -RT_ERROR;
        }

        /* 队列排空后，重新提交一条，验证槽已回收。 */
        second_result = ui_notification_submit(
            title, sizeof(title) - 1U,
            body, sizeof(body) - 1U,
            UI_EVENT_SOURCE_TEST, &second_id);

        if (second_result != RT_EOK) return second_result;

        result = notice_diag_wait_idle();
        if (result != RT_EOK) return result;

        result = ui_notification_get_result(second_id, &terminal);
    }
    else
    {
        passed = first_result == RT_EOK &&
                 first_id != 0U &&
                 second_result == -RT_EBUSY &&
                 second_id == 0U;

        if (passed != RT_TRUE)
        {
            rt_kprintf(
                "[NOTICE][STRESS] mode=busy first=%d second=%d\n",
                (int)first_result, (int)second_result);
            return -RT_ERROR;
        }

        result = ui_notification_get_result(first_id, &terminal);
    }

    if (result != RT_EOK) return result;

    result = ui_notification_get_stats(&after);
    if (result != RT_EOK) return result;

    passed = terminal.error == RT_EOK &&
             (terminal.outcome == UI_NOTICE_SHOWN ||
              terminal.outcome == UI_NOTICE_REPLACED ||
              terminal.outcome == UI_NOTICE_MERGED) &&
             after.active_request_id == 0U &&
             (uint32_t)(after.queued - before.queued) == 1U &&
             (uint32_t)(after.completed - before.completed) == 1U;

    if (queue_full == RT_TRUE)
    {
        passed = passed &&
                 (uint32_t)(after.post_fail - before.post_fail) == 1U;
    }
    else
    {
        passed = passed &&
                 (uint32_t)(after.busy - before.busy) == 1U;
    }

    rt_kprintf(
        "[NOTICE][STRESS] mode=%s pass=%u "
        "queued_delta=%lu completed_delta=%lu busy_delta=%lu "
        "post_fail_delta=%lu active=%lu\n",
        queue_full ? "full" : "busy",
        (unsigned int)passed,
        (unsigned long)(after.queued - before.queued),
        (unsigned long)(after.completed - before.completed),
        (unsigned long)(after.busy - before.busy),
        (unsigned long)(after.post_fail - before.post_fail),
        (unsigned long)after.active_request_id);

    return passed ? RT_EOK : -RT_ERROR;
}

/*
 * 简介：打印诊断命令用法。
 * 输入：无。返回值：无。
 * 说明：仅FinSH线程调用。
 */
static void notice_diag_usage(void)
{
    rt_kprintf("notice_diag status\n");
    rt_kprintf("notice_diag send basic|alt|empty|max|mixed\n");
    rt_kprintf("notice_diag send bad_utf8|nul|over_title|over_body\n");
    rt_kprintf("notice_diag result <id>\n");
    rt_kprintf("notice_diag close <display_id>\n");
    rt_kprintf("notice_diag nav home|menu|back|left|right\n");
    rt_kprintf("notice_diag nav sensor|settings|info\n");
    rt_kprintf("notice_diag stress busy|full\n");
}

/*
 * 简介：分发有限通知诊断命令。
 * 输入：argc、argv由FinSH提供。
 * 返回值：实际操作结果。
 * 说明：通知和导航均走正式事件入口，测试默认不运行。
 */
static int notice_diag(int argc, char **argv)
{
    uint32_t id;
    rt_err_t result;

    if (argc == 1)
    {
        notice_diag_usage();
        return RT_EOK;
    }

    if (argc == 2 && strcmp(argv[1], "status") == 0)
    {
        return notice_diag_status();
    }

    if (argc == 3 && strcmp(argv[1], "send") == 0)
    {
        id = 0U;
        result = notice_diag_submit_sample(argv[2], &id);

        rt_kprintf("[NOTICE][SUBMIT] sample=%s result=%d id=%lu\n",
                   argv[2], (int)result, (unsigned long)id);
        return result;
    }

    if (argc == 3 &&
        (strcmp(argv[1], "result") == 0 ||
         strcmp(argv[1], "close") == 0))
    {
        result = notice_diag_parse_id(argv[2], &id);
        if (result != RT_EOK) return result;

        if (strcmp(argv[1], "result") == 0)
        {
            return notice_diag_result(id);
        }

        return notice_diag_close(id);
    }

    if (argc == 3 && strcmp(argv[1], "nav") == 0)
    {
        return notice_diag_nav(argv[2]);
    }

    if (argc == 3 && strcmp(argv[1], "stress") == 0)
    {
        if (strcmp(argv[2], "busy") == 0)
            return notice_diag_stress(RT_FALSE);

        if (strcmp(argv[2], "full") == 0)
            return notice_diag_stress(RT_TRUE);
    }

    notice_diag_usage();
    return -RT_EINVAL;
}

MSH_CMD_EXPORT(notice_diag, bounded local notification diagnostics);

#endif
