#include "ui_notification.h"
#include "rthw.h"
#include <stdint.h>
#include <string.h>
#include "ui_runtime.h"

typedef enum
{
    UI_NOTICE_SLOT_FREE = 0,  // 等待槽空闲，可以接收新通知
    UI_NOTICE_SLOT_WRITING,   // 提交线程已认领槽，正在复制通知文本
    UI_NOTICE_SLOT_READY,     // 文本复制完成，等待UI线程按请求ID领取
    UI_NOTICE_SLOT_CONSUMING  // UI线程正在处理，写入终态结果后释放槽
} ui_notice_slot_state_t;

typedef struct
{
    ui_notice_slot_state_t slot_state; // 等待槽状态
    ui_notice_message_t message;       // 等待槽中的通知值副本
    ui_notice_result_t pending;        // 当前活动请求的PENDING记录

    ui_notice_result_t results[UI_NOTICE_RESULT_COUNT]; // 最近4个终态结果
    uint32_t result_write_index;      // 下一条终态结果的写入下标
    uint32_t last_request_id;         // 最近分配的请求ID

    ui_notice_stats_t stats;          // 提交、拒绝、入队和完成统计
} ui_notice_context_t;

/* 本模块唯一上下文，仅在当前.c文件内可见。 */
static ui_notice_context_t g_notice_context =
{
    .slot_state = UI_NOTICE_SLOT_FREE
};

//公开函数声明
/* 普通线程调用。成功仅代表入队，输出ID用于查询处理结果。 */
rt_err_t ui_notification_submit(const char *title, size_t title_length, const char *body,
                                size_t body_length,ui_event_source_t source,uint32_t *request_id);

/* 仅UI线程领取。成功后须调用complete结束该请求。 */
rt_err_t ui_notification_take(uint32_t request_id, ui_notice_message_t *message);       //按请求ID领取等待槽中的通知副本
rt_err_t ui_notification_complete(uint32_t request_id,  ui_notice_outcome_t outcome, rt_err_t error); //ui_notification_complete

rt_err_t ui_notification_get_result(uint32_t request_id, ui_notice_result_t *result);   //按请求ID查询通知处理结果
rt_err_t ui_notification_get_stats(ui_notice_stats_t *stats);                           //取得通知模块的统计副本
rt_err_t ui_notification_decode_utf8(const char *text, size_t length, size_t *offset, uint32_t *codepoint);//从指定位置解码一个完整的UTF-8码点

//私有函数声明
static rt_err_t ui_notification_validate_text(const char *text,size_t length,
                                            size_t maximum_length, rt_bool_t allow_newline);    //检查一段通知文本的长度、UTF-8编码和控制字符
//---------------------------------------------------------------公开函数---------------------------------------------------------------
/*
 * 简介：校验并复制一条通知，非阻塞投递UI事件。
 * 输入：title、body指向对应长度的可读数据，长度为0时指针可为空；
 *       title_length、body_length为有效字节数，不包含终止符；
 *       source为合法UI事件来源，request_id为非空输出指针。
 * 返回值：RT_EOK表示成功入队；参数或文本非法返回校验错误；
 *         等待槽被占用返回-RT_EBUSY，ID耗尽返回-RT_EFULL；
 *         UI投递失败时返回投递错误。
 * 说明：仅普通线程调用，调用期间输入文本必须保持稳定。
 *       成功后输出非零请求ID，失败时输出ID为0。
 *       入队成功后，通过get_result查询UI处理结果。
 */
rt_err_t ui_notification_submit(const char *title, size_t title_length,
                                const char *body, size_t body_length,
                                ui_event_source_t source,  uint32_t *request_id)
{
    rt_base_t level;
    ui_event_t event = {0};
    uint32_t reserved_id;
    rt_err_t result = -RT_EINVAL;

    /* 提交入口的所有调用均计入尝试次数。 */
    level = rt_hw_interrupt_disable();
    g_notice_context.stats.submit_attempt++;
    rt_hw_interrupt_enable(level);

    if (request_id == RT_NULL)
    {
        goto invalid_input;
    }

    *request_id = 0U;

    if (source < UI_EVENT_SOURCE_APP || source >= UI_EVENT_SOURCE_COUNT)
    {
        goto invalid_input;
    }

    result = ui_notification_validate_text( title, title_length, UI_NOTICE_TITLE_MAX_BYTES, RT_FALSE);
    if (result != RT_EOK)
    {
        goto invalid_input;
    }

    result = ui_notification_validate_text( body, body_length, UI_NOTICE_BODY_MAX_BYTES, RT_TRUE);
    if (result != RT_EOK)
    {
        goto invalid_input;
    }

    /* 认领槽和初始化活动请求必须作为一组操作完成。 */
    level = rt_hw_interrupt_disable();

    if (g_notice_context.slot_state != UI_NOTICE_SLOT_FREE)
    {
        g_notice_context.stats.busy++;
        g_notice_context.stats.last_error = -RT_EBUSY;

        rt_hw_interrupt_enable(level);
        return -RT_EBUSY;
    }

    /* 本次启动内不复用ID，防止迟到事件匹配到新请求。 */
    if (g_notice_context.last_request_id == UINT32_MAX)
    {
        g_notice_context.stats.last_error = -RT_EFULL;

        rt_hw_interrupt_enable(level);
        return -RT_EFULL;
    }

    reserved_id = g_notice_context.last_request_id + 1U;
    g_notice_context.last_request_id = reserved_id;

    g_notice_context.message.request_id = reserved_id;

    g_notice_context.pending.request_id = reserved_id;
    g_notice_context.pending.outcome = UI_NOTICE_PENDING;
    g_notice_context.pending.error = RT_EOK;
    g_notice_context.pending.completed_tick = 0;

    g_notice_context.stats.active_request_id = reserved_id;
    g_notice_context.slot_state = UI_NOTICE_SLOT_WRITING;

    rt_hw_interrupt_enable(level);

    /*
     * WRITING期间其它提交者会被拒绝，UI也不能领取。
     * 完整文本复制放在临界区外。
     */
    if (title_length > 0U)
    {
        memcpy(g_notice_context.message.title, title, title_length);
    }
    g_notice_context.message.title[title_length] = '\0';
    g_notice_context.message.title_length = (uint16_t)title_length;

    if (body_length > 0U)
    {
        memcpy(g_notice_context.message.body, body, body_length);
    }
    g_notice_context.message.body[body_length] = '\0';
    g_notice_context.message.body_length = (uint16_t)body_length;

    /* 文本完全就绪后，才允许UI领取。 */
    level = rt_hw_interrupt_disable();
    g_notice_context.slot_state = UI_NOTICE_SLOT_READY;
    rt_hw_interrupt_enable(level);

    event.type = UI_ACTION_NOTICE_READY;
    event.source = source;
    event.tick = rt_tick_get();
    event.value = reserved_id;

    result = ui_event_post(&event);

    level = rt_hw_interrupt_disable();

    if (result == RT_EOK)
    {
        /*
         * UI可能已经抢占并完成处理。
         * 此处只更新投递统计，不能再次修改槽或消息。
         */
        g_notice_context.stats.queued++;
        g_notice_context.stats.last_error = RT_EOK;

        rt_hw_interrupt_enable(level);

        *request_id = reserved_id;
        return RT_EOK;
    }

    /* 入队失败，回收本次认领的槽。 */
    g_notice_context.stats.post_fail++;
    g_notice_context.stats.last_error = result;

    if (g_notice_context.slot_state == UI_NOTICE_SLOT_READY && g_notice_context.message.request_id == reserved_id)
    {
        memset(&g_notice_context.pending, 0, sizeof(g_notice_context.pending));

        g_notice_context.stats.active_request_id = 0U;
        g_notice_context.slot_state = UI_NOTICE_SLOT_FREE;
    }
    else
    {
        /* 状态违反约定时保留现场，避免释放其它请求的槽。 */
        result = -RT_ERROR;
        g_notice_context.stats.last_error = result;
    }

    rt_hw_interrupt_enable(level);
    return result;

invalid_input:
    level = rt_hw_interrupt_disable();

    g_notice_context.stats.invalid++;
    g_notice_context.stats.last_error = result;

    rt_hw_interrupt_enable(level);
    return result;
}

/*
 * 简介：按请求ID领取等待槽中的通知副本。
 * 输入：request_id为UI事件携带的非零请求ID；message为非空、可写的输出指针。
 * 返回值：成功返回RT_EOK，参数或ID不匹配返回-RT_EINVAL，
 *         等待槽未处于READY时返回-RT_EBUSY。
 * 说明：仅UI线程调用。成功后槽保持CONSUMING，
 *       UI完成显示或拒绝处理后，必须调用complete释放槽。
 *       失败时不修改输出，也不释放当前等待槽。
 */
rt_err_t ui_notification_take(uint32_t request_id, ui_notice_message_t *message)
{
    rt_base_t level;

    if (request_id == 0U || message == RT_NULL)
    {
        return -RT_EINVAL;
    }

    level = rt_hw_interrupt_disable();

    /* 只有完整复制并发布的消息才能被领取。 */
    if (g_notice_context.slot_state != UI_NOTICE_SLOT_READY)
    {
        rt_hw_interrupt_enable(level);
        return -RT_EBUSY;
    }

    /* 拒绝迟到或错误的事件，保留当前有效消息。 */
    if (g_notice_context.message.request_id != request_id)
    {
        rt_hw_interrupt_enable(level);
        return -RT_EINVAL;
    }

    /* 先取得消费所有权，继续占用槽。 */
    g_notice_context.slot_state = UI_NOTICE_SLOT_CONSUMING;

    rt_hw_interrupt_enable(level);

    /*
     * CONSUMING期间生产者不能改写消息。
     * 将完整消息复制放在临界区外，缩短关中断时间。
     */
    *message = g_notice_context.message;

    return RT_EOK;
}
/*
 * 简介：记录通知请求的终态并释放等待槽。
 * 输入：request_id为当前CONSUMING请求的非零ID；
 *       outcome为显示、替换、合并或拒绝终态；
 *       error为处理结果，成功终态使用RT_EOK，拒绝使用非零错误码。
 * 返回值：成功返回RT_EOK，参数或ID不匹配返回-RT_EINVAL，
 *         槽未处于CONSUMING时返回-RT_EBUSY。
 * 说明：仅UI线程调用。先保存终态结果和统计，最后发布FREE。
 *       失败时保留当前等待槽及其请求。
 */
rt_err_t ui_notification_complete(uint32_t request_id, ui_notice_outcome_t outcome, rt_err_t error)
{
    rt_base_t level;
    uint32_t index;
    ui_notice_result_t completed_result;

    /* PENDING及未知枚举值均不能作为终态提交。 */
    if (request_id == 0U ||
        (outcome != UI_NOTICE_SHOWN &&
         outcome != UI_NOTICE_REPLACED &&
         outcome != UI_NOTICE_MERGED &&
         outcome != UI_NOTICE_REJECTED))
    {
        return -RT_EINVAL;
    }

    /* 保证终态与错误码含义一致。 */
    if ((outcome == UI_NOTICE_REJECTED && error == RT_EOK) ||
        (outcome != UI_NOTICE_REJECTED && error != RT_EOK))
    {
        return -RT_EINVAL;
    }

    level = rt_hw_interrupt_disable();

    if (g_notice_context.slot_state != UI_NOTICE_SLOT_CONSUMING)
    {
        rt_hw_interrupt_enable(level);
        return -RT_EBUSY;
    }

    /* 只允许结束当前已领取的请求。 */
    if (g_notice_context.message.request_id != request_id || g_notice_context.pending.request_id != request_id)
    {
        rt_hw_interrupt_enable(level);
        return -RT_EINVAL;
    }

    completed_result.request_id = request_id;
    completed_result.outcome = outcome;
    completed_result.error = error;
    completed_result.completed_tick = rt_tick_get();

    /* 写入结果环，覆盖当前位置的最旧记录。 */
    index = g_notice_context.result_write_index;
    g_notice_context.results[index] = completed_result;

    index++;
    if (index >= UI_NOTICE_RESULT_COUNT)
    {
        index = 0U;
    }
    g_notice_context.result_write_index = index;

    /* 完成统计包含UI阶段的拒绝。 */
    g_notice_context.stats.completed++;
    g_notice_context.stats.last_error = error;

    /* 终态保存后，清除活动记录，最后释放槽。 */
    memset(&g_notice_context.pending, 0, sizeof(g_notice_context.pending));

    g_notice_context.stats.active_request_id = 0U;
    g_notice_context.slot_state = UI_NOTICE_SLOT_FREE;

    rt_hw_interrupt_enable(level);

    return RT_EOK;
}
/*
 * 简介：从指定位置解码一个完整的UTF-8码点。
 * 输入：text指向至少length字节的可读数据；
 *       offset为非空输入输出，调用前须满足*offset < length；
 *       codepoint为非空输出，用于接收Unicode码点。
 * 返回值：RT_EOK表示成功，-RT_EINVAL表示参数或编码无效。
 * 说明：成功后推进offset，失败时两个输出均保持原值。
 *       函数无共享状态，可在普通线程中调用。
 *       U+0000和换行等字符的业务限制由文本校验函数处理。
 *      函数作用：从一段 UTF‑8 文本中读出一个字符对应的 Unicode 码点，并把读取位置移到下一个字符
 */
rt_err_t ui_notification_decode_utf8(const char *text,size_t length,size_t *offset,uint32_t *codepoint)
{
    size_t start;
    size_t byte_count;
    size_t index;
    uint8_t first;
    uint8_t next;
    uint32_t value;
    uint32_t minimum;

    if (text == RT_NULL || offset == RT_NULL || codepoint == RT_NULL)
    {
        return -RT_EINVAL;
    }

    //从哪里开始读
    start = *offset;
    if (start >= length)
    {
        return -RT_EINVAL;
    }

    /* 转为无符号字节，避免char的符号扩展影响位运算。 */
    first = (uint8_t)text[start];

    //根据首字节判断UTF-8编码的字节数
    if (first <= 0x7FU)
    {
        byte_count = 1U;
        value = first;
        minimum = 0U;
    }
    else if (first >= 0xC2U && first <= 0xDFU)
    {
        byte_count = 2U;
        value = first & 0x1FU;
        minimum = 0x80U;
    }
    else if (first >= 0xE0U && first <= 0xEFU)
    {
        byte_count = 3U;
        value = first & 0x0FU;
        minimum = 0x800U;
    }
    else if (first >= 0xF0U && first <= 0xF4U)
    {
        byte_count = 4U;
        value = first & 0x07U;
        minimum = 0x10000U;
    }
    else
    {
        return -RT_EINVAL;
    }

    /* 使用减法判断剩余长度，避免start + byte_count先发生溢出。 */
    if (byte_count > length - start)
    {
        return -RT_EINVAL;
    }

    for (index = 1U; index < byte_count; index++)
    {
        next = (uint8_t)text[start + index];

        /* UTF-8续字节的最高两位必须为10。 */
        if ((next & 0xC0U) != 0x80U)
        {
            return -RT_EINVAL;
        }

        value = (value << 6) | (uint32_t)(next & 0x3FU);
    }

    /* 拒绝过长编码和超出Unicode范围的值。 */
    if (value < minimum || value > 0x10FFFFU)
    {
        return -RT_EINVAL;
    }

    /* UTF-16代理区中的值不能作为UTF-8编码的独立码点。 */
    if (value >= 0xD800U && value <= 0xDFFFU)
    {
        return -RT_EINVAL;
    }

    /* 全部检查通过后，再一次性提交输出。 */
    *codepoint = value;
    *offset = start + byte_count;

    return RT_EOK;
}

/*
 * 简介：取得通知模块的统计副本。
 * 输入：stats为非空、可写的输出指针。
 * 返回值：成功返回RT_EOK，参数无效返回-RT_EINVAL。
 * 说明：普通线程调用，短临界区内复制统计，退出后写入输出。
 *       参数无效时不修改输出。
 */
rt_err_t ui_notification_get_stats(ui_notice_stats_t *stats)
{
    rt_base_t level;
    ui_notice_stats_t snapshot;

    if (stats == RT_NULL)
    {
        return -RT_EINVAL;
    }

    level = rt_hw_interrupt_disable();

    snapshot = g_notice_context.stats;

    rt_hw_interrupt_enable(level);

    *stats = snapshot;
    return RT_EOK;
}

/*
 * 简介：按请求ID查询通知处理结果。
 * 输入：request_id为非零请求ID，result为非空、可写的输出指针。
 * 返回值：找到记录返回RT_EOK，参数无效返回-RT_EINVAL，
 *         记录不存在或已被淘汰时返回-RT_ERROR。
 * 说明：普通线程调用，活动请求和终态结果在同一短临界区内查询。
 *       返回成功表示找到记录，处理是否完成须检查outcome。
 *       查询失败时不修改输出。
 */
rt_err_t ui_notification_get_result(uint32_t request_id, ui_notice_result_t *result)
{
    rt_base_t level;
    uint32_t index;
    rt_bool_t found = RT_FALSE;
    ui_notice_result_t snapshot;

    if (request_id == 0U || result == RT_NULL)
    {
        return -RT_EINVAL;
    }

    level = rt_hw_interrupt_disable();

    /* 当前活动请求可能仍在复制、等待UI或由UI处理。 */
    if (g_notice_context.slot_state != UI_NOTICE_SLOT_FREE && g_notice_context.pending.request_id == request_id)
    {
        snapshot = g_notice_context.pending;
        found = RT_TRUE;
    }
    else
    {
        /* 最多检查4个终态记录，临界区内工作量固定。 */
        for (index = 0U; index < UI_NOTICE_RESULT_COUNT; index++)
        {
            if (g_notice_context.results[index].request_id == request_id)
            {
                snapshot = g_notice_context.results[index];
                found = RT_TRUE;
                break;
            }
        }
    }

    rt_hw_interrupt_enable(level);

    if (found != RT_TRUE)
    {
        return -RT_ERROR;
    }

    *result = snapshot;
    return RT_EOK;
}

//---------------------------------------------------------------私有函数---------------------------------------------------------------
/*
 * 简介：检查一段通知文本的长度、UTF-8编码和控制字符。
 * 输入：text指向至少length字节的可读数据，length为0时可为空；
 *       maximum_length为允许的最大字节数，不包含终止符；
 *       allow_newline为RT_TRUE时允许换行字符U+000A。
 * 返回值：RT_EOK表示文本有效，非法输入返回-RT_EINVAL；
 *         UTF-8解码失败时返回解码器错误。
 * 说明：本函数只读取输入，无共享状态。
 *       输入可以尚未以'\0'结尾，检查严格限定在length字节内。
 */
static rt_err_t ui_notification_validate_text(const char *text,size_t length, size_t maximum_length,rt_bool_t allow_newline)
{
    size_t offset = 0U;
    uint32_t codepoint;
    rt_err_t result;

    /* 长度按UTF-8字节数判断。 */
    if (length > maximum_length)
    {
        return -RT_EINVAL;
    }

    /* 空文本无需读取内存，允许对应指针为空。 */
    if (length == 0U)
    {
        return RT_EOK;
    }

    if (text == RT_NULL)
    {
        return -RT_EINVAL;
    }

    while (offset < length)
    {
        result = ui_notification_decode_utf8(text,length, &offset, &codepoint);
        if (result != RT_EOK)
        {
            return result;
        }

        /* 换行单独判断，只有正文允许。 */
        if (codepoint == 0x0AU)
        {
            if (allow_newline != RT_TRUE)
            {
                return -RT_EINVAL;
            }

            continue;
        }

        /*
         * 拒绝C0控制字符、DEL和C1控制字符。
         * 包含NUL、制表符和回车。
         */
        if (codepoint < 0x20U ||
            (codepoint >= 0x7FU && codepoint <= 0x9FU))
        {
            return -RT_EINVAL;
        }
    }

    return RT_EOK;
}