#ifndef UI_NOTIFICATION_H
#define UI_NOTIFICATION_H

#include <stddef.h>
#include <stdint.h>
#include "rtthread.h"
#include "ui_event.h"

#define UI_NOTICE_TITLE_MAX_BYTES 48U       // 标题最大UTF-8字节数，不含字符串终止符
#define UI_NOTICE_BODY_MAX_BYTES  160U      // 正文最大UTF-8字节数，不含字符串终止符
#define UI_NOTICE_RESULT_COUNT    4U        // 保留最近4个请求的终态结果
#define UI_NOTICE_TIMEOUT_MS      10000U    // 弹层默认显示时长，单位毫秒，使用单调时间计时

 typedef enum
{
    UI_NOTICE_PENDING = 0, // 请求等待UI处理，尚无终态结果
    UI_NOTICE_SHOWN,       // 本次请求已创建可见通知，项目侧显示处理完成
    UI_NOTICE_REPLACED,    // 本次请求已替换当前弹层内容
    UI_NOTICE_MERGED,      // 与当前通知内容重复，已合并且不延长显示时间
    UI_NOTICE_REJECTED     // UI处理阶段拒绝请求，具体原因见error
} ui_notice_outcome_t;

typedef struct
{
    uint32_t request_id;                    // 本次启动内的请求标识，0为无效值
    uint16_t title_length;                  // 标题有效UTF-8字节数，不含终止符，最大48
    uint16_t body_length;                   // 正文有效UTF-8字节数，不含终止符，最大160
    char title[UI_NOTICE_TITLE_MAX_BYTES + 1U]; // 标题值副本，额外1字节存放'\0'
    char body[UI_NOTICE_BODY_MAX_BYTES + 1U];   // 正文值副本，额外1字节存放'\0'
} ui_notice_message_t;

typedef struct
{
    uint32_t request_id;          // 此结果对应的通知请求标识
    ui_notice_outcome_t outcome;  // 请求状态，包括等待、显示、替换、合并或拒绝
    rt_err_t error;               // 处理错误码，RT_EOK表示无错误，需结合outcome判断
    rt_tick_t completed_tick;     // UI写入终态结果时的单调tick，PENDING时无完成时间含义
} ui_notice_result_t;

typedef struct
{
    uint32_t submit_attempt;    // 通知提交入口的累计调用次数
    uint32_t invalid;           // 因参数、长度、来源或编码非法而被拒绝的次数
    uint32_t busy;              // 因等待槽被占用而拒绝新请求的次数
    uint32_t post_fail;         // 已认领等待槽，但UI事件投递失败的次数
    uint32_t queued;            // 通知事件成功进入UI队列的累计次数
    uint32_t completed;         // UI已写入终态结果的累计次数，包含UI阶段的拒绝
    uint32_t active_request_id; // 当前占用等待槽的请求标识，0表示槽空闲
    rt_err_t last_error;        // 最近记录的错误码，RT_EOK表示该次操作无错误
} ui_notice_stats_t;

/* 普通线程调用。成功仅代表入队，输出ID用于查询处理结果。 */
rt_err_t ui_notification_submit(const char *title, size_t title_length, const char *body,
                                size_t body_length,ui_event_source_t source,uint32_t *request_id);

/* 仅UI线程领取。成功后须调用complete结束该请求。 */
rt_err_t ui_notification_take(uint32_t request_id, ui_notice_message_t *message);
rt_err_t ui_notification_complete(uint32_t request_id,  ui_notice_outcome_t outcome, rt_err_t error);

/* 线程间只复制值。结果保留最近4个终态，PENDING单独保存。 */
rt_err_t ui_notification_get_result(uint32_t request_id, ui_notice_result_t *result);
rt_err_t ui_notification_get_stats(ui_notice_stats_t *stats);

/* 供校验和UI字形回退共同使用。一次解码一个完整码点。 */
rt_err_t ui_notification_decode_utf8(const char *text, size_t length, size_t *offset, uint32_t *codepoint);

#endif