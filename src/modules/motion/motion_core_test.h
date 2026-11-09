#ifndef SMARTWATCH_MOTION_CORE_TEST_H
#define SMARTWATCH_MOTION_CORE_TEST_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t groups_run;
    uint32_t checks;
    uint32_t passed;
    uint32_t failed;
    uint32_t setup_errors;
} motion_core_test_summary_t;

typedef void (*motion_core_test_report_t)(const char *name, bool passed, void *user_data);

/*
 * 简介：运行独立核心的九组确定性测试。
 * 输入：summary非NULL；report可为空；user_data仅在同步调用期间有效。
 * 返回值：全部通过0；测试或准备失败-1；参数非法-2。
 * 说明：调用者保证串行，不得在report回调中重入。只使用测试静态实例和
 *       虚拟tick，不访问RT设备、正式服务或系统时钟；不保存外部指针。
 */
int motion_core_test_run(motion_core_test_summary_t *summary,
                         motion_core_test_report_t report, void *user_data);

#endif
