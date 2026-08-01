//字符串转换宏
#define STRINGIFY_INNER(x) #x
#define STRINGIFY(x) STRINGIFY_INNER(x)

//缺省保护
#ifndef SIFLI_VERSION
#define SIFLI_VERSION 0
#endif

#ifndef SIFLI_BUILD
#define SIFLI_BUILD unknown
#endif

#ifndef SMARTWATCH_GIT_HASH
#define SMARTWATCH_GIT_HASH unknown
#endif

//dirty后缀
#ifdef SMARTWATCH_GIT_DIRTY
#define SMARTWATCH_GIT_SUFFIX "-dirty"
#else
#define SMARTWATCH_GIT_SUFFIX ""
#endif

#define SMARTWATCH_GIT_TEXT \
        STRINGIFY(SMARTWATCH_GIT_HASH) SMARTWATCH_GIT_SUFFIX

#include "system_diag.h"
#include "bf0_hal.h"


rt_err_t system_diag_init(void);
const system_diag_info_t *system_diag_get_info(void);
void system_diag_print_boot_info(void);


static  system_diag_info_t g_system_info=
{
    .project_name = "SF32LB52 SmartWatch",
    .board_name = "sf32lb52-lchspi-ulp",
    .project_version = "0.1.0-phase1",
    .build_type = "development",
    .project_commit = SMARTWATCH_GIT_TEXT,
    .sdk_commit = STRINGIFY(SIFLI_BUILD),
    .sdk_version_raw = SIFLI_VERSION,
    .pmu_wsr_raw = 0
};


rt_err_t system_diag_init(void)
{
    g_system_info.pmu_wsr_raw = HAL_PMU_GET_WSR();
    return RT_EOK;
}

const system_diag_info_t *system_diag_get_info(void)
{
    return &g_system_info;
}

void system_diag_print_boot_info(void)
{
    const system_diag_info_t *info = system_diag_get_info();
    rt_kprintf("[BOOT][01][INFO] Project Name: %s\n", info->project_name);
    rt_kprintf("[BOOT][01][INFO] Board Name: %s\n", info->board_name);
    rt_kprintf("[BOOT][01][INFO] Project Version: %s\n", info->project_version);
    rt_kprintf("[BOOT][01][INFO] Build Type: %s\n", info->build_type);
    rt_kprintf("[BOOT][01][INFO] Project Commit: %s\n", info->project_commit);
    rt_kprintf("[BOOT][01][INFO] SDK Commit: %s\n", info->sdk_commit);

    rt_kprintf("[BOOT][01][INFO] SDK Version Raw: 0x%08X\n", info->sdk_version_raw);
    uint32_t major = (info->sdk_version_raw >> 24) & 0xFF;
    uint32_t minor = (info->sdk_version_raw >> 16) & 0xFF;
    uint32_t revision = info->sdk_version_raw & 0xFFFF;
    rt_kprintf("[BOOT][01][INFO] SDK Version: %lu.%lu.%lu\n", major, minor, revision);

    rt_kprintf("[BOOT][01][INFO] PMU WSR Raw: 0x%08X\n", info->pmu_wsr_raw);

}



