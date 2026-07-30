#ifndef SYSTEM_DIAG_H
#define SYSTEM_DIAG_H

#include "rtthread.h"
#include <stdint.h>

typedef struct
{
    const char *project_name;   //项目名称      项目固定定义 这是什么项目？
    const char *board_name;     //板级名称      当前板型     固件为哪块板编译？
    const char *project_version;//项目版本      项目固定定义 当前手表软件版本？
    const char *build_type;     //构建类型      项目固定定义 开发版还是发布版？
    const char *project_commit; //项目提交ID    项目Git仓库  项目源码基于哪个提交？
    const char *sdk_commit;     //SDK提交ID     SIFLI_BUILD 使用哪个SDK提交构建？

    uint32_t sdk_version_raw;   //SDK版本号     SIFLI_VERSION SDK版本的原始编码？
    uint32_t pmu_wsr_raw;       //PMU WSR寄存器原始值   项目启动时读取到什么唤醒源信息？
} system_diag_info_t;

rt_err_t system_diag_init(void);    //初始化系统诊断模块
const system_diag_info_t *system_diag_get_info(void);   //获取系统诊断信息
void system_diag_print_boot_info(void); //打印系统启动信息

#endif

