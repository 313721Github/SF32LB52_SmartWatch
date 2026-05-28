# SmartWatch 项目架构文档

## 项目概览

基于 **RT-Thread** 实时操作系统 + **SiFli (思澈) SF32LB52** SDK + **LVGL** 图形库的智能手表 C 项目。

 清理命令 (Clean):

scons --board=sf32lb52-lchspi-ulp_hcpu -c

 编译命令 (Build):

scons --board=sf32lb52-lchspi-ulp_hcpu

下载/烧录命令 (Download)

.\build_sf32lb52-lchspi-ulp_hcpu\uart_download.bat



## 目录树

```
SmartWatch_Project/
├── sifli-sdk/                    # 官方 SDK（只读，禁止修改）
├── project/                      # 构建配置入口
│   ├── SConstruct                # SCons 顶层构建脚本
│   ├── SConscript                # 项目级编译脚本（引用 SDK + src）
│   ├── Kconfig / Kconfig.proj    # RT-Thread 内核配置
│   ├── proj.conf                 # Kconfig 默认配置值
│   ├── rtconfig.py               # 芯片/板级参数 (CHIP, OUTPUT_DIR 等)
│   ├── rtconfig_project.h        # 编译器相关宏
│   └── build_*/                  # 构建产物（不入版本控制）
└── src/                          # 应用层源码（MVC 架构）
    ├── SConscript                # 中央编译注册表（所有模块在此登记）
    │
    ├── main/                     # [入口层] 系统启动
    │   ├── SConscript
    │   └── main.c                # 系统入口 main()
    │
    ├── hal_port/                 # [硬件适配层] LCD / Touch / 背光 / 按键
    │   └── SConscript
    │
    ├── modules/                  # [数据层 Model] 传感器 / 蓝牙 / 计步 / 电源
    │   ├── SConscript
    │   └── mem/                  # LVGL 内存管理（SRAM/PSRAM 缓存分配）
    │       ├── SConscript
    │       ├── app_mem.c
    │       └── app_mem.h
    │
    ├── gui/                      # [视图层 View] LVGL 界面
    │   ├── SConscript
    │   └── resource/
    │       ├── fonts/            # FreeType 矢量字体 + 位图字体
    │       │   ├── SConscript
    │       │   ├── bitmap/       # lv_font_montserrat_*.c (21个位图字体)
    │       │   │   └── SConscript
    │       │   └── freetype/     # .ttf 矢量字体文件
    │       └── strings/          # 多语言翻译 .json
    │           └── SConscript
    │
    └── apps/                     # [应用层 Controller] 表盘 / 设置 / 通知
        └── SConscript
```

## MVC 分层职责

```
┌─────────────────────────────────────────────────┐
│  apps/  控制器层                                 │
│  表盘逻辑、设置菜单、通知管理                      │
│  把 modules/ 的数据和 gui/ 的界面粘合起来          │
├─────────────────────────────────────────────────┤
│  gui/   视图层                    modules/ 数据层 │
│  LVGL 控件、屏幕布局             传感器采集、蓝牙  │
│  字体/图片资源、动画             计步算法、电源管理 │
├─────────────────────────────────────────────────┤
│  hal_port/  硬件适配层                           │
│  LCD 驱动、触摸屏、背光、按键封装                  │
├─────────────────────────────────────────────────┤
│  main/  系统入口                                 │
│  main() 初始化各子系统，进入 RT-Thread 调度        │
└─────────────────────────────────────────────────┘
```

## SConscript 编译链

```
project/SConstruct
  └─ project/SConscript
       ├─ $SIFLI_SDK/SConscript          → sifli_sdk/
       └─ src/SConscript                  → src/
            ├─ main/SConscript            → main/
            ├─ hal_port/SConscript        → hal_port/
            ├─ modules/SConscript         → modules/
            ├─ modules/mem/SConscript     → modules_mem/
            ├─ gui/SConscript             → gui/
            ├─ apps/SConscript            → apps/
            ├─ gui/resource/strings/SConscript  → resource_strings/
            ├─ gui/resource/fonts/SConscript    → resource_fonts/
            └─ gui/resource/fonts/bitmap/SConscript → resource_bitmap_fonts/
```

## 全局头文件搜索路径 (CPPPATH)

`src/SConscript` 向所有源文件提供以下路径：

| 路径 | 用途 |
|------|------|
| `src/` | 跨模块引用根 (如 `#include "hal_port/lcd.h"`) |
| `src/main/` | main 模块头文件 |
| `src/hal_port/` | 硬件适配头文件 |
| `src/modules/` | 模块层头文件 |
| `src/modules/mem/` | 内存管理头文件 (app_mem.h) |
| `src/gui/` | GUI 头文件 |
| `src/apps/` | 应用层头文件 |

## 核心铁律

1. **禁止修改 `sifli-sdk/`** — 所有官方 SDK 源码只读
2. **移动/新建 .c 文件必须同步更新 SConscript** — 否则 GCC 找不到文件
3. **新模块登记** — 在 `src/` 下新建目录后，须在 `src/SConscript` 中补三处：
   - `inc` 列表（头文件路径）
   - `src.extend(SConscript(...))` 调用（编译链）
4. **子目录 SConscript 标准模板：**
   ```python
   import os
   from building import *
   
   cwd = GetCurrentDir()
   src = Glob('*.c')
   inc = [cwd]
   
   group = DefineGroup('module_name', src, depend = [''], CPPPATH = inc)
   Return('group')
   ```

## 当前源码统计

| 目录 | .c 文件 | .h 文件 | SConscript | 状态 |
|------|---------|---------|------------|------|
| `src/main/` | 1 (main.c) | 0 | 1 | 已有 |
| `src/hal_port/` | 0 | 0 | 1 | 待开发 |
| `src/modules/` | 0 | 0 | 1 | 待开发 |
| `src/modules/mem/` | 1 (app_mem.c) | 1 (app_mem.h) | 1 | 已有 |
| `src/gui/` | 0 | 0 | 1 | 待开发 |
| `src/gui/resource/fonts/bitmap/` | 21 | 0 | 1 | 已有 |
| `src/apps/` | 0 | 0 | 1 | 待开发 |
| **合计** | **23** | **1** | **11** | — |
