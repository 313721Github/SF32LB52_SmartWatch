# SmartWatch 项目架构与构建说明

> 适用项目：`D:\Embedded_App\SF32LB52\SiFil_Project\SmartWatch_Project`  
> 适用SDK：SiFli SDK v2.4.6，`release/v2.4`  
> 当前开发板：黄山派 SF32LB52-ULP  
> 当前板型参数：`sf32lb52-lchspi-ulp`

## 1. 先理解项目与SDK的关系

本工程由两个彼此独立、在构建时连接的目录组成：

```text
D:\Embedded_App\SF32LB52\
├─ SiFil_Project\SmartWatch_Project\   ← 你自己的项目
└─ SiFli_SDK\sifli-sdk\                ← 官方SDK
```

### 1.1 SmartWatch_Project负责什么

项目目录保存：

- 你的应用源码；
- 项目功能配置；
- SCons构建入口；
- VS Code工作区和任务；
- 项目文档；
- 构建后生成的固件和下载脚本。

以后表盘、传感器服务、BLE应用协议、设置和业务逻辑主要写在项目的 `src` 下。

### 1.2 sifli-sdk负责什么

SDK提供：

- SF32LB52芯片启动、CMSIS和寄存器定义；
- HAL驱动；
- RT-Thread内核及设备驱动；
- 开发板BSP；
- CO5300、FT6146等板级外设驱动；
- LVGL、EPIC、eZip、Bluetooth等中间件；
- SCons/Kconfig构建工具；
- Bootloader、FTAB、下载工具接口和官方示例。

SDK不是“开发板所有功能已经自动完成”，而是提供可以被项目配置和调用的基础能力。

### 1.3 两个目录如何建立连接

真正的构建连接不是VS Code左侧同时显示两个文件夹，而是下面这条链：

```text
SiFli开发终端设置 SIFLI_SDK 环境变量
        │
        ▼
project/SConstruct 读取 SIFLI_SDK
        │
        ▼
project/SConscript 导入 sifli-sdk/SConscript
        │
        ├─ 编译SDK、RT-Thread、BSP和中间件
        └─ 导入项目 src/SConscript，编译自己的代码
```

`project/SConstruct` 中会检查 `SIFLI_SDK`。没有这个环境变量时，构建会提示先设置SDK环境并退出。

`SmartWatch.code-workspace` 把项目和SDK同时放进VS Code工作区，主要用于浏览、跳转和代码补全；它不是编译连接本身。

### 1.4 只复制项目到新电脑能否编译

不能直接编译。项目源码可以复制，但新电脑仍需具备：

1. 兼容版本的SiFli SDK，当前基线是v2.4.6；
2. SiFli构建环境、Python、SCons和Arm GNU工具链；
3. 正确的 `SIFLI_SDK` 环境变量；
4. 下载工具和开发板驱动；
5. 正确板型参数。

SDK不要求放在旧电脑的同一个绝对路径，只要新电脑重新配置 `SIFLI_SDK` 即可。但 `.vscode/c_cpp_properties.json` 中可能存在旧电脑的绝对路径，迁移后应通过当前构建重新生成编译数据库或更新IDE配置。

项目仓库不应把完整SDK复制进来。更合理的方式是：项目README记录所需SDK版本和提交，新电脑单独安装对应SDK。

## 2. 当前板型与 `_base` 的关系

SDK中与当前开发板相关的目录是：

```text
customer/boards/
├─ sf32lb52-lchspi-ulp/
└─ sf32lb52-lchspi-ulp_base/
```

### 2.1 `sf32lb52-lchspi-ulp`

这是构建命令使用的具体板型入口。它表示黄山派/立创高速SPI ULP板型，负责选择这块板对应的配置、分区表及基础板型定义。

其中 `Kconfig.board` 会继续引用 `_base` 目录的配置。也就是说，这个目录更像“具体型号入口”。

### 2.2 `sf32lb52-lchspi-ulp_base`

这是该系列板型共用的基础实现，包含：

- `bsp_init.c`：时钟、PSRAM、Flash和基础硬件预初始化；
- `bsp_pinmux.c`：LCD、触摸、UART、TF、充电I2C等引脚复用；
- `bsp_lcd_tp.c`：LCD和触摸的板级电源、复位处理；
- `bsp_power.c`：板级电源恢复和部分充电器初始化；
- `board.h/bsp_board.h`：板级声明；
- `Kconfig.board`：板级默认配置。

它不是另一块名为“ULP Base”的开发板，也不是构建时应传入的板型参数，而是被具体板型复用的公共代码。

### 2.3 为什么构建参数不能写 `_hcpu`

正确板型参数是：

```text
sf32lb52-lchspi-ulp
```

构建系统根据该板型默认构建HCPU应用，并自动生成输出目录：

```text
build_sf32lb52-lchspi-ulp_hcpu
```

因此：

- `sf32lb52-lchspi-ulp` 是构建系统认识的板型名称；
- `_hcpu` 是构建系统追加到输出目录的核心标识。

把 `_hcpu` 擅自加入 `--board` 参数，会要求构建系统寻找一个并不存在或未注册的板型名称。

## 3. 正确的环境、清理、编译和烧录流程

## 3.1 前提：必须使用已加载SiFli环境的终端

普通PowerShell中可能找不到 `scons`、Python、GCC和 `sftool`。请使用你已经验证能够成功构建的SiFli专用终端或插件环境。

构建前至少确认：

```text
SIFLI_SDK 指向 D:\Embedded_App\SF32LB52\SiFli_SDK\sifli-sdk
scons 可执行
Python 可执行
arm-none-eabi-gcc 可执行
sftool 在烧录时可执行
```

不要仅因为VS Code能跳转代码，就认为构建环境已经加载。

## 3.2 进入构建目录

所有以下命令都从项目的 `project` 目录执行：

```powershell
cd D:\Embedded_App\SF32LB52\SiFil_Project\SmartWatch_Project\project
```

## 3.3 清理当前板型构建产物

```powershell
scons --board=sf32lb52-lchspi-ulp -c
```

作用：让SCons清理当前板型的生成目标，用于排除旧对象文件和旧配置缓存。

注意：

- `-c` 是clean参数；
- 清理前应保存重要构建日志；
- 不要把Git清理与SCons清理混为一谈；
- 不要手工删除来源不明的目录；
- 旧的 `build_sf32lb52-lcd_n16r8_hcpu` 是历史板型产物，确认无用前先保留。

## 3.4 编译当前工程

```powershell
scons --board=sf32lb52-lchspi-ulp -j8
```

作用：选择 `sf32lb52-lchspi-ulp` BSP并使用最多8个并行任务构建。

成功后主要输出位于：

```text
project/build_sf32lb52-lchspi-ulp_hcpu/
```

关键产物：

| 产物 | 作用 |
|---|---|
| `main.elf` | 带符号的HCPU应用，可用于调试和分析 |
| `main.bin` | HCPU应用二进制，烧录到Flash |
| `main.hex` | 带地址信息的Hex格式应用镜像 |
| `main.map` | 链接映射，用于查看符号和内存占用 |
| `bootloader/bootloader.bin` | 启动加载程序镜像 |
| `ftab/ftab.bin` | Flash分区/镜像表相关数据 |
| `compile_commands.json` | 真实编译命令数据库，供IDE跳转和分析
| `uart_download.bat` | UART烧录脚本 |
| `download.bat` | J-Link烧录入口 |

`-j8`只影响并行度，不改变功能。如果电脑资源有限可降低并行数。

## 3.5 UART下载/烧录

从 `project` 目录执行：

```powershell
.\build_sf32lb52-lchspi-ulp_hcpu\uart_download.bat
```

脚本会提示输入串口数字。例如设备是COM12时，输入：

```text
12
```

不要输入 `COM12`，因为脚本会自动拼接 `COM`。

当前生成脚本使用 `sftool`，写入：

```text
ftab/ftab.bin             → 0x12000000
bootloader/bootloader.bin → 0x12010000
main.bin                  → 0x12020000
```

“下载”和“烧录”在这里指同一过程：把生成镜像写入开发板Flash。它不是把源码复制到开发板。

烧录前关闭占用同一COM口的串口终端；烧录完成后再重新打开串口查看日志。

## 3.6 J-Link下载（当前不是阶段0主路径）

如果已正确安装并连接J-Link，可使用：

```powershell
.\build_sf32lb52-lchspi-ulp_hcpu\download.bat
```

该脚本通过 `jlink.exe` 和 `download.jlink` 写入相同三类镜像。阶段0优先使用已经验证的UART下载链路，不同时调试UART和J-Link两条链路。

## 3.7 推荐的一次完整操作顺序

```text
打开SiFli开发终端
  → 确认SIFLI_SDK和工具可见
  → cd到SmartWatch_Project/project
  → 可选：清理当前板型
  → 使用sf32lb52-lchspi-ulp构建
  → 检查main.bin/ELF/MAP和下载脚本
  → 关闭占用COM口的串口工具
  → 运行uart_download.bat并输入串口数字
  → 烧录完成后重新打开串口
  → 复位开发板并保存完整启动日志
```

## 4. 项目目录与文件职责

```text
SmartWatch_Project/
├─ .vscode/
│  ├─ tasks.json                 # VS Code构建和UART下载任务
│  ├─ c_cpp_properties.json      # IDE补全配置，不是唯一构建事实
│  └─ launch.json                # OpenOCD/DAPLink调试配置
├─ project/
│  ├─ SConstruct                 # SCons顶层入口，组织Bootloader、应用、FTAB和下载脚本
│  ├─ SConscript                 # 把官方SDK与项目src加入同一次构建
│  ├─ Kconfig/Kconfig.proj       # 项目Kconfig定义和入口
│  ├─ proj.conf                  # 项目默认功能配置
│  ├─ rtconfig_project.h         # 少量项目级编译配置
│  └─ build_*/                   # 生成产物，不是源码
├─ src/
│  ├─ SConscript                 # 登记项目各源码子目录和公共头文件路径
│  ├─ main/
│  │  ├─ SConscript             # 把main.c加入构建
│  │  └─ main.c                 # RT-Thread应用main线程入口；当前为LCD白屏冒烟测试
│  ├─ hal_port/                  # 未来的项目级硬件适配封装
│  ├─ modules/                   # 未来的传感器、BLE、存储、电源等服务/模块
│  │  └─ mem/                    # 已有图形缓存和SRAM/PSRAM内存辅助代码
│  ├─ gui/                       # 未来的LVGL页面和资源
│  └─ apps/                      # 未来的手表应用与业务控制
├─ PROJECT_ARCHITECTURE.md       # 本说明文档
└─ SmartWatch.code-workspace     # 同时打开项目与SDK的VS Code工作区
```

## 4.1 `.vscode/tasks.json`

用于给VS Code提供快捷任务。目前主要是：

- 在 `project` 目录运行SCons构建；
- 执行生成的UART下载脚本。

它只是把终端命令包装成按钮。真正的构建规则仍来自SConstruct/SConscript和SDK构建系统。

## 4.2 `project/SConstruct`

它是一次构建的总导演，主要按顺序做：

1. 读取并检查 `SIFLI_SDK`；
2. 加载SiFli构建工具；
3. 准备编译环境；
4. 加入Bootloader构建；
5. 编译SDK和项目应用；
6. 加入FTAB构建；
7. 生成UART/J-Link下载脚本。

现阶段不需要掌握每个Python函数的内部实现，只需知道输入、输出和调用顺序。

## 4.3 `project/proj.conf`

它是项目的默认功能选择，例如：

- RT-Thread主线程、finsh、ulog；
- LVGL与SiFli适配；
- EPIC新API、eZip；
- FatFS；
- PSRAM和图像缓存；
- GUI应用框架、Button库、CPU usage profiler；
- 字体和资源配置。

它不是最终生成头文件。构建系统会把项目配置、板型配置和各组件Kconfig合并，生成build目录中的 `.config` 和 `rtconfig.h`。

不要直接编辑生成的 `rtconfig.h`，下次构建可能覆盖它。

## 4.4 `src/SConscript`

它不是普通的文件夹管理器，而是项目源码的构建登记表。它负责：

- 调用各子目录的SConscript；
- 收集需要编译的源码；
- 提供头文件搜索路径；
- 根据配置决定某些源码组是否加入。

将 `.c` 文件放进目录并不一定会自动编译；必须确认相应SConscript能够找到并返回它。

## 4.5 `src/main/main.c`

它是RT-Thread启动完成后创建的应用main线程入口，不是芯片复位后执行的第一段代码。

当前程序用于LCD冒烟测试：

1. 打印Hello World；
2. 查找并打开名为 `lcd` 的RT-Thread设备；
3. 设置亮度和RGB565格式；
4. 使用50行缓冲分块将390×450 AMOLED刷白；
5. 进入带延时的循环。

它证明显示设备链基本可用，但不是最终手表架构。后续不应把UI、传感器和BLE全部继续堆进该文件。

## 5. SCons构建链

```text
project/SConstruct
  ├─ AddBootLoader(...)                    → Bootloader
  ├─ PrepareBuilding(...)
  │    └─ project/SConscript
  │         ├─ $SIFLI_SDK/SConscript       → SDK、RT-Thread、BSP、中间件
  │         └─ src/SConscript              → 项目源码
  │              ├─ main/SConscript
  │              ├─ modules/SConscript
  │              ├─ modules/mem/SConscript
  │              ├─ gui/SConscript
  │              ├─ hal_port/SConscript
  │              ├─ apps/SConscript
  │              └─ 字体和语言资源SConscript
  ├─ DoBuilding(...)                       → main.elf/main.bin等
  ├─ AddFTAB(...)                          → ftab.bin
  └─ GenDownloadScript(...)                → UART/J-Link下载脚本
```

## 6. SDK板级路径

当前板型重点路径：

```text
sifli-sdk/customer/boards/sf32lb52-lchspi-ulp/
sifli-sdk/customer/boards/sf32lb52-lchspi-ulp_base/
sifli-sdk/customer/peripherals/co5300/
sifli-sdk/customer/peripherals/ft6146/
```

当前阶段只读SDK。如果确实发现SDK缺陷，先在项目层寻找配置或覆盖方式，再单独记录补丁理由；不要直接边学边改SDK。

## 7. 当前源码状态

| 目录 | 当前状态 | 阶段0处理方式 |
|---|---|---|
| `src/main` | 已有LCD白屏冒烟测试 | 理解并作为运行基线，不继续堆功能 |
| `src/hal_port` | 只有构建骨架 | 阶段1/2再设计封装 |
| `src/modules` | 大部分为空 | 传感器、BLE、存储阶段逐步建立 |
| `src/modules/mem` | 已有复杂图形内存辅助代码 | 阶段0只读公开接口，不深入修改 |
| `src/gui` | 无实际页面，已有大量资源 | 阶段2建立最小页面，不先加载全部资源 |
| `src/apps` | 只有构建骨架 | 阶段4建立业务应用 |

## 8. 常见错误排查顺序

### 8.1 找不到 `scons`

先确认是否从正确的SiFli环境入口打开终端，再检查工具路径。不要先改项目源码。

### 8.2 提示没有 `SIFLI_SDK`

说明构建终端未加载SiFli SDK环境，或环境指向错误目录。

### 8.3 `board not found`

确认参数使用：

```text
sf32lb52-lchspi-ulp
```

不要写成：

```text
sf32lb52-lchspi-ulp_hcpu
```

### 8.4 增量构建成功、完整构建失败

说明旧缓存可能掩盖环境或源码问题。保存第一处error，优先解决完整构建。

### 8.5 UART下载找不到COM口

检查设备管理器、USB线、串口驱动、端口是否被占用；输入脚本时只输入数字。

### 8.6 烧录成功但没有串口日志

依次检查：串口号、波特率、串口工具是否在烧录后重新打开、板卡是否复位、是否使用了当前build目录。

### 8.7 有日志但屏幕未变白

根据日志区分 `lcd` 查找失败、open失败、亮度/格式失败或绘制异常。先保存证据，不立即开始写LVGL。

## 9. 工程纪律

1. SDK目录默认只读。
2. 构建和烧录都要记录SDK、板型、命令和输出目录。
3. 不提交 `build_*`、`.elf`、`.bin`、`.o` 等生成文件。
4. 不直接编辑build目录中的生成配置。
5. 新源码必须确认SConscript已纳入构建。
6. 一次只改变一个关键变量。
7. 所有性能和功耗数字必须来自实测。
8. 阶段0先建立可信基线，再进入手表功能开发。

