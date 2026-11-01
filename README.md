# Dust Linux Architecture

Dust Linux Architecture 是一个面向 Linux 的 C++17 机器人控制骨架。它提供组件初始化与线程生命周期管理、进程内消息通道、Kconfig 构建期功能选择、每线程 YAML 配置、运行时调试 shell，以及一层封装总线通讯的驱动层（当前落地 EtherCAT 主站）。`temp/` 下是参考工程与资料，不参与根 CMake 构建。

骨架本身不含业务：`project/thread/` 里只有 `test`（自检）与 `ethercat`（总线周期线程）两个验证载体。业务工程在各自的工作区里组合这些能力。

## 架构分层

```text
Kconfig                    构建期功能选择的描述文件（根 + 各层）
scripts/kconfig.py         Kconfig 前端：求值并生成 .config / kconfig.cmake / autoconf.h
framework/                 可复用的基础设施与能力
├── init/                  组件注册、启动阶段、线程生命周期与关闭
├── shell/                 调试台：命令分派、日志、运行时变量
├── cmd/                   构建脚本、YAML 配置读取、链接脚本
├── topic/                 线程间消息通道
├── modules/               可复用设备模块（骨架阶段为空装配位）
├── drivers/               总线与硬件驱动
│   └── communication/
│       └── ethercat/      EtherCAT 主站（SOEM 封装）
└── algorithm/             与线程、设备 I/O 解耦的算法
    ├── filter/            滤波估计（Kalman / EKF、四元数姿态）
    └── math/              基础数学工具
project/                   本工程的业务装配
├── Kconfig                业务门禁与使用开关
├── thread/                业务线程（test / ethercat）
└── CMakeLists.txt         业务工程装配入口
```

### 各层的含义

| 层 | 负责什么 | 不应承担什么 |
| --- | --- | --- |
| `framework/algorithm/` | 可测试、可复用的纯计算逻辑；通过输入/输出对象表达结果 | 不碰硬件、不管理线程、不做 I/O |
| `framework/modules/` | 封装一项完整的设备能力（设备句柄、协议状态机） | 不编排完整应用流程 |
| `framework/drivers/` | 靠近硬件/总线的适配：打开设备、收发帧、维护链路状态 | 不放业务语义，不认识具体从站的数据含义 |
| `framework/topic/` | 定义跨线程消息与传递机制，隔离生产者与消费者 | 不承载隐式全局可变业务状态 |
| `project/thread/` | 把驱动、算法、配置和 topic 组合成具体业务流程 | 不重复实现驱动或算法内部细节 |
| `framework/shell/` | 命令分派表、日志系统、运行时变量 | 不耦合某个具体驱动的内部实现 |
| `framework/cmd/` | 构建脚本、YAML 读取、链接脚本 | 不放调试台能力（归 shell） |
| `framework/init/` | 按阶段统一初始化、启动线程、停止、join 和清理 | 不放业务代码 |

### 大分层：每一块为什么独立存在

目录不是按“文件类型”分类，而是把系统拆成**业务装配、可复用能力、数据交换、系统治理、构建期选择**五类职责。新增代码时按这个判断该落在哪一层。

#### `project/`：应用层，回答“这个工程当前要怎么工作”

`project/thread/` 把驱动、算法、YAML 参数和 topic 接起来。它是胶水层与策略编排层，不是通用库。业务流程变化时（增加设备、改变时序、换总线策略），改的是线程层，驱动层不动。

#### `framework/`：基础能力层，回答“有哪些可复用的积木”

- `framework/algorithm/`：纯计算。当前包含 `filter/`（Kalman、EKF、四元数姿态）和 `math/`（角度工具）。
- `framework/modules/`：面向业务的设备能力，封装一项完整能力的实现细节。骨架阶段为空装配位。
- `framework/drivers/`：总线与硬件适配。当前落地 EtherCAT 主站，见 [framework/drivers/README.md](framework/drivers/README.md)。
- `framework/topic/`：进程内数据交换。定义“谁发布什么、消费者如何独立读取、关闭时如何唤醒”。
- `framework/shell/`：调试台。命令族注册、日志分频、运行时变量。
- `framework/cmd/`：构建脚本（`build/`）、YAML 读取（`config/`）、链接脚本（`linker/`）。
- `framework/init/`：应用级生命周期协调器。收集组件声明的初始化/线程回调，统一安排阶段、优先级、停止、join 和逆序清理。

几种容易混淆的角色：**driver 接近设备协议，module 提供业务可用的设备能力，thread 组织运行流程，algorithm 负责计算，topic 传递数据，init 管理生命周期，shell 提供运维入口。**

#### `Kconfig`：构建期选择，回答“这次构建要哪些代码”

功能开关写在 Kconfig 文件里：根 `Kconfig` 引入各层描述文件，`scripts/kconfig.py` 求值得到确定结果，CMake 只消费结果。

#### `framework/init/`：不是“初始化函数集合”，而是生命周期架构

每个组件声明资源初始化和清理回调，线程声明启动、停止请求与 join 回调；启动器按 `Pre/Early/Mid/Late/App` 阶段推进，阶段内按优先级排序。退出时逆序发停止、逆序等待线程、再逆序清理资源。

这解决了组件增多后的常见隐患：初始化顺序散落、失败只清理一半、线程还在访问对象时资源已释放、关机时消费者永久卡住。

#### `framework/drivers/`：不认识业务语义

驱动只做“打开网口、枚举从站、按 IOmap 收发帧”，**不定义具体从站的 PDO 结构**。过程数据的类型由使用方通过模板参数传入（`Ecat<Out, In>`），驱动只负责 `memcpy` 与长度校验。换从站只改使用方的结构体定义，驱动不动。

#### `framework/topic/`：显式的数据边界，而非随手共享的全局变量

消息结构定义边界字段，Channel 定义发布/订阅和关闭语义，每个消费者持自己的 Subscriber。增加消费者不需要借用别人的游标，也不需要生产者直接调用所有消费者。

### 依赖方向与运行时组合

```text
                    ┌──────────── framework/init ────────────┐
                    │ 统一调度组件创建、线程启动与退出清理     │
                    └─────────────────────────────────────────┘

Kconfig ──求值──> .config ──生成──> kconfig.cmake / autoconf.h ──> CMake 目标与编译宏

project/thread ──调用──> framework/drivers + framework/modules + framework/algorithm
      │                              │
      └──────────发布/订阅──────────> framework/topic
```

依赖关系的目标是：应用层可以组合基础能力，但基础能力不认识应用层。构建期同理：业务开关 select 架构能力，架构层不认识具体业务。

### 重要目录与入口文件

| 文件/目录 | 在当前工程中的位置 |
| --- | --- |
| `Kconfig` | 构建期功能选择入口，`source` 各层 Kconfig 与 `project/Kconfig` |
| `scripts/kconfig.py` | Kconfig 前端：解析、求值，生成 `.config`、`kconfig.cmake`、`autoconf.h` |
| `framework/algorithm/Kconfig` | 算法能力符号（`DUST_FLT_*`、`DUST_MATH_ANGLE`） |
| `framework/shell/Kconfig` | 调试台能力符号（`DUST_CMD_SHELL` / `_LOG` / `_VAR`） |
| `framework/cmd/Kconfig` | YAML 配置读取能力符号（`DUST_CMD_CONFIG`） |
| `framework/drivers/Kconfig` | 驱动能力符号（`DUST_COM_ETHERCAT`） |
| `framework/topic/Kconfig` | 话题通道能力符号 |
| `project/Kconfig` | 业务门禁与使用开关（`USE_*` / `TRD_*`），靠 `select` 拉起架构能力 |
| `framework/init/startup/main.cpp` | 进程入口，只调用统一启动、等待和关闭流程 |
| `framework/init/thread/thread.hpp` | 线程外壳：受管任务、周期调度、统计、暂停/恢复、故障处置 |
| `framework/init/thread/thread_fault.hpp` | 故障策略、Retry/Exit 钩子、默认处置 |
| `framework/init/startup/Init_entry.hpp` | `REGISTER_INIT` / `REGISTER_THREAD` 与阶段枚举 |
| `framework/shell/shell_cmd.hpp` | `REGISTER_SHELL_CMD` 命令族注册 |
| `framework/shell/token.hpp` | 行切分与子命令查表分派 |
| `framework/drivers/communication/ethercat/ecat.hpp` | EtherCAT 主站驱动（SOEM） |
| `project/thread/test/trd_test.cpp` | 自检线程：验证线程模板、日志与注册链路 |
| `project/thread/ethercat/trd_ethercat.cpp` | EtherCAT 周期线程 |
| `project/CMakeLists.txt` | 创建最终 `dust_app`，链接框架库并配置链接脚本 |
| `framework/CMakeLists.txt` | 按 `CONFIG_*` 装配各层库 |
| `framework/cmd/build/` | 构建与运行脚本（`dust`、`build.sh`、`run.sh`） |

## 构建期功能选择（Kconfig 链路）

### 文件与分层

| 文件 | 符号 | 谁改它 |
| --- | --- | --- |
| `Kconfig` | `PRJ_MAIN` 总门禁 | 维护者 |
| `framework/algorithm/Kconfig` | `DUST_FLT_KALMAN`、`DUST_FLT_KALMAN_EKF`、`DUST_FLT_QUATERNION`、`DUST_MATH_ANGLE` | 架构层 |
| `framework/shell/Kconfig` | `DUST_CMD_SHELL`、`DUST_CMD_SHELL_LOG`、`DUST_CMD_SHELL_VAR` | 架构层 |
| `framework/cmd/Kconfig` | `DUST_CMD_CONFIG` | 架构层 |
| `framework/drivers/Kconfig` | `DUST_COM_ETHERCAT` | 架构层 |
| `framework/topic/Kconfig` | 话题通道能力符号 | 架构层 |
| `project/Kconfig` | `USE_CMD_SHELL`、`USE_CMD_CONFIG`、`TRD_TEST`、`TRD_ETHERCAT` | 业务层 |

规则：

- **架构层 `DUST_*` 默认 `n`**：不显式启用就不进镜像，也不去找它的外部依赖。
- **业务层用 `select` 拉起架构能力**，不在别处硬写 `CONFIG_DUST_*=y`；业务线程自己的开关用 `TRD_*`，通用使用开关用 `USE_*`。
- 依赖写在 Kconfig 里，依赖不满足时符号被置 `n`，对应源文件不会编进程序。
- 业务符号保持原名、不带 `DUST_` 前缀；被 select 的架构符号统一 `DUST_` 前缀。

### 求值与生成

配置阶段由根 CMake 调用 `scripts/kconfig.py`（纯 Python 标准库，不需要 kconfiglib）：解析 Kconfig 树 → 按 `select` / `depends on` / `default` 求值 → 生成

```text
project/build/kconfig/.config         规范化结果（含 "# CONFIG_X is not set"）
project/build/kconfig/kconfig.cmake   set(CONFIG_X ON/OFF)，供 CMake 消费
project/build/kconfig/autoconf.h      #define CONFIG_X 1，强制 include 给所有编译单元
```

CMake 侧只用结果：`include(kconfig.cmake)` 得到 `CONFIG_*`，`-include autoconf.h` 让 C++ 侧看见同一个符号。

### 用哪份配置构建

默认直接用 Kconfig 文件里的 `default`。要另一套组合时，给它一份已存在的 `.config`：

```bash
./framework/cmd/build/build.sh --config /absolute/path/to/.config
```

### 改动开关时的落点

| 你要做的事情 | 改哪里 |
| --- | --- |
| 新增一个架构能力 | 对应层的 `Kconfig` 加 `DUST_*`（`default n`），并在该层 CMake 里按它裁源文件 |
| 新增一个业务线程 | `project/Kconfig` 加 `TRD_*`，`project/thread/CMakeLists.txt` 加条件源文件 |
| 新增一个通用使用开关 | `project/Kconfig` 的“使用开关”分区加 `USE_*` |
| 关闭某项功能 | 改 Kconfig 里的 `default`，或给它一份 `.config` |

## 启动与线程生命周期

`framework/init/startup/main.cpp` 只调用统一启动、等待和关闭入口。业务组件在自己的源文件中通过 `REGISTER_INIT`、`REGISTER_THREAD` 注册回调，链接脚本保留注册段；启动框架按阶段执行注册项。

- `REGISTER_INIT`：初始化资源，并提供可处理部分初始化失败的清理回调。
- `REGISTER_THREAD`：启动回调应快速创建线程并返回；停止回调负责发出停止请求、解除阻塞；join 回调等待线程退出。
- 停止顺序：请求停止 → 等待线程退出 → 释放资源。不要在线程仍运行时释放它正在使用的设备、通道或缓冲区。

阶段顺序是 `PreInit → EarlyInit → MidInit → LateInit → AppInit`，每个阶段完成初始化后再启动对应阶段的线程。初始化阶段内按 `High → Mid → Low` 排序；初始化项可以选择失败后中止启动或继续。

### 线程外壳

`init::Thread` 把“一拍做什么”和“什么时候跑”分开：

- `Start(entry, period, context, name, fault)`：`period` 是两拍之间的间隔，任务体只管写一拍，**不自己 sleep、不做周期计算**。
- 任务返回 `TaskResult::Continue`（下一拍继续）、`Retry`（交给 `RetryHook` 处置）、`Exit`（跳过钩子直接按策略处置）。
- **正常路径只写在任务体里**；建链、重连、计数、退避、节流日志全部放 `RetryHook`。
- `ThreadFaultConfig` 携带 `RetryHook` / `ExitHook` / `ThreadFaultPolicy` / `keep_alive`，在 `Start()` 调用点一次写死。

### 故障处置

| 策略 | 适用 | 行为 |
| --- | --- | --- |
| `Isolate` | 影响小（自检、心跳） | 收尾后只退出自己 |
| `Recover` | 影响大（总线、姿态） | 救不回来时停掉全部业务线程，保留 shell |

## 如何在当前架构中开发

### 新增一个业务功能

1. **先定义数据边界。** 确认输入、输出、时间戳、线程所有权和失败状态；跨线程数据通过 `framework/topic/` 中明确的消息传递，不用裸全局变量。
2. **把计算放进算法层。** 在 `framework/algorithm/` 中实现不依赖线程和硬件 I/O 的逻辑。
3. **把硬件操作留在驱动/模块层。** 设备打开、协议解析、链路重连细节不要散落到业务算法中。
4. **在线程层编排。** 在 `project/thread/` 读取本线程配置、调用驱动/模块/算法；始终定义停止和阻塞唤醒行为。
5. **注册生命周期。** 新增初始化或线程时使用现有注册机制，并在相应 CMake `target_sources` 中加入源文件。注册源必须直接进入最终可执行程序，否则会被链接器裁掉。
6. **补构建期开关。** 架构能力写进所属层 `Kconfig`，业务线程写进 `project/Kconfig`。
7. **配置与验证。** 线程 YAML 放在所属线程目录，程序以当前工作目录为基准读取；配置必须校验必需字段、类型和值域。

### 按改动类型选择落点

| 你要做的事情 | 优先修改的位置 | 典型例子 |
| --- | --- | --- |
| 改进纯算法 | `framework/algorithm/` | 滤波参数、姿态解算 |
| 增加或调整总线驱动 | `framework/drivers/communication/` | 新总线、新协议 |
| 增加设备能力 | `framework/modules/` | 电机、IMU、遥控器封装 |
| 增加线程间数据 | `framework/topic/<name>/` | 观测结果或控制反馈消息 |
| 改变业务处理顺序 | `project/thread/` | 接入新设备、改变时序 |
| 增加可调参数 | 对应业务线程 YAML | 网口、周期、超时 |
| 增加运行时命令 | `framework/shell/` | 与具体设备无关的调试命令 |
| 改变启动/停止行为 | `framework/init/` 与注册调用处 | 新的初始化阶段或生命周期动作 |
| 增删一个功能开关 | 对应层 `Kconfig`（+ 该层 CMake 条件） | 新线程、新能力、新驱动 |

判断方式：**固定输入能离线验证 → 算法层；调用系统/总线接口 → 驱动层；决定“何时调用谁、数据发给谁” → 线程层。**

### YAML 配置接入流程

1. 把配置放到拥有该功能的线程目录，如 `project/thread/ethercat/trd_ethercat.yaml`。
2. 程序按“`DUST_CONFIG_ROOT` 指定的配置根，未设置时当前工作目录”解析相对路径；从仓库根运行即可找到。
3. 初始化时使用 `cmd::config::Config::Load(path, {必需字段...})`；读取 API 对类型和值域做验证。
4. 将验证后的值拷贝到业务配置结构，再创建驱动/模块。不要在热路径反复读 YAML。
5. 配置注释写明单位、边界含义、是否重启生效。
6. 编译期开关只决定“有没有这段代码”，运行期 YAML 只决定“这次用什么参数”。

### 调试与定位建议

1. 先看配置阶段输出：Kconfig 求值结果与最终 `CONFIG_*`。
2. 再确认线程是否启动、配置是否加载、资源是否初始化成功（`thread list` / `thread show <name>`）。
3. 驱动问题先拿独立工具复现（如 `slaveinfo`），区分“链路不通”和“代码问题”。
4. 多线程问题检查状态归属、锁保护范围、阻塞等待是否可被关闭打断、停止/join/释放顺序。
5. 改动后至少执行 `git diff --check`。

## C++ 代码风格

本项目使用 C++17，并以现有代码为准保持一致：

- 缩进 4 个空格，不使用 Tab；控制块和函数定义使用 Allman 风格大括号。
- 类型、函数/方法使用 `PascalCase`；成员变量后缀 `_`；常量 `k` 前缀；枚举用 `enum class`。
- 整数类型统一用定宽类型（`uint32_t` / `uint16_t` / `uint8_t` / `int32_t`），不混用裸 `int`；只有第三方签名固定为 `int` 时保留并 `static_cast` 收口。
- 优先使用明确的类型、`const`、引用和 RAII；资源获取/释放与线程停止/join 成对。
- 头文件使用 `#pragma once`；新增 `.cpp` 必须接入 CMake（本项目不扫描目录）。
- 类成员按职责拆成子结构体分组（如 `link_` / `pdo_` / `stats_`），不用注释标签分组。
- 不为了“通用化”引入尚无第二个调用场景的抽象；不顺手重命名或整理不相关代码。

## 注释与 Doxygen 风格

- 对外可见的类/接口和关键实现使用多行 Doxygen `/** ... */`，用 `@brief` 描述职责；参数、返回值、异常确实有帮助时才写。
- **禁止把 Doxygen 压缩成单行** `/** @brief ... */`。
- 函数实现的说明放在实现所在的 `.cpp`，紧挨实现；头文件只在声明附近保留必需的简短契约。
- 成员、函数原型、函数内部逻辑用 `//`。**分组标签（`// --- xxx ---`）一律不用。**
- 注释解释“为什么这样做”、单位、线程约束、失败和回退语义；不要逐行翻译代码。
- 短函数（一行 getter/setter）、构造函数、析构函数不写注释。
- 改动文件时同步更新文件头的 `@version` 和 `@date`。
- 日志一律用 `DUST_LOG_*` 宏，不用 `printf` / `std::cout`。失败分支必须有日志，且带上实际值与期望值。

示例：

```cpp
/**
 * @brief 把输出镜像写进 IOmap，并发送一帧过程数据
 *
 * @return true=帧已提交
 */
bool Send();
```

## 日志与调试台

`DUST_LOG_*`（`framework/shell/log/log.hpp`）：

| 宏 | 用途 |
| --- | --- |
| `DUST_LOG_INF/ERR/OK/WRN` | 一次性日志，调用即打印 |
| `DUST_LOG_DBG("名字", ...)` | 流式日志，**默认静默**，`log on <名字>` 后按分频打印 |

运行时命令：

```text
log list
log on <name>
log mode <name> on|off      # 纯文本（VOFA）与着色切换
var list
var get <name>
var set <name> <value>
thread list
thread show <name>
thread pause <name>
thread resume <name>
boot timing
```

## 配置、构建与运行

### 依赖

C++17、CMake 3.16+、Python 3（运行 Kconfig 前端）、pthread、Eigen3、yaml-cpp。启用 `DUST_COM_ETHERCAT` 时还需要系统安装的 SOEM（`/usr/local/lib/libsoem.a` + `/usr/local/include/soem/`）。

### 构建

```bash
./framework/cmd/build/dust build     # 配置 + 构建到 project/build
./framework/cmd/build/dust run       # 运行 project/build/project/dust_app

# 或者把脚本目录加进 PATH
export PATH="$PWD/framework/cmd/build:$PATH"
dust build
dust run
```

手工构建：

```bash
cmake -S . -B project/build
cmake --build project/build --parallel
```

### 运行

```bash
./project/build/project/dust_app
```

YAML 以当前工作目录为基准解析，**从仓库根运行**即可找到 `project/thread/<线程>/<文件>.yaml`。

### 常见构建/运行问题

| 现象 | 优先检查 |
| --- | --- |
| 配置阶段报 Kconfig 求值失败 | 报错会给出符号和出处；多半是依赖不满足却要求 `=y`，或符号名拼错 |
| `CONFIG_*` 没生效 | `project/build/kconfig/` 是否更新；`project/build/CMakeCache.txt` 里是否有旧的 `KCONFIG_CONFIG` |
| 报 `SOEM not found` | 系统是否装了 SOEM；或把 `CONFIG_DUST_COM_ETHERCAT` 关掉 |
| 报 `bad file ...trd_ethercat.yaml` | YAML 解析基准不对：确认在仓库根运行，或设 `DUST_CONFIG_ROOT` |
| 找不到网口 / 扫不到从站 | `ip -br link` 看哪个口有 `LOWER_UP`；用 `slaveinfo <口名>` 独立验证链路 |
| 从站进不了 OP | 看驱动日志里的 SM2/SM3 长度与 CoE PDO 是否一致、`alstatus` 码 |
| 新增源文件后符号未链接 | 检查对应 `target_sources()` 与该源文件所在功能的 `CONFIG_*` 是否为 `y` |
| 注册的初始化/线程没有执行 | 注册源是否进入最终可执行文件，链接脚本是否保留注册段 |
| Ctrl+C 后终端不恢复 | 检查 shell 的 `poll` 返回后是否判断了 `StopRequested()` |

## 文档索引

| 文档 | 内容 |
| --- | --- |
| [通讯驱动层与 EtherCAT 主站落地规划](doc/通讯驱动层与EtherCAT主站落地规划.md) | drivers 层骨架与 EtherCAT 落地步骤 |
| [EtherCAT 主站重写规划](doc/EtherCAT主站重写规划.md) | 主站实现的分层与重写过程 |
| [EtherCAT 首次进入 OP 缓慢调研](doc/EtherCAT首次进入OP缓慢调研.md) | 从站进入 OP 慢的原因排查 |
| [EtherCAT 从站当前状态-给主站](doc/EtherCAT从站当前状态-给主站.md) | 从站身份、PDO 布局与主站侧对接说明 |
| [子架构部署与父架构功能补齐规划](doc/子架构部署与父架构功能补齐规划.md) | 从视觉子架构到当前骨架的部署与补齐记录 |
| [驱动层说明](framework/drivers/README.md) | drivers 层边界、三类通讯形态与新增驱动四步 |
| [构建脚本说明](framework/cmd/build/README.md) | `dust` / `build.sh` / `run.sh` 参数 |
