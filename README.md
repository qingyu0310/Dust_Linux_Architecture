# Dust Vision

Dust Vision 是一个面向 Linux 的原生 C++17 机器人视觉程序，当前主要用于读取海康工业相机图像、通过 TensorRT YOLO 模型检测 RoboMaster 装甲板，并显示检测结果。工程同时提供轻量的组件初始化/线程生命周期管理、进程内消息通道、Kconfig 构建期功能选择、每线程 YAML 配置和运行时调试 shell。

当前运行主链是**单帧二维装甲板检测与显示**。PnP/距离估计、跨帧跟踪、目标锁定与切换、云台控制和自动开火尚未接入；`ArmorRefiner` 是编译期可开关的局部灯条角点精修，是否编进程序由 Kconfig 决定。`temp/` 下是参考工程与资料，不参与根 CMake 构建。

## 当前数据流

```text
海康 MVS 相机
  → project/thread/camera：采集并得到 BGR 帧
  → framework/topic/camera_to：发布最新帧
  → project/thread/vision：调用 ArmorDetector
  → framework/algorithm/vision：预处理、TensorRT 推理、后处理
  → framework/topic/vision_to：发布同帧图像、时间戳与装甲板结果
  → project/thread/display：主线程绘制并显示
```

`camera_to` 和 `vision_to` 使用单槽最新消息语义：每个订阅者有独立游标；慢消费者会跳过旧消息，不会让延迟持续堆积。图像通过 `cv::Mat` 浅拷贝共享像素；需要修改图像的消费者应先 `clone()`。识别结果与对应原图、采集时间戳一起传递，避免把检测框画到另一帧上。

## 架构分层

```text
Kconfig                    构建期功能选择的描述文件（根 + 各层）
scripts/kconfig.py         Kconfig 前端：求值并生成 .config / kconfig.cmake / autoconf.h
framework/                 可复用基础设施与算法
├── init/                  组件注册、启动阶段、线程生命周期和关闭
├── cmd/                   控制台线程、日志、运行时变量、YAML 配置读取
├── topic/                 线程间消息通道和消息类型
├── modules/               可复用设备/功能模块，如 HikRobot 相机
├── drivers/               更底层的设备和通信驱动（当前尚无源文件）
└── algorithm/             与线程、设备 I/O 解耦的算法
    ├── filter/            滤波估计（Kalman / EKF、四元数姿态）
    ├── math/              基础数学工具
    └── vision/
        ├── core/          视觉领域类型、结果契约和几何工具
        ├── detect/        图像检测器和模型输出解码
        └── refine/        检测结果的可选局部角点精修
project/                   本项目的业务装配
├── Kconfig                业务门禁与使用开关
├── thread/                相机、测试、识别、显示等业务线程（每个线程一份 YAML）
└── CMakeLists.txt         业务工程装配入口
```

### 各层的含义

| 层 | 负责什么 | 不应承担什么 |
| --- | --- | --- |
| `framework/algorithm/` | 可测试、可复用的计算逻辑；通过输入/输出对象表达结果 | 不直接读相机、不管理业务线程、不操作 GUI |
| `framework/modules/` | 封装可复用的设备或功能能力，例如相机生命周期和 SDK 调用 | 不编排完整应用流程 |
| `framework/drivers/` | 靠近硬件/通信协议的底层适配 | 不放目标选择等高层业务策略 |
| `framework/topic/` | 定义跨线程消息和传递机制，隔离生产者与消费者 | 不承载隐式全局可变业务状态 |
| `project/thread/` | 将模块、算法、配置和 topic 组合成具体业务流程 | 不重复实现算法内部细节或绕过 topic 共享状态 |
| `framework/cmd/` | 通用配置读取、日志、交互 shell 和运行时变量机制 | 不直接耦合某个算法的内部实现 |
| `framework/init/` | 按阶段统一初始化、启动线程、停止、join 和清理 | 不放相机检测等业务代码 |

### 大分层：每一块为什么独立存在

上面的目录不是简单按“文件类型”分类，而是把系统拆成**业务装配、可复用能力、数据交换、系统治理、构建期选择**五类职责。理解这几类，新增代码时就知道该落在哪一层：

#### `project/`：应用层，回答“这个机器人当前要怎么工作”

`project/` 放本项目的具体业务组合。当前 `project/thread/` 把相机模块、视觉算法、YAML 参数和 topic 接起来：相机线程采图并发布，视觉线程订阅并检测，显示线程消费检测结果。它是应用的“胶水层”和策略编排层，不是通用算法库。

当业务流程变化时，例如增加跟踪、切换目标，通常由应用线程决定何时调用对应模块、结果发往哪里；跟踪算法本身不应该直接塞到相机 SDK 或 detector 内部。这样同一个算法可以被离线工具、仿真入口或其他业务流程复用。

#### `framework/`：基础能力层，回答“项目有哪些可复用的积木”

`framework/` 是多个业务组件可共享的基础设施和算法。它内部不是单一层，而是各自有边界的子系统：

- `framework/algorithm/`：数学、滤波和视觉算法。它应尽量接收明确输入并返回明确结果，不依赖当前业务线程、设备句柄或 GUI。当前包含 `filter/`（Kalman、EKF、四元数姿态）、`math/`（角度工具）和 `vision/`（检测与精修）。
- `framework/modules/`：面向业务使用的功能模块，封装一项相对完整的可复用能力。当前已落地的是 `modules/camera/hikrobot.*`，它把 MVS SDK 的设备探测、初始化、取帧和释放收进相机模块；业务层不需要在多个线程里重复处理 SDK 细节。
- `framework/drivers/`：预留给更底层、偏协议或硬件访问的驱动适配。当前该目录尚无实际源文件，因此 README 描述的是架构边界，不表示已经实现了某种串口/总线驱动。
- `framework/topic/`：进程内的数据交换层。它定义“谁发布什么消息、消费者如何独立读取、关闭时如何唤醒”，使生产者不需要知道所有消费者的存在。
- `framework/cmd/`：调试和配置入口，包括 YAML 读取、日志系统、交互 shell 和运行时变量。它提供通用机制；具体业务参数由对应线程读取，设备变更仍交回设备所属线程处理。
- `framework/init/`：应用级生命周期协调器。它不执行业务算法，而是收集组件声明的初始化/线程回调，统一安排阶段、优先级、停止、join 和逆序清理。

可以把几种常被混淆的角色这样区分：**driver 接近设备协议，module 提供业务可用的设备能力，thread 组织运行流程，algorithm 负责计算，topic 传递数据，init 管理生命周期，cmd 提供运维/调试入口。** 例如 `HikRobot` 是 module；运行它的相机线程是 project；二者之间的帧通过 topic 交接；相机线程何时创建和销毁由 init 统一调度。

#### `Kconfig`：构建期选择，回答“这次构建要哪些代码”

功能开关不写进 CMake option，而是写在 Kconfig 文件里：根 `Kconfig` 引入各层描述文件，`scripts/kconfig.py` 求值得到确定结果，CMake 只消费结果。分层规则见[构建期功能选择](#构建期功能选择kconfig-链路)。

#### `framework/init/`：不是“初始化函数集合”，而是生命周期架构

`init` 的价值不只是少写几行 `main()`：每个组件声明资源初始化和清理回调，线程声明启动、停止请求与 join 回调；启动器按照 `Pre/Early/Mid/Late/App` 阶段推进，并在阶段内按优先级排序。退出则逆序发停止、逆序等待线程、再逆序清理资源。

这解决了组件逐渐增多后常见的隐患：初始化顺序散落在多个文件、失败后只清理一半、线程还在访问对象时资源已释放、关机时消费者永久卡在等待。`main()` 因此保持稳定，业务增长主要体现在新增组件和注册声明上，而不是不断膨胀主函数。

#### `framework/modules/`：不是线程，也不是算法

module 持有并封装一个可复用能力的实现细节，例如设备句柄和 SDK 调用；它通常不决定进程里何时启动，也不应该负责把完整业务数据流编排起来。业务线程拥有 module 的生命周期，并在需要时调用它。

以相机为例：`HikRobot` 负责打开设备、配置采集、取得/转换图像和关闭；`trd_camera.cpp` 负责根据 YAML 初始化它、循环取帧、处理断线重连、发布 `camera_to` 并响应停止。将二者分开，未来换相机时主要替换 module/适配层，而不是重写 YOLO 检测器；未来换检测模型时也不需要改 MVS 设备封装。

#### `framework/algorithm/`：计算核心与运行时隔离

算法库应尽量能通过一组输入、输出、配置独立理解。以视觉为例，`ArmorDetector::Detect(image)` 不创建窗口、不订阅 topic、不知道相机线程在哪里；它只执行模型相关计算并返回结构化结果。线程负责把它放进实时数据流。

其它算法目录也遵循相同目标：`math/` 是基础数学工具，`filter/` 处理滤波估计。把它们与线程隔开后，可以单独调整算法或建立离线验证入口；算法不必依赖某一个具体设备和进程启动方式。

#### `framework/topic/`：显式的数据边界，而非随手共享的全局变量

topic 的核心价值是契约与所有权：消息结构定义边界字段，Channel 定义发布/订阅和关闭语义，每个消费者持自己的 Subscriber。由此识别和显示可以独立消费相机数据；增加消费者不需要把另一个消费者的 Subscriber 借过来，也不需要让生产者直接调用所有消费者。

当前通道针对图像实时性选择最新值语义，不是万能消息队列。是否允许跳帧、是否必须逐条处理、生产者和消费者谁拥有数据，都由通道类型与消息约定明确表达，而不是隐藏在偶然的线程调度里。

#### 依赖方向与运行时组合

```text
                    ┌──────────── framework/init ────────────┐
                    │ 统一调度组件创建、线程启动与退出清理     │
                    └─────────────────────────────────────────┘

Kconfig ──求值──> .config ──生成──> kconfig.cmake / autoconf.h ──> CMake 目标与编译宏

project/thread ──调用──> framework/modules + framework/algorithm
      │                              │
      └──────────发布/订阅──────────> framework/topic

framework/cmd 提供通用 YAML、日志、shell、运行时变量能力
modules 可使用 drivers；算法与业务线程不反向依赖 project
```

依赖关系的目标是：应用层可以组合基础能力，但基础能力不认识应用层。构建期同理：业务开关 select 架构能力，架构层不认识具体业务。

`core/`、`detect/`、`refine/` 是视觉算法内部的职责分层：`core` 定义共同语言，`detect` 从图像产生候选，`refine` 在候选基础上做可选修正。当前 `DetectedArmor` 表示一帧中的二维检测结果，不是跟踪目标或三维位姿。

### 重要目录与入口文件

| 文件/目录 | 在当前工程中的位置 |
| --- | --- |
| `Kconfig` | 构建期功能选择入口，`source` 各层 Kconfig 与 `project/Kconfig` |
| `scripts/kconfig.py` | Kconfig 前端：解析、求值，生成 `.config`、`kconfig.cmake`、`autoconf.h` |
| `framework/algorithm/Kconfig` | 算法能力符号（`DUST_VISION_YOLO`、`DUST_VISION_YOLO_REFINER`、`DUST_FLT_*`、`DUST_MATH_ANGLE`） |
| `framework/cmd/Kconfig` | 控制台、日志、变量与 YAML 配置读取能力符号 |
| `framework/topic/Kconfig` | 话题通道能力符号 |
| `framework/modules/Kconfig` | 设备模块能力符号（`DUST_MOD_DEV_HIKROBOT`） |
| `project/Kconfig` | 业务门禁与使用开关（`USE_*` / `TRD_*`），靠 `select` 拉起架构能力 |
| `framework/init/main.cpp` | 进程入口，只负责调用统一启动、等待和关闭流程，不放业务逻辑 |
| `framework/init/` | `REGISTER_INIT` / `REGISTER_THREAD`、阶段顺序、失败清理和线程停止/join |
| `framework/cmd/config/` | YAML 文件加载、必需字段/未知字段检查和带范围的类型读取 |
| `framework/cmd/shell/` | 日志传输、交互命令、运行时变量和变量变更通知 |
| `framework/topic/channel.hpp` | 单槽最新值通道模板：发布、订阅者游标、阻塞等待、关闭与清空 |
| `framework/topic/camera_to/` | 相机图像和采集时间戳的消息契约及相机帧通道 |
| `framework/topic/vision_to/` | 同帧图像、检测列表、时间戳和推理耗时的消息契约 |
| `framework/modules/camera/hikrobot.*` | 海康 MVS SDK 封装，提供设备打开、采图、转换和关闭能力 |
| `framework/algorithm/vision/core/` | `DetectedArmor`、颜色/尺寸/标签枚举、几何校验和精修使用的灯条类型 |
| `framework/algorithm/vision/detect/` | TensorRT 检测器、预处理/推理/后处理和模型类别映射 |
| `framework/algorithm/vision/refine/` | 给定 YOLO 粗四点后的可选局部灯条提取与角点细化 |
| `project/thread/CMakeLists.txt` | 业务线程源文件加入可执行程序，并把各线程 YAML 复制到可执行文件旁 |
| `project/CMakeLists.txt` | 创建最终 `vision_app`，链接框架库并配置链接脚本/运行库路径 |
| `framework/CMakeLists.txt` | 按 `CONFIG_*` 装配算法、topic、core 与 modules 各库 |
| `framework/cmd/build/` | 构建与运行脚本（`dust`、`build.sh`、`run.sh`） |

`framework/` 和 `project/` 的依赖方向原则上是：**基础设施/算法供业务线程使用，而不是反过来依赖业务线程**。例如 `ArmorDetector` 不应 include `trd_vision.cpp`；相机模块不应知道 `vision_to`；线程负责把独立模块连接起来。

## 构建期功能选择（Kconfig 链路）

### 文件与分层

| 文件 | 符号 | 谁改它 |
| --- | --- | --- |
| `Kconfig` | `PRJ_MAIN` 总门禁 | 维护者 |
| `framework/algorithm/Kconfig` | `DUST_VISION_YOLO`、`DUST_VISION_YOLO_REFINER`、`DUST_FLT_*`、`DUST_MATH_ANGLE` | 架构层 |
| `framework/cmd/Kconfig` | `DUST_CMD_SHELL`、`DUST_CMD_SHELL_LOG`、`DUST_CMD_SHELL_VAR`、`DUST_CMD_CONFIG` | 架构层 |
| `framework/topic/Kconfig` | `DUST_TPC_CAMERA_TO`、`DUST_TPC_VISION_TO` | 架构层 |
| `framework/modules/Kconfig` | `DUST_MOD_DEV_HIKROBOT` | 架构层 |
| `project/Kconfig` | `USE_CMD_SHELL`、`USE_CMD_CONFIG`、`TRD_CAMERA`、`TRD_TEST`、`TRD_VISION`、`TRD_DISPLAY` | 业务层 |

规则：

- **架构层 `DUST_*` 默认 `n`**：不显式启用就不进镜像，也不去找它的外部依赖。
- **业务层用 `select` 拉起架构能力**，不在别处硬写 `CONFIG_DUST_*=y`；业务线程自己的开关用 `TRD_*`，通用使用开关用 `USE_*`。
- 依赖写在 Kconfig 里（例如 `TRD_DISPLAY` 依赖 `TRD_VISION`），依赖不满足时符号被置 `n`，对应源文件不会编进程序。
- 业务符号保持原名、不带 `DUST_` 前缀；被 select 的架构符号统一 `DUST_` 前缀。

### 求值与生成

配置阶段由根 CMake 调用 `scripts/kconfig.py`（纯 Python 标准库，不需要 kconfiglib）：解析 Kconfig 树 → 按 `select` / `depends on` / `default` 求值 → 生成

```text
build/kconfig/.config         规范化结果（含 "# CONFIG_X is not set"）
build/kconfig/kconfig.cmake   set(CONFIG_X ON/OFF)，供 CMake 消费
build/kconfig/autoconf.h      #define CONFIG_X 1，强制 include 给所有编译单元
```

CMake 侧只用结果：`include(kconfig.cmake)` 得到 `CONFIG_*`，`-include autoconf.h` 让 C++ 侧看见同一个符号。装配动作按层分工：

- `project/thread/CMakeLists.txt`：`TRD_*` 决定哪些业务线程源文件进 `vision_app`；
- `framework/CMakeLists.txt`：`DUST_*` 决定 `framework_core` / `framework_vision` 的源文件与依赖；
- `framework/modules/CMakeLists.txt`：`DUST_MOD_DEV_HIKROBOT` 决定是否查找 MVS SDK；
- 根 `CMakeLists.txt`：`DUST_VISION_YOLO` 决定是否查找 CUDA/TensorRT 与 OpenCV dnn。

求值失败（例如依赖不满足却要求 `=y`）会在配置阶段直接报错，不会带着半套配置往下走。

### 用哪份配置构建

默认直接用 Kconfig 文件里的 `default`。要另一套组合时，给它一份已存在的 `.config`：

```bash
cmake -S . -B build -DKCONFIG_CONFIG=/absolute/path/to/.config
```

### 改动开关时的落点

| 你要做的事情 | 改哪里 |
| --- | --- |
| 新增一个架构能力 | 对应层的 `Kconfig` 加 `DUST_*`（`default n`），并在该层 CMake 里按它裁源文件 |
| 新增一个业务线程 | `project/Kconfig` 加 `TRD_*`（`default y`，按需要 `select` 架构能力），`project/thread/CMakeLists.txt` 加条件源文件 |
| 新增一个通用使用开关 | `project/Kconfig` 的“使用开关”分区加 `USE_*`，照同族符号的写法 `select` 架构能力 |
| 关闭某项功能 | 改 Kconfig 里的 `default`，或给它一份 `.config` |

## 启动与线程生命周期

`framework/init/main.cpp` 只调用统一启动、等待和关闭入口。业务组件在自己的源文件中通过 `REGISTER_INIT`、`REGISTER_THREAD` 注册回调，链接脚本保留注册段；启动框架按阶段执行注册项。

- `REGISTER_INIT`：初始化资源，并提供可处理部分初始化失败的清理回调。
- `REGISTER_THREAD`：启动回调应快速创建线程并返回；停止回调负责发出停止请求、解除阻塞；join 回调等待线程退出。
- 停止顺序：请求停止 → 等待线程退出 → 释放资源。不要在线程仍运行时释放它正在使用的检测器、相机或通道。

启动器的阶段顺序是 `PreInit → EarlyInit → MidInit → LateInit → AppInit`，每个阶段完成初始化后再启动对应阶段的线程。初始化阶段内按 `High → Mid → Low` 排序；初始化项可以选择失败后中止启动或继续。主线程既可等待停止信号，也可挂接非阻塞事件回调（当前显示使用主线程循环，因此 HighGUI 窗口事件在主线程处理）。`System_Shutdown()` 可重复调用，并在清理回调异常时继续清理其他资源、记录错误状态。

业务组件生命周期可按以下模板理解：

```text
REGISTER_INIT: 创建/校验资源 ──失败时清理本组件部分资源
REGISTER_THREAD: 创建工作线程并快速返回
运行期间: 线程拥有自己的状态和 Subscriber
request_stop: 设置停止标志、关闭/唤醒等待点
join: 等待工作线程彻底退出
cleanup: 释放线程不再使用的模型、设备与缓存
```

注册不是依赖静态构造函数调用一个全局 registry 的隐式魔法：宏把注册项指针放入专用链接段，启动器遍历链接脚本提供的段边界。这使组件声明分散在所属文件，同时启动顺序仍集中可控。新增注册源若没有进入 `vision_app` 或链接段未保留，注册项就不会运行，所以 CMake/链接配置是生命周期机制的一部分。

## 如何在当前架构中开发

### 新增一个业务功能

1. **先定义数据边界。** 确认输入、输出、时间戳、线程所有权和失败状态；跨线程数据通过 `framework/topic/` 中明确的消息传递，不用裸全局变量。
2. **把计算放进算法层。** 在 `framework/algorithm/` 中实现不依赖线程和硬件 I/O 的逻辑，提供清楚的输入/输出类型；有状态计算要明确状态所有者和生命周期。
3. **把硬件操作留在模块/驱动层。** SDK、串口和设备重连细节不要散落到业务算法中。
4. **在线程层编排。** 在 `project/thread/` 读取本线程配置、订阅输入 topic、调用模块/算法、发布结果；始终定义停止和阻塞唤醒行为。
5. **注册生命周期。** 新增初始化或线程时，使用现有注册机制，并在相应 CMake `target_sources` 中加入源文件。确认注册源直接进入最终可执行程序，避免静态库链接时被裁掉。
6. **补构建期开关。** 架构能力写进所属层 `Kconfig`，业务线程写进 `project/Kconfig`；默认值保持与当前行为一致。
7. **配置与验证。** 线程 YAML 放在所属线程目录，构建时复制到可执行文件旁；配置必须校验必需字段、类型和值域。为算法逻辑增加可重复的离线样例或测试，设备相关验证单独标明硬件前提。

### 以当前视觉链为例

- 新增一种图像算法：在 `framework/algorithm/vision/` 下按职责归入 `detect/`、`refine/` 或新增清晰子层；线程只负责调用它。
- 增加检测结果字段：先判断字段属于单帧检测、位姿还是跨帧目标；更新对应结果契约及消费者，不要把跟踪状态直接塞进 `DetectedArmor`。
- 增加跨线程输出：定义明确消息并确认发布者、订阅者和通道关闭行为；沿用同帧图像与采集时间戳的关联原则。
- 增加线程参数：放到对应线程 YAML，通过 `cmd::config::Config` 加载并验证；不要让算法模块自行到处读取 YAML。
- 新增算法源文件：同时更新相关 `CMakeLists.txt`（并考虑是否需要新的 `CONFIG_*` 条件）。本项目不是靠扫描目录自动编译。

开发新功能前建议先读：`project/thread/vision/trd_vision.cpp`（线程编排）、`framework/topic/vision_to/vision_to.hpp`（消息契约）、`framework/algorithm/vision/detect/armor_detector.hpp`（算法接口）和同目录 CMake 文件（构建接入）。

### 按改动类型选择落点

| 你要做的事情 | 优先修改的位置 | 典型例子 |
| --- | --- | --- |
| 改进纯图像/几何算法 | `framework/algorithm/vision/` | 角点质量评估、PnP 输入点整理、候选匹配算法 |
| 增加或调整检测器 | `framework/algorithm/vision/detect/` | 新模型输出解码、新后处理步骤 |
| 改善设备访问 | `framework/modules/` 或 `framework/drivers/` | 相机能力封装、串口协议底层读写 |
| 增加线程间数据 | `framework/topic/<name>/` | 明确的观测结果或控制反馈消息 |
| 改变业务处理顺序 | `project/thread/` | 接入跟踪器、把结果发给新的消费者 |
| 增加可调参数 | 对应业务线程 YAML + 线程初始化读取 | 颜色策略、检测配置或超时参数 |
| 增加通用运行时命令 | `framework/cmd/` | 与具体设备/算法无关的 shell 能力 |
| 改变启动/停止行为 | `framework/init/` 与注册调用处 | 新的初始化阶段或生命周期动作 |
| 增删一个功能开关 | 对应层 `Kconfig`（+ 该层 CMake 条件） | 新线程、新算法能力、新设备模块 |

一个实用判断方式：如果函数可以拿固定输入离线调用并验证，优先放算法层；如果代码在调用 MVS/CUDA/串口等外部接口，放驱动或模块；如果代码决定“何时调用谁、数据发给谁”，放业务线程。

### 新增视觉处理模块的落地清单

以新增一个独立视觉算法模块为例，建议按下面顺序逐项完成：

1. 写清楚算法输入/输出的坐标系、单位、点顺序、时间戳和失败表示。不要只返回一个数值而让调用者猜它是否有效。
2. 若需要新领域类型，放在合适的 `core/` 或结果契约文件中；确认它表达的是检测、位姿还是跟踪状态，避免把不同生命周期的数据混成一个类。
3. 实现算法本体，使其不依赖 `project/thread`、相机 SDK、shell 全局状态或显示窗口。配置由调用者传入，或由线程解析后构造算法配置对象。
4. 在模块实现旁增加 Doxygen 函数说明，并为坐标变换、筛选门限、回退逻辑等非显然决策写行内解释。
5. 在所属 CMake 库中显式登记新增 `.cpp`；若新增目录，也更新 include 路径或目标配置。根 CMake 不会自动发现源文件。
6. 在线程中创建/持有模块实例，在正确生命周期阶段初始化；处理每帧时只调用接口并组织消息，不把算法细节搬到线程里。
7. 设计验证数据：至少覆盖正常输入、空/越界/非有限输入、边界条件和失败回退。GPU/相机依赖的集成验证和纯算法验证分开描述。
8. 检查错误关闭路径：通道等待是否会被唤醒、停止请求是否能被观察、join 是否发生在资源释放之前。

### YAML 配置接入流程

1. 把配置放到拥有该功能的线程目录，如 `project/thread/vision/trd_vision.yaml`。
2. 构建时该文件会被复制到可执行文件旁（`build/project/thread/<线程>/`），程序按“`DUST_CONFIG_ROOT` 指定的配置根，未设置时可执行文件目录”解析相对路径。
3. 初始化时使用 `cmd::config::Config::Load(path, {必需字段...})`。当前 Config 会检查必需字段、未知字段和重复字段；读取 API 对类型和值域做验证。
4. 将验证后的值拷贝到业务配置结构，再创建算法/模块。不要让处理每帧的热路径反复读 YAML。
5. 配置注释写明单位、边界含义、是否重启生效和对行为/实时性的影响。
6. 编译期开关只决定“有没有这段代码”，运行期 YAML 只决定“这次用什么参数”；不要用 YAML 字段代替构建期裁剪，也不要让构建期开关去否决 YAML 里合法存在的参数。

### 增加跨线程消息的检查项

- 先判断是否真的需要线程间消息；同一线程内部处理不必为了抽象而造 topic。
- 消息字段要能关联同一帧或同一命令，例如采集时间戳、帧序号或请求序号；不能把“处理完成时间”误当“图像采集时间”。
- 指定唯一发布者和消费者线程；每个消费线程各自创建 `Subscriber`，不要多个线程共享一个订阅者游标。
- 定义数据所有权。`cv::Mat` 浅拷贝共享内存，发布后视为只读；修改前 `clone()`。
- 定义通道关闭/清空与等待的退出语义。停止发布后要能唤醒阻塞消费者，消费者退出后再清理相关资源。
- 单槽通道代表“最新状态”，不保证每条消息都送达；控制命令、事件或必须逐条处理的数据要评估是否需要不同通道策略。

### 调试与定位建议

1. 先看配置阶段输出：Kconfig 求值结果、CUDA/TensorRT 版本、最终 `CONFIG_*`；再确认线程是否启动、配置是否加载、资源是否初始化成功。
2. 再确认 topic 发布计数、消费者是否取到消息、时间戳是否持续更新；区分“没帧”和“算法没检出”。
3. 视觉问题先固定一帧/视频离线复现，分别观察原图、预处理输入、模型输出统计、NMS 后结果和最终 `DetectedArmor`。
4. 性能问题分别记录预处理、推理、后处理、帧龄；不要只看总 FPS 推断瓶颈。
5. 多线程问题检查状态归属、锁保护范围、阻塞等待是否可被关闭打断、停止/join/释放顺序。
6. 改动后至少执行 `git diff --check`；代码改动再针对目标构建和相关回归验证。依赖硬件的检查说明硬件型号、engine 和运行条件。

## C++ 代码风格

本项目使用 C++17，并以现有代码为准保持一致：

- 缩进使用 4 个空格，不使用 Tab；控制块和函数定义使用 Allman 风格大括号。
- 类型使用 `PascalCase`，函数/方法按现有接口使用 `PascalCase`，局部变量和成员变量沿用所在模块的命名习惯；避免为了个人偏好批量改名。
- 常量使用 `k` 前缀（如 `kWindowName`）；成员变量后缀 `_`（如 `config_`）；枚举使用 `enum class`。
- 优先使用明确的类型、`const`、引用和 RAII；资源获取/释放与线程停止/join 要成对，避免裸所有权和隐式共享状态。
- 头文件使用 `#pragma once`；包含项按项目现有顺序分组，避免无用 include；新增 `.cpp` 必须接入 CMake。
- 算法代码与 I/O、配置、线程编排分离；跨线程修改状态时明确同步手段和所有权。
- 函数尽量保持单一职责；复杂步骤拆成有名字的辅助函数，让调用顺序体现数据处理阶段。
- 校验外部输入和配置边界，在模块入口尽早失败；不要让非法状态传播到后续线程或控制逻辑。
- 返回结果表达失败时优先使用项目已有结果/异常约定；不要用特殊坐标、负置信度等魔数表示无效。
- 不为了“通用化”引入尚无第二个调用场景的抽象；不顺手重命名或整理不相关代码。
- 并发逻辑明确谁拥有状态、谁能修改、锁覆盖哪些数据；不要认为持有 `cv::Mat` 引用计数就能保证像素写入线程安全。

## 注释与 Doxygen 风格

- 对外可见的类/接口和关键实现使用 Doxygen `/** ... */`，用 `@brief` 描述职责；参数、返回值、异常等确实有帮助时再写 `@param`、`@return`、`@throws`。
- **函数实现的说明注释放在函数实现所在的 `.cpp`，紧挨实现；不要为了放注释而把实现搬进 `.hpp`。** 头文件只在声明附近保留理解接口所必需的简短契约说明。
- 注释解释“为什么这样做”、坐标/单位/线程约束、失败和回退语义；不要逐行翻译代码。
- 配置项注释写清单位、范围、默认值含义和运行影响；坐标、角点顺序、时间戳基准必须明确注明。
- Kconfig 符号的 `help` 写清“打开它会编译/链接什么”，不要在 `help` 里写实现原理。
- 代码变更时同步维护过时注释和文档，但避免添加与代码重复且容易失效的大段注释。
- 文件头按同目录现有格式写 `@file` 和简短 `@brief`；不要为了新文件复制无关历史信息或臆造版本/作者字段。
- 注释用中文或英文均可，但同一文件/模块保持一致；术语、单位和缩写首次出现时写清楚。
- 对生命周期回调说明启动、停止、join、异常和清理责任；对线程共享数据说明拥有者及同步方式。
- 对数值阈值说明坐标单位、开闭区间和来源/调参依据；如果目前只是待验证的经验值，应明确注明。
- 当行为由外部库约束（如 HighGUI 主线程要求、MVS 缓冲区生命周期）时，注释说明约束和采取的措施。

示例：

```cpp
/**
 * @brief 处理一帧图像并返回有效装甲板观测
 *
 * @param bgr_image 输入的 BGR 图像，只读
 * @return 检测结果及统计信息
 * @throws std::invalid_argument 输入图像为空或类型不符合要求
 */
ArmorDetectionResult ArmorDetector::Detect(const cv::Mat& bgr_image)
{
    // 实现中的注释解释关键约束或非显然处理，不复述每行代码
}
```

## 当前视觉检测说明

当前检测主链使用 TensorRT 10 加载 0526 engine：图像等比例缩放并放入 `640×640` 画布，模型输出候选经 objectness 门限、角点坐标映射、NMS、颜色/标签/尺寸解码和几何校验后，生成 `DetectedArmor`。显示置信度是 objectness，不代表角点精度或类别分支概率。

局部灯条精修（`ArmorRefiner`）由构建期开关 `DUST_VISION_YOLO_REFINER` 决定是否编进程序：编进来就生效，YAML 里必须给出六个精修参数；没编进来时这些字段允许存在但不参与运行。详细流程见[视觉识别流程说明](doc/framework视觉识别流程说明.md)，构建期开关见[Kconfig 构建链路引入规划](doc/Kconfig构建链路引入规划.md)。

## 配置、构建与运行

### 依赖

根 CMake 当前依赖 C++17、CMake 3.16+、Python 3（运行 Kconfig 前端）、pthread、Eigen3、OpenCV（core/highgui/imgproc/dnn）、yaml-cpp、CUDA Toolkit 12+、TensorRT 10.x 和完整海康 MVS SDK（包含 USB3 transport 库）。CUDA/TensorRT 默认从项目 `.venv/` 查找；MVS SDK 根目录可用 `HIKROBOT_MVS_ROOT` 指定。只有 `DUST_VISION_YOLO` 打开时才会去找 CUDA/TensorRT 与 OpenCV dnn。

### 构建

```bash
cmake -S . -B build
cmake --build build --parallel
```

配置阶段会先跑 Kconfig 前端，打印各符号结果，并生成 `build/kconfig/` 下的三份文件。也可以直接用仓库自带的脚本（`framework/cmd/build/`）：

```bash
dust build        # 等价于上面的配置 + 构建
dust run          # 运行 build/project/vision_app
```

### 运行配置

每个业务线程维护自己的 YAML，源文件在 `project/thread/<线程>/`。构建时它们会被复制到可执行文件旁（`build/project/thread/<线程>/`），因此直接运行即可加载；若要让程序改用别处的配置，设置 `DUST_CONFIG_ROOT` 指向包含 `thread/` 的配置根（源码树中可指向 `<repo>/project`），相对路径会以它为基准。视觉模型的 `engine_path` 写在 `trd_vision.yaml` 里，必须是能直接打开的路径，不由 CMake 生成或复制。

```bash
./build/project/vision_app
DUST_CONFIG_ROOT="$PWD/project" ./build/project/vision_app
```

运行需要可用的 MVS SDK、CUDA/TensorRT 动态库和海康相机；OpenCV 窗口显示需要图形桌面。

运行时可在 `dust>` shell 查看日志和变量，例如：

```text
log list
log on vision
var list
var set display.show 0
```

### 常见构建/运行问题

| 现象 | 优先检查 |
| --- | --- |
| 配置阶段报 Kconfig 求值失败 | 报错会给出符号和出处；多半是依赖不满足却要求 `=y`，或符号名拼错 |
| `CONFIG_*` 没生效 | 生成目录 `build/kconfig/` 是否更新；`build/CMakeCache.txt` 里是否有旧的 `KCONFIG_CONFIG` |
| CMake 找不到 CUDA/TensorRT | `.venv/` 是否包含匹配的 Toolkit、TensorRT 头文件和 `nvinfer`；必要时设置 `DUST_DEPENDENCY_ROOT` / `CUDAToolkit_ROOT`；确认 `DUST_VISION_YOLO` 是否打开 |
| 报告 TensorRT 版本不符 | 当前根工程要求 TensorRT 10.x，确认实际 include 和 library 来自同一安装 |
| 找不到 engine | 看 `trd_vision.yaml` 的 `engine_path` 是否是可打开的路径（当前写成绝对路径） |
| 报 `bad file ...thread/vision/trd_vision.yaml` | YAML 解析基准不对：确认构建（会复制 YAML）或设置 `DUST_CONFIG_ROOT` 指向含 `thread/` 的根 |
| 找不到 MVS 或运行时加载失败 | 检查 `HIKROBOT_MVS_ROOT`、SDK 的库和 USB3 transport 组件，以及动态库搜索路径 |
| 新增源文件后符号未链接 | 检查对应 `target_sources()` / `add_library()`，以及该源文件所在功能的 `CONFIG_*` 是否为 `y` |
| 注册的初始化/线程没有执行 | 注册源是否进入最终可执行文件，链接脚本是否保留注册段，注册函数是否符合回调签名 |

## 文档索引

| 文档 | 内容 |
| --- | --- |
| [Kconfig 构建链路引入规划](doc/Kconfig构建链路引入规划.md) | Kconfig 与 CMake 的职责分工、生成物和落地步骤 |
| [视觉配置架构引入规划](doc/视觉配置架构引入规划.md) | 构建期开关、线程 YAML 与算法 Config 的三层配置边界 |
| [视觉识别流程说明](doc/framework视觉识别流程说明.md) | 当前 YOLO 检测、后处理、精修边界和数据流 |
| [视觉自瞄下一步规划](doc/视觉自瞄系统下一步实施规划.md) | 评估、阵营过滤、精修、PnP、跟踪、锁定与只读瞄准建议的建议顺序 |
| [视觉算法分层规划](doc/视觉算法目录分层规划.md) | 视觉算法目录职责与依赖方向 |
| [海康相机接口说明](doc/HikRobot相机模块与SDK接口说明.md) | MVS 相机模块和采集接口 |
| [海康相机断线重连](doc/HikRobot断线重连机制.md) | 相机掉线恢复与退出行为 |
| [相机运行时调参](doc/相机运行时线程内调参规划.md) | 曝光、增益等参数如何安全提交到相机线程 |
| [初始化流程检查报告](doc/电控架构初始化流程检查报告.md) | 注册、启动、失败清理和关闭顺序 |
| [相机帧出口与显示解耦](doc/相机图像出口与显示解耦规划.md) | 相机帧通道和显示架构 |
| [SIGPIPE 处理说明](doc/SIGPIPE问题说明与解决方案.md) | 日志输出断管时的处理与验证 |
