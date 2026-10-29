# 通讯驱动层与 EtherCAT 主站落地规划

> 2026-10-30。
>
> 目标工程：`/home/rmconstruction/Desktop/qingyu`（Linux / C++17 骨架，`framework/drivers/` 当前**是空目录、未入库**）
> 从站实物：HPM6E00EVK（HPM6E80 片内 ESC），状态见 `doc/EtherCAT从站当前状态-给主站.md`
> 主站机器：本机 `rmconstruction-NUC10i7FNH`，2× Intel I225-LM（`igc`）

---

## 0. 结论先行

**做两件事：把 `framework/drivers/` 这一层立起来（骨架），然后 EtherCAT 主站作为它的第一个落地实现。**

骨架不写空抽象，只交付四样东西：**目录约定 + Kconfig 门禁命名族 + CMake 装配模式 + README 判据**。
第一个实现跑通后，它就是后来者的模板——这比先写一个没人实现的 `Stream` 接口更实在。

EtherCAT 走 **SOEM**（`ecx_*` 2.0 API），不走 IgH。决定性原因：本机是 `igc` 网卡，IgH 原生驱动列表里没有它。

**一个必须先说的前提**：本机 **SOEM 未安装**（`/usr/local/include/soem` 不存在、`slaveinfo` 不在 PATH）。
`temp/dust-control-git/tests/SOEM/SOEM` 有一份 v2.0.0 源码可作参照，但**不能直接拿来链**——它是另一工程的 vendored 副本。
P3 步 3-1 必须先装，否则 `CONFIG_DUST_COM_ETHERCAT=y` 一开就 `FATAL_ERROR`。

---

## 1. 现状实测（规划的唯一基线）

### 1.1 本工程

| 项 | 实测 | 影响 |
|---|---|---|
| `framework/drivers/` | 存在但**空**，`git ls-files framework/drivers` 输出为空 | 整层从零建，没有可照抄的同仓先例 |
| `framework/CMakeLists.txt` | 已装配 `framework_algorithm` / `framework_topic` / `framework_core` / `framework_shell` / `framework_modules`，**无 drivers 段** | 要加一段 |
| 根 `Kconfig` | source 了 algorithm / cmd / shell / modules / topic / project，**无 drivers** | 要加一行 |
| 构建形态 | **静态库 + `target_sources`**（`framework_core` 是 STATIC，shell 层用 `target_include_directories(framework_core PUBLIC ...)`） | 与父架构 `target_sources(app)` 不同，照本工程 |
| `framework/modules/` | 空装配位（`CMakeLists.txt` 只有注释，`Kconfig` 只有注释） | 层骨架的**现成范本**，drivers 照它写 |
| 线程外壳 | `init::Thread`，`Start(entry, period, context, name, ThreadFaultConfig)`，`TaskResult{Continue, Retry, Exit}` | EtherCAT 周期线程直接套 |
| 日志 | `DUST_LOG_INF/ERR/OK/WRN/DBG`（`framework/shell/log/log.hpp`） | 驱动内不许裸 `printf` |
| 配置 | `cmd::config::Config`（YAML，`Load/Has/GetInt/GetDouble/GetBool/GetString/GetPath`） | 线程参数走它 |
| 注册 | `REGISTER_INIT(fn, stage, level, fail, cleanup, name)` / `REGISTER_THREAD(fn, stage, stop, join, name)` | 线程接入走它 |

### 1.2 从站（`doc/EtherCAT从站当前状态-给主站.md`）

| 项 | 值 |
|---|---|
| Vendor ID | `0x0048504D` |
| Product Code / Revision | `0x00000001` / `0x00000001` |
| 设备名 | `digital_io` |
| RxPDO（主→从） | SM2 `0x1100`，`0x1600`，**32 字节**（8×`0x7010:01~08` UDINT） |
| TxPDO（从→主） | SM3 `0x1400`，`0x1A00`，**32 字节**（8×`0x6000:01~08` UDINT） |
| 接线 | 主站 → **PORTA**；PORTB 空（ESC 内部回环，正常） |
| 速率 | 100BASE-TX 全双工，主站网卡**不能设 10M** |
| 同步 | SM-Synchron（`AssignActivate 0x0000`）优先，DC `0x0300` 从站支持但未调优 |
| 测试行为 | TxPDO 填 `00 01 02 ... 1F`（32 字节递增），RxPDO 只打日志 |

**这两个 32 字节就是 P1 的验证判据**：主站读到 `00 01 ... 1F` 即链路通。

### 1.3 环境

| 项 | 实测 | 结论 |
|---|---|---|
| SOEM | **未安装**（无头文件、无库、无 `slaveinfo`） | P3 必须先装 |
| `libpcap` | 未安装（`/usr/include/pcap.h` 不存在） | SOEM 走 AF_PACKET，**不需要 pcap**，可不装 |
| 网卡 | `enp2s0` / `enp3s0` 均 DOWN、NO-CARRIER；`wlp4s0` UP | EtherCAT 用 `enp2s0`，`ip link set up` 即可，**不配 IP** |
| 实时权限 | 见 `EtherCAT主站选型与落地规划`（已删，`git show HEAD:doc/EtherCAT主站选型与落地规划.md` 可取回） | 1 ms 周期不要求 RT，先跑通再说 |

---

## 2. 分层设计：drivers 层长什么样

### 2.1 三类通讯形态，一个目录族

父架构 `framework/drivers/` 的目录是 `communication/{can,spi,stream}` + `device/{gpio,pwm}`。
本工程沿用**同一套目录名**（便于两架构互相对照），但**只建 communication**——本机没有 GPIO/PWM 需求。

```text
framework/drivers/
├── Kconfig                  ← 门禁符号（DUST_COM_* / DUST_DEV_*）
├── CMakeLists.txt           ← 按 CONFIG_* 装配
├── README.md                ← 本层判据：什么该进、什么不该进、新增照哪抄
└── communication/
    └── ethercat/            ← 第一个落地实现
        ├── ethercat.hpp
        └── ethercat.cpp
```

**三类形态与命名（写死，后来者照此归类）**：

| 形态 | 特征 | 未来落点 | Kconfig 前缀 |
|---|---|---|---|
| 字节流 | 无序字节，`Read/Send`，有接收事件 | `communication/stream/{uart,usb,rs485}/` | `DUST_COM_UART` / `DUST_COM_USB` / `DUST_COM_RS485` |
| 报文 | 定长帧 + ID，有丢帧语义 | `communication/can/` | `DUST_COM_CAN` |
| 周期过程数据 | 固定周期整帧交换，无队列 | `communication/ethercat/` | `DUST_COM_ETHERCAT` |

### 2.2 为什么 P0 不写 `stream.hpp` 抽象

父架构有 `Stream`（`Read`/`Send` + `k_sem sem_`），本工程**没有任何实现者**。

现在写一个只有声明的 `Stream`：
- 无法验证（没有子类，编不出东西）
- 形状靠猜（Linux 版该用 `init::Semaphore` 还是 `poll` fd？EtherCAT 要不要继承它？）
- 后来者照着错的形状实现，改起来比从零写更贵

**判据与 `doc/shell下层形态与thread绝对调度规划.md` §① 的「何时才该拆」同源**：抽象要等有第二个实现者时再抽。
EtherCAT 落地后，若 UART 也接进来，那时两者的共同点才是真共同点。

**这条列为待拍板 A**（见 §9）。

### 2.3 EtherCAT 为什么不套 `Stream`

EtherCAT 主站是**周期整帧交换**，不是字节流：
- 没有「收到一段数据」事件，只有「这一拍 WKC 对不对」
- 没有发送队列，只有「把 IOmap 里的当前值发出去」
- 一次收发要跨全部从站，不是单个通道

硬套 `Stream::Read/Send` 会得到一个语义错位的壳。所以 `EthercatMaster` 是**独立类**，不进 Stream 族。

---

## 3. Kconfig 与构建链

### 3.1 符号（照架构层命名，不自创前缀）

```kconfig
# framework/drivers/Kconfig
config DUST_COM_ETHERCAT
    bool "EtherCAT master (SOEM)"
    default n
    help
      用户态 EtherCAT 主站：打开网口、枚举从站、周期交换过程数据。
      依赖系统已安装 SOEM（/usr/local/lib/libsoem.so）。
```

**为什么是 `DUST_COM_`**：与父架构 `framework/drivers/Kconfig` 的 `DUST_COM_UART` / `DUST_COM_CAN` / `DUST_COM_USB` 同族。
**不许写 `DUST_DRV_` 或 `DUST_ETHERCAT`**——那是自创前缀。

### 3.2 引入链（两处，缺一不可）

| 位置 | 动作 |
|---|---|
| 根 `Kconfig` | `source "framework/drivers/Kconfig"`（插在 `framework/modules/Kconfig` 之后，与层顺序一致） |
| `framework/CMakeLists.txt` | `add_subdirectory(drivers)`（插在 `add_subdirectory(modules)` 之前，drivers 是 modules 的下层） |

**本工程的 `scripts/kconfig.py` 只支持 `source`，不支持 `rsource`**（`kconfig.py:142` 只匹配 `source\s+"..."`），路径从根 Kconfig 出发写全。

### 3.3 装配模式（照本工程，不照父架构）

父架构用 `target_sources(app ...)`；本工程是**静态库**，drivers 层建自己的 target：

```cmake
# framework/drivers/CMakeLists.txt
add_library(framework_drivers INTERFACE)      # 无源时是空接口库
target_include_directories(framework_drivers INTERFACE ${PROJECT_SOURCE_DIR})
```

有实现时改成 STATIC 并 `target_sources`，由 `project/CMakeLists.txt` 按 `CONFIG_*` 链上。

---

## 4. P0：drivers 层骨架

> **P0 拆成 4 步，每次只执行一步。**
> 用户说"开始 P0 第 N 步"= 只改表格里那一个文件，改完停下报告，等"继续"。
> 不许一次做多步，不许跨步连带改（必要时见每步的「连带」栏）。

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| **0-1** | `framework/drivers/Kconfig`（新建） | 只写注释头 + 命名说明，**先不放任何 config**（还没有实现者） | 无 | 无 |
| **0-2** | `framework/drivers/CMakeLists.txt`（新建） | `add_library(framework_drivers INTERFACE)` + include 目录 | 无 | 无 |
| **0-3** | `framework/drivers/README.md`（新建） | 本层判据：放什么/不放什么/三类形态/新增照哪抄 | 无 | 无 |
| **0-4** | 根 `Kconfig` + `framework/CMakeLists.txt` | 各加一行把 drivers 接进链 | 0-1、0-2 | 无 |

**编译状态**：四步全程可编。0-1~0-3 是新增文件不进构建；0-4 只是把空库接进链，`INTERFACE` 库无源文件，链接零影响。

### 4.1 步 0-1：`framework/drivers/Kconfig`

```kconfig
# drivers/Kconfig —— 底层通讯/设备驱动层。
# 架构层能力默认 n，由业务层 select 拉起。
#
# 命名族（照父架构 framework/drivers/Kconfig，不许自创前缀）：
#   DUST_COM_*  通讯驱动（UART / USB / RS485 / CAN / ETHERCAT）
#   DUST_DEV_*  设备驱动（GPIO / PWM）
#
# 骨架阶段没有任何实现。新增驱动时在这里加 config，
# 并在 CMakeLists.txt 里按同一个符号建 target。
```

### 4.2 步 0-2：`framework/drivers/CMakeLists.txt`

```cmake
# ============================================================================
#
# framework/drivers/CMakeLists.txt —— 底层通讯/设备驱动装配
#
# 骨架阶段没有驱动实现，只导出 include 路径。
# 新增驱动时照 CONFIG_DUST_COM_* / CONFIG_DUST_DEV_* 加块，
# 源文件挂到 framework_drivers 上。
#
# ============================================================================

add_library(framework_drivers INTERFACE)
target_include_directories(framework_drivers INTERFACE ${PROJECT_SOURCE_DIR})
```

### 4.3 步 0-3：`framework/drivers/README.md`

要写清四件事（照 `framework/modules/CMakeLists.txt` 的注释密度）：

1. **放什么**：靠近硬件/总线的协议实现（打开设备、收发字节、维护状态机）
2. **不放什么**：业务策略、PDO 语义、算法——那些归 `modules/` 和 `algorithm/`
3. **三类形态表**（本文 §2.1 那张表）
4. **新增驱动的四步**：Kconfig 加符号 → 目录建 `communication/<名字>/` → CMake 加 `if(CONFIG_*)` 块 → 业务层 `select`

### 4.4 步 0-4：接链

根 `Kconfig`：

```kconfig
source "framework/modules/Kconfig"
source "framework/drivers/Kconfig"     ← 新增这一行
source "framework/topic/Kconfig"
```

`framework/CMakeLists.txt` 末尾：

```cmake
add_subdirectory(modules)

# ============================================================================
#
# drivers：底层通讯/设备驱动（骨架无实现，保留装配位）
#
# ============================================================================

add_subdirectory(drivers)              ← 新增这一段
```

---

## 5. P1：EtherCAT 主站驱动

> **P1 拆成 5 步。**
> 用户说"开始 P1 第 N 步"= 只改表格里那一个文件，改完停下报告，等"继续"。

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| **1-1** | `framework/drivers/communication/ethercat/ethercat.hpp`（新建） | `Config` / `SlaveState` / `Master` 类声明 | P0 全 | 无 |
| **1-2** | `framework/drivers/communication/ethercat/ethercat.cpp`（新建） | `Init` / `Close` / `Send` / `Receive` / `RequestOperational` / 状态查询 | 1-1 | 无 |
| **1-3** | `framework/drivers/Kconfig` | 加 `DUST_COM_ETHERCAT`（`default n`） | 0-1 | 无 |
| **1-4** | `framework/drivers/CMakeLists.txt` | 改成 STATIC + `if(CONFIG_DUST_COM_ETHERCAT)` 块 + `find_path/find_library` | 1-2、1-3 | 无 |
| **1-5** | `project/CMakeLists.txt` | 链接段加 `if(CONFIG_DUST_COM_ETHERCAT) target_link_libraries(... framework_drivers)` | 1-4 | 无 |

**编译状态**：1-1、1-2 是新增文件、未进构建 → 可编。1-3 加符号 `default n` → 可编。1-4 有 `if(CONFIG_...)` 保护，默认不进 → 可编。**1-5 之后若把符号开成 `y` 而 SOEM 没装，会在配置阶段 `FATAL_ERROR`**——这是设计意图，不是 bug。

### 5.1 步 1-1：`ethercat.hpp`

```cpp
/**
 * @file ethercat.hpp
 * @author qingyu
 * @brief EtherCAT 主站（SOEM）—— 网口初始化、从站枚举、周期过程数据交换
 * @version 0.1
 * @date 2026-10-30
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace drivers::ethercat
{

/**
 * @brief 主站配置
 *
 * 由调用方从 YAML 读出后填入；本类不读配置文件。
 */
struct Config
{
    std::string interface_name;          // 网口名（enp2s0），不配 IP
    int         timeout_us = 2000;       // 单次过程数据接收超时（µs）
    bool        use_dc     = false;      // 是否启用分布式时钟
};

/**
 * @brief 从站运行状态快照
 */
struct SlaveState
{
    uint16_t index;                      // 从站序号（1 起）
    uint16_t al_state;                   // AL 状态（EC_STATE_*）
    uint16_t al_status_code;             // AL 状态码，错误时非 0
    uint32_t output_bytes;               // RxPDO 字节数（主→从）
    uint32_t input_bytes;                // TxPDO 字节数（从→主）
};

/**
 * @brief EtherCAT 主站，基于 SOEM
 *
 * 单实例、单网口。Init 成功后由上层线程每拍调用 Send() / Receive()。
 * 本类不做周期等待——节拍由调用方（线程外壳或任务体）决定。
 * 所有调用必须来自同一线程：SOEM 的 ecx_context 不可重入。
 */
class Master final
{
public:
    Master() = default;
    ~Master();

    Master(const Master&)            = delete;
    Master& operator=(const Master&) = delete;

    /**
     * @brief 打开网口、枚举从站、映射过程数据
     * @param config 主站配置
     * @return true=至少一个从站且映射成功
     * @throws std::runtime_error 网口打开失败、无从站、映射失败
     */
    bool Init(const Config& config);

    /**
     * @brief 关闭主站并释放 SOEM 上下文；可重复调用
     */
    void Close() noexcept;

    /**
     * @brief 把所有从站切到 OP 并等待就绪
     * @return true=全部进入 OP
     */
    bool RequestOperational();

    /**
     * @brief 发送一帧过程数据
     * @return true=提交成功
     */
    bool Send();

    /**
     * @brief 接收一帧过程数据并校验工作计数器
     * @return true=WKC 达到期望值
     */
    bool Receive();

    /**
     * @brief 从站数量
     */
    uint16_t SlaveCount() const { return slave_count_; }

    /**
     * @brief 取某个从站的运行状态
     * @param index 从站序号（1 起）
     * @return 状态快照；越界时 index=0
     */
    SlaveState GetSlaveState(uint16_t index) const;

    /**
     * @brief 取某个从站的输出缓冲基址（RxPDO，主→从）
     * @param index 从站序号（1 起）
     * @return 指向 IOmap 内该从站输出区的指针；越界或未初始化返回 nullptr
     */
    uint8_t* Outputs(uint16_t index);

    /**
     * @brief 取某个从站的输入缓冲基址（TxPDO，从→主）
     * @param index 从站序号（1 起）
     * @return 指向 IOmap 内该从站输入区的指针；越界或未初始化返回 nullptr
     */
    const uint8_t* Inputs(uint16_t index) const;

    /**
     * @brief 期望工作计数器
     */
    int ExpectedWorkingCounter() const { return expected_wkc_; }

private:
    void*                context_ = nullptr;   // SOEM ecx_contextt，锁在 .cpp 里
    std::vector<uint8_t> io_map_;              // 过程数据映射缓冲
    uint16_t             slave_count_ = 0;     // 从站数量
    int                  expected_wkc_ = 0;    // 期望 WKC
    bool                 initialized_ = false; // 是否已初始化
};

} // namespace drivers::ethercat
```

**`context_` 为什么是 `void*`**：头文件不 include SOEM，把 SOEM 完全锁进 `.cpp`。
上层（线程）编译不依赖 SOEM 的 include 路径，将来换栈（IgH）只改 `.cpp`。

### 5.2 步 1-2：`ethercat.cpp` 的关键四段

**① Init 的 IOmap 大小必须按真实位宽算**——这是最容易踩的坑：

```cpp
    // 按实际 IO 位宽求和，尾部留余量。
    // 给小了 ecx_config_map_group 返回 0 或写越界。
    uint32_t bits = 0;
    for (int i = 1; i <= ctx->slavecount; ++i) {
        bits += ctx->slave[i].Obits + ctx->slave[i].Ibits;
    }
    io_map_.assign((bits + 7U) / 8U + 64U, 0);

    if (ecx_config_map_group(ctx, io_map_.data(), 0) <= 0) {
        Close();
        throw std::runtime_error("ecx_config_map_group failed");
    }

    // 期望 WKC：每组输出算 2（读写各 1）+ 输入算 1，取自 SOEM 自己算的值
    expected_wkc_ = (ctx->grouplist[0].outputsWKC * 2) + ctx->grouplist[0].inputsWKC;
```

**② Init 的完整顺序**（SOEM 2.0 `ecx_*` API，本地 `temp/dust-control-git/tests/SOEM/SOEM/include/soem/ec_main.h:581-613` 已核对签名）：

```text
new ecx_contextt
  → ecx_init(ctx, ifname)              失败 → Close + throw
  → ecx_config_init(ctx) <= 0          失败 → Close + throw（"no slave found"）
  → 算 bits → io_map_.assign
  → ecx_config_map_group(ctx, io_map, 0) <= 0  失败 → Close + throw
  → expected_wkc_ = grouplist[0].outputsWKC*2 + inputsWKC
  → if (use_dc) ecx_configdc(ctx)
  → initialized_ = true
```

**③ Receive 只判 WKC，不做重连**：

```cpp
bool Master::Receive()
{
    if (!initialized_) {
        return false;
    }
    const int wkc = ecx_receive_processdata(static_cast<ecx_contextt*>(context_), config_.timeout_us);

    return wkc >= expected_wkc_;
}
```

重连策略归线程（连续失败 N 拍后上报故障），驱动不做——驱动越权做重连会让线程的故障策略失效。

**④ RequestOperational 的等待形状**：

```text
for i in 1..slave_count: ctx->slave[i].state = EC_STATE_OPERATIONAL
ecx_send_processdata(ctx)
ecx_receive_processdata(ctx, timeout)
ecx_writestate(ctx, 0)                       // 组级状态请求
循环（10ms 一次，总超时 5000ms）:
    ecx_statecheck(ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE)
    ecx_readstate(ctx)
    全部 state == EC_STATE_OPERATIONAL → return true
return false
```

**⑤ `Close()` 必须幂等**：`Release` 会在「Init 抛异常」和「正常退出」两条路径都调它。

### 5.3 步 1-3：Kconfig 符号

见 §3.1，插在 `framework/drivers/Kconfig` 的注释头之后。

### 5.4 步 1-4：CMake 装配

```cmake
if(CONFIG_DUST_COM_ETHERCAT)
    find_path(SOEM_INCLUDE_DIR soem/soem.h HINTS /usr/local/include /usr/include)
    find_library(SOEM_LIBRARY NAMES soem HINTS /usr/local/lib /usr/lib)

    if(NOT SOEM_INCLUDE_DIR OR NOT SOEM_LIBRARY)
        message(FATAL_ERROR
            "SOEM not found. Build and install SOEM first:\n"
            "  git clone https://github.com/OpenEtherCATsociety/SOEM.git\n"
            "  cmake -S SOEM -B SOEM/build -DCMAKE_BUILD_TYPE=Release\n"
            "  cmake --build SOEM/build -j && sudo cmake --install SOEM/build")
    endif()

    add_library(framework_ethercat STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/communication/ethercat/ethercat.cpp)
    target_include_directories(framework_ethercat PUBLIC
        ${PROJECT_SOURCE_DIR} ${SOEM_INCLUDE_DIR})
    target_link_libraries(framework_ethercat PUBLIC
        ${SOEM_LIBRARY} Threads::Threads)
    target_compile_options(framework_ethercat PRIVATE -Wall -Wextra -Wpedantic)
endif()
```

**注意 `soem/soem.h` 的实际安装路径**：SOEM 2.0 的头文件在 `include/soem/`，install 后落到 `/usr/local/include/soem/`，`#include "soem/soem.h"` 成立。
本地源码 `temp/dust-control-git/tests/SOEM/SOEM/include/soem/soem.h` 已验证这个相对路径。

**`soem.h` 会 include `nicdrv.h`**（`soem.h:24`），而 `nicdrv.h` 在 `oshw/linux/` 下、install 时**不装**。
若 `find_path` 找到的头文件用不了，退路是**把 SOEM 作为 CMake 子目录一起编**（`add_subdirectory(${SOEM_SOURCE_DIR})`），此时 `$<BUILD_INTERFACE>` 的 include 路径自动带 `oshw/linux`。
**这条列为待拍板 B**（见 §9）——两条路选一条，不要都写。

### 5.5 步 1-5：链接

`project/CMakeLists.txt` 的链接段：

```cmake
if(CONFIG_DUST_COM_ETHERCAT)
    target_link_libraries(dust_app PRIVATE framework_ethercat)
endif()
```

---

## 6. P2：周期线程接入

> **P2 拆成 4 步。**
> 用户说"开始 P2 第 N 步"= 只改表格里那一个文件，改完停下报告，等"继续"。

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| **2-1** | `project/Kconfig` | 加 `TRD_ETHERCAT`（`default n`，`select DUST_COM_ETHERCAT` + `select DUST_CMD_CONFIG`） | 1-3 | 无 |
| **2-2** | `project/thread/ethercat/trd_ethercat.cpp`（新建） | 周期线程 + `REGISTER_INIT`/`REGISTER_THREAD` + `Recover` 策略 | 2-1 | 无 |
| **2-3** | `project/thread/ethercat/trd_ethercat.yaml`（新建） | 网口名、周期、超时、DC 开关 | 2-2 | 无 |
| **2-4** | `project/thread/CMakeLists.txt` | `if(CONFIG_TRD_ETHERCAT)` 装配 + include | 2-2 | 无 |

**编译状态**：2-1 加符号 `default n` → 可编。2-2、2-3 是新增文件，2-4 装配前不进构建 → 可编。**2-4 之后开 `TRD_ETHERCAT=y` 才编进去**，此时需 P1 全绿 + SOEM 已装。

### 6.1 步 2-1：`project/Kconfig`

```kconfig
# =================              TRD_ETHERCAT              ====================

config TRD_ETHERCAT
    bool "EtherCAT cyclic thread"
    default n
    select DUST_COM_ETHERCAT
    select DUST_CMD_CONFIG
    help
      EtherCAT 主站周期线程：每拍发一帧、收一帧，校验工作计数器。
      需要系统已安装 SOEM，且从站接在主站网口的 PORTA。
```

### 6.2 步 2-2：`trd_ethercat.cpp`

**周期用绝对时刻，不用外壳 `period`**：

外壳 `Thread::Start(..., period, ...)` 的语义是「任务结束后再等 N」（`thread.hpp` 的 `WaitForStop(task_.period)`），1 ms 硬周期会累积漂移。
所以 `period` 传 `0`，节拍在 `Task` 内部用 `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` 自己算。

```cpp
/**
 * @brief 任务单拍：发一帧、收一帧、按 WKC 判定链路健康
 *
 * 周期等待由任务体自己用绝对时刻完成：外壳的 period 语义是「任务结束后延时」，
 * 会随拍数累积漂移，不适合 1 ms 硬周期。
 *
 * @param context 未使用
 * @return Continue=正常；Retry=单拍失败未超限；Exit=连续失败达上限
 */
static init::Thread::TaskResult Task(void* context)
{
    (void)context;

    static timespec next {};
    static bool     armed = false;
    if (!armed) {
        clock_gettime(CLOCK_MONOTONIC, &next);
        armed = true;
    }

    master_.Send();

    if (master_.Receive()) {
        fail_count_ = 0;
    } else {
        ++fail_count_;
        if (fail_count_ >= kFailLimit) {
            DUST_LOG_ERR("ethercat: %u consecutive WKC failures", fail_count_);
            return init::Thread::TaskResult::Exit;
        }
        return init::Thread::TaskResult::Retry;
    }

    next.tv_nsec += static_cast<long>(cycle_us_) * 1000L;
    while (next.tv_nsec >= 1000000000L) {
        next.tv_nsec -= 1000000000L;
        next.tv_sec  += 1;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);

    return init::Thread::TaskResult::Continue;
}
```

**故障策略 = `Recover`**：总线掉了所有挂在它上面的轴全部失控，只停自己没意义——与 `doc/子架构部署与父架构功能补齐规划.md` §12 B 条同一判断。

```cpp
    thread_.Start(Task, std::chrono::milliseconds(0), nullptr, "ethercat",
                  init::ThreadFaultConfig{nullptr, nullptr, ThreadFaultPolicy::Recover, false});
```

**线程三件套 + 注册**（照 `project/thread/test/trd_test.cpp` 的现行形状）：

```cpp
bool thread_init();      // 读 YAML → master_.Init → 打从站数
bool thread_start();     // RequestOperational → thread_.Start
void thread_stop();      // thread_.RequestStop()
void thread_join();      // thread_.Join() → master_.Close()

REGISTER_INIT(thread_init, MidInit, High, HaltOnFail, Release, "ethercat_init");
REGISTER_THREAD(thread_start, MidThread, thread_stop, thread_join, "ethercat_start");
```

**`Release` 要支持未初始化与重复调用**（`REGISTER_INIT` 的 cleanup 在部分初始化失败时也会被调）。

### 6.3 步 2-3：`trd_ethercat.yaml`

```yaml
# EtherCAT 主站网口名；该网口不配 IP，专供总线。
interface: enp2s0

# 通信周期，单位 µs；整数，范围 [100, 100000]。
cycle_us: 1000

# 单次过程数据接收超时，单位 µs；整数，范围 [100, 100000]。
timeout_us: 2000

# 是否启用分布式时钟同步；从站支持 DC 且需要多轴同步时打开。
use_dc: false
```

**注意**：本工程当前**没有任何 YAML 文件**，也没有 CMake 拷贝规则。
`Config::Load` 的路径解析是「`DUST_CONFIG_ROOT` → 否则 `/proc/self/exe` 所在目录」。
所以运行时要 `export DUST_CONFIG_ROOT=$PWD/project`，否则找不到 `thread/ethercat/trd_ethercat.yaml`。
**这条列为待拍板 C**（见 §9）：是否加 CMake 拷贝规则。

### 6.4 步 2-4：装配

```cmake
if(CONFIG_TRD_ETHERCAT)
    target_sources(dust_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ethercat/trd_ethercat.cpp)
    target_include_directories(dust_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ethercat)
endif()
```

---

## 7. P3：环境与验证

> **P3 拆成 3 步。** 这三步是**用户动作**，AI 只给命令、不代跑（编译/烧录/装包一律用户执行）。

| 步 | 对象 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| **3-1** | SOEM | clone → cmake → install，`which slaveinfo` 有输出 | P1 全 | 无 |
| **3-2** | 权限 | 运行用 `sudo`，或 `setcap cap_net_raw,cap_net_admin+eip` | 3-1 | 无 |
| **3-3** | 实测 | 从站接 PORTA → 启动程序 → 核对 `00 01 ... 1F` | 3-2、P2 全 | 无 |

### 7.1 步 3-1：装 SOEM

```bash
git clone https://github.com/OpenEtherCATsociety/SOEM.git ~/src/SOEM
cmake -S ~/src/SOEM -B ~/src/SOEM/build -DCMAKE_BUILD_TYPE=Release -DSOEM_BUILD_SAMPLES=ON
cmake --build ~/src/SOEM/build -j"$(nproc)"
sudo cmake --install ~/src/SOEM/build
which slaveinfo        # 期望 /usr/local/bin/slaveinfo
```

**不需要 `libpcap-dev`**：SOEM Linux 后端走 `AF_PACKET`（`oshw/linux/nicdrv.c:46` include `<netpacket/packet.h>`），不依赖 pcap。
（旧规划里写的 `libpcap-dev` 是冗余的，这里更正。）

### 7.2 步 3-2：网口与权限

```bash
sudo ip link set enp2s0 up      # 不配 IP
sudo ./build/project/dust_app # 需要 CAP_NET_RAW
```

免 sudo 的替代：`sudo setcap cap_net_raw,cap_net_admin+eip ./build/project/dust_app`。

### 7.3 步 3-3：判据

| 检查点 | 期望 |
|---|---|
| 启动日志 | `ethercat: 1 slaves on enp2s0` |
| `thread list` | 有 `ethercat` 行，`run_count` 每秒涨 ~1000 |
| 读 TxPDO | 32 字节序列 `00 01 02 ... 1F` |
| 写 RxPDO | 从站串口打印 `ecat rx 32 bytes` |
| 拔网线 | 10 拍后 `ethercat: 10 consecutive WKC failures` → `Recover` 处置 |

---

## 8. 平台映射与不搬清单

### 8.1 父架构 → 本工程

| 父架构（Zephyr） | 本工程（Linux） | 理由 |
|---|---|---|
| `target_sources(app ...)` | 静态库 + `target_link_libraries` | 本工程无 app target 直挂模式 |
| `k_sem sem_`（Stream 基类） | 无（EtherCAT 不用） | 周期整帧交换没有接收事件 |
| `k_msleep(period)` | `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` | 硬周期不能累积漂移 |
| `ThreadPrio` 枚举 | 不搬 | 本工程 `Start()` 无优先级参数，优先级靠 `pthread_setschedparam`（P3 可选） |
| `k_thread_abort` | 不搬 | pthread 不能强杀；本工程线程是协作式退出 |
| `printk` | `DUST_LOG_*` | 项目铁律 |
| `DEVICE_DT_GET` / devicetree | 网口名字符串 | Linux 没有 devicetree |

### 8.2 明确不搬（写死，别后来又想加）

| 项 | 为什么不搬 |
|---|---|
| `Stream` 抽象 | 本工程没有实现者，空壳无验证价值（§2.2） |
| CiA402 状态机（`modules/ethercat_slave/`） | 当前从站是 `digital_io`（8×UDINT 计数器），不是伺服；等真接伺服再建 `modules/` |
| IgH 传输后端 | 本机 `igc` 网卡 IgH 不支持（旧规划已定案） |
| 多组映射 `ecx_config_overlap_map_group` | 单从站单组，`group=0` 够用 |
| `ecx_SDOread` / CoE 参数读写 | 当前只做过程数据；SDO 等有需求再说 |
| `setcap` 自动化进 CMake | 权限是用户环境，AI 不碰 |

---

## 9. 待拍板（5 条，已给推荐值）

| # | 问题 | 推荐 | 影响 |
|---|---|---|---|
| **A** | P0 是否现在就立 `communication/stream/stream.hpp` 抽象 | **不立**，等第二个串行实现者出现再抽 | 少一个空壳；将来加 UART 时多一步 |
| **B** | SOEM 依赖走「系统安装 + `find_path`」还是「CMake 子目录一起编」 | **系统安装**（`find_path`）；若 `nicdrv.h` 找不到再退回子目录 | 前者干净，后者不受 install 布局影响 |
| **C** | YAML 是否加 CMake 拷贝规则到可执行文件旁 | **加**（照旧规划「YAML 已由构建复制到可执行文件旁」的既定形态） | 不加就得每次 `export DUST_CONFIG_ROOT` |
| **D** | `TRD_ETHERCAT` 默认值 | **`n`** | 默认 `y` 会让没接从站的机器启动即 `HaltOnFail` |
| **E** | 线程 `MidInit/MidThread` 还是 `LateInit/LateThread` | **`Mid`** | EtherCAT 要在业务线程之前就绪，但晚于 shell 起来（故障要能打日志） |

---

## 10. 风险与回退

| 风险 | 表现 | 回退 |
|---|---|---|
| SOEM 未装 | 配置阶段 `FATAL_ERROR: SOEM not found` | 按 §7.1 装，或把 `CONFIG_DUST_COM_ETHERCAT=n` |
| `nicdrv.h` 找不到 | 编译期 `fatal error: nicdrv.h: No such file` | 改走待拍板 B 的子目录方案 |
| 网卡不支持/非 100M | `ecx_config_init` 返回 0 | 确认网线在 PORTA、网卡 100M 全双工 |
| 从站进不了 OP | `RequestOperational` 返回 false | 用 `slaveinfo enp2s0` 单独扫，核对 PDO 与 §1.2 一致 |
| WKC 一直不对 | 每拍 `Retry`，10 拍后 `Exit` | 查 PORTB 是否空着（正常）、SM 配置 |
| 周期抖动大 | `run_ns` 波动 | P3 可选：`isolcpus` + `SCHED_FIFO`（本次不做） |

---

## 11. 执行顺序总览

```text
P0（4 步，全程可编）  →  P1（5 步，1-5 后开符号才需 SOEM）
                       →  P2（4 步，2-4 后开 TRD_ETHERCAT 才编）
                       →  P3（3 步，用户执行）
```

**任何一步做完就停，报告改了什么、还剩什么，等"继续"。**
