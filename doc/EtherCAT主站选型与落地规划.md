# EtherCAT 主站选型与落地规划

> 2026-10-28。
>
> 目标机器：`rmconstruction-NUC10i7FNH`（Ubuntu 22.04.5，内核 6.8.0-138-generic，i7-12650H，16 逻辑核）
> 目标：让这台机器成为 EtherCAT 主站，跑周期通信；代码最终接入 `qingyu` 分层架构。

---

## 0. 结论先行

**走 SOEM，不走 IgH。** 唯一原因：这台机器的两块网卡是 **Intel I225-LM（`8086:125c`），驱动 `igc`**，而 IgH EtherLab 的原生驱动列表里**没有 `igc`**。

IgH 只支持这些网卡的原生驱动（`devices/` 目录）：`e1000` / `e1000e` / `r8169` / `8139too` / `r8152` / `igb` / `ccat` / `genet` / `lan9252` / `ax58100` 等，外加 `generic`（走内核网络栈，抖动大）。i225 不在其中。

| | SOEM | IgH |
|---|---|---|
| i225 / `igc` | ✅ 用户态 AF_PACKET，任何网卡 | ❌ 只能 `ec_generic` 或自写内核模块 |
| 安装成本 | `cmake` 一编 | 编内核模块 + 绑驱动 + 内核版本绑定 |
| 1 ms 周期 | ✅ 够用 | ✅ 更好 |
| 分布式时钟 DC | 有，但同步精度弱 | **强项** |
| 从站枚举调试 | `slaveinfo` 一条命令 | `ethercat slaves` + `dmesg` |
| 与现有工程 | 纯 C 库，直接链 | `ecrt.h`，也是纯 C 库 |

**换 IgH 的触发条件**（写死，不提前纠结）：① 需要多轴同步插补、DC 亚微秒对齐；② SOEM 实测抖动超过控制周期容限的 10%。

---

## 1. 机器实测基线

| 项 | 实测值 | 判断 |
|---|---|---|
| 系统 | Ubuntu 22.04.5 LTS | ✅ |
| 内核 | 6.8.0-138-generic | ✅ |
| 抢占模型 | `CONFIG_PREEMPT_DYNAMIC=y` + `CONFIG_PREEMPTION=y` | ✅ **不用装 RT 内核**，`preempt=full` 启动参数即可 |
| 时钟 | `CONFIG_HZ=1000`、`CONFIG_NO_HZ_FULL=y`、`CONFIG_HIGH_RES_TIMERS=y` | ✅ 1 ms 周期的时钟源足够 |
| CPU | i7-12650H，16 逻辑核 | ✅ 可隔离 4~6 核 |
| 网卡 | 2× Intel I225-LM `8086:125c`，驱动 `igc` | ⚠️ IgH 不支持，SOEM 支持 |
| 网卡状态 | `enp2s0` / `enp3s0` 均 DOWN、NO-CARRIER | 可用，需 `ip link set up` |
| 实时权限 | 无 `realtime` 组，`ulimit -r = 0` | ❌ 必须配，否则 `SCHED_FIFO` 失败 |
| 内核头文件 | `/usr/src/linux-headers-6.8.0-138-generic` | ✅ 已装 |
| 已装 | ROS 2 Humble、gcc/g++/cmake/make/git | ✅ |
| 缺 | `ethtool`、`libpcap-dev`、`rt-tests`、任何 EtherCAT 栈 | 待装 |
| 缺（硬件） | **EtherCAT 从站实物** | 没有从站，`slaveinfo` 只能报 0 个 |

---

## 2. 分层落点（按 `qingyu` 现有分层）

```text
framework/drivers/communication/ethercat/
    ethercat_master.hpp/.cpp      ← SOEM 封装：init / config / 周期收发 / 状态查询
    pdo.hpp                       ← PDO 偏移与类型化读写（按从站生成）

framework/modules/ethercat_slave/
    cia402.hpp/.cpp               ← CiA402 状态机（伺服驱动器的标准协议）
    slave_config.hpp              ← 从站配置（厂商/型号/PDO 布局）

project/thread/ethercat/
    trd_ethercat.cpp              ← 周期线程：ec_send/receive + 看门狗 + 故障处置
    trd_ethercat.yaml             ← 网口名、周期、从站列表、超时
```

**依赖方向**：`project/thread` → `framework/modules` → `framework/drivers`，与现有分层一致。

**Kconfig 符号**（照架构层命名，不许自创前缀）：

```kconfig
# framework/drivers/Kconfig
config DUST_COM_ETHERCAT
    bool "EtherCAT master (SOEM)"
    default n
    help
      用户态 EtherCAT 主站：周期收发过程数据、从站状态机。

# framework/modules/Kconfig
config DUST_MOD_DEV_CIA402
    bool "CiA402 servo drive state machine"
    default n
    select DUST_COM_ETHERCAT

# project/Kconfig
config TRD_ETHERCAT
    bool "EtherCAT cyclic thread"
    default n
    select DUST_MOD_DEV_CIA402
```

**线程故障策略 = `Recover`**（按 `doc/子架构部署与父架构功能补齐规划.md` §12 B 条的同一判断逻辑）：EtherCAT 总线掉了，所有挂在它上面的轴全部失控，只停自己没意义。

---

## 3. 环境准备（P0）

> **P0 拆成 6 步。**
> 用户说"开始 P0 第 N 步"= 只做表格里那一件事，做完停下报告，等"继续"。

| 步 | 对象 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| **0-1** | apt | `sudo apt install -y libpcap-dev ethtool rt-tests` | 无 | 无 |
| **0-2** | 内核参数 | `/etc/default/grub` 的 `GRUB_CMDLINE_LINUX_DEFAULT` 追加 `preempt=full`，`update-grub` 后**重启** | 0-1 | 重启后所有进程短暂中断 |
| **0-3** | 实时权限 | 建 `realtime` 组、加用户、`limits.d` 写 `rtprio 99` + `memlock unlimited`，**重新登录** | 0-1 | 无 |
| **0-4** | 网口 | `sudo ip link set enp2s0 up`；**不配 IP** | 0-1 | 无 |
| **0-5** | 实时性实测 | `cyclictest -m -Sp90 -i1000 -h100 -p95 -t4`，记录 Max | 0-2、0-3 | 无 |
| **0-6** | SOEM | clone → `cmake -DCMAKE_BUILD_TYPE=Release` → `make -j` → `sudo make install` | 0-1 | 无 |

### 3.1 步 0-2 的具体内容

```bash
sudo cp /etc/default/grub /etc/default/grub.bak
# 编辑 GRUB_CMDLINE_LINUX_DEFAULT，追加 preempt=full
#   改前：GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"
#   改后：GRUB_CMDLINE_LINUX_DEFAULT="quiet splash preempt=full"
sudo update-grub
sudo reboot
```

**为什么不用装 `linux-image-lowlatency`**：当前内核已 `CONFIG_PREEMPT_DYNAMIC=y`，`preempt=full` 就是把它切到 full preempt，**同一个内核、零安装成本**。Ubuntu 官方 `linux-lowlatency-hwe-22.04` 也能装（候选 6.8.0-146），但那是第二套内核，会多一份维护面。

**验证**：

```bash
cat /sys/kernel/debug/sched/preempt    # 需 root，应显示 full
```

### 3.2 步 0-3 的具体内容

```bash
sudo groupadd -f realtime
sudo usermod -aG realtime $USER

sudo tee /etc/security/limits.d/99-realtime.conf > /dev/null <<'EOF'
@realtime - rtprio 99
@realtime - memlock unlimited
@realtime - nice -20
EOF
```

**必须重新登录**（`newgrp realtime` 只对当前 shell 生效，`pam_limits` 在登录时读）。

**验证**：`ulimit -r` 应显示 `99`（当前是 `0`）。

### 3.3 步 0-5：抖动量级判据

```bash
sudo cyclictest -m -Sp90 -i1000 -h100 -p95 -t4 -a 2-5
```

| Max 抖动 | 结论 |
|---|---|
| < 50 µs | 优秀，1 ms 周期毫无压力 |
| 50 ~ 100 µs | 可用，1 ms 周期稳定 |
| 100 ~ 250 µs | 只能跑 2 ms 以上周期 |
| > 250 µs | 先查 CPU 隔离与 `preempt=full` 是否生效，再考虑 RT 内核 |

**这是唯一该不该上 RT 内核的判据，测完再决定。**

### 3.4 步 0-6：SOEM 编译

```bash
git clone https://github.com/OpenEtherCATsociety/SOEM.git ~/src/SOEM
cd ~/src/SOEM && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

产物：`/usr/local/lib/libsoem.so`、`/usr/local/include/soem/`、`/usr/local/bin/slaveinfo`。

---

## 4. 从站枚举验证（P1）

> **P1 拆成 3 步。**

| 步 | 对象 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| **1-1** | 物理连接 | 主站 `enp2s0` → 从站 IN，从站 OUT → 下一个从站 IN（菊花链） | P0 全部 | 无 |
| **1-2** | 枚举 | `sudo slaveinfo enp2s0` | 1-1 | 无 |
| **1-3** | 记录 PDO 布局 | 把每个从站的 `RxPDO` / `TxPDO` 索引、子索引、位宽抄进 `slave_config.hpp` | 1-2 | 无 |

### 4.1 步 1-2 的期望输出

```text
slaveinfo enp2s0
Starting slaveinfo
ec_init on enp2s0 succeeded.
2 slaves found and configured.
Slave:1
  State:   OP
  Name:    <驱动器型号>
  Output size: 32 bits
  Input size:  96 bits
  ...
```

**这一步不过，后面全部白搭。** 常见失败：

| 现象 | 原因 |
|---|---|
| `0 slaves found` | 网线没接、从站没上电、接的是 OUT 口不是 IN 口 |
| `ec_init failed` | 网口没 `up`，或没有 `CAP_NET_RAW`（用 `sudo`） |
| 从站数不对 | 菊花链中间断了一节 |

### 4.2 步 1-3：PDO 记录格式

每个从站记成一张表，直接能抄进代码：

| 从站 | 方向 | 索引 | 子索引 | 名称 | 位宽 | IOmap 偏移 |
|---|---|---|---|---|---|---|
| 1 | RxPDO（主→从） | `0x6040` | 0 | Controlword | 16 | 待填 |
| 1 | RxPDO | `0x607A` | 0 | Target position | 32 | 待填 |
| 1 | TxPDO（从→主） | `0x6041` | 0 | Statusword | 16 | 待填 |
| 1 | TxPDO | `0x6064` | 0 | Actual position | 32 | 待填 |

**偏移量不用手算**：SOEM 的 `ec_slave[i].outputs` / `.inputs` 就是指向 IOmap 里该从站首字节的指针，配置完 `ec_config_map` 之后才有效。

---

## 5. 主站核心实现（P2）

> **P2 拆成 8 步。**
> 用户说"开始 P2 第 N 步"= 只改表格里那一个文件，改完停下报告，等"继续"。

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| **2-1** | `framework/drivers/Kconfig` | 加 `DUST_COM_ETHERCAT` | 无 | 无 |
| **2-2** | `framework/drivers/communication/ethercat/ethercat_master.hpp`（新建） | 类声明 + `Config` 结构 | 2-1 | 无 |
| **2-3** | `framework/drivers/communication/ethercat/ethercat_master.cpp`（新建） | `Init` / `Close` / `SendProcessData` / `ReceiveProcessData` / `SlaveCount` / `IsOperational` | 2-2 | 无 |
| **2-4** | `framework/drivers/CMakeLists.txt` | `if(CONFIG_DUST_COM_ETHERCAT)` 块 + `find_library(soem)` | 2-3 | 无 |
| **2-5** | `framework/modules/Kconfig` | 加 `DUST_MOD_DEV_CIA402` | 2-1 | 无 |
| **2-6** | `framework/modules/ethercat_slave/cia402.hpp`（新建） | 状态机枚举 + `Controlword` / `Statusword` 位定义 + 状态转换函数 | 2-5 | 无 |
| **2-7** | `project/Kconfig` | 加 `TRD_ETHERCAT` | 2-5 | 无 |
| **2-8** | `project/thread/ethercat/trd_ethercat.cpp`（新建） | 周期线程 + `REGISTER_INIT` / `REGISTER_THREAD` + `Recover` 策略 | 2-4、2-7 | 无 |

**编译状态**：步 2-1~2-7 全程可编（新增符号默认 `n`，不进构建）；步 2-8 之后需要 `CONFIG_TRD_ETHERCAT=y` 才编进去。

### 5.1 步 2-2：`ethercat_master.hpp`

```cpp
/**
 * @file ethercat_master.hpp
 * @author qingyu
 * @brief EtherCAT 主站（SOEM）—— 初始化、周期收发与从站状态查询
 * @version 0.1
 * @date 2026-10-28
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
 */
struct Config
{
    std::string interface_name;        // 网口名（enp2s0），不配 IP
    uint32_t    cycle_us      = 1000;  // 通信周期，单位 µs
    uint32_t    timeout_us    = 2000;  // 单次过程数据超时，单位 µs（EC_TIMEOUTRET 量级）
    bool        use_dc        = false; // 是否启用分布式时钟同步
};

/**
 * @brief 从站运行状态快照
 */
struct SlaveState
{
    uint16_t index;         // 从站序号（1 起）
    uint16_t al_state;      // AL 状态（EC_STATE_*）
    uint16_t al_status;     // AL 状态码（错误时非 0）
    bool     operational;   // 是否在 OP
};

/**
 * @brief EtherCAT 主站，基于 SOEM
 *
 * 单实例、单网口。Init 成功后由上层线程每周期调用 Send/Receive。
 * 本类不做周期等待，节拍由调用方决定。
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
     *
     * @param config 主站配置
     * @return true 成功且至少一个从站
     * @throws std::runtime_error 网口打开失败、无从站、映射失败
     */
    bool Init(const Config& config);

    /**
     * @brief 关闭主站并释放 SOEM 上下文
     */
    void Close() noexcept;

    /**
     * @brief 发送一帧过程数据
     */
    void SendProcessData();

    /**
     * @brief 接收一帧过程数据
     * @return true 工作计数器全部正确
     */
    bool ReceiveProcessData();

    /**
     * @brief 把所有从站切到 OP 并等待就绪
     * @return true 全部进入 OP
     */
    bool RequestOperational();

    /**
     * @brief 从站数量
     */
    uint16_t SlaveCount() const;

    /**
     * @brief 取某个从站的运行状态
     * @param index 从站序号（1 起）
     */
    SlaveState GetSlaveState(uint16_t index) const;

    /**
     * @brief 取某个从站的输出缓冲基址（RxPDO，主→从）
     * @param index 从站序号（1 起）
     * @return 指向 IOmap 内该从站输出区的指针；越界返回 nullptr
     */
    uint8_t* Outputs(uint16_t index) const;

    /**
     * @brief 取某个从站的输入缓冲基址（TxPDO，从→主）
     * @param index 从站序号（1 起）
     * @return 指向 IOmap 内该从站输入区的指针；越界返回 nullptr
     */
    const uint8_t* Inputs(uint16_t index) const;

    /**
     * @brief 期望工作计数器（每个从站输入/输出各一个）
     */
    uint16_t ExpectedWorkingCounter() const;

private:
    Config          config_ {};             // 主站配置
    void*           context_   = nullptr;   // SOEM ecx_context（避免头文件暴露 SOEM）
    std::vector<uint8_t> io_map_;           // 过程数据映射缓冲
    uint16_t        slave_count_ = 0;       // 从站数量
    bool            initialized_ = false;   // 是否已初始化
};

} // namespace drivers::ethercat
```

**为什么 `context_` 用 `void*`**：`ethercat_master.hpp` 不 include SOEM 头文件，把 SOEM 完全锁在 `.cpp` 里。这样上层（modules / project）编译时不依赖 SOEM 的 include 路径，换栈（比如以后换 IgH）只改 `.cpp`。

### 5.2 步 2-3：`ethercat_master.cpp` 关键实现

```cpp
#include "ethercat_master.hpp"

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>

#include "soem/soem.h"

namespace drivers::ethercat
{

namespace
{
    constexpr int kTimeoutRet = 2000;    // 接收超时，单位 µs
}

Master::~Master()
{
    Close();
}

/**
 * @brief 打开网口、枚举从站、映射过程数据
 */
bool Master::Init(const Config& config)
{
    if (initialized_) {
        throw std::logic_error("ethercat master already initialized");
    }
    if (config.interface_name.empty()) {
        throw std::invalid_argument("ethercat interface name is empty");
    }

    config_ = config;

    auto* ctx = new ecx_contextt{};
    context_ = ctx;

    if (!ecx_init(ctx, config.interface_name.c_str())) {
        Close();
        throw std::runtime_error("ecx_init failed on " + config.interface_name);
    }

    // FALSE = 不用配置文件，直接扫链
    if (ecx_config_init(ctx) <= 0) {
        Close();
        throw std::runtime_error("no EtherCAT slave found on " + config.interface_name);
    }
    slave_count_ = ctx->slavecount;

    // 按实际 IO 大小分配映射缓冲（SOEM 要求缓冲不小于各组 IO 字节数）
    const uint32_t io_bytes = static_cast<uint32_t>(ctx->slavecount) * 64U;  // 保守估计，见下方说明
    io_map_.assign(io_bytes, 0);

    if (ecx_config_map_group(ctx, io_map_.data(), 0) <= 0) {
        Close();
        throw std::runtime_error("ecx_config_map_group failed");
    }

    if (config.use_dc) {
        ecx_configdc(ctx);
    }

    initialized_ = true;
    return true;
}
```

**`io_map_` 大小**：SOEM 的 `ecx_config_map_group` 要求调用方给的缓冲**不小于**所有从站 IO 位总和。上面的 `slavecount * 64` 是粗估，**正式版必须在 `ecx_config_init` 之后按真实位宽算**：

```cpp
uint32_t bits = 0;
for (uint16_t i = 1; i <= slave_count_; ++i) {
    bits += ctx->slave[i].Obits + ctx->slave[i].Ibits;
}
const uint32_t io_bytes = (bits + 7U) / 8U + 64U;   // 尾部留余量
```

**这是最容易踩的坑**：缓冲给小了，`ecx_config_map_group` 会返回 0 或者写越界。

```cpp
/**
 * @brief 接收一帧过程数据
 */
bool Master::ReceiveProcessData()
{
    if (!initialized_) return false;

    auto* ctx = static_cast<ecx_contextt*>(context_);
    const int wkc = ecx_receive_processdata(ctx, kTimeoutRet);

    // 期望工作计数器：每个从站输入、输出各贡献 1
    return wkc >= static_cast<int>(ExpectedWorkingCounter());
}

/**
 * @brief 把所有从站切到 OP
 */
bool Master::RequestOperational()
{
    auto* ctx = static_cast<ecx_contextt*>(context_);

    // 先全部请求 OP
    for (uint16_t i = 1; i <= slave_count_; ++i) {
        ctx->slave[i].state = EC_STATE_OPERATIONAL;
    }
    ecx_send_processdata(ctx);
    ecx_receive_processdata(ctx, kTimeoutRet);

    // 组级状态请求
    ecx_writestate(ctx, 0);

    // 轮询直到 OP 或超时
    constexpr int kPollMs      = 10;
    constexpr int kTimeoutMs   = 5000;
    for (int waited = 0; waited < kTimeoutMs; waited += kPollMs)
    {
        ecx_statecheck(ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
        ecx_readstate(ctx);

        bool all_op = true;
        for (uint16_t i = 1; i <= slave_count_; ++i) {
            if (ctx->slave[i].state != EC_STATE_OPERATIONAL) { all_op = false; break; }
        }
        if (all_op) return true;

        std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
    }
    return false;
}
```

**`ecx_statecheck` 的第三个参数**是超时（µs 或 `EC_TIMEOUTSTATE`），它内部会轮询；外面再套一层是为了拿总超时并打印是哪个从站没起来。

```cpp
/**
 * @brief 取从站输出/输入缓冲
 */
uint8_t* Master::Outputs(uint16_t index) const
{
    if (!initialized_ || index == 0 || index > slave_count_) return nullptr;
    return static_cast<ecx_contextt*>(context_)->slave[index].outputs;
}

const uint8_t* Master::Inputs(uint16_t index) const
{
    if (!initialized_ || index == 0 || index > slave_count_) return nullptr;
    return static_cast<ecx_contextt*>(context_)->slave[index].inputs;
}

/**
 * @brief 关闭主站
 */
void Master::Close() noexcept
{
    if (context_ != nullptr) {
        auto* ctx = static_cast<ecx_contextt*>(context_);
        if (initialized_) {
            ecx_close(ctx);
        }
        delete ctx;
        context_ = nullptr;
    }
    io_map_.clear();
    slave_count_ = 0;
    initialized_ = false;
}
```

### 5.3 步 2-4：CMake 接入

`framework/drivers/CMakeLists.txt` 追加：

```cmake
# ============================================================================
#
# CONFIG_DUST_COM_ETHERCAT：用户态 EtherCAT 主站（SOEM）
#
# ============================================================================

if(CONFIG_DUST_COM_ETHERCAT)
    find_path(SOEM_INCLUDE_DIR soem/soem.h HINTS /usr/local/include /usr/include)
    find_library(SOEM_LIBRARY NAMES soem HINTS /usr/local/lib /usr/lib)
    if(NOT SOEM_INCLUDE_DIR OR NOT SOEM_LIBRARY)
        message(FATAL_ERROR "SOEM not found: build and install SOEM first")
    endif()

    add_library(framework_ethercat STATIC communication/ethercat/ethercat_master.cpp)
    target_include_directories(framework_ethercat PUBLIC ${PROJECT_SOURCE_DIR} ${SOEM_INCLUDE_DIR})
    target_link_libraries(framework_ethercat PUBLIC ${SOEM_LIBRARY} pcap)
    target_compile_options(framework_ethercat PRIVATE -Wall -Wextra -Wpedantic)
endif()
```

`project/CMakeLists.txt` 的链接段追加：

```cmake
if(CONFIG_DUST_COM_ETHERCAT)
    target_link_libraries(vision_app PRIVATE framework_ethercat)
endif()
```

**注意**：SOEM 需要 `CAP_NET_RAW`。运行方式二选一：

```bash
sudo ./build/project/vision_app                    # 简单
sudo setcap cap_net_raw,cap_net_admin+eip ./build/project/vision_app   # 免 sudo
```

### 5.4 步 2-6：CiA402 状态机

CiA402 是伺服驱动器的标准协议，状态字 `0x6041` 和控制字 `0x6040` 的位定义是固定的。

```cpp
/**
 * @file cia402.hpp
 * @author qingyu
 * @brief CiA402 伺服驱动器状态机 —— 控制字/状态字位定义与状态转换
 * @version 0.1
 * @date 2026-10-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>

namespace modules::cia402
{

/**
 * @brief CiA402 状态机状态（由状态字解码）
 */
enum class State : uint8_t
{
    NotReadyToSwitchOn = 0,
    SwitchOnDisabled   = 1,
    ReadyToSwitchOn    = 2,
    SwitchedOn         = 3,
    OperationEnabled   = 4,
    QuickStopActive    = 5,
    FaultReactionActive= 6,
    Fault              = 7,
};

/**
 * @brief 控制字命令（写入 0x6040）
 */
namespace cmd
{
    constexpr uint16_t kShutdown          = 0x0006;   // → SwitchOnDisabled → ReadyToSwitchOn
    constexpr uint16_t kSwitchOn          = 0x0007;   // → SwitchedOn
    constexpr uint16_t kEnableOperation   = 0x000F;   // → OperationEnabled
    constexpr uint16_t kDisableVoltage    = 0x0000;
    constexpr uint16_t kQuickStop         = 0x0002;
    constexpr uint16_t kFaultReset        = 0x0080;   // bit7 上升沿
}

/**
 * @brief 从状态字解码状态机状态（CiA402 §6.1 掩码表）
 *
 * @param statusword 状态字（0x6041）
 * @return 解码出的状态
 */
inline State DecodeState(uint16_t statusword)
{
    // bit0 Ready / bit1 SwitchedOn / bit2 OperationEnabled / bit3 Fault / bit5 QuickStop / bit6 SwitchOnDisabled
    if ((statusword & 0x004F) == 0x0000) return State::NotReadyToSwitchOn;
    if ((statusword & 0x004F) == 0x0040) return State::SwitchOnDisabled;
    if ((statusword & 0x006F) == 0x0021) return State::ReadyToSwitchOn;
    if ((statusword & 0x006F) == 0x0023) return State::SwitchedOn;
    if ((statusword & 0x006F) == 0x0027) return State::OperationEnabled;
    if ((statusword & 0x006F) == 0x0007) return State::QuickStopActive;
    if ((statusword & 0x004F) == 0x000F) return State::FaultReactionActive;
    if ((statusword & 0x004F) == 0x0008) return State::Fault;
    return State::NotReadyToSwitchOn;
}

/**
 * @brief 判断是否处于故障态
 */
inline bool IsFault(uint16_t statusword)
{
    return DecodeState(statusword) == State::Fault;
}

} // namespace modules::cia402
```

**掩码表来源**：CiA 402-2 §6.1。`& 0x004F` 取 bit0/1/2/3/6，`& 0x006F` 再含 bit5（QuickStop）。这是标准表，不是自创。

### 5.5 步 2-8：周期线程

```cpp
/**
 * @file trd_ethercat.cpp
 * @author qingyu
 * @brief EtherCAT 周期线程 —— 过程数据收发、从站状态看护与故障上报
 * @version 0.1
 * @date 2026-10-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "framework/cmd/config/config.hpp"
#include "framework/cmd/shell/log/log.hpp"
#include "framework/drivers/communication/ethercat/ethercat_master.hpp"
#include "framework/init/startup/Init_entry.hpp"
#include "framework/init/thread/thread.hpp"
#include <chrono>
#include <ctime>
#include <stdexcept>

namespace thread::ethercat
{

namespace
{
    drivers::ethercat::Master master_ {};                     // 主站实例
    init::Thread               thread_ {};                    // 周期线程
    uint32_t                   cycle_us_ = 1000;              // 周期，单位 µs
    uint32_t                   consecutive_fail_ = 0;         // 连续接收失败计数
    constexpr uint32_t         kFailLimit = 10;               // 连续失败上限（10 个周期）
}

/**
 * @brief 周期任务：发一帧、收一帧、检查工作计数器
 *
 * 周期等待由 Task 自己用 clock_nanosleep 绝对时间完成（外壳的 period 语义是
 * 「任务结束后延时」，会累积漂移，不适合 1 ms 硬周期）。
 *
 * @param context 未使用
 * @return Continue=正常；Retry=接收失败但未超限；Exit=总线失效
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

    master_.SendProcessData();

    if (master_.ReceiveProcessData())
    {
        consecutive_fail_ = 0;
    }
    else
    {
        ++consecutive_fail_;
        if (consecutive_fail_ >= kFailLimit) {
            DUST_LOG_ERR("ethercat: %u consecutive WKC failures", consecutive_fail_);
            return init::Thread::TaskResult::Exit;
        }
        return init::Thread::TaskResult::Retry;
    }

    // 下一拍绝对时刻 = 本拍 + 周期（不累积漂移）
    next.tv_nsec += static_cast<long>(cycle_us_) * 1000L;
    while (next.tv_nsec >= 1000000000L) {
        next.tv_nsec -= 1000000000L;
        next.tv_sec  += 1;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);

    return init::Thread::TaskResult::Continue;
}

/**
 * @brief 读取 YAML 并初始化主站
 * @return true 初始化成功
 */
static bool thread_init()
{
    cmd::config::Config yaml;
    yaml.Load("thread/ethercat/trd_ethercat.yaml", {"interface", "cycle_us"}, {"timeout_us", "use_dc"});

    drivers::ethercat::Config config;
    config.interface_name = yaml.GetString("interface");
    config.cycle_us       = static_cast<uint32_t>(yaml.GetInt("cycle_us", 100, 100000));
    config.timeout_us     = static_cast<uint32_t>(yaml.GetInt("timeout_us", 100, 100000));
    config.use_dc         = yaml.Has("use_dc") && yaml.GetBool("use_dc");

    cycle_us_ = config.cycle_us;

    if (!master_.Init(config)) {
        return false;
    }

    DUST_LOG_OK("ethercat: %u slaves on %s, cycle=%u us", master_.SlaveCount(), config.interface_name.c_str(), config.cycle_us);
    return true;
}

/**
 * @brief 启动周期线程
 * @return true 启动成功
 */
static bool thread_start()
{
    if (!master_.RequestOperational()) {
        DUST_LOG_ERR("ethercat: slaves failed to reach OP");
        return false;
    }

    thread_.Start(Task, std::chrono::milliseconds(0), nullptr, "ethercat",
                  init::ThreadFaultConfig{ nullptr, nullptr, init::ThreadFaultPolicy::Recover });
    return true;
}

/**
 * @brief 通知周期线程停止
 */
static void thread_stop()
{
    thread_.RequestStop();
}

/**
 * @brief 等待线程退出后关闭主站
 */
static void thread_join()
{
    thread_.Join();
}

/**
 * @brief 关闭主站，支持未初始化与重复调用
 */
static void Release()
{
    master_.Close();
}

REGISTER_INIT(thread_init, MidInit, High, HaltOnFail, Release, "ethercat_init");
REGISTER_THREAD(thread_start, MidThread, HaltOnFail, "ethercat_start");

} // namespace thread::ethercat
```

**`trd_ethercat.yaml`**：

```yaml
# EtherCAT 主站网口名；该网口不配 IP，专供总线。
interface: enp2s0

# 通信周期，单位 µs；整数，范围 [100, 100000]。
# 调大：CPU 占用低、抖动余量大；调小：响应快，但对实时性要求高。
cycle_us: 1000

# 单次过程数据接收超时，单位 µs；整数，范围 [100, 100000]。
timeout_us: 2000

# 是否启用分布式时钟同步；从站支持 DC 且需要多轴同步时打开。
use_dc: false
```

**为什么 `period` 传 `0`**：外壳的 `period` 是"任务结束后再等 N"，会累积漂移。1 ms 硬周期必须用 `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` 算绝对时刻——这和你规划里 thread 的 `PeriodMode::Absolute` 是同一件事，`Task` 内部自己做了。

---

## 6. 实时性配置（P3，可选）

> **P3 拆成 4 步。** 只在 P2 实测抖动不达标时才做。

| 步 | 对象 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| **3-1** | `/etc/default/grub` | 追加 `isolcpus=2-7 nohz_full=2-7 rcu_nocbs=2-7` | P0 步 0-2 | 重启；这几个核不再跑普通任务 |
| **3-2** | `trd_ethercat.cpp` | `thread_start` 里用 `pthread_setaffinity_np` 绑核 | 3-1 | 无 |
| **3-3** | `trd_ethercat.cpp` | `pthread_setschedparam` 设 `SCHED_FIFO` 优先级 80 | 0-3 | 无 |
| **3-4** | 实测 | `cyclictest` + 主站跑 30 分钟看最大抖动 | 3-1~3-3 | 无 |

**绑核代码**（放进 `thread_start`，在 `thread_.Start` 之后）：

```cpp
    // 绑到隔离核 2，避免被普通任务抢占
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(2, &set);
    pthread_setaffinity_np(thread_.Tid(), sizeof(set), &set);

    // SCHED_FIFO 优先级 80（需要 realtime 组权限）
    sched_param param {};
    param.sched_priority = 80;
    pthread_setschedparam(thread_.Tid(), SCHED_FIFO, &param);
```

**注意 `thread_.Tid()`**：当前 `Thread` 的 `Tid()` 返回 `pthread_t`（子架构 `thread.hpp:260-263`），可直接传给这两个函数。**前提是 `thread_start` 里 `Start()` 已返回**——`std::thread` 创建后 `pthread_self()` 可能还没跑到 `life_.tid.store`，需要加一个短自旋等 `Tid() != 0`：

```cpp
    // 等线程入口存下 tid（Start 之后 tid 是异步写入的）
    for (int i = 0; i < 1000 && thread_.Tid() == 0; ++i) {
        std::this_thread::yield();
    }
```

---

## 7. 验证清单

| # | 阶段 | 动作 | 通过标准 |
|---|---|---|---|
| 1 | P0 | `cat /sys/kernel/debug/sched/preempt` | `full` |
| 2 | P0 | `ulimit -r` | `99` |
| 3 | P0 | `sudo cyclictest -m -Sp90 -i1000 -h100 -p95 -t4` | Max < 100 µs |
| 4 | P0 | `which slaveinfo` | `/usr/local/bin/slaveinfo` |
| 5 | P1 | `sudo slaveinfo enp2s0` | 从站数与实物一致 |
| 6 | P2 | 启动程序 | `ethercat: N slaves on enp2s0, cycle=1000 us` |
| 7 | P2 | `thread list` | 有 `ethercat` 行，`RUN` 每秒涨 ~1000 |
| 8 | P2 | 写控制字让从站 LED 闪 | 从站响应正确 |
| 9 | P2 | 拔网线 | `ethercat: 10 consecutive WKC failures` → `thread ethercat fatal, stopping all threads` |
| 10 | P2 | Ctrl+C | 正常退出，`Release` 里 `ecx_close` 无报错 |
| 11 | P3 | 30 分钟运行 | 无 WKC 异常，最大抖动 < 100 µs |

---

## 8. 风险与回退

| 风险 | 症状 | 处理 |
|---|---|---|
| 网卡不支持 | `ecx_init` 返回 0 | 换一块 Intel **i210/i219**（IgH 与 SOEM 都支持），或确认网口名拼写 |
| `io_map_` 给小了 | `ecx_config_map_group` 返回 0，或运行期段错误 | 按真实 `Obits`/`Ibits` 求和分配（§5.2） |
| 从站进不了 OP | `slaves failed to reach OP` | 看 `ec_slave[i].ALstatuscode`，对照 ETG 错误码表；多数是 PDO 配置与 ESI 文件不符 |
| 接收超时 | `consecutive WKC failures` | 先查周期是否小于从站支持的最小周期；再查网线/终端电阻 |
| 抖动大 | `cyclictest` Max > 250 µs | 先确认 `preempt=full` 生效，再上 `isolcpus`，最后才换 RT 内核 |
| `SCHED_FIFO` 失败 | `pthread_setschedparam` 返回 `EPERM` | `ulimit -r` 是 0，检查 `realtime` 组与是否重新登录 |
| 需要 DC 同步 | 多轴不同步 | `use_dc: true` + 确认从站支持 DC（`slaveinfo` 的 `DC` 字段） |

---

## 9. 与现有工程的关系

**独立于 `qingyu` 视觉链**：EtherCAT 是控制总线，与相机/视觉线程没有数据依赖。它只依赖：

- `framework/init`（线程模板、注册）
- `framework/cmd`（日志、YAML 配置）

**Kconfig 默认关闭**（`DUST_COM_ETHERCAT` / `TRD_ETHERCAT` 都 `default n`），不影响现有视觉工程的编译与运行。

**如果以后要做「视觉 → EtherCAT 控制」闭环**，中间加一层 `framework/topic/vision_to_ethercat/`，按现有 topic 的「单槽最新值」语义传递目标角度——但这是另一件事，不在本次范围。

---

## 10. 执行顺序

```text
P0 环境（6 步）
  ↓  slaveinfo 能枚举
P1 从站枚举与 PDO 记录（3 步）
  ↓  PDO 表填完
P2 主站实现（8 步）
  ↓  周期通信跑通
P3 实时性加固（4 步，可选）
```

**总步数 21 步。每次只做一步，做完停下报告。**
