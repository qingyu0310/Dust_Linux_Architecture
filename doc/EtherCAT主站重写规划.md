# EtherCAT 主站重写规划（严格照子树驱动风格）

日期：2026-10-30。范围：重写 `framework/drivers/communication/ethercat/` 与 `project/thread/ethercat/trd_ethercat.cpp`。本文只做规划。

## 1. 反面清单（现在这份代码错在哪）

| # | 现状 | 违反 |
|---|---|---|
| 1 | 拆了 `Ecat` 模板 + `EcatCore` 非模板两层 | 子树驱动**一个类**（`UartDma` / `Can`），不拆两层 |
| 2 | `Config` 里塞 `cycle_us` / `wkc_fail_limit` / `op_timeout_ms` | `Config` 只放**硬件参数** |
| 3 | 驱动里有重连退避、失败计数、故障策略 | 运行期策略归**线程** |
| 4 | `Init` 串了 7 个阶段且阶段函数对外公开 | 子树只露 `Init` / `Send` / `Read` |
| 5 | 类成员 13 个平铺 | 父架构 `thread.hpp` 用子结构体分组 |

## 2. 子树驱动的实际形状（照抄这个）

单类、`Config` 内嵌、`Init` / `Send` / `Read` 三个对外动作、状态全在私有成员：

```cpp
class UartDma final : public Stream
{
public:
    struct Config                   // 只放硬件参数，全带默认值
    {
        uart_config line_cfg { 921600, ... };
        BaseConfig  base_cfg {};
    };

    bool     Init(const struct device* dev, const Config& cfg);
    uint16_t Read(uint8_t* buf, uint16_t max_len);
    bool     Send(const uint8_t* data, uint32_t len);

private:
    static constexpr uint16_t kMaxBufSize = 256;
    uint8_t  dma_buf_[2][kMaxBufSize] {};
    const device* dev_ = nullptr;
    Config        config_ {};
    atomic_t      ready_ = ATOMIC_INIT(0);
};
```

## 3. 重写后的 `Ecat`

### 3.1 一个模板类，不分两层

PDO 布局由从站固件/ESI 决定，属于业务，由使用方给类型：

```cpp
/**
 * @brief EtherCAT 主站驱动
 *
 * 单实例、单网口。Init 建链后由上层线程每拍 Send / Read。
 * Out / In 是主从约定的过程数据结构，定义在使用方。
 * 所有调用必须来自同一线程，SOEM 的 ecx_context 不可重入。
 *
 * @tparam Out 输出过程数据类型（RxPDO，主→从）
 * @tparam In  输入过程数据类型（TxPDO，从→主）
 */
template <typename Out, typename In>
class Ecat final
{
public:
    Ecat() = default;
    ~Ecat() { Close(); }

    Ecat(const Ecat&)            = delete;
    Ecat& operator=(const Ecat&) = delete;

    bool Init(const Config& cfg);                   // 建链：打开网口到全部从站进入 OP
    Out& OutputsRef() { return outputs_; }          // 待发送的输出镜像
    const In& InputsRef() const { return inputs_; } // 已收到的输入镜像
    bool Send();                                    // outputs_ 写进 IOmap 并发一帧
    int  Read();                                    // 收一帧并校验 WKC，返回收到的字节数；<0 失败

    void Close() noexcept;                          // 关闭总线并释放 context

    uint16_t           SlaveCount() const;          // 从站数量
    const SlaveInfo&   Slave(uint16_t index) const; // 从站信息；越界返回空项
    const Config&      GetConfig() const;           // 主站配置
    const std::string& InterfaceName() const;       // 实际使用的网口
    uint32_t           ExpectedWkc() const;         // 期望工作计数器
    bool               IsReady() const;             // 链路是否已建好

private:
    struct Link { ... };                            // 连接与上下文
    struct Pdo  { ... };                            // 过程数据布局与期望 WKC
    struct Stats{ ... };                            // 运行统计

    bool Open();                                    // 以下阶段函数全部 private，只由 Init 调用
    bool Discover();
    uint16_t Enumerate();
    bool ConfigureSlaves();
    bool ConfigurePdo();
    bool ConfigureDc();
    bool RequestState(uint16_t index, uint16_t state);
    bool RequestOperational();

    Config                 config_ {};              // 主站配置
    Link                   link_ {};                // 连接与上下文
    std::vector<SlaveInfo> slaves_ {};              // 从站信息
    Pdo                    pdo_ {};                 // 过程数据
    Stats                  stats_ {};               // 运行统计
    Out                    outputs_ {};             // 输出过程数据镜像
    In                     inputs_  {};             // 输入过程数据镜像
};
```

要点：

- **模板直接做类本体，不再套 `EcatCore`**。SOEM 头文件只出现在 `.cpp`；模板成员函数在 `.cpp` 里用 `template class Ecat<PdoOutputs, PdoInputs>;` 显式实例化，或者成员函数实现留在 `.hpp`（含 `#include "soem/soem.h"` 会漏到使用方，所以走显式实例化）。
- `Send` / `Read` 只碰 `outputs_` / `inputs_` 两个镜像，驱动里**不出现业务字段名**。
- 阶段函数（`Open`/`Discover`/...）全部 **private**，只由 `Init` 调用；对外只露 `Init` / `Send` / `Read` / `Close` + 只读查询。
- **不对外暴露 `Recover` / `Monitor`**：恢复是线程 `OnRetry` 重新 `Init`，健康由 `IsReady()` + `Read` 返回值体现。

### 3.2 配置：只留硬件参数

```cpp
/**
 * @brief 主站配置
 */
struct Config
{
    std::string interface_name = "";                // 网口名；留空=自动选第一个有载波的网口
    SyncMode    sync_mode      = SyncMode::SmSync;  // 同步模式
    uint32_t    rx_timeout_us  = 2000;              // 单次接收超时（µs）
};
```

删掉的项及其归属：

| 删掉 | 归属 |
|---|---|
| `cycle_us` | 线程 `Start()` 的周期，线程自己定 |
| `op_timeout_ms` | 建链超时是线程的重试策略 |
| `wkc_fail_limit` | 故障阈值归线程 `OnRetry` |

### 3.3 成员分组

| 子结构体 | 成员 |
|---|---|
| `config_` | `Config` 直接是成员（照 `UartDma` 的 `config_`） |
| `link_` | `context`（`void*`）、`io_map`、`opened`、`discovered` |
| `slaves_` | `std::vector<SlaveInfo>`（单成员，直接用成员不套结构体） |
| `pdo_` | `expected_wkc`、`configured` |
| `stats_` | `last_wkc`（状态查询用；失败计数归线程） |
| `outputs_` / `inputs_` | `Out` / `In` 镜像，直接是成员 |

清空用 `x = {};`。

### 3.4 日志

`Init` 打网口/扫描数/每站 SM/期望 WKC/进 OP 耗时；`Send` 失败打 `sent`；`Read` 失败打 `wkc` vs `expected`。每拍用 `DBG("ecat", ...)`，建链用 `INF`。

## 4. 线程结构（照 `trd_remote` / `trd_imu` 的薄启动）

```cpp
namespace thread::ethercat {

/** 过程数据：主站写给从站（RxPDO） */
struct PdoOutputs { uint32_t counter[8]; };

/** 过程数据：从站回给主站（TxPDO） */
struct PdoInputs { uint32_t counter[8]; };

static drivers::ethercat::Ecat<PdoOutputs, PdoInputs> ecat_ {};   // 主站实例
static init::Thread                                  thread_ {}; // 周期线程

bool thread_init()
{
    return ecat_.Init({});
}

bool thread_start()
{
    thread_.Start(Task, DUST_MSEC(1), nullptr, "ethercat",
                  ThreadFaultConfig{OnRetry, nullptr, ThreadFaultPolicy::Recover, false});
    return true;
}

}
```

线程侧：

- `thread_init()`：只 `ecat_.Init({})`
- `Task()`：填 `OutputsRef()` → `Send()` → `Read()` → 失败 `return Retry`
- `OnRetry()`：`ecat_.Close()` → 重新 `Init`；**不计数、不退避**
- 周期写死 `DUST_MSEC(1)`

## 5. 分步表

> **开始第 N 步 = 只改这一步指定的那一个文件，改完停，等下一条指令。**

| 步 | 文件 | 动作 | 依赖 | 连带 |
|---|---|---|---|---|
| 1 | `ecat.hpp` | 删 `ecat_impl.hpp`、删 `EcatCore`，模板 `Ecat<Out, In>` 单类；`Config` 收成三项；阶段函数转 private；成员分组；`Send()`/`Read()` 无参 | 无 | 无 |
| 2 | `ecat.cpp` | 由 `ecat_impl.cpp` 改名，类名改 `Ecat<Out, In>`，成员函数带 `template <typename Out, typename In>`；文件末尾显式实例化；删策略参数 | 步 1 | 无 |
| 3 | 删除 `ecat_impl.hpp` / `ecat_impl.cpp` | 内容已并入步 1/2 | 步 2 | 无 |
| 4 | `framework/drivers/CMakeLists.txt` | 源文件 `ecat_impl.cpp` → `ecat.cpp` | 步 3 | 无 |
| 5 | `trd_ethercat.cpp` | `PdoOutputs` / `PdoInputs` 留在 trd；改用 `Ecat<PdoOutputs, PdoInputs>`；删计数与退避；`OnRetry` 只 Close + Init | 步 4 | **与步 6 一起编过** |
| 6 | 编译 + 上板 | 首次连接不空等、WKC 达标、双向 32 字节可核对 | 步 5 | 用户执行 |

## 6. 待拍板

| # | 问题 | 建议 |
|---|---|---|
| A | 显式实例化放 `ecat.cpp` 末尾，还是成员函数实现留 `.hpp` | 显式实例化（SOEM 头文件不能漏到使用方） |
| B | `SyncMode` 放 `Config` 还是独立成员 | 放 `Config` |

## 7. 验收

1. 驱动目录只有 `ecat.hpp` + `ecat.cpp`，**没有 `EcatCore`、没有两层**。
2. `Config` 只有 `interface_name` / `sync_mode` / `rx_timeout_us` 三项。
3. 驱动里 `grep cycle_us\|wkc_fail_limit\|op_timeout` 为空。
4. 驱动里 `grep counter` 为空；PDO 结构只在 `trd_ethercat.cpp`。
5. 阶段函数全部 private，对外只露 `Init`/`Send`/`Read`/`Close` 与只读查询。
6. 线程文件级只有 `ecat_` 和 `thread_`。
