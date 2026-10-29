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
    std::string interface_name;          // 网口名；留空=自动选第一个有载波的网口
    uint32_t    timeout_us = 2000;       // 单次过程数据接收超时（µs）
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
 * @brief 过程数据：主站写入从站的输出（RxPDO，SM2 / 0x1600）
 *
 * 从站固件按 8 个 UDINT、合计 32 字节配置。
 */
struct Outputs
{
    uint32_t counter[8];                 // 输出计数器
};

/**
 * @brief 过程数据：从站回给主站的输入（TxPDO，SM3 / 0x1A00）
 *
 * 从站固件按 8 个 UDINT、合计 32 字节配置。
 */
struct Inputs
{
    uint32_t counter[8];                 // 输入计数器
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
     *
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
     *
     * @return true=全部进入 OP
     */
    bool RequestOperational();

    /**
     * @brief 发送一帧过程数据
     *
     * @return true=提交成功
     */
    bool Send();

    /**
     * @brief 接收一帧过程数据并校验工作计数器
     *
     * @return true=WKC 达到期望值
     */
    bool Receive();

    /**
     * @brief 从站数量
     */
    uint16_t SlaveCount() const { return slave_count_; }

    /**
     * @brief 实际使用的网口名
     *
     * Init 之后才有意义；配置里留空时这里是自动探测到的那个。
     */
    const std::string& InterfaceName() const { return config_.interface_name; }

    /**
     * @brief 取某个从站的运行状态
     *
     * @param index 从站序号（1 起）
     * @return 状态快照；越界时 index=0
     */
    SlaveState GetSlaveState(uint16_t index) const;

    /**
     * @brief 写某个从站的输出过程数据（RxPDO，主→从）
     *
     * 按结构体整块拷进 IOmap，下一帧随 Send() 发出。
     *
     * @param index 从站序号（1 起）
     * @param outputs 输出数据
     * @return true=写入成功；越界或未初始化返回 false
     */
    bool SetOutputs(uint16_t index, const Outputs& outputs);

    /**
     * @brief 读某个从站的输入过程数据（TxPDO，从→主）
     *
     * 按结构体整块从 IOmap 拷出，取的是最近一次 Receive() 的内容。
     *
     * @param index 从站序号（1 起）
     * @param inputs 输出：输入数据
     * @return true=读取成功；越界或未初始化返回 false
     */
    bool GetInputs(uint16_t index, Inputs& inputs) const;

    /**
     * @brief 期望工作计数器
     */
    uint32_t ExpectedWorkingCounter() const { return expected_wkc_; }

private:
    Config               config_ {};              // 主站配置
    void*                context_ = nullptr;      // SOEM ecx_contextt，锁在 .cpp 里
    std::vector<uint8_t> io_map_;                 // 过程数据映射缓冲
    uint16_t             slave_count_ = 0;        // 从站数量
    uint32_t             expected_wkc_ = 0;       // 期望 WKC
    bool                 initialized_ = false;    // 是否已初始化
};

} // namespace drivers::ethercat
