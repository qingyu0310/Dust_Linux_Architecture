/**
 * @file ethercat.cpp
 * @author qingyu
 * @brief EtherCAT 主站（SOEM）实现 —— 网口初始化、从站枚举、周期过程数据交换
 * @version 0.1
 * @date 2026-10-30
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ethercat.hpp"
#include "framework/shell/log/log.hpp"

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

#include <ifaddrs.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "soem/soem.h"

namespace drivers::ethercat
{

namespace
{

constexpr uint32_t kOperationalPollMs    = 10;       // OP 状态轮询间隔
constexpr uint32_t kOperationalTimeoutMs = 5000;     // OP 状态总超时
constexpr int      kStateCheckUs         = 10000;    // 单次状态查询超时（µs）
constexpr uint32_t kIoMapBytes           = 4096;     // IOmap 缓冲上限（字节）

/**
 * @brief 找第一个有载波、非回环、非无线的网口
 *
 * 载波由内核 SIOCGIFFLAGS 的 IFF_RUNNING 位反映，网线没插的口不会置位。
 *
 * @return 网口名；找不到返回空串
 */
std::string FindCarrierInterface()
{
    struct ifaddrs* head = nullptr;
    if (getifaddrs(&head) != 0)
    {
        return {};
    }

    std::string found;
    for (auto* item = head; item != nullptr; item = item->ifa_next)
    {
        if (item->ifa_name == nullptr)
        {
            continue;
        }

        const unsigned int flags = item->ifa_flags;
        if ((flags & IFF_LOOPBACK) != 0 || (flags & IFF_RUNNING) == 0)
        {
            continue;
        }
        if (std::string(item->ifa_name).rfind("wl", 0) == 0)
        {
            continue;   // 无线口不接 EtherCAT
        }

        found = item->ifa_name;
        break;
    }

    freeifaddrs(head);

    return found;
}

} // namespace

Master::~Master()
{
    Close();
}

/**
 * @brief 打开网口、枚举从站、映射过程数据
 *
 * 顺序：ecx_init → ecx_config_init → 按真实位宽分配 IOmap →
 * ecx_config_map_group → 取期望 WKC → 可选 DC。
 *
 * @param config 主站配置；interface_name 留空时自动选第一个有载波的网口
 * @return true=至少一个从站且映射成功
 * @throws std::runtime_error 无可用网口、网口打开失败、无从站、映射失败
 */
bool Master::Init(const Config& config)
{
    if (initialized_)
    {
        throw std::logic_error("ethercat master already initialized");
    }
    config_ = config;

    if (config_.interface_name.empty())
    {
        config_.interface_name = FindCarrierInterface();
        if (config_.interface_name.empty())
        {
            throw std::runtime_error("no network interface with carrier");
        }
    }

    DUST_LOG_INF("ethercat: interface %s", config_.interface_name.c_str());

    auto* ctx = new ecx_contextt{};
    context_ = ctx;

    if (!ecx_init(ctx, config_.interface_name.c_str()))
    {
        Close();
        throw std::runtime_error("ecx_init failed on " + config_.interface_name);
    }

    const int found = ecx_config_init(ctx);
    DUST_LOG_INF("ethercat: ecx_config_init found %d slaves", found);

    if (found <= 0)
    {
        Close();
        throw std::runtime_error("no EtherCAT slave found on " + config_.interface_name);
    }
    slave_count_ = static_cast<uint16_t>(ctx->slavecount);

    // 真实 PDO 长度要等 map_group 读完 CoE 才知道，先给足缓冲。
    io_map_.assign(kIoMapBytes, 0);

    const int mapped = ecx_config_map_group(ctx, io_map_.data(), 0);
    DUST_LOG_INF("ethercat: ecx_config_map_group mapped %d bytes (io_map %u bytes)", mapped, static_cast<unsigned>(io_map_.size()));

    if (mapped <= 0)
    {
        Close();
        throw std::runtime_error("ecx_config_map_group failed");
    }
    if (static_cast<uint32_t>(mapped) > io_map_.size())
    {
        Close();
        throw std::runtime_error("IOmap too small: mapped bytes exceed buffer");
    }

    // EEPROM 里 SM2/SM3 写的是 4 字节，从站固件实际按 32 字节配置，两者对不上会让
    // 从站停在 SAFE-OP。以 CoE 读出的 PDO 映射为准，把 SM2/SM3 改写成 Obytes/Ibytes。
    for (uint16_t i = 1; i <= slave_count_; ++i)
    {
        auto& slave = ctx->slavelist[i];
        const uint32_t sm_out = etohs(slave.SM[2].SMlength);
        const uint32_t sm_in  = etohs(slave.SM[3].SMlength);

        DUST_LOG_INF("ethercat: slave %u SM2=%u B SM3=%u B, CoE says Obytes=%u Ibytes=%u",
                     static_cast<unsigned>(i),
                     static_cast<unsigned>(sm_out),
                     static_cast<unsigned>(sm_in),
                     static_cast<unsigned>(slave.Obytes),
                     static_cast<unsigned>(slave.Ibytes));

        if (slave.Obytes > 0)
        {
            slave.SM[2].SMlength = htoes(static_cast<uint16_t>(slave.Obytes));
            const int w = ecx_FPWR(&ctx->port, slave.configadr, ECT_REG_SM2, sizeof(ec_smt), &slave.SM[2], EC_TIMEOUTRET3);
            DUST_LOG_INF("ethercat: slave %u SM2 rewrite to %u B, wkc=%d", static_cast<unsigned>(i), static_cast<unsigned>(slave.Obytes), w);
        }
        if (slave.Ibytes > 0)
        {
            slave.SM[3].SMlength = htoes(static_cast<uint16_t>(slave.Ibytes));
            const int w = ecx_FPWR(&ctx->port, slave.configadr, ECT_REG_SM3, sizeof(ec_smt), &slave.SM[3], EC_TIMEOUTRET3);
            DUST_LOG_INF("ethercat: slave %u SM3 rewrite to %u B, wkc=%d", static_cast<unsigned>(i), static_cast<unsigned>(slave.Ibytes), w);
        }
    }

    // 期望 WKC 取自 SOEM 自己按组算好的值：输出读写各计 1、输入计 1
    expected_wkc_ = static_cast<uint32_t>(ctx->grouplist[0].outputsWKC) * 2U + static_cast<uint32_t>(ctx->grouplist[0].inputsWKC);
    DUST_LOG_INF("ethercat: expected WKC=%u (outWKC=%u inWKC=%u)",
                 static_cast<unsigned>(expected_wkc_),
                 static_cast<unsigned>(ctx->grouplist[0].outputsWKC),
                 static_cast<unsigned>(ctx->grouplist[0].inputsWKC));

    if (config.use_dc)
    {
        (void)ecx_configdc(ctx);
    }

    initialized_ = true;

    return true;
}

/**
 * @brief 关闭主站并释放 SOEM 上下文；可重复调用
 */
void Master::Close() noexcept
{
    if (context_ != nullptr)
    {
        auto* ctx = static_cast<ecx_contextt*>(context_);
        if (initialized_)
        {
            ecx_close(ctx);
        }
        delete ctx;
        context_ = nullptr;
    }

    io_map_.clear();
    slave_count_  = 0;
    expected_wkc_ = 0;
    initialized_  = false;
}

/**
 * @brief 把所有从站切到 OP 并等待就绪
 *
 * @return true=全部进入 OP
 */
bool Master::RequestOperational()
{
    if (!initialized_)
    {
        DUST_LOG_ERR("ethercat: RequestOperational before Init");
        return false;
    }

    auto* ctx = static_cast<ecx_contextt*>(context_);

    for (uint16_t i = 1; i <= slave_count_; ++i)
    {
        DUST_LOG_INF("ethercat: slave %u %s -> OP request (Obytes=%u Ibytes=%u)",
                     static_cast<unsigned>(i),
                     ctx->slavelist[i].name,
                     static_cast<unsigned>(ctx->slavelist[i].Obytes),
                     static_cast<unsigned>(ctx->slavelist[i].Ibytes));
        ctx->slavelist[i].state = EC_STATE_OPERATIONAL;
    }

    // ecx_writestate(ctx, 0) 广播的是 slavelist[0].state，必须一并设置成 OP，
    // 否则广播出去的是 0（INIT）
    ctx->slavelist[0].state = EC_STATE_OPERATIONAL;

    DUST_LOG_INF("ethercat: writestate group 0 (broadcast state=0x%02x)", static_cast<unsigned>(ctx->slavelist[0].state));
    (void)ecx_send_processdata(ctx);
    (void)ecx_receive_processdata(ctx, config_.timeout_us);
    (void)ecx_writestate(ctx, 0);

    const auto begin = std::chrono::steady_clock::now();
    uint16_t   last_state = 0xFFFF;

    while (true)
    {
        // 切 OP 期间必须持续交换过程数据：从站的 SM 看门狗要看到过程数据才放行
        (void)ecx_send_processdata(ctx);
        (void)ecx_receive_processdata(ctx, config_.timeout_us);
        (void)ecx_statecheck(ctx, 0, EC_STATE_OPERATIONAL, kStateCheckUs);
        (void)ecx_readstate(ctx);

        const auto elapsed_ms = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count());

        bool all_operational = true;
        for (uint16_t i = 1; i <= slave_count_; ++i)
        {
            const uint16_t state = ctx->slavelist[i].state;
            if (state != EC_STATE_OPERATIONAL)
            {
                all_operational = false;
            }

            // 只在状态变化时打日志，不按轮次刷屏
            if (state != last_state)
            {
                last_state = state;
                DUST_LOG_INF("ethercat: %u ms slave %u state=0x%02x alstatus=0x%04x",
                             static_cast<unsigned>(elapsed_ms),
                             static_cast<unsigned>(i),
                             static_cast<unsigned>(state),
                             static_cast<unsigned>(ctx->slavelist[i].ALstatuscode));
            }
        }

        if (all_operational)
        {
            DUST_LOG_OK("ethercat: all slaves in OP after %u ms", static_cast<unsigned>(elapsed_ms));
            return true;
        }

        if (elapsed_ms >= kOperationalTimeoutMs)
        {
            DUST_LOG_ERR("ethercat: OP timeout after %u ms, last state=0x%02x alstatus=0x%04x",
                         static_cast<unsigned>(elapsed_ms),
                         static_cast<unsigned>(ctx->slavelist[1].state),
                         static_cast<unsigned>(ctx->slavelist[1].ALstatuscode));
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kOperationalPollMs));
    }
}

/**
 * @brief 发送一帧过程数据
 *
 * @return true=提交成功
 */
bool Master::Send()
{
    if (!initialized_)
    {
        DUST_LOG_ERR("ethercat: Send before Init");
        return false;
    }

    const int sent = ecx_send_processdata(static_cast<ecx_contextt*>(context_));
    if (sent <= 0)
    {
        DUST_LOG_ERR("ethercat: send_processdata returned %d", sent);
    }

    return sent > 0;
}

/**
 * @brief 接收一帧过程数据并校验工作计数器
 *
 * 只判 WKC，不做重连——重连策略归线程，驱动越权重连会让线程的故障策略失效。
 *
 * @return true=WKC 达到期望值
 */
bool Master::Receive()
{
    if (!initialized_)
    {
        DUST_LOG_ERR("ethercat: Receive before Init");
        return false;
    }

    const int wkc = ecx_receive_processdata(static_cast<ecx_contextt*>(context_), static_cast<int>(config_.timeout_us));
    if (wkc < 0 || static_cast<uint32_t>(wkc) < expected_wkc_)
    {
        DUST_LOG_ERR("ethercat: WKC %d < expected %u", wkc, static_cast<unsigned>(expected_wkc_));
        return false;
    }

    return true;
}

/**
 * @brief 写某个从站的输出过程数据（RxPDO，主→从）
 *
 * 按结构体整块拷进 IOmap，下一帧随 Send() 发出。
 *
 * @param index 从站序号（1 起）
 * @param outputs 输出数据
 * @return true=写入成功；越界或未初始化返回 false
 */
bool Master::SetOutputs(uint16_t index, const Outputs& outputs)
{
    if (!initialized_ || index == 0 || index > slave_count_)
    {
        DUST_LOG_ERR("ethercat: SetOutputs index %u out of range", static_cast<unsigned>(index));
        return false;
    }

    const auto& slave = static_cast<ecx_contextt*>(context_)->slavelist[index];
    if (slave.outputs == nullptr)
    {
        DUST_LOG_ERR("ethercat: slave %u has no output area", static_cast<unsigned>(index));
        return false;
    }
    if (slave.Obytes < sizeof(Outputs))
    {
        DUST_LOG_ERR("ethercat: slave %u output size %u < struct %u",
                     static_cast<unsigned>(index),
                     static_cast<unsigned>(slave.Obytes),
                     static_cast<unsigned>(sizeof(Outputs)));
        return false;
    }

    std::memcpy(slave.outputs, &outputs, sizeof(Outputs));

    return true;
}

/**
 * @brief 读某个从站的输入过程数据（TxPDO，从→主）
 *
 * 按结构体整块从 IOmap 拷出，取的是最近一次 Receive() 的内容。
 *
 * @param index 从站序号（1 起）
 * @param inputs 输出：输入数据
 * @return true=读取成功；越界或未初始化返回 false
 */
bool Master::GetInputs(uint16_t index, Inputs& inputs) const
{
    if (!initialized_ || index == 0 || index > slave_count_)
    {
        DUST_LOG_ERR("ethercat: GetInputs index %u out of range", static_cast<unsigned>(index));
        return false;
    }

    const auto& slave = static_cast<const ecx_contextt*>(context_)->slavelist[index];
    if (slave.inputs == nullptr)
    {
        DUST_LOG_ERR("ethercat: slave %u has no input area", static_cast<unsigned>(index));
        return false;
    }
    if (slave.Ibytes < sizeof(Inputs))
    {
        DUST_LOG_ERR("ethercat: slave %u input size %u < struct %u",
                     static_cast<unsigned>(index),
                     static_cast<unsigned>(slave.Ibytes),
                     static_cast<unsigned>(sizeof(Inputs)));
        return false;
    }

    std::memcpy(&inputs, slave.inputs, sizeof(Inputs));

    return true;
}

/**
 * @brief 取某个从站的运行状态
 *
 * @param index 从站序号（1 起）
 * @return 状态快照；越界时 index=0
 */
SlaveState Master::GetSlaveState(uint16_t index) const
{
    SlaveState state{};

    if (!initialized_ || index == 0 || index > slave_count_)
    {
        return state;
    }

    const auto* ctx = static_cast<const ecx_contextt*>(context_);
    const auto& slave = ctx->slavelist[index];

    state.index          = index;
    state.al_state       = slave.state;
    state.al_status_code = slave.ALstatuscode;
    state.output_bytes   = slave.Obytes;
    state.input_bytes    = slave.Ibytes;

    return state;
}

} // namespace drivers::ethercat
