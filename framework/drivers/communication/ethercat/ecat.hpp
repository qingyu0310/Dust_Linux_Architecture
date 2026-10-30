/**
 * @file ecat.hpp
 * @author qingyu
 * @brief EtherCAT 主站驱动 —— 建链、过程数据收发
 * @version 3.1
 * @date 2026-10-30
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <ifaddrs.h>
#include <net/if.h>
#include <unistd.h>

#include "framework/shell/log/log.hpp"
#include "soem/soem.h"

namespace drivers::ethercat
{

/**
 * @brief 从站信息
 */
struct SlaveInfo
{
    uint16_t    index;              // 从站序号（1 起）
    uint16_t    configadr;          // 配置站地址
    std::string name;               // 从站名（SII 读出）
    uint16_t    al_state;           // AL 状态
    uint16_t    al_status;          // AL 状态码
    uint32_t    output_bytes;       // RxPDO 字节数（主→从）
    uint32_t    input_bytes;        // TxPDO 字节数（从→主）
    uint32_t    output_offset;      // RxPDO 在 IOmap 的偏移
    uint32_t    input_offset;       // TxPDO 在 IOmap 的偏移
};

/**
 * @brief 同步模式
 */
enum class SyncMode : uint32_t
{
    FreeRun,    // 主站发多快从站处理多快
    SmSync,     // SM2 写事件触发从站输出处理
    Dc,         // 分布式时钟，按 DC 周期对齐
};

/**
 * @brief 主站配置
 */
struct Config
{
    std::string interface_name = "";                // 网口名；留空=自动选第一个有载波的网口
    SyncMode    sync_mode      = SyncMode::SmSync;  // 同步模式
    uint32_t    rx_timeout_us  = 2000;              // 单次接收超时（µs）
};

/**
 * @brief EtherCAT 主站驱动
 *
 * 单实例、单网口。Init 建链后由上层线程每拍 Send / Read。
 * Out / In 是主从约定的过程数据结构，由使用方定义。
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

    /**
     * @brief 建链：打开网口、枚举从站、映射 PDO、全部进入 OP
     *
     * @param cfg 主站配置
     * @return true=全部从站进入 OP
     */
    bool Init(const Config& cfg)
    {
        Close();

        config_ = cfg;

        return Open() && Discover() && (Enumerate() > 0) && ConfigureSlaves() && ConfigurePdo() && ConfigureDc() && RequestOperational();
    }

    Out&       OutputsRef() { return outputs_; }          // 待发送的输出镜像
    const In&  InputsRef() const { return inputs_; }      // 已收到的输入镜像

    /**
     * @brief 把输出镜像写进 IOmap，并用逻辑地址发一帧过程数据
     *
     * @return true=帧已提交
     */
    bool Send()
    {
        if (!pdo_.configured || link_.context == nullptr || slaves_.empty())
        {
            DUST_LOG_ERR("ecat: Send before Init");
            return false;
        }

        const SlaveInfo& info = slaves_[0];

        if (info.output_bytes < sizeof(Out))
        {
            DUST_LOG_ERR("ecat: slave 1 output %u B < struct %u B", static_cast<unsigned>(info.output_bytes), static_cast<unsigned>(sizeof(Out)));
            return false;
        }
        if ((static_cast<size_t>(info.output_offset) + sizeof(Out)) > link_.io_map.size())
        {
            DUST_LOG_ERR("ecat: slave 1 output offset %u + %u B out of io_map %u", static_cast<unsigned>(info.output_offset), static_cast<unsigned>(sizeof(Out)), static_cast<unsigned>(link_.io_map.size()));
            return false;
        }

        std::memcpy(link_.io_map.data() + info.output_offset, &outputs_, sizeof(Out));

        auto* ctx = static_cast<ecx_contextt*>(link_.context);

        const int sent = ecx_send_processdata(ctx);
        if (sent <= 0)
        {
            DUST_LOG_ERR("ecat: send_processdata returned %d", sent);
            return false;
        }

        return true;
    }

    /**
     * @brief 收一帧过程数据，校验 WKC 后把 IOmap 回填到输入镜像
     *
     * @return 收到的字节数；失败返回 -1
     */
    int Read()
    {
        if (!pdo_.configured || link_.context == nullptr || slaves_.empty())
        {
            DUST_LOG_ERR("ecat: Read before Init");
            return -1;
        }

        const SlaveInfo& info = slaves_[0];

        if (info.input_bytes < sizeof(In))
        {
            DUST_LOG_ERR("ecat: slave 1 input %u B < struct %u B", static_cast<unsigned>(info.input_bytes), static_cast<unsigned>(sizeof(In)));
            return -1;
        }
        if ((static_cast<size_t>(info.input_offset) + sizeof(In)) > link_.io_map.size())
        {
            DUST_LOG_ERR("ecat: slave 1 input offset %u + %u B out of io_map %u",
                         static_cast<unsigned>(info.input_offset),
                         static_cast<unsigned>(sizeof(In)),
                         static_cast<unsigned>(link_.io_map.size()));
            return -1;
        }

        auto* ctx = static_cast<ecx_contextt*>(link_.context);

        const int wkc = ecx_receive_processdata(ctx, static_cast<int>(config_.rx_timeout_us));

        stats_.last_wkc = (wkc > 0) ? static_cast<uint32_t>(wkc) : 0U;

        if (wkc < 0 || static_cast<uint32_t>(wkc) < pdo_.expected_wkc)
        {
            DUST_LOG_ERR("ecat: receive wkc=%d expected=%u", wkc, static_cast<unsigned>(pdo_.expected_wkc));
            return -1;
        }

        std::memcpy(&inputs_, link_.io_map.data() + info.input_offset, sizeof(In));

        DUST_LOG_DBG("ecat", "wkc=%d expected=%u", wkc, static_cast<unsigned>(pdo_.expected_wkc));

        return static_cast<int>(sizeof(In));
    }

    /**
     * @brief 关闭总线并释放 context；可重复调用
     */
    void Close() noexcept
    {
        if (link_.context != nullptr)
        {
            auto* ctx = static_cast<ecx_contextt*>(link_.context);
            if (link_.opened)
            {
                ecx_close(ctx);
            }
            delete ctx;
        }

        link_   = {};
        slaves_ = {};
        pdo_    = {};
        stats_  = {};
    }

    uint16_t SlaveCount() const { return static_cast<uint16_t>(slaves_.size()); }  // 从站数量

    /**
     * @brief 取从站信息
     *
     * @param index 从站序号（1 起）
     * @return 从站信息；越界返回空项
     */
    const SlaveInfo& Slave(uint16_t index) const
    {
        static const SlaveInfo kEmpty {};

        if (index == 0 || index > slaves_.size())
        {
            return kEmpty;
        }

        return slaves_[index - 1];
    }

    const Config&      GetConfig() 	   const { return config_; }                    // 主站配置
    const std::string& InterfaceName() const { return config_.interface_name; }  	// 实际使用的网口
    uint32_t           ExpectedWkc()   const { return pdo_.expected_wkc; }         	// 期望工作计数器
    bool               IsReady() 	   const { return pdo_.configured;   }          // 链路是否已建好

private:
    struct Link
    {
        void*                context = nullptr;     // SOEM ecx_contextt
        std::vector<uint8_t> io_map;                // 过程数据映射缓冲
        bool                 opened = false;        // 网口已打开
        bool                 discovered = false;    // 从站已扫描
    };

    struct Pdo
    {
        uint32_t expected_wkc = 0;                  // 期望工作计数器
        bool     configured = false;                // PDO 已映射
    };

    struct Stats
    {
        uint32_t last_wkc = 0;                      // 最近一次收到的 WKC
    };

    static constexpr uint32_t kIoMapBytes          = 4096;   	// IOmap 缓冲上限（字节）
    static constexpr uint32_t kStateCheckUs        = 1000;   	// 单次状态查询超时（µs）
    static constexpr uint32_t kWriteRetryYieldMs   = 1;      	// 写请求失败后的重试间隔（ms）
    static constexpr uint32_t kStateWriteTimeoutMs = 1000; 	 	// 状态请求写入/到达的总超时（ms）

	Config                 config_  {};             // 主站配置
    Link                   link_    {};             // 连接与上下文
    std::vector<SlaveInfo> slaves_  {};             // 从站信息
    Pdo                    pdo_ 	{};             // 过程数据
    Stats                  stats_   {};             // 运行统计
    Out                    outputs_ {};             // 输出过程数据镜像
    In                     inputs_  {};             // 输入过程数据镜像

    /**
     * @brief 打开网口并建立 SOEM context
     *
     * @return true=网口已打开
     */
    bool Open()
    {
        if (config_.interface_name.empty())
        {
            config_.interface_name = FindCarrierInterface();
            if (config_.interface_name.empty())
            {
                DUST_LOG_ERR("ecat: no network interface with carrier");
                return false;
            }
        }

        auto* ctx = new ecx_contextt{};
        link_.context = ctx;

        if (!ecx_init(ctx, config_.interface_name.c_str()))
        {
            DUST_LOG_ERR("ecat: ecx_init failed on %s", config_.interface_name.c_str());
            delete ctx;
            link_.context = nullptr;
            return false;
        }

        link_.opened = true;

        DUST_LOG_INF("ecat: interface %s opened, sync_mode=%s rx_timeout=%u us", config_.interface_name.c_str(), SyncModeName(config_.sync_mode), static_cast<unsigned>(config_.rx_timeout_us));

        return true;
    }

    /**
     * @brief 自动递增扫描从站并分配配置站地址
     *
     * @return true=至少一个从站
     */
    bool Discover()
    {
        auto* ctx = static_cast<ecx_contextt*>(link_.context);

        const int found = ecx_config_init(ctx);
        if (found <= 0)
        {
            DUST_LOG_ERR("ecat: ecx_config_init found %d slaves", found);
            return false;
        }

        link_.discovered = true;

        DUST_LOG_INF("ecat: discovered %d slaves", found);

        return true;
    }

    /**
     * @brief 枚举从站并填 SlaveInfo
     *
     * @return 枚举到的从站数量
     */
    uint16_t Enumerate()
    {
        auto* ctx = static_cast<ecx_contextt*>(link_.context);

        (void)ecx_readstate(ctx);

        slaves_.clear();
        slaves_.reserve(ctx->slavecount);

        for (uint16_t i = 1; i <= ctx->slavecount; ++i)
        {
            auto& slave = ctx->slavelist[i];

            SlaveInfo info {};
            info.index        = i;
            info.configadr    = slave.configadr;
            info.name         = slave.name;
            info.al_state     = slave.state;
            info.al_status    = slave.ALstatuscode;
            info.output_bytes = slave.Obytes;
            info.input_bytes  = slave.Ibytes;

            slaves_.push_back(info);

            DUST_LOG_INF("ecat: slave %u name=%s configadr=0x%04x state=0x%02x alstatus=0x%04x out=%u B in=%u B",
                         static_cast<unsigned>(i),
                         info.name.c_str(),
                         static_cast<unsigned>(info.configadr),
                         static_cast<unsigned>(info.al_state),
                         static_cast<unsigned>(info.al_status),
                         static_cast<unsigned>(info.output_bytes),
                         static_cast<unsigned>(info.input_bytes));
        }

        return SlaveCount();
    }

    /**
     * @brief 读回 mailbox 与过程数据 SM 配置
     *
     * @return true=mailbox SM 已启用
     */
    bool ConfigureSlaves()
    {
        auto* ctx    = static_cast<ecx_contextt*>(link_.context);
        bool  all_ok = true;

        for (uint16_t i = 1; i <= SlaveCount(); ++i)
        {
            const uint16_t configadr = ctx->slavelist[i].configadr;

            ec_smt    sm[4] = {};
            const int wkc   = ecx_FPRD(&ctx->port, configadr, ECT_REG_SM0, sizeof(sm), sm, EC_TIMEOUTRET3);

            const uint16_t sm0 = etohs(sm[0].SMlength);
            const uint16_t sm1 = etohs(sm[1].SMlength);
            const uint16_t sm2 = etohs(sm[2].SMlength);
            const uint16_t sm3 = etohs(sm[3].SMlength);

            DUST_LOG_INF("ecat: slave %u SM read wkc=%d SM0=%u SM1=%u SM2=%u SM3=%u",
                         static_cast<unsigned>(i),
                         wkc,
                         static_cast<unsigned>(sm0),
                         static_cast<unsigned>(sm1),
                         static_cast<unsigned>(sm2),
                         static_cast<unsigned>(sm3));

            if (wkc <= 0)
            {
                DUST_LOG_ERR("ecat: slave %u SM read failed, wkc=%d", static_cast<unsigned>(i), wkc);
                all_ok = false;
                continue;
            }
            if (sm0 == 0 || sm1 == 0)
            {
                DUST_LOG_ERR("ecat: slave %u mailbox SM not enabled, SM0=%u SM1=%u",
                             static_cast<unsigned>(i),
                             static_cast<unsigned>(sm0),
                             static_cast<unsigned>(sm1));
                all_ok = false;
            }
        }

        return all_ok;
    }

    /**
     * @brief 映射 IOmap、写 SM2/SM3、计算期望 WKC
     *
     * @return true=映射成功
     */
    bool ConfigurePdo()
    {
        auto* ctx = static_cast<ecx_contextt*>(link_.context);

        link_.io_map.assign(kIoMapBytes, 0);

        const int mapped = ecx_config_map_group(ctx, link_.io_map.data(), 0);
        DUST_LOG_INF("ecat: ecx_config_map_group mapped %d bytes (io_map %u bytes)", mapped, static_cast<unsigned>(link_.io_map.size()));

        if (mapped <= 0)
        {
            DUST_LOG_ERR("ecat: ecx_config_map_group failed, mapped=%d", mapped);
            return false;
        }
        if (static_cast<uint32_t>(mapped) > link_.io_map.size())
        {
            DUST_LOG_ERR("ecat: IOmap too small, mapped %d > buffer %u",
                         mapped, static_cast<unsigned>(link_.io_map.size()));
            return false;
        }

        for (uint16_t i = 1; i <= SlaveCount(); ++i)
        {
            auto&      slave = ctx->slavelist[i];
            SlaveInfo& info  = slaves_[i - 1];

            if (slave.Obytes > 0)
            {
                slave.SM[2].SMlength = htoes(static_cast<uint16_t>(slave.Obytes));
                const int wkc = ecx_FPWR(&ctx->port, slave.configadr, ECT_REG_SM2, sizeof(ec_smt), &slave.SM[2], EC_TIMEOUTRET3);
                DUST_LOG_INF("ecat: slave %u SM2 rewrite to %u B, wkc=%d", static_cast<unsigned>(i), static_cast<unsigned>(slave.Obytes), wkc);
            }
            if (slave.Ibytes > 0)
            {
                slave.SM[3].SMlength = htoes(static_cast<uint16_t>(slave.Ibytes));
                const int wkc = ecx_FPWR(&ctx->port, slave.configadr, ECT_REG_SM3, sizeof(ec_smt), &slave.SM[3], EC_TIMEOUTRET3);
                DUST_LOG_INF("ecat: slave %u SM3 rewrite to %u B, wkc=%d", static_cast<unsigned>(i), static_cast<unsigned>(slave.Ibytes), wkc);
            }

            info.output_bytes  = slave.Obytes;
            info.input_bytes   = slave.Ibytes;
            info.output_offset = (slave.outputs != nullptr) ? static_cast<uint32_t>(slave.outputs - link_.io_map.data()) : 0U;
            info.input_offset  = (slave.inputs != nullptr) ? static_cast<uint32_t>(slave.inputs - link_.io_map.data()) : 0U;

            DUST_LOG_INF("ecat: slave %u io_map offset out=%u in=%u", static_cast<unsigned>(i), static_cast<unsigned>(info.output_offset), static_cast<unsigned>(info.input_offset));
        }

        // 期望 WKC 取自 SOEM 按组算好的值：输出读写各计 1、输入计 1
        pdo_.expected_wkc = static_cast<uint32_t>(ctx->grouplist[0].outputsWKC) * 2U + static_cast<uint32_t>(ctx->grouplist[0].inputsWKC);

        pdo_.configured = true;

        DUST_LOG_INF("ecat: expected WKC=%u (outWKC=%u inWKC=%u)", static_cast<unsigned>(pdo_.expected_wkc), static_cast<unsigned>(ctx->grouplist[0].outputsWKC), static_cast<unsigned>(ctx->grouplist[0].inputsWKC));

        return true;
    }

    /**
     * @brief 按同步模式配置分布式时钟
     *
     * @return true=配置成功（非 Dc 模式直接成功）
     */
    bool ConfigureDc()
    {
        if (config_.sync_mode != SyncMode::Dc)
        {
            DUST_LOG_INF("ecat: sync mode %s, DC disabled", SyncModeName(config_.sync_mode));
            return true;
        }

        auto* ctx = static_cast<ecx_contextt*>(link_.context);
        const int ret = ecx_configdc(ctx);

        if (ret <= 0)
        {
            DUST_LOG_ERR("ecat: ecx_configdc returned %d", ret);
            return false;
        }

        DUST_LOG_INF("ecat: DC configured");

        return true;
    }

    /**
     * @brief 请求单个从站的目标状态
     *
     * 请求必须真正写进去才算发起：写返回 <=0 时立即重写，绝不用超时空等。
     *
     * @param index 从站序号（1 起）
     * @param state 目标 AL 状态
     * @return true=状态已到达
     */
    bool RequestState(uint16_t index, uint16_t state)
    {
        auto*          ctx     = static_cast<ecx_contextt*>(link_.context);
        const uint16_t configadr = ctx->slavelist[index].configadr;
        const auto     begin = std::chrono::steady_clock::now();

        while (true)
        {
            ctx->slavelist[index].state = state;
            const int write_wkc         = ecx_writestate(ctx, index);

            uint16_t  alctl    = 0;
            const int read_wkc = ecx_FPRD(&ctx->port, configadr, ECT_REG_ALCTL, sizeof(alctl), &alctl, EC_TIMEOUTRET3);

            DUST_LOG_DBG("ecat", "ALCTL write slave=%u state=0x%02x wkc=%d readback wkc=%d value=0x%04x",
                         static_cast<unsigned>(index),
                         static_cast<unsigned>(state),
                         write_wkc,
                         read_wkc,
                         static_cast<unsigned>(etohs(alctl)));

            if (write_wkc > 0)
            {
                DUST_LOG_INF("ecat: slave %u request state 0x%02x, readback=0x%04x", static_cast<unsigned>(index), static_cast<unsigned>(state), static_cast<unsigned>(etohs(alctl)));
                break;
            }

            DUST_LOG_WRN("ecat: slave %u ALCTL write wkc=%d, rewriting", static_cast<unsigned>(index), write_wkc);

            const uint32_t elapsed_ms = ElapsedMs(begin);
            if (elapsed_ms >= kStateWriteTimeoutMs)
            {
                DUST_LOG_ERR("ecat: slave %u state request never accepted, %u ms", static_cast<unsigned>(index), static_cast<unsigned>(elapsed_ms));
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(kWriteRetryYieldMs));
        }

        // 切 OP 期间必须持续交换过程数据：从站要收到首帧输出才放行 SAFE-OP → OP
        while (true)
        {
            const int sent = ecx_send_processdata(ctx);
            const int wkc  = ecx_receive_processdata(ctx, static_cast<int>(config_.rx_timeout_us));

            const uint16_t current = ecx_statecheck(ctx, index, state, static_cast<int>(kStateCheckUs));

            DUST_LOG_DBG("ecat", "wait state slave=%u sent=%d wkc=%d expected=%u state=0x%02x alstatus=0x%04x",
                         static_cast<unsigned>(index),
                         sent,
                         wkc,
                         static_cast<unsigned>(pdo_.expected_wkc),
                         static_cast<unsigned>(current),
                         static_cast<unsigned>(ctx->slavelist[index].ALstatuscode));

            if (current == state)
            {
                slaves_[index - 1].al_state  = current;
                slaves_[index - 1].al_status = ctx->slavelist[index].ALstatuscode;
                DUST_LOG_OK("ecat: slave %u reached state 0x%02x", static_cast<unsigned>(index), static_cast<unsigned>(state));
                return true;
            }

            const uint32_t elapsed_ms = ElapsedMs(begin);
            if (elapsed_ms >= kStateWriteTimeoutMs)
            {
                DUST_LOG_ERR("ecat: slave %u state 0x%02x not reached in %u ms, state=0x%02x alstatus=0x%04x sent=%d wkc=%d",
                             static_cast<unsigned>(index),
                             static_cast<unsigned>(state),
                             static_cast<unsigned>(elapsed_ms),
                             static_cast<unsigned>(current),
                             static_cast<unsigned>(ctx->slavelist[index].ALstatuscode),
                             sent,
                             wkc);
                return false;
            }
        }
    }

    /**
     * @brief 全部从站进入 OP
     *
     * @return true=全部到达 OP
     */
    bool RequestOperational()
    {
        const auto begin = std::chrono::steady_clock::now();

        for (uint16_t i = 1; i <= SlaveCount(); ++i)
        {
            if (!RequestState(i, EC_STATE_OPERATIONAL))
            {
                DUST_LOG_ERR("ecat: slave %u failed to enter OP", static_cast<unsigned>(i));
                return false;
            }
        }

        DUST_LOG_OK("ecat: all %u slaves in OP in %u ms", static_cast<unsigned>(SlaveCount()), static_cast<unsigned>(ElapsedMs(begin)));

        return true;
    }

    /**
     * @brief 同步模式转字符串
     *
     * @param mode 同步模式
     * @return 模式名
     */
    static const char* SyncModeName(SyncMode mode)
    {
        switch (mode)
        {
            case SyncMode::FreeRun:
                return "freerun";
            case SyncMode::SmSync:
                return "smsync";
            case SyncMode::Dc:
                return "dc";
        }

        return "unknown";
    }

    /**
     * @brief 从起点到现在的毫秒数
     *
     * @param begin 起点
     * @return 毫秒数
     */
    static uint32_t ElapsedMs(const std::chrono::steady_clock::time_point& begin)
    {
        return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count());
    }

    /**
     * @brief 找第一个有载波、非回环、非无线的网口
     *
     * 载波由内核 SIOCGIFFLAGS 的 IFF_RUNNING 位反映，网线没插的口不会置位。
     *
     * @return 网口名；找不到返回空串
     */
    static std::string FindCarrierInterface()
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
};

} // namespace drivers::ethercat
