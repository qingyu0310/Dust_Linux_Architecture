/**
 * @file trd_ethercat.cpp
 * @author qingyu
 * @brief EtherCAT 周期线程 —— 过程数据收发、工作计数器看护与链路重连
 * @version 0.3
 * @date 2026-10-30
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "Init_entry.hpp"
#include "log.hpp"
#include "thread.hpp"

#include "framework/cmd/config/config.hpp"
#include "framework/drivers/communication/ethercat/ethercat.hpp"

#include <chrono>
#include <cstdint>
#include <ctime>
#include <exception>

namespace thread::ethercat
{

static drivers::ethercat::Master  master_  {};  		// 主站实例
static drivers::ethercat::Config  config_  {};  		// YAML 读出的主站配置
static drivers::ethercat::Outputs outputs_ {}; 			// 待写入从站的输出过程数据
static drivers::ethercat::Inputs  inputs_  {};  		// 从站回给主站的输入过程数据

static uint32_t           cycle_us_          = 1000;  	// 通信周期（µs）
static uint32_t           fail_count_        = 0;     	// 连续 WKC 失败拍数
static uint32_t           reconnect_count_   = 0;     	// 总线重连尝试次数
static uint32_t           reconnect_wait_    = 0;     	// 距下次重连还要等几拍
static constexpr uint32_t kFailLimit         = 10;    	// 连续失败上限
static constexpr uint32_t kReconnectCycles   = 1000;  	// 重连间隔（拍数，1ms 周期下 = 1s）
static constexpr uint32_t kReconnectLogEvery = 10;    	// 每 N 次重连打一条日志

static init::Thread thread_ {};           				// 周期线程

/**
 * @brief 任务单拍：写输出、发一帧、收一帧、读输入
 *
 * 只走正常路径。链路未建、发送失败、WKC 不达标一律返回 Retry，
 * 由 OnRetry 决定是重建链路还是继续重试。
 *
 * @param context 未使用
 * @return Continue=本拍完成；Retry=交给 OnRetry 处置
 */
static init::Thread::TaskResult Task(void* context)
{
    (void)context;

    if (master_.SlaveCount() == 0)
    {
        return init::Thread::TaskResult::Retry;   // 未建链，交给 OnRetry
    }

    for (uint32_t i = 0; i < 8; ++i)
    {
        outputs_.counter[i] = i;
    }

    if (!master_.SetOutputs(1, outputs_))
    {
        return init::Thread::TaskResult::Retry;   // 从站 PDO 长度与结构体不符
    }

    if (!master_.Send() || !master_.Receive())
    {
        return init::Thread::TaskResult::Retry;
    }

    if (!master_.GetInputs(1, inputs_))
    {
        return init::Thread::TaskResult::Retry;
    }

    DUST_LOG_DBG("ethercat", "out[0]=0x%08x in[0]=0x%08x", outputs_.counter[0], inputs_.counter[0]);

    fail_count_ = 0;

    return init::Thread::TaskResult::Continue;
}

/**
 * @brief 链路故障处置：重建链路、退避重试与连续失败计数都在这里
 *
 * 不阻塞——退避按拍数递减，停机请求由外壳判断。
 *
 * @param error 框架构造的异常，此处只用于说明
 * @param thread 出故障的线程，本钩子不使用
 * @param context 未使用
 * @return Restart=重进任务
 */
static RecoveryAction OnRetry(const std::exception& error, init::Thread& thread, void* context)
{
    (void)error;
    (void)thread;
    (void)context;

    // 退避未到点：直接重进任务
    if (reconnect_wait_ > 0)
    {
        --reconnect_wait_;
        return RecoveryAction::Restart;
    }

    // 未建链：重新枚举从站并切 OP
    if (master_.SlaveCount() == 0)
    {
        ++reconnect_count_;
        DUST_LOG_INF("ethercat: opening bus (attempt %u)", static_cast<unsigned>(reconnect_count_));

        bool linked = false;
        try
        {
            linked = master_.Init(config_) && master_.RequestOperational();
        }
        catch (const std::exception& error)
        {
            DUST_LOG_ERR("ethercat: %s", error.what());
        }

        if (linked)
        {
            fail_count_     = 0;
            reconnect_count_ = 0;
            DUST_LOG_OK("ethercat: %u slaves on %s, cycle=%u us", static_cast<unsigned>(master_.SlaveCount()), master_.InterfaceName().c_str(), static_cast<unsigned>(cycle_us_));
            return RecoveryAction::Restart;
        }

        DUST_LOG_ERR("ethercat: slaves failed to reach OP on %s (attempt %u)", master_.InterfaceName().c_str(), static_cast<unsigned>(reconnect_count_));

        master_.Close();
        reconnect_wait_ = kReconnectCycles;

        return RecoveryAction::Restart;
    }

    // 已建链但收发失败：累计到上限就拆链重来
    if (++fail_count_ >= kFailLimit)
    {
        DUST_LOG_ERR("ethercat: %u consecutive WKC failures", static_cast<unsigned>(fail_count_));
        master_.Close();
        fail_count_     = 0;
        reconnect_wait_ = kReconnectCycles;
        ++reconnect_count_;
    }

    return RecoveryAction::Restart;
}

/**
 * @brief 读取 YAML 参数；总线建链推迟到任务里，没接从站时不拖垮启动
 *
 * @return true=参数读取成功
 */
bool thread_init()
{
    cmd::config::Config yaml;
    yaml.Load("project/thread/ethercat/trd_ethercat.yaml", {"interface", "cycle_us"}, {"timeout_us", "use_dc"});

    config_.interface_name = yaml.GetString("interface");
    config_.timeout_us     = static_cast<uint32_t>(yaml.GetInt("timeout_us", 100, 100000));
    config_.use_dc         = yaml.Has("use_dc") && yaml.GetBool("use_dc");

    cycle_us_ = static_cast<uint32_t>(yaml.GetInt("cycle_us", 100, 100000));

    return true;
}

/**
 * @brief 启动周期线程
 *
 * 总线掉了挂在它上面的所有设备全部失控，只停自己没意义，按 Recover 处置。
 *
 * @return true=启动成功
 */
bool thread_start()
{
    thread_.Start(Task, std::chrono::milliseconds(cycle_us_ / 1000), nullptr, "ethercat", ThreadFaultConfig{OnRetry, nullptr, ThreadFaultPolicy::Recover, false});

    return true;
}

/**
 * @brief 通知周期线程停止
 */
void thread_stop()
{
    thread_.RequestStop();
}

/**
 * @brief 等待周期线程退出后关闭主站
 */
void thread_join()
{
    thread_.Join();
}

/**
 * @brief 关闭主站，支持未初始化与重复调用
 */
void Release()
{
    master_.Close();
}

REGISTER_INIT(thread_init, MidInit, High, HaltOnFail, Release, "ethercat_init");
REGISTER_THREAD(thread_start, MidThread, thread_stop, thread_join, "ethercat_start");

} // namespace thread::ethercat
