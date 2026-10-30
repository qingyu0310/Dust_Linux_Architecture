/**
 * @file trd_ethercat.cpp
 * @author qingyu
 * @brief EtherCAT 周期线程 —— 建链与过程数据收发
 * @version 3.2
 * @date 2026-10-30
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "Init_entry.hpp"
#include "log.hpp"
#include "thread.hpp"
#include "framework/drivers/communication/ethercat/ecat.hpp"

namespace thread::ethercat
{

/**
 * @brief 过程数据：主站写给从站（RxPDO）
 *
 * 从站固件按 8 个 UDINT、合计 32 字节配置。
 */
struct PdoOutputs
{
    uint32_t counter[8];    // 输出计数器
};

/**
 * @brief 过程数据：从站回给主站（TxPDO）
 *
 * 从站固件按 8 个 UDINT、合计 32 字节配置。
 */
struct PdoInputs
{
    uint32_t counter[8];    // 输入计数器
};

static drivers::ethercat::Ecat<PdoOutputs, PdoInputs> ecat_ {};    	// 主站实例
static init::Thread thread_ {};   									// 周期线程

/**
 * @brief 任务单拍：填输出、发一帧、收一帧
 *
 * 只走正常路径，失败一律返回 Retry，处置交给 OnRetry。
 *
 * @param context 未使用
 * @return Continue=本拍完成；Retry=交给 OnRetry 处置
 */
static init::Thread::TaskResult Task(void* context)
{
    (void)context;

    if (!ecat_.IsReady())
    {
        DUST_LOG_WRN("ecat: task fail: bus not ready");
        return init::Thread::TaskResult::Retry;
    }

    for (uint32_t i = 0; i < 8; ++i)
    {
        ecat_.OutputsRef().counter[i] = i;
    }

    if (!ecat_.Send())
    {
        DUST_LOG_WRN("ecat: task fail: send failed");
        return init::Thread::TaskResult::Retry;
    }

    if (ecat_.Read() < 0)
    {
        DUST_LOG_WRN("ecat: task fail: read failed");
        return init::Thread::TaskResult::Retry;
    }

    DUST_LOG_DBG("ethercat", "out[0]=0x%08x in[0]=0x%08x", ecat_.OutputsRef().counter[0], ecat_.InputsRef().counter[0]);

    return init::Thread::TaskResult::Continue;
}

/**
 * @brief 链路故障处置：拆链后重新建链
 *
 * 不阻塞，停机请求由外壳判断。
 *
 * @param error   框架构造的异常
 * @param thread  出故障的线程，本钩子不使用
 * @param context 未使用
 * @return Restart=重进任务
 */
static RecoveryAction OnRetry(const std::exception& error, init::Thread& thread, void* context)
{
    (void)thread;
    (void)context;

    DUST_LOG_WRN("ecat: task retry: %s", error.what());

    ecat_.Close();

    try
    {
        if (!ecat_.Init({}))
        {
            DUST_LOG_ERR("ecat: bring-up failed on %s", ecat_.InterfaceName().c_str());
        }
    }
    catch (const std::exception& bring_up_error)
    {
        DUST_LOG_ERR("ecat: bring-up exception: %s", bring_up_error.what());
    }

    return RecoveryAction::Restart;
}

/**
 * @brief 建链，没建链成功不放行
 *
 * @return true=全部从站进入 OP
 */
bool thread_init()
{
    while (!ecat_.Init({}))
    {
        DUST_LOG_ERR("ecat: init failed, retry in 10 ms");
        std::this_thread::sleep_for(DUST_MSEC(1000));
    }

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
    thread_.Start(Task, DUST_MSEC(1), nullptr, "ethercat", ThreadFaultConfig{OnRetry, nullptr, ThreadFaultPolicy::Recover, false});

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
    ecat_.Close();
}

REGISTER_INIT(thread_init, MidInit, High, HaltOnFail, Release, "ethercat_init");
REGISTER_THREAD(thread_start, MidThread, thread_stop, thread_join, "ethercat_start");

} // namespace thread::ethercat
