/**
 * @file trd_test.cpp
 * @author qingyu
 * @brief 骨架自检线程 —— 周期推进仿真时间并输出 VOFA 波形
 * @version 0.1
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "Init_entry.hpp"
#include "log.hpp"
#include "thread.hpp"
#include <cmath>
#include <cstdint>

namespace thread::test
{

static init::Thread thread_ {};         // 自检线程
static float        t_ = 0.0f;          // 跨拍状态：受管任务每拍重新进入函数

/**
 * @brief 任务单拍：推进仿真时间并输出一组双通道波形
 *
 * 不依赖任何外设，只验证线程模板、周期等待与日志链路是否装配成功。
 *
 * @param context 未使用
 * @return Continue=正常
 */
static init::Thread::TaskResult Task(void* context)
{
    (void)context;

    t_ += 0.01f;

    const float ch0 = std::sinh(t_);
    const float ch1 = std::cosh(t_) * 2.0f;

    DUST_LOG_DBG("test", "ch0=%f ch1=%f", static_cast<double>(ch0), static_cast<double>(ch1));

    return init::Thread::TaskResult::Continue;
}

/**
 * @brief 初始化：骨架自检没有需要打开的资源
 * @return true=成功
 */
bool thread_init()
{
    return true;
}

/**
 * @brief 启动自检线程
 *
 * 自检挂了不影响任何业务，按 Isolate 处置：收尾后只退自己。
 *
 * @return true=成功
 */
bool thread_start()
{
    thread_.Start(Task, std::chrono::milliseconds(10), nullptr, "test",
                  ThreadFaultConfig{nullptr, nullptr, ThreadFaultPolicy::Isolate});
    return true;
}

/**
 * @brief 通知自检线程停止并解除等待
 */
void thread_stop()
{
    thread_.RequestStop();
}

/**
 * @brief 等待自检线程退出
 */
void thread_join()
{
    thread_.Join();
}

REGISTER_INIT(thread_init, LateInit, High, HaltOnFail, nullptr, "test_init");
REGISTER_THREAD(thread_start, LateThread, thread_stop, thread_join, "test_start");

} // namespace thread::test
