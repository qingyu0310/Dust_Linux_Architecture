/**
 * @file thread_fault.cpp
 * @author qingyu
 * @brief 线程故障默认处置函数的弱实现
 * @version 0.1
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "thread_fault.hpp"
#include "thread.hpp"
#include "thread_registry.hpp"
#include "log.hpp"

/**
 * @brief 默认处置：按线程声明的策略兜底
 *
 * 只在 RetryHook 处理不了故障时进入，即「任务自己已经救不回来了」。
 *
 * Linux 版是协作式：Exit() 只请求停止，不 join、不立即注销；
 * 亡语与注销由目标线程自己的 Run() 收尾路径完成。
 *
 * @param thread 出故障的线程
 */
void DefaultHandle(init::ThreadBase* thread)
{
    if (thread == nullptr)
    {
        DUST_LOG_ERR("thread fault with null thread");
        System_RequestStop(1);
        return;
    }

    switch (thread->FaultPolicy())
    {
        case ThreadFaultPolicy::Isolate:
        {
            // 影响小：收尾后自己退出，不拖累系统
            DUST_LOG_WRN("thread %s fault, isolating", thread->Name());
            thread->Exit();
            return;
        }

        case ThreadFaultPolicy::Recover:
        default:
        {
            // 影响大且无法恢复：停掉全部业务线程，shell 留下供观察
            DUST_LOG_ERR("thread %s fatal, stopping all threads", thread->Name());

            for (uint8_t i = 0; i < thread_registry::Count(); ++i)
            {
                init::ThreadBase* t = thread_registry::At(i);
                if (t == nullptr || t == thread || t->KeepAlive())
                {
                    continue;
                }
                t->Exit();      // 协作式：只请求停止，注册表项由目标线程自己移除
            }
            return;             // 故障线程自己从 Run 返回，进程继续（shell 存活）
        }
    }
}
