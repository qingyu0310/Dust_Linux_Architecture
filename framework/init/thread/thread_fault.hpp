/**
 * @file thread_fault.hpp
 * @author qingyu
 * @brief 线程故障策略、钩子类型、配置结构与默认处置函数
 * @version 0.1
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include <exception>

namespace init
{
class ThreadBase;
class Thread;
}

/**
 * @brief 线程故障策略：声明本线程出故障时的影响面
 *
 * 同一个故障对不同线程的影响面不同：自检挂了系统照跑，总线挂了必须全停。
 * 线程在 Start() 时通过 ThreadFaultConfig 传入，DefaultHandle() 据此决定处置方式。
 */
enum class ThreadFaultPolicy : uint8_t
{
    Isolate = 0,    // 影响小：收尾后退出，不拖累系统
    Recover = 1,    // 影响大：救不回来时停掉全部业务线程，保留 shell
};

/**
 * @brief 异常恢复结果：正常停止由线程停止标志单独判断
 */
enum class RecoveryAction
{
    Restart,    	// 恢复成功，重新执行任务入口
    Stop        	// 无法恢复，交给 DefaultHandle 按策略处置
};

/**
 * @brief 异常恢复钩子：Task 抛异常或返回 Retry 时由线程自己就地处理
 *
 * Linux 版带异常与线程引用（父架构无异常，故为 bool(void*)）：
 * 业务需要按异常类型判可恢复性，或靠 Thread& 查询停止状态后重连。
 */
using RetryHook = RecoveryAction (*)(const std::exception&, init::Thread&, void*);

/**
 * @brief 退出钩子（亡语）：线程临死前最后一次发言
 *
 * 只负责收尾与交代后事，不负责杀线程。由 Run() 的收尾路径唯一触发。
 */
using ExitHook = void (*)(void* context);

/**
 * @brief 线程故障配置
 *
 * 由 Thread::Start() 的第 5 个参数传入，与线程名同处一个调用点。
 */
struct ThreadFaultConfig
{
    RetryHook         retry      = nullptr;                        // 异常恢复钩子
    ExitHook          exit       = nullptr;                        // 亡语
    ThreadFaultPolicy policy     = ThreadFaultPolicy::Recover;     // 故障策略
    bool              keep_alive = false;                          // Recover 处置时是否跳过本线程
};

/**
 * @brief 默认故障处置函数：按线程声明的策略兜底
 *
 * 弱符号，项目可在自己编译单元里重定义同名函数整体替换。
 *
 * @param thread 出故障的线程
 */
__attribute__((weak)) void DefaultHandle(init::ThreadBase* thread);
