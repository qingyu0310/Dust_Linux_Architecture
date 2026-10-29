/**
 * @file Init_entry.cpp
 * @author qingyu
 * @brief 统一启动实现：注册表遍历、启动耗时记录、停止请求和逆序资源清理
 * @version 0.2
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "Init_entry.hpp"
#include "System_startup.hpp"
#include "framework/init/signal_handler.hpp"
#include "thread_registry.hpp"
#include "boot_timing.hpp"
#include "framework/shell/log/log.hpp"
#include <atomic>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <vector>

// 链接脚本导出指针表边界；注册结构不直接按段内字节布局遍历。
extern "C"
{
    extern const init::InitEntry*   const __user_init_start[];
    extern const init::InitEntry*   const __user_init_end[];
    extern const init::ThreadEntry* const __user_thread_start[];
    extern const init::ThreadEntry* const __user_thread_end[];
}

namespace
{
    
std::vector<const init::InitEntry*> resources;     // 成功初始化资源，按实际顺序登记
std::vector<const init::ThreadEntry*> threads;     // 已尝试启动线程，包含部分启动失败项
std::atomic<bool> stop{false};                  // 主线程与业务线程共享停止请求
std::atomic<int>  status{0};                    // 退出状态，非零错误不会被正常停止覆盖

bool started = false;                              // 仅主线程访问，不支持重复启动
void (*main_loop)() = nullptr;                     // 主线程事件回调，不持有具体业务对象

/**
 * @brief 同阶段初始化执行顺序，与子树保持 High → Mid → Low
 */
static constexpr init::InitLevel kLevelOrder[] = {
    init::InitLevel::High,
    init::InitLevel::Mid,
    init::InitLevel::Low,
};

/**
 * @brief 从起点到现在的纳秒数，供启动耗时槽使用
 */
uint64_t ElapsedNs(const std::chrono::steady_clock::time_point& begin)
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - begin).count());
}

/**
 * @brief 执行清理回调；异常转为退出错误，继续清理其他资源
 */
void Cleanup(init::CleanupFunc function)
{
    if (!function)
    {
        return;
    }

    try
    {
        function();
    }
    catch (...)
    {
        System_RequestStop(1);
    }
}

} // namespace

/**
 * @brief 线程登记入口：Thread::Start 创建线程后调用
 * @param thread 已创建的线程对象
 */
void init::RegisterThreadInstance(init::ThreadBase* thread)
{
    (void)thread_registry::Add(thread);
}

/**
 * @brief 线程注销入口：Run 收尾时调用
 * @param thread 即将退出的线程对象
 */
void init::UnregisterThreadInstance(init::ThreadBase* thread)
{
    (void)thread_registry::Remove(thread);
}

/**
 * @brief 提交停止请求并保存非零退出状态
 */
void System_RequestStop(int exit_code)
{
    if (exit_code != 0)
    {
        status.store(exit_code);
    }
    stop.store(true);
}

/**
 * @brief 初始化阶段名（阶段横幅用）
 */
static const char* StageName(init::InitStage stage)
{
    switch (stage)
    {
        case init::InitStage::PreInit:   return "PreInit";
        case init::InitStage::EarlyInit: return "EarlyInit";
        case init::InitStage::MidInit:   return "MidInit";
        case init::InitStage::LateInit:  return "LateInit";
        case init::InitStage::AppInit:   return "AppInit";
        default:                         return "?";
    }
}

/**
 * @brief 线程阶段名（阶段横幅用）
 */
static const char* StageName(init::ThreadStage stage)
{
    switch (stage)
    {
        case init::ThreadStage::PreThread:   return "PreThread";
        case init::ThreadStage::EarlyThread: return "EarlyThread";
        case init::ThreadStage::MidThread:   return "MidThread";
        case init::ThreadStage::LateThread:  return "LateThread";
        case init::ThreadStage::AppThread:   return "AppThread";
        default:                             return "?";
    }
}

/**
 * @brief 执行一个初始化阶段，同阶段先 High、再 Mid、最后 Low
 *
 * 同等级允许多项，按链接表顺序遍历；失败策略与等级独立。
 * @param stage 当前初始化阶段
 * @return true 阶段完成；false 启动中止
 */
static bool RunStage(init::InitStage stage)
{
    for (const auto level : kLevelOrder)
    {
        for (auto item = __user_init_start; item < __user_init_end; ++item)
        {
            const auto entry = *item;
            if (entry->stage != stage || entry->level != level)
            {
                continue;
            }
            if (stop.load() || init::SignalStopRequested())
            {
                return false;
            }

            DUST_LOG_INF("[boot] %s init", entry->name);

            const auto begin = std::chrono::steady_clock::now();
            bool success = false;
            try
            {
                success = entry->function();
            }
            catch (const std::exception& error)
            {
                DUST_LOG_ERR("%s: %s", entry->name, error.what());
            }
            catch (...)
            {
                success = false;
            }
            BootTiming::Record(entry->timing, ElapsedNs(begin), success);

            if (success)
            {
                resources.push_back(entry);
                continue;
            }

            Cleanup(entry->cleanup); // 部分初始化失败也须清理本项资源。
            DUST_LOG_ERR("[boot] %s init failed", entry->name);
            if (entry->fail == init::InitFailPolicy::HaltOnFail)
            {
                System_RequestStop(1);
                return false;
            }
        }
    }

    // 阶段横幅：阶段内条目全部执行完再打印，表示哪个阶段初始化完成
    DUST_LOG_OK("=====     [boot] %s ok     =====\n", StageName(stage));
    return true;
}

/**
 * @brief 执行一个线程阶段，按链接表遍历启动项
 *
 * 线程启动项不另设数字优先级，同阶段资源初始化完成后才执行。
 * @param stage 当前线程阶段
 * @return true 阶段完成；false 启动中止
 */
static bool RunStage(init::ThreadStage stage)
{
    for (auto item = __user_thread_start; item < __user_thread_end; ++item)
    {
        const auto entry = *item;
        if (entry->stage != stage)
        {
            continue;
        }
        if (stop.load() || init::SignalStopRequested())
        {
            return false;
        }

        DUST_LOG_OK("[boot] %s start", entry->name);
        threads.push_back(entry); // 覆盖线程部分启动失败后的清理。

        const auto begin = std::chrono::steady_clock::now();
        bool success = false;
        try
        {
            success = entry->start();
        }
        catch (...)
        {
            success = false;
        }
        BootTiming::Record(entry->timing, ElapsedNs(begin), success);

        if (!success)
        {
            System_RequestStop(1);
            return false;
        }
    }

    // 阶段横幅：线程阶段全部启动完再打印
    DUST_LOG_OK("=====     [boot] %s ok     =====\n", StageName(stage));
    return true;
}

/**
 * @brief 系统启动顺序：每对阶段先初始化资源，再启动对应线程
 */
static bool StartupEntries()
{
    if (started)
    {
        throw std::logic_error("startup already called");
    }
    started = true;
    resources.reserve(__user_init_end - __user_init_start);
    threads.reserve(__user_thread_end - __user_thread_start);

    if (!RunStage(init::InitStage::PreInit)   || !RunStage(init::ThreadStage::PreThread))
    {
        return false;
    }
    if (!RunStage(init::InitStage::EarlyInit) || !RunStage(init::ThreadStage::EarlyThread))
    {
        return false;
    }
    if (!RunStage(init::InitStage::MidInit)   || !RunStage(init::ThreadStage::MidThread))
    {
        return false;
    }
    if (!RunStage(init::InitStage::LateInit)  || !RunStage(init::ThreadStage::LateThread))
    {
        return false;
    }
    return RunStage(init::InitStage::AppInit) &&  RunStage(init::ThreadStage::AppThread);
}

/**
 * @brief 准备平台启动上下文并执行注册表，异常统一转为启动失败
 */
bool System_Startup()
{
    try {
        init::InstallSignalHandlers();
    }
    catch (...) {
        // 信号策略尚未建立时，不向可能断管的 stderr 输出。
        System_RequestStop(1);
        return false;
    }

    try {
        return StartupEntries();
    }
    catch (const std::exception& error) {
        System_RequestStop(1);
        DUST_LOG_ERR("%s", error.what());
        return false;
    }
}

/**
 * @brief 装配主线程事件回调，仅由主线程设置或取消
 *
 * @param callback 事件处理函数；nullptr 表示取消
 */
void System_SetMainLoop(void (*callback)())
{
    main_loop = callback;
}

/**
 * @brief 主线程处理已装配的事件回调并等待统一停止
 */
void System_WaitForStop()
{
    while (!stop.load() && !init::SignalStopRequested())
    {
        if (main_loop != nullptr)
        {
            try
            {
                main_loop();
            }
            catch (const std::exception& error)
            {
                System_RequestStop(1);
                DUST_LOG_ERR("main loop: %s", error.what());
            }
            catch (...)
            {
                System_RequestStop(1);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

/**
 * @brief 统一退出：先通知所有线程，再等待退出，最后释放资源
 */
int System_Shutdown()
{
    stop.store(true);

    for (auto it = threads.rbegin(); it != threads.rend(); ++it)
    {
        Cleanup((*it)->request_stop);
    }
    for (auto it = threads.rbegin(); it != threads.rend(); ++it)
    {
        Cleanup((*it)->join);
    }
    threads.clear();

    for (auto it = resources.rbegin(); it != resources.rend(); ++it)
    {
        Cleanup((*it)->cleanup);
    }

    resources.clear();
    return status.load();
}
