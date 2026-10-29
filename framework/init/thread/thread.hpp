/**
 * @file thread.hpp
 * @author qingyu
 * @brief Linux 线程封装：受管任务外壳、运行统计、暂停恢复、停止通知与故障处置
 * @version 0.3
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <stdexcept>
#include <string>
#include <pthread.h>
#include <unistd.h>
#include "framework/init/startup/System_startup.hpp"
#include "framework/shell/log/log.hpp"
#include "thread_fault.hpp"

namespace init
{

/**
 * @brief 线程运行统计快照
 */
struct ThreadStats
{
    const char* name;               // 线程名
    uint32_t    run_count;          // 任务体进入次数
    uint32_t    retry_count;        // 可恢复错误次数
    uint32_t    exit_count;         // 任务主动退出次数
    uint32_t    drop_count;         // 业务丢帧/丢包计数
    uint64_t    run_ns;             // 任务体累计执行时间
    uint64_t    max_run_ns;         // 单次最长执行时间
    uint64_t    last_run_ns;        // 最近一次执行时间
    bool        running;            // 线程是否仍在运行
    bool        paused;             // 是否被暂停
};

/**
 * @brief 线程非模板基类：供注册表与 shell 命令统一访问
 */
class ThreadBase
{
public:
    virtual ~ThreadBase() = default;

    virtual const char* Name() const = 0;
    virtual pthread_t   Tid() const = 0;
    virtual bool        GetStats(ThreadStats& out) const = 0;
    virtual void        ResetStats() = 0;

    /**
     * @brief 暂停线程：在下一拍开始前挂起，保留全部现场
     */
    virtual void Pause() = 0;

    /**
     * @brief 恢复线程：从挂起处继续执行
     */
    virtual void Resume() = 0;

    /**
     * @brief 是否处于暂停状态
     */
    virtual bool Paused() const = 0;

    /**
     * @brief 请求终止线程：置停止标志并唤醒所有等待点
     *
     * 协作式：目标线程从 Run 返回时执行亡语并注销注册表，本函数不 join。
     * 自己调用即自杀（当前拍结束后退出），其他线程调用即请求它退出。
     */
    virtual void Exit() = 0;

    /**
     * @brief 本线程的故障策略
     */
    virtual ThreadFaultPolicy FaultPolicy() const = 0;

    /**
     * @brief 是否带保留标记
     */
    virtual bool KeepAlive() const = 0;
};

/**
 * @brief 线程登记入口（定义在 Init_entry.cpp，供 Thread::Start 调用）
 * @param thread 已创建的线程对象
 */
void RegisterThreadInstance(ThreadBase* thread);

/**
 * @brief 线程注销入口（定义在 Init_entry.cpp，供 Run 收尾调用）
 * @param thread 即将退出的线程对象
 */
void UnregisterThreadInstance(ThreadBase* thread);

/**
 * @brief Linux 线程与可中断等待封装
 *
 * 拥有者先 RequestStop，再 Join，最后释放线程访问的资源。
 * 析构执行停止与等待；不得在被管理线程内部析构本对象。
 */
class Thread final : public ThreadBase
{
public:
    /**
     * @brief 析构时通知停止并等待线程退出
     */
    ~Thread() override
    {
        RequestStop();
        Join();
    }

    Thread() = default;
    Thread(const Thread&) = delete;
    Thread& operator=(const Thread&) = delete;

    /**
     * @brief 受管任务单拍结果，代替异常
     */
    enum class TaskResult : uint8_t
    {
        Continue = 0,   // 本拍正常结束：外壳等下一拍再调一次
        Retry    = 1,   // 可恢复错误：交给 RetryHook
        Exit     = 2,   // 需要退出：跳过 RetryHook，直接交给 DefaultHandle
    };

    /**
     * @brief 受管任务入口，每拍调用一次
     */
    using TaskFunc = TaskResult (*)(void* context);

    /**
     * @brief 启动受管任务：外壳负责循环、周期等待与停止判断，任务体只写一拍
     *
     * @param entry 任务入口，每拍调用一次
     * @param period 两拍之间的等待时长；0 表示任务自己阻塞，不额外等待
     * @param context 任务上下文，每拍原样传回
     * @param name Linux 线程名，最多 15 字节
     * @param fault 故障配置（钩子 / 策略 / 保留标记），不传即全默认
     * @throws std::exception 入口无效、线程未回收或创建失败
     */
    void Start(TaskFunc entry, std::chrono::milliseconds period = std::chrono::milliseconds(0), void* context = nullptr, const char* name = nullptr, const ThreadFaultConfig& fault = {})
    {
        if (entry == nullptr)
        {
            throw std::invalid_argument("thread entry is null");
        }
        if (life_.worker.joinable())
        {
            throw std::logic_error("thread already started");
        }

        life_.name = name == nullptr ? "" : name;
        if (life_.name.size() > 15)
        {
            throw std::invalid_argument("thread name exceeds 15 bytes");
        }

        {
            std::lock_guard<std::mutex> lock(stop_.mutex);
            stop_.stop = false;
        }

        task_.entry = entry;
        task_.period = period;
        task_.context = context;
        fault_ = fault;
        life_.worker = std::thread(&Thread::Run, this);

        RegisterThreadInstance(this);   // 登记到注册表，供 shell 的 thread 命令遍历
    }

    /**
     * @brief 绑定停止唤醒描述符，供以 poll 等待的业务线程使用
     *
     * @param fd 可写文件描述符；负数表示不启用
     * @note 必须在 Start() 之前调用；描述符由调用方创建和关闭，本类不做回收
     */
    void SetWakeFd(int fd)
    {
        stop_.wake_fd = fd;
    }

    /**
     * @brief 设置停止标志并唤醒等待中的业务线程
     */
    void RequestStop()
    {
        {
            std::lock_guard<std::mutex> lock(stop_.mutex);
            stop_.stop = true;
        }
        stop_.condition.notify_all();

        {
            std::lock_guard<std::mutex> lock(pause_.mutex);
        }
        pause_.condition.notify_all();   // 暂停中的线程也要能醒来退出

        if (stop_.wake_fd >= 0)
        {
            const uint64_t one = 1;
            const auto written = ::write(stop_.wake_fd, &one, sizeof(one));
            (void)written; // EAGAIN 表示计数已饱和，等待方本来就可读
        }
    }

    /**
     * @brief 休眠指定时长，让出 CPU
     *
     * @param duration 休眠时长
     */
    void Sleep(std::chrono::milliseconds duration)
    {
        std::this_thread::sleep_for(duration);
    }

    /**
     * @brief 读取停止标志，供 poll 返回后判断是否退出
     *
     * @return true 已收到停止请求
     */
    bool StopRequested() const
    {
        return stop_.stop.load();
    }

    /**
     * @brief 等待线程退出；仅由拥有者调用，不能在线程自身调用
     */
    void Join()
    {
        if (life_.worker.joinable())
        {
            life_.worker.join();
        }
    }

    const char* Name() const override
    {
        return life_.name.c_str();
    }

    pthread_t Tid() const override
    {
        return life_.tid.load();
    }

    /**
     * @brief 取运行统计快照
     *
     * @param out 输出
     * @return true=已启动过，数据有效
     */
    bool GetStats(ThreadStats& out) const override
    {
        out.name        = life_.name.c_str();
        out.run_count   = counters_.run_count.load();
        out.retry_count = counters_.retry_count.load();
        out.exit_count  = counters_.exit_count.load();
        out.drop_count  = counters_.drop_count.load();
        out.run_ns      = counters_.run_ns.load();
        out.max_run_ns  = counters_.max_run_ns.load();
        out.last_run_ns = counters_.last_run_ns.load();
        out.running     = life_.started.load() && !life_.finished.load();
        out.paused      = Paused();

        return life_.started.load();
    }

    /**
     * @brief 清零运行统计（不清线程生命周期与暂停状态）
     */
    void ResetStats() override
    {
        counters_.run_count.store(0);
        counters_.retry_count.store(0);
        counters_.exit_count.store(0);
        counters_.drop_count.store(0);
        counters_.run_ns.store(0);
        counters_.max_run_ns.store(0);
        counters_.last_run_ns.store(0);
    }

    /**
     * @brief 暂停线程：在下一拍开始前挂起，保留全部现场
     *
     * @note Linux 没有内核级挂起，挂起点在受管外壳的拍与拍之间；正在执行的那一拍会先跑完。
     */
    void Pause() override
    {
        std::lock_guard<std::mutex> lock(pause_.mutex);
        pause_.paused = true;
    }

    /**
     * @brief 恢复被暂停的线程，从挂起处继续执行
     */
    void Resume() override
    {
        {
            std::lock_guard<std::mutex> lock(pause_.mutex);
            pause_.paused = false;
        }
        pause_.condition.notify_all();
    }

    bool Paused() const override
    {
        std::lock_guard<std::mutex> lock(pause_.mutex);
        return pause_.paused;
    }

    /**
     * @brief 请求终止线程：置停止标志并唤醒所有等待点
     *
     * 协作式：目标线程从 Run 返回时执行亡语并注销注册表，本函数不 join。
     * 带保留标记的线程（shell）拒绝退出。
     */
    void Exit() override
    {
        if (fault_.keep_alive)
        {
            DUST_LOG_WRN("thread %s is reserved, exit refused", Name());
            return;
        }
        RequestStop();
    }

    /**
     * @brief 本线程的故障策略
     */
    ThreadFaultPolicy FaultPolicy() const override
    {
        return fault_.policy;
    }

    /**
     * @brief 是否带保留标记
     */
    bool KeepAlive() const override
    {
        return fault_.keep_alive;
    }

    /**
     * @brief 业务侧计数一次丢帧/丢包
     */
    void CountDrop()
    {
        counters_.drop_count.fetch_add(1);
    }

private:
    /**
     * @brief 线程句柄与生命周期状态
     */
    struct Lifecycle
    {
        std::string            name;                    // 线程名，最多 15 字节
        std::thread            worker;                  // 工作线程
        std::atomic<bool>      started  {false};     // 是否已启动过
        std::atomic<bool>      finished {false};     // 线程入口是否已退出
        std::atomic<pthread_t> tid {};                  // 线程号
    };

    /**
     * @brief 停止请求与等待
     */
    struct StopSignal
    {
        std::mutex              mutex;                  // 保护停止标志
        std::atomic<bool>       stop {false};        // 停止请求
        std::condition_variable condition;              // 停止时唤醒等待者
        int                     wake_fd = -1;           // 停止唤醒描述符；-1 表示业务线程不使用 poll 等待
    };

    /**
     * @brief 拍间暂停状态
     */
    struct PauseState
    {
        bool               paused = false;              // 拍间挂起标志
        mutable std::mutex mutex;                       // 保护暂停标志
        std::condition_variable condition;              // 暂停/恢复时唤醒外壳
    };

    /**
     * @brief 受管任务与异常恢复
     */
    struct TaskSlot
    {
        TaskFunc     entry = nullptr;                   // 受管任务入口
        void*        context = nullptr;                 // 受管任务上下文
        std::chrono::milliseconds period {0};           // 两拍之间的等待时长；0 表示不额外等待
    };

    /**
     * @brief 运行统计计数
     */
    struct Counters
    {
        std::atomic<uint32_t> run_count   {0};      // 任务体进入次数
        std::atomic<uint32_t> retry_count {0};      // 可恢复错误次数
        std::atomic<uint32_t> exit_count  {0};      // 任务主动退出次数
        std::atomic<uint32_t> drop_count  {0};      // 业务计数
        std::atomic<uint64_t> run_ns      {0};      // 累计执行时间
        std::atomic<uint64_t> max_run_ns  {0};      // 单次最长执行时间
        std::atomic<uint64_t> last_run_ns {0};      // 最近一次执行时间
    };

    Lifecycle        life_;
    StopSignal       stop_;
    PauseState       pause_;
    TaskSlot         task_;
    Counters         counters_;
    ThreadFaultConfig fault_ {};                        // 故障处置（Start 时由参数定死）

    /**
     * @brief 拍间挂起，返回 false 表示挂起期间收到了停止请求
     */
    bool WaitWhilePaused()
    {
        std::unique_lock<std::mutex> lock(pause_.mutex);
        pause_.condition.wait(lock, [this] { return !pause_.paused || stop_.stop.load(); });

        return !stop_.stop.load();
    }

    /**
     * @brief 记录一拍耗时
     */
    void RecordRun(std::chrono::steady_clock::duration elapsed)
    {
        const auto ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count());
        counters_.run_ns.fetch_add(ns);
        counters_.last_run_ns.store(ns);

        uint64_t longest = counters_.max_run_ns.load();
        while (ns > longest && !counters_.max_run_ns.compare_exchange_weak(longest, ns)) {}
    }

    /**
     * @brief 公共线程入口：设置名称、循环调用任务并统一处理异常与退出
     *
     * 停止请求只在这里判断，任务体不感知；任务体抛出的异常交给恢复钩子，
     * 恢复失败则请求统一退出。
     */
    void Run()
    {
        life_.started.store(true);
        life_.tid.store(pthread_self());

        try
        {
            if (!life_.name.empty() && pthread_setname_np(pthread_self(), life_.name.c_str()) != 0)
            {
                throw std::runtime_error("cannot set thread name");
            }

            while (!StopRequested())
            {
                if (!WaitWhilePaused())
                {
                    break;
                }

                const auto begin = std::chrono::steady_clock::now();
                counters_.run_count.fetch_add(1);
                TaskResult result = TaskResult::Continue;
                try
                {
                    result = task_.entry(task_.context);
                }
                catch (const std::exception& error)
                {
                    if (StopRequested())
                    {
                        break;                  // 停机路径：任务被唤醒后抛的异常不算故障
                    }

                    counters_.retry_count.fetch_add(1);
                    if (fault_.retry == nullptr ||
                        fault_.retry(error, *this, task_.context) != RecoveryAction::Restart)
                    {
                        if (StopRequested())
                        {
                            break;
                        }
                        DefaultHandle(this);
                        break;
                    }
                }

                RecordRun(std::chrono::steady_clock::now() - begin);

                if (result == TaskResult::Exit)
                {
                    counters_.exit_count.fetch_add(1);

                    // 停机路径下的 Exit 是正常退出（下游通道被关闭），不按故障处置
                    if (StopRequested())
                    {
                        break;
                    }

                    DefaultHandle(this);
                    break;
                }

                if (result == TaskResult::Retry)
                {
                    counters_.retry_count.fetch_add(1);

                    const std::runtime_error retry("task retry");
                    if (fault_.retry == nullptr || fault_.retry(retry, *this, task_.context) != RecoveryAction::Restart)
                    {
                        if (StopRequested())
                        {
                            break;
                        }
                        DefaultHandle(this);
                        break;
                    }
                }

                if (task_.period.count() > 0)
                {
                    Sleep(task_.period);
                }
            }
        }
        catch (const std::exception& error)
        {
            System_RequestStop(1);
            DUST_LOG_ERR("%s", error.what());
        }
        catch (...)
        {
            System_RequestStop(1);
        }

        // 统一收尾：亡语 → 注销注册表 → 结束
        if (fault_.exit != nullptr)
        {
            fault_.exit(task_.context);
        }
        UnregisterThreadInstance(this);
        DUST_LOG_WRN("thread %s exited", Name());

        life_.finished.store(true);
    }
};
} // namespace init
