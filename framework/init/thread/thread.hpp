/**
 * @file thread.hpp
 * @author qingyu
 * @brief Linux 线程封装：受管任务外壳、运行统计、暂停恢复与停止通知
 * @version 0.2
 * @date 2026-10-22
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
#include "framework/cmd/shell/log/log.hpp"

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
    uint32_t    fatal_count;        // 不可恢复错误次数
    uint32_t    drop_count;         // 业务丢帧/丢包计数
    uint64_t    run_ns;             // 任务体累计执行时间
    uint64_t    max_run_ns;         // 单次最长执行时间
    uint64_t    last_run_ns;        // 最近一次执行时间
    uint32_t    stack_unused;       // 栈剩余字节（Linux 不采集，恒为 0）
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
};

/**
 * @brief 线程登记入口（定义在 Init_entry.cpp，供 Thread::Start 调用）
 * @param thread 已创建的线程对象
 */
void RegisterThreadInstance(ThreadBase* thread);

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
        Retry    = 1,   // 可恢复错误：交给恢复钩子
        Fatal    = 2,   // 不可恢复错误：记录后退出
    };

    /**
     * @brief 受管任务入口，每拍调用一次
     */
    using TaskFunc = TaskResult (*)(void* context);

    /**
     * @brief 异常恢复结果，正常停止由线程停止标志单独判断
     */
    enum class RecoveryAction
    {
        Restart,    // 恢复成功，重新执行任务入口
        Stop        // 无法恢复，请求统一退出
    };

    using RecoveryHook = RecoveryAction (*)(const std::exception&, Thread&, void*);

    /**
     * @brief 注册可选异常恢复回调，未注册时保持统一退出行为
     * @param hook 恢复回调，空指针表示禁用
     * @param context 业务上下文，生命周期必须覆盖 Join()
     * @throws std::logic_error 线程尚未回收
     * @note 仅由拥有者在 Start() 前调用，不得与启动或回收并发
     */
    void SetRecoveryHook(RecoveryHook hook, void* context = nullptr)
    {
        if (worker_.joinable())
        {
            throw std::logic_error("cannot change recovery hook before thread is joined");
        }
        recovery_hook_ = hook;
        recovery_context_ = context;
    }

    /**
     * @brief 启动受管任务：外壳负责循环与停止判断，任务体只写一拍
     *
     * @param entry 任务入口，每拍调用一次
     * @param context 任务上下文，每拍原样传回
     * @param name Linux 线程名，最多 15 字节
     * @throws std::exception 入口无效、线程未回收或创建失败
     */
    void Start(TaskFunc entry, void* context = nullptr, const char* name = nullptr)
    {
        if (entry == nullptr)
        {
            throw std::invalid_argument("thread entry is null");
        }
        if (worker_.joinable())
        {
            throw std::logic_error("thread already started");
        }

        name_ = name == nullptr ? "" : name;
        if (name_.size() > 15)
        {
            throw std::invalid_argument("thread name exceeds 15 bytes");
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = false;
        }

        task_ = entry;
        task_context_ = context;
        worker_ = std::thread(&Thread::Run, this);

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
        wake_fd_ = fd;
    }

    /**
     * @brief 设置停止标志并唤醒等待中的业务线程
     */
    void RequestStop()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        condition_.notify_all();

        {
            std::lock_guard<std::mutex> lock(pause_mutex_);
        }
        pause_condition_.notify_all();   // 暂停中的线程也要能醒来退出

        if (wake_fd_ >= 0)
        {
            const uint64_t one = 1;
            const auto written = ::write(wake_fd_, &one, sizeof(one));
            (void)written; // EAGAIN 表示计数已饱和，等待方本来就可读
        }
    }

    /**
     * @brief 等待停止或超时，供业务线程替代不可中断 sleep
     *
     * @param duration 最大等待时间
     * @return true 收到停止请求；false 等待超时
     */
    bool WaitForStop(std::chrono::milliseconds duration)
    {
        std::unique_lock<std::mutex> lock(mutex_);

        return condition_.wait_for(lock, duration, [this] { return stop_.load(); });
    }

    /**
     * @brief 读取停止标志，供 poll 返回后判断是否退出
     *
     * @return true 已收到停止请求
     */
    bool StopRequested() const
    {
        return stop_.load();
    }

    /**
     * @brief 等待线程退出；仅由拥有者调用，不能在线程自身调用
     */
    void Join()
    {
        if (worker_.joinable())
        {
            worker_.join();
        }
    }

    const char* Name() const override
    {
        return name_.c_str();
    }

    pthread_t Tid() const override
    {
        return tid_.load();
    }

    /**
     * @brief 取运行统计快照
     *
     * @param out 输出
     * @return true=已启动过，数据有效
     */
    bool GetStats(ThreadStats& out) const override
    {
        out.name        = name_.c_str();
        out.run_count   = run_count_.load();
        out.retry_count = retry_count_.load();
        out.fatal_count = fatal_count_.load();
        out.drop_count  = drop_count_.load();
        out.run_ns      = run_ns_.load();
        out.max_run_ns  = max_run_ns_.load();
        out.last_run_ns = last_run_ns_.load();
        out.stack_unused = 0;   // Linux 不采集栈剩余
        out.running     = started_.load() && !finished_.load();
        out.paused      = Paused();

        return started_.load();
    }

    /**
     * @brief 清零运行统计（不清线程生命周期与暂停状态）
     */
    void ResetStats() override
    {
        run_count_.store(0);
        retry_count_.store(0);
        fatal_count_.store(0);
        drop_count_.store(0);
        run_ns_.store(0);
        max_run_ns_.store(0);
        last_run_ns_.store(0);
    }

    /**
     * @brief 暂停线程：在下一拍开始前挂起，保留全部现场
     *
     * @note Linux 没有内核级挂起，挂起点在受管外壳的拍与拍之间；正在执行的那一拍会先跑完。
     */
    void Pause() override
    {
        std::lock_guard<std::mutex> lock(pause_mutex_);
        paused_ = true;
    }

    /**
     * @brief 恢复被暂停的线程，从挂起处继续执行
     */
    void Resume() override
    {
        {
            std::lock_guard<std::mutex> lock(pause_mutex_);
            paused_ = false;
        }
        pause_condition_.notify_all();
    }

    bool Paused() const override
    {
        std::lock_guard<std::mutex> lock(pause_mutex_);
        return paused_;
    }

    /**
     * @brief 业务侧计数一次丢帧/丢包
     */
    void CountDrop()
    {
        drop_count_.fetch_add(1);
    }

private:
    std::string name_;
    std::thread worker_;
    std::mutex  mutex_;
    std::atomic<bool> stop_ {false};
    std::condition_variable condition_;

    mutable std::mutex      pause_mutex_;           // 保护暂停标志
    std::condition_variable pause_condition_;       // 暂停/恢复时唤醒外壳
    bool                    paused_ = false;        // 拍间挂起标志

    std::atomic<bool>      started_  {false};       // 是否已启动过
    std::atomic<bool>      finished_ {false};       // 线程入口是否已退出
    std::atomic<pthread_t> tid_ {};                 // 线程号

    int wake_fd_ = -1;                                  // 停止唤醒描述符；-1 表示业务线程不使用 poll 等待

    void*        recovery_context_ = nullptr;           // 由调用方持有的恢复上下文
    RecoveryHook recovery_hook_    = nullptr;           // 可选业务异常恢复入口

    TaskFunc     task_             = nullptr;           // 受管任务入口
    void*        task_context_     = nullptr;           // 受管任务上下文

    std::atomic<uint32_t> run_count_   {0};             // 任务体进入次数
    std::atomic<uint32_t> retry_count_ {0};             // 可恢复错误次数
    std::atomic<uint32_t> fatal_count_ {0};             // 不可恢复错误次数
    std::atomic<uint32_t> drop_count_  {0};             // 业务计数
    std::atomic<uint64_t> run_ns_      {0};             // 累计执行时间
    std::atomic<uint64_t> max_run_ns_  {0};             // 单次最长执行时间
    std::atomic<uint64_t> last_run_ns_ {0};             // 最近一次执行时间

    /**
     * @brief 拍间挂起，返回 false 表示挂起期间收到了停止请求
     */
    bool WaitWhilePaused()
    {
        std::unique_lock<std::mutex> lock(pause_mutex_);
        pause_condition_.wait(lock, [this] { return !paused_ || stop_.load(); });

        return !stop_.load();
    }

    /**
     * @brief 记录一拍耗时
     */
    void RecordRun(std::chrono::steady_clock::duration elapsed)
    {
        const auto ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count());
        run_ns_.fetch_add(ns);
        last_run_ns_.store(ns);

        uint64_t longest = max_run_ns_.load();
        while (ns > longest && !max_run_ns_.compare_exchange_weak(longest, ns))
        {
        }
    }

    /**
     * @brief 公共线程入口：设置名称、循环调用任务并统一处理异常与退出
     *
     * 停止请求只在这里判断，任务体不感知；任务体抛出的异常交给恢复钩子，
     * 恢复失败则请求统一退出。
     */
    void Run()
    {
        started_.store(true);
        tid_.store(pthread_self());

        try
        {
            if (!name_.empty() && pthread_setname_np(pthread_self(), name_.c_str()) != 0)
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
                run_count_.fetch_add(1);

                TaskResult result = TaskResult::Continue;
                try
                {
                    result = task_(task_context_);
                }
                catch (const std::exception& error)
                {
                    if (StopRequested())
                    {
                        break;
                    }

                    if (recovery_hook_ == nullptr)
                    {
                        throw;
                    }

                    retry_count_.fetch_add(1);
                    if (recovery_hook_(error, *this, recovery_context_) != RecoveryAction::Restart)
                    {
                        throw;
                    }
                }

                RecordRun(std::chrono::steady_clock::now() - begin);

                if (result == TaskResult::Fatal)
                {
                    fatal_count_.fetch_add(1);
                    DUST_LOG_ERR("thread %s fatal", name_.c_str());
                    break;
                }

                if (result == TaskResult::Retry)
                {
                    retry_count_.fetch_add(1);
                    if (recovery_hook_ == nullptr)
                    {
                        break;
                    }

                    const std::runtime_error retry("task retry");
                    if (recovery_hook_(retry, *this, recovery_context_) != RecoveryAction::Restart)
                    {
                        break;
                    }
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

        finished_.store(true);
    }
};
} // namespace init
