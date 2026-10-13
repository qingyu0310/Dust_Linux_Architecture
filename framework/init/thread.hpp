/**
 * @file thread.hpp
 * @author qingyu
 * @brief Linux 线程封装：可中断等待、停止通知和线程回收
 * @version 0.1
 * @date 2026-10-12
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
#include "System_startup.hpp"
#include "framework/cmd/shell/log.hpp"

namespace init
{
/**
 * @brief Linux 线程与可中断等待封装
 *
 * 拥有者先 RequestStop，再 Join，最后释放线程访问的资源。
 * 析构执行停止与等待；不得在被管理线程内部析构本对象。
 */
class Thread
{
public:
    /**
     * @brief 析构时通知停止并等待线程退出
     */
    ~Thread()
    {
        RequestStop();
        Join();
    }

    Thread() = default;
    Thread(const Thread&) = delete;
    Thread& operator=(const Thread&) = delete;

    /**
     * @brief 线程入口签名，与子树 Task(void*, void*, void*) 保持一致
     */
    using Entry = void (*)(void*, void*, void*);

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
     * @brief 启动独立任务函数，异常由公共线程入口捕获
     *
     * @param entry 任务入口，必须非空
     * @param p1 任务上下文；其余两个参数保留为空
     * @param name Linux 线程名，最多 15 字节
     * @throws std::exception 入口无效、线程未回收或创建失败
     */
    void Start(Entry entry, void* p1 = nullptr, const char* name = nullptr)
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
        worker_ = std::thread(&Thread::Run, this, entry, p1);
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

private:
    std::string name_;
    std::thread worker_;
    std::mutex  mutex_;
    std::atomic<bool> stop_ {false};
    std::condition_variable condition_;

    int wake_fd_ = -1; // 停止唤醒描述符；-1 表示业务线程不使用 poll 等待

    void*        recovery_context_ = nullptr;           // 由调用方持有的恢复上下文
    RecoveryHook recovery_hook_    = nullptr;           // 可选业务异常恢复入口

    /**
     * @brief 公共线程入口：设置名称、异常恢复及统一退出；恢复失败不再次进入 hook
     */
    void Run(Entry entry, void* p1)
    {
        try
        {
            if (!name_.empty() && pthread_setname_np(pthread_self(), name_.c_str()) != 0)
            {
                throw std::runtime_error("cannot set thread name");
            }

            while (!StopRequested())
            {
                try
                {
                    entry(p1, nullptr, nullptr);
                    return;
                }
                catch (const std::exception& error)
                {
                    if (StopRequested()) {
                        return;
                    }

                    if (recovery_hook_ == nullptr) {
                        throw;
                    }

                    const auto action = recovery_hook_(error, *this, recovery_context_);
                    if (StopRequested()) {
                        return;
                    }
                    if (action != RecoveryAction::Restart) {
                        throw;
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
    }
};
} // namespace init
