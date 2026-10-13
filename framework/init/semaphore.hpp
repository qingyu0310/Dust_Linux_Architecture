/**
 * @file semaphore.hpp
 * @author qingyu
 * @brief POSIX 信号量 — 互斥、通知、限时获取和作用域释放
 * @version 0.1
 * @date 2026-10-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cerrno>
#include <chrono>
#include <exception>
#include <semaphore.h>
#include <stdexcept>
#include <system_error>
#include <time.h>

namespace init
{

/**
 * @brief 进程内 POSIX 信号量，销毁前须结束所有访问线程
 */
class Semaphore final
{
public:
    /**
     * @brief 建立进程内信号量
     * @param initial_value 初始计数；互斥使用 1，事件通知使用 0
     * @throws std::system_error 系统初始化失败
     */
    explicit Semaphore(unsigned int initial_value = 1)
    {
        if (::sem_init(&semaphore_, 0, initial_value) != 0)
        {
            throw std::system_error(errno, std::generic_category(), "sem_init");
        }
    }

    /**
     * @brief 销毁信号量，调用方须确保没有持有者或等待者
     */
    ~Semaphore()
    {
        ::sem_destroy(&semaphore_);
    }

    Semaphore(const Semaphore&) = delete;
    Semaphore& operator=(const Semaphore&) = delete;

    /**
     * @brief 阻塞获取设备许可，信号中断后重试
     * @throws std::system_error 系统等待失败
     */
    void Acquire()
    {
        while (::sem_wait(&semaphore_) != 0)
        {
            if (errno != EINTR)
            {
                throw std::system_error(errno, std::generic_category(), "sem_wait");
            }
        }
    }

    /**
     * @brief 非阻塞获取一次许可
     * @return true 已获取；false 当前没有可用许可
     * @throws std::system_error 系统调用失败
     */
    bool TryAcquire()
    {
        while (::sem_trywait(&semaphore_) != 0)
        {
            if (errno == EAGAIN)
            {
                return false;
            }
            if (errno != EINTR)
            {
                throw std::system_error(errno, std::generic_category(), "sem_trywait");
            }
        }
        return true;
    }

    /**
     * @brief 限时获取许可，截止时间使用 POSIX 实时钟
     * @param duration 等待预算，单位 ms，不得为负
     * @return true 已获取；false 已超时
     * @throws std::exception 等待预算无效或系统调用失败
     */
    bool TryAcquire(std::chrono::milliseconds duration)
    {
        if (duration.count() < 0)
        {
            throw std::invalid_argument("negative semaphore wait duration");
        }
        timespec deadline{};
        if (::clock_gettime(CLOCK_REALTIME, &deadline) != 0)
        {
            throw std::system_error(errno, std::generic_category(), "clock_gettime");
        }
        deadline.tv_sec += duration.count() / 1000;
        deadline.tv_nsec += (duration.count() % 1000) * 1000000;
        if (deadline.tv_nsec >= 1000000000)
        {
            ++deadline.tv_sec;
            deadline.tv_nsec -= 1000000000;
        }
        while (::sem_timedwait(&semaphore_, &deadline) != 0)
        {
            if (errno == ETIMEDOUT)
            {
                return false;
            }
            if (errno != EINTR)
            {
                throw std::system_error(errno, std::generic_category(), "sem_timedwait");
            }
        }
        return true;
    }

    /**
     * @brief 归还已获取许可，系统失败表明信号量生命周期或配对已损坏
     */
    void Release() noexcept
    {
        if (::sem_post(&semaphore_) != 0)
        {
            std::terminate();
        }
    }

    /**
     * @brief 作用域设备许可，异常展开时自动归还
     */
    class Guard final
    {
    public:
        /**
         * @brief 阻塞获取许可
         * @param semaphore 被保护的设备信号量
         */
        explicit Guard(Semaphore& semaphore) : semaphore_(semaphore)
        {
            semaphore_.Acquire();
        }

        /**
         * @brief 在等待预算内获取，超时不持有许可
         * @param semaphore 被保护的设备信号量
         * @param duration 最大等待时间，单位 ms
         * @throws std::runtime_error 等待超时
         */
        Guard(Semaphore& semaphore, std::chrono::milliseconds duration) : semaphore_(semaphore)
        {
            if (!semaphore_.TryAcquire(duration))
            {
                throw std::runtime_error("device busy, retry");
            }
        }

        /**
         * @brief 归还本作用域获取的许可
         */
        ~Guard()
        {
            semaphore_.Release();
        }

        Guard(const Guard&) = delete;
        Guard& operator=(const Guard&) = delete;

    private:
        Semaphore& semaphore_;    // 生命周期须覆盖本作用域
    };

private:
    sem_t semaphore_{};           // 计数语义由构造初值和 Acquire/Release 配对决定
};

} // namespace init
