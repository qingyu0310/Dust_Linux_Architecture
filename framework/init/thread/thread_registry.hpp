/**
 * @file thread_registry.hpp
 * @author qingyu
 * @brief 线程注册表 — 已启动线程登记，供 shell 命令遍历
 * @version 0.1
 * @date 2026-10-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include <cstring>
#include "thread.hpp"   // ThreadBase / ThreadStats

/**
 * @brief 线程注册表
 *
 * Thread::Start 时登记，shell 的 thread 命令按名字查找/遍历。
 * 静态数组，无动态内存；表满只影响命令可见性，不影响线程启动。
 */
class thread_registry
{
public:
    static constexpr uint8_t kMaxThreads = 16;              // 可登记线程数上限

    /**
     * @brief 登记一个已启动的线程
     * @param thread 线程对象
     * @return true=登记成功；false=表已满
     */
    static bool Add(init::ThreadBase* thread)
    {
        if (thread == nullptr) return false;

        for (uint8_t i = 0; i < count_; ++i) {
            if (threads_[i] == thread) return true;         // 同一对象重复登记：视为成功
        }

        if (count_ >= kMaxThreads) return false;

        threads_[count_++] = thread;
        return true;
    }

    /**
     * @brief 按序号取线程
     * @param index 序号（0 起）
     * @return 线程对象；越界返回 nullptr
     */
    static init::ThreadBase* At(uint8_t index)
    {
        return (index < count_) ? threads_[index] : nullptr;
    }

    /**
     * @brief 已登记线程数
     */
    static uint8_t Count() { return count_; }

    /**
     * @brief 按名字查找线程
     * @param name 线程名
     * @return 线程对象；未找到返回 nullptr
     */
    static init::ThreadBase* Find(const char* name)
    {
        if (name == nullptr) return nullptr;

        for (uint8_t i = 0; i < count_; ++i)
        {
            if (std::strcmp(threads_[i]->Name(), name) == 0) return threads_[i];
        }
        return nullptr;
    }

private:
    static inline init::ThreadBase* threads_[kMaxThreads] {};   // 已登记线程
    static inline uint8_t           count_ = 0;                 // 已登记数量
};
