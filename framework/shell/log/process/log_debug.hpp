/**
 * @file log_debug.hpp
 * @author qingyu
 * @brief DUST_LOG DBG 源注册表：名字注册、单源选择与分频
 * @version 0.3
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <string>

namespace debug
{

constexpr size_t kMaxLogEntries = 64;

/**
 * @brief DBG 日志条目；拥有名字，避免 Linux 调用方临时字符串悬空
 */
struct LogEntry
{
    std::string           name;             	// DBG 名字（log on 用，运行时创建）
    std::atomic<uint16_t> divisor {1};      	// 分频周期：1=不分频，N=每 N 条打 1 条
    std::atomic<uint16_t> quota   {0};      	// 剩余可丢条数（0=本次放行）
};

/**
 * @brief DBG 注册表，调用方持有日志锁后访问
 */
class DebugSourceRegistry
{
public:
    /**
     * @brief 清空选中状态、条目计数与选择代数
     */
    void Reset()
    {
        count_ = 0;
        active_ = nullptr;
        ++epoch_;
    }

    /**
     * @brief 按名字查条目，首见创建
     * @param name DBG 名字
     * @return 条目指针；名字无效或池满返回 nullptr
     */
    LogEntry* FindOrCreate(const char* name)
    {
        if (name == nullptr || *name == '\0')
        {
            return nullptr;
        }
        if (auto entry = Find(name))
        {
            return entry;
        }
        if (count_ == entries_.size())
        {
            return nullptr;
        }
        auto entry = &entries_[count_++];
        entry->name = name;
        entry->divisor.store(1);
        entry->quota.store(0);
        return entry;
    }

    /**
     * @brief 选中条目并设置分频周期
     *
     * 配额清零：换名字或重设周期都从头数，否则切完看不到第一条。
     *
     * @param name    DBG 名字
     * @param divisor 分频周期（1=不分频；0 按 1 处理）
     * @return true 选中成功；false 条目不存在
     */
    bool Select(const char* name, uint16_t divisor = 1)
    {
        auto entry = Find(name);
        if (entry == nullptr)
        {
            return false;
        }
        entry->divisor.store((divisor == 0) ? 1 : divisor);
        entry->quota.store(0);
        active_ = entry;
        ++epoch_;
        return true;
    }

    /**
     * @brief 取消选中（代数递增作废在途）
     */
    void Deselect()
    {
        active_ = nullptr;
        ++epoch_;
    }

    const LogEntry* Active() const
    {
        return active_;
    }

    uint64_t ActiveEpoch() const
    {
        return epoch_;
    }

    LogEntry* First()
    {
        return count_ == 0 ? nullptr : &entries_[0];
    }

    LogEntry* Next(const LogEntry* entry)
    {
        for (size_t i = 0; i < count_; ++i)
        {
            if (entry == &entries_[i])
            {
                return i + 1 < count_ ? &entries_[i + 1] : nullptr;
            }
        }
        return nullptr;
    }

private:
    LogEntry* Find(const char* name)
    {
        if (name == nullptr)
        {
            return nullptr;
        }
        for (size_t i = 0; i < count_; ++i)
        {
            if (entries_[i].name == name)
            {
                return &entries_[i];
            }
        }
        return nullptr;
    }

    std::array<LogEntry, kMaxLogEntries> entries_{};
    size_t count_ = 0;
    LogEntry* active_ = nullptr;
    uint64_t epoch_ = 0;
};

} // namespace debug
