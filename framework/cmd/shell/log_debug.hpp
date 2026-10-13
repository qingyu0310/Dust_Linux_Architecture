/**
 * @file log_debug.hpp
 * @author qingyu
 * @brief DUST_LOG DBG 源注册表：名字注册、单源选择与 VOFA 模式
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once
#include <array>
#include <string>
#include <cstdint>

namespace debug
{
constexpr size_t kMaxLogEntries = 64;

/**
 * @brief DBG 日志条目；拥有名字，避免 Linux 调用方临时字符串悬空
 */
struct LogEntry
{
    std::string name;
    bool vofa = false;
};

/**
 * @brief DBG 注册表，调用方持有日志锁后访问
 */
class DebugSourceRegistry
{
public:
    void Reset()
    {
        count_ = 0;
        active_ = nullptr;
        ++epoch_;
    }

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
        entry->vofa = false;
        return entry;
    }

    bool Select(const char* name)
    {
        auto entry = Find(name);
        if (entry == nullptr)
        {
            return false;
        }
        active_ = entry;
        ++epoch_;
        return true;
    }

    bool SetVofa(const char* name, bool on)
    {
        auto entry = Find(name);
        if (entry == nullptr)
        {
            return false;
        }
        entry->vofa = on;
        ++epoch_; // 作废切换前的在途 DBG 帧。
        return true;
    }

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
