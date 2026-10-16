/**
 * @file log_queue.hpp
 * @author qingyu
 * @brief DUST_LOG 有界帧队列：事件、命令、DBG 优先级与过期帧处理
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once
#include <array>
#include <algorithm>
#include <cstring>
#include "log_record.hpp"

namespace debug
{

constexpr size_t kTxPoolSize  = 8;
constexpr size_t kTxFrameSize = 320;

/**
 * @brief 发送帧；拥有完整输出字节，不引用调用方数据
 */
struct TxFrame
{
    std::array<char, kTxFrameSize> data{};
    TxPriority priority = TxPriority::Event;
    uint64_t   epoch = 0;
};

/**
 * @brief 三档优先级 FIFO，调用方持有日志锁后操作
 */
class TxFrameQueue
{
public:
    void Reset()
    {
        count_ = 0;
    }

    bool Push(const TxFrame& frame)
    {
        if (count_ == frames_.size())
        {
            size_t victim = count_;
            for (size_t i = 0; i < count_; ++i)
            {
                if (frames_[i].priority > frame.priority &&
                    (victim == count_ || frames_[i].priority > frames_[victim].priority))
                {
                    victim = i;
                }
            }
            if (victim == count_)
            {
                return false;
            }
            Erase(victim);
        }
        size_t position = 0;
        while (position < count_ && frames_[position].priority <= frame.priority)
        {
            ++position;
        }
        for (size_t i = count_; i > position; --i)
        {
            frames_[i] = frames_[i - 1];
        }
        frames_[position] = frame;
        ++count_;
        return true;
    }

    bool Pop(TxFrame& frame, uint64_t epoch)
    {
        while (count_ != 0)
        {
            frame = frames_[0];
            Erase(0);
            if (frame.priority != TxPriority::Dbg || frame.epoch == epoch)
            {
                return true;
            }
        }
        return false;
    }

private:
    size_t count_ = 0;
    std::array<TxFrame, kTxPoolSize> frames_{};

    void Erase(size_t position)
    {
        for (size_t i = position; i + 1 < count_; ++i)
        {
            frames_[i] = frames_[i + 1];
        }
        --count_;
    }
};

/**
 * @brief 包装前缀、颜色、行尾；帧内容最多 127 字节
 */
inline TxFrame FormatRecordToFrame(const LogRecord& record)
{
    const auto& policy = GetLogChannelPolicy(record.channel);
    
    TxFrame frame;
    frame.priority = policy.priority;
    frame.epoch    = record.epoch;
    char head[64]{};

    if (record.ansi)
    {
        const auto rgb = static_cast<uint32_t>(policy.color);
        std::snprintf(head, sizeof(head), "\x1b[38;2;%u;%u;%um%s", (rgb >> 16) & 255, (rgb >> 8) & 255, rgb & 255, policy.prefix);
    }
    else
    {
        std::snprintf(head, sizeof(head), "%s", policy.prefix);
    }
    const char* tail = record.ansi ? "\x1b[0m\r\n" : "\r\n";
    const auto room = frame.data.size() - 1 - std::strlen(head) - std::strlen(tail);
    std::snprintf(frame.data.data(), frame.data.size(), "%s%.*s%s", head, static_cast<int>(std::min(room, std::strlen(record.text.data()))), record.text.data(), tail);
    return frame;
}

} // namespace debug
