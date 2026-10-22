/**
 * @file log_record.hpp
 * @author qingyu
 * @brief DUST_LOG Linux 记录：调用点格式化为自有文本，避免 32 位参数槽
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once
#include <array>
#include <cstdarg>
#include <cstdio>
#include <cstdint>
#include "log_policy.hpp"

namespace debug
{
    
constexpr size_t kLogBufSize = 512;

/**
 * @brief 已格式化日志记录，不存储调用方字符串或参数指针
 */
struct LogRecord
{
    std::array<char, kLogBufSize> text{};
    LogChannel channel = LogChannel::Info;
    uint64_t epoch = 0;
    bool ansi = false;
};

/**
 * @brief 在 va_list 有效期内格式化；异步队列仅接收自有文本
 */
inline LogRecord BuildRecordFromVaList(LogChannel channel, uint64_t epoch, bool ansi, const char* fmt, va_list ap)
{
    LogRecord record;
    record.channel = channel;
    record.epoch = epoch;
    record.ansi = ansi;
    if (fmt != nullptr)
    {
        std::vsnprintf(record.text.data(), record.text.size(), fmt, ap);
    }
    return record;
}

} // namespace debug
