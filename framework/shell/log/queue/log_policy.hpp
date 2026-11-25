/**
 * @file log_policy.hpp
 * @author qingyu
 * @brief DUST_LOG 通道策略表 — 通道/颜色/前缀/优先级集中定义
 * @version 0.4
 * @date 2026-11-25
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>

namespace debug
{

/**
 * @brief 日志颜色（ANSI 前景色编码）
 */
enum class LogColor : uint32_t
{
    Black   = 0x000000,     // 黑（INF）
    Red     = 0xF50002,     // 亮红（ERR）
    Green   = 0x00F700,     // 亮绿（OK）
    Orange  = 0xF6A753,     // 亮橙（WRN）
    Mint    = 0x7BF7CD,     // 薄荷绿（DBG）
};

/**
 * @brief 发送优先级（数值越小越靠前发）
 */
enum class TxPriority : uint8_t
{
    Event = 0,              // INF/ERR/OK/WRN：最高，插队头，永不挤
    Cmd = 1,                // 命令响应（var/log 输出）：中，插事件后、DBG 前
    Dbg = 2,                // DBG 流式：最低，排队尾，先被挤
};

/**
 * @brief 日志语义通道（对外入口按通道取策略）
 */
enum class LogChannel : uint8_t
{
    Info,                   // 一次性普通日志
    Error,                  // 一次性错误日志
    Ok,                     // 一次性成功日志
    Warning,                // 一次性警告日志
    Debug,                  // DBG 流式调试日志（默认静默，log on 选中）
    Command,                // 命令响应
};

/**
 * @brief 单通道输出策略
 */
struct LogChannelPolicy
{
    TxPriority priority;    // 展开后的发送优先级
    LogColor color;         // ANSI 颜色
    const char* prefix;     // 行首前缀
};

/**
 * @brief 通道策略表（索引 = LogChannel 值）
 */
inline constexpr LogChannelPolicy kLogChannelPolicies[] = {
    {TxPriority::Event,  LogColor::Black,    "[inf] "},
    {TxPriority::Event,  LogColor::Red,      "[err] "},
    {TxPriority::Event,  LogColor::Green,    "[ok] "},
    {TxPriority::Event,  LogColor::Orange,   "[wrn] "},
    {TxPriority::Dbg,    LogColor::Mint,     ""},
    {TxPriority::Cmd,    LogColor::Black,    ""},
};

/**
 * @brief 取通道策略
 * @param channel 日志通道
 * @return 该通道的策略（静态表项引用）
 */
inline const LogChannelPolicy& GetLogChannelPolicy(LogChannel channel)
{
    return kLogChannelPolicies[static_cast<uint8_t>(channel)];
}

} // namespace debug
