/**
 * @file log_policy.hpp
 * @author qingyu
 * @brief DUST_LOG 通道策略表 — 通道/颜色/前缀/ANSI/优先级集中定义
 * @version 0.3
 * @date 2026-10-05
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
    LogChannel channel;     // 通道
    TxPriority priority;    // 展开后的发送优先级
    LogColor color;         // ANSI 颜色
    const char* prefix;     // 行首前缀
    bool ansi_color;        // 通道默认是否上 ANSI 颜色（DBG 实际形态由条目标志覆盖）
    bool selectable;        // 是否可被 log on/off 选择
    bool stale_on_switch;   // 切换选择时是否作废在途记录
    bool evictable;         // 池满时是否可被挤出
};

/**
 * @brief 通道策略表（索引 = LogChannel 值）
 */
inline constexpr LogChannelPolicy kLogChannelPolicies[] = {
    {LogChannel::Info,      TxPriority::Event,  LogColor::Black,    "[inf] ",   true,  false, false, false},
    {LogChannel::Error,     TxPriority::Event,  LogColor::Red,      "[err] ",   true,  false, false, false},
    {LogChannel::Ok,        TxPriority::Event,  LogColor::Green,    "[ok] ",    true,  false, false, false},
    {LogChannel::Warning,   TxPriority::Event,  LogColor::Orange,   "[wrn] ",   true,  false, false, false},
    {LogChannel::Debug,     TxPriority::Dbg,    LogColor::Mint,     "",         true,  true,  true , true},
    {LogChannel::Command,   TxPriority::Cmd,    LogColor::Black,    "",         false, false, false, false},
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

/**
 * @brief 发送优先级策略（帧池满时的挤出规则）
 */
struct TxPriorityPolicy
{
    TxPriority priority;  // 优先级档位
    bool may_be_evicted;  // 池满时是否可被挤出
    bool may_evict_lower; // 池满时是否可挤出更低档
    bool preserve_fifo;   // 同档是否保持 FIFO
};

/**
 * @brief 发送优先级策略表（索引 = TxPriority 值）
 */
inline constexpr TxPriorityPolicy kTxPriorityPolicies[] = {
    {TxPriority::Event, false, true, true},
    {TxPriority::Cmd,   true,  true, true},
    {TxPriority::Dbg,   true,  true, true},
};

/**
 * @brief 取发送优先级策略
 * @param prio 优先级档位
 * @return 该档位的策略（静态表项引用）
 */
inline const TxPriorityPolicy& GetTxPriorityPolicy(TxPriority prio)
{
    return kTxPriorityPolicies[static_cast<uint8_t>(prio)];
}

} // namespace debug
