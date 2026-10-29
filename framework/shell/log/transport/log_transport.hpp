/**
 * @file log_transport.hpp
 * @author qingyu
 * @brief [④ 发送层] DUST_LOG 终端输出后端 — 只负责把字节写到 stderr
 * @version 0.3
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 * # 在四层里的位置
 *
 * ```text
 * ① 命令层 cmd/  ② 加工层 process/  ③ 队列层 queue/  ④ 发送层 transport/  ← 本文件
 * ```
 *
 * 只做一件事：把一帧已格式化好的字节写到 stderr，并记录首次故障。
 * **不感知输入行、光标、终端属性**——那些归 editor/ 层，由加工层在发送前后协调。
 */

#pragma once

#include <cstddef>
#include "log_queue.hpp"

namespace debug
{

/**
 * @brief 终端输出后端
 *
 * 同步写；失败后记住首个 errno，不再反复写失效后端。
 */
class LogTransport
{
public:
    void Reset();                       // 重置发送故障；仅在初始化时调用
    bool SupportsColor() const;         // stderr 是否为终端
    bool Send(const TxFrame& frame);    // 输出一帧日志
    bool Write(const char* text, size_t size);  // 输出裸字节（供编辑层重绘）
    int  LastError() const;             // 首次发送失败的 errno

private:
    int error_ = 0;                     // 0=正常；非零=首次后端故障
};

} // namespace debug
