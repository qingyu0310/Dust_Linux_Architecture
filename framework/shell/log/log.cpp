/**
 * @file log.cpp
 * @author qingyu
 * @brief [门面] DUST_LOG 门面实现 — 只做四层组装
 * @version 0.3
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "log.hpp"
#include "log_cmd.hpp"
#include "log_process.hpp"

namespace debug
{

/**
 * @brief 初始化日志系统：重置加工层持有的四层实例
 *
 * DBG 条目由 FindOrCreateDebugEntry 运行时创建，无链接段遍历。
 *
 * @return true 初始化成功
 */
bool Log::Init()
{
    return logproc::Init();
}

/**
 * @brief log 命令入口：转发到命令层
 * @param line 子命令参数（不含 "log" 前缀）
 */
void Log::ProcessLogCommand(uint8_t* line)
{
    logcmd::Process(line);
}

/**
 * @brief DBG 流式打印：可变参数不能二次转发，只能在这里拆开再交给加工层
 * @param entry DBG 条目
 * @param fmt   格式化串
 */
void Log::PrintSelectedDebug(LogEntry* entry, const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    logproc::DebugV(entry, fmt, ap);
    va_end(ap);
}

/**
 * @brief 一次性日志，黑色
 * @param fmt 格式化串
 */
void Log::PrintInfo(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    logproc::Emit(LogChannel::Info, fmt, ap);
    va_end(ap);
}

/**
 * @brief 一次性日志，红色
 * @param fmt 格式化串
 */
void Log::PrintError(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    logproc::Emit(LogChannel::Error, fmt, ap);
    va_end(ap);
}

/**
 * @brief 一次性日志，绿色（状态正常）
 * @param fmt 格式化串
 */
void Log::PrintOk(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    logproc::Emit(LogChannel::Ok, fmt, ap);
    va_end(ap);
}

/**
 * @brief 一次性日志，橘色
 * @param fmt 格式化串
 */
void Log::PrintWarning(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    logproc::Emit(LogChannel::Warning, fmt, ap);
    va_end(ap);
}

} // namespace debug
