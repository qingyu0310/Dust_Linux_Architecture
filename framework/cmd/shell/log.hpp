/**
 * @file log.hpp
 * @author qingyu
 * @brief DUST_LOG 日志入口：四色事件与按名字选择的 DBG 日志
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdarg>
#include <cstdint>
#include <mutex>
#include "log_debug.hpp"
#include "log_queue.hpp"
#include "log_transport.hpp"

namespace debug
{

/**
 * @brief DUST_LOG Linux 实现，保持子树调用接口与日志选择语义
 *
 * INF/ERR/OK/WRN 调用即登记输出，无等级过滤；DBG 默认静默，按名字单选。
 * Linux 在调用点格式化自有文本，shell 线程驱动优先级帧队列。
 */
class Log
{
public:
    static bool      Init();                                                                                      // 初始化：清选中、条目计数和帧队列；业务线程启动前调用
    static LogEntry* FindOrCreateDebugEntry(const char* name);                                                    // 按名字查 DBG 条目，首见创建（返回 nullptr=池满）
    static bool      SelectDebugEntry(const char* name);                                                          // 选中已注册条目，同一时间只保留一条
    static void      DeselectDebugEntry();                                                                        // 停止打印：清选中，代数递增作废在途 DBG 帧
    static bool      SetDebugVofa(const char* name, bool on);                                                     // 设置 DBG 模式（on=VOFA 纯文本，off=颜色模式）

    static void PrintSelectedDebug(LogEntry* entry, const char* fmt, ...) __attribute__((format(printf, 2, 3)));  // DBG 流式打印：仅当前选中条目输出，形态取条目标志
    static void PrintInfo(const char* fmt, ...) __attribute__((format(printf, 1, 2)));                            // 一次性，黑色
    static void PrintError(const char* fmt, ...) __attribute__((format(printf, 1, 2)));                           // 一次性，红色
    static void PrintOk(const char* fmt, ...) __attribute__((format(printf, 1, 2)));                              // 一次性，绿色
    static void PrintWarning(const char* fmt, ...) __attribute__((format(printf, 1, 2)));                         // 一次性，橘色

    static void SendCommandLine(const char* text);                                                                // 命令响应：不经过 DBG 选择，纯文本带行尾
    static void MarkShellThreadReady();                                                                           // shell 线程接管发送（Task 首行调用）
    static void ProcessLogCommand(uint8_t* line);                                                                 // log 命令入口（list/on/off/mode）
    static void PrintLogList();                                                                                   // log list：输出名字、选中状态和模式
    static void SendNextFrameIfIdle();                                                                            // 终端发送泵：弹出待发帧，跳过过期 DBG 帧
    static int  WakeFd();                                                                                         // 发送泵等待描述符：Init 创建，供 shell 线程 poll 等待
    static void ConsumeWake();                                                                                    // 清唤醒计数；调用方须在 Drain 之前调用
    static void Flush();                                                                                          // 排空队列，恢复启动期直发模式
    static bool BeginInput();                                                                                     // 接管同一终端的命令回显；重定向时不启用
    static void UpdateInput(const char* line, size_t cursor);                                                     // 保存输入行和光标，与日志输出串行重绘
    static void CommitInput();                                                                                    // 回车时保留命令显示并清空输入快照
    static void EndInput();                                                                                       // 关闭调试台时恢复终端属性

private:
    static void PrintColoredLog(LogChannel channel, const char* fmt, va_list ap);                                 // 通用：按通道策略格式化、上色和提交发送
    static void Submit(const LogRecord& record);                                                                  // 自有文本记录包装为帧后入队；调用方持有 mutex_
    static void Drain();                                                                                          // 发送队列排空；调用方持有 mutex_

    static inline std::mutex          mutex_;                                                                     // 注册表、帧队列及终端输出并发保护
    static inline DebugSourceRegistry dbg_ {};                                                                    // DBG 源注册表（名字、选中、遍历）
    static inline TxFrameQueue        txq_ {};                                                                    // 发送帧池与三档优先级队列
    static inline LogTransport        pump_ {};                                                                   // Linux 终端发送后端
    static inline int                 wake_fd_ = -1;                                                              // 发送泵唤醒描述符；Init 创建，-1 表示不可用
    static inline bool               shell_own_ = false;                                                          // shell 已接管发送；启动早期为 false，调用点直发
};

} // namespace debug

// DBG：流式日志，默认静默；log on <name> 选中后输出。
#define DUST_LOG_DBG(name_, ...) ::debug::Log::PrintSelectedDebug(::debug::Log::FindOrCreateDebugEntry(name_), __VA_ARGS__)

// 一次性四色：无名字、无选择；前缀与颜色由通道策略提供。
#define DUST_LOG_INF(...) ::debug::Log::PrintInfo(__VA_ARGS__)
#define DUST_LOG_ERR(...) ::debug::Log::PrintError(__VA_ARGS__)
#define DUST_LOG_OK(...)  ::debug::Log::PrintOk(__VA_ARGS__)
#define DUST_LOG_WRN(...) ::debug::Log::PrintWarning(__VA_ARGS__)
