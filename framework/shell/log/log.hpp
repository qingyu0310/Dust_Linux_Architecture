/**
 * @file log.hpp
 * @author qingyu
 * @brief [门面] DUST_LOG 组装层 — 四层串接 + 对外宏与装配入口
 * @version 0.3
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 * # 四层结构
 *
 * ```text
 * ① 命令层  cmd/       log_cmd.hpp/.cpp     词法解析 + 查表匹配 + 子命令处理
 * ② 加工层  process/   log_process.hpp/.cpp 日志入口 / 分频 / 格式化 / 上色 / 帧包装
 * ③ 队列层  queue/     log_policy / log_record / log_queue
 * ④ 发送层  transport/ log_transport.hpp/.cpp
 * ```
 *
 * **单向流水线**：① → ② → ③ → ④，禁止跳层、反向、同层互调。
 * 加工层是唯一持有四层实例的地方，因此命令层拿不到实例，越权无从下手。
 *
 * # 本文件职责
 *
 * 只做两件事：定义对外宏（宏直接落到加工层）；提供 `Log` 类把 shell 线程需要的
 * 装配与驱动入口转发给加工层。**不含任何实现**。
 */

#pragma once

#include <cstdarg>
#include <cstdint>
#include "log_process.hpp"

namespace debug
{

/**
 * @brief DUST_LOG 门面（纯转发，无状态）
 *
 * 一次性四色日志（INF/ERR/OK/WRN）调用即打印，无名字无选择；
 * DBG 流式日志带名字、默认静默，`log on <name> [N]` 选中后按分频打印。
 * 所有发送统一走三档优先级帧队列（事件 > 命令响应 > DBG）。
 *
 * 只保留 shell 线程装配与驱动需要的入口；终端编辑、输入重绘这类由 shell
 * 直接调 `logproc::`，不在这里再包一层。
 */
class Log
{
public:
    /**
     * @brief 初始化：重置加工层持有的四层实例
     * @return true 初始化成功
     */
    static bool Init();

    /**
     * @brief 标记 shell 线程已接管发送
     */
    static void MarkShellThreadReady() { logproc::MarkShellThreadReady(); }

    /**
     * @brief 终端发送泵：DMA 空闲则续发下一帧（shell 线程每轮驱动）
     */
    static void SendNextFrameIfIdle() { logproc::SendNextFrameIfIdle(); }

    /**
     * @brief 命令响应直发（不经过 log 过滤，带行尾，中档优先级）
     * @param text 响应文本
     */
    static void SendCommandLine(const char* text) { logproc::CommandLine(text); }

    /**
     * @brief log 命令入口（转发到 ① 命令层）
     * @param line 子命令参数（不含 "log" 前缀）
     */
    static void ProcessLogCommand(uint8_t* line);

    /**
     * @brief DBG 流式打印：仅当前选中条目输出，形态取条目标志
     * @param entry DBG 条目
     * @param fmt   格式化串
     */
    static void PrintSelectedDebug(LogEntry* entry, const char* fmt, ...) __attribute__((format(printf, 2, 3)));

    /**
     * @brief 一次性日志，黑色
     * @param fmt 格式化串
     */
    static void PrintInfo(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

    /**
     * @brief 一次性日志，红色
     * @param fmt 格式化串
     */
    static void PrintError(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

    /**
     * @brief 一次性日志，绿色（状态正常）
     * @param fmt 格式化串
     */
    static void PrintOk(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

    /**
     * @brief 一次性日志，橘色
     * @param fmt 格式化串
     */
    static void PrintWarning(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

    /**
     * @brief 接管同一终端的命令回显（终端编辑层）；重定向时不启用
     * @return true 成功
     */
    static bool BeginInput() { return logproc::BeginInput(); }

    /**
     * @brief 保存输入行和光标，与日志输出串行重绘
     * @param line   当前命令
     * @param cursor 光标位置
     */
    static void UpdateInput(const char* line, size_t cursor) { logproc::UpdateInput(line, cursor); }

    /**
     * @brief 回车时保留命令显示并清空输入快照
     */
    static void CommitInput() { logproc::CommitInput(); }

    /**
     * @brief 关闭调试台时恢复终端属性
     */
    static void EndInput() { logproc::EndInput(); }

    /**
     * @brief 终端发送泵的等待描述符，供 shell 线程 poll
     * @return 描述符；负数表示 Init 未成功
     */
    static int WakeFd() { return logproc::WakeFd(); }

    /**
     * @brief 清唤醒计数；调用方须在排空之前调用
     */
    static void ConsumeWake() { logproc::ConsumeWake(); }

    /**
     * @brief 排空队列，恢复启动期直发模式
     */
    static void Flush() { logproc::Flush(); }
};

} // namespace debug

// ========== 真实实现：宏直接落到 ② 加工层，不经门面，避免可变参数二次转发 ==========

// DBG：流式调试日志（默认静默，log on <name> [N] 选中后按分频打印）
// 名字是字符串字面量，运行时 FindOrCreateDebugEntry 创建/复用条目，无需 DEFINE/链接段
#define DUST_LOG_DBG(name_, ...) ::debug::logproc::Debug(::debug::logproc::FindOrCreateDebugEntry(name_), ##__VA_ARGS__)

// 一次性四色（调用即打，无名字；[等级] 前缀与颜色由通道策略补）
#define DUST_LOG_INF(...) ::debug::logproc::Info(__VA_ARGS__)
#define DUST_LOG_ERR(...) ::debug::logproc::Error(__VA_ARGS__)
#define DUST_LOG_OK(...)  ::debug::logproc::Ok(__VA_ARGS__)
#define DUST_LOG_WRN(...) ::debug::logproc::Warning(__VA_ARGS__)
