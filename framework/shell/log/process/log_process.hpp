/**
 * @file log_process.hpp
 * @author qingyu
 * @brief [② 加工层] DUST_LOG 加工 — 日志入口 / 分频过滤 / 格式化 / 上色 / 帧包装
 * @version 0.1
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 * # 在四层里的位置
 *
 * ```text
 * ① 命令层 cmd/log_cmd       词法解析 + 查表
 * ② 加工层 process/          ← 本文件
 * ③ 队列层 queue/            记录结构 + 帧队列
 * ④ 发送层 transport/        终端后端
 * ```
 *
 * 加工层是唯一持有四层实例的地方（`mutex_` / `dbg_` / `txq_` / `pump_` / `wake_fd_` / `shell_own_`），
 * 也是「一条日志变成可发送文本」的全部工作所在：
 *
 * - 取通道策略（颜色 / 前缀 / 优先级）
 * - 分频过滤（DBG，`PassDivisor`）
 * - 调用点格式化 + 上色 + 帧包装
 * - 排空队列与唤醒 shell 线程
 *
 * **禁用的依赖**：不许 include `log_cmd.hpp`（命令层是上层）。
 */

#pragma once

#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include "log_policy.hpp"
#include "log_debug.hpp"

namespace debug::logproc
{

/**
 * @brief 初始化加工层持有的四层实例
 * @return true 初始化成功
 */
bool Init();

// ========== 对外日志入口（宏的目标）==========

/**
 * @brief 一次性日志（按通道策略决定颜色与前缀）
 * @param channel 日志通道
 * @param fmt     格式化串
 * @param ap      可变参数
 */
void Emit(LogChannel channel, const char* fmt, va_list ap);

/**
 * @brief 一次性日志，黑色
 * @param fmt 格式化串
 */
void Info(const char* fmt, ...);

/**
 * @brief 一次性日志，红色
 * @param fmt 格式化串
 */
void Error(const char* fmt, ...);

/**
 * @brief 一次性日志，绿色（状态正常）
 * @param fmt 格式化串
 */
void Ok(const char* fmt, ...);

/**
 * @brief 一次性日志，橘色
 * @param fmt 格式化串
 */
void Warning(const char* fmt, ...);

/**
 * @brief DBG 流式打印（仅当 e 是当前选中条目且通过分频才发）
 * @param e   DBG 条目（FindOrCreateDebugEntry 返回值，nullptr=池满静默）
 * @param fmt 格式化串
 */
void Debug(LogEntry* e, const char* fmt, ...);

/**
 * @brief DBG 流式打印的 va_list 版本，供门面转发可变参数
 * @param e   DBG 条目
 * @param fmt 格式化串
 * @param ap  可变参数
 */
void DebugV(LogEntry* e, const char* fmt, va_list ap);

/**
 * @brief 命令响应直发（不经过 DBG 选择，带行尾）
 * @param text 响应文本
 */
void CommandLine(const char* text);

// ========== 加工原语 ==========

/**
 * @brief 分频过滤：本条 DBG 是否放行
 *
 * divisor<=1 时直接放行（短路，不碰 quota）；
 * 否则递减配额，配额用尽则放行并重填。
 *
 * @param e DBG 条目
 * @return true=放行；false=本次丢弃
 */
bool PassDivisor(LogEntry* e);

// ========== 向命令层暴露的 DBG 操作 ==========

/**
 * @brief 按名字查 DBG 条目，首见创建（同名复用同一条目）
 * @param name DBG 名字（字符串字面量）
 * @return 条目指针；池满返回 nullptr
 */
LogEntry* FindOrCreateDebugEntry(const char* name);

/**
 * @brief 选中 DBG 条目并设置分频周期
 * @param name    DBG 名字
 * @param divisor 分频周期（1=不分频；0 按 1 处理）
 * @return true 选中成功；false 条目不存在
 */
bool SelectDebugEntry(const char* name, uint16_t divisor = 1);

/**
 * @brief 取消选中（代数递增作废在途）
 */
void DeselectDebugEntry();

/**
 * @brief 遍历所有已注册 DBG 条目输出（名字 + 选中状态 + 分频）
 */
void PrintLogList();

// ========== 发送与终端 ==========

/**
 * @brief 终端发送泵：弹出待发帧，跳过过期 DBG 帧
 */
void SendNextFrameIfIdle();

/**
 * @brief 取发送泵唤醒描述符，供 shell 线程 poll 等待
 * @return 描述符；负数表示 Init 未成功
 */
int WakeFd();

/**
 * @brief 清唤醒计数；调用方须在排空之前调用
 */
void ConsumeWake();

/**
 * @brief 排空队列，恢复启动期直发模式
 */
void Flush();

/**
 * @brief 标记 shell 线程已接管发送
 */
void MarkShellThreadReady();

/**
 * @brief 接管同一终端的命令回显；重定向时不启用
 * @return true 成功
 */
bool BeginInput();

/**
 * @brief 保存输入行和光标，与日志输出串行重绘
 * @param line   当前命令
 * @param cursor 光标位置
 */
void UpdateInput(const char* line, size_t cursor);

/**
 * @brief 回车时保留命令显示并清空输入快照
 */
void CommitInput();

/**
 * @brief 关闭调试台时恢复终端属性
 */
void EndInput();

/**
 * @brief 当前是否已接管终端（供 shell 判断要不要重绘）
 */
bool EditingInput();

} // namespace debug::logproc
