/**
 * @file log.cpp
 * @author qingyu
 * @brief DUST_LOG Linux 实现：日志注册选择、优先级发送和命令解析
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "log.hpp"
#include "framework/init/System_startup.hpp"
#include <sstream>
#include <string>
#include <sys/eventfd.h>
#include <unistd.h>

namespace debug
{

/**
 * @brief 初始化日志系统
 *
 * 业务线程启动前调用，清空 DBG 注册表与帧队列，恢复启动期直发模式。
 * 同时创建发送泵唤醒描述符：创建失败按初始化失败上报，不退化成定时轮询。
 *
 * @return true 初始化成功
 */
bool Log::Init()
{
    std::lock_guard<std::mutex> lock(mutex_);
    dbg_.Reset();
    txq_.Reset();
    pump_.Reset();
    shell_own_ = false;
    wake_fd_ = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    return wake_fd_ >= 0;
}

/**
 * @brief 按名字查 DBG 条目，首见创建，同名复用
 *
 * @param name DBG 名字，注册表保存名字副本
 * @return 条目指针；名字无效或池满返回 nullptr
 */
LogEntry* Log::FindOrCreateDebugEntry(const char* name)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return dbg_.FindOrCreate(name);
}

/**
 * @brief 选中 DBG 条目：静默转为流式输出
 *
 * 只选择已注册条目，不创建不存在的名字；同一时间只选中一条。
 * 选择代数递增，旧 DBG 帧出队时作废。
 *
 * @param name DBG 名字
 * @return true 选中成功；false 名字不存在
 */
bool Log::SelectDebugEntry(const char* name)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return dbg_.Select(name);
}

/**
 * @brief 停止 DBG 输出
 *
 * 清空选中状态并递增代数，在途旧 DBG 帧出队时作废。
 */
void Log::DeselectDebugEntry()
{
    std::lock_guard<std::mutex> lock(mutex_);
    dbg_.Deselect();
}

/**
 * @brief 设置 DBG 条目的输出模式
 *
 * 只修改已注册条目；模式切换递增代数，避免旧模式帧继续输出。
 * 颜色模式仅在终端支持时启用 ANSI，重定向时保持纯文本。
 *
 * @param name DBG 名字
 * @param on true=VOFA 纯文本；false=颜色模式
 * @return true 设置成功；false 名字不存在
 */
bool Log::SetDebugVofa(const char* name, bool on)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return dbg_.SetVofa(name, on);
}

/**
 * @brief DBG 流式打印，仅当前选中条目输出
 *
 * 在调用点格式化为自有文本，队列不保留调用方字符串或参数指针。
 * 输出模式取条目标志，发送优先级为 DBG。
 *
 * @param entry DBG 条目，FindOrCreateDebugEntry 返回值
 * @param fmt 格式化串
 */
void Log::PrintSelectedDebug(LogEntry* entry, const char* fmt, ...)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (entry == nullptr || entry != dbg_.Active())
    {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    const auto record = BuildRecordFromVaList(LogChannel::Debug, dbg_.ActiveEpoch(),
                                              !entry->vofa && pump_.SupportsColor(), fmt, ap);
    va_end(ap);
    Submit(record);
}

/**
 * @brief 通用：按通道策略格式化、上色和提交发送
 *
 * 启动期调用点直接发送；shell 接管后进入帧队列，由发送泵驱动。
 *
 * @param channel 日志通道
 * @param fmt 格式化串
 * @param ap 可变参数，在调用点有效期内消费
 */
void Log::PrintColoredLog(LogChannel channel, const char* fmt, va_list ap)
{
    std::lock_guard<std::mutex> lock(mutex_);
    Submit(BuildRecordFromVaList(channel, 0, pump_.SupportsColor(), fmt, ap));
}

/**
 * @brief 一次性日志，黑色
 *
 * @param fmt 格式化串
 */
void Log::PrintInfo(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    PrintColoredLog(LogChannel::Info, fmt, ap);
    va_end(ap);
}

/**
 * @brief 一次性日志，红色
 *
 * @param fmt 格式化串
 */
void Log::PrintError(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    PrintColoredLog(LogChannel::Error, fmt, ap);
    va_end(ap);
}

/**
 * @brief 一次性日志，绿色（状态正常）
 *
 * @param fmt 格式化串
 */
void Log::PrintOk(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    PrintColoredLog(LogChannel::Ok, fmt, ap);
    va_end(ap);
}

/**
 * @brief 一次性日志，橘色
 *
 * @param fmt 格式化串
 */
void Log::PrintWarning(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    PrintColoredLog(LogChannel::Warning, fmt, ap);
    va_end(ap);
}

/**
 * @brief 包装日志记录并按优先级入帧队列
 *
 * 调用方已持有 mutex_。池满且无法挤出低档帧时，DBG 丢弃；
 * 事件与命令先同步排空队列再入队。启动期直接驱动发送。
 *
 * @param record 拥有格式化文本的日志记录
 */
void Log::Submit(const LogRecord& record)
{
    const auto frame = FormatRecordToFrame(record);
    if (!txq_.Push(frame))
    {
        if (frame.priority == TxPriority::Dbg)
        {
            return;
        }
        // 事件不能被同级满池静默丢弃：先同步排空，再登记。
        Drain();
        txq_.Push(frame);
    }
    if (!shell_own_)
    {
        Drain();
        return;
    }
    // shell 接管后由发送泵驱动：入队即唤醒，泵空闲时无需定时轮询。
    const uint64_t one = 1;
    const auto written = ::write(wake_fd_, &one, sizeof(one));
    (void)written; // EAGAIN 表示计数已饱和，泵本来就可读；此处不能报错退出
}

/**
 * @brief 排空发送队列，跳过选择代数已过期的 DBG 帧
 *
 * 调用方已持有 mutex_。终端发送失败时报告进程错误停止。
 */
void Log::Drain()
{
    TxFrame frame;
    while (txq_.Pop(frame, dbg_.ActiveEpoch()))
    {
        if (!pump_.Send(frame))
        {
            System_RequestStop(1);
        }
    }
}

/**
 * @brief 驱动 Linux 终端发送泵，由 shell 线程在等待返回后调用
 */
void Log::SendNextFrameIfIdle()
{
    std::lock_guard<std::mutex> lock(mutex_);
    Drain();
}

/**
 * @brief 取发送泵唤醒描述符
 *
 * 可读表示队列中有帧待发或收到停止请求；shell 线程把它和 stdin 一起放进 poll。
 *
 * @return 描述符；负数表示 Init 未成功
 */
int Log::WakeFd()
{
    return wake_fd_;
}

/**
 * @brief 清唤醒计数，供下一次 poll 阻塞
 *
 * 调用方须在 Drain 之前调用：先清计数再发送，发送期间新入队的帧会重新置位，
 * 下一次 poll 立即返回；反过来（先发后清）会把这个置位清掉，留下没人唤醒的待发帧。
 */
void Log::ConsumeWake()
{
    if (wake_fd_ < 0)
    {
        return;
    }
    uint64_t value = 0;
    const auto size = ::read(wake_fd_, &value, sizeof(value));
    (void)size; // 一次性读走全部计数；EAGAIN 表示计数已清零
}

/**
 * @brief 关闭时排空队列并恢复直发模式
 *
 * 恢复直发后，后续资源清理日志不再依赖已停止的 shell 线程。
 */
void Log::Flush()
{
    std::lock_guard<std::mutex> lock(mutex_);
    Drain();
    shell_own_ = false;
}

/**
 * @brief 平台终端编辑准备，输入与日志共用一把输出锁
 */
bool Log::BeginInput()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return pump_.BeginInput();
}

/**
 * @brief 更新正在输入的命令；后端故障只请求停止，不递归打印
 */
void Log::UpdateInput(const char* line, size_t cursor)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pump_.UpdateInput(line, cursor))
    {
        System_RequestStop(1);
    }
}

/**
 * @brief 命令提交时换行，保持后续日志输出与新提示符一致
 */
void Log::CommitInput()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pump_.CommitInput())
    {
        System_RequestStop(1);
    }
}

/**
 * @brief 关闭时恢复终端，输出失效也不能跳过终端恢复
 */
void Log::EndInput()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pump_.EndInput())
    {
        System_RequestStop(1);
    }
}

/**
 * @brief shell 线程接管日志发送
 */
void Log::MarkShellThreadReady()
{
    std::lock_guard<std::mutex> lock(mutex_);
    shell_own_ = true;
}

/**
 * @brief 命令响应，不经过 DBG 选择，带行尾
 *
 * @param text 响应文本，nullptr 按空文本处理
 */
void Log::SendCommandLine(const char* text)
{
    std::lock_guard<std::mutex> lock(mutex_);
    LogRecord record;
    record.channel = LogChannel::Command;
    std::snprintf(record.text.data(), record.text.size(), "%s", text == nullptr ? "" : text);
    Submit(record);
}

/**
 * @brief log list：遍历注册表输出名字、选中状态和输出模式
 */
void Log::PrintLogList()
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto entry = dbg_.First(); entry != nullptr; entry = dbg_.Next(entry))
    {
        LogRecord record;
        record.channel = LogChannel::Command;
        std::snprintf(record.text.data(), record.text.size(), "%s %s %s", entry->name.c_str(),
                      entry == dbg_.Active() ? "[ON]" : "[off]",
                      entry->vofa ? "[vofa]" : "[color]");
        Submit(record);
    }
}

/**
 * @brief log 命令入口（list/on/off/mode）
 *
 * @param line 子命令参数，不含 log 前缀
 */
void Log::ProcessLogCommand(uint8_t* line)
{
    if (line == nullptr)
    {
        return;
    }
    std::istringstream input(reinterpret_cast<const char*>(line));
    std::string command, name, mode, extra;
    input >> command;
    if (command == "list" && !(input >> extra))
    {
        PrintLogList();
    }
    else if (command == "on" && (input >> name) && !(input >> extra))
    {
        SendCommandLine(SelectDebugEntry(name.c_str()) ? "log on: ok" : "log on: not found");
    }
    else if (command == "off" && !(input >> extra))
    {
        DeselectDebugEntry();
        SendCommandLine("log off: ok");
    }
    else if (command == "mode" && (input >> name >> mode) && !(input >> extra))
    {
        if (mode == "on" || mode == "vofa" || mode == "off" || mode == "color")
        {
            const bool on = mode == "on" || mode == "vofa";
            SendCommandLine(SetDebugVofa(name.c_str(), on)
                                ? (on ? "log mode: vofa" : "log mode: color")
                                : "log mode: not found");
        }
        else
        {
            SendCommandLine("?: log mode <name> on|off");
        }
    }
    else
    {
        SendCommandLine("?: log list|on <name>|off|mode <name> on|off");
    }
}
} // namespace debug
