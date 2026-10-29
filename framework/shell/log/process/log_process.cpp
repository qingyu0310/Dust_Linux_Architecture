/**
 * @file log_process.cpp
 * @author qingyu
 * @brief [② 加工层] DUST_LOG 加工层实现 — 日志入口 / 分频过滤 / 格式化 / 上色 / 帧包装
 * @version 0.1
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "log_process.hpp"
#include "log_queue.hpp"
#include "log_record.hpp"
#include "log_transport.hpp"
#include "log_editor.hpp"
#include "framework/init/startup/System_startup.hpp"
#include <cstdio>
#include <cstring>
#include <mutex>
#include <sys/eventfd.h>
#include <unistd.h>

namespace debug::logproc
{

namespace
{

std::mutex          mutex_;             // 注册表、帧队列及终端输出并发保护
DebugSourceRegistry dbg_ {};            // DBG 源注册表（名字、选中、遍历）
TxFrameQueue        txq_ {};            // 发送帧池与三档优先级队列
LogTransport        pump_ {};           // 终端输出后端（只写字节）
LogEditor           editor_ {};         // 终端行编辑（光标、重绘、原始终端模式）
int                 wake_fd_ = -1;      // 发送泵唤醒描述符；Init 创建，-1 表示不可用
bool                shell_own_ = false; // shell 已接管发送；启动早期为 false，调用点直发

void Drain();

/**
 * @brief 包装日志记录并按优先级入帧队列
 *
 * 调用方已持有 mutex_。池满且无法挤出低档帧时，DBG 丢弃；
 * 事件与命令先同步排空队列再入队。启动期直接驱动发送。
 *
 * @param record 拥有格式化文本的日志记录
 */
void Submit(const LogRecord& record)
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
void Drain()
{
    TxFrame frame;
    while (txq_.Pop(frame, dbg_.ActiveEpoch()))
    {
        // 日志整行输出会打断正在编辑的命令行：先清掉输入显示，发完再重绘
        if (!editor_.Clear())
        {
            System_RequestStop(1);
        }
        if (!pump_.Send(frame))
        {
            System_RequestStop(1);
        }
        if (!editor_.Draw())
        {
            System_RequestStop(1);
        }
    }
}

/**
 * @brief 通用：按通道策略格式化、上色和提交发送
 *
 * 启动期调用点直接发送；shell 接管后进入帧队列，由发送泵驱动。
 *
 * @param channel 日志通道
 * @param fmt     格式化串
 * @param ap      可变参数，在调用点有效期内消费
 */
void PrintColoredLog(LogChannel channel, const char* fmt, va_list ap)
{
    std::lock_guard<std::mutex> lock(mutex_);
    Submit(BuildRecordFromVaList(channel, 0, pump_.SupportsColor(), fmt, ap));
}

} // namespace

/**
 * @brief 初始化加工层持有的四层实例
 *
 * 清空 DBG 注册表与帧队列，恢复启动期直发模式。
 * 同时创建发送泵唤醒描述符：创建失败按初始化失败上报，不退化成定时轮询。
 *
 * @return true 初始化成功
 */
bool Init()
{
    std::lock_guard<std::mutex> lock(mutex_);
    dbg_.Reset();
    txq_.Reset();
    pump_.Reset();
    shell_own_ = false;
    wake_fd_ = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    BindBackend(&pump_);
    return wake_fd_ >= 0;
}

/**
 * @brief 分频过滤：本条 DBG 是否放行
 *
 * 用递减配额而不是取余：重填只在放行的 1/N 次执行，1kHz 循环下开销更低。
 *
 * @param e DBG 条目
 * @return true=放行；false=本次丢弃
 */
bool PassDivisor(LogEntry* e)
{
    const uint16_t divisor = e->divisor.load();
    if (divisor <= 1)
    {
        return true;                                    // 不分频：短路，不碰 quota
    }

    if (e->quota.fetch_sub(1) > 0)
    {
        return false;                                   // 还有配额：丢弃
    }

    e->quota.store(static_cast<uint16_t>(divisor - 1)); // 配额用尽：放行并重填
    return true;
}

/**
 * @brief 一次性日志（按通道策略决定颜色与前缀）
 * @param channel 日志通道
 * @param fmt     格式化串
 * @param ap      可变参数
 */
void Emit(LogChannel channel, const char* fmt, va_list ap)
{
    PrintColoredLog(channel, fmt, ap);
}

/**
 * @brief 一次性日志，黑色
 */
void Info(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    Emit(LogChannel::Info, fmt, ap);
    va_end(ap);
}

/**
 * @brief 一次性日志，红色
 */
void Error(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    Emit(LogChannel::Error, fmt, ap);
    va_end(ap);
}

/**
 * @brief 一次性日志，绿色（状态正常）
 */
void Ok(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    Emit(LogChannel::Ok, fmt, ap);
    va_end(ap);
}

/**
 * @brief 一次性日志，橘色
 */
void Warning(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    Emit(LogChannel::Warning, fmt, ap);
    va_end(ap);
}

/**
 * @brief DBG 流式打印，仅当前选中条目输出
 *
 * 分频判断在格式化之前：被丢弃的调用不构造记录、不入队。
 *
 * @param e   DBG 条目，FindOrCreateDebugEntry 返回值
 * @param fmt 格式化串
 */
void Debug(LogEntry* e, const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    DebugV(e, fmt, ap);
    va_end(ap);
}

/**
 * @brief DBG 流式打印的 va_list 版本
 *
 * 门面无法二次转发 va_list，所以把拆包放在这里；分频判断仍在格式化之前。
 *
 * @param e   DBG 条目
 * @param fmt 格式化串
 * @param ap  可变参数
 */
void DebugV(LogEntry* e, const char* fmt, va_list ap)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (e == nullptr || e != dbg_.Active())
    {
        return;
    }
    if (!PassDivisor(e))
    {
        return;
    }

    Submit(BuildRecordFromVaList(LogChannel::Debug, dbg_.ActiveEpoch(), pump_.SupportsColor(), fmt, ap));
}

/**
 * @brief 命令响应，不经过 DBG 选择，带行尾
 *
 * @param text 响应文本，nullptr 按空文本处理
 */
void CommandLine(const char* text)
{
    std::lock_guard<std::mutex> lock(mutex_);
    LogRecord record;
    record.channel = LogChannel::Command;
    std::snprintf(record.text.data(), record.text.size(), "%s", text == nullptr ? "" : text);
    Submit(record);
}

/**
 * @brief 按名字查 DBG 条目，首见创建
 * @param name DBG 名字
 * @return 条目指针；池满返回 nullptr
 */
LogEntry* FindOrCreateDebugEntry(const char* name)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return dbg_.FindOrCreate(name);
}

/**
 * @brief 选中 DBG 条目并设置分频周期
 * @param name    DBG 名字
 * @param divisor 分频周期
 * @return true 选中成功；false 条目不存在
 */
bool SelectDebugEntry(const char* name, uint16_t divisor)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return dbg_.Select(name, divisor);
}

/**
 * @brief 取消选中（代数递增作废在途）
 */
void DeselectDebugEntry()
{
    std::lock_guard<std::mutex> lock(mutex_);
    dbg_.Deselect();
}

/**
 * @brief log list：遍历注册表输出名字、选中状态与分频
 */
void PrintLogList()
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto entry = dbg_.First(); entry != nullptr; entry = dbg_.Next(entry))
    {
        LogRecord record;
        record.channel = LogChannel::Command;
        std::snprintf(record.text.data(), record.text.size(), "%-15s %-5s x%u",
                      entry->name.c_str(),
                      entry == dbg_.Active() ? "[ON]" : "[off]",
                      static_cast<unsigned>(entry->divisor.load()));
        Submit(record);
    }
}

/**
 * @brief 驱动终端发送泵，由 shell 线程在等待返回后调用
 */
void SendNextFrameIfIdle()
{
    std::lock_guard<std::mutex> lock(mutex_);
    Drain();
}

/**
 * @brief 取发送泵唤醒描述符
 * @return 描述符；负数表示 Init 未成功
 */
int WakeFd()
{
    return wake_fd_;
}

/**
 * @brief 清唤醒计数，供下一次 poll 阻塞
 *
 * 调用方须在排空之前调用：先清计数再发送，发送期间新入队的帧会重新置位，
 * 下一次 poll 立即返回；反过来（先发后清）会把这个置位清掉，留下没人唤醒的待发帧。
 */
void ConsumeWake()
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
void Flush()
{
    std::lock_guard<std::mutex> lock(mutex_);
    Drain();
    shell_own_ = false;
}

/**
 * @brief shell 线程接管日志发送
 */
void MarkShellThreadReady()
{
    std::lock_guard<std::mutex> lock(mutex_);
    shell_own_ = true;
}

/**
 * @brief 平台终端编辑准备，输入与日志共用一把输出锁
 */
bool BeginInput()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return editor_.Begin();
}

/**
 * @brief 更新正在输入的命令；后端故障只请求停止，不递归打印
 */
void UpdateInput(const char* line, size_t cursor)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!editor_.Update(line, cursor))
    {
        System_RequestStop(1);
    }
}

/**
 * @brief 命令提交时换行，保持后续日志输出与新提示符一致
 */
void CommitInput()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!editor_.Commit())
    {
        System_RequestStop(1);
    }
}

/**
 * @brief 关闭时恢复终端，输出失效也不能跳过终端恢复
 */
void EndInput()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!editor_.End())
    {
        System_RequestStop(1);
    }
}

/**
 * @brief 当前是否已接管终端
 * @return true=原始终端模式已启用
 */
bool EditingInput()
{
    return editor_.Editing();
}

} // namespace debug::logproc
