/**
 * @file shell.cpp
 * @author qingyu
 * @brief Linux 调试台线程：poll 输入、日志泵和注册式启动
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "shell.hpp"
#include "shell_cmd.hpp"
#include "token.hpp"
#include "var.hpp"
#include "log.hpp"
#include "framework/init/startup/Init_entry.hpp"
#include "framework/init/thread/thread.hpp"
#include <poll.h>
#include <unistd.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>

namespace debug
{
    
static Shell shell_{};
static init::Thread thread_{};

/**
 * @brief 调试台拍与拍之间要保留的状态
 *
 * 循环搬进线程外壳后，输入是否仍打开不能再是任务体的局部变量，由 Task 的 context 带入。
 */
struct InputState
{
    bool input_open = true;    // stdin 仍可读；EOF 或描述符故障后只驱动发送泵
};

static InputState input_state_ {};

/**
 * @brief 输出帮助：遍历命令表，每族一行
 *
 * 命令族由各模块 REGISTER_SHELL_CMD 注册，帮助文本随注册项自动出现。
 */
void Shell::PrintHelp()
{
    for (const ShellCmdEntry* e = __start_shell_cmd; e < __stop_shell_cmd; ++e)
    {
        char line[80];
        std::snprintf(line, sizeof(line), "%-8s %s", e->name, e->help);
        Log::SendCommandLine(line);
    }
}

/**
 * @brief 解析一行命令并按命令表分派
 *
 * 取第一个 token 作为命令族名，在 shell_cmd 段内线性匹配，
 * 命中则把剩余参数串交给该族的 handler。
 *
 * @param line 行缓冲指针（可写，解析时会被修改）
 */
void Shell::ProcessLine(uint8_t* line)
{
    const char* cmd = nullptr;
    uint8_t* rest   = NextToken(line, cmd);

    if (*cmd == '\0') return;

    for (const ShellCmdEntry* e = __start_shell_cmd; e < __stop_shell_cmd; ++e)
    {
        if (std::strcmp(e->name, cmd) == 0)
        {
            e->handler(rest);
            return;
        }
    }

    Log::SendCommandLine("unknown command");
}

/**
 * @brief 帮助命令入口（注册到命令表）
 * @param args 未使用
 */
static void HelpCmd(uint8_t* args)
{
    (void)args;
    Shell::PrintHelp();
}

REGISTER_SHELL_CMD("h", "help", HelpCmd);

/**
 * @brief 处理输入字节，回显通过日志后端统一输出
 */
void Shell::ProcessInput(const char* buffer, size_t size, StopCheck stopping, Service between_commands)
{
    for (size_t i = 0; i < size; ++i)
    {
        if (stopping != nullptr && stopping())
        {
            return;
        }
        if (ProcessChar(buffer[i]) && between_commands != nullptr)
        {
            between_commands();
        }
    }
}

/**
 * @brief 编辑 ASCII 调试命令，支持退格、左右、首尾、Delete 和 Ctrl+U
 *
 * 命令最长 255 字节；超长后整行丢弃，不执行截断命令。
 */
void Shell::PushHistory(const std::string& command)
{
    if (command.empty())
    {
        return;
    }
    if (history_count_ > 0 && history_[history_count_ - 1] == command)
    {
        return; // 相邻重复不重复记录
    }

    if (history_count_ < kHistorySize)
    {
        history_[history_count_++] = command;
    }
    else
    {
        for (size_t i = 1; i < kHistorySize; ++i)
        {
            history_[i - 1] = std::move(history_[i]);
        }
        history_[kHistorySize - 1] = command;
    }

    history_pos_ = history_count_;
}

/**
 * @brief 上方向键：先存下当前草稿，再回看更早的一条
 */
void Shell::HistoryPrev()
{
    if (history_count_ == 0 || history_pos_ == 0)
    {
        return;
    }
    if (history_pos_ == history_count_)
    {
        history_draft_ = line_; // 首次进入历史：记住还没提交的半行
    }

    --history_pos_;
    line_   = history_[history_pos_];
    cursor_ = line_.size();
}

/**
 * @brief 下方向键：逐条回到最新，越过最新后恢复草稿
 */
void Shell::HistoryNext()
{
    if (history_count_ == 0 || history_pos_ == history_count_)
    {
        return;
    }

    ++history_pos_;
    line_   = history_pos_ == history_count_ ? history_draft_ : history_[history_pos_];
    cursor_ = line_.size();
}

bool Shell::ProcessChar(char ch)
{
    bool command_boundary = false;
    if (skip_lf_ && ch == '\n')
    {
        skip_lf_ = false;
        return false;
    }
    skip_lf_ = false;

    if (!escape_.empty())
    {
        escape_ += ch;
        if (escape_.size() == 2 && (ch == '[' || ch == 'O'))
        {
            return false;
        }
        if (escape_.size() < 16 && (ch < '@' || ch > '~'))
        {
            return false;
        }
        if (escape_ == "\x1b[D" && cursor_ > 0)
        {
            --cursor_;
        }
        else if (escape_ == "\x1b[C" && cursor_ < line_.size())
        {
            ++cursor_;
        }
        else if (escape_ == "\x1b[H" || escape_ == "\x1bOH")
        {
            cursor_ = 0;
        }
        else if (escape_ == "\x1b[F" || escape_ == "\x1bOF")
        {
            cursor_ = line_.size();
        }
        else if (escape_ == "\x1b[3~" && cursor_ < line_.size())
        {
            line_.erase(cursor_, 1);
        }
        else if (escape_ == "\x1b[A" || escape_ == "\x1bOA")
        {
            HistoryPrev();
        }
        else if (escape_ == "\x1b[B" || escape_ == "\x1bOB")
        {
            HistoryNext();
        }
        escape_.clear();
    }
    else if (ch == '\x1b')
    {
        escape_ = ch;
        return false;
    }
    else if (ch == '\n' || ch == '\r')
    {
        command_boundary = true;
        skip_lf_ = ch == '\r';
        Log::CommitInput();
        if (!discard_line_ && !line_.empty())
        {
            PushHistory(line_); // 必须在 ProcessLine 之前：它会原地把行切成 token
            ProcessLine(reinterpret_cast<uint8_t*>(line_.data()));
        }
        line_.clear();
        cursor_ = 0;
        discard_line_ = false;
    }
    else if (ch == '\b' || ch == 127)
    {
        if (cursor_ > 0)
        {
            line_.erase(--cursor_, 1);
        }
    }
    else if (ch == 21)
    {
        line_.clear();
        cursor_ = 0;
    }
    else if (ch == 4 && line_.empty())
    {
        System_RequestStop();
    }
    else if (!discard_line_ && ch >= ' ' && ch <= '~')
    {
        if (line_.size() < 255)
        {
            line_.insert(cursor_++, 1, ch);
        }
        else
        {
            discard_line_ = true;
            line_.clear();
            cursor_ = 0;
            Log::SendCommandLine("command too long");
        }
    }
    Log::UpdateInput(line_.c_str(), cursor_);
    return command_boundary;
}

/**
 * @brief 命令边界查询停止，避免批量输入在退出时继续执行
 */
static bool ShellStopping()
{
    return thread_.StopRequested();
}

/**
 * @brief 命令之间推进已有日志泵，不创建额外线程
 */
static void ServiceCommands()
{
    Log::ConsumeWake();
    Log::SendNextFrameIfIdle();
}

/**
 * @brief 调试台单拍：输入、待发帧任一就绪即醒，stdin EOF 后只等发送泵
 *
 * @param context 本线程的输入状态
 * @return Continue=继续下一拍
 */
static init::Thread::TaskResult Task(void* context)
{
    auto& state = *static_cast<InputState*>(context);

    pollfd descriptors[2] = {
        {STDIN_FILENO,  POLLIN, 0},
        {Log::WakeFd(), POLLIN, 0}
    };

    if (!state.input_open) {
        descriptors[0] = descriptors[1];
    }
    const nfds_t count = state.input_open ? 2 : 1;

    const int ready = ::poll(descriptors, count, -1);
    if (ready < 0)
    {
        if (errno == EINTR)
        {
            return init::Thread::TaskResult::Continue;
        }
        state.input_open = false; // 输入描述符故障：退回只驱动发送泵
        return init::Thread::TaskResult::Continue;
    }

    if (state.input_open && (descriptors[0].revents & (POLLIN | POLLHUP)))
    {
        char buffer[256];
        const auto size = ::read(STDIN_FILENO, buffer, sizeof(buffer));
        if (size == 0)
        {
            state.input_open = false;
            Log::EndInput();
        }
        else if (size < 0 && errno != EINTR && errno != EAGAIN)
        {
            state.input_open = false;
        }
        if (size > 0)
        {
            shell_.ProcessInput(buffer, static_cast<size_t>(size), ShellStopping, ServiceCommands);
        }
    }
    if (state.input_open && (descriptors[0].revents & (POLLERR | POLLNVAL)))
    {
        state.input_open = false;
    }

    Log::ConsumeWake(); // 先清计数再发送：发送期间入队的帧会重新置位，poll 立刻返回

    Log::SendNextFrameIfIdle();

    return init::Thread::TaskResult::Continue;
}

/**
 * @brief 初始化调试台日志系统
 *
 * @return true 初始化成功
 */
static bool thread_init()
{
    Var::Init();
    
    return Log::Init() && Log::BeginInput();
}

/**
 * @brief 启动调试台线程，并把发送泵唤醒描述符交给它
 *
 * @return true 启动成功
 */
static bool thread_start()
{
    if (Log::WakeFd() < 0)
    {
        return false; // 日志未初始化成功，宁可不启动也不退化成定时轮询
    }
    
    thread_.SetWakeFd(Log::WakeFd());
    // shell 是日志唯一消费者与停机后的观察窗口：Recover 全杀时保留
    thread_.Start(Task, std::chrono::milliseconds(0), &input_state_, "shell",
                  ThreadFaultConfig{nullptr, nullptr, ThreadFaultPolicy::Recover, true});

    return true;
}

/**
 * @brief 通知调试台线程停止并解除等待
 */
static void thread_stop()
{
    thread_.RequestStop();
}

/**
 * @brief 等待调试台线程退出并排空日志
 */
static void thread_join()
{
    thread_.Join();
    Log::EndInput();
    Log::Flush();
}

/**
 * @brief 释放调试台资源，保留后续清理日志的直发能力
 */
static void Release()
{
    Log::EndInput();
    Log::Flush();
}

REGISTER_INIT(thread_init, PreInit, High, HaltOnFail, Release, "shell_init");
REGISTER_THREAD(thread_start, PreThread, thread_stop, thread_join, "shell_start");

} // namespace debug
