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
#include "var.hpp"
#include "log.hpp"
#include "framework/init/Init_entry.hpp"
#include "framework/init/thread.hpp"
#include <poll.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <string>

namespace debug
{
static Shell shell_{};
static init::Thread thread_{};

/**
 * @brief 输出当前已实现的日志和变量调试命令
 */
void Shell::CmdHelp() const
{
    Log::SendCommandLine("var list|get <name>|set <name> <value>");
    Log::SendCommandLine("log list                列出 DBG 条目");
    Log::SendCommandLine("log on <name>           选择一条 DBG 日志");
    Log::SendCommandLine("log off                 停止 DBG 输出");
    Log::SendCommandLine("log mode <name> on|off  VOFA 纯文本/颜色模式");
}

/**
 * @brief 解析一行命令并分派到对应命令族
 * @param line 行缓冲指针，解析时会修改内容
 */
void Shell::ProcessLine(uint8_t* line)
{
    auto text = reinterpret_cast<char*>(line);
    while (*text == ' ')
    {
        ++text;
    }
    char* arguments = text;
    while (*arguments != '\0' && *arguments != ' ')
    {
        ++arguments;
    }
    if (*arguments == ' ')
    {
        *arguments++ = '\0';
    }
    if (std::strcmp(text, "log") == 0)
    {
        Log::ProcessLogCommand(reinterpret_cast<uint8_t*>(arguments));
    }
    else if (std::strcmp(text, "var") == 0)
    {
        Var::Process(reinterpret_cast<uint8_t*>(arguments));
    }
    else if (std::strcmp(text, "h") == 0 || std::strcmp(text, "?") == 0)
    {
        CmdHelp();
    }
    else if (*text != '\0')
    {
        Log::SendCommandLine("?: log/var/h");
    }
}

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
 * @brief 独立调试台任务；输入、待发帧和停止请求三选一才醒，stdin EOF 后只等发送泵
 */
static void Task(void*, void*, void*)
{
    Log::MarkShellThreadReady();
    bool input_open = true;

    for (;;)
    {
        pollfd descriptors[2] = {
            {STDIN_FILENO,  POLLIN, 0},
            {Log::WakeFd(), POLLIN, 0}
        };

        if (!input_open) {
            descriptors[0] = descriptors[1];
        }
        const nfds_t count = input_open ? 2 : 1;

        const int ready = ::poll(descriptors, count, -1);
        if (ready < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            input_open = false; // 输入描述符故障：退回只驱动发送泵
            continue;
        }

        if (thread_.StopRequested()) {
            break;
        }

        if (input_open && (descriptors[0].revents & (POLLIN | POLLHUP)))
        {
            char buffer[256];
            const auto size = ::read(STDIN_FILENO, buffer, sizeof(buffer));
            if (size == 0)
            {
                input_open = false;
                Log::EndInput();
            }
            else if (size < 0 && errno != EINTR && errno != EAGAIN)
            {
                input_open = false;
            }
            if (size > 0)
            {
                shell_.ProcessInput(buffer, static_cast<size_t>(size), ShellStopping, ServiceCommands);
            }
        }
        if (input_open && (descriptors[0].revents & (POLLERR | POLLNVAL)))
        {
            input_open = false;
        }

        Log::ConsumeWake(); // 先清计数再发送：发送期间入队的帧会重新置位，poll 立刻返回

        Log::SendNextFrameIfIdle();

        if (thread_.StopRequested())
        {
            break;
        }
    }

    Log::EndInput();
    Log::Flush();
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
    thread_.Start(Task, nullptr, "shell");
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
