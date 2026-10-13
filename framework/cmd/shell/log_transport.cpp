/**
 * @file log_transport.cpp
 * @author qingyu
 * @brief Linux 终端日志与命令输入协调，发送故障不递归打印
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "log_transport.hpp"
#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace debug
{

/**
 * @brief 初始化发送状态，不清除尚未恢复的终端属性
 */
void LogTransport::Reset()
{
    error_ = 0;
    std::clearerr(stderr);
}

/**
 * @brief 判断输出是否连接终端
 */
bool LogTransport::SupportsColor() const
{
    return ::isatty(STDERR_FILENO) != 0;
}

/**
 * @brief 保存首次故障；失效后不再反复写同一输出端
 */
bool LogTransport::Write(const char* text, size_t size)
{
    if (error_ != 0)
    {
        return false;
    }
    errno = 0;
    if (std::fwrite(text, 1, size, stderr) != size || std::fflush(stderr) != 0)
    {
        error_ = errno == 0 ? EIO : errno;
        return false;
    }
    return true;
}

/**
 * @brief 日志发送前清除输入显示，发送后原样重绘输入与光标
 */
bool LogTransport::Send(const TxFrame& frame)
{
    if (editing_ && !Write("\r\x1b[2K", 5))
    {
        return false;
    }
    if (!Write(frame.data.data(), std::strlen(frame.data.data())))
    {
        return false;
    }
    return !editing_ || DrawInput();
}

/**
 * @brief 输入输出位于同一终端时关闭内核回显与整行输入，保留信号按键
 *
 * 重定向或 TERM=dumb 时保持原输入模式，不输出提示符控制序列。
 */
bool LogTransport::BeginInput()
{
    if (editing_)
    {
        return true;
    }

    struct stat input_stat {};
    struct stat output_stat {};

    const char* term = std::getenv("TERM");
    if (!::isatty(STDIN_FILENO) || !::isatty(STDERR_FILENO) ||
        (term != nullptr && std::strcmp(term, "dumb") == 0))
    {
        return true;
    }
    if (::fstat(STDIN_FILENO, &input_stat) != 0 || ::fstat(STDERR_FILENO, &output_stat) != 0)
    {
        error_ = errno;
        return false;
    }
    if (input_stat.st_rdev != output_stat.st_rdev)
    {
        return true;
    }
    if (::tcgetattr(STDIN_FILENO, &saved_) != 0)
    {
        error_ = errno;
        return false;
    }
    auto attributes = saved_;
    attributes.c_lflag &= ~(ICANON | ECHO | ECHONL | IEXTEN);
    attributes.c_iflag &= ~(IXON | ICRNL | INLCR | IGNCR);
    attributes.c_cc[VMIN] = 1;
    attributes.c_cc[VTIME] = 0;
    if (::tcsetattr(STDIN_FILENO, TCSANOW, &attributes) != 0)
    {
        error_ = errno;
        return false;
    }
    editing_ = true;
    input_.fill('\0');
    cursor_ = 0;
    return DrawInput();
}

/**
 * @brief 根据终端宽度显示单行窗口，避免长命令换行后无法清除旧输入
 */
bool LogTransport::DrawInput()
{
    struct winsize window {};

    const size_t columns =
        ::ioctl(STDERR_FILENO, TIOCGWINSZ, &window) == 0 && window.ws_col > 1 ? window.ws_col : 80;
    const char* prompt = columns > 8 ? "dust> " : "";
    const size_t prefix = std::strlen(prompt);
    const size_t room = columns - prefix - 1;
    const size_t begin = cursor_ >= room ? cursor_ - room + 1 : 0;
    const size_t length = std::min(room, std::strlen(input_.data()) - begin);
    char display[320];
    const int size = std::snprintf(display, sizeof(display), "\r\x1b[2K%s%.*s\r\x1b[%zuG", prompt,
                                   static_cast<int>(length), input_.data() + begin,
                                   prefix + cursor_ - begin + 1);
    return Write(display, static_cast<size_t>(size));
}

/**
 * @brief 拷贝当前命令，由统一输出锁保护显示内容
 */
bool LogTransport::UpdateInput(const char* line, size_t cursor)
{
    if (!editing_)
    {
        return true;
    }
    const size_t length = std::strlen(line);
    if (length >= input_.size() || cursor > length)
    {
        error_ = EINVAL;
        return false;
    }
    std::memcpy(input_.data(), line, length + 1);
    cursor_ = cursor;
    return DrawInput();
}

/**
 * @brief 提交时保留命令显示，后续输出使用空输入缓冲
 */
bool LogTransport::CommitInput()
{
    if (!editing_)
    {
        return true;
    }
    input_[0] = '\0';
    cursor_ = 0;
    return Write("\r\n", 2);
}

/**
 * @brief 无论输出是否失效，都尝试恢复终端属性；允许重复调用
 */
bool LogTransport::EndInput()
{
    if (!editing_)
    {
        return true;
    }
    const bool output_ok = Write("\r\x1b[2K", 5);
    if (::tcsetattr(STDIN_FILENO, TCSANOW, &saved_) != 0)
    {
        if (error_ == 0)
        {
            error_ = errno;
        }
        return false;
    }
    editing_ = false;
    return output_ok;
}

/**
 * @brief 获取首次后端错误，不经日志接口递归输出
 */
int LogTransport::LastError() const
{
    return error_;
}

} // namespace debug
