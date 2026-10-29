/**
 * @file log_editor.cpp
 * @author qingyu
 * @brief [终端编辑层] 命令行显示与重绘实现 — 单行窗口、长命令横向滚动
 * @version 0.1
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "log_editor.hpp"
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

namespace
{

LogTransport* backend_ = nullptr;   // 由加工层在 Init 时绑定

/**
 * @brief 写裸字节到终端；后端未绑定时视为成功（重定向场景）
 */
bool Write(const char* text, size_t size)
{
    return backend_ == nullptr || backend_->Write(text, size);
}

} // namespace

/**
 * @brief 绑定终端输出后端，由加工层在初始化时调用
 * @param backend 输出后端
 */
void BindBackend(LogTransport* backend)
{
    backend_ = backend;
}

/**
 * @brief 启用原始终端模式，关闭内核回显与整行输入，保留信号按键
 */
bool LogEditor::Begin()
{
    if (editing_)
    {
        return true;
    }

    struct stat input_stat {};
    struct stat output_stat {};

    const char* term = std::getenv("TERM");
    if (!::isatty(STDIN_FILENO) || !::isatty(STDERR_FILENO) || (term != nullptr && std::strcmp(term, "dumb") == 0))
    {
        return true;
    }
    if (::fstat(STDIN_FILENO, &input_stat) != 0 || ::fstat(STDERR_FILENO, &output_stat) != 0)
    {
        return false;
    }
    if (input_stat.st_rdev != output_stat.st_rdev)
    {
        return true;
    }
    if (::tcgetattr(STDIN_FILENO, &saved_) != 0)
    {
        return false;
    }

    auto attributes = saved_;
    attributes.c_lflag &= ~(ICANON | ECHO | ECHONL | IEXTEN);
    attributes.c_iflag &= ~(IXON | ICRNL | INLCR | IGNCR);
    attributes.c_cc[VMIN] = 1;
    attributes.c_cc[VTIME] = 0;
    if (::tcsetattr(STDIN_FILENO, TCSANOW, &attributes) != 0)
    {
        return false;
    }
	
    editing_ = true;
    input_.fill('\0');
    cursor_ = 0;
    return Draw();
}

/**
 * @brief 根据终端宽度显示单行窗口，避免长命令换行后无法清除旧输入
 */
bool LogEditor::Draw()
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
bool LogEditor::Update(const char* line, size_t cursor)
{
    if (!editing_)
    {
        return true;
    }

    const size_t length = std::strlen(line);
    if (length >= input_.size() || cursor > length)
    {
        return false;
    }

    std::memcpy(input_.data(), line, length + 1);
    cursor_ = cursor;
    return Draw();
}

/**
 * @brief 提交时保留命令显示，后续输出使用空输入缓冲
 */
bool LogEditor::Commit()
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
 * @brief 清掉当前输入行显示，供日志整行输出前调用
 */
bool LogEditor::Clear()
{
    if (!editing_)
    {
        return true;
    }
    return Write("\r\x1b[2K", 5);
}

/**
 * @brief 无论输出是否失效，都尝试恢复终端属性；允许重复调用
 */
bool LogEditor::End()
{
    if (!editing_)
    {
        return true;
    }

    const bool output_ok = Write("\r\x1b[2K", 5);
    if (::tcsetattr(STDIN_FILENO, TCSANOW, &saved_) != 0)
    {
        return false;
    }
    editing_ = false;
    return output_ok;
}

} // namespace debug
