/**
 * @file log_transport.cpp
 * @author qingyu
 * @brief [④ 发送层] DUST_LOG 终端输出后端实现 — 写 stderr 并记录首次故障
 * @version 0.3
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "log_transport.hpp"
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <unistd.h>

namespace debug
{

/**
 * @brief 初始化发送状态
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
 * @brief 输出一帧日志
 *
 * 帧内容已是完整一行（含前缀、颜色、行尾），直接写。
 * 输入行重绘由编辑层在调用本函数前后自己处理。
 */
bool LogTransport::Send(const TxFrame& frame)
{
    return Write(frame.data.data(), std::strlen(frame.data.data()));
}

/**
 * @brief 获取首次后端错误，不经日志接口递归输出
 */
int LogTransport::LastError() const
{
    return error_;
}

} // namespace debug
