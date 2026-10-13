/**
 * @file shell.hpp
 * @author qingyu
 * @brief Linux 调试台：终端命令分发和日志发送驱动
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <string>

namespace debug
{

/**
 * @brief 保留 Shell 命令分发边界，Linux 使用 stdin 输入、stderr 输出
 */
class Shell
{
public:
    void ProcessLine(uint8_t* line);
    void CmdHelp() const;
    using StopCheck = bool (*)();
    using Service = void (*)();

    /**
     * @brief 逐字编辑命令，在命令之间检查停止并推进日志服务
     * @param buffer 输入字节，调用期间保持有效
     * @param size 字节数量
     * @param stopping 可选停止查询，收到停止后不执行后续命令
     * @param between_commands 可选服务回调，命令结束且锁释放后执行
     */
    void ProcessInput(const char* buffer, size_t size,
                      StopCheck stopping = nullptr, Service between_commands = nullptr);

private:
    bool ProcessChar(char ch);                          // 处理回车、退格与光标按键

    std::string line_;                                  // 实际命令，独立于日志显示
    std::string escape_;                                // 分次读取的终端按键序列

    size_t cursor_        = 0;                          // ASCII 命令光标
    bool   skip_lf_      = false;                       // CRLF 只提交一次
    bool   discard_line_ = false;                       // 超长命令丢弃至回车
};

} // namespace debug
