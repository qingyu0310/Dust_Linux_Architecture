/**
 * @file log_transport.hpp
 * @author qingyu
 * @brief DUST_LOG Linux 终端后端：发送、输入行重绘与终端状态恢复
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once
#include <array>
#include <termios.h>
#include "log_queue.hpp"

namespace debug
{
    
/**
 * @brief 同步终端后端；输入重绘与日志输出共用日志互斥锁
 */
class LogTransport
{
public:
    void Reset();                                      // 重置发送故障；仅在初始化时调用
    bool SupportsColor() const;                        // stderr 是否为终端
    bool Send(const TxFrame& frame);                    // 输出日志并恢复当前输入行
    bool BeginInput();                                 // 同一终端启用逐字输入；重定向时跳过
    bool UpdateInput(const char* line, size_t cursor);  // 保存输入及光标，重画提示符
    bool CommitInput();                                // 提交命令时换行并清空显示缓冲
    bool EndInput();                                   // 清除提示符，恢复原终端属性
    int LastError() const;                             // 首次发送或终端操作失败的 errno

private:
    bool Write(const char* text, size_t size);          // 失败后不重复写入失效后端
    bool DrawInput();                                  // 单行可视窗口，长命令横向滚动

    std::array<char, 256> input_{};                     // 当前命令副本，不引用 shell 缓冲
    size_t cursor_ = 0;                                // ASCII 命令光标位置
    struct termios saved_ {};                          // 启用编辑前的终端属性
    bool editing_ = false;                             // 终端属性已接管，需要恢复
    int error_ = 0;                                    // 0=正常；非零=首次后端故障
};

} // namespace debug
