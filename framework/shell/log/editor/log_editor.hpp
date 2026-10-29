/**
 * @file log_editor.hpp
 * @author qingyu
 * @brief [终端编辑层] 命令行的显示与重绘 — 与日志输出串行共用同一终端
 *
 * @version 0.1
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 * # 为什么单独一层
 *
 * 日志输出与命令行输入抢同一块终端：日志要整行打，命令要原地重绘。
 * 把「写字节」和「管光标」分开——`transport/` 只写字节，本层管行内容、光标与终端属性，
 * 加工层在每次发送前后调 `BeforeSend` / `AfterSend` 协调。
 *
 * 本层**不产生日志**，也不碰队列；它只知道自己正在显示哪一行。
 */

#pragma once

#include <array>
#include <cstddef>
#include <termios.h>

namespace debug
{

class LogTransport;

/**
 * @brief 绑定终端输出后端，由加工层在初始化时调用
 * @param backend 输出后端
 */
void BindBackend(LogTransport* backend);

/**
 * @brief 终端行编辑：接管原始终端模式、保存当前输入行、按需重绘
 *
 * 只在 stdin 与 stderr 指向同一终端时启用；重定向或 TERM=dumb 时全部退化为空操作。
 */
class LogEditor
{
public:
    /**
     * @brief 保存输入行并清空光标
     * @param line   当前命令
     * @param cursor 光标位置
     * @return true 成功；false 参数越界
     */
    bool Update(const char* line, size_t cursor);

    /**
     * @brief 提交命令时换行并清空显示缓冲
     * @return true 成功
     */
    bool Commit();

    /**
     * @brief 清掉当前输入行显示，供日志整行输出前调用
     * @return true 成功
     */
    bool Clear();

    /**
     * @brief 按当前输入行与光标重绘提示符
     * @return true 成功
     */
    bool Draw();

    /**
     * @brief 是否已接管终端（决定加工层要不要调 Clear/Draw）
     */
    bool Editing() const { return editing_; }

    /**
     * @brief 启用原始终端模式，关闭内核回显与整行输入，保留信号按键
     *
     * 重定向、TERM=dumb、stdin/stderr 不同终端时保持原输入模式。
     *
     * @return true 成功（含「不需要接管」的情况）
     */
    bool Begin();

    /**
     * @brief 恢复原终端属性；允许重复调用
     * @return true 成功
     */
    bool End();

private:
    std::array<char, 256> input_{};     // 当前命令副本，不引用 shell 缓冲
    size_t   cursor_ = 0;               // ASCII 命令光标位置
    termios  saved_ {};                 // 启用编辑前的终端属性
    bool     editing_ = false;          // 终端属性已接管，需要恢复
};

} // namespace debug
