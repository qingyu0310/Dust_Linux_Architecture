/**
 * @file trd_display_var.cpp
 * @author qingyu
 * @brief 显示线程调参 — display.show 的 var 注册与开关回调
 * @version 0.1
 * @date 2026-10-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_display.hpp"
#include "framework/cmd/shell/var/var.hpp"
#include <mutex>
#include <stdexcept>

namespace thread::display
{

/**
 * @brief var 设置回调，由 shell 线程调用
 *
 * 只改开关并唤醒主线程：窗口的创建与销毁必须落在主线程（Qt 约束）。
 * 开关没有硬约束范围，所以用不带范围的 REGISTER_SHELL_VAR_NOTIFY 注册。
 *
 * @param entry 显示开关条目
 * @param value 新值，非 0 表示显示
 * @param context 被绑定的开关变量
 * @throws std::logic_error 上下文不是本模块的开关
 */
static void SetShowFromVar(const debug::VarEntry& entry, double value, void* context)
{
    (void)entry; // 回调只用 value
    if (context != &show_.show)
    {
        throw std::logic_error("invalid display context");
    }
    {
        std::lock_guard<std::mutex> lock(show_.mutex);
        show_.show = value != 0.0;
    }
    show_.condition.notify_all();
}

/**
 * @brief 显示开关：非 0 显示、0 不显示；无范围约束，由 var 机制提交新值
 */
REGISTER_SHELL_VAR_NOTIFY("display.show", show_.show, show_.mutex, SetShowFromVar, &show_.show);

} // namespace thread::display
