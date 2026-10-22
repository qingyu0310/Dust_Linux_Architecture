/**
 * @file trd_display.hpp
 * @author qingyu
 * @brief 显示线程 — 显示开关、窗口状态与主线程回调参数
 * @version 0.2
 * @date 2026-10-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <condition_variable>
#include <cstdint>
#include <mutex>

namespace thread::display
{

/**
 * @brief 显示开关，shell 改开关、主线程消费
 */
struct ShowState
{
    bool                    show = true;   // true 显示，false 不显示
    std::mutex              mutex;         // 保护开关，并作为条件变量的锁
    std::condition_variable condition;     // 开关变化时唤醒主线程回调
};

/**
 * @brief 显示窗口与主线程回调参数
 */
struct WindowState
{
    bool         open = false;   // 窗口是否已创建，仅主线程访问
    std::int32_t wait_ms = 50;   // 无新帧时回调最长等待，只为泵 Qt 事件
};

inline ShowState   show_ {};
inline WindowState window_ {};

} // namespace thread::display
