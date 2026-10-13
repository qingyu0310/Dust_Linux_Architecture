/**
 * @file signal_handler.hpp
 * @author qingyu
 * @brief Linux 退出信号处理接口
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

namespace init
{
/**
 * @brief 安装 SIGINT/SIGTERM 处理函数，运行于主线程启动阶段
 *
 * @throws std::runtime_error 信号处理安装失败
 */
void InstallSignalHandlers();

/**
 * @brief 读取退出信号标志
 *
 * @return true 已收到退出信号
 */
bool SignalStopRequested();

} // namespace init
