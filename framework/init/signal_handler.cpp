/**
 * @file signal_handler.cpp
 * @author qingyu
 * @brief SIGPIPE 忽略与 SIGINT/SIGTERM 停止通知
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "signal_handler.hpp"
#include <csignal>
#include <stdexcept>

namespace init
{

namespace
{

volatile sig_atomic_t signal_stop = 0;

/**
 * @brief 异步信号上下文：只设置标志，不调用日志、锁或设备释放
 */
void HandleSignal(int)
{
    signal_stop = 1;
}

} // namespace

/**
 * @brief 建立进程退出信号处理，实际清理由正常执行上下文完成
 */
void InstallSignalHandlers()
{
    signal_stop = 0;

    struct sigaction pipe_action {};

    pipe_action.sa_handler = SIG_IGN;
    sigemptyset(&pipe_action.sa_mask);
    if (sigaction(SIGPIPE, &pipe_action, nullptr) != 0)
    {
        throw std::runtime_error("cannot ignore SIGPIPE");
    }

    struct sigaction action {};

    action.sa_handler = HandleSignal;
    sigemptyset(&action.sa_mask);

    if (sigaction(SIGINT, &action, nullptr) != 0 || sigaction(SIGTERM, &action, nullptr) != 0)
    {
        throw std::runtime_error("cannot install signal handlers");
    }
}

/**
 * @brief 查询是否收到退出信号
 */
bool SignalStopRequested()
{
    return signal_stop != 0;
}

} // namespace init
