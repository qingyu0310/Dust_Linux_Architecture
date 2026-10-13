/**
 * @file trd_test.cpp
 * @author qingyu
 * @brief 基础业务装配：初始化注册、心跳线程与退出清理
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "framework/cmd/config/config.hpp"
#include <algorithm>
#include "framework/init/Init_entry.hpp"
#include "framework/init/System_startup.hpp"
#include "framework/init/thread.hpp"
#include "framework/cmd/shell/log.hpp"

namespace thread::test
{
/**
 * @brief test 线程参数，只由本线程初始化项读取和校验
 */
struct Params
{
    int heartbeat_ms;    // 心跳间隔，单位 ms，范围 [1, 60000]
    int run_duration_ms; // 运行时长，单位 ms；0=等待退出信号
};

static Params params_{};
static init::Thread thread_{};

/**
 * @brief 初始化基础应用，当前不访问摄像头设备
 */
bool thread_init()
{
    cmd::config::Config yaml;
    yaml.Load("thread/test/trd_test.yaml", {"heartbeat_ms", "run_duration_ms"});

    params_.heartbeat_ms = yaml.GetInt("heartbeat_ms", 1, 60000);
    params_.run_duration_ms = yaml.GetInt("run_duration_ms", 0, 86400000);

    DUST_LOG_OK("test init");
    return true;
}

/**
 * @brief 释放项目资源，支持未初始化和部分初始化状态
 */
void Release()
{
    DUST_LOG_INF("application resources released");
}

/**
 * @brief 基础线程任务：可中断等待、心跳与有限时长停止
 */
static void Task(void*, void*, void*)
{
    const auto begin = std::chrono::steady_clock::now();

    for (;;)
    {
        auto delay = std::chrono::milliseconds(params_.heartbeat_ms);

        if (params_.run_duration_ms > 0)
        {
            const auto elapsed   = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin);
            const auto remaining = std::chrono::milliseconds(params_.run_duration_ms) - elapsed;

            if (remaining.count() <= 0)
            {
                System_RequestStop();
                break;
            }
            delay = std::min(delay, remaining);
        }

        if (thread_.WaitForStop(delay))
        {
            break;
        }
        DUST_LOG_DBG("test", "application heartbeat");
    }
}

/**
 * @brief 启动线程，任务逻辑由独立 Task 函数承载
 */
bool thread_start()
{
    thread_.Start(Task, nullptr, "test");
    return true;
}

/**
 * @brief 通知业务线程停止并解除等待
 */
void thread_stop()
{
    thread_.RequestStop();
}

/**
 * @brief 等待业务线程退出，确保后续资源释放安全
 */
void thread_join()
{
    thread_.Join();
    DUST_LOG_OK("application thread joined");
}

REGISTER_INIT(thread_init, AppInit, High, HaltOnFail, Release, "test_init");
REGISTER_THREAD(thread_start, AppThread, thread_stop, thread_join, "test_start");

} // namespace thread::test
