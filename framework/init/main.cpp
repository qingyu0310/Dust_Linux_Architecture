/**
 * @file main.cpp
 * @author qingyu
 * @brief Linux 系统入口：调用统一启动层，等待停止后关闭
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "System_startup.hpp"

/**
 * @brief 系统入口，不持有项目对象或业务初始化列表
 * @return 统一关闭后的进程退出状态
 */
int main()
{
    if (System_Startup())
    {
        System_WaitForStop();
    }
    return System_Shutdown();
}
