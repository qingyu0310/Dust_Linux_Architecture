/**
 * @file System_startup.hpp
 * @author qingyu
 * @brief Linux 进程统一启动、等待停止和关闭接口
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once
/**
 * @brief 按阶段启动已注册组件，初始化项按 High → Mid → Low 执行；仅允许主线程调用一次
 *
 * @return true 启动成功；false 中止启动，调用方仍须执行 System_Shutdown
 * @throws std::exception 重复启动、注册项排序冲突或资源分配失败
 */
bool System_Startup();
/**
 * @brief 请求进程停止，可由业务线程调用
 *
 * @param exit_code 0 表示正常停止；非零状态在关闭时保留
 */
void System_RequestStop(int exit_code = 0);
/**
 * @brief 设置主线程事件回调，由统一等待循环调用
 *
 * @param callback 非阻塞事件处理函数；nullptr 表示取消
 * @note 仅由主线程设置，回调异常转为统一错误停止
 */
void System_SetMainLoop(void (*callback)());
/**
 * @brief 主线程等待退出信号或业务线程的停止请求
 */
void System_WaitForStop();
/**
 * @brief 通知全部线程停止，再逆序等待和释放资源；支持重复调用
 *
 * @return 0 正常退出；非零表示启动、运行或清理发生错误
 */
int System_Shutdown();
