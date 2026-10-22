/**
 * @file trd_camera.hpp
 * @author qingyu
 * @brief 海康相机线程 — 调参命令槽、采集状态与运行状态
 * @version 0.2
 * @date 2026-10-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "framework/init/semaphore.hpp"
#include "framework/init/thread/thread.hpp"
#include "framework/modules/camera/hikrobot.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <mutex>

namespace thread::camera
{

/**
 * @brief shell 提交给相机线程的调参命令类型
 */
enum class Command
{
    SetExposure,
    SetGain
};

/**
 * @brief shell 提交给相机线程的单条曝光或增益设置命令
 */
struct CommandRequest
{
    Command command  = Command::SetExposure;
    double requested_value = 0.0;
    bool   pending   = false;                // 已提交待执行：提交者置位，相机线程取走
    bool   executing = false;                // 正在执行 SDK 设置：相机线程置位
    bool   completed = false;                // 结果可取：相机线程置位，提交者读走并清除
    std::exception_ptr error;                // 执行结果：nullptr 表示成功
};

/**
 * @brief 调参命令槽：同一时间只允许一条命令在途
 *
 * shell 线程填 request 并释放 signal，然后阻塞等 done；
 * 相机线程在 Task 里 TryAcquire 取走执行，做完置 completed 再唤醒提交者。
 */
struct CommandSlot
{
    init::Semaphore         signal {0};      // 计数信号量：通知相机线程有命令待执行
    std::mutex              mutex;           // 保护命令槽及执行结果
    CommandRequest          request {};      // 在途命令
    std::condition_variable done;            // 提交者等待执行结果
};

/**
 * @brief 采集拍与拍之间要保留的状态
 */
struct CaptureState
{
    std::chrono::steady_clock::time_point last_frame {};    // 最近一帧的 SDK 时间戳
    std::chrono::steady_clock::time_point last_report {};   // 上次 fps 汇总时刻
    bool         first_frame = true;                        // 首帧分辨率只打一次
    unsigned int frames      = 0;                           // 本汇总窗口内的帧数
};

/**
 * @brief 相机线程的运行状态
 */
struct RunState
{
    std::int32_t     max_frame_gap_ms = 0;      // 连续无有效帧时限，单位 ms
    std::atomic<bool> ready {false};            // 仅表示 Task 可以接受设置命令
};

inline modules::camera::HikRobot camera_ {};    // 相机设备，仅相机线程调用 SDK
inline init::Thread              thread_ {};    // 采集工作线程

inline CommandSlot  command_ {};    // 调参命令槽
inline CaptureState capture_ {};    // 采集拍间状态
inline RunState     run_ {};        // 运行状态

/**
 * @brief 取消尚未执行的相机设置命令并唤醒 shell
 * @param error 返回给提交者的异常
 */
void CancelCommand(std::exception_ptr error);

/**
 * @brief 提交曝光或增益设置命令并等待相机线程返回 SDK 结果
 * @param command 要执行的相机设置命令
 * @param requested_value 曝光时间或增益值
 * @throws std::exception 相机不可用、正在停止或 SDK 设置失败
 */
void SubmitCommand(Command command, double requested_value);

/**
 * @brief 非阻塞取出一条相机设置命令并执行 SDK 设置
 * @throws std::exception SDK 设置失败，交由恢复 hook 处理
 */
void ExecuteCommand();

} // namespace thread::camera
