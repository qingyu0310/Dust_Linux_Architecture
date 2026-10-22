/**
 * @file trd_camera_cmd.cpp
 * @author qingyu
 * @brief 相机线程调参命令 — shell 提交、相机线程执行、结果回传
 * @version 0.1
 * @date 2026-10-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_camera.hpp"
#include "framework/cmd/shell/var/var.hpp"
#include <mutex>
#include <stdexcept>
#include <utility>

namespace thread::camera
{

/**
 * @brief 取消尚未执行的相机设置命令并唤醒 shell
 * @param error 返回给提交者的异常
 */
void CancelCommand(std::exception_ptr error)
{
    bool notify = false;
    {
        std::lock_guard<std::mutex> lock(command_.mutex);
        if (command_.request.pending && !command_.request.executing)
        {
            command_.request.pending = false;
            command_.request.completed = true;
            command_.request.error = std::move(error);
            notify = true;
        }
    }

    if (notify)
    {
        command_.done.notify_all();
    }
}

/**
 * @brief 提交曝光或增益设置命令并等待相机线程返回 SDK 结果
 *
 * 由 shell 线程调用，提交后阻塞到相机线程做完；不做排队，在途命令占位时直接报忙。
 *
 * @param command 要执行的相机设置命令
 * @param requested_value 曝光时间或增益值
 * @throws std::exception 相机不可用、正在停止或 SDK 设置失败
 */
void SubmitCommand(Command command, double requested_value)
{
    if (!run_.ready.load() || thread_.StopRequested()) {
        throw std::runtime_error("camera unavailable");
    }

    std::unique_lock<std::mutex> lock(command_.mutex);
    // 锁内复查：等锁期间停止流程可能已经清掉就绪标志或取消本命令
    if (!run_.ready.load() || thread_.StopRequested()) {
        throw std::runtime_error("camera unavailable");
    }
    if (command_.request.pending || command_.request.executing || command_.request.completed) {
        throw std::runtime_error("camera command busy");
    }

    command_.request.command = command;
    command_.request.requested_value = requested_value;
    command_.request.pending = true;
    command_.request.error = nullptr;

    command_.signal.Release();
    command_.done.wait(lock, [] { return command_.request.completed; });

    auto error = command_.request.error;
    command_.request.completed = false;
    command_.request.error = nullptr;

    lock.unlock();
    if (error != nullptr)
    {
        std::rethrow_exception(error);
    }
}

/**
 * @brief 非阻塞取出一条相机设置命令并执行 SDK 设置
 *
 * 用信号量而不是阻塞等待：采集循环不能被命令等待挡住。
 * SDK 调用不持 command_.mutex，否则 shell 提交新命令会被这几毫秒的写入挡住。
 */
void ExecuteCommand()
{
    if (!command_.signal.TryAcquire()) {
        return;
    }

    Command command;
    double requested_value;
    {
        std::lock_guard<std::mutex> lock(command_.mutex);
        if (!command_.request.pending)
        {
            return;
        }

        command = command_.request.command;
        requested_value = command_.request.requested_value;
        command_.request.pending = false;
        command_.request.executing = true;
    }

    std::exception_ptr error;
    try
    {
        if (command == Command::SetExposure)
        {
            camera_.SetExposure(requested_value);
        }
        else
        {
            camera_.SetGain(requested_value);
        }
    }
    catch (...)
    {
        error = std::current_exception();
    }

    {
        std::lock_guard<std::mutex> lock(command_.mutex);
        command_.request.executing = false;
        command_.request.completed = true;
        command_.request.error = error;
    }
    command_.done.notify_all();

    // 先让提交者拿到结果，再把同一个异常抛给框架：重连还是停止由恢复 hook 决定
    if (error != nullptr)
    {
        std::rethrow_exception(error);
    }
}

/**
 * @brief 提交曝光请求，由相机线程设置设备并返回结果
 *
 * 在 shell 线程执行，会阻塞到相机线程设完；改设备必须绕这一圈，不能在 shell 里直接写 SDK。
 *
 * @param entry 相机类内曝光条目
 * @param value 已钳制曝光，单位 μs
 * @param context 相机实例
 * @throws std::exception 设备未就绪、正在停止或 SDK 设置失败
 */
static void SetExposureFromVar(const debug::VarEntry& entry, double value, void* context)
{
    (void)entry; // 范围钳制已由 var 机制完成，回调只用 value
    if (context != &camera_)
    {
        throw std::logic_error("invalid camera context");
    }
    SubmitCommand(Command::SetExposure, value / 1000.0); // var 用 μs，SDK 用 ms
}

REGISTER_SHELL_VAR_RANGE_NOTIFY("camera_exposure_us", camera_.GetParameters().exposure_us,
    camera_.GetParameters().exposure_min_us, camera_.GetParameters().exposure_max_us,
    camera_.GetParameters().mutex, SetExposureFromVar, &camera_);

/**
 * @brief 提交增益请求，由相机线程设置设备并返回结果
 *
 * 与曝光同一条路径：shell 只提交，SDK 写入固定在相机线程。
 *
 * @param entry 相机类内增益条目
 * @param value 已钳制增益，单位 dB
 * @param context 相机实例
 * @throws std::exception 设备未就绪、正在停止或 SDK 设置失败
 */
static void SetGainFromVar(const debug::VarEntry& entry, double value, void* context)
{
    (void)entry; // 范围钳制已由 var 机制完成，回调只用 value
    if (context != &camera_)
    {
        throw std::logic_error("invalid camera context");
    }
    SubmitCommand(Command::SetGain, value);
}

REGISTER_SHELL_VAR_RANGE_NOTIFY("camera_gain_db", camera_.GetParameters().gain_db,
    camera_.GetParameters().gain_min_db, camera_.GetParameters().gain_max_db,
    camera_.GetParameters().mutex, SetGainFromVar, &camera_);

} // namespace thread::camera
