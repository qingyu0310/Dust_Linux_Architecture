/**
 * @file trd_camera.cpp
 * @author qingyu
 * @brief 海康相机线程 — 参数读取、连续采集、断线重连与生命周期
 * @version 0.2
 * @date 2026-10-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_camera.hpp"
#include "framework/cmd/config/config.hpp"
#include "framework/cmd/shell/log/log.hpp"
#include "framework/init/startup/Init_entry.hpp"
#include "framework/init/signal_handler.hpp"
#include "framework/topic/camera_to/camera_to.hpp"
#include <chrono>
#include <cstdint>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <thread>
#include <utility>

namespace thread::camera
{

using namespace modules::camera;
using namespace topic::camera_to;
using namespace init;

/**
 * @brief 初始化相机，参数仅从本线程所属 YAML 读取
 *
 * 相机是整条视觉链路的底座：设备类故障（没插、没上电、USB 异常）时在这里一直重试，
 * 直到设备就绪才放行后续启动阶段，不放一个没有相机的系统上去跑。
 * 配置类错误仍然直接失败，不用重试掩盖。
 *
 * @return true 设备已打开；false 重试期间收到退出信号，启动中止
 * @throws std::exception 配置读取失败或非设备类初始化错误
 */
static bool thread_init()
{
    HikRobot::Config config;
    cmd::config::Config yaml;
    yaml.Load("thread/camera/trd_camera.yaml", {"exposure_us", "gain_db", "frame_rate", "timeout_ms", "max_frame_gap_ms"});

    // YAML 用 μs、HikRobot 用 ms：只在这里换算一次，两边不混用单位
    // 这里的上下限只防手滑，不是硬件上限：帧率上限由分辨率/曝光/链路带宽决定，
    // 曝光上限由设备节点决定（都靠设备自己报错或实际帧率体现，别在这儿替设备做主）
    config.exposure_ms = yaml.GetInt("exposure_us",      1,     10000000) / 1000.0;
    config.gain        = yaml.GetDouble("gain_db",       0.0,   48.0);
    config.frame_rate  = yaml.GetInt("frame_rate",       1,     1000);
    config.timeout_ms  = yaml.GetInt("timeout_ms",       1,     1000);
    run_.max_frame_gap_ms  = yaml.GetInt("max_frame_gap_ms", 1000,  60000);

    // 先等设备被系统枚举到，再初始化：设备没插时一次都不碰 MVS 运行时，
    // 反复 MV_CC_Initialize / MV_CC_Finalize 会把 SDK 拖进不稳定状态。
    bool waiting_logged = false;
    for (;;)
    {
        if (HikRobot::ProbeUsb(config))
        {
            if (waiting_logged)
            {
                DUST_LOG_OK("camera %s detected, initializing", config.vid_pid.c_str());
            }

            try
            {
                camera_.Init(config);
                break;
            }
            catch (const HikRobotError& error)
            {
                if (!error.IsRecoverable())
                {
                    throw; // 非设备类故障：照旧让启动失败，不用重试掩盖
                }
                DUST_LOG_WRN("camera not ready, retrying: %s", error.what());
            }
        }
        else if (!waiting_logged)
        {
            waiting_logged = true;
            DUST_LOG_WRN("camera %s not present, waiting for device", config.vid_pid.c_str());
        }

        if (init::SignalStopRequested())
        {
            return false; // 等待期间收到退出信号：中止启动，交给统一关闭流程
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    run_.ready.store(true);
    return true;
}

/**
 * @brief 异常恢复 hook：仅重试设备故障，清理失败则交由框架统一退出
 *
 * @return Restart 重新执行任务；Stop 无法恢复或收到停止请求
 */
static Thread::RecoveryAction Recover(const std::exception& error, Thread& worker, void*)
{
    run_.ready.store(false);                      // 先切断新命令：重连期间不再往设备写设置
    CancelCommand(std::current_exception());

    const auto* fault = dynamic_cast<const HikRobotError*>(&error);
    if (fault == nullptr || !fault->IsRecoverable())
    {
        return Thread::RecoveryAction::Stop;
    }
    DUST_LOG_WRN("camera recovery: %s", error.what());

    if (!camera_.Stop())
    {
        throw std::runtime_error("camera cleanup failed before reconnect");
    }

    pub_camera_to_.Clear(); // 丢掉断线前的旧帧，重连后只认新序号

    while (!worker.WaitForStop(std::chrono::milliseconds(1000)))
    {
        if (worker.StopRequested()) {
            return Thread::RecoveryAction::Stop;
        }

        try
        {
            camera_.Reopen();
            run_.ready.store(true);

            DUST_LOG_OK("camera ready; waiting for first frame");

            return Thread::RecoveryAction::Restart;
        }
        catch (const HikRobotError& failure)
        {
            if (!failure.IsRecoverable()) {
                throw;
            }

            if (!camera_.Stop()) {
                throw std::runtime_error("camera cleanup failed during reconnect");
            }

            DUST_LOG_WRN("camera reconnect failed: %s", failure.what());
        }
    }

    return Thread::RecoveryAction::Stop;
}

/**
 * @brief 采集一帧并提交，由主线程显示图像
 *
 * 一拍取一帧：SDK 限时等待后返回，不在采集线程调用任何窗口 API。
 * 连续无有效帧或 SDK 取帧失败时抛给恢复 hook，由它决定重连还是停止。
 *
 * @param context 本线程的采集状态
 * @return Continue=继续下一拍；失败经异常交给恢复 hook
 */
static Thread::TaskResult Task(void* context)
{
    auto& state = *static_cast<CaptureState*>(context);

    if (!camera_.IsOpen())
    {
        // 启动时设备没就绪，或运行中掉线：交给恢复 hook 重连，不在业务循环里自己重试
        throw HikRobotError("camera is not ready", 0, true);
    }

    ExecuteCommand();

    cv::Mat image;
    std::chrono::steady_clock::time_point timestamp;

    bool     received     = camera_.Read(image, timestamp);
    uint32_t frame_number = camera_.GetFrameNumber();

    if (received)
    {
        // 相机安装方向上下、左右均相反；-1 同时翻转两个轴，相当于旋转 180° 后再发布
        cv::flip(image, image, -1);

        state.last_frame = timestamp; // 以 SDK 帧时间戳为准，本地时间只用于统计
        ++state.frames;

        if (state.first_frame)
        {
            DUST_LOG_OK("camera image: %d x %d BGR", image.cols, image.rows);
            state.first_frame = false;
        }

        // 发布到相机话题：入槽即返回，没有订阅者取走的上一帧直接被覆盖丢弃
        pub_camera_to_.Publish({std::move(image), timestamp});
    }
    else if (std::chrono::steady_clock::now() - state.last_frame >= std::chrono::milliseconds(run_.max_frame_gap_ms))
    {
        // 抛给恢复 hook：重连还是停止由 Recover 决定，这里不自己重启设备
        throw HikRobotError("camera: no valid frame within max_frame_gap_ms", 0, true);
    }

    const auto   now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - state.last_report).count();
    if (elapsed >= 1.0)
    {
        DUST_LOG_DBG("camera", "fps=%.1f frame=%u published=%llu", state.frames / elapsed, frame_number, static_cast<unsigned long long>(pub_camera_to_.Published()));
        state.frames      = 0;
        state.last_report = now;
    }

    return Thread::TaskResult::Continue;
}

/**
 * @brief 启动相机任务，业务循环由独立 Task 承载
 * @return true 线程创建成功
 */
static bool thread_start()
{
    thread_.SetRecoveryHook(Recover);
    thread_.Start(Task, std::chrono::milliseconds(0), &capture_, "camera");

    return true;
}

/**
 * @brief 通知采集任务停止，正在进行的 SDK 等待按配置超时返回
 */
static void thread_stop()
{
    run_.ready.store(false);
    thread_.RequestStop();
    pub_camera_to_.Close(); // 唤醒等帧的订阅者，让它们跟着一起退出
    CancelCommand(std::make_exception_ptr(std::runtime_error("camera stopping")));
}

/**
 * @brief 等待采集任务退出，之后才允许关闭设备和清理帧缓存
 */
static void thread_join()
{
    thread_.Join();
    DUST_LOG_OK("camera thread joined");
}

/**
 * @brief 释放相机资源，支持初始化失败回滚和重复清理
 * @throws std::runtime_error SDK 清理失败
 */
static void Release()
{
    run_.ready.store(false);
    CancelCommand(std::make_exception_ptr(std::runtime_error("camera stopping")));

    const bool stopped = camera_.Stop();

    pub_camera_to_.Clear(); // 释放槽内图像，保留序号

    if (!stopped)
    {
        throw std::runtime_error("camera resource cleanup failed");
    }
}

REGISTER_INIT(thread_init, EarlyInit, High, HaltOnFail, Release, "camera_init");
REGISTER_THREAD(thread_start, EarlyThread, thread_stop, thread_join, "camera_start");

} // namespace thread::camera
