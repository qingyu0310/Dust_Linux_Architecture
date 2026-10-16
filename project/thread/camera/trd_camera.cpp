/**
 * @file trd_camera.cpp
 * @author qingyu
 * @brief 海康相机线程 — 参数读取、连续采集与实时图像显示
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "framework/cmd/config/config.hpp"
#include "framework/cmd/shell/log.hpp"
#include "framework/cmd/shell/var.hpp"
#include "framework/init/Init_entry.hpp"
#include "framework/init/semaphore.hpp"
#include "framework/init/signal_handler.hpp"
#include "framework/init/thread.hpp"
#include "framework/modules/camera/hikrobot.hpp"
#include "framework/topic/camera_to/camera_to.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <mutex>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <thread>
#include <utility>

namespace thread::camera
{

using namespace modules::camera;
using namespace topic::camera_to;
using namespace init;

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

// 调参命令槽：同一时间只允许一条命令在途。
// shell 线程填 request_ 并 Release 信号量，然后阻塞等 condition_；
// 相机线程在 Task 循环里 TryAcquire 取走执行，做完置 completed 再唤醒提交者。
static Semaphore      signal_ {0};                   // 计数信号量：通知相机线程有命令待执行
static std::mutex     mutex_;                                       // 保护命令槽及执行结果
static CommandRequest request_ {};
static std::condition_variable condition_;                          // 提交者等待执行结果

static int max_frame_gap_ms_ = 0;                                   // 连续无有效帧时限，单位 ms
static std::atomic<bool> camera_ready_ {false};                  // 仅表示 Task 可以接受设置命令

static HikRobot camera_ {};

static Thread   thread_ {};

/**
 * @brief 取消尚未执行的相机设置命令并唤醒 shell
 * @param error 返回给提交者的异常
 */
static void CancelCommand(std::exception_ptr error)
{
    bool notify = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (request_.pending && !request_.executing)
        {
            request_.pending = false;
            request_.completed = true;
            request_.error = std::move(error);
            notify = true;
        }
    }

    if (notify)
    {
        condition_.notify_all();
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
static void SubmitCommand(Command command, double requested_value)
{
    if (!camera_ready_.load() || thread_.StopRequested()) {
        throw std::runtime_error("camera unavailable");
    }

    std::unique_lock<std::mutex> lock(mutex_);
    // 锁内复查：等锁期间停止流程可能已经清掉就绪标志或取消本命令
    if (!camera_ready_.load() || thread_.StopRequested()) {
        throw std::runtime_error("camera unavailable");
    }
    if (request_.pending || request_.executing || request_.completed) {
        throw std::runtime_error("camera command busy");
    }

    request_.command = command;
    request_.requested_value = requested_value;
    request_.pending = true;
    request_.error = nullptr;

    signal_.Release();
    condition_.wait(lock, [] { return request_.completed; });

    auto error = request_.error;
    request_.completed = false;
    request_.error = nullptr;

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
 * SDK 调用不持 mutex_，否则 shell 提交新命令会被这几毫秒的写入挡住。
 */
static void ExecuteCommand()
{
    if (!signal_.TryAcquire()) {
        return;
    }

    Command command;
    double requested_value;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!request_.pending)
        {
            return;
        }

        command = request_.command;
        requested_value = request_.requested_value;
        request_.pending = false;
        request_.executing = true;
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
        std::lock_guard<std::mutex> lock(mutex_);
        request_.executing = false;
        request_.completed = true;
        request_.error = error;
    }
    condition_.notify_all();

    // 先让提交者拿到结果，再把同一个异常抛给框架：重连还是停止由恢复 hook 决定
    if (error != nullptr)
    {
        std::rethrow_exception(error);
    }
}

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
bool thread_init()
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
    max_frame_gap_ms_  = yaml.GetInt("max_frame_gap_ms", 1000,  60000);

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

    camera_ready_.store(true);
    return true;
}

/**
 * @brief 异常恢复 hook：仅重试设备故障，清理失败则交由框架统一退出
 *
 * @return Restart 重新执行任务；Stop 无法恢复或收到停止请求
 */
static Thread::RecoveryAction Recover(const std::exception& error, Thread& worker, void*)
{
    camera_ready_.store(false);                      // 先切断新命令：重连期间不再往设备写设置
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
            camera_ready_.store(true);

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
 * @brief 连续采集并提交最新帧，由主线程显示图像
 *
 * SDK 限时等待后检查停止标志；不在采集线程调用任何窗口 API。
 * 连续无有效帧或 SDK 取帧失败时进入重连，等待可由停止请求唤醒。
 */
static void Task(void*, void*, void*)
{
    auto last_frame  = std::chrono::steady_clock::now();
    auto last_report = last_frame;

    bool first_frame = true;
    unsigned int frames = 0;

    while (!thread_.StopRequested())
    {
        if (!camera_.IsOpen()) 
        {
            // 启动时设备没就绪，或运行中掉线：交给恢复 hook 重连，不在业务循环里自己重试
            throw HikRobotError("camera is not ready", 0, true);
        }

        ExecuteCommand();

        if (thread_.StopRequested()) {
            break;
        }

        cv::Mat image;
        std::chrono::steady_clock::time_point timestamp;

        bool     received     = camera_.Read(image, timestamp);
        uint32_t frame_number = camera_.GetFrameNumber();

        if (received)
        {
            // SDK 等待期间收到停止请求：丢掉这一帧，不再往下走
            if (thread_.StopRequested()) {
                break;
            }

            // 相机安装方向上下、左右均相反；-1 同时翻转两个轴，相当于旋转 180° 后再发布
            cv::flip(image, image, -1);

            last_frame = timestamp; // 以 SDK 帧时间戳为准，本地时间只用于统计
            ++frames;

            if (first_frame)
            {
                DUST_LOG_OK("camera image: %d x %d BGR", image.cols, image.rows);
                first_frame = false;
            }

            // 发布到相机话题：入槽即返回，没有订阅者取走的上一帧直接被覆盖丢弃
            pub_camera_to_.Publish({std::move(image), timestamp});
        }
        else if (std::chrono::steady_clock::now() - last_frame >= std::chrono::milliseconds(max_frame_gap_ms_))
        {
            // 抛给恢复 hook：重连还是停止由 Recover 决定，这里不自己重启设备
            throw HikRobotError("camera: no valid frame within max_frame_gap_ms", 0, true);
        }

        const auto   now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - last_report).count();
        if (elapsed >= 1.0)
        {
            DUST_LOG_DBG("camera", "fps=%.1f frame=%u published=%llu", frames / elapsed, frame_number, static_cast<unsigned long long>(pub_camera_to_.Published()));
            frames      = 0;
            last_report = now;
        }
    }
}

/**
 * @brief 启动相机任务，业务循环由独立 Task 承载
 * @return true 线程创建成功
 */
bool thread_start()
{
    thread_.SetRecoveryHook(Recover);
    thread_.Start(Task, nullptr, "camera");

    return true;
}

/**
 * @brief 通知采集任务停止，正在进行的 SDK 等待按配置超时返回
 */
void thread_stop()
{
    camera_ready_.store(false);
    thread_.RequestStop();
    pub_camera_to_.Close(); // 唤醒等帧的订阅者，让它们跟着一起退出
    CancelCommand(std::make_exception_ptr(std::runtime_error("camera stopping")));
}

/**
 * @brief 等待采集任务退出，之后才允许关闭设备和清理帧缓存
 */
void thread_join()
{
    thread_.Join();
    DUST_LOG_OK("camera thread joined");
}

/**
 * @brief 释放相机资源，支持初始化失败回滚和重复清理
 * @throws std::runtime_error SDK 清理失败
 */
void Release()
{
    camera_ready_.store(false);
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
