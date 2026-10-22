/**
 * @file trd_calib.cpp
 * @author qingyu
 * @brief 标定线程 — 参数读取、任务拍、生命周期与快门 var
 * @version 0.2
 * @date 2026-10-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <filesystem>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <system_error>
#include <utility>

#include "trd_calib.hpp"
#include "framework/cmd/shell/log/log.hpp"
#include "framework/cmd/shell/var/var.hpp"
#include "framework/init/startup/Init_entry.hpp"
#include "framework/init/startup/System_startup.hpp"
#include "framework/topic/vision_to/vision_to.hpp"

namespace thread::calib
{

using algorithm::calib::SampleShape;

/**
 * @brief 取走一次快门请求并清除，等下一帧再判合格
 *
 * 与 shell 的 `var set calib.capture` 共用一把锁：shell 直接写这个变量，这里读走再写回 0，
 * 所以一次按键只会被消费一次。
 *
 * @return true 本次收到过快门
 */
static bool TakeShotRequest()
{
    std::lock_guard<std::mutex> lock(shutter_.mutex);
    const bool requested = shutter_.request;
    shutter_.request     = false;

    return requested;
}

/**
 * @brief 标定任务：取帧 → 画圆心 → 转发 → 按快门判合格
 *
 * 相机帧由相机线程发布；本线程取一帧、把圆心画在副本上再发到 vision_to，
 * 显示线程照旧只订阅 vision_to，所以标定构建下屏幕上就是带圆心标记的画面。
 *
 * 快门那一帧要过四道才算合格：识别出完整圆点阵、签名可算、与已保留样本够不同、
 * 距上次保留够 sample_interval_ms。不合格的打一行说明原因、不占样本位，要重试就再按一次。
 *
 * @param context 本线程的采集状态
 * @return Continue=继续下一拍；通道关闭或采集超时为 Fatal
 */
static init::Thread::TaskResult Task(void* context)
{
    auto& state = *static_cast<CaptureState*>(context);

    // 阻塞等待相机最新帧；相机通道关闭时退出标定线程
    topic::camera_to::Message message;
    if (!sub_camera_to.Wait(message))
    {
        return init::Thread::TaskResult::Fatal;
    }

    if (std::chrono::steady_clock::now() >= state.deadline)
    {
        DUST_LOG_ERR("calib: only %zu of %d samples within %ds", samples_.centers.size(), params_.sample_count, params_.capture_timeout_s);
        System_RequestStop(1);
        return init::Thread::TaskResult::Fatal;
    }

    // 每帧都识别一次：结果既画进转发的图，也是快门判合格的第一道
    std::vector<cv::Point2f> centers;
    const bool detected = algorithm::calib::DetectBoard(message.image, params_.board, centers);

    // 识别结果为空：这一帧只是画面，没有待发布的装甲板结果
    topic::vision_to::pub_vision_to_.Publish({DrawCenters(message.image, centers), message.timestamp, {}, 0.0});

    const auto now = std::chrono::steady_clock::now();
    if (now >= state.next_report)
    {
        DUST_LOG_INF("calib: waiting samples=%zu/%d shots=%d detected=%s", samples_.centers.size(), params_.sample_count, state.shots, detected ? "yes" : "no");
        state.next_report = now + std::chrono::seconds(kProgressIntervalS);
        state.shots       = 0;
    }

    if (!TakeShotRequest())
    {
        return init::Thread::TaskResult::Continue; // 没按快门就只当预览
    }
    ++state.shots;

    if (!detected)
    {
        DUST_LOG_WRN("calib: shot dropped: no circle grid in the frame");
        return init::Thread::TaskResult::Continue;
    }

    const std::optional<SampleShape> shape = algorithm::calib::MeasureShape(params_.board, centers);
    if (!shape)
    {
        DUST_LOG_WRN("calib: shot dropped: degenerate circle grid");
        return init::Thread::TaskResult::Continue;
    }
    if (!algorithm::calib::IsDistinctSample(*shape, samples_.shapes, params_.gate))
    {
        DUST_LOG_WRN("calib: shot dropped: pose repeats a kept sample");
        return init::Thread::TaskResult::Continue;
    }
    if (std::chrono::steady_clock::now() < state.next_accept)
    {
        DUST_LOG_WRN("calib: shot dropped: within sample_interval_ms of the last kept sample");
        return init::Thread::TaskResult::Continue;
    }

    // 先落图再记账：图写不出去说明目录有问题，这时上报比留一批没图的样本好
    const std::string name = std::to_string(samples_.centers.size() + 1) + ".jpg";
    const std::string path = params_.samples_dir + "/" + name;
    if (!cv::imwrite(path, message.image))
    {
        DUST_LOG_ERR("calib: cannot write %s", path.c_str());
        System_RequestStop(1);
        return init::Thread::TaskResult::Fatal;
    }

    // 存的是原始帧：圆心坐标要对应未被标记污染的像素
    samples_.image_size = message.image.size();
    samples_.shapes.push_back(*shape);
    samples_.centers.push_back(std::move(centers));
    state.next_accept = std::chrono::steady_clock::now() + std::chrono::milliseconds(params_.sample_interval_ms);

    DUST_LOG_INF("calib: sample %zu/%d center=(%.0f,%.0f) span=%.0fpx pitch=%.2f yaw=%.2f -> %s", samples_.centers.size(), params_.sample_count, shape->center.x,
                 shape->center.y, shape->span_px, shape->pitch_ratio, shape->yaw_ratio, name.c_str());

    FinishIfComplete();

    return init::Thread::TaskResult::Continue;
}

/**
 * @brief 读取配置并准备样本目录
 *
 * 相机由相机线程持有，这里只做本线程自己的准备；取帧在 thread_start 之后由 Task 完成。
 *
 * @return true 配置有效且目录可用
 * @throws std::exception 配置读取或字段校验失败
 */
static bool thread_init()
{
    LoadParams();

    std::error_code error;
    std::filesystem::create_directories(params_.samples_dir, error);
    if (error)
    {
        DUST_LOG_ERR("calib: cannot create %s: %s", params_.samples_dir.c_str(), error.message().c_str());
        return false;
    }

    DUST_LOG_INF("calib: board %dx%d center_distance=%.1fmm target=%d samples", params_.board.cols, params_.board.rows, params_.board.center_distance_mm, params_.sample_count);
    DUST_LOG_INF("calib: press the shutter with `var set calib.capture 1`");

    return true;
}

/**
 * @brief 启动标定工作线程
 * @return true 线程创建成功
 */
static bool thread_start()
{
    capture_.deadline    = std::chrono::steady_clock::now() + std::chrono::seconds(params_.capture_timeout_s);
    capture_.next_accept = std::chrono::steady_clock::now();
    capture_.next_report = capture_.next_accept;

    thread_.Start(Task, std::chrono::milliseconds(0), &capture_, "calib");

    return true;
}

/**
 * @brief 请求标定线程停止并关闭结果通道
 *
 * 关闭结果通道用于唤醒正在等待结果的显示侧；标定线程仍由 camera_to 关闭或
 * 当前帧处理完观察停止标志退出。
 */
static void thread_stop()
{
    thread_.RequestStop();
    topic::vision_to::pub_vision_to_.Close();
}

/**
 * @brief 等待标定线程退出
 */
static void thread_join()
{
    thread_.Join();
}

/**
 * @brief 清空结果通道槽内的图像，支持重复清理
 */
static void Release()
{
    topic::vision_to::pub_vision_to_.Clear();
}

REGISTER_INIT(thread_init, AppInit, High, HaltOnFail, Release, "calib_init");
REGISTER_THREAD(thread_start, AppThread, thread_stop, thread_join, "calib_start");

REGISTER_SHELL_VAR("calib.capture", shutter_.request, shutter_.mutex);

} // namespace thread::calib
