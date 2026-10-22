/**
 * @file trd_calib.cpp
 * @author qingyu
 * @brief 标定线程 — 订阅相机帧、把圆心画上后转发，按快门采样本、解内参并打印
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "framework/algorithm/calib/camera_calib.hpp"
#include "framework/cmd/config/config.hpp"
#include "framework/cmd/shell/log/log.hpp"
#include "framework/cmd/shell/var/var.hpp"
#include "framework/init/startup/Init_entry.hpp"
#include "framework/init/startup/System_startup.hpp"
#include "framework/init/thread/thread.hpp"
#include "framework/topic/camera_to/camera_to.hpp"
#include "framework/topic/vision_to/vision_to.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace thread::calib
{

using algorithm::calib::Board;
using algorithm::calib::CaptureGate;
using algorithm::calib::Intrinsics;
using algorithm::calib::SampleShape;

// 等待情况每隔这么久打印一次，单位 s；采集期间逐帧打印会刷屏
constexpr std::int32_t kProgressIntervalS = 5;

// 主点相对图像几何中心的允许偏移比例；参考那台偏移 26.09 px，合 0.024·H
constexpr double kMaxPrincipalOffsetRatio = 0.05;

// fx 与 fy 的允许相对偏差；方形像素下应为 1，参考那台是 1.00044
constexpr double kMaxFocalRatioDeviation = 0.01;

// 圆心标记的半径与线宽，单位 pixel
constexpr std::int32_t kMarkerRadiusPx    = 8;
constexpr std::int32_t kMarkerThicknessPx = 2;

// 圆心标记按行取色：行序或列序反了（转置）时颜色分布会立刻不对
const std::array<cv::Scalar, 6> kRowColors {{{0, 0, 255}, {0, 200, 255}, {0, 255, 0}, {255, 200, 0}, {255, 0, 0}, {255, 0, 200}}};

/**
 * @brief 标定组件的运行参数，只由本组件的初始化项读取
 */
struct Params
{
    Board        board;                                             // 标定板规格
    CaptureGate  gate;                                              // 快门合格性门限
    std::int32_t sample_count {0};                                  // 需要保留的合格样本数
    std::int32_t sample_interval_ms {0};                            // 两次保留之间的最短间隔，单位 ms
    std::int32_t capture_timeout_s {0};                             // 采集总时限，单位 s
    double       max_reprojection_error_px {0.0};                   // 重投影误差上限，单位 pixel
    std::string  samples_dir;                                       // 合格样本图片目录，已按配置根解析
};

static Params params_ {};

static topic::camera_to::Subscriber sub_camera_to {topic::camera_to::pub_camera_to_};  // 本线程独占的相机帧游标

static init::Thread thread_ {};                                     // 标定工作线程

static std::vector<std::vector<cv::Point2f>> samples_;              // 已保留样本的圆心
static std::vector<SampleShape> shapes_;                            // 与 samples_ 一一对应的签名
static cv::Size image_size_ {};                                     // 合格样本那一帧的尺寸，求解时当作标定尺寸
static bool solved_ {false};                                        // 本轮是否已经求解并打印过

static bool capture_request_ {false};                               // shell 的快门请求：非 0 表示按了一次快门
static std::mutex capture_mutex_;                                   // 保护快门请求，同时作为该 var 条目的锁

/**
 * @brief 标定拍与拍之间要保留的状态
 *
 * 循环搬进线程外壳后，这些量不能再是任务体的局部变量，由 Task 的 context 带入。
 */
struct CaptureState
{
    std::chrono::steady_clock::time_point deadline {};       // 采集总时限的绝对时刻
    std::chrono::steady_clock::time_point next_accept {};    // 下次允许保留样本的时刻
    std::chrono::steady_clock::time_point next_report {};    // 下次进度汇报的时刻
    std::int32_t shots = 0;                                  // 本汇报周期内的快门次数
};

static CaptureState capture_ {};

/**
 * @brief 读取本组件 YAML：字段全部必需
 *
 * @throws std::exception 文件缺失、字段不符或数值越界
 */
static void LoadParams()
{
    cmd::config::Config yaml;
    yaml.Load("thread/calib/trd_calib.yaml",
              {"pattern_cols", "pattern_rows", "center_distance_mm", "sample_count", "min_shift_px", "min_scale_change", "min_ratio_change", "sample_interval_ms", "capture_timeout_s", "max_reprojection_error_px", "samples_dir"});

    params_.board.cols                = yaml.GetInt("pattern_cols",          2,          1000);
    params_.board.rows                = yaml.GetInt("pattern_rows",          2,          1000);
    params_.board.center_distance_mm  = yaml.GetDouble("center_distance_mm", 0.001,      100000.0);

    params_.sample_count              = yaml.GetInt("sample_count",          3,          1000);
    params_.sample_interval_ms        = yaml.GetInt("sample_interval_ms",    0,          600000);
    params_.capture_timeout_s         = yaml.GetInt("capture_timeout_s",     1,          86400);
    params_.samples_dir               = yaml.GetPath("samples_dir");
    params_.max_reprojection_error_px = yaml.GetDouble("max_reprojection_error_px", 0.0, 1000.0);

    params_.gate.min_shift_px         = static_cast<float>(yaml.GetDouble("min_shift_px",     0.0, 100000.0));
    params_.gate.min_span_change      = static_cast<float>(yaml.GetDouble("min_scale_change", 0.0, 1.0));
    params_.gate.min_ratio_change     = static_cast<float>(yaml.GetDouble("min_ratio_change", 0.0, 100.0));
}

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
    std::lock_guard<std::mutex> lock(capture_mutex_);
    const bool requested = capture_request_;
    capture_request_     = false;

    return requested;
}

/**
 * @brief 在每个认出来的圆心上套一个圈，画在副本上
 *
 * 圈按行换色：圆点阵的行序或列序要是反了（转置），颜色分布会立刻不对，比纯看位置容易发现。
 * 画的是 clone——通道里的像素与显示侧共享，约定要求改图的一方自己复制。
 *
 * @param image 相机帧，整幅 BGR
 * @param centers 本帧识别到的圆心，识别失败时为空
 * @return 叠加好圆心的整幅图像
 */
static cv::Mat DrawCenters(const cv::Mat& image, const std::vector<cv::Point2f>& centers)
{
    cv::Mat drawn = image.clone();

    const std::size_t cols = static_cast<std::size_t>(params_.board.cols);
    for (std::size_t index = 0; index < centers.size(); ++index)
    {
        cv::circle(drawn, centers[index], kMarkerRadiusPx, kRowColors[(index / cols) % kRowColors.size()], kMarkerThicknessPx, cv::LINE_AA);
    }

    return drawn;
}

/**
 * @brief 按保留到的样本反解内参、畸变与重投影误差
 *
 * @param intrinsics 输出的求解结果
 * @return true 求解成功
 */
static bool Solve(Intrinsics& intrinsics)
{
    const std::optional<Intrinsics> solved = algorithm::calib::SolveIntrinsics(params_.board, samples_, image_size_);
    if (!solved)
    {
        DUST_LOG_ERR("calib: solve failed, samples=%zu image=%dx%d", samples_.size(), image_size_.width, image_size_.height);
        return false;
    }

    intrinsics = *solved;
    return true;
}

/**
 * @brief 三项验收：重投影误差、方形像素、主点位置
 *
 * 门限从参考那台的数字来：重投影 0.0595 px、fx/fy = 1.00044、主点偏移 0.024·H。
 * 任一项不过都说明这批样本不足以解出可信内参，宁可判失败也不报出坏值。
 *
 * @param intrinsics 求解结果
 * @return true 三项都在界内
 */
static bool VerifyIntrinsics(const Intrinsics& intrinsics)
{
    if (!(intrinsics.reprojection_error_px <= params_.max_reprojection_error_px))
    {
        DUST_LOG_ERR("calib: reprojection_error %.4fpx exceeds %.4fpx", intrinsics.reprojection_error_px, params_.max_reprojection_error_px);
        return false;
    }

    const double fx = intrinsics.camera_matrix[0];
    const double fy = intrinsics.camera_matrix[4];
    if (!(fy > 0.0) || std::abs(fx / fy - 1.0) > kMaxFocalRatioDeviation)
    {
        DUST_LOG_ERR("calib: fx=%.4f fy=%.4f deviate more than %.2f", fx, fy, kMaxFocalRatioDeviation);
        return false;
    }

    const double x_limit = static_cast<double>(intrinsics.image_width) * kMaxPrincipalOffsetRatio;
    const double y_limit = static_cast<double>(intrinsics.image_height) * kMaxPrincipalOffsetRatio;
    if (std::abs(intrinsics.camera_matrix[2] - static_cast<double>(intrinsics.image_width) * 0.5) > x_limit ||
        std::abs(intrinsics.camera_matrix[5] - static_cast<double>(intrinsics.image_height) * 0.5) > y_limit)
    {
        DUST_LOG_ERR("calib: principal point (%.2f,%.2f) is too far from the image centre", intrinsics.camera_matrix[2], intrinsics.camera_matrix[5]);
        return false;
    }

    return true;
}

/**
 * @brief 按配置文件的键名打印一次标定结果，好让人直接复制粘贴
 *
 * 每条日志在彩色模式下只有约 101 字节可用，所以一行一个键，不往一行里堆。
 *
 * @param intrinsics 求解结果
 */
static void LogIntrinsics(const Intrinsics& intrinsics)
{
    DUST_LOG_INF("calib: image_width: %d", intrinsics.image_width);
    DUST_LOG_INF("calib: image_height: %d", intrinsics.image_height);
    DUST_LOG_INF("calib: camera_fx: %.10g", intrinsics.camera_matrix[0]);
    DUST_LOG_INF("calib: camera_fy: %.10g", intrinsics.camera_matrix[4]);
    DUST_LOG_INF("calib: camera_cx: %.10g", intrinsics.camera_matrix[2]);
    DUST_LOG_INF("calib: camera_cy: %.10g", intrinsics.camera_matrix[5]);
    DUST_LOG_INF("calib: distort_k1: %.10g", intrinsics.distort_coeffs[0]);
    DUST_LOG_INF("calib: distort_k2: %.10g", intrinsics.distort_coeffs[1]);
    DUST_LOG_INF("calib: distort_p1: %.10g", intrinsics.distort_coeffs[2]);
    DUST_LOG_INF("calib: distort_p2: %.10g", intrinsics.distort_coeffs[3]);
    DUST_LOG_INF("calib: distort_k3: %.10g", intrinsics.distort_coeffs[4]);
    DUST_LOG_INF("calib: reprojection_error_px: %.6g", intrinsics.reprojection_error_px);
}

/**
 * @brief 保留满样本后求解、验收并打印一次
 *
 * 只在第一次收满时执行：验收不过就请求统一停止（这批样本解不出可信内参，继续转着也没意义），
 * 通过则打印 12 个键供人粘进视觉线程的 YAML，线程继续转着让画面留在屏幕上。
 */
static void FinishIfComplete()
{
    if (solved_ || static_cast<std::int32_t>(samples_.size()) < params_.sample_count)
    {
        return;
    }
    solved_ = true;

    Intrinsics intrinsics;
    if (!Solve(intrinsics) || !VerifyIntrinsics(intrinsics))
    {
        System_RequestStop(1);
        return;
    }

    LogIntrinsics(intrinsics);
    DUST_LOG_OK("calib: paste the keys above into thread/vision/trd_vision.yaml");
}

/**
 * @brief 标定任务：取帧 → 画圆心 → 转发 → 按快门判合格
 *
 * 相机帧由相机线程发布；本线程取一帧、把圆心画在副本上再发到 vision_to，
 * 显示线程照旧只订阅 vision_to，所以标定构建下屏幕上就是带圆心标记的画面。
 *
 * 快门那一帧要过四道才算合格：识别出完整圆点阵、签名可算、与已保留样本够不同、
 * 距上次保留够 sample_interval_ms。不合格的打一行说明原因、不占样本位，要重试就再按一次。
 */
static Thread::TaskResult Task(void* context)
{
    auto& state = *static_cast<CaptureState*>(context);

    // 阻塞等待相机最新帧；相机通道关闭时退出标定线程
    topic::camera_to::Message message;
    if (!sub_camera_to.Wait(message))
    {
        return Thread::TaskResult::Fatal;
    }

    if (std::chrono::steady_clock::now() >= state.deadline)
    {
        DUST_LOG_ERR("calib: only %zu of %d samples within %ds", samples_.size(), params_.sample_count, params_.capture_timeout_s);
        System_RequestStop(1);
        return Thread::TaskResult::Fatal;
    }

    // 每帧都识别一次：结果既画进转发的图，也是快门判合格的第一道
    std::vector<cv::Point2f> centers;
    const bool detected = algorithm::calib::DetectBoard(message.image, params_.board, centers);

    // 识别结果为空：这一帧只是画面，没有待发布的装甲板结果
    topic::vision_to::pub_vision_to_.Publish({DrawCenters(message.image, centers), message.timestamp, {}, 0.0});

    const auto now = std::chrono::steady_clock::now();
    if (now >= state.next_report)
    {
        DUST_LOG_INF("calib: waiting samples=%zu/%d shots=%d detected=%s", samples_.size(), params_.sample_count, state.shots, detected ? "yes" : "no");
        state.next_report = now + std::chrono::seconds(kProgressIntervalS);
        state.shots       = 0;
    }

    if (!TakeShotRequest())
    {
        return Thread::TaskResult::Continue; // 没按快门就只当预览
    }
    ++state.shots;

    if (!detected)
    {
        DUST_LOG_WRN("calib: shot dropped: no circle grid in the frame");
        return Thread::TaskResult::Continue;
    }

    const std::optional<SampleShape> shape = algorithm::calib::MeasureShape(params_.board, centers);
    if (!shape)
    {
        DUST_LOG_WRN("calib: shot dropped: degenerate circle grid");
        return Thread::TaskResult::Continue;
    }
    if (!algorithm::calib::IsDistinctSample(*shape, shapes_, params_.gate))
    {
        DUST_LOG_WRN("calib: shot dropped: pose repeats a kept sample");
        return Thread::TaskResult::Continue;
    }
    if (std::chrono::steady_clock::now() < state.next_accept)
    {
        DUST_LOG_WRN("calib: shot dropped: within sample_interval_ms of the last kept sample");
        return Thread::TaskResult::Continue;
    }

    // 先落图再记账：图写不出去说明目录有问题，这时上报比留一批没图的样本好
    const std::string name = std::to_string(samples_.size() + 1) + ".jpg";
    const std::string path = params_.samples_dir + "/" + name;
    if (!cv::imwrite(path, message.image))
    {
        DUST_LOG_ERR("calib: cannot write %s", path.c_str());
        System_RequestStop(1);
        return Thread::TaskResult::Fatal;
    }

    // 存的是原始帧：圆心坐标要对应未被标记污染的像素
    image_size_ = message.image.size();
    shapes_.push_back(*shape);
    samples_.push_back(std::move(centers));
    state.next_accept = std::chrono::steady_clock::now() + std::chrono::milliseconds(params_.sample_interval_ms);

    DUST_LOG_INF("calib: sample %zu/%d center=(%.0f,%.0f) span=%.0fpx pitch=%.2f yaw=%.2f -> %s", samples_.size(), params_.sample_count, shape->center.x,
                 shape->center.y, shape->span_px, shape->pitch_ratio, shape->yaw_ratio, name.c_str());

    FinishIfComplete();

    return Thread::TaskResult::Continue;
}

/**
 * @brief 读取配置并准备样本目录
 *
 * 相机由相机线程持有，这里只做本线程自己的准备；取帧在 thread_start 之后由 Task 完成。
 *
 * @return true 配置有效且目录可用
 * @throws std::exception 配置读取或字段校验失败
 */
bool thread_init()
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
bool thread_start()
{
    capture_.deadline    = std::chrono::steady_clock::now() + std::chrono::seconds(params_.capture_timeout_s);
    capture_.next_accept = std::chrono::steady_clock::now();
    capture_.next_report = capture_.next_accept;

    thread_.Start(Task, &capture_, "calib");
    return true;
}

/**
 * @brief 请求标定线程停止并关闭结果通道
 *
 * 关闭结果通道用于唤醒正在等待结果的显示侧；标定线程仍由 camera_to 关闭或
 * 当前帧处理完观察停止标志退出。
 */
void thread_stop()
{
    thread_.RequestStop();
    topic::vision_to::pub_vision_to_.Close();
}

/**
 * @brief 等待标定线程退出
 */
void thread_join()
{
    thread_.Join();
}

/**
 * @brief 清空结果通道槽内的图像，支持重复清理
 */
void Release()
{
    topic::vision_to::pub_vision_to_.Clear();
}

REGISTER_INIT(thread_init, AppInit, High, HaltOnFail, Release, "calib_init");
REGISTER_THREAD(thread_start, AppThread, thread_stop, thread_join, "calib_start");

// 采集快门：shell 里 `var set calib.capture 1` 拍下当前帧，采集循环读过就写回 0。
// 用直接写入而不是 NOTIFY：真正的动作在采集循环那一步，shell 线程只负责置位；而且
// NOTIFY 会在回调返回后再把条目写一次，与循环的清零交错时一次按键会被消费两次。
REGISTER_SHELL_VAR("calib.capture", capture_request_, capture_mutex_);

} // namespace thread::calib
