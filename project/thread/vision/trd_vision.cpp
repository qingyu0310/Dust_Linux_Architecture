/**
 * @file trd_vision.cpp
 * @author qingyu
 * @brief 识别线程 — 订阅最新相机帧并执行 0526 TensorRT 检测
 * @version 0.2
 * @date 2026-10-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "framework/algorithm/vision/detect/armor_detector.hpp"
#include "framework/cmd/config/config.hpp"
#include "framework/cmd/shell/log.hpp"
#include "framework/init/Init_entry.hpp"
#include "framework/init/thread.hpp"
#include "framework/topic/camera_to/camera_to.hpp"
#include "framework/topic/vision_to/vision_to.hpp"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#ifdef CONFIG_DUST_VISION_YOLO_REFINER
#include "framework/algorithm/vision/refine/armor_refiner.hpp"
#endif

namespace thread::vision
{

using namespace algorithm::vision;
using namespace topic::camera_to;
using namespace init;

/**
 * @brief 相机内参与畸变，由标定组件产出、从本线程 YAML 读入
 *
 * 标定本身不用这两个量，只有把像素反投影到相机系时才需要，所以它住在识别线程这里。
 */
struct CameraIntrinsics
{
    std::int32_t image_width {0};               // 标定时的图像宽度，单位 pixel
    std::int32_t image_height {0};              // 标定时的图像高度，单位 pixel
    double       fx {0.0};                      // x 方向焦距，单位 pixel
    double       fy {0.0};                      // y 方向焦距，单位 pixel
    double       cx {0.0};                      // 主点横坐标，单位 pixel
    double       cy {0.0};                      // 主点纵坐标，单位 pixel
    double       k1 {0.0};                      // 径向畸变一阶系数
    double       k2 {0.0};                      // 径向畸变二阶系数
    double       p1 {0.0};                      // 切向畸变一阶系数
    double       p2 {0.0};                      // 切向畸变二阶系数
    double       k3 {0.0};                      // 径向畸变三阶系数，标定时固定为 0
    double       reprojection_error_px {0.0};   // 标定时的重投影误差，单位 pixel
};

/**
 * @brief 一秒统计窗口
 *
 * 保存该窗口内的帧数、各级候选统计、平均耗时累加值和最高置信度目标；
 * 只由识别线程访问，不需要额外加锁。
 */
struct ReportWindow
{
    uint32_t frames              {0};               // 完成识别的帧数
    uint64_t score_passed        {0};               // 通过 objectness 阈值的原始候选数
    uint64_t nms_kept            {0};               // NMS 后保留的候选数
    uint64_t accepted            {0};               // 最终接受的装甲板数
    uint64_t rejected_confidence {0};               // 因置信度不足被拒绝的候选数
    uint64_t rejected_color      {0};               // 因颜色输出无效被拒绝的候选数
    uint64_t rejected_label_type {0};               // 因标签与尺寸不匹配被拒绝的候选数
    uint64_t rejected_geometry   {0};               // 因四点几何无效被拒绝的候选数
    float    max_objectness      {0.0f};            // 窗口内最高原始 objectness
    double   preprocess_ms       {0.0};             // 预处理耗时累加值，单位 ms
    double   inference_ms        {0.0};             // TensorRT 推理耗时累加值，单位 ms
    double   postprocess_ms      {0.0};             // 后处理耗时累加值，单位 ms
    double   frame_age_ms        {0.0};             // 相机时间戳到识别完成的帧龄累加值，单位 ms
    std::optional<DetectedArmor> best;              // 窗口内置信度最高的有效装甲板

    /**
     * @brief 累加一帧检测结果
     * @param result 当前帧的检测结果与分段统计
     * @param age_ms 当前帧从相机时间戳到识别完成的帧龄，单位 ms
     */
    void Add(const ArmorDetectionResult& result, double age_ms)
    {
        ++frames;
        score_passed        += result.stats.score_passed;
        nms_kept            += result.stats.nms_kept;
        accepted            += result.stats.accepted;

        rejected_confidence += result.stats.rejected_confidence;
        rejected_color      += result.stats.rejected_color;
        rejected_label_type += result.stats.rejected_label_type;
        rejected_geometry   += result.stats.rejected_geometry;

        max_objectness       = std::max(max_objectness, result.stats.max_objectness);
        preprocess_ms       += result.stats.preprocess_ms;
        inference_ms        += result.stats.inference_ms;
        postprocess_ms      += result.stats.postprocess_ms;
        frame_age_ms        += age_ms;

        for (const DetectedArmor& armor : result.armors)
        {
            if (!best || armor.GetConfidence() > best->GetConfidence())
            {
                best = armor;
            }
        }
    }

    /**
     * @brief 清空当前窗口，开始下一秒统计
     */
    void Clear()
    {
        *this = ReportWindow {};
    }
};

static CameraIntrinsics camera_intrinsics_ {};                          // 本线程读入的内参，后续 PnP 使用

static std::unique_ptr<ArmorDetector> detector_;                        // 本线程独占的 TensorRT 检测器

#ifdef CONFIG_DUST_VISION_YOLO_REFINER
static std::unique_ptr<ArmorRefiner> refiner_;                          // 可选的局部灯条角点精修器
#endif

static Subscriber sub_camera_to {pub_camera_to_};             // vision 独占的相机帧订阅游标

static Thread thread_ {};                                               // 识别工作线程

/**
 * @brief 把装甲板颜色转换为日志字符串
 * @param color 工程颜色枚举
 * @return 固定字符串 red、blue 或 unknown
 */
static const char* ColorName(TeamColor color)
{
    switch (color)
    {
        case TeamColor::Red:
            return "red";
        case TeamColor::Blue:
            return "blue";
        default:
            return "unknown";
    }
}

/**
 * @brief 把装甲板标签转换为日志字符串
 * @param label 工程标签枚举
 * @return 固定类别字符串
 */
static const char* LabelName(ArmorLabel label)
{
    switch (label)
    {
        case ArmorLabel::One:
            return "1";
        case ArmorLabel::Two:
            return "2";
        case ArmorLabel::Three:
            return "3";
        case ArmorLabel::Four:
            return "4";
        case ArmorLabel::Five:
            return "5";
        case ArmorLabel::Sentry:
            return "sentry";
        case ArmorLabel::Outpost:
            return "outpost";
        case ArmorLabel::Base:
            return "base";
        default:
            return "not_armor";
    }
}

/**
 * @brief 一次读完视觉线程 YAML：检测器参数与相机内参
 *
 * 内参那批键由标定组件打印、人粘进本 YAML，是标定与视觉之间唯一的接口；
 * 缺键或越界都会在 Load 里直接报错，不带默认值悄悄跑。
 */
static void LoadVisionConfig(ArmorDetector::Config& detector_config)
{
    cmd::config::Config yaml;
    yaml.Load("thread/vision/trd_vision.yaml",
              {"engine_path",          "cuda_device",         "score_threshold", "confidence_threshold", "nms_threshold"},
              {"brightness_threshold", "min_light_length_px", "min_light_ratio", "max_light_ratio",      "max_light_tilt_deg", "max_endpoint_distance_px", "image_width", "image_height", "camera_fx", "camera_fy", "camera_cx", "camera_cy", "distort_k1", "distort_k2", "distort_p1", "distort_p2", "distort_k3", "reprojection_error_px"});

    detector_config.engine_path = yaml.GetPath("engine_path");
    if (detector_config.engine_path.empty())
    {
        throw std::runtime_error("vision config: engine_path must not be empty");
    }
    detector_config.cuda_device          = yaml.GetInt("cuda_device", 0, std::numeric_limits<int>::max());
    detector_config.nms_threshold        = static_cast<float>(yaml.GetDouble("nms_threshold", 0.0, 1.0));
    detector_config.score_threshold      = static_cast<float>(yaml.GetDouble("score_threshold", 0.0, 1.0));
    detector_config.confidence_threshold = static_cast<float>(yaml.GetDouble("confidence_threshold", 0.0, 1.0));

    camera_intrinsics_.image_width           = yaml.GetInt("image_width",   1,          100000);
    camera_intrinsics_.image_height          = yaml.GetInt("image_height",  1,          100000);

    camera_intrinsics_.fx                    = yaml.GetDouble("camera_fx",  1.0,        1000000.0);
    camera_intrinsics_.fy                    = yaml.GetDouble("camera_fy",  1.0,        1000000.0);
    camera_intrinsics_.cx                    = yaml.GetDouble("camera_cx",  -1000000.0, 1000000.0);
    camera_intrinsics_.cy                    = yaml.GetDouble("camera_cy",  -1000000.0, 1000000.0);

    camera_intrinsics_.k1                    = yaml.GetDouble("distort_k1", -1000.0,    1000.0);
    camera_intrinsics_.k2                    = yaml.GetDouble("distort_k2", -1000.0,    1000.0);
    camera_intrinsics_.p1                    = yaml.GetDouble("distort_p1", -1000.0,    1000.0);
    camera_intrinsics_.p2                    = yaml.GetDouble("distort_p2", -1000.0,    1000.0);
    camera_intrinsics_.k3                    = yaml.GetDouble("distort_k3", -1000.0,    1000.0);

    camera_intrinsics_.reprojection_error_px = yaml.GetDouble("reprojection_error_px", 0.0, 1000000.0);

    DUST_LOG_INF("vision: intrinsics %dx%d fx=%.3f fy=%.3f cx=%.3f cy=%.3f", camera_intrinsics_.image_width, camera_intrinsics_.image_height, camera_intrinsics_.fx, camera_intrinsics_.fy, camera_intrinsics_.cx, camera_intrinsics_.cy);
}

#ifdef CONFIG_DUST_VISION_YOLO_REFINER
/**
 * @brief 从视觉线程 YAML 读取精修参数
 *
 * 精修器编进程序就生效，参数必须齐备。
 */
static void LoadRefinerConfig(ArmorRefiner::Config& refiner_config)
{
    cmd::config::Config yaml;
    yaml.Load("thread/vision/trd_vision.yaml",
              {"engine_path", "cuda_device", "score_threshold", "confidence_threshold", "nms_threshold"},
              {"brightness_threshold", "min_light_length_px", "min_light_ratio", "max_light_ratio", "max_light_tilt_deg", "max_endpoint_distance_px"});

    const char* const refiner_keys[] = {
        "brightness_threshold",
        "min_light_length_px",
        "min_light_ratio",
        "max_light_ratio",
        "max_light_tilt_deg",
        "max_endpoint_distance_px"
    };
    for (const char* key : refiner_keys)
    {
        if (!yaml.Has(key))
        {
            throw std::runtime_error(std::string("vision config: missing refiner field: ") + key);
        }
    }

    refiner_config.brightness_threshold = yaml.GetInt("brightness_threshold", 0, 255);
    refiner_config.min_light_ratio      = static_cast<float>(yaml.GetDouble("min_light_ratio",     0.0, 1000000.0));
    refiner_config.max_light_ratio      = static_cast<float>(yaml.GetDouble("max_light_ratio",     0.0, 1000000.0));
    refiner_config.min_light_length_px  = static_cast<float>(yaml.GetDouble("min_light_length_px", 0.0, 1000000.0));

    const double max_tilt_deg = yaml.GetDouble("max_light_tilt_deg", 0.0, 90.0);
    refiner_config.max_light_tilt_rad       = static_cast<float>(max_tilt_deg * 3.14159265358979323846 / 180.0);
    refiner_config.max_endpoint_distance_px = static_cast<float>(yaml.GetDouble("max_endpoint_distance_px", 0.0, 1000000.0));

    refiner_config.Validate();
}
#endif

/**
 * @brief 输出一条识别窗口汇总日志
 * @param report 已完成的一秒统计窗口
 * @param elapsed 窗口实际持续时间，单位 s
 */
static void LogReport(const ReportWindow& report, double elapsed)
{
    const double denominator = report.frames == 0 ? 1.0 : static_cast<double>(report.frames);
    if (!report.best)
    {
        DUST_LOG_DBG("vision", "fps=%.1f taken=%llu dropped=%llu raw=25200 score=%llu nms=%llu accepted=%llu reject(conf/color/type/geometry)=%llu/%llu/%llu/%llu max_obj=%.3f ms(pre/infer/post/age)=%.2f/%.2f/%.2f/%.2f target=none", report.frames / elapsed, static_cast<unsigned long long>(sub_camera_to.Taken()), static_cast<unsigned long long>(sub_camera_to.Dropped()), static_cast<unsigned long long>(report.score_passed), static_cast<unsigned long long>(report.nms_kept), static_cast<unsigned long long>(report.accepted), static_cast<unsigned long long>(report.rejected_confidence), static_cast<unsigned long long>(report.rejected_color), static_cast<unsigned long long>(report.rejected_label_type), static_cast<unsigned long long>(report.rejected_geometry), report.max_objectness, report.preprocess_ms / denominator, report.inference_ms / denominator, report.postprocess_ms / denominator, report.frame_age_ms / denominator);
        return;
    }

    const DetectedArmor& armor = *report.best;
    const auto& corners = armor.GetCorners();
    DUST_LOG_DBG("vision", "fps=%.1f taken=%llu dropped=%llu raw=25200 score=%llu nms=%llu accepted=%llu reject(conf/color/type/geometry)=%llu/%llu/%llu/%llu max_obj=%.3f ms(pre/infer/post/age)=%.2f/%.2f/%.2f/%.2f target=%s/%s conf=%.3f corners=(%.1f,%.1f)(%.1f,%.1f)(%.1f,%.1f)(%.1f,%.1f)", report.frames / elapsed, static_cast<unsigned long long>(sub_camera_to.Taken()), static_cast<unsigned long long>(sub_camera_to.Dropped()), static_cast<unsigned long long>(report.score_passed), static_cast<unsigned long long>(report.nms_kept), static_cast<unsigned long long>(report.accepted), static_cast<unsigned long long>(report.rejected_confidence), static_cast<unsigned long long>(report.rejected_color), static_cast<unsigned long long>(report.rejected_label_type), static_cast<unsigned long long>(report.rejected_geometry), report.max_objectness, report.preprocess_ms / denominator, report.inference_ms / denominator, report.postprocess_ms / denominator, report.frame_age_ms / denominator, ColorName(armor.GetColor()), LabelName(armor.GetLabel()), armor.GetConfidence(), corners[0].x, corners[0].y, corners[1].x, corners[1].y, corners[2].x, corners[2].y, corners[3].x, corners[3].y);
}

/**
 * @brief 取最新帧同步推理；慢于相机时沿用单槽通道的丢旧帧语义
 */
static void Task(void*, void*, void*)
{
    if (!detector_) {
        throw std::runtime_error("vision detector is not initialized");
    }

    ReportWindow report;
    auto last_report = std::chrono::steady_clock::now();

    while (!thread_.StopRequested())
    {
        // 阻塞等待相机最新帧；相机通道关闭时退出识别线程
        Message message;
        if (!sub_camera_to.Wait(message)) {
            break;
        }

        // 对当前相机帧同步执行预处理、TensorRT 推理和后处理
        const auto processing_begin = std::chrono::steady_clock::now();
        ArmorDetectionResult result = detector_->Detect(message.image);

        #ifdef CONFIG_DUST_VISION_YOLO_REFINER
        {
            if (refiner_)
            {
                for (DetectedArmor& armor : result.armors)
                {
                    const ArmorRefinementResult refinement = refiner_->Refine(message.image, armor.GetCorners());
                    if (!refinement.Applied())
                    {
                        continue;
                    }

                    const std::optional<DetectedArmor> refined = armor.WithCorners(refinement.final_corners, message.image.size());
                    if (refined)
                    {
                        armor = *refined;
                    }
                }
            }
        }
        #endif

        const auto   now = std::chrono::steady_clock::now();
        const double processing_ms         = std::chrono::duration<double, std::milli>(now - processing_begin).count();
        const double age_ms                = std::chrono::duration<double, std::milli>(now - message.timestamp).count();
        report.Add(result, age_ms);

        // 停止期间不再发布刚完成的结果，避免关闭后的结果通道被重新写入
        if (thread_.StopRequested()) {
            break;
        }

        // 原图只做浅拷贝，识别结果和图像始终属于同一帧；显示侧需要绘制时再 clone
        topic::vision_to::pub_vision_to_.Publish({message.image, message.timestamp, std::move(result.armors), processing_ms});

        // 日志每秒汇总一次，避免逐帧输出影响实时识别
        const double elapsed = std::chrono::duration<double>(now - last_report).count();
        if (elapsed >= 1.0)
        {
            LogReport(report, elapsed);
            report.Clear();
            last_report = now;
        }
    }
}

/**
 * @brief 一次加载 engine、校验固定张量并分配 CUDA/TensorRT 资源
 * @return true 检测器创建成功
 * @throws std::exception engine 读取、TensorRT 校验或 CUDA 资源分配失败
 */
bool thread_init()
{
    ArmorDetector::Config detector_config;
    LoadVisionConfig(detector_config);
    detector_ = std::make_unique<ArmorDetector>(detector_config);

#ifdef CONFIG_DUST_VISION_YOLO_REFINER
    ArmorRefiner::Config refiner_config;
    LoadRefinerConfig(refiner_config);

    refiner_ = std::make_unique<ArmorRefiner>(refiner_config);
    const char* const refiner_state = "on";
#else
    const char* const refiner_state = "off";
#endif

    DUST_LOG_INF("[vision] TensorRT device=%d engine=%s score=%.3f confidence=%.3f nms=%.3f refiner=%s input=%s/%s [1,3,%d,%d] output=%s FP32 [1,%d,%d]", detector_config.cuda_device, detector_config.engine_path.c_str(), detector_config.score_threshold, detector_config.confidence_threshold, detector_config.nms_threshold, refiner_state, detector_->GetInputName().c_str(), detector_->GetInputTypeName().c_str(), detector_->GetInputHeight(), detector_->GetInputWidth(), detector_->GetOutputName().c_str(), detector_->GetOutputRows(), detector_->GetOutputColumns());
    return true;
}

/**
 * @brief 启动识别工作线程
 * @return true 线程创建成功
 */
bool thread_start()
{
    thread_.Start(Task, nullptr, "vision");
    return true;
}

/**
 * @brief 请求识别线程停止并关闭结果通道
 *
 * 关闭结果通道用于唤醒正在等待识别结果的显示侧；识别线程仍由 camera_to 关闭或
 * 当前 Detect 返回后观察停止标志退出。
 */
void thread_stop()
{
    thread_.RequestStop();
    topic::vision_to::pub_vision_to_.Close();
}

/**
 * @brief 等待识别线程退出，之后才允许释放 TensorRT/CUDA 资源
 */
void thread_join()
{
    thread_.Join();
}

/**
 * @brief 在线程退出并 join 后释放 TensorRT/CUDA 资源
 */
void Release()
{
#ifdef CONFIG_DUST_VISION_YOLO_REFINER
{
    refiner_.reset();
}
#endif
    detector_.reset();
    topic::vision_to::pub_vision_to_.Clear();
}

REGISTER_INIT(thread_init, AppInit, High, HaltOnFail, Release, "vision_init");
REGISTER_THREAD(thread_start, AppThread, thread_stop, thread_join, "vision_start");

} // namespace thread::vision
