/**
 * @file trd_vision.hpp
 * @author qingyu
 * @brief 识别线程 — 相机内参、统计窗口、线程状态与生命周期入口
 * @version 0.1
 * @date 2026-10-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "framework/algorithm/vision/detect/armor_detector.hpp"
#include "framework/algorithm/vision/pose/armor_pose.hpp"
#include "framework/algorithm/vision/track/multi_target.hpp"
#include "framework/init/thread/thread.hpp"
#include "framework/topic/camera_to/camera_to.hpp"
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace thread::vision
{

using namespace algorithm::vision;
using namespace topic::camera_to;

/**
 * @brief 相机内参与畸变，由标定组件产出、从本线程 YAML 读入
 *
 * 标定本身不用这两个量，只有把像素反投影到相机系时才需要，所以它住在识别线程这里。
 */
struct CameraIntrinsics
{
    int32_t image_width  {0};                       // 标定时的图像宽度，单位 pixel
    int32_t image_height {0};                       // 标定时的图像高度，单位 pixel
    double  fx {0.0};                               // x 方向焦距，单位 pixel
    double  fy {0.0};                               // y 方向焦距，单位 pixel
    double  cx {0.0};                               // 主点横坐标，单位 pixel
    double  cy {0.0};                               // 主点纵坐标，单位 pixel
    double  k1 {0.0};                               // 径向畸变一阶系数
    double  k2 {0.0};                               // 径向畸变二阶系数
    double  p1 {0.0};                               // 切向畸变一阶系数
    double  p2 {0.0};                               // 切向畸变二阶系数
    double  k3 {0.0};                               // 径向畸变三阶系数，标定时固定为 0
    double  reprojection_error_px {0.0};            // 标定时的重投影误差，单位 pixel
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
    std::optional<algorithm::vision::DetectedArmor> best;   // 窗口内置信度最高的有效装甲板

    uint64_t pnp_ok              {0};               // 解出位姿的装甲板数
    uint64_t pnp_invalid         {0};               // 尺寸不明或角点退化、未解算的装甲板数
    double   pnp_reprojection_px {0.0};             // 解出位姿的反投误差累加值，单位 pixel
    double   pnp_distance_m      {0.0};             // 解出位姿的板心距离累加值，单位 m

    /**
     * @brief 累加一帧的位姿解算结果
     */
    void AddPose(uint64_t solved, uint64_t invalid, double reprojection_px, double distance_m);

    /**
     * @brief 累加一帧检测结果
     * @param result 当前帧的检测结果与分段统计
     * @param age_ms 当前帧从相机时间戳到识别完成的帧龄，单位 ms
     */
    void Add(const algorithm::vision::ArmorDetectionResult& result, double age_ms);

    /**
     * @brief 清空当前窗口，开始下一秒统计
     */
    void Clear();
};

/**
 * @brief 识别拍与拍之间要保留的状态
 */
struct VisionState
{
    ReportWindow report {};                                     // 当前一秒统计窗口
    std::chrono::steady_clock::time_point last_report {};       // 上次汇总时刻
};

/**
 * @brief 从 YAML 读入的相机与跟踪参数
 */
struct VisionConfig
{
    CameraIntrinsics  camera {};        // 本线程读入的内参，后续 PnP 使用
    GimbalExtrinsics  extrinsics {};    // 相机相对云台的外参，未标定时为单位阵
    MultiTargetConfig tracker {};       // 多目标跟踪参数
};

/**
 * @brief 本线程独占的算法对象与订阅游标
 */
struct VisionRuntime
{
    std::unique_ptr<ArmorDetector>      detector;   // TensorRT 检测器
#ifdef CONFIG_DUST_VISION_YOLO_REFINER
    std::unique_ptr<ArmorRefiner>       refiner;    // 可选的局部灯条角点精修器
#endif
    std::unique_ptr<MultiTargetTracker> multi;      // 多目标跟踪，槽位由 kTargetLabels 决定
    Subscriber                          camera_to {pub_camera_to_};   // 独占的相机帧订阅游标
    std::optional<std::chrono::steady_clock::time_point> last_frame_time;       // 上一帧时间戳，用于跟踪的 dt
};

inline VisionConfig  config_ {};            // 配置
inline VisionRuntime runtime_ {};           // 算法对象
inline VisionState   vision_state_ {};      // 统计窗口
inline init::Thread  thread_ {};            // 识别工作线程

/**
 * @brief 一次读完视觉线程 YAML：检测器参数与相机内参
 *
 * @param detector_config 输出的检测器配置
 * @throws std::exception 文件缺失、字段不符或数值越界
 */
void LoadVisionConfig(algorithm::vision::ArmorDetector::Config& detector_config);

#ifdef CONFIG_DUST_VISION_YOLO_REFINER
/**
 * @brief 从视觉线程 YAML 读取精修参数
 *
 * @param refiner_config 输出的精修器配置
 * @throws std::exception 文件缺失、字段不符或数值越界
 */
void LoadRefinerConfig(algorithm::vision::ArmorRefiner::Config& refiner_config);
#endif

/**
 * @brief 把线程读入的内参折算成算法层的针孔模型
 */
algorithm::vision::CameraModel ToCameraModel(const CameraIntrinsics& intrinsics);

/**
 * @brief 推进多目标跟踪并把结果描在副本上
 *
 * @param image 相机帧，整幅 BGR
 * @param armors 本帧检测结果
 * @param report 本帧位姿统计，解出与未解出的块数都累加进去
 * @return 叠加好轮廓的图像
 */
cv::Mat DrawPoses(const cv::Mat& image, const std::vector<algorithm::vision::DetectedArmor>& armors, ReportWindow& report);

/**
 * @brief 输出一条识别窗口汇总日志
 * @param report 已完成的一秒统计窗口
 * @param elapsed 窗口实际持续时间，单位 s
 */
void LogReport(const ReportWindow& report, double elapsed);

} // namespace thread::vision
