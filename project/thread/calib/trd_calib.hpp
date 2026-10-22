/**
 * @file trd_calib.hpp
 * @author qingyu
 * @brief 标定线程 — 运行参数、采集状态、样本集与快门请求
 * @version 0.2
 * @date 2026-10-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "framework/algorithm/calib/camera_calib.hpp"
#include "framework/init/thread/thread.hpp"
#include "framework/topic/camera_to/camera_to.hpp"
#include <chrono>
#include <cstdint>
#include <mutex>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace thread::calib
{

/**
 * @brief 标定组件的运行参数，只由本组件的初始化项读取
 */
struct Params
{
    algorithm::calib::Board       board;                            // 标定板规格
    algorithm::calib::CaptureGate gate;                             // 快门合格性门限
    std::int32_t sample_count {0};                                  // 需要保留的合格样本数
    std::int32_t sample_interval_ms {0};                            // 两次保留之间的最短间隔，单位 ms
    std::int32_t capture_timeout_s {0};                             // 采集总时限，单位 s
    double       max_reprojection_error_px {0.0};                   // 重投影误差上限，单位 pixel
    std::string  samples_dir;                                       // 合格样本图片目录，已按配置根解析
};

/**
 * @brief 标定拍与拍之间要保留的状态
 */
struct CaptureState
{
    std::chrono::steady_clock::time_point deadline {};       // 采集总时限的绝对时刻
    std::chrono::steady_clock::time_point next_accept {};    // 下次允许保留样本的时刻
    std::chrono::steady_clock::time_point next_report {};    // 下次进度汇报的时刻
    std::int32_t shots = 0;                                  // 本汇报周期内的快门次数
};

/**
 * @brief 已保留的样本与求解结果
 */
struct SampleSet
{
    std::vector<std::vector<cv::Point2f>> centers;   // 每帧识别到的圆心
    std::vector<algorithm::calib::SampleShape> shapes;   // 与 centers 一一对应的签名
    cv::Size image_size {};                          // 合格样本那一帧的尺寸，求解时当作标定尺寸
    bool     solved = false;                         // 本轮是否已经求解并打印过
};

/**
 * @brief shell 的快门请求，一次按键只消费一次
 */
struct Shutter
{
    bool       request = false;   // 非 0 表示按了一次快门
    std::mutex mutex;             // 保护请求，同时作为该 var 条目的锁
};

inline Params       params_ {};
inline CaptureState capture_ {};
inline SampleSet    samples_ {};
inline Shutter      shutter_ {};

inline topic::camera_to::Subscriber sub_camera_to {topic::camera_to::pub_camera_to_};   // 本线程独占的相机帧游标

inline init::Thread                 thread_ {};                                                  // 标定工作线程

// 等待情况每隔这么久打印一次，单位 s；采集期间逐帧打印会刷屏
inline constexpr std::int32_t kProgressIntervalS = 5;

/**
 * @brief 读取本组件 YAML：字段全部必需
 *
 * @throws std::exception 文件缺失、字段不符或数值越界
 */
void LoadParams();

/**
 * @brief 在每个认出来的圆心上套一个圈，画在副本上
 * @param image 相机帧，整幅 BGR
 * @param centers 本帧识别到的圆心，识别失败时为空
 * @return 叠加好圆心的整幅图像
 */
cv::Mat DrawCenters(const cv::Mat& image, const std::vector<cv::Point2f>& centers);

/**
 * @brief 保留满样本后求解、验收并打印一次
 */
void FinishIfComplete();

} // namespace thread::calib
