/**
 * @file vision_to.hpp
 * @author qingyu
 * @brief 视觉识别结果通道：同帧图像、装甲板结果与处理耗时
 * @version 0.1
 * @date 2026-10-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "framework/algorithm/vision/core/detections.hpp"
#include "framework/topic/channel.hpp"

#include <chrono>
#include <opencv2/core.hpp>
#include <vector>

namespace topic::vision_to
{

/**
 * @brief 单帧视觉识别结果
 *
 * source_image 与送入模型的相机帧共享像素，只允许消费者读取；需要绘制时由显示侧复制。
 */
struct Message
{
    cv::Mat source_image;                                             // 与识别结果严格对应的原始 BGR 图像
    std::chrono::steady_clock::time_point timestamp;                  // 沿用相机消息的采集时间
    std::vector<algorithm::vision::DetectedArmor> armors;             // 本帧通过后处理的装甲板
    double processing_ms {0.0};                                       // Detect 完整调用耗时，单位 ms
};

using Channel    = topic::Channel<Message>;
using Subscriber = topic::Subscriber<Message>;

/**
 * @brief 视觉结果单槽通道，由 vision 线程发布、display 消费
 */
inline Channel pub_vision_to_;

} // namespace topic::vision_to
