/**
 * @file camera_to.hpp
 * @author qingyu
 * @brief 相机帧通道：BGR 图像与采集时间
 * @version 0.1
 * @date 2026-10-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <chrono>
#include <opencv2/core.hpp>
#include "framework/topic/channel.hpp"

namespace topic::camera_to
{

/**
 * @brief 相机帧：BGR 图像与 SDK 帧时间戳
 */
struct Message
{
    cv::Mat                               image;      // 浅拷贝：像素由引用计数共享，订阅者不许 clone
    std::chrono::steady_clock::time_point timestamp;  // 相机 SDK 给出的采集时间
};

using Channel    = topic::Channel<Message>;         // 对应子树的 pub_camera_to 通道
using Subscriber = topic::Subscriber<Message>;       // 每个消费者各自声明，不共用

/**
 * @brief 相机帧通道，由相机线程发布
 *
 * 对应子树的 ZBUS_CHAN_DECLARE(pub_camera_to)。子树在通道旁还定义一个共享观察者
 * （ZBUS_SUBSCRIBER_DEFINE(sub_xxx_to)），这里不那样做：显示与识别要各自独立地取帧，
 * 共用观察者会互相抢消息，所以订阅者由消费者在自己的线程里声明。
 */
inline Channel pub_camera_to_;

} // namespace topic::camera_to
