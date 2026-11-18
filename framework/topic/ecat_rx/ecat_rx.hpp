/**
 * @file ecat_rx.hpp
 * @author qingyu
 * @brief EtherCAT 接收话题 —— 完整 256 字节 PDO
 * @version 0.1
 * @date 2026-11-18
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include "framework/topic/channel.hpp"

namespace topic::ecat_rx
{

/**
 * @brief 原始接收 PDO，包含业务包及尾部填充，不附加协议字段
 */
struct Message
{
    uint8_t data[256] {};    // 从站 TxPDO 原始字节
};

extern Channel<Message> channel_;    // 唯一发布者为 EtherCAT 线程

} // namespace topic::ecat_rx

namespace topic::ecat_tx
{

/**
 * @brief ROS 2 下发的完整 PDO，接收时间用于主站命令停更检测
 */
struct Message
{
    uint8_t data[256] {};
    std::chrono::steady_clock::time_point received_at {};
};

extern Channel<Message> channel_;    // 唯一发布者为 ROS 2 线程

} // namespace topic::ecat_tx
