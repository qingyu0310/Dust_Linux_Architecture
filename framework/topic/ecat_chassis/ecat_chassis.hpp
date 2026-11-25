/**
 * @file ecat_chassis.hpp
 * @author qingyu
 * @brief 底盘数据报文 —— 底盘控制线程与 EtherCAT 线程之间的上下行数据契约
 * @version 0.3
 * @date 2026-11-25
 *
 * @copyright Copyright (c) 2026
 *
 * # 数据流
 *
 * ```text
 * 遥控（从站 TxPDO RemoteFeedback）
 *        │  EtherCAT 线程发布 chassis_remote_channel_
 *        ▼
 * 底盘控制线程：逆运动学 + 四轮速度环
 *        │  发布 chassis_command_channel_
 *        ▼
 * EtherCAT 线程：写 RxPDO 电机命令区
 *        │
 *        ▼
 * 从站电机反馈（TxPDO 电机反馈区）
 *        │  EtherCAT 线程发布 chassis_feedback_channel_
 *        ▼
 * 底盘控制线程
 * ```
 *
 * # 报文布局
 *
 * - 下行：C620 电流控制帧（CAN 0x200，8 B），写入 RxPDO 电机命令区前 8 B（偏移 48），其余全零。
 * - 上行：4 槽 × 12 B 电机反馈（48 B），映射 TxPDO 电机反馈区（偏移 108）。
 *   槽位 0~3 = 四个全向轮，编号与逆运动学 `target[0..3]` 一致。
 */

#pragma once

#include <cstdint>
#include "framework/topic/channel.hpp"

namespace topic::ecat_chassis
{

constexpr uint8_t kMotorSlots = 4;     // 电机槽位数（四个 C620）

#pragma pack(push, 1)

/**
 * @brief 上行报文单槽：一个轮的反馈（12 B）
 */
struct ChassisFeedbackSlot
{
    uint16_t id;              // 槽位号 0..3
    uint16_t status;          // bit0=在线；bit1=故障
    float    omega;           // 轮角速度 rad/s（从站已换算到轮侧）
    float    current;         // 实际电流 A
};

#pragma pack(pop)

static_assert(sizeof(ChassisFeedbackSlot) == 12, "ChassisFeedbackSlot must be 12 bytes");

/**
 * @brief 底盘下行报文：C620 电流控制帧（CAN 0x200，8 B）
 *
 * 从站收到后原样转发到 CAN：`data` 0~7 字节 = 4 × int16 电流原始值（小端），
 * 依次对应 CAN ID 0x201~0x204 的目标电流；换算 raw = current(A) × 819.2（±16384 ↔ ±20 A）。
 *
 * 底盘控制线程每个控制周期发布一次；EtherCAT 线程取最新一帧写入 RxPDO 电机命令区前 8 B，
 * 停更 100 ms 整区归零。
 */
struct ChassisCommandMessage
{
    uint32_t can_id = 0x200;   // CAN 标识符（标准帧）
    uint8_t  dlc    = 8;       // 数据长度
    uint8_t  data[8] {};       // 4 × int16 电流原始值，小端
};

/**
 * @brief 底盘上行数据报文：4 槽 × 12 B（48 B），直接映射 TxPDO 电机反馈区
 *
 * EtherCAT 线程每收到一帧校验通过的 TxPDO 发布一次；未实现电机反馈的从站整区为零。
 */
struct ChassisFeedbackMessage
{
    ChassisFeedbackSlot slot[kMotorSlots] {};
};

/**
 * @brief 遥控状态报文：TxPDO RemoteFeedback 的字段副本
 *
 * 底盘线程据此决定使能（controls.bit0）与小陀螺（controls.bit1），并用 axis[0] / axis[1] 作平移输入。
 */
struct ChassisRemoteMessage
{
    float    axis[4] {};          		// chassisx / chassisy / yaw / pitch，归一化 [-1, 1]
    uint16_t controls       = 0;        // bit0=整车总开关；bit1=Spin；其余见协议文档 §2.2
    uint32_t source_seq     = 0;     	// 遥控消息帧序号，未更新即未刷新
    uint32_t sample_tick_us = 0;  		// 该帧采样时刻，MCU 本地 µs（用于判断失联）
    uint8_t  status 		= 0;        // bit0=收到过有效遥控帧
};

static_assert(sizeof(ChassisCommandMessage::data)  == 8,   "C620 control frame data must be 8 bytes");
static_assert(sizeof(ChassisFeedbackMessage) == 48, "chassis feedback area must be 48 bytes");

inline Channel<ChassisCommandMessage>  chassis_command_channel_;   // 唯一发布者：底盘控制线程
inline Channel<ChassisFeedbackMessage> chassis_feedback_channel_;  // 唯一发布者：EtherCAT 线程
inline Channel<ChassisRemoteMessage>   chassis_remote_channel_;    // 唯一发布者：EtherCAT 线程

} // namespace topic::ecat_chassis
