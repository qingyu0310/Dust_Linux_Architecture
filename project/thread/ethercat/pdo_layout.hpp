/**
 * @file pdo_layout.hpp
 * @author qingyu
 * @brief 上位机 PDO 协议 v1 布局 —— RxPDO / TxPDO packed 结构（无填充）
 * @version 0.2
 * @date 2026-11-25
 *
 * @copyright Copyright (c) 2026
 *
 * 全部 1 字节对齐、无填充、无预留字段：结构体只声明实际要用的字段，
 * 线上帧固定 256 B，主站结构之外的部分由 SOEM 的 IOmap 保持全零。
 * 字节序按 x86/riscv 的小端直接映射，浮点为 IEEE-754 binary32。
 */

#pragma once

#include <cstdint>
#include <limits>
#include "framework/topic/ecat_chassis/ecat_chassis.hpp"

namespace thread::ethercat
{

#pragma pack(push, 1)

/**
 * @brief TxPDO 反馈头（偏移 0，30 B）
 */
struct FeedbackHeader
{
    uint32_t magic;                  // 固定 0x31424450（"PDB1"）
    uint16_t proto_major;            // 1
    uint16_t proto_minor;            // 0
    uint32_t mcu_boot_id;            // 每次上电重新生成，主站据此判断从站重启
    uint32_t feedback_seq;           // 每 1 ms 自增，溢出回绕
    uint32_t mcu_tick_us;            // 本帧组帧时刻，MCU 本地 µs
    uint32_t applied_command_seq;    // 已执行的 command_seq（无执行器时恒 0）
    uint16_t accepted_host_session;  // 被接纳的主机会话号；0=无有效会话
    uint16_t status_flags;           // bit0=已接纳会话
    uint16_t fault_code;             // 故障码；0=正常
};

/**
 * @brief 遥控反馈（偏移 30，28 B）
 */
struct RemoteFeedback
{
    uint32_t source_seq;             // 遥控消息帧序号，每解析一帧自增
    uint32_t sample_tick_us;         // 该帧采样时刻，MCU 本地 µs
    float    axis[4];                // chassisx / chassisy / yaw / pitch，归一化 [-1, 1]
    uint16_t controls;               // 开关位：bit0 总开关，bit1 Spin，bit2~bit10 其余开关
    uint8_t  protocol;               // 0=未报告
    uint8_t  status;                 // bit0=已收到过有效遥控帧
};

/**
 * @brief IMU 反馈（偏移 58，50 B）
 */
struct ImuFeedback
{
    uint32_t source_seq;             // 姿态消息帧序号，每解算一帧自增
    uint32_t sample_tick_us;         // 该帧采样时刻，MCU 本地 µs
    float    quaternion[4];          // w, x, y, z
    float    gyro[3];                // rad/s
    float    accel[3];               // m/s²
    uint16_t flags;                  // bit0=已产生过姿态
};

/**
 * @brief TxPDO：从站 → 主站，实际映射 156 B（线上帧 256 B，其余保持零）
 */
struct PdoInputs
{
    FeedbackHeader header;           // 偏移 0
    RemoteFeedback remote;           // 偏移 30
    ImuFeedback    imu;              // 偏移 58
    topic::ecat_chassis::ChassisFeedbackSlot motor_feedback[topic::ecat_chassis::kMotorSlots];  // 偏移 108，4 槽 × 12 B
};

/**
 * @brief RxPDO 命令头（偏移 0，36 B）
 */
struct CommandHeader
{
    uint32_t magic;                  // 必须 0x31434D43（"CMC1"）
    uint16_t proto_major;            // 1
    uint16_t proto_minor;            // 0
    uint16_t byte_length;            // 线上帧长，必须 256
    uint16_t slot_map_version;       // 必须 2
    uint16_t host_session;           // 主机会话号，会话生命周期内不变
    uint32_t command_seq;            // 只有产生新决策时才自增
    uint32_t mcu_boot_id_echo;       // 回显最近一次 TxPDO 的 mcu_boot_id
    uint32_t feedback_seq_echo;      // 本命令依据的 feedback_seq
    uint32_t lease_us;               // 租约上限 µs，从站侧再夹一层
    uint32_t compute_time_us;        // 主站组帧耗时，诊断用
    uint16_t flags;                  // bit0 请求 arm；bit1 请求 disarm
};

/**
 * @brief RxPDO：主站 → 从站，实际映射 44 B（线上帧 256 B，其余保持零）
 */
struct PdoOutputs
{
    CommandHeader header;            // 偏移 0
    uint8_t       motor_command[8];  // 偏移 36，C620 电流帧（CAN 0x200 数据段）
};

#pragma pack(pop)

static_assert(sizeof(FeedbackHeader) == 30, "FeedbackHeader must be 30 bytes");
static_assert(sizeof(RemoteFeedback) == 28, "RemoteFeedback must be 28 bytes");
static_assert(sizeof(ImuFeedback)    == 50, "ImuFeedback must be 50 bytes");
static_assert(sizeof(PdoInputs)     == 156, "TxPDO mapped part must be 156 bytes");
static_assert(sizeof(CommandHeader) == 36, "CommandHeader must be 36 bytes");
static_assert(sizeof(PdoOutputs)    == 44, "RxPDO mapped part must be 44 bytes");
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559, "PDO requires IEEE 754 binary32");

constexpr uint32_t kFeedbackMagic  = 0x31424450;  // "PDB1"，小端读为 50 44 42 31
constexpr uint32_t kCommandMagic   = 0x31434D43;  // "CMC1"
constexpr uint16_t kProtoMajor     = 1;
constexpr uint16_t kProtoMinor     = 0;
constexpr uint16_t kSlotMapVersion = 2;
constexpr uint16_t kPdoBytes       = 256;
constexpr uint32_t kLeaseUs        = 100000;      // 租约 100 ms，与从站停更判定同量级

} // namespace thread::ethercat
