/**
 * @file pdo_layout.hpp
 * @author qingyu
 * @brief 上位机 PDO 协议 v1 布局 —— RxPDO / TxPDO 各 256 B packed 结构
 * @version 0.3
 * @date 2026-11-26
 *
 * @copyright Copyright (c) 2026
 *
 * 与从站 v1（slot_map_version=1）逐字节对齐：全部 packed、无填充，
 * 字节序按 x86/riscv 的小端直接映射，浮点为 IEEE-754 binary32。
 */

#pragma once

#include <cstdint>
#include <limits>

namespace thread::ethercat
{

#pragma pack(push, 1)

/**
 * @brief TxPDO 反馈头（偏移 0，40 B）
 */
struct FeedbackHeader
{
    uint32_t magic;                  // 固定 0x31424450（"PDB1"）
    uint16_t proto_major;            // 1
    uint16_t proto_minor;            // 0
    uint32_t mcu_boot_id;            // 每次上电重新生成，主站据此判断从站重启
    uint32_t feedback_seq;           // 每 1 ms 自增，溢出回绕
    uint32_t mcu_tick_us;            // 本帧组帧时刻，MCU 本地 µs
    uint32_t applied_command_seq;    // 已执行的 command_seq
    uint16_t accepted_host_session;  // 被接纳的主机会话号；0=无有效会话
    uint16_t operation_ack_seq;      // 操作应答序号
    uint16_t status_flags;           // bit0=已接纳会话
    uint16_t fault_code;             // 故障码；0=正常
    uint8_t  armed_mask;             // 本版恒 0
    uint8_t  can_health;             // 本版恒 0
    uint8_t  reserved[6];            // 全零
};

/**
 * @brief 遥控反馈（偏移 40，32 B）
 */
struct RemoteFeedback
{
    uint32_t source_seq;             // 遥控消息帧序号，每解析一帧自增
    uint32_t sample_tick_us;         // 该帧采样时刻，MCU 本地 µs
    float    axis[4];                // chassisx / chassisy / yaw / pitch，归一化 [-1, 1]
    uint16_t controls;               // 开关位：bit0 总开关，bit1 Spin，bit2~bit10 其余开关
    uint8_t  protocol;               // 0=未报告
    uint8_t  status;                 // bit0=已收到过有效遥控帧
    uint8_t  reserved[4];            // 全零
};

/**
 * @brief IMU 反馈（偏移 72，56 B）
 */
struct ImuFeedback
{
    uint32_t source_seq;             // 姿态消息帧序号，每解算一帧自增
    uint32_t sample_tick_us;         // 该帧采样时刻，MCU 本地 µs
    float    quaternion[4];          // w, x, y, z
    float    gyro[3];                // rad/s
    float    accel[3];               // m/s²
    float    temperature;            // ℃
    uint16_t flags;                  // bit0=已产生过姿态
    uint8_t  reserved[2];            // 全零
};

/**
 * @brief TxPDO：从站 → 主站，256 B
 */
struct PdoInputs
{
    FeedbackHeader header;           // 偏移 0
    RemoteFeedback remote;           // 偏移 40
    ImuFeedback    imu;              // 偏移 72
    uint8_t        motor_feedback[128];  // 偏移 128，本版全零
};

/**
 * @brief RxPDO 命令头（偏移 0，48 B）
 */
struct CommandHeader
{
    uint32_t magic;                  // 必须 0x31434D43（"CMC1"）
    uint16_t proto_major;            // 1
    uint16_t proto_minor;            // 0
    uint16_t byte_length;            // 必须 256
    uint16_t slot_map_version;       // 必须 1
    uint16_t host_session;           // 主机会话号，会话生命周期内不变
    uint16_t reserved0;              // 0
    uint32_t command_seq;            // 只有产生新决策时才自增
    uint32_t mcu_boot_id_echo;       // 回显最近一次 TxPDO 的 mcu_boot_id
    uint32_t feedback_seq_echo;      // 本命令依据的 feedback_seq
    uint32_t lease_us;               // 租约上限 µs，从站侧再夹一层
    uint32_t compute_time_us;        // 主站组帧耗时，诊断用
    uint16_t flags;                  // bit0 请求 arm；bit1 请求 disarm
    uint16_t enable_mask;            // 本版忽略
    uint8_t  reserved[8];            // 全零
};

/**
 * @brief RxPDO：主站 → 从站，256 B
 */
struct PdoOutputs
{
    CommandHeader header;            // 偏移 0
    uint8_t       motor_command[128];    // 偏移 48，前 8 B = C620 电流帧（CAN 0x200），其余全零
    uint8_t       device_operation[32];  // 偏移 176，本版写零
    uint8_t       diagnostic[16];        // 偏移 208，暂未处理
    uint8_t       reserved[32];          // 偏移 224，必须全零
};

#pragma pack(pop)

static_assert(sizeof(FeedbackHeader) == 40, "FeedbackHeader must be 40 bytes");
static_assert(sizeof(RemoteFeedback) == 32, "RemoteFeedback must be 32 bytes");
static_assert(sizeof(ImuFeedback)    == 56, "ImuFeedback must be 56 bytes");
static_assert(sizeof(CommandHeader)  == 48, "CommandHeader must be 48 bytes");
static_assert(sizeof(PdoInputs)      == 256, "TxPDO must be 256 bytes");
static_assert(sizeof(PdoOutputs)     == 256, "RxPDO must be 256 bytes");
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559, "PDO requires IEEE 754 binary32");

constexpr uint32_t kFeedbackMagic  = 0x31424450;  // "PDB1"，小端读为 50 44 42 31
constexpr uint32_t kCommandMagic   = 0x31434D43;  // "CMC1"
constexpr uint16_t kProtoMajor     = 1;
constexpr uint16_t kProtoMinor     = 0;
constexpr uint16_t kSlotMapVersion = 1;
constexpr uint16_t kPdoBytes       = 256;
constexpr uint32_t kLeaseUs        = 100000;      // 租约 100 ms，与从站停更判定同量级

} // namespace thread::ethercat
