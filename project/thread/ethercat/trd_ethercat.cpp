/**
 * @file trd_ethercat.cpp
 * @author qingyu
 * @brief EtherCAT 周期线程 —— 建链与过程数据收发
 * @version 3.2
 * @date 2026-10-30
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "Init_entry.hpp"
#include "log.hpp"
#include "thread.hpp"
#include "var.hpp"
#include <cmath>
#include <cstring>
#include <limits>
#include "framework/drivers/communication/ethercat/ecat.hpp"

#if defined(CONFIG_DUST_TPC_ECAT_RX)
#include "framework/topic/ecat_rx/ecat_rx.hpp"

namespace topic::ecat_rx
{
Channel<Message> channel_;
}

namespace topic::ecat_tx
{
Channel<Message> channel_;
}
#endif

namespace thread::ethercat
{

// 保持从站映射：双向各 64 个 uint32_t，业务包从 PDO 首字节开始。
struct PdoOutputs
{
    uint32_t counter[64];
};

struct PdoInputs
{
    uint32_t counter[64];
};

static_assert(sizeof(PdoOutputs) == 256, "RxPDO must be 256 bytes");
static_assert(sizeof(PdoInputs) == 256, "TxPDO must be 256 bytes");
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,"Velocity requires IEEE 754 float32");

/**
 * @brief MCU 状态包解析结果；当前业务字段为占位值，不代表真实裁判数据
 */
struct StatusPacket
{
    uint8_t  sequence;
    uint8_t  game_type;
    uint16_t stage_remain_s;
    uint8_t  game_progress;
    uint16_t current_hp;
    uint16_t maximum_hp;
    uint8_t  robot_id;
    uint8_t  chassis_power_enabled;
    uint16_t zone_state_and_rfid;
    uint16_t ally_control_degree;
    uint16_t enemy_control_degree;
};

static drivers::ethercat::Ecat<PdoOutputs, PdoInputs> ecat_ {};
static init::Thread thread_ {};
static std::mutex   command_mutex_;
static float linear_x_ = 0.0f;       // 前向速度 m/s，向前为正
static float linear_y_ = 0.0f;       // 横向速度 m/s，向左为正
static bool  status_invalid_ = false;

static topic::Subscriber<topic::ecat_tx::Message> command_subscriber_ {topic::ecat_tx::channel_};
static topic::ecat_tx::Message ros_command_ {};
static bool ros_command_received_ = false;
static bool ros_command_active_ = false;                			// ROS 命令正在下发
static std::chrono::steady_clock::time_point last_ros_log_ {}; 		// 下发日志限频

/**
 * @brief 命令包与状态包共用 CRC16：初值 0xFFFF，右移多项式 0x8408，无最终异或
 */
static uint16_t Crc16(const uint8_t* data, size_t size)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : crc >> 1;
        }
    }

    return crc;
}

/**
 * @brief 从未对齐的协议字节读取小端 uint16_t
 */
static uint16_t ReadUint16(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8));
}

/**
 * @brief 将 float 的 IEEE 754 位模式按小端写入，不做数值转整数
 */
static void WriteFloat(uint8_t* data, float value)
{
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));

    for (unsigned i = 0; i < sizeof(bits); ++i) {
        data[i] = static_cast<uint8_t>(bits >> (8 * i));
    }
}

/**
 * @brief 快照 var 速度并组装 15 字节命令，其余 PDO 清零；当前角速度固定为零
 */
static void UpdateCommand()
{
    float linear_x = 0.0f;
    float linear_y = 0.0f;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        linear_x = linear_x_;
        linear_y = linear_y_;
    }

    if (!std::isfinite(linear_x) || !std::isfinite(linear_y)) {
        linear_x = 0.0f;
        linear_y = 0.0f;
    }

    if (command_subscriber_.TakeLatest(ros_command_))
    {
        ros_command_received_ = true;
    }

    if (ros_command_received_)
    {
        const auto age = std::chrono::steady_clock::now() - ros_command_.received_at;
        if (ros_command_.received_at != std::chrono::steady_clock::time_point {} && age < std::chrono::milliseconds(100))
        {
            // 完整 PDO 原样下发，协议头、CRC 和速度有限性由 MCU 检查。
            std::memcpy(&ecat_.OutputsRef(), ros_command_.data, sizeof(ros_command_.data));
            const auto now = std::chrono::steady_clock::now();
            if (!ros_command_active_ || now - last_ros_log_ >= std::chrono::seconds(1))
            {
                last_ros_log_ = now;
                const auto* data = ros_command_.data;
                const uint16_t received_crc = ReadUint16(data + 13);
                const uint16_t calculated_crc = Crc16(data, 13);
                DUST_LOG_INF("ecat: ROS command selected age_ms=%lld header=0x%02x crc=0x%04x calculated=0x%04x first_float=%02x %02x %02x %02x",
                             static_cast<long long>(std::chrono::duration_cast<std::chrono::milliseconds>(age).count()),
                             static_cast<unsigned>(data[0]), static_cast<unsigned>(received_crc), static_cast<unsigned>(calculated_crc),
                             static_cast<unsigned>(data[1]), static_cast<unsigned>(data[2]), static_cast<unsigned>(data[3]), static_cast<unsigned>(data[4]));
            }
            ros_command_active_ = true;
            return;
        }

        if (ros_command_active_)
        {
            DUST_LOG_WRN("ecat: ROS command expired or invalid, sending zero velocity");
        }
        ros_command_active_ = false;

        // ROS 接管后停更不回退到 var，组装合法零速度包。
        linear_x = 0.0f;
        linear_y = 0.0f;
    }

    auto& outputs = ecat_.OutputsRef();
    outputs = {};
    auto* data = reinterpret_cast<uint8_t*>(&outputs);
    data[0] = 0x6B;
    WriteFloat(data + 1, linear_x);
    WriteFloat(data + 5, linear_y);
    WriteFloat(data + 9, 0.0f);

    const uint16_t crc = Crc16(data, 13);
    data[13] = static_cast<uint8_t>(crc);
    data[14] = static_cast<uint8_t>(crc >> 8);
}

/**
 * @brief 校验 20 字节状态包，成功后才提交解析结果；不检查序号连续性
 */
static bool ParseStatus(const PdoInputs& inputs, StatusPacket& status)
{
    const auto* data = reinterpret_cast<const uint8_t*>(&inputs);

    if (data[0] != 0x6D || ReadUint16(data + 18) != Crc16(data, 18)) {
        return false;
    }

    status.sequence               = data[1];
    status.game_type              = data[2];
    status.stage_remain_s         = ReadUint16(data + 3);
    status.game_progress          = data[5];
    status.current_hp             = ReadUint16(data + 6);
    status.maximum_hp             = ReadUint16(data + 8);
    status.robot_id               = data[10];
    status.chassis_power_enabled  = data[11];
    status.zone_state_and_rfid    = ReadUint16(data + 12);
    status.ally_control_degree    = ReadUint16(data + 14);
    status.enemy_control_degree   = ReadUint16(data + 16);
    return true;
}

/**
 * @brief 任务单拍：填输出、发一帧、收一帧
 *
 * 只走正常路径，失败一律返回 Retry，处置交给 OnRetry。
 *
 * @param context 未使用
 * @return Continue=本拍完成；Retry=交给 OnRetry 处置
 */
static init::Thread::TaskResult Task(void* context)
{
    (void)context;

    if (!ecat_.IsReady())
    {
        DUST_LOG_WRN("ecat: task fail: bus not ready");
        return init::Thread::TaskResult::Retry;
    }

    UpdateCommand();

    if (!ecat_.Send())
    {
        DUST_LOG_WRN("ecat: task fail: send failed");
        return init::Thread::TaskResult::Retry;
    }

    if (ecat_.Read() < 0)
    {
        DUST_LOG_WRN("ecat: task fail: read failed");
        return init::Thread::TaskResult::Retry;
    }

	#if defined(CONFIG_DUST_TPC_ECAT_RX)
	{
		topic::ecat_rx::Message received {};
		static_assert(sizeof(received.data) == sizeof(PdoInputs));
		std::memcpy(received.data, &ecat_.InputsRef(), sizeof(received.data));
		topic::ecat_rx::channel_.Publish(received);
	}
	#endif

    StatusPacket status {};
    const bool valid = ParseStatus(ecat_.InputsRef(), status);
    if (!valid)
    {
        if (!status_invalid_) {
            DUST_LOG_WRN("ecat: invalid status header or CRC");
        }
        status_invalid_ = true;
        // 业务包错误不重建链路，继续收下一拍。
        return init::Thread::TaskResult::Continue;
    }
    if (status_invalid_) {
        DUST_LOG_OK("ecat: status packet valid again");
    }
    status_invalid_ = false;

    DUST_LOG_DBG("ethercat",
                 "seq=%u game_type=%u stage_remain_s=%u progress=%u hp=%u/%u robot_id=%u power=%u zone_rfid=0x%04x control=%u/%u wkc=%u/%u",
                 static_cast<unsigned>(status.sequence), static_cast<unsigned>(status.game_type),
                 static_cast<unsigned>(status.stage_remain_s), static_cast<unsigned>(status.game_progress),
                 static_cast<unsigned>(status.current_hp), static_cast<unsigned>(status.maximum_hp),
                 static_cast<unsigned>(status.robot_id), static_cast<unsigned>(status.chassis_power_enabled),
                 static_cast<unsigned>(status.zone_state_and_rfid),
                 static_cast<unsigned>(status.ally_control_degree), static_cast<unsigned>(status.enemy_control_degree),
                 static_cast<unsigned>(ecat_.LastWkc()), static_cast<unsigned>(ecat_.ExpectedWkc()));

    return init::Thread::TaskResult::Continue;
}

/**
 * @brief 链路故障处置：先尝试原地恢复，恢复不了再重建链路
 *
 * 从站退 SAFE-OP 只是输出了停，链路还在，`Recover()` 确认错误位后重新请求 OP，
 * 比拆链重建快得多；只有链路真断了（读不到 AL 状态）才 `Close` + `Init`。
 *
 * @param error   框架构造的异常
 * @param thread  出故障的线程，本钩子不使用
 * @param context 未使用
 * @return Restart=重进任务
 */
static RecoveryAction OnRetry(const std::exception& error, init::Thread& thread, void* context)
{
    (void)thread;
    (void)context;

    DUST_LOG_WRN("ecat: task retry: %s", error.what());

    if (ecat_.Recover())
    {
        return RecoveryAction::Restart;
    }

    DUST_LOG_WRN("ecat: recover failed, rebuilding link");

    ecat_.Close();

    try
    {
        if (!ecat_.Init({}))
        {
            DUST_LOG_ERR("ecat: bring-up failed on %s", ecat_.InterfaceName().c_str());
        }
    }
    catch (const std::exception& bring_up_error)
    {
        DUST_LOG_ERR("ecat: bring-up exception: %s", bring_up_error.what());
    }

    return RecoveryAction::Restart;
}

/**
 * @brief 建链，没建链成功不放行
 *
 * @return true=全部从站进入 OP
 */
bool thread_init()
{
    UpdateCommand();
    while (!ecat_.Init({}))
    {
        DUST_LOG_ERR("ecat: init failed, retry in 10 ms");
        std::this_thread::sleep_for(DUST_MSEC(1000));
    }

    return true;
}

/**
 * @brief 启动周期线程
 *
 * 总线掉了挂在它上面的所有设备全部失控，只停自己没意义，按 Recover 处置。
 *
 * @return true=启动成功
 */
bool thread_start()
{
    thread_.Start(Task, DUST_MSEC(1), nullptr, "ethercat", ThreadFaultConfig{OnRetry, nullptr, ThreadFaultPolicy::Recover, false});

    return true;
}

/**
 * @brief 通知周期线程停止
 */
void thread_stop()
{
    thread_.RequestStop();
}

/**
 * @brief 等待周期线程退出后关闭主站
 */
void thread_join()
{
    thread_.Join();
}

/**
 * @brief 关闭主站，支持未初始化与重复调用
 */
void Release()
{
    ecat_.Close();
}

REGISTER_SHELL_VAR("ecat.linear_x", linear_x_, command_mutex_);
REGISTER_SHELL_VAR("ecat.linear_y", linear_y_, command_mutex_);

REGISTER_INIT(thread_init, MidInit, High, HaltOnFail, Release, "ethercat_init");
REGISTER_THREAD(thread_start, MidThread, thread_stop, thread_join, "ethercat_start");

} // namespace thread::ethercat
