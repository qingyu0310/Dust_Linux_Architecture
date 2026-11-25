/**
 * @file trd_ethercat.cpp
 * @author qingyu
 * @brief EtherCAT 周期线程 —— 建链、上位机 PDO 协议 v1 双向收发与电机命令下发
 * @version 4.7
 * @date 2026-11-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "Init_entry.hpp"
#include "log.hpp"
#include "thread.hpp"
#include "pdo_layout.hpp"
#include <chrono>
#include <cstdint>
#include <cstring>
#include "framework/drivers/communication/ethercat/ecat.hpp"

namespace thread::ethercat
{

using namespace drivers::ethercat;

static Ecat<PdoOutputs, PdoInputs> ecat_ {};
static init::Thread thread_ {};

/**
 * @brief 主站配置：网口留空，由驱动自动选第一个有载波的网口
 * @return 主站配置
 */
static Config MainConfig()
{
    Config config;

    return config;
}

static uint16_t host_session_     = 0;      // 当前主机会话号；0=未建立
static uint16_t session_counter_  = 0;      // 会话号递增源
static uint32_t command_seq_      = 0;      // 新决策计数
static uint32_t boot_id_          = 0;      // 最近有效反馈的 mcu_boot_id
static uint32_t last_feedback_seq_ = 0;     // 最近有效反馈的 feedback_seq
static bool     have_boot_id_     = false;  // 是否已读到过有效反馈
static bool     feedback_invalid_ = false;  // 上一帧反馈是否校验失败
static bool     session_accepted_ = false;  // 从站是否已接纳当前会话

/**
 * @brief 校验 TxPDO 头并维护会话状态；boot id 变化按从站重启处理
 *
 * 校验失败只标记本帧无效，不打断链路；从站重启后重新生成会话号，
 * 下一次组帧时把新的 boot id 回显出去。
 *
 * @param inputs 从站反馈镜像
 * @return true=本帧通过 magic 与版本校验
 */
static bool ParseFeedback(const PdoInputs& inputs)
{
    const auto& header = inputs.header;

    if (header.magic != kFeedbackMagic || header.proto_major != kProtoMajor)
    {
        if (!feedback_invalid_)
        {
            DUST_LOG_WRN("ecat: feedback rejected magic=0x%08x expected=0x%08x proto=%u.%u",
                         static_cast<unsigned>(header.magic), static_cast<unsigned>(kFeedbackMagic),
                         static_cast<unsigned>(header.proto_major), static_cast<unsigned>(header.proto_minor));
        }
        feedback_invalid_ = true;
        return false;
    }
    if (feedback_invalid_)
    {
        DUST_LOG_OK("ecat: feedback valid again magic=0x%08x proto=%u.%u",
                    static_cast<unsigned>(header.magic),
                    static_cast<unsigned>(header.proto_major), static_cast<unsigned>(header.proto_minor));
    }
    feedback_invalid_ = false;

    if (!have_boot_id_ || header.mcu_boot_id != boot_id_)
    {
        if (have_boot_id_)
        {
            DUST_LOG_WRN("ecat: slave restarted boot=0x%08x -> 0x%08x",
                         static_cast<unsigned>(boot_id_), static_cast<unsigned>(header.mcu_boot_id));
        }
        have_boot_id_ = true;
        boot_id_ = header.mcu_boot_id;

        // 新 boot id 视为新决策：换会话号并让 command_seq 前进一次
        ++session_counter_;
        host_session_ = (session_counter_ == 0) ? 1 : session_counter_;
        ++command_seq_;
        session_accepted_ = false;

        DUST_LOG_INF("ecat: new host session=%u boot=0x%08x command_seq=%u",
                     static_cast<unsigned>(host_session_), static_cast<unsigned>(boot_id_),
                     static_cast<unsigned>(command_seq_));
    }

    last_feedback_seq_ = header.feedback_seq;

    const bool accepted = header.accepted_host_session == host_session_ && (header.status_flags & 0x0001u) != 0;
    if (accepted != session_accepted_)
    {
        session_accepted_ = accepted;
        if (accepted)
        {
            DUST_LOG_OK("ecat: session %u accepted", static_cast<unsigned>(host_session_));
        }
        else
        {
            DUST_LOG_WRN("ecat: session %u not accepted accepted_session=%u status_flags=0x%04x",
                         static_cast<unsigned>(host_session_), static_cast<unsigned>(header.accepted_host_session),
                         static_cast<unsigned>(header.status_flags));
        }
    }

    return true;
}

/**
 * @brief 按协议 v1 组一帧 RxPDO：命令头 + 电机命令区，其余预留区保持全零
 *
 * 组帧耗时用 steady_clock 实测后写入 compute_time_us，供从站侧诊断。
 */
static void UpdateCommand()
{
    auto& outputs = ecat_.OutputsRef();
    outputs = {};

    const auto begin = std::chrono::steady_clock::now();

    auto& header = outputs.header;

    header.magic 	  		 = kCommandMagic;
    header.proto_major		 = kProtoMajor;
    header.proto_minor 		 = kProtoMinor;
    header.byte_length       = kPdoBytes;
    header.slot_map_version  = kSlotMapVersion;
    header.host_session 	 = host_session_;
    header.command_seq  	 = command_seq_;
    header.mcu_boot_id_echo  = boot_id_;
    header.feedback_seq_echo = last_feedback_seq_;
    header.lease_us 		 = kLeaseUs;
    header.flags			 = 0;

    const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin).count();
    header.compute_time_us = static_cast<uint32_t>(elapsed_us);
}

/**
 * @brief 任务单拍：填输出、发一帧、收一帧、解析反馈
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

    const PdoInputs& inputs = ecat_.InputsRef();
    if (!ParseFeedback(inputs))
    {
        // 业务包错误不重建链路，继续收下一拍。
        return init::Thread::TaskResult::Continue;
    }

    const auto& header = inputs.header;
    const auto& remote = inputs.remote;
    const auto& imu    = inputs.imu;

    DUST_LOG_DBG("ethercat",
                 "seq=%u tick_us=%u boot=0x%08x applied=%u session=%u accepted=%u status=0x%04x "
                 "remote=%u axis=%.3f/%.3f/%.3f/%.3f ctrl=0x%04x "
                 "imu=%u quat=%.3f/%.3f/%.3f/%.3f gyro=%.2f/%.2f/%.2f wkc=%u/%u",
                 static_cast<unsigned>(header.feedback_seq), static_cast<unsigned>(header.mcu_tick_us),
                 static_cast<unsigned>(header.mcu_boot_id), static_cast<unsigned>(header.applied_command_seq),
                 static_cast<unsigned>(host_session_), static_cast<unsigned>(header.accepted_host_session),
                 static_cast<unsigned>(header.status_flags),
                 static_cast<unsigned>(remote.source_seq),
                 static_cast<double>(remote.axis[0]), static_cast<double>(remote.axis[1]),
                 static_cast<double>(remote.axis[2]), static_cast<double>(remote.axis[3]),
                 static_cast<unsigned>(remote.controls),
                 static_cast<unsigned>(imu.source_seq),
                 static_cast<double>(imu.quaternion[0]), static_cast<double>(imu.quaternion[1]),
                 static_cast<double>(imu.quaternion[2]), static_cast<double>(imu.quaternion[3]),
                 static_cast<double>(imu.gyro[0]), static_cast<double>(imu.gyro[1]), static_cast<double>(imu.gyro[2]),
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
        if (!ecat_.Init(MainConfig()))
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
    while (!ecat_.Init(MainConfig()))
    {
        DUST_LOG_ERR("ecat: init failed, retry in 1000 ms");
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

REGISTER_INIT(thread_init, MidInit, High, HaltOnFail, Release, "ethercat_init");
REGISTER_THREAD(thread_start, MidThread, thread_stop, thread_join, "ethercat_start");

} // namespace thread::ethercat
