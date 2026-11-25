/**
 * @file trd_chassis.cpp
 * @author qingyu
 * @brief 全向轮底盘线程 —— 遥控速度 → 逆运动学 → 四轮角速度环 → 电机命令话题
 * @version 0.1
 * @date 2026-11-25
 *
 * @copyright Copyright (c) 2026
 *
 * 控制闭环全部在上位机：遥控与电机反馈来自 EtherCAT 线程发布的 topic，
 * 算出的四轮电流经 topic::ecat_chassis::chassis_command_channel_ 交给 EtherCAT 线程
 * 写进 RxPDO 电机命令区，再由主站周期下发到从站。
 */

#include "Init_entry.hpp"
#include "log.hpp"
#include "thread.hpp"
#include "var.hpp"
#include "trd_chassis.hpp"
#include "framework/algorithm/controller/pid/pid.hpp"
#include "framework/topic/ecat_chassis/ecat_chassis.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>

namespace thread::chassis
{

using namespace topic::ecat_chassis;
using namespace topic;
using namespace alg::pid;

static init::Thread thread_ {};
static std::mutex   var_mutex_;           			// chassis_v / chassis_w 的 shell 访问锁

static Pid   pid_[kMotorCount] {};        			// 四轮速度环
static float last_target_[kMotorCount] {};        	// 惯性前馈上一拍目标
static bool  target_valid_[kMotorCount] {};       	// 闭环切入首拍不做目标差分

static Subscriber<ChassisRemoteMessage>   remote_sub_   {chassis_remote_channel_};       // 遥控订阅者
static Subscriber<ChassisFeedbackMessage> feedback_sub_ {chassis_feedback_channel_};   // 电机反馈订阅者

static ChassisRemoteMessage   remote_state_ {};          		// 最近一次遥控状态 
static ChassisFeedbackMessage feedback_state_ {};        		// 最近一次电机反馈

static std::chrono::steady_clock::time_point last_tick_ {};  	// 上一拍时刻
static bool have_last_tick_ = false;              				// 是否已有上一拍时刻

/**
 * @brief 摩擦前馈电流：按该轮辨识的库仑+黏性+静摩擦模型算出维持该转速所需电流
 *
 * 方向由目标转速决定（几乎阶跃，保证低速也能给满静摩擦），
 * 静摩擦按实际转速衰减（轮子真正停住时才是满值）。
 *
 * @param wheel  轮号
 * @param target 目标转速 rad/s
 * @param omega  实际转速 rad/s
 * @return 前馈电流 A
 */
static float FrictionFeedforward(uint8_t wheel, float target, float omega)
{
    const bool  forward  = (target >= 0.0f);
    const float a        = forward ? kFricA[wheel] : kFricANeg[wheel];
    const float b        = forward ? kFricB[wheel] : kFricBNeg[wheel];
    const float a_static = forward ? kFricStatic[wheel] : kFricStaticNeg[wheel];

    // 方向：用目标转速，平滑宽度小，低速目标也能拿到满静摩擦
    const float s = std::tanh(target / kFricStaticSmooth);

    // 静摩擦随实际转速平方指数衰减回库仑值
    const float r     = omega / kFricSmooth;
    const float a_eff = a + (a_static - a) * std::exp(-r * r);

    return a_eff * s + b * target;
}

/**
 * @brief 加速度前馈：按目标转速变化率给出 J·α 对应的电流
 *
 * @param wheel 轮号
 * @param omega 目标转速 rad/s
 * @param dt    控制周期 s
 * @return 惯性前馈电流 A
 */
static float InertiaFeedforward(uint8_t wheel, float omega, float dt)
{
    float accel = 0.0f;

    if (target_valid_[wheel] && dt > 1.0e-4f)
    {
        accel = std::clamp((omega - last_target_[wheel]) / dt, -kAccelLimit, kAccelLimit);
    }

    last_target_[wheel]  = omega;
    target_valid_[wheel] = true;

    return kJEq[wheel] * accel;
}

/**
 * @brief 每轮速度 PI + 摩擦/惯性前馈，总电流限幅并按总输出做条件积分
 */
static float SpeedCurrent(uint8_t wheel, float target, float omega, float dt)
{
    auto& ctrl   = pid_[wheel];
    const float feedforward     = FrictionFeedforward(wheel, target, omega) + InertiaFeedforward(wheel, target, dt);
    const float integral_before = ctrl.GetIntegralError();

    float feedback = ctrl.Calc(target, omega);

    const float integral_delta = ctrl.GetKi() * (ctrl.GetIntegralError() - integral_before);
    const float requested      = feedback + feedforward;

    // PI 内部只能看到反馈电流；前馈使总输出饱和时，也不能继续向饱和方向积分。
    if ((requested > kControlCurrentMax && integral_delta > 0.0f) || (requested < -kControlCurrentMax && integral_delta < 0.0f))
    {
        ctrl.SetIntegralError(integral_before);
        feedback -= integral_delta;
    }

    return std::clamp(feedback + feedforward, -kControlCurrentMax, kControlCurrentMax);
}

/**
 * @brief 重置速度环
 */
static void ResetSpeedControl(uint8_t wheel)
{
    pid_[wheel].SetIntegralError(0.0f);
    target_valid_[wheel] = false;
}

/**
 * @brief 电流 A → C620 原始值（±20 A ↔ ±16384），按 int16 位模式返回
 */
static uint16_t CurrentRaw(float current)
{
    const float scaled = std::clamp(current * kCurrentRawPerAmp, -kCurrentRawLimit, kCurrentRawLimit);
    return static_cast<uint16_t>(static_cast<int16_t>(scaled));
}

/**
 * @brief 任务单拍：读遥控与电机反馈 → 逆运动学 → 四轮速度环 → 发布 CAN 电流帧
 *
 * @param context 未使用
 * @return Continue=正常
 */
static init::Thread::TaskResult Task(void* context)
{
    (void)context;

    const auto now = std::chrono::steady_clock::now();
    float dt = 0.001f;

    if (have_last_tick_)
    {
        dt = std::chrono::duration<float>(now - last_tick_).count();
        dt = std::clamp(dt, 0.0001f, 0.05f);
    }
    last_tick_      = now;
    have_last_tick_ = true;

    // 每拍取最新状态，不积压历史通知；读不到保留上一帧值
    ChassisRemoteMessage remote {};
    if (remote_sub_.TakeLatest(remote))
    {
        remote_state_ = remote;
    }

    ChassisFeedbackMessage feedback {};
    if (feedback_sub_.TakeLatest(feedback))
    {
        feedback_state_ = feedback;
    }

    float v_limit = 0.0f, w_limit = 0.0f;
    {
        std::lock_guard<std::mutex> lock(var_mutex_);
        v_limit = chassis_v;
        w_limit = chassis_w;
    }

    const bool  enabled = (remote_state_.controls & 0x0001u) != 0;
    const bool  spin    = (remote_state_.controls & 0x0002u) != 0;
    const float vx      = enabled ? remote_state_.axis[0] * v_limit : 0.0f;
    const float vy      = enabled ? remote_state_.axis[1] * v_limit : 0.0f;

    float target[kMotorCount] {};
    float current[kMotorCount] {};

    if (!enabled)
    {
        for (uint8_t i = 0; i < kMotorCount; ++i)
        {
            ResetSpeedControl(i);
        }
    }
    else
    {
        static constexpr float kSqrt2 = 1.41421356f;   // 对角线 → 边投影

        // 轮角速度 = 底盘线速度 / 轮半径
        const float mx = vx / kWheelRadius;
        const float my = vy / kWheelRadius;
        const float wz = spin ? w_limit : 0.0f;

        target[0] = (-mx + my) * kSqrt2 + wz;
        target[1] = (-mx - my) * kSqrt2 + wz;
        target[2] = ( mx - my) * kSqrt2 + wz;
        target[3] = ( mx + my) * kSqrt2 + wz;

        for (uint8_t i = 0; i < kMotorCount; ++i)
        {
            const float omega = feedback_state_.slot[i].omega;

            // 摩擦前馈补偿维持该转速所需的电流，惯性前馈补偿加减速，PI 只修残差
            current[i] = SpeedCurrent(i, target[i], omega, dt);
        }
    }

    ChassisCommandMessage command {};
    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        const uint16_t raw = CurrentRaw(current[i]);
        command.data[2 * i]     = static_cast<uint8_t>(raw & 0xFFu);
        command.data[2 * i + 1] = static_cast<uint8_t>(raw >> 8);
    }
    chassis_command_channel_.Publish(command);

    DUST_LOG_DBG("chassis",
                 "enabled=%u spin=%u vx=%.3f vy=%.3f w=%.1f current=%.2f/%.2f/%.2f/%.2f "
                 "omega=%.2f/%.2f/%.2f/%.2f dt_ms=%.2f",
                 enabled ? 1U : 0U, spin ? 1U : 0U,
                 static_cast<double>(vx), static_cast<double>(vy), static_cast<double>(spin ? w_limit : 0.0f),
                 static_cast<double>(current[0]), static_cast<double>(current[1]),
                 static_cast<double>(current[2]), static_cast<double>(current[3]),
                 static_cast<double>(feedback_state_.slot[0].omega), static_cast<double>(feedback_state_.slot[1].omega),
                 static_cast<double>(feedback_state_.slot[2].omega), static_cast<double>(feedback_state_.slot[3].omega),
                 static_cast<double>(dt * 1000.0f));

    return init::Thread::TaskResult::Continue;
}

/**
 * @brief 初始化四轮速度环参数
 *
 * 电流域模型 J*dω/dt+b*ω=I-I_friction，按每轮 J/b 配置相同的名义闭环极点。
 *
 * @return true=成功
 */
bool thread_init()
{
    Pid::Config pid_cfg {};

    pid_cfg.outMax  = kControlCurrentMax;
    pid_cfg.iOutMax = kIntegralCurrentMax;
    pid_cfg.dt      = 0.001f;
    pid_cfg.kd      = 0.0f;

    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        const float b  = 0.5f * (kFricB[i] + kFricBNeg[i]);
        const float kp = 2.0f * kSpeedLoopZeta * kSpeedLoopWn * kJEq[i] - b;
        const float ki = kJEq[i] * kSpeedLoopWn * kSpeedLoopWn;

        pid_cfg.kp = kp;
        pid_cfg.ki = ki;
        pid_[i].Init(pid_cfg);

        DUST_LOG_INF("wheel%u PI: kp=%.6f ki=%.6f", static_cast<unsigned>(i), static_cast<double>(kp), static_cast<double>(ki));
    }

    DUST_LOG_INF("chassis ready");

    return true;
}

/**
 * @brief 启动底盘线程
 * @return true=成功
 */
bool thread_start()
{
    thread_.Start(Task, DUST_MSEC(1), nullptr, "chassis", ThreadFaultConfig{nullptr, nullptr, ThreadFaultPolicy::Isolate, false});

    return true;
}

/**
 * @brief 通知底盘线程停止
 */
void thread_stop()
{
    thread_.RequestStop();
}

/**
 * @brief 等待底盘线程退出
 */
void thread_join()
{
    thread_.Join();
}

REGISTER_SHELL_VAR("chassis_v", chassis_v, var_mutex_);
REGISTER_SHELL_VAR("chassis_w", chassis_w, var_mutex_);

REGISTER_INIT(thread_init, LateInit, Mid, ContinueOnFail, nullptr, "chassis_init");
REGISTER_THREAD(thread_start, LateThread, thread_stop, thread_join, "chassis_start");

} // namespace thread::chassis
