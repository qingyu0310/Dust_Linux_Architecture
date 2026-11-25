/**
 * @file pid.hpp
 * @author qingyu
 * @brief 位置式 PID 控制器（D 先行 / 变速积分 / 积分分离 / D 项 LPF）
 * @version 0.1
 * @date 2026-04-29
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include <stdint.h>

namespace alg::pid {

/// 微分先行模式（避免目标值突变引起的微分冲击）
enum class DFirst : uint8_t {
    Disable = 0,  // D 作用于误差
    Enable,       // D 作用于测量值（推荐用于定值跟踪）
};

/**
 * @brief 位置式 PID 控制器
 *
 * @par 功能特性
 *      - 标准 P/I/D + 前馈
 *      - 死区
 *      - 变速积分（变速积分）
 *      - 积分分离（积分分离）
 *      - 微分先行（微分先行）
 *      - D 项低通滤波
 *      - 输出 / 积分限幅
 *      - 角度模式（劣弧环绕处理）
 *
 * @par 用法
 * @code
 *   alg::pid::Pid pid({.kp = 3.0f, .ki = 0.1f, .kd = 0.05f, .outMax = 60.0f});
 *   float out = pid.Calc(target, now);
 * @endcode
 */
class Pid final
{
public:
    /**
 * @brief PID 配置参数（全部有默认值）
     *
     * @note 推荐使用 C++20 指定初始化器：
     *       @c Pid::Config{.kp=3, .ki=0.1, .outMax=60}
     */
    struct Config {
        float kp               = 0.0f;
        float ki               = 0.0f;
        float kd               = 0.0f;
        float kf               = 0.0f;                  // 前馈增益
        float iOutMax          = 0.0f;                  // 0 = 无积分限幅
        float outMax           = 0.0f;                  // 0 = 无输出限幅
        float dt               = 0.001f;                // 控制周期（秒）
        float deadZone         = 0.0f;
        float iSpeedThreshLo   = 0.0f;                  // 变速积分误差阈值下限（|error| ≤ Lo → 全速积分）
        float iSpeedThreshHi   = 0.0f;                  // 变速积分误差阈值上限（|error| ≥ Hi → 停积分 + 清零）
        float iSeparateThresh  = 0.0f;                  // 积分分离阈值（0 = 关闭。与 iSpeedThreshHi 功能重叠，建议二选一）
        DFirst dFirst          = DFirst::Disable;
        float dLpfCutoffHz     = 0.0f;                  // D 项 LPF 截止频率（Hz，0 = 无滤波；alpha = 1/(1 + 2πfc·dt)）
    };

    Pid() = default;
    explicit Pid(const Config& cfg);

    void Init(const Config& cfg);

    float Calc(float target, float now)
    {
        SetTarget(target);
        SetNow(now);
        return Calc();
    }

    float Calc();

    float CalcAngle();

    void SetKp(float v)             { cfg_.kp = v; }
    void SetKi(float v)             { cfg_.ki = v; }
    void SetKd(float v)             { cfg_.kd = v; }
    void SetKf(float v)             { cfg_.kf = v; }
    void SetIOutMax(float v)        { cfg_.iOutMax = v; }
    void SetOutMax(float v)         { cfg_.outMax = v; }
    void SetTarget(float v)         { target_ = v; }
    void SetNow(float v)            { now_ = v; }
    void SetIntegralError(float v)  { integralError_ = v; }

    const Config& GetConfig() const { return cfg_; }

    float GetKp()             const { return cfg_.kp; }
    float GetKi()             const { return cfg_.ki; }
    float GetKd()             const { return cfg_.kd; }
    float GetKf()             const { return cfg_.kf; }

	constexpr float &GetKp()        { return cfg_.kp; }
    constexpr float &GetKi()        { return cfg_.ki; }
	constexpr float &GetKd()        { return cfg_.kd; }
    constexpr float &GetKf()        { return cfg_.kf; }

    float GetOutMax()         const { return cfg_.outMax; }
    float GetIOutMax()        const { return cfg_.iOutMax; }
    float GetDt()             const { return cfg_.dt; }

    float GetOut()            const { return out_; }
    float GetTarget()         const { return target_; }
    float GetIntegralError()  const { return integralError_; }
    float GetError()          const { return preError_; }

private:

    Config cfg_ {};

    float target_        = 0.0f;
    float now_           = 0.0f;
    float out_           = 0.0f;

    float preNow_        = 0.0f;
    float preTarget_     = 0.0f;
    float preOut_        = 0.0f;
    float preError_      = 0.0f;

    float integralError_ = 0.0f;
    float dLpfAlpha_     = 0.0f;
    float dLpfOutput_    = 0.0f;

    float CalcImpl(float error);
};

} // namespace alg::pid
