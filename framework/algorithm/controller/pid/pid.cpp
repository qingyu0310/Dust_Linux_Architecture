/**
 * @file pid.cpp
 * @author qingyu
 * @brief 位置式 PID 控制器实现（D 先行 / 变速积分 / 积分分离 / D 项 LPF）
 * @version 0.1
 * @date 2026-05-10
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma message "Compiling Algorithm/Controller/Pid"

#include "pid.hpp"
#include <math.h>
#include <algorithm>

namespace alg::pid {

namespace {
    constexpr float kPi  = 3.141592653589793f;
    constexpr float k2Pi = 6.283185307179586f;
}

/**
 * @brief 构造 PID，同时预计算 LPF alpha
 * @param cfg  PID 配置参数
 */
Pid::Pid(const Config& cfg) : cfg_(cfg)
{
    if (cfg_.dLpfCutoffHz > 0.0f) {
        float tau = 1.0f / (2.0f * kPi * cfg_.dLpfCutoffHz);
        dLpfAlpha_ = tau / (tau + cfg_.dt);
    }
}

/**
 * @brief 重配置 PID 参数，同时预计算 LPF alpha
 * @param cfg  PID 配置参数
 */
void Pid::Init(const Config& cfg)
{
    cfg_ = cfg;
    if (cfg_.dLpfCutoffHz > 0.0f) {
        float tau = 1.0f / (2.0f * kPi * cfg_.dLpfCutoffHz);
        dLpfAlpha_ = tau / (tau + cfg_.dt);
    } else {
        dLpfAlpha_ = 0.0f;
    }
}

/**
 * @brief 位置式 PID 计算
 * @return 控制器输出
 */
float Pid::Calc()
{
    return CalcImpl(target_ - now_);
}

/**
 * @brief 角度 PID（劣弧 + 防卡角）
 * @return 控制器输出
 */
float Pid::CalcAngle()
{
    float error = target_ - now_;
    if (error > kPi)
        error -= k2Pi;
    else if (error < -kPi)
        error += k2Pi;

    return CalcImpl(error);
}

/**
 * @brief 公共 PID 计算（死区 → 变速积分 → PID → 前馈 → 限幅）
 */
float Pid::CalcImpl(float error)
{
    // 误差
    float absError = fabsf(error);

    // 死区 
    if (absError < cfg_.deadZone) {
        target_   = now_;
        error     = 0.0f;
        absError  = 0.0f;
    } else if (error > 0.0f) {
        error    -= cfg_.deadZone;
        absError  = fabsf(error);
    } else if (error < 0.0f) {
        error    += cfg_.deadZone;
        absError  = fabsf(error);
    }

    // 变速积分（|error| ≤ Lo → 全速；|error| ≥ Hi → 停积分 + 清零；[Lo, Hi] → 线性降速）
    float speedRatio = 1.0f;
    if (cfg_.iSpeedThreshLo != 0.0f || cfg_.iSpeedThreshHi != 0.0f)
    {
        if (absError <= cfg_.iSpeedThreshLo) {
            speedRatio = 1.0f;
        } else if (absError >= cfg_.iSpeedThreshHi) {
            speedRatio = 0.0f;
            integralError_ = 0.0f;          // 超上限 → 清零防饱和
        } else {
            float denom = cfg_.iSpeedThreshHi - cfg_.iSpeedThreshLo;
            if (denom > 0.0f)
                speedRatio = (cfg_.iSpeedThreshHi - absError) / denom;
        }
    }

    // P项
    const float pOut = cfg_.kp * error;

    // I项
    float iOut = 0.0f;

    // 积分限幅 (防除零)
    if (cfg_.iOutMax != 0.0f && cfg_.ki != 0.0f) {
        float iClamp = cfg_.iOutMax / cfg_.ki;
        integralError_ = std::clamp(integralError_, -iClamp, iClamp);
    }

    // 防卡角: 输出饱和或误差反向时清零
    float absOut = fabsf(out_);
    if ((cfg_.outMax != 0.0f && absOut >= cfg_.outMax) || (preError_ > 0.0f && error < 0.0f) || (preError_ < 0.0f && error > 0.0f))
    {
        integralError_ = 0.0f;
    }

    // 积分分离（与 iSpeedThreshHi 功能重叠，关闭一个即可）
    if (cfg_.iSeparateThresh == 0.0f || absError < cfg_.iSeparateThresh) 
	{
        integralError_ += speedRatio * cfg_.dt * error;
        iOut = cfg_.ki * integralError_;
    } else {
        integralError_ = 0.0f;
    }

    // D项
    float dRaw;
    if (cfg_.dFirst == DFirst::Enable) {
        dRaw = -cfg_.kd * (now_ - preNow_) / cfg_.dt;
    } else {
        dRaw = cfg_.kd * (error - preError_) / cfg_.dt;
    }

    float dOut = dRaw;
    if (dLpfAlpha_ > 0.0f) {
        // 一阶低通：y(n) = α · y(n-1) + (1-α) · x(n)
        // 化简为一次乘法：y(n) = x(n) + α · (y(n-1) - x(n))
        dOut = dRaw + dLpfAlpha_ * (dLpfOutput_ - dRaw);
        dLpfOutput_ = dOut;
    }

    // 前馈
    const float fOut = cfg_.kf * (target_ - preTarget_);

    // 输出
    out_ = pOut + iOut + dOut + fOut;
    if (cfg_.outMax != 0.0f) {
        out_ = std::clamp(out_, -cfg_.outMax, cfg_.outMax);
    }

    // 状态保持
    preNow_    = now_;
    preTarget_ = target_;
    preOut_    = out_;
    preError_  = error;

    return out_;
}

} // namespace alg::pid
