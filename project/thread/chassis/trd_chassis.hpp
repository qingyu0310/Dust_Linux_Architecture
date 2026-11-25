/**
 * @file trd_chassis.hpp
 * @author qingyu
 * @brief 四轮全向轮底盘参数 —— 运动学、摩擦/惯量辨识与速度环设计值
 * @version 0.1
 * @date 2026-11-25
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>

namespace thread::chassis
{

constexpr uint8_t kMotorCount  = 4;         // 全向轮数量
constexpr float   kWheelRadius = 0.285f;    // 轮半径 m（570 mm 直径）

inline float chassis_v = 5.0f;              // 目标 v 比例（遥控归一化 [-1,1] 乘以该值）
inline float chassis_w = 10.0f;             // 小陀螺目标旋转速度 rad/s

// 摩擦前馈（辨识结果，单位 A）：I = a·tanh(ω/ωs) + b·ω
constexpr float kFricA[kMotorCount]        = { 0.33983f, 0.27259f, 0.53885f, 0.49975f };  // 正转库仑
constexpr float kFricB[kMotorCount]        = { 0.01966f, 0.02159f, 0.04012f, 0.02587f };  // 正转黏性
constexpr float kFricANeg[kMotorCount]     = { 0.31649f, 0.28193f, 0.47052f, 0.50784f };  // 反转库仑
constexpr float kFricBNeg[kMotorCount]     = { 0.02408f, 0.02293f, 0.04561f, 0.02563f };  // 反转黏性

// 静摩擦（起转电流，低速时取代库仑值，随转速衰减）
constexpr float kFricStatic[kMotorCount]    = { 0.65f, 0.63f, 0.84f, 0.74f };  // 正转起转
constexpr float kFricStaticNeg[kMotorCount] = { 0.44f, 0.53f, 1.15f, 0.68f };  // 反转起转

constexpr float kFricSmooth        = 0.5f;   // 静摩擦随实际转速衰减的宽度 rad/s
constexpr float kFricStaticSmooth  = 0.05f;  // 方向项平滑宽度 rad/s，几乎阶跃

// 等效惯量（辨识结果，单位 A/(rad/s^2)）
constexpr float kJEq[kMotorCount] = { 0.01816f, 0.01951f, 0.01997f, 0.01881f };

// 速度 PI 设计目标：J*s^2+(b+Kp)*s+Ki = J*(s+wn)^2，b 取正反向均值
constexpr float kSpeedLoopWn        = 10.0f;   // 自然角频率 rad/s
constexpr float kSpeedLoopZeta      = 1.0f;    // 名义线性模型临界阻尼
constexpr float kControlCurrentMax  = 5.0f;    // PI 与前馈相加后的总电流限幅 A
constexpr float kIntegralCurrentMax = 2.0f;    // 积分补偿限幅 A
constexpr float kAccelLimit         = 200.0f;  // 加速度前馈限幅 rad/s^2，抑制阶跃尖峰

// C620 电流原始值换算：±20 A ↔ ±16384
constexpr float kCurrentRawPerAmp = 819.2f;
constexpr float kCurrentRawLimit  = 16384.0f;

} // namespace thread::chassis
