/**
 * @file armor_tracker.hpp
 * @author qingyu
 * @brief 单辆车的跨帧状态估计 — 旋转中心、转角、半径与四面推导
 * @version 0.1
 * @date 2026-10-17
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "framework/algorithm/filter/kalman/ekf.hpp"
#include "framework/algorithm/vision/core/armor_types.hpp"
#include "framework/algorithm/vision/pose/armor_pose.hpp"

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <opencv2/core.hpp>
#include <vector>

namespace algorithm::vision
{

/**
 * @brief 一个面的状态：中心位置与法向转角
 *
 * 四面共用一套旋转中心与半径，每个面只是转角相差 2π/armor_num。
 */
struct ArmorState
{
    Eigen::Vector3d xyz;                        // 装甲板中心，相机系，单位 m
    double yaw_rad {0.0};                       // 该面法向绕竖直轴的转角，单位 rad
};

/**
 * @brief 跟踪器参数
 *
 * 过程噪声与点噪声都取自参考的量级（`target.cpp` 的 v1/v2、`standard3.yaml` 的 8 px），
 * 但**必须按本机数据重定**：那是另一台相机、另一个帧率下标出来的。
 */
struct ArmorTrackerConfig
{
    int32_t armor_num {4};                      // 整车面数；四面车为 4
    double  radius_init_m {0.2};                // 半径初值，只在初始化时用
    double  radius_min_m {0.05};                // 半径物理下限，单位 m
    double  radius_max_m {0.50};                // 半径物理上限，单位 m
    double  armor_pitch_rad {0.0};              // 装甲板相对竖直面的倾角，板子正立为 0
    double  accel_variance {10.0};              // 位置加速度方差，量级取自参考 v1
    double  angular_accel_variance {100.0};     // 角加速度方差，量级取自参考 v2
    double  shape_variance {0.1};               // r/l/h 的过程噪声，防止协方差崩塌
    double  point_sigma_px {8.0};               // 四点观测噪声基准，单位 pixel（按本机实测重定）
    double  sigma_distance_scale {0.0};         // 观测噪声随距离平方增长的系数，单位 1/m²；0 表示不缩放
    double  nis_threshold {0.0};                // NIS 卡方阈值；0 表示不判（按本工程 8 维观测重定）
    int32_t converge_updates {4};               // 收敛所需的最少更新次数
    GimbalExtrinsics extrinsics {};             // 相机相对云台的外参；默认单位阵时工作系即相机系
};

/**
 * @brief 单辆车的跨帧跟踪器
 *
 * 状态 11 维、全在**云台系**：`[cx, vx, cy, vy, cz, vz, a, w, r, l, h]`，其中 `a` 是当前面
 * 的转角、`w` 角速度、`r` 主半径、`l` 第二半径差、`h` 高度差。云台系与相机系由
 * `ArmorTrackerConfig::extrinsics` 联系：外参默认单位阵时两者重合，跟踪器实际就在相机系里
 * 工作；标定出外参后状态自动变成云台系量，算法一行不用改。
 *
 * 观测是**像素四点**而不是 yaw/pitch：本工程的 EKF 没有角度归一化与自定义减法，把观测留在
 * 像素域就绕开了这一点。内部流程是"云台系状态 → 相机系位姿 → 像素四点"，与实测四点求残差。
 *
 * 未收敛时 `Armors()` 也会返回四面，但只有当前可见那一面有实际观测支撑；调用方应先查
 * `Converged()` 再决定画不画另外三面（未收敛时外推误差可达画面宽的 1/3，见规划 §2）。
 */
class ArmorTracker final
{
public:
    explicit ArmorTracker(const ArmorTrackerConfig& config);

    /**
     * @brief 用一帧的四点观测推进并修正状态
     *
     * 首次调用会把状态初始化到这一帧的位姿上（半径取配置初值），之后的调用都是
     * 先按 dt 预测、再修正。
     *
     * @param corners 图像四点，顺序 LT、RT、RB、LB
     * @param size 装甲板大小，决定物点宽度
     * @param dimensions 实物尺寸，单位 m
     * @param camera 内参与畸变
     * @param dt_s 距上一次更新的时间，单位 s；首次调用忽略
     * @param face 这一帧看到的是第几面；四面车的面 1、3 用第二半径与高度差
     * @return true 状态可用；false 观测无效、面序越界或修正后状态发散
     */
    bool Update(const std::array<cv::Point2f, 4>& corners, ArmorSize size, const ArmorDimensions& dimensions, const CameraModel& camera, double dt_s, std::int32_t face);

    /**
     * @brief 只按 dt 推进状态，不修正
     *
     * 帧间没有匹配到观测时用，让状态和协方差随过程噪声自然扩散。
     */
    void Predict(double dt_s);

    /**
     * @brief 把状态按指定面投成像素四点，不修改状态
     *
     * 供上层做面序判定与配对代价：对每个面各投一次，与观测四点比重投影残差，取最小者。
     * 与 `Update` 用的是同一套观测模型，所以"代价最小的面"就是"更新时该用的面"。
     *
     * @param size 装甲板大小
     * @param dimensions 实物尺寸
     * @param camera 内参与畸变
     * @param face 面序，须在 [0, armor_num) 内
     * @return 四点像素坐标；未初始化、面序越界或投影失败时返回 std::nullopt
     */
    std::optional<std::array<cv::Point2f, 4>> ProjectFace(ArmorSize size, const ArmorDimensions& dimensions, const CameraModel& camera, std::int32_t face) const;

    /**
     * @brief 四面的中心与转角，按面序返回
     */
    std::vector<ArmorState> Armors() const;

    /**
     * @brief 是否已收敛：更新次数够且半径没跑出物理范围
     */
    bool Converged() const;

    /**
     * @brief 是否已经用一帧观测初始化过状态
     */
    bool Active() const;

    /**
     * @brief 半径或第二半径是否已跑出物理范围
     */
    bool Diverged() const;

    /**
     * @brief 已完成的观测更新次数
     */
    std::int32_t UpdateCount() const;

    /**
     * @brief 当前面的转角估计，单位 rad
     */
    double Angle() const;

    /**
     * @brief 角速度估计，单位 rad/s
     */
    double Omega() const;

    /**
     * @brief 当前主半径估计，单位 m
     */
    double Radius() const;

    /**
     * @brief 当前第二半径差估计（面 1、3 相对面 0、2 的半径差），单位 m
     */
    double RadiusGap() const;

    /**
     * @brief 全部更新里 NIS 超阈值的比例
     *
     * 用来判"跟丢了 / 收敛到坏值"：残差持续偏大但半径还没跑出物理范围时，`Diverged()`
     * 看不出来，这个数看得出来。
     *
     * @return [0, 1]；尚未更新时返回 0
     */
    double NisFailureRate() const;

private:
    // 状态下标：顺序即状态向量的定义，改这里必须同步改过程和观测模型
    static constexpr std::int32_t kCenterX      = 0;
    static constexpr std::int32_t kVelocityX    = 1;
    static constexpr std::int32_t kCenterY      = 2;
    static constexpr std::int32_t kVelocityY    = 3;
    static constexpr std::int32_t kCenterZ      = 4;
    static constexpr std::int32_t kVelocityZ    = 5;
    static constexpr std::int32_t kAngle        = 6;
    static constexpr std::int32_t kOmega        = 7;
    static constexpr std::int32_t kRadius       = 8;
    static constexpr std::int32_t kRadiusGap    = 9;
    static constexpr std::int32_t kHeightGap    = 10;

    using Filter    = algorithm::filter::ExtendedKalmanFilter<11, 8>;
    using State     = Filter::State;
    using StateMat  = Filter::StateMatrix;
    using Measure   = Filter::Measurement;
    using Jacobian  = Filter::ObserveJacobian;

    /**
     * @brief 某个面的中心位置，即观测模型里的位置部分
     *
     * @param state 用于取值的那份状态，不一定是滤波器当前状态：数值雅可比要对扰动后的
     *              状态求观测，必须把 plus/minus 传进来，否则雅可比全零、状态不会被修正
     * @param face 面序
     */
    Eigen::Vector3d ArmorCenter(const State& state, std::int32_t face) const;

    /**
     * @brief 状态到像素四点的观测模型
     *
     * @param state 用于取值的那份状态，不一定是滤波器当前状态：数值雅可比要传入扰动状态
     * @param size 装甲板大小
     * @param dimensions 实物尺寸
     * @param camera 内参与畸变
     * @param face 这一帧看到的是第几面
     */
    Measure Observe(const State& state, ArmorSize size, const ArmorDimensions& dimensions, const CameraModel& camera, std::int32_t face) const;

    /**
     * @brief 由状态推出某个面的转角
     */
    double FaceAngle(const State& state, std::int32_t face) const;

    /**
     * @brief 用一帧位姿把状态初始化到该帧
     */
    void Initialize(const std::array<cv::Point2f, 4>& corners, ArmorSize size, const ArmorDimensions& dimensions, const CameraModel& camera);

    /**
     * @brief 把半径与第二半径钳回物理范围
     */
    void ApplyLimits();

    ArmorTrackerConfig config_;
    Filter             filter_;
    bool               initialized_ {false};
    std::int32_t       update_count_ {0};
};

} // namespace algorithm::vision
