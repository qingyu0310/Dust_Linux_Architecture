/**
 * @file armor_tracker.cpp
 * @author qingyu
 * @brief 单辆车跨帧状态估计实现 — 过程模型、像素四点观测与四面推导
 * @version 0.1
 * @date 2026-10-17
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "framework/algorithm/vision/track/armor_tracker.hpp"

#include "framework/algorithm/math/angle.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <opencv2/calib3d.hpp>

namespace algorithm::vision
{

namespace
{

/**
 * @brief 数值雅可比的扰动步长
 *
 * 位置类状态用 1e-4 m、角度用 1e-4 rad：与参考 `target.cpp` 的中心差分同量级，
 * 小到不引入可观的非线性误差，大到不会让差分被双精度舍入吃掉。
 */
constexpr double kPositionStepM = 1.0e-4;
constexpr double kAngleStepRad  = 1.0e-4;

} // namespace

/**
 * @brief 建立跟踪器，状态清零直到第一次 Update
 */
ArmorTracker::ArmorTracker(const ArmorTrackerConfig& config) : config_(config)
{
    State state = State::Zero();
    state[kRadius] = config_.radius_init_m;

    // 角度分量在每步加法后包回 (−π, π]：状态里有转角，平移与修正都可能让它越过 ±π。
    // 这个钩子挂在滤波器内部后，预测与修正两条路径都不会漏掉归一化。
    filter_.set_add([](const State& first, const State& second)
    {
        State sum = first + second;
        sum[kAngle] = algorithm::math::normalize_radian_pm_pi(sum[kAngle]);
        return sum;
    });

    filter_.set_nis_threshold(config_.nis_threshold);
    filter_.set_state(state, StateMat::Identity());
}

/**
 * @brief 由状态推出某个面的转角
 */
double ArmorTracker::FaceAngle(const State& state, std::int32_t face) const
{
    const double step = 2.0 * M_PI / static_cast<double>(config_.armor_num);
    return algorithm::math::normalize_radian_pm_pi(state[kAngle] + static_cast<double>(face) * step);
}

/**
 * @brief 某个面的中心位置，在云台系下
 *
 * 车的旋转绕云台系竖直轴（y 向下，水平面是 x-z）：面心 = 旋转中心 − r·法向，法向
 * `n = (cos a, 0, sin a)`（与 ArmorRotation 同一约定）。高度差沿 y。
 * 只有四面车的面 1、3 用第二半径与高度差（照参考 `target.cpp` 的 `h_armor_xyz`）。
 */
Eigen::Vector3d ArmorTracker::ArmorCenter(const State& state, std::int32_t face) const
{
    const double angle = FaceAngle(state, face);
    const bool use_second = config_.armor_num == 4 && (face == 1 || face == 3);

    const double radius = use_second ? state[kRadius] + state[kRadiusGap] : state[kRadius];

    return Eigen::Vector3d(state[kCenterX] - radius * std::cos(angle), state[kCenterY] + (use_second ? state[kHeightGap] : 0.0), state[kCenterZ] - radius * std::sin(angle));
}

/**
 * @brief 状态到像素四点的观测模型
 *
 * 用位姿把板系物点反投回像素。物点由板尺寸决定，所以观测函数需要 size 与 dimensions；
 * 二者是常量参数，不参与状态，也不进雅可比。状态在云台系，投影要先把面心换到相机系；
 * 外参默认单位阵时这一步是恒等。
 *
 * `face` 决定的是"这一帧看到的是车上哪一面"：四面车的面 1、3 用第二半径与高度差，
 * 所以同一个状态按不同的面投出来的四点不同——上层正是靠这一点分辨面序。
 */
ArmorTracker::Measure ArmorTracker::Observe(const State& state, ArmorSize size, const ArmorDimensions& dimensions, const CameraModel& camera, std::int32_t face) const
{
    Measure projected;
    projected.setConstant(std::numeric_limits<double>::quiet_NaN());

    const std::optional<std::array<cv::Point3f, 4>> object_points = MakeObjectPoints(size, dimensions);
    if (!object_points)
    {
        return projected;
    }

    const double angle = FaceAngle(state, face);
    // 状态在云台系：位置与姿态都换到相机系后再投影
    const Eigen::Vector3d center = GimbalToCamera(ArmorCenter(state, face), config_.extrinsics);
    const cv::Vec3d gimbal_rvec = ArmorRotation(angle, config_.armor_pitch_rad);

    const cv::Vec3d tvec(center.x(), center.y(), center.z());
    const cv::Vec3d rvec = GimbalToCameraRotation(gimbal_rvec, config_.extrinsics);

    const std::optional<std::array<cv::Point2f, 4>> points = ProjectPose(*object_points, rvec, tvec, camera);
    if (!points) {
        return projected;
    }

    for (std::size_t index = 0; index < points->size(); ++index)
    {
        projected[static_cast<Eigen::Index>(2 * index)]     = (*points)[index].x;
        projected[static_cast<Eigen::Index>(2 * index + 1)] = (*points)[index].y;
    }
    return projected;
}

/**
 * @brief 把状态按指定面投成像素四点，不修改状态
 */
std::optional<std::array<cv::Point2f, 4>> ArmorTracker::ProjectFace(ArmorSize size, const ArmorDimensions& dimensions, const CameraModel& camera, std::int32_t face) const
{
    if (!initialized_ || face < 0 || face >= config_.armor_num)
    {
        return std::nullopt;
    }

    const std::optional<std::array<cv::Point3f, 4>> object_points = MakeObjectPoints(size, dimensions);
    if (!object_points)
    {
        return std::nullopt;
    }

    const State& state = filter_.state();
    const double angle = FaceAngle(state, face);
    const Eigen::Vector3d center = GimbalToCamera(ArmorCenter(state, face), config_.extrinsics);
    const cv::Vec3d rvec = GimbalToCameraRotation(ArmorRotation(angle, config_.armor_pitch_rad), config_.extrinsics);

    return ProjectPose(*object_points, rvec, cv::Vec3d(center.x(), center.y(), center.z()), camera);
}

/**
 * @brief 用一帧位姿把状态初始化到该帧
 *
 * PnP 解出的是相机系位姿，先按外参换到云台系（单位阵时是恒等）；旋转中心由面心减去
 * r·法向得到，与观测模型里 `面心 = 旋转中心 + r·法向` 互为逆运算，所以初始帧残差为 0。
 *
 * @note 云台系下法向由相机系法向经方向变换得到，再取方位角。外参为单位阵时与直接用
 *       相机系法向等价。
 */
void ArmorTracker::Initialize(const std::array<cv::Point2f, 4>& corners, ArmorSize size, const ArmorDimensions& dimensions, const CameraModel& camera)
{
    const std::optional<std::array<cv::Point3f, 4>> object_points = MakeObjectPoints(size, dimensions);
    if (!object_points) {
        return;
    }

    const std::optional<ArmorPose> pose = SolvePose(corners, *object_points, camera);
    if (!pose) {
        return;
    }

    cv::Matx33d rotation;
    cv::Rodrigues(cv::Mat(pose->rvec), rotation);
    // 相机系下法向是旋转矩阵第一列，换到云台系后再取水平面内的方位角
    const Eigen::Vector3d camera_normal(rotation(0, 0), rotation(1, 0), rotation(2, 0));
    const Eigen::Vector3d gimbal_normal = CameraToGimbalDirection(camera_normal, config_.extrinsics);
    const double yaw = std::atan2(gimbal_normal.z(), gimbal_normal.x());

    const double radius = config_.radius_init_m;
    const Eigen::Vector3d normal(-std::cos(yaw), 0.0, -std::sin(yaw));
    const Eigen::Vector3d face_center = CameraToGimbal(Eigen::Vector3d(pose->tvec[0], pose->tvec[1], pose->tvec[2]), config_.extrinsics);

    State state = State::Zero();
    const Eigen::Vector3d center = face_center - radius * normal;
    state[kCenterX] = center.x();
    state[kCenterY] = center.y();
    state[kCenterZ] = center.z();
    state[kAngle]   = yaw;
    state[kRadius]  = radius;

    // 初值不确定度：位置 ±0.1 m、角度 ±0.3 rad、半径 ±0.2 m；速度未知但量级小，
    // 给 0.01（std 0.1 m/s）——给太大（如 1.0）会让滤波器把弧线运动解释成平移而不修半径
    StateMat covariance = StateMat::Zero();
    covariance.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.09, 0.25, 0.04, 0.01, 0.01;

    // 旋转中心由面心减去 r·法向算出，所以 r 的初值误差必然带着中心一起偏：
    // ∂center/∂r = n（水平面方向）。把这条相关性写进协方差，滤波器不用再从观测里
    // 慢慢学它——漏掉它正是半径收敛慢的主因。
    constexpr double kRadiusVariance = 0.04;
    covariance(kCenterX, kRadius) = -normal.x() * kRadiusVariance;
    covariance(kCenterZ, kRadius) = -normal.z() * kRadiusVariance;
    covariance(kRadius, kCenterX) = covariance(kCenterX, kRadius);
    covariance(kRadius, kCenterZ) = covariance(kCenterZ, kRadius);

    filter_.set_state(state, covariance);

    initialized_ = true;
    update_count_ = 0;
}

/**
 * @brief 把半径与第二半径钳回物理范围
 */
void ArmorTracker::ApplyLimits()
{
    if (config_.armor_num != 4)
    {
        return;
    }

    State state = filter_.state();
    state[kRadius]      = std::clamp(state[kRadius], config_.radius_min_m, config_.radius_max_m);
    const double second = std::clamp(state[kRadius] + state[kRadiusGap], config_.radius_min_m, config_.radius_max_m);
    state[kRadiusGap]   = second - state[kRadius];

    filter_.set_state(state, filter_.covariance());
}

/**
 * @brief 只按 dt 推进状态，不修正
 *
 * 过程模型是位置匀加速 + 转角匀速；`r/l/h` 只加一点噪声防止协方差长期崩塌（照参考）。
 * 角度在传播后包回 (−π, π]：本工程的 EKF 没有角度归一化，只能在这里手动补。
 */
void ArmorTracker::Predict(double dt_s)
{
    if (!initialized_)
    {
        return;
    }

    const double dt = dt_s;

    StateMat transition = StateMat::Identity();
    transition(kCenterX, kVelocityX) = dt;
    transition(kCenterY, kVelocityY) = dt;
    transition(kCenterZ, kVelocityZ) = dt;
    transition(kAngle,   kOmega)     = dt;

    // 逐段白噪声：位置与角度的匀加速模型
    const double a = dt * dt * dt * dt / 4.0;
    const double b = dt * dt * dt / 2.0;
    const double c = dt * dt;

    StateMat noise = StateMat::Zero();
    const double v1 = config_.accel_variance;
    const double v2 = config_.angular_accel_variance;
    noise(kCenterX,   kCenterX) = a * v1; noise(kCenterX,   kVelocityX) = b * v1;
    noise(kVelocityX, kCenterX) = b * v1; noise(kVelocityX, kVelocityX) = c * v1;
    noise(kCenterY,   kCenterY) = a * v1; noise(kCenterY,   kVelocityY) = b * v1;
    noise(kVelocityY, kCenterY) = b * v1; noise(kVelocityY, kVelocityY) = c * v1;
    noise(kCenterZ,   kCenterZ) = a * v1; noise(kCenterZ,   kVelocityZ) = b * v1;
    noise(kVelocityZ, kCenterZ) = b * v1; noise(kVelocityZ, kVelocityZ) = c * v1;
    noise(kAngle,     kAngle)   = a * v2; noise(kAngle,     kOmega)     = b * v2;
    noise(kOmega,     kAngle)   = b * v2; noise(kOmega,     kOmega)     = c * v2;

    // 形状量只给很小的过程噪声，否则半径会被噪声推着乱走
    noise(kRadius,    kRadius)    = config_.shape_variance * c;
    noise(kRadiusGap, kRadiusGap) = config_.shape_variance * c;
    noise(kHeightGap, kHeightGap) = config_.shape_variance * c;

    // Q 必须在 predict 之前写入：predict 内部会读 q_
    filter_.set_process_noise(noise);

    // 角度归一化由滤波器的加法钩子负责，这里不再手动包回 ±π
    (void)filter_.predict([dt](const State& x)
    {
        State next = x;
        next[kCenterX] += x[kVelocityX] * dt;
        next[kCenterY] += x[kVelocityY] * dt;
        next[kCenterZ] += x[kVelocityZ] * dt;
        next[kAngle]   += x[kOmega]     * dt;
        return next;
    }, transition);
}

/**
 * @brief 用一帧的四点观测推进并修正状态
 */
bool ArmorTracker::Update(const std::array<cv::Point2f, 4>& corners, ArmorSize size, const ArmorDimensions& dimensions, const CameraModel& camera, double dt_s, std::int32_t face)
{
    if (face < 0 || face >= config_.armor_num) {
        return false;
    }

    const std::optional<std::array<cv::Point3f, 4>> object_points = MakeObjectPoints(size, dimensions);
    if (!object_points) {
        return false;
    }

    for (const cv::Point2f& point : corners)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
            return false;
        }
    }

    if (!initialized_)
    {
        // 初始化内部会解一次 PnP；解不出来说明这帧观测本身不可用
        Initialize(corners, size, dimensions, camera);
        if (!initialized_) {
            return false;
        }
        ++update_count_;

        return true;
    }

    Predict(dt_s);

    Measure measured;
    for (std::size_t index = 0; index < corners.size(); ++index)
    {
        measured[static_cast<Eigen::Index>(2 * index)]     = corners[index].x;
        measured[static_cast<Eigen::Index>(2 * index + 1)] = corners[index].y;
    }

    // 观测雅可比用中心差分数值求（照参考 `target.cpp` 的做法），省去手推解析式
    const State x = filter_.state();
    Jacobian jacobian = Jacobian::Zero();
    constexpr std::array<std::int32_t, 7> kObservedStates = {kCenterX, kCenterY, kCenterZ, kAngle, kRadius, kRadiusGap, kHeightGap};

    for (const std::int32_t index : kObservedStates)
    {
        const double step = (index == kAngle) ? kAngleStepRad : kPositionStepM;
        State plus    = x;
        State minus   = x;
        plus[index]  += step;
        minus[index] -= step;

        if (index == kAngle)
        {
            plus[index]  = algorithm::math::normalize_radian_pm_pi(plus[index]);
            minus[index] = algorithm::math::normalize_radian_pm_pi(minus[index]);
        }

        const Measure forward  = Observe(plus,  size, dimensions, camera, face);
        const Measure backward = Observe(minus, size, dimensions, camera, face);
        if (!forward.allFinite() || !backward.allFinite())
        {
            return false;
        }
        jacobian.col(index) = (forward - backward) / (2.0 * step);
    }

    // 观测噪声随距离放大：同样的像素噪声，目标越远对应的三维误差越大（σ_d ∝ d²）。
    // 参考 `target.cpp` 的 R 里也用 log 项随距离调权重；这里取更直接的平方缩放，
    // 系数为 0 时退化成常数噪声。
    const double face_distance = ArmorCenter(filter_.state(), face).norm();
    const double distance_factor = 1.0 + config_.sigma_distance_scale * face_distance * face_distance;
    const double sigma_px = config_.point_sigma_px * distance_factor;

    Filter::MeasureMatrix measurement_noise = Filter::MeasureMatrix::Zero();
    measurement_noise.diagonal().setConstant(sigma_px * sigma_px);
    filter_.set_measurement_noise(measurement_noise);

    (void)filter_.correct(measured, [this, size, &dimensions, &camera, face](const State& state)
    {
        return Observe(state, size, dimensions, camera, face);
    }, jacobian);

    ApplyLimits();

    if (!filter_.state().allFinite() || !filter_.covariance().allFinite())
    {
        initialized_ = false;
        return false;
    }

    ++update_count_;
    return true;
}

/**
 * @brief 四面的中心与转角
 */
std::vector<ArmorState> ArmorTracker::Armors() const
{
    std::vector<ArmorState> armors;
    armors.reserve(static_cast<std::size_t>(config_.armor_num));

    const State& state = filter_.state();
    for (std::int32_t face = 0; face < config_.armor_num; ++face)
    {
        ArmorState armor;
        armor.xyz     = ArmorCenter(state, face);
        armor.yaw_rad = FaceAngle(state, face);
        armors.push_back(armor);
    }
    return armors;
}

/**
 * @brief 是否已经用一帧观测初始化过状态
 */
bool ArmorTracker::Active() const
{
    return initialized_;
}

/**
 * @brief 是否已收敛
 */
bool ArmorTracker::Converged() const
{
    return initialized_ && update_count_ >= config_.converge_updates && !Diverged();
}

/**
 * @brief 半径或第二半径是否已跑出物理范围
 */
bool ArmorTracker::Diverged() const
{
    const State& state = filter_.state();
    const double radius = state[kRadius];
    const double second = state[kRadius] + state[kRadiusGap];

    return radius < config_.radius_min_m || radius > config_.radius_max_m || second < config_.radius_min_m || second > config_.radius_max_m;
}

/**
 * @brief 已完成的观测更新次数
 */
std::int32_t ArmorTracker::UpdateCount() const
{
    return update_count_;
}

/**
 * @brief 当前主半径估计
 */
double ArmorTracker::Radius() const
{
    return filter_.state()[kRadius];
}

/**
 * @brief 当前第二半径差估计
 */
double ArmorTracker::RadiusGap() const
{
    return filter_.state()[kRadiusGap];
}

/**
 * @brief 全部更新里 NIS 超阈值的比例
 */
double ArmorTracker::NisFailureRate() const
{
    return filter_.nis_failure_rate();
}

/**
 * @brief 当前面的转角估计
 */
double ArmorTracker::Angle() const
{
    return filter_.state()[kAngle];
}

/**
 * @brief 角速度估计
 */
double ArmorTracker::Omega() const
{
    return filter_.state()[kOmega];
}

} // namespace algorithm::vision
