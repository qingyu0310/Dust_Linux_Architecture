/**
 * @file multi_target.cpp
 * @author qingyu
 * @brief 多目标实现 — 编号路由、面序匹配与后备门
 * @version 0.1
 * @date 2026-10-17
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "framework/algorithm/vision/track/multi_target.hpp"

#include "framework/algorithm/math/angle.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <utility>

namespace algorithm::vision
{

namespace
{

/**
 * @brief 观测位姿给出的面序判据：板法向在水平面内的转角
 *
 * 参考 `target.cpp` 的面序匹配用的是角度差而不是"把预测投成像素再比"：四个面在圆上只差
 * 90°，投影到画面后彼此很接近，加上半径还没收敛时预测本身偏差大，像素残差的 argmin 会
 * 恒选同一个面。角度直接从观测的 PnP 解里取，不受半径误差影响。
 */
struct ObservationAngles
{
    double normal_yaw {0.0};     // 板法向在水平面内的转角，单位 rad
};

/**
 * @brief 由观测四点解出位姿，取法向转角
 *
 * @return 角度；PnP 失败时返回 std::nullopt
 */
std::optional<ObservationAngles> MeasureAngles(const std::array<cv::Point2f, 4>& corners, ArmorSize size, const ArmorDimensions& dimensions, const CameraModel& camera)
{
    const std::optional<std::array<cv::Point3f, 4>> object_points = MakeObjectPoints(size, dimensions);
    if (!object_points)
    {
        return std::nullopt;
    }

    const std::optional<ArmorPose> pose = SolvePose(corners, *object_points, camera);
    if (!pose)
    {
        return std::nullopt;
    }

    cv::Matx33d rotation;
    cv::Rodrigues(cv::Mat(pose->rvec), rotation);

    // 板系 x 轴是法向，它在水平面（相机系 x-z）内的方位角，与预测侧同一套定义
    ObservationAngles angles;
    angles.normal_yaw = std::atan2(rotation(2, 0), rotation(0, 0));

    return angles;
}

/**
 * @brief 两个角的最短差，落在 [0, π]
 */
double AngularDistance(double first, double second)
{
    return std::abs(algorithm::math::normalize_radian_pm_pi(first - second));
}

} // namespace

/**
 * @brief 把编号换算成槽序
 *
 * 槽表里的 `ArmorLabel::Any` 匹配任意编号，用于"只跟一辆车、但不限定是几号"的场景；
 * 表里只要有一个 `Any`，它就是唯一的兜底槽，其余具体编号仍优先精确匹配。
 */
std::int32_t MultiTargetTracker::SlotOf(ArmorLabel label)
{
    for (std::int32_t slot = 0; slot < kMaxTargets; ++slot)
    {
        if (kTargetLabels[static_cast<std::size_t>(slot)] == label)
        {
            return slot;
        }
    }

    for (std::int32_t slot = 0; slot < kMaxTargets; ++slot)
    {
        if (kTargetLabels[static_cast<std::size_t>(slot)] == ArmorLabel::Any)
        {
            return slot;
        }
    }

    return -1;
}

/**
 * @brief 按角度差定出这一帧看到的是第几面
 *
 * 判据照参考 `target.cpp` 的 `angle_error`：观测与预测的**法向转角差**加上**板心方位角差**，
 * 取最小者。不用"把预测投成像素再比重投影"——四个面在圆上只差 90°，投影后彼此很接近，
 * 半径未收敛时预测本身偏差又大，像素残差的 argmin 会恒选同一个面（实测 `f=0` 恒定、
 * 半径被推到钳制边界）。
 *
 * @return 面序与角度差之和（rad）；该槽未初始化或观测解不出位姿时返回 std::nullopt
 */
std::optional<std::pair<std::int32_t, double>> MultiTargetTracker::MatchFace(std::int32_t slot, ArmorSize size, const std::array<cv::Point2f, 4>& corners, const ArmorDimensions& dimensions, const CameraModel& camera) const
{
    const ArmorTracker& tracker = trackers_[static_cast<std::size_t>(slot)];
    if (!tracker.Active())
    {
        return std::nullopt;
    }

    const std::optional<ObservationAngles> observed = MeasureAngles(corners, size, dimensions, camera);
    if (!observed)
    {
        return std::nullopt;
    }

    const std::vector<ArmorState> predicted = tracker.Armors();
    if (predicted.empty())
    {
        return std::nullopt;
    }

    std::int32_t best_face = -1;
    double best_error = std::numeric_limits<double>::max();

    for (std::int32_t face = 0; face < config_.tracker.armor_num; ++face)
    {
        const ArmorState& state = predicted[static_cast<std::size_t>(face)];

        // 只比法向转角：四个面的法向相差 90°，本身就能唯一确定面序；位置方位角在斜视角下
        // 四个面彼此只差几度，加进来只会把判定推偏
        const double error = AngularDistance(observed->normal_yaw, state.yaw_rad);
        if (error < best_error)
        {
            best_error = error;
            best_face = face;
        }
    }

    if (best_face < 0)
    {
        return std::nullopt;
    }
    return std::make_pair(best_face, best_error);
}

/**
 * @brief 用一帧检测推进所有槽
 *
 * 分派规则：编号不在 `kTargetLabels` 里的检测直接忽略；同号多检出时本帧拒绝更新
 * （记 `duplicates`），不做"取第一个"。
 *
 * 预测由跟踪器自己负责，这里不统一 `Predict`：有检测的槽走 `Update`（内部自己预测再修正），
 * 没检测的槽才由这里调 `Predict` 让转角继续随 `w` 走——两条路径不能同时走，否则一帧会推进
 * 两个 `dt`，角度按两倍速前进，滤波器被迫压低转速与半径去凑。
 */
void MultiTargetTracker::Update(const std::vector<DetectedArmor>& armors, const ArmorDimensions& dimensions, const CameraModel& camera, double dt_s)
{
    // 把检测按编号分派到槽；同号多检出记为重复
    struct Assignment
    {
        const DetectedArmor* armor {nullptr};
        bool duplicate {false};
    };
    std::array<Assignment, kMaxTargets> assignments {};

    for (const DetectedArmor& armor : armors)
    {
        const std::int32_t slot = SlotOf(armor.GetLabel());
        if (slot < 0)
        {
            continue; // 不是要跟的车（哨兵/基地/编号不在表里）
        }

        Assignment& assignment = assignments[static_cast<std::size_t>(slot)];
        if (assignment.armor != nullptr)
        {
            assignment.duplicate = true; // 同号出现第二个检测：本帧拒绝更新
            continue;
        }
        assignment.armor = &armor;
    }

    // 逐槽处理：没检测的只推进，有检测的定面序 → 过后备门 → 更新
    for (std::int32_t slot = 0; slot < kMaxTargets; ++slot)
    {
        TargetReport& report  = reports_ [static_cast<std::size_t>(slot)];
        ArmorTracker& tracker = trackers_[static_cast<std::size_t>(slot)];
        const Assignment& assignment = assignments[static_cast<std::size_t>(slot)];

        if (assignment.duplicate)
        {
            ++report.duplicates;
        }
        if (assignment.armor == nullptr)
        {
            // 本帧没有该编号的检测：只推进，转角靠 w 继续走，回来时按预测方位接上
            if (tracker.Active())
            {
                tracker.Predict(dt_s);
                ++report.losses;
            }
            continue;
        }

        const DetectedArmor& armor = *assignment.armor;
        const std::array<cv::Point2f, 4> corners = armor.GetCorners();

        // 未初始化的槽：这一帧就是它的初值，不需要定面（初始化即把当前面命名为面 0）
        if (!tracker.Active())
        {
            if (!tracker.Update(corners, armor.GetSize(), dimensions, camera, dt_s, 0))
            {
                continue;
            }
            report.label     = armor.GetLabel();
            report.active    = true;
            report.updates   = tracker.UpdateCount();
            report.converged = tracker.Converged();
            continue;
        }

        // 已初始化的槽：按角度差定面，再过面序门
        const std::optional<std::pair<std::int32_t, double>> match = MatchFace(slot, armor.GetSize(), corners, dimensions, camera);
        if (!match || match->second > config_.face_gate_rad)
        {
            report.last_face = -1;
            report.last_cost = match ? match->second : -1.0;
            ++report.gate_rejects;
            continue;
        }

        if (!tracker.Update(corners, armor.GetSize(), dimensions, camera, dt_s, match->first))
        {
            continue;
        }
        report.label     = armor.GetLabel();
        report.updates   = tracker.UpdateCount();
        report.converged = tracker.Converged();
        report.last_face = match->first;
        report.last_cost = match->second;
    }
}

/**
 * @brief 第 s 个槽的四面的中心与转角
 */
std::vector<ArmorState> MultiTargetTracker::Armors(std::int32_t slot) const
{
    if (slot < 0 || slot >= kMaxTargets)
    {
        return {};
    }
    return trackers_[static_cast<std::size_t>(slot)].Armors();
}

/**
 * @brief 第 s 个槽是否已收敛
 */
bool MultiTargetTracker::Converged(std::int32_t slot) const
{
    if (slot < 0 || slot >= kMaxTargets)
    {
        return false;
    }
    return trackers_[static_cast<std::size_t>(slot)].Converged();
}

/**
 * @brief 第 s 个槽的主半径估计
 */
double MultiTargetTracker::Radius(std::int32_t slot) const
{
    if (slot < 0 || slot >= kMaxTargets)
    {
        return 0.0;
    }
    return trackers_[static_cast<std::size_t>(slot)].Radius();
}

/**
 * @brief 第 s 个槽是否已初始化过
 */
bool MultiTargetTracker::Active(std::int32_t slot) const
{
    if (slot < 0 || slot >= kMaxTargets)
    {
        return false;
    }
    return trackers_[static_cast<std::size_t>(slot)].Active();
}

/**
 * @brief 各槽的运行统计
 */
std::array<TargetReport, kMaxTargets> MultiTargetTracker::Reports() const
{
    return reports_;
}

} // namespace algorithm::vision
