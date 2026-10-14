/**
 * @file armor_board.cpp
 * @author qingyu
 * @brief 装甲板纯几何配对器的实现
 * @version 0.1
 * @date 2026-10-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "framework/algorithm/vision/core/armor_board.hpp"

#include "framework/algorithm/vision/core/geometry_util.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace algorithm::vision
{

namespace
{

using detail::IsFinite;
using detail::Distance;
using detail::SignedArea;
using detail::IsValidQuadrilateral;

/**
 * @brief 将浮点值限制到 [0, 1]
 */
float ClampUnit(float value)
{
    return std::clamp(value, 0.0f, 1.0f);
}

/**
 * @brief 按无向长轴的 pi 周期计算最小方向差
 */
float AngleDifference(float first, float second)
{
    constexpr float pi = static_cast<float>(CV_PI);
    return std::abs(std::remainder(first - second, pi));
}

/**
 * @brief 计算灯条长轴与两灯条中心连线偏离垂直关系的角度
 */
float RectangularError(const LightBar& light, const cv::Point2f& center_line)
{
    const cv::Point2f light_axis    = (light.GetBottom() - light.GetTop()) / light.GetLength();
    const float       center_length = std::sqrt(center_line.dot(center_line));
    const cv::Point2f center_axis   = center_line / center_length;

    return std::asin(std::clamp(std::abs(light_axis.dot(center_axis)), 0.0f, 1.0f));
}

/**
 * @brief 候选排序规则：高分优先，同分时用灯条 ID 保证确定性
 */
bool CandidateBetter(const ArmorCandidate& first, const ArmorCandidate& second)
{
    if (first.GetGeometryScore() != second.GetGeometryScore())
    {
        return first.GetGeometryScore() > second.GetGeometryScore();
    }

    if (first.GetGeometryCost() != second.GetGeometryCost())
    {
        return first.GetGeometryCost() < second.GetGeometryCost();
    }

    if (first.GetLeftLight().GetId() != second.GetLeftLight().GetId())
    {
        return first.GetLeftLight().GetId() < second.GetLeftLight().GetId();
    }

    return first.GetRightLight().GetId() < second.GetRightLight().GetId();
}

} // namespace

/**
 * @brief 创建纯几何配对器
 *
 * 构造期一次校验配置，之后 Pair() 不再处理配置错误。
 *
 * @param config 灯条配对门限与评分权重
 * @throws std::invalid_argument 参数非有限、范围关系错误、权重为负或权重总和为 0
 */
ArmorGeometry::ArmorGeometry(const PairingConfig& config) : config_(config)
{
    config_.thresholds.Validate();
    config_.weights.Validate();
}

/**
 * @brief 尝试将两根灯条组成装甲板候选
 *
 * 依次过同 ID、颜色、左右顺序、长度差、方向差、高度差、间距、矩形关系与角点有效性；
 * 任一门限不过就返回对应拒绝原因，全部通过后按权重算归一化几何代价并给出尺寸分类。
 *
 * @param first 第一根灯条，输入顺序不限
 * @param second 第二根灯条，输入顺序不限
 * @return 成功时携带只读 ArmorCandidate，失败时携带明确拒绝原因
 */
PairResult ArmorGeometry::Pair(const LightBar& first, const LightBar& second) const
{
    if (first.GetId() == second.GetId()) {
        return PairResult(PairRejectReason::SameLightBar);
    }

    if (first.GetColor() == TeamColor::Unknown || first.GetColor() != second.GetColor()) {
        return PairResult(PairRejectReason::ColorMismatch);
    }

    const LightBar& left  = first.GetCenter().x < second.GetCenter().x ? first : second;
    const LightBar& right = first.GetCenter().x < second.GetCenter().x ? second : first;

    if (left.GetCenter().x >= right.GetCenter().x)
    {
        return PairResult(PairRejectReason::InvalidOrder);
    }

    const std::array<cv::Point2f, 4> corners{left.GetTop(), right.GetTop(), right.GetBottom(), left.GetBottom()};

    const cv::Point2f center_line = right.GetCenter() - left.GetCenter();

    const float average_length    = 0.5f * (left.GetLength() + right.GetLength());
    const float area              = SignedArea(corners);

    const float length_error      = std::abs(left.GetLength() - right.GetLength()) / average_length;
    const float rectangular_error = std::max(RectangularError(left, center_line), RectangularError(right, center_line));
    const float color_error       = std::abs(left.GetColorScore() - right.GetColorScore());
    const float angle_error       = AngleDifference(left.GetTilt(), right.GetTilt());

    const float y_offset_ratio    = std::abs(center_line.y) / average_length;
    const float spacing_ratio     = Distance(left.GetCenter(), right.GetCenter()) / average_length;

    const Thresholds& thresholds = config_.thresholds;
    const Weights&    weights    = config_.weights;

    if (length_error > thresholds.max_length_error)
    {
        return PairResult(PairRejectReason::LengthMismatch);
    }
    if (angle_error > thresholds.max_angle_diff_rad)
    {
        return PairResult(PairRejectReason::AngleMismatch);
    }
    if (y_offset_ratio > thresholds.max_y_offset_ratio)
    {
        return PairResult(PairRejectReason::YOffsetTooLarge);
    }
    if (spacing_ratio < thresholds.min_armor_spacing || spacing_ratio > thresholds.max_armor_spacing)
    {
        return PairResult(PairRejectReason::SpacingOutOfRange);
    }
    if (rectangular_error > thresholds.max_rectangular_error_rad)
    {
        return PairResult(PairRejectReason::RectangularErrorTooLarge);
    }
    if (!IsValidQuadrilateral(corners) || !IsFinite(area))
    {
        return PairResult(PairRejectReason::InvalidCorners);
    }

    const float weighted_error = weights.length      * ClampUnit(length_error      / thresholds.max_length_error)          +
                                 weights.angle       * ClampUnit(angle_error       / thresholds.max_angle_diff_rad)        +
                                 weights.height      * ClampUnit(y_offset_ratio    / thresholds.max_y_offset_ratio)        +
                                 weights.rectangular * ClampUnit(rectangular_error / thresholds.max_rectangular_error_rad) +
                                 weights.color       * ClampUnit(color_error       / thresholds.max_color_difference);

    const float geometry_cost = ClampUnit(weighted_error / weights.Sum());
    const ArmorSize size      = spacing_ratio >= thresholds.large_armor_spacing ? ArmorSize::Large : ArmorSize::Small;

    const ArmorMetrics metrics {
                spacing_ratio,       length_error,
              angle_error,       y_offset_ratio,
        rectangular_error,    color_error,
           area,               geometry_cost
    };

    return PairResult(ArmorCandidate(left, right, corners, size, metrics));
}

/**
 * @brief 检查候选四角点是否全部位于图像有效像素范围
 *
 * @param candidate 已通过几何配对的候选
 * @param image_size 图像宽高，单位 pixel
 * @return true 四角点均位于 [0, width-1] 与 [0, height-1]
 */
bool ArmorGeometry::CornersInsideImage(const ArmorCandidate& candidate, const cv::Size& image_size)
{
    if (image_size.width <= 0 || image_size.height <= 0)
    {
        return false;
    }

    return std::all_of(candidate.GetCorners().begin(), candidate.GetCorners().end(), [&image_size](const cv::Point2f& point)
    {
        return IsFinite(point) && point.x >= 0.0f && point.y >= 0.0f && point.x <= static_cast<float>(image_size.width - 1) && point.y <= static_cast<float>(image_size.height - 1);
    });
}

/**
 * @brief 按几何得分排序并移除共享灯条的候选
 *
 * 先排序，再贪心选取：灯条已被占用的候选直接丢弃。
 *
 * @param candidates 已通过 Pair() 的候选集合
 * @return 从高分到低分排列且任意两项不共享灯条的候选集合
 */
std::vector<ArmorCandidate>
ArmorGeometry::ResolveLightConflicts(std::vector<ArmorCandidate> candidates)
{
    std::sort(candidates.begin(), candidates.end(), CandidateBetter);

    std::unordered_set<std::uint32_t> used_lights;
    std::vector<ArmorCandidate> selected;
    selected.reserve(candidates.size());

    for (const ArmorCandidate& candidate : candidates)
    {
        const std::uint32_t left_id  = candidate.GetLeftLight() .GetId();
        const std::uint32_t right_id = candidate.GetRightLight().GetId();

        if (used_lights.count(left_id) != 0 || used_lights.count(right_id) != 0)
        {
            continue;
        }
        used_lights.insert(left_id);
        used_lights.insert(right_id);
        selected.push_back(candidate);
    }
    return selected;
}

} // namespace algorithm::vision
