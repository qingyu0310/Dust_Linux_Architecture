/**
 * @file armor_board.hpp
 * @author qingyu
 * @brief 装甲板纯几何配对 — 几何度量、候选、配对结果与配对器
 * @version 0.1
 * @date 2026-10-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "framework/algorithm/vision/core/armor_types.hpp"
#include "framework/algorithm/vision/core/light_bar.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <opencv2/core.hpp>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace algorithm::vision
{

class ArmorGeometry;

/**
 * @brief 候选装甲板的几何度量
 *
 * 全部由 ArmorGeometry 在配对时算出并随候选保存；归一化量的分母都是两灯条的
 * 平均长度。数字分类结果不属于本结构，两者不混在一起。
 */
struct ArmorMetrics
{
    float spacing_ratio         {0.0f};         // 两灯条中心距 / 平均灯条长度
    float length_error          {0.0f};         // 两灯条长度差 / 平均灯条长度
    float angle_error_rad       {0.0f};         // 两灯条长轴方向差，单位 rad
    float y_offset_ratio        {0.0f};         // 两灯条中心高度差 / 平均灯条长度
    float rectangular_error_rad {0.0f};         // 长轴偏离垂直关系的最大角度，rad
    float color_error           {0.0f};         // 两灯条颜色分数之差的绝对值
    float quadrilateral_area    {0.0f};         // 四角点围成的四边形面积，单位 pixel^2
    float geometry_cost         {0.0f};         // 归一化加权几何代价，范围 [0, 1]，越小越好
};

/**
 * @brief 两根灯条构成的二维装甲板候选
 *
 * 候选只能由 ArmorGeometry 创建。角点顺序恒为左上、右上、右下、左下；
 * 所有误差均保留原始量，几何得分不与后续数字置信度混合。
 */
class ArmorCandidate final
{
public:
    ArmorCandidate(const ArmorCandidate&) = default;
    ArmorCandidate(ArmorCandidate&&) noexcept = default;
    ArmorCandidate& operator=(const ArmorCandidate&) = default;
    ArmorCandidate& operator=(ArmorCandidate&&) noexcept = default;

    /**
     * @brief 返回左灯条
     */
    const LightBar& GetLeftLight() const noexcept
    {
        return left_;
    }

    /**
     * @brief 返回右灯条
     */
    const LightBar& GetRightLight() const noexcept
    {
        return right_;
    }

    /**
     * @brief 返回左上、右上、右下、左下四角点，单位 pixel
     */
    const std::array<cv::Point2f, 4>& GetCorners() const noexcept
    {
        return corners_;
    }

    /**
     * @brief 返回两灯条中心的平均位置，单位 pixel
     */
    const cv::Point2f& GetCenter() const noexcept
    {
        return center_;
    }

    /**
     * @brief 返回装甲板颜色
     */
    TeamColor GetColor() const noexcept
    {
        return color_;
    }

    /**
     * @brief 返回按灯条间距判断的装甲板尺寸
     */
    ArmorSize GetSize() const noexcept
    {
        return size_;
    }

    /**
     * @brief 返回数字标签；阶段 1 固定为 NotArmor，阶段 3 由分类器填写
     */
    ArmorLabel GetLabel() const noexcept
    {
        return label_;
    }

    /**
     * @brief 返回两灯条中心距除以平均灯条长度
     */
    float GetSpacingRatio() const noexcept
    {
        return metrics_.spacing_ratio;
    }

    /**
     * @brief 返回两灯条长度差除以平均灯条长度
     */
    float GetLengthError() const noexcept
    {
        return metrics_.length_error;
    }

    /**
     * @brief 返回两灯条最小方向夹角，单位 rad
     */
    float GetAngleError() const noexcept
    {
        return metrics_.angle_error_rad;
    }

    /**
     * @brief 返回两灯条中心高度差除以平均灯条长度
     */
    float GetYOffsetRatio() const noexcept
    {
        return metrics_.y_offset_ratio;
    }

    /**
     * @brief 返回灯条长轴与中心连线偏离垂直关系的最大角度，单位 rad
     */
    float GetRectangularError() const noexcept
    {
        return metrics_.rectangular_error_rad;
    }

    /**
     * @brief 返回两灯条颜色分数之差的绝对值
     */
    float GetColorError() const noexcept
    {
        return metrics_.color_error;
    }

    /**
     * @brief 返回按固定角点顺序计算的四边形面积，单位 pixel^2
     */
    float GetArea() const noexcept
    {
        return metrics_.quadrilateral_area;
    }

    /**
     * @brief 返回归一化加权几何代价，范围 [0, 1]，越小越好
     */
    float GetGeometryCost() const noexcept
    {
        return metrics_.geometry_cost;
    }

    /**
     * @brief 返回几何得分，等于 1 - geometry_cost，范围 [0, 1]
     */
    float GetGeometryScore() const noexcept
    {
        return 1.0f - metrics_.geometry_cost;
    }

    /**
     * @brief 返回数字分类置信度；阶段 1 固定为 0
     */
    float GetDigitConfidence() const noexcept
    {
        return digit_confidence_;
    }

    /**
     * @brief 返回数字第一、第二名置信度差；阶段 1 固定为 0
     */
    float GetDigitMargin() const noexcept
    {
        return digit_margin_;
    }

private:
    friend class ArmorGeometry;

    /**
     * @brief 由配对结果构造候选：中心由两灯条算出，几何度量整组原样保存
     *
     * 颜色取自左灯条：进入本构造前两灯条颜色已被 ArmorGeometry 判为相同。
     */
    ArmorCandidate(const LightBar& left, const LightBar& right, const std::array<cv::Point2f, 4>& corners, ArmorSize size, const ArmorMetrics& metrics)
        : left_(left), right_(right), corners_(corners), center_((left.GetCenter() + right.GetCenter()) * 0.5f), color_(left.GetColor()), size_(size), metrics_(metrics)
    {
    }

    LightBar     left_;                         // 左灯条
    LightBar     right_;                        // 右灯条

    std::array<cv::Point2f, 4> corners_;        // 左上、右上、右下、左下，pixel
    cv::Point2f  center_;                       // 两灯条中心平均值，pixel

    TeamColor    color_;                        // 与两灯条一致的颜色

    ArmorSize    size_;                         // 按归一化间距判断的尺寸
    ArmorLabel   label_{ArmorLabel::NotArmor};  // 数字分类标签
    ArmorMetrics metrics_;                      // 几何度量

    float digit_margin_     {0.0f};             // 数字前两名置信度差
    float digit_confidence_ {0.0f};             // 数字分类置信度
};

/**
 * @brief 灯条配对拒绝原因
 */
enum class PairRejectReason : std::uint8_t
{
    None,                                       // 配对成功
    SameLightBar,                               // 两个输入使用相同灯条 ID
    ColorMismatch,                              // 颜色不同或尚未判色
    InvalidOrder,                               // 中心横坐标相同，无法区分左右
    LengthMismatch,                             // 归一化长度差超过门限
    AngleMismatch,                              // 灯条方向差超过门限
    YOffsetTooLarge,                            // 中心高度差超过门限
    SpacingOutOfRange,                          // 归一化中心距不在允许范围
    RectangularErrorTooLarge,                   // 灯条与中心连线不满足近似垂直
    InvalidCorners                              // 四角点非凸、自交或面积无效
};

/**
 * @brief 单次灯条配对结果
 */
class PairResult final
{
public:
    /**
     * @brief 查询配对是否成功
     */
    bool Accepted() const noexcept
    {
        return reason_ == PairRejectReason::None;
    }

    /**
     * @brief 返回成功或拒绝原因
     */
    PairRejectReason GetReason() const noexcept
    {
        return reason_;
    }

    /**
     * @brief 返回候选装甲板
     * @return 配对成功时返回候选地址；失败时返回 nullptr
     */
    const ArmorCandidate* GetCandidate() const noexcept
    {
        return candidate_ ? &candidate_.value() : nullptr;
    }

private:
    friend class ArmorGeometry;

    PairRejectReason reason_;                       // 成功或拒绝原因
    std::optional<ArmorCandidate> candidate_;       // 仅成功时存在

    /**
     * @brief 构造拒绝结果，不携带候选
     */
    explicit PairResult(PairRejectReason reason) : reason_(reason) {}

    /**
     * @brief 构造成功结果，候选随结果一起移动
     */
    explicit PairResult(ArmorCandidate candidate) : reason_(PairRejectReason::None), candidate_(std::move(candidate)) {}
};

/**
 * @brief 装甲板纯几何配对器
 *
 * 构造时一次校验全部参数，之后 Pair() 只处理满足 LightBar 类不变量的对象。
 * 本类不读取图像、不加载模型、不写文件，也不持有跨帧状态。
 */
/**
 * @brief 校验辅助：全部参数都是有限正数
 */
template <typename... Values>
bool AllPositiveFinite(Values... values)
{
    return (... && (std::isfinite(values) && values > 0.0f));
}

/**
 * @brief 校验辅助：全部参数都是有限且非负
 */
template <typename... Values>
bool AllNonNegativeFinite(Values... values)
{
    return (... && (std::isfinite(values) && values >= 0.0f));
}

class ArmorGeometry final
{
public:
    /**
     * @brief 灯条配对门限：任一项不过就拒绝这对灯条
     */
    struct Thresholds
    {
        float max_length_error          {0.5f};         // 归一化长度差上限，必须 > 0
        float max_angle_diff_rad        {0.35f};        // 灯条方向差上限，rad，必须 > 0
        float max_y_offset_ratio        {0.5f};         // 归一化中心高度差上限，必须 > 0
        float min_armor_spacing         {1.0f};         // 归一化中心距下限，必须 > 0
        float max_armor_spacing         {5.0f};         // 归一化中心距上限，必须大于下限
        float large_armor_spacing       {3.0f};         // 大小装甲分界，须落在中心距范围内
        float max_rectangular_error_rad {0.44f};        // 矩形关系误差上限，rad，必须 > 0
        float max_color_difference      {2.0f};         // 颜色误差归一化上限，必须 > 0

        /**
         * @brief 校验门限：全部为有限正数，且中心距上下限与大小装甲分界自洽
         *
         * @throws std::invalid_argument 任一约束不满足
         */
        void Validate() const
        {
            const bool positive = AllPositiveFinite(max_length_error, max_angle_diff_rad, max_y_offset_ratio, min_armor_spacing, max_armor_spacing, large_armor_spacing, max_rectangular_error_rad, max_color_difference);
            const bool spacing_ordered = max_armor_spacing > min_armor_spacing && large_armor_spacing >= min_armor_spacing && large_armor_spacing <= max_armor_spacing;

            if (!positive || !spacing_ordered)
            {
                throw std::invalid_argument("invalid armor geometry thresholds");
            }
        }
    };

    /**
     * @brief 几何代价的权重：只有相对比例有意义，代价按权重总和归一
     */
    struct Weights
    {
        float length      {1.0f};           // 长度误差权重，必须 >= 0
        float angle       {1.0f};           // 方向误差权重，必须 >= 0
        float height      {1.0f};           // 高度误差权重，必须 >= 0
        float rectangular {1.0f};           // 矩形关系误差权重，必须 >= 0
        float color       {0.0f};           // 颜色误差权重，必须 >= 0

        /**
         * @brief 校验权重：全部有限且非负，且总和为正
         *
         * @throws std::invalid_argument 任一约束不满足
         */
        void Validate() const
        {
            if (!AllNonNegativeFinite(length, angle, height, rectangular, color) || Sum() <= 0.0f)
            {
                throw std::invalid_argument("invalid armor geometry weights");
            }
        }

        /**
         * @brief 权重总和，几何代价按它归一
         *
         * @return 五个权重之和
         */
        float Sum() const
        {
            return length + angle + height + rectangular + color;
        }
    };

    /**
     * @brief 配对门限与评分权重
     */
    struct PairingConfig
    {
        Thresholds thresholds;      // 判据：哪对灯条算合格
        Weights    weights;         // 代价：合格候选中谁更可信
    };

    explicit ArmorGeometry(const PairingConfig& config);

    PairResult Pair(const LightBar& first, const LightBar& second) const;

    static bool CornersInsideImage(const ArmorCandidate& candidate, const cv::Size& image_size);

    static std::vector<ArmorCandidate> ResolveLightConflicts(std::vector<ArmorCandidate> candidates);

    /**
     * @brief 返回当前配对配置
     */
    const PairingConfig& GetConfig() const noexcept
    {
        return config_;
    }

private:
    PairingConfig config_; // 构造时完成校验，运行期只读
};

} // namespace algorithm::vision
