/**
 * @file multi_target.hpp
 * @author qingyu
 * @brief 多辆车的编号路由 — 槽位与机器人编号编译期绑定，每帧按号分派
 * @version 0.1
 * @date 2026-10-17
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "framework/algorithm/vision/core/detections.hpp"
#include "framework/algorithm/vision/track/armor_tracker.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <opencv2/core.hpp>
#include <optional>
#include <utility>
#include <vector>

namespace algorithm::vision
{

/**
 * @brief 槽位与机器人编号的绑定表，编译期常量
 *
 * 第 s 个槽固定跟踪编号为 `kTargetLabels[s]` 的车。对手数量写在这张表的元素个数里；
 * 换编制/换对手就增删这里的编号并重新编译——这就是"不做运行时关联"的代价，换来的是
 * 没有代价矩阵、没有贪心配对、没有生灭管理。槽位数由这张表的大小决定。
 *
 * 注意 `ArmorLabel` 是**机器人编号**（同车四面同号），不是"第几面"；分面由
 * `ArmorTracker` 内部按重投影残差各帧独立判定（见 `装甲板位姿与四面绘制规划.md` §5）。
 */
constexpr std::array kTargetLabels = {
    ArmorLabel::Three,
};

/**
 * @brief 槽位数，即对手数量；由 `kTargetLabels` 的大小决定
 */
constexpr std::int32_t kMaxTargets = static_cast<std::int32_t>(kTargetLabels.size());

/**
 * @brief 多目标配置
 */
struct MultiTargetConfig
{
    ArmorTrackerConfig tracker {};                  // 每个槽共用同一份跟踪器参数
    double face_gate_rad {0.6};                     // 面序门：法向角差与方位角差之和的上限，单位 rad
};

/**
 * @brief 一个槽的运行统计，仅供日志与调试
 */
struct TargetReport
{
    ArmorLabel label {ArmorLabel::NotArmor};        // 该槽绑定的编号
    bool       active {false};                      // 是否已初始化过
    bool       converged {false};                   // 是否已收敛
    int32_t    updates {0};                         // 累计观测更新次数
    int32_t    losses {0};                          // 累计未匹配帧数
    int32_t    duplicates {0};                      // 累计"同号多检出"而拒绝更新的帧数
    int32_t    gate_rejects {0};                    // 累计被后备门拒掉的帧数
    int32_t    last_face {-1};                      // 最近一次匹配到的面序，-1 表示本帧没更新
    double     last_cost {0.0};                     // 最近一次匹配的最小残差平方和
};

/**
 * @brief 多辆车的编号路由与状态管理
 *
 * 每帧做三件事：① 每个槽按 dt 推进状态；② 把检测按编号分派到对应槽；③ 在有检测的槽上
 * 按四面的重投影残差定面序并更新。槽位常驻，不存在销毁，`MultiTargetTracker` 不涉及
 * 生灭——一辆车离开视野后该槽只是持续 `Predict`，回来时按预测方位直接接上。
 *
 * 该类只调度，不做几何计算：面序、观测模型、状态约束都在 `ArmorTracker` 里。
 */
namespace detail
{

/**
 * @brief 构造 kMaxTargets 份跟踪器的下标序列展开
 *
 * `ArmorTracker` 没有默认构造（构造即建滤波器），`std::array` 不能循环赋值，所以用下标
 * 序列把配置展开成 kMaxTargets 份。槽位数只由 `kTargetLabels` 决定，这里不写死个数。
 */
template <std::size_t... Index>
std::array<ArmorTracker, sizeof...(Index)> MakeTrackersImpl(const ArmorTrackerConfig& config, std::index_sequence<Index...>)
{
    return {((void)Index, ArmorTracker(config))...};
}

/**
 * @brief 构造 kTargetLabels.size() 个槽，每个槽一份独立的跟踪器
 */
inline std::array<ArmorTracker, static_cast<std::size_t>(kMaxTargets)> MakeTrackers(const ArmorTrackerConfig& config)
{
    return MakeTrackersImpl(config, std::make_index_sequence<static_cast<std::size_t>(kMaxTargets)> {});
}

} // namespace detail

class MultiTargetTracker final
{
public:
    /**
     * @brief 建立多目标跟踪器，各槽共用同一份跟踪器配置
     */
    explicit MultiTargetTracker(const MultiTargetConfig& config) : config_(config), trackers_(detail::MakeTrackers(config.tracker)) {}

    /**
     * @brief 用一帧检测推进所有槽
     *
     * @param armors 当帧检测结果
     * @param dimensions 装甲板实物尺寸，单位 m
     * @param camera 内参与畸变
     * @param dt_s 距上一帧的时间，单位 s
     */
    void Update(const std::vector<DetectedArmor>& armors, const ArmorDimensions& dimensions, const CameraModel& camera, double dt_s);

    /**
     * @brief 第 s 个槽的四面的中心与转角
     *
     * @param slot 槽序，须在 [0, kMaxTargets)
     * @return 四面状态；该槽未初始化时返回空
     */
    std::vector<ArmorState> Armors(std::int32_t slot) const;

    /**
     * @brief 第 s 个槽是否已收敛
     */
    bool Converged(std::int32_t slot) const;

    /**
     * @brief 第 s 个槽的主半径估计，单位 m
     *
     * @param slot 槽序，须在 [0, kMaxTargets)
     * @return 半径；槽序越界或该槽未初始化时返回 0
     */
    double Radius(std::int32_t slot) const;

    /**
     * @brief 第 s 个槽是否已初始化过
     */
    bool Active(std::int32_t slot) const;

    /**
     * @brief 各槽的运行统计
     */
    std::array<TargetReport, kMaxTargets> Reports() const;

private:
    /**
     * @brief 把编号换算成槽序
     *
     * @param label 检测的机器人编号
     * @return 槽序；该编号不在 `kTargetLabels` 里时返回 -1
     */
    static std::int32_t SlotOf(ArmorLabel label);

    /**
     * @brief 按四面的重投影残差取最小，定出这一帧看到的是第几面
     *
     * @param slot 槽序
     * @param size 装甲板大小
     * @param corners 观测四点
     * @param dimensions 板尺寸
     * @param camera 内参与畸变
     * @return 面序与最小残差（像素平方和）；该槽未初始化或投影全失败时返回 std::nullopt
     */
    std::optional<std::pair<std::int32_t, double>> MatchFace(std::int32_t slot, ArmorSize size, const std::array<cv::Point2f, 4>& corners, const ArmorDimensions& dimensions, const CameraModel& camera) const;

    MultiTargetConfig config_;
    std::array<ArmorTracker, kMaxTargets> trackers_;   // 槽位常驻，每个套一份独立的跟踪器
    std::array<TargetReport, kMaxTargets> reports_ {};
};

} // namespace algorithm::vision
