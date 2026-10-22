/**
 * @file trd_vision_report.cpp
 * @author qingyu
 * @brief 识别线程统计 — 一秒窗口累加与汇总日志
 * @version 0.1
 * @date 2026-10-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_vision.hpp"
#include "framework/cmd/shell/log/log.hpp"
#include <algorithm>
#include <array>
#include <cstdint>

namespace thread::vision
{

/**
 * @brief 把装甲板颜色转换为日志字符串
 * @param color 工程颜色枚举
 * @return 固定字符串 red、blue 或 unknown
 */
static const char* ColorName(TeamColor color)
{
    switch (color)
    {
        case TeamColor::Red:
            return "red";
        case TeamColor::Blue:
            return "blue";
        default:
            return "unknown";
    }
}

/**
 * @brief 把装甲板标签转换为日志字符串
 * @param label 工程标签枚举
 * @return 固定类别字符串
 */
static const char* LabelName(ArmorLabel label)
{
    switch (label)
    {
        case ArmorLabel::One:
            return "1";
        case ArmorLabel::Two:
            return "2";
        case ArmorLabel::Three:
            return "3";
        case ArmorLabel::Four:
            return "4";
        case ArmorLabel::Five:
            return "5";
        case ArmorLabel::Sentry:
            return "sentry";
        case ArmorLabel::Outpost:
            return "outpost";
        case ArmorLabel::Base:
            return "base";
        case ArmorLabel::Any:
            return "any";
        default:
            return "not_armor";
    }
}

/**
 * @brief 累加一帧的位姿解算结果
 *
 * 与检测统计分开累加：位姿是检测的下游，只有解出位姿的那几块才计入这里。
 */
void ReportWindow::AddPose(uint64_t solved, uint64_t invalid, double reprojection_px, double distance_m)
{
    pnp_ok              += solved;
    pnp_invalid         += invalid;
    pnp_reprojection_px += reprojection_px;
    pnp_distance_m      += distance_m;
}

/**
 * @brief 累加一帧检测结果
 */
void ReportWindow::Add(const ArmorDetectionResult& result, double age_ms)
{
    ++frames;
    score_passed        += result.stats.score_passed;
    nms_kept            += result.stats.nms_kept;
    accepted            += result.stats.accepted;

    rejected_confidence += result.stats.rejected_confidence;
    rejected_color      += result.stats.rejected_color;
    rejected_label_type += result.stats.rejected_label_type;
    rejected_geometry   += result.stats.rejected_geometry;

    max_objectness       = std::max(max_objectness, result.stats.max_objectness);
    preprocess_ms       += result.stats.preprocess_ms;
    inference_ms        += result.stats.inference_ms;
    postprocess_ms      += result.stats.postprocess_ms;
    frame_age_ms        += age_ms;

    for (const DetectedArmor& armor : result.armors)
    {
        if (!best || armor.GetConfidence() > best->GetConfidence())
        {
            best = armor;
        }
    }
}

/**
 * @brief 清空当前窗口，开始下一秒统计
 */
void ReportWindow::Clear()
{
    *this = ReportWindow {};
}

/**
 * @brief 输出一条识别窗口汇总日志
 * @param report 已完成的一秒统计窗口
 * @param elapsed 窗口实际持续时间，单位 s
 */
void LogReport(const ReportWindow& report, double elapsed)
{
    const double denominator     = report.frames == 0 ? 1.0 : static_cast<double>(report.frames);
    const double pnp_denominator = report.pnp_ok == 0 ? 1.0 : static_cast<double>(report.pnp_ok);
    if (!report.best)
    {
        DUST_LOG_DBG("vision", "fps=%.1f taken=%llu dropped=%llu raw=25200 score=%llu nms=%llu accepted=%llu reject(conf/color/type/geometry)=%llu/%llu/%llu/%llu max_obj=%.3f ms(pre/infer/post/age)=%.2f/%.2f/%.2f/%.2f target=none", report.frames / elapsed, static_cast<unsigned long long>(runtime_.camera_to.Taken()), static_cast<unsigned long long>(runtime_.camera_to.Dropped()), static_cast<unsigned long long>(report.score_passed), static_cast<unsigned long long>(report.nms_kept), static_cast<unsigned long long>(report.accepted), static_cast<unsigned long long>(report.rejected_confidence), static_cast<unsigned long long>(report.rejected_color), static_cast<unsigned long long>(report.rejected_label_type), static_cast<unsigned long long>(report.rejected_geometry), report.max_objectness, report.preprocess_ms / denominator, report.inference_ms / denominator, report.postprocess_ms / denominator, report.frame_age_ms / denominator);
    }
    else
    {
        const DetectedArmor& armor = *report.best;
        const auto& corners = armor.GetCorners();
        DUST_LOG_DBG("vision", "fps=%.1f taken=%llu dropped=%llu raw=25200 score=%llu nms=%llu accepted=%llu reject(conf/color/type/geometry)=%llu/%llu/%llu/%llu max_obj=%.3f ms(pre/infer/post/age)=%.2f/%.2f/%.2f/%.2f target=%s/%s conf=%.3f corners=(%.1f,%.1f)(%.1f,%.1f)(%.1f,%.1f)(%.1f,%.1f)", report.frames / elapsed, static_cast<unsigned long long>(runtime_.camera_to.Taken()), static_cast<unsigned long long>(runtime_.camera_to.Dropped()), static_cast<unsigned long long>(report.score_passed), static_cast<unsigned long long>(report.nms_kept), static_cast<unsigned long long>(report.accepted), static_cast<unsigned long long>(report.rejected_confidence), static_cast<unsigned long long>(report.rejected_color), static_cast<unsigned long long>(report.rejected_label_type), static_cast<unsigned long long>(report.rejected_geometry), report.max_objectness, report.preprocess_ms / denominator, report.inference_ms / denominator, report.postprocess_ms / denominator, report.frame_age_ms / denominator, ColorName(armor.GetColor()), LabelName(armor.GetLabel()), armor.GetConfidence(), corners[0].x, corners[0].y, corners[1].x, corners[1].y, corners[2].x, corners[2].y, corners[3].x, corners[3].y);
    }

    // 位姿单独一行：err 是重投影、dist 是板心距离，做距离核对时只盯这一行
    DUST_LOG_DBG("vision", "pnp ok=%llu invalid=%llu err=%.3fpx dist=%.3fm", static_cast<unsigned long long>(report.pnp_ok), static_cast<unsigned long long>(report.pnp_invalid), report.pnp_reprojection_px / pnp_denominator, report.pnp_distance_m / pnp_denominator);

    // 跟踪单独一行：每个槽一行，看收敛状态与半径
    if (runtime_.multi)
    {
        const std::array<TargetReport, kMaxTargets> reports = runtime_.multi->Reports();
        for (std::int32_t slot = 0; slot < kMaxTargets; ++slot)
        {
            const TargetReport& target = reports[static_cast<std::size_t>(slot)];
            DUST_LOG_DBG("vision", "track slot=%d label=%s active=%d converged=%d updates=%d losses=%d dup=%d gate=%d r=%.3fm",
                         slot, LabelName(target.label), target.active ? 1 : 0, target.converged ? 1 : 0,
                         target.updates, target.losses, target.duplicates, target.gate_rejects, runtime_.multi->Radius(slot));
        }
    }
}

} // namespace thread::vision
