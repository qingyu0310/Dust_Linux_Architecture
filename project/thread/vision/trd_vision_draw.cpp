/**
 * @file trd_vision_draw.cpp
 * @author qingyu
 * @brief 识别线程绘制 — 内参折算、实测四点轮廓、收敛目标四面轮廓
 * @version 0.1
 * @date 2026-10-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_vision.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <vector>

namespace thread::vision
{

/**
 * @brief 把装甲板颜色转换为绘制标签
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
 * @brief 把装甲板标签转换为绘制标签
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
 * @brief 把线程读入的内参折算成算法层的针孔模型
 */
CameraModel ToCameraModel(const CameraIntrinsics& intrinsics)
{
    CameraModel camera;
    camera.fx = intrinsics.fx;
    camera.fy = intrinsics.fy;
    camera.cx = intrinsics.cx;
    camera.cy = intrinsics.cy;
    camera.k1 = intrinsics.k1;
    camera.k2 = intrinsics.k2;
    camera.p1 = intrinsics.p1;
    camera.p2 = intrinsics.p2;
    camera.k3 = intrinsics.k3;

    return camera;
}

/**
 * @brief 把一个已收敛目标推出来的四面轮廓描在图上
 *
 * 画的是状态推出来的四面，不是实测四点：每面按 `Armors()` 给出的中心与转角反投。尚未收敛的
 * 目标不调用本函数——外推误差可达画面宽的 1/3（规划 §2）。
 *
 * @param drawn 输出图像，原地绘制
 * @param faces 该目标四面的中心与转角
 * @param dimensions 板尺寸
 * @param camera 内参与畸变
 * @param extrinsics 相机相对云台的外参
 * @param color 该目标的绘制颜色
 */
static void DrawFaces(cv::Mat& drawn, const std::vector<ArmorState>& faces, const ArmorDimensions& dimensions, const CameraModel& camera, const GimbalExtrinsics& extrinsics, const cv::Scalar& color)
{
    const std::optional<std::array<cv::Point3f, 4>> object_points = MakeObjectPoints(ArmorSize::Small, dimensions);
    if (!object_points)
    {
        return;
    }

    for (const ArmorState& face : faces)
    {
        // 状态在云台系：换到相机系再投影
        const Eigen::Vector3d center = GimbalToCamera(face.xyz, extrinsics);
        const cv::Vec3d rvec = GimbalToCameraRotation(ArmorRotation(face.yaw_rad, 0.0), extrinsics);

        const std::optional<std::array<cv::Point2f, 4>> points = ProjectPose(*object_points, rvec, cv::Vec3d(center.x(), center.y(), center.z()), camera);
        if (!points)
        {
            continue;
        }

        std::vector<cv::Point> contour;
        contour.reserve(points->size());
        for (const cv::Point2f& point : *points)
        {
            contour.emplace_back(cv::Point(static_cast<int32_t>(std::lround(point.x)), static_cast<int32_t>(std::lround(point.y))));
        }
        cv::polylines(drawn, contour, true, color, 2, cv::LINE_AA);
    }
}

/**
 * @brief 推进多目标跟踪并把结果描在副本上
 *
 * 未收敛的目标只描本帧实测到的那块（解 PnP 后的四点），收敛后描状态推出来的四面。
 * 画的是原图副本——通道里的像素与显示侧共享，改图的一方自己复制。
 *
 * @param image 相机帧，整幅 BGR
 * @param armors 本帧检测结果
 * @param report 本帧位姿统计，解出与未解出的块数都累加进去
 * @return 叠加好轮廓的图像
 */
cv::Mat DrawPoses(const cv::Mat& image, const std::vector<DetectedArmor>& armors, ReportWindow& report)
{
    cv::Mat drawn = image.clone();
    const CameraModel camera = ToCameraModel(config_.camera);
    const ArmorDimensions dimensions; // 实测确定值，常量在 armor_pose.hpp

    // 跟踪的 dt：用相机帧时间戳，缺首帧时给一帧的标称间隔
    const auto now = std::chrono::steady_clock::now();
    const double dt_s = runtime_.last_frame_time ? std::chrono::duration<double>(now - *runtime_.last_frame_time).count() : (1.0 / 30.0);
    runtime_.last_frame_time = now;

    if (runtime_.multi)
    {
        runtime_.multi->Update(armors, dimensions, camera, dt_s);
    }

    uint64_t solved  = 0;
    uint64_t invalid = 0;
    double reprojection_sum = 0.0;
    double distance_sum     = 0.0;

    // 先描未收敛目标"实测到的那块"：它是本帧真实观测，收敛前的唯一依据
    for (const DetectedArmor& armor : armors)
    {
        const std::optional<std::array<cv::Point3f, 4>> object_points = MakeObjectPoints(armor.GetSize(), dimensions);
        const std::optional<ArmorPose> pose = object_points ? SolvePose(armor.GetCorners(), *object_points, camera) : std::nullopt;
        if (!pose)
        {
            ++invalid;
            continue;
        }

        ++solved;
        reprojection_sum += pose->reprojection_error_px;
        distance_sum     += cv::norm(pose->tvec);

        std::vector<cv::Point> contour;
        contour.reserve(pose->projected.size());
        for (const cv::Point2f& point : pose->projected)
        {
            contour.emplace_back(cv::Point(static_cast<int32_t>(std::lround(point.x)), static_cast<int32_t>(std::lround(point.y))));
        }
        cv::polylines(drawn, contour, true, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

        const cv::Scalar color = armor.GetColor() == TeamColor::Red ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
        char text[64];
        std::snprintf(text, sizeof(text), "%s/%s %.2f", ColorName(armor.GetColor()), LabelName(armor.GetLabel()), armor.GetConfidence());
        const cv::Point text_position(static_cast<int32_t>(contour[0].x), static_cast<int32_t>(std::max(18.0f, contour[0].y - 6.0f)));
        cv::putText(drawn, text, text_position, cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv::LINE_AA);
    }

    // 再描已收敛目标的四面：每槽一套颜色，便于分辨不同目标
    static const cv::Scalar kTargetColors[] = {
        cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255), cv::Scalar(255, 255, 0),
    };

    if (runtime_.multi)
    {
        for (std::int32_t slot = 0; slot < kMaxTargets; ++slot)
        {
            if (!runtime_.multi->Converged(slot))
            {
                continue;
            }
            const cv::Scalar& color = kTargetColors[static_cast<std::size_t>(slot) % (sizeof(kTargetColors) / sizeof(kTargetColors[0]))];
            DrawFaces(drawn, runtime_.multi->Armors(slot), dimensions, camera, config_.extrinsics, color);
        }
    }

    report.AddPose(solved, invalid, reprojection_sum, distance_sum);
    return drawn;
}

} // namespace thread::vision
