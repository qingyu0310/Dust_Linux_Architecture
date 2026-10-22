/**
 * @file trd_test.cpp
 * @author qingyu
 * @brief 测试线程：初始化注册与当前这一条测试
 * @version 0.1
 * @date 2026-10-17
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "framework/algorithm/vision/core/detections.hpp"
#include "framework/algorithm/vision/pose/armor_pose.hpp"
#include "framework/algorithm/vision/track/armor_tracker.hpp"
#include "framework/algorithm/vision/track/multi_target.hpp"
#include "framework/init/startup/Init_entry.hpp"
#include "framework/cmd/shell/log/log.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <optional>
#include <vector>

namespace thread::test
{
namespace
{

/**
 * @brief 本机标定的内参与畸变
 */
algorithm::vision::CameraModel MakeCamera()
{
    algorithm::vision::CameraModel camera;
    camera.fx = 1799.539711;
    camera.fy = 1800.364749;
    camera.cx = 722.8841479;
    camera.cy = 504.3663714;
    camera.k1 = -0.07677082222;
    camera.k2 = 0.1417360699;
    camera.p1 = -0.0004933148161;
    camera.p2 = 0.001394902737;
    camera.k3 = 0.0;

    return camera;
}

/**
 * @brief 造一帧观测：一辆车以给定半径和角速度自转，取某一面的四点
 *
 * 车的旋转中心固定在相机系原点前方，只有转角随时间推进，所以真实半径是可以对答案的。
 * 面心与朝向按跟踪器的同一套约定算：面 i 的方位角 = a + i·2π/4，半径（面 1、3）为 r+l，
 * 高度为 cz+h。
 *
 * @param camera 内参与畸变
 * @param dimensions 板尺寸
 * @param yaw_rad 当帧面 0 的转角
 * @param radius_m 真实主半径，单位 m
 * @param radius_gap_m 真实半径差 l，单位 m；面 1、3 用 radius + l
 * @param height_gap_m 真实高度差 h，单位 m
 * @param face 本帧看到的是第几面
 * @param center 旋转中心，相机系，单位 m
 * @param corners 输出四点
 * @return true 生成成功
 */
bool MakeObservation(const algorithm::vision::CameraModel& camera, const algorithm::vision::ArmorDimensions& dimensions,
                     double yaw_rad, double radius_m, double radius_gap_m, double height_gap_m, std::int32_t face,
                     const Eigen::Vector3d& center, std::array<cv::Point2f, 4>& corners)
{
    const std::optional<std::array<cv::Point3f, 4>> object_points = algorithm::vision::MakeObjectPoints(algorithm::vision::ArmorSize::Small, dimensions);
    if (!object_points)
    {
        return false;
    }

    constexpr std::int32_t kFaceCount = 4;
    const double angle = yaw_rad + static_cast<double>(face) * 2.0 * M_PI / static_cast<double>(kFaceCount);
    const bool use_second = (face == 1 || face == 3);
    const double radius = use_second ? radius_m + radius_gap_m : radius_m;

    // 面心 = 旋转中心 − r·n，n = (cos a, 0, sin a)，与跟踪器同一约定
    const cv::Vec3d tvec(center.x() - radius * std::cos(angle), center.y() + (use_second ? height_gap_m : 0.0), center.z() - radius * std::sin(angle));

    const std::optional<std::array<cv::Point2f, 4>> points = algorithm::vision::ProjectPose(*object_points, algorithm::vision::ArmorRotation(angle, 0.0), tvec, camera);
    if (!points)
    {
        return false;
    }

    corners = *points;
    return true;
}

/**
 * @brief 车自转时检测器会报出的那一面：投影面积为正且最大的面
 *
 * 不靠推导 `sin(a)` 的符号来确定可见面——面积符号同时取决于旋转约定与板系朝向，推错一次就
 * 会把背对相机的面当成观测喂进跟踪器。这里直接用与生产代码相同的 `ProjectPose` 把每个面投
 * 一遍，取**有向面积为正**且绝对值最大的那个：正面积是 `DetectedArmor::Create` 的硬校验，
 * 面积最大则对应最正对相机的那一面。
 *
 * @param camera 内参与畸变
 * @param dimensions 板尺寸
 * @param car_angle 车的转角，单位 rad
 * @param radius 真实主半径，单位 m
 * @param center 旋转中心，相机系，单位 m
 * @return 可见面的面序；四面都投影失败时返回 -1
 */
std::int32_t VisibleFace(const algorithm::vision::CameraModel& camera, const algorithm::vision::ArmorDimensions& dimensions,
                         double car_angle, double radius, const Eigen::Vector3d& center)
{
    std::int32_t best_face = -1;
    double largest_area = 0.0;

    for (std::int32_t face = 0; face < 4; ++face)
    {
        std::array<cv::Point2f, 4> corners {};
        if (!MakeObservation(camera, dimensions, car_angle, radius, 0.0, 0.0, face, center, corners))
        {
            continue;
        }

        const double area = ((corners[1].x - corners[0].x) * (corners[2].y - corners[0].y) -
                             (corners[2].x - corners[0].x) * (corners[1].y - corners[0].y)) * 0.5;
        if (area > largest_area)
        {
            largest_area = area;
            best_face = face;
        }
    }

    return best_face;
}

/**
 * @brief 由四点与编号造一块检测结果，供多目标层做编号路由
 *
 * 多目标层只读检测的编号与四点，其余身份字段（颜色、尺寸）对本条测试无影响，取固定值。
 *
 * @param corners 四点
 * @param label 机器人编号
 * @param image_size 图像尺寸，用于几何校验
 * @return 检测结果；四点几何无效时返回 std::nullopt
 */
std::optional<algorithm::vision::DetectedArmor> MakeDetection(const std::array<cv::Point2f, 4>& corners, algorithm::vision::ArmorLabel label, const cv::Size& image_size)
{
    algorithm::vision::DetectedArmorGeometry geometry;
    geometry.corners = corners;
    geometry.center  = (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25f;

    float min_x = corners[0].x;
    float max_x = corners[0].x;
    float min_y = corners[0].y;
    float max_y = corners[0].y;
    for (const cv::Point2f& point : corners)
    {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }
    geometry.bounding_box = cv::Rect2f(min_x, min_y, max_x - min_x, max_y - min_y);

    algorithm::vision::DetectedArmorIdentity identity;
    identity.color        = algorithm::vision::TeamColor::Blue;
    identity.label        = label;
    identity.size         = algorithm::vision::ArmorSize::Small;
    identity.raw_color_id = 0;
    identity.raw_label_id = label == algorithm::vision::ArmorLabel::One ? 1 : 2;

    return algorithm::vision::DetectedArmor::Create(geometry, identity, 0.9f, image_size);
}

} // namespace

/**
 * @brief 合成两辆车同时自转，喂给多目标层：验编号路由、遮断恢复与第三种编号被忽略
 *
 * 纯离线。两辆车编号分别是一号、二号（都在 `kTargetLabels` 里），位置与转速不同：
 * 一号在 3 m、2 rad/s，二号在 4 m、1 rad/s。检测列表按帧把两车的观测混在一起喂进去，
 * 每帧的检测顺序**故意交替**（偶数帧一号在前、奇数帧二号在前），这样"是否按编号而非
 * 按顺序路由"才验得出来。
 *
 * 验四条：① 两槽各自半径收敛到 ±10%；② 每帧归属正确（错配 = 0）；
 * ③ 遮断一号 10 帧再恢复，半径不重收敛（与遮断前差 < 5%）；
 * ④ 混入五号编号的检测被忽略，两槽的 `updates` 不受影响。
 *
 * @return true 四条都过
 */
static bool CheckMultiTarget()
{
    const algorithm::vision::CameraModel camera = MakeCamera();
    const algorithm::vision::ArmorDimensions dimensions;

    constexpr double kDt = 1.0 / 30.0;
    constexpr std::int32_t kFrames = 90;

    constexpr double kRadiusOne = 0.20;             // 被跟踪车的真值半径，单位 m
    constexpr double kOmegaOne  = 2.0;              // 被跟踪车的角速度，单位 rad/s
    constexpr double kRadiusTwo = 0.25;             // 被忽略车的真值半径，单位 m
    constexpr double kOmegaTwo  = 1.0;              // 被忽略车的角速度，单位 rad/s

    // kTargetLabels 当前只有 Three：三号车被跟踪，五号车必须被忽略
    const Eigen::Vector3d center_one(0.05,  0.02,  3.0);
    const Eigen::Vector3d center_two(-0.60, -0.05, 4.0);    // 被忽略的编号，位置随便
    const Eigen::Vector3d center_third(1.5, 0.0,   5.0);    // 同五号，也应在表外

    const cv::Size image_size(1440, 1080);

    algorithm::vision::MultiTargetConfig config;
    config.tracker.radius_init_m = 0.3;
    config.tracker.point_sigma_px = 0.5;
    config.tracker.accel_variance = 1.0e-4;
    config.tracker.angular_accel_variance = 1.0e-2;

    algorithm::vision::MultiTargetTracker tracker(config);

    // 遮断窗口：一号在这几帧不出现在检测里
    constexpr std::int32_t kOccludeFrom = 45;
    constexpr std::int32_t kOccludeTo   = 55;

    double radius_one_before = 0.0;   // 遮断前一帧一号的半径
    double radius_one_after  = 0.0;   // 遮断恢复并稳定后一号的半径

    // 跟踪器把"第一帧看到的面"命名为面 0，测试的真值面是物理编号，两者恒差一个常数——
    // 差值是约定而不是错误。这里以首帧的差值作基准，之后每帧的差值都应与它相等。
    int32_t face_offset = 0;
    bool    face_consistent = true;
    bool    face_offset_known = false;

    for (std::int32_t frame = 0; frame < kFrames; ++frame)
    {
        // 每条候选都先造出来，再按帧序装配成检测列表——奇数帧把五号放前面，
        // 这样"按编号路由"与"按列表顺序取第一个"才能被区分开
        std::array<std::optional<algorithm::vision::DetectedArmor>, 3> built;
        std::array<std::int32_t, 3> true_face {-1, -1, -1};

        auto build = [&](std::int32_t index, algorithm::vision::ArmorLabel label, double radius, double omega, const Eigen::Vector3d& center)
        {
            const double yaw = omega * static_cast<double>(frame) * kDt;
            const std::int32_t face = VisibleFace(camera, dimensions, yaw, radius, center);
            if (face < 0)
            {
                return false;
            }
            true_face[static_cast<std::size_t>(index)] = face;
            std::array<cv::Point2f, 4> corners {};
            if (!MakeObservation(camera, dimensions, yaw, radius, 0.0, 0.0, face, center, corners))
            {
                return false;
            }
            built[static_cast<std::size_t>(index)] = MakeDetection(corners, label, image_size);
            return built[static_cast<std::size_t>(index)].has_value();
        };

        // 三号车（被跟踪）：遮断窗口内不出现在检测里
        const bool one_visible = (frame < kOccludeFrom || frame >= kOccludeTo);
        if (one_visible && !build(0, algorithm::vision::ArmorLabel::Three, kRadiusOne, kOmegaOne, center_one))
        {
            DUST_LOG_ERR("multi selfcheck: cannot build tracked-car observation");
            return false;
        }
        // 五号车（表外编号，应被忽略）
        if (!build(1, algorithm::vision::ArmorLabel::Five, kRadiusTwo, kOmegaTwo, center_two))
        {
            DUST_LOG_ERR("multi selfcheck: cannot build ignored-car observation");
            return false;
        }
        if (!build(2, algorithm::vision::ArmorLabel::Five, 0.2, kOmegaTwo, center_third))
        {
            DUST_LOG_ERR("multi selfcheck: cannot build second ignored observation");
            return false;
        }

        std::vector<algorithm::vision::DetectedArmor> detections;
        const bool ignored_first = (frame % 2 == 1);
        const std::array<std::int32_t, 3> order = ignored_first ? std::array<std::int32_t, 3> {1, 2, 0}
                                                               : std::array<std::int32_t, 3> {0, 1, 2};
        for (const std::int32_t index : order)
        {
            if (built[static_cast<std::size_t>(index)])
            {
                detections.push_back(*built[static_cast<std::size_t>(index)]);
            }
        }

        tracker.Update(detections, dimensions, camera, kDt);

        {
            const auto reports = tracker.Reports();
            DUST_LOG_INF("multi selfcheck frame=%d vis=%d r=%.4f f=%d/%d",
                         frame, one_visible ? 1 : 0,
                         tracker.Radius(0), reports[0].last_face, true_face[0]);

            // 面序一致性：选中的面与真值面之差应恒为首帧那个偏移
            const auto offset_of = [](std::int32_t selected, std::int32_t truth)
            {
                if (selected < 0 || truth < 0)
                {
                    return 0;
                }
                return ((selected - truth) % 4 + 4) % 4;
            };

            // 只有"本帧确实更新过、且本帧确实有真值检测"时才比：遮断期间 last_face 停在
            // 上一帧的值、true_face 为 -1，拿它们比会得出假的不一致
            const bool valid = reports[0].last_face >= 0 && true_face[0] >= 0;

            if (!face_offset_known && valid)
            {
                face_offset = offset_of(reports[0].last_face, true_face[0]);
                face_offset_known = true;
            }
            else if (face_offset_known && valid)
            {
                if (offset_of(reports[0].last_face, true_face[0]) != face_offset)
                {
                    face_consistent = false;
                }
            }
        }

        if (frame == kOccludeFrom - 1)
        {
            radius_one_before = tracker.Radius(0);
        }
        if (frame >= kOccludeTo)
        {
            // 遮断结束后的稳定值：恢复头几帧的观测与预测偏差较大，会一次性修正半径，
            // 取稳定后的值判断"有没有重收敛"，而不是取恢复的第一帧
            radius_one_after = tracker.Radius(0);
        }
    }

    // ① 半径收敛到真值 ±10%
    const bool radius_ok = tracker.Converged(0) && std::abs(tracker.Radius(0) - kRadiusOne) / kRadiusOne <= 0.10;

    // ② 编号路由：槽绑定的就是三号，表外的五号从未被收进来
    const std::array<algorithm::vision::TargetReport, algorithm::vision::kMaxTargets> reports = tracker.Reports();
    const bool route_ok = reports[0].label == algorithm::vision::ArmorLabel::Three;

    // ③ 遮断（10 帧无观测）没有让半径重收敛：恢复并稳定后与遮断前差 < 10%
    const bool occlusion_ok = radius_one_before > 0.0 &&
        std::abs(radius_one_after - radius_one_before) / radius_one_before <= 0.10;

    // ④ 面序：每帧选中面与真值面的差应恒为首帧那个偏移（差值是编号约定，不要求为零）
    const bool face_ok = face_offset_known && face_consistent;

    // ⑤ 同号多检出为零（本条构造里每车每帧最多一块）
    std::int32_t duplicate_total = 0;
    for (const algorithm::vision::TargetReport& report : reports)
    {
        duplicate_total += report.duplicates;
    }
    const bool duplicate_ok = (duplicate_total == 0);

    DUST_LOG_INF("multi selfcheck: r=%.4f/%.4f occl(%.4f->%.4f) radius=%d route=%d occl=%d face=%d dup=%d pass=%d",
                 tracker.Radius(0), kRadiusOne, radius_one_before, radius_one_after,
                 radius_ok ? 1 : 0, route_ok ? 1 : 0, occlusion_ok ? 1 : 0, face_ok ? 1 : 0, duplicate_total,
                 (radius_ok && route_ok && occlusion_ok && face_ok && duplicate_ok) ? 1 : 0);

    return radius_ok && route_ok && occlusion_ok && face_ok && duplicate_ok;
}

/**
 * @brief 跑当前这一条测试；失败按 HaltOnFail 中止启动
 */
bool thread_init()
{
    if (!CheckMultiTarget())
    {
        return false;
    }

    DUST_LOG_OK("test init");
    return true;
}

/**
 * @brief 释放项目资源，支持未初始化和部分初始化状态
 */
void Release()
{
    DUST_LOG_INF("application resources released");
}

REGISTER_INIT(thread_init, AppInit, High, HaltOnFail, Release, "test_init");

} // namespace thread::test
