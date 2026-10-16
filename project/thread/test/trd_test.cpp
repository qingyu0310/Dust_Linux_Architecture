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

#include "framework/algorithm/vision/pose/armor_pose.hpp"
#include "framework/algorithm/vision/track/armor_tracker.hpp"
#include "framework/init/Init_entry.hpp"
#include "framework/cmd/shell/log.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <optional>

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
 * @brief 造一帧观测：一辆车以给定半径和角速度自转，取面 0 的四点
 *
 * 车的旋转中心固定在相机系原点前方，只有转角随时间推进，所以真实半径是可以对答案的。
 *
 * @param camera 内参与畸变
 * @param dimensions 板尺寸
 * @param yaw_rad 当帧面 0 的转角
 * @param radius_m 真实旋转半径
 * @param center 旋转中心，相机系，单位 m
 * @param corners 输出四点
 * @return true 生成成功
 */
bool MakeObservation(const algorithm::vision::CameraModel& camera, const algorithm::vision::ArmorDimensions& dimensions,
                     double yaw_rad, double radius_m, const Eigen::Vector3d& center, std::array<cv::Point2f, 4>& corners)
{
    const std::optional<std::array<cv::Point3f, 4>> object_points = algorithm::vision::MakeObjectPoints(algorithm::vision::ArmorSize::Small, dimensions);
    if (!object_points)
    {
        return false;
    }

    // 面 0 的中心 = 旋转中心 + r·法向，法向 n = (−cos a, 0, −sin a)，与跟踪器同一约定
    const cv::Vec3d tvec(center.x() - radius_m * std::cos(yaw_rad),
                         center.y(),
                         center.z() - radius_m * std::sin(yaw_rad));

    const std::optional<std::array<cv::Point2f, 4>> points = algorithm::vision::ProjectPose(*object_points, algorithm::vision::ArmorRotation(yaw_rad, 0.0), tvec, camera);
    if (!points)
    {
        return false;
    }

    corners = *points;
    return true;
}

} // namespace

/**
 * @brief 合成一串带已知半径的自转观测，喂给跟踪器，看半径能否收敛
 *
 * 纯离线：观测由 projectPoints 造出，没有相机、没有噪声。真实半径 0.20 m、角速度 2 rad/s、
 * 30 fps，与规划 §2 算"17 帧"用的工况一致；半径初值故意给到 0.35 m（偏离 75%），
 * 只有观测真的约束了半径才收得回来。
 *
 * @return true 半径收敛到真值 ±10% 且帧数不超过 17
 */
static bool CheckTracker()
{
    const algorithm::vision::CameraModel camera = MakeCamera();
    const algorithm::vision::ArmorDimensions dimensions;

    constexpr double kTrueRadius    = 0.20;         // 真实半径，单位 m
    constexpr double kOmega         = 2.0;          // 角速度，单位 rad/s
    constexpr double kDt            = 1.0 / 30.0;
    constexpr double kTolerance     = 0.10;         // 收敛判据：±10%
    constexpr int32_t kAcceptFrames = 17;           // 验收帧数上限（规划 §2 算的界）
    constexpr int32_t kMaxFrames    = 60;           // 诊断窗口：先跑到 60 帧看是收敛慢还是卡住

    algorithm::vision::ArmorTrackerConfig config;
    config.radius_init_m = 0.35;                    // 故意给偏，验证观测能把它拉回来

    // 合成观测由 projectPoints 直接造出，没有噪声；点噪声按"无噪声"给。
    // 若沿用真实的 8 px，半径在 3 m 处的可观测性（约 0.6 px/m）会被噪声方差淹没、几乎收不动
    config.point_sigma_px = 0.5;

    // 自检的车心静止、转速恒定，是光滑运动；过程噪声给参考量级会让滤波器拿"中心平移"
    // 去解释弧线运动，半径就修不动。这里按光滑运动给极小的加速度方差：车心确实不动，
    // 就把它钉住，逼滤波器只能去修半径（真机需按实测重定）
    config.accel_variance = 1.0e-4;
    config.angular_accel_variance = 1.0e-2;

    algorithm::vision::ArmorTracker tracker(config);

    const Eigen::Vector3d center(0.05, 0.02, 3.0);

    std::int32_t converged_frame = -1;
    for (std::int32_t frame = 0; frame < kMaxFrames; ++frame)
    {
        const double yaw = kOmega * static_cast<double>(frame) * kDt;

        std::array<cv::Point2f, 4> corners {};
        if (!MakeObservation(camera, dimensions, yaw, kTrueRadius, center, corners))
        {
            DUST_LOG_ERR("tracker selfcheck: cannot build observation");
            return false;
        }

        if (!tracker.Update(corners, algorithm::vision::ArmorSize::Small, dimensions, camera, kDt))
        {
            DUST_LOG_ERR("tracker selfcheck: update rejected at frame %d", frame);
            return false;
        }

        const double radius = tracker.Radius();
        const double relative_error = std::abs(radius - kTrueRadius) / kTrueRadius;
        DUST_LOG_INF("tracker selfcheck frame=%d radius=%.4f m error=%.1f%% a=%.3f w=%.3f truth_a=%.3f", frame, radius, relative_error * 100.0, tracker.Angle(), tracker.Omega(), yaw);

        if (relative_error <= kTolerance)
        {
            converged_frame = frame + 1;
            break;
        }
    }

    if (converged_frame < 0)
    {
        DUST_LOG_ERR("tracker selfcheck failed: radius did not converge within %d frames (final %.4f m, truth %.4f m)", kMaxFrames, tracker.Radius(), kTrueRadius);
        return false;
    }

    DUST_LOG_INF("tracker selfcheck: radius %.4f m within %.0f%% after %d frames (limit %d)", tracker.Radius(), kTolerance * 100.0, converged_frame, kAcceptFrames);
    return converged_frame <= kAcceptFrames;
}

/**
 * @brief 跑当前这一条测试；失败按 HaltOnFail 中止启动
 */
bool thread_init()
{
    if (!CheckTracker())
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
