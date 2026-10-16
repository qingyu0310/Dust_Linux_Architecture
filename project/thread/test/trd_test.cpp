/**
 * @file trd_test.cpp
 * @author qingyu
 * @brief 测试线程：初始化注册与当前这一条测试
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "framework/algorithm/vision/pose/armor_pose.hpp"
#include "framework/init/Init_entry.hpp"
#include "framework/cmd/shell/log.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <optional>
#include <vector>

namespace thread::test
{
/**
 * @brief 用合成观测反解一遍单块装甲板位姿，验证物点、单位与 PnP 链
 *
 * 内参取本机标定值，按几组板姿态用 projectPoints 造出像素四点，再喂给 SolvePose。
 * 输入本来就是用这组物点生成的，解回去必须落回同一组数上：解不回来说明物点顺序、单位
 * 或 Rodrigues 用错了，跟相机没关系。
 *
 * 注意这一条**验不了板系的符号分配**：造样本和求解用的是同一份物点，y/z 谁在左谁在上
 * 都能自洽地回去。符号是否与实物板一致只能靠实拍（把板放在量好距离上比 ‖tvec‖）。
 *
 * @return true 反解位姿与设定值一致
 */
static bool CheckPose()
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

    const algorithm::vision::ArmorDimensions dimensions; // 缺省即实物尺寸
    const std::optional<std::array<cv::Point3f, 4>> object_points = algorithm::vision::MakeObjectPoints(algorithm::vision::ArmorSize::Small, dimensions);
    if (!object_points)
    {
        DUST_LOG_ERR("pose selfcheck: MakeObjectPoints returned nothing");
        return false;
    }

    const std::vector<cv::Point3f> object_list(object_points->begin(), object_points->end());
    const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << camera.fx, 0.0, camera.cx,
                                                                        0.0, camera.fy, camera.cy,
                                                                        0.0, 0.0, 1.0);
    const cv::Mat distort_coeffs = (cv::Mat_<double>(1, 5) << camera.k1, camera.k2, camera.p1, camera.p2, camera.k3);

    // 板心放在光轴附近的不同距离上，每组都带倾角：只靠正对光轴也能凑出观测，那样验不到倾斜。
    // 距离覆盖实拍用过的 0.3~0.5 m 与更远的 1.0~2.0 m——近处的角点离主点远，畸变位移大，
    // 若重投影只在近处涨，说明求解链没把畸变吃进去（若是网络角点噪声，近处远处都应一样）。
    const double poses[5][6] = {
        { 0.15,  0.10, 0.00, -0.03, -0.02, 0.3},
        { 0.15,  0.10, 0.00, -0.04, -0.03, 0.4},
        { 0.10,  0.10, 0.00, -0.05, -0.03, 0.5},
        { 0.00,  0.00, 0.00, -0.02, -0.01, 1.0},
        { 0.15, -0.20, 0.20, -0.04, -0.05, 2.0},
    };

    double tvec_error_max   = 0.0;
    double rotation_error_max = 0.0;
    double reprojection_max = 0.0;

    for (const auto& pose : poses)
    {
        const cv::Vec3d rvec_in(pose[0], pose[1], pose[2]);
        const cv::Vec3d tvec_in(pose[3], pose[4], pose[5]);

        std::vector<cv::Point2f> projected;
        cv::projectPoints(object_list, rvec_in, tvec_in, camera_matrix, distort_coeffs, projected);

        std::array<cv::Point2f, 4> corners {};
        std::copy(projected.begin(), projected.end(), corners.begin());

        const std::optional<algorithm::vision::ArmorPose> solved = algorithm::vision::SolvePose(corners, *object_points, camera);
        if (!solved)
        {
            DUST_LOG_ERR("pose selfcheck: SolvePose returned nothing");
            return false;
        }

        double tvec_error = 0.0;
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            tvec_error = std::max(tvec_error, std::abs(solved->tvec[axis] - tvec_in[axis]));
        }
        tvec_error_max = std::max(tvec_error_max, tvec_error);

        // 旋转量比矩阵而不是比 rvec 分量：同一旋转的 Rodrigues 表示可以有多种写法
        cv::Mat rotation_in;
        cv::Mat rotation_solved;
        cv::Rodrigues(rvec_in, rotation_in);
        cv::Rodrigues(solved->rvec, rotation_solved);
        rotation_error_max = std::max(rotation_error_max, cv::norm(rotation_in, rotation_solved, cv::NORM_INF));

        reprojection_max = std::max(reprojection_max, solved->reprojection_error_px);

        // 逐距离打一行：重投影随距离的变化就是本条的判据
        DUST_LOG_INF("pose selfcheck d=%.2fm tvec=%.3e m reprojection=%.3e px", tvec_in[2], tvec_error, solved->reprojection_error_px);
    }

    // 合成观测没有噪声，但角点存在 float 里：像素坐标 ~700 px 时尾数量化台阶约 4e-5 px，
    // 重投影的地板就是这个量级（实测 2.3e-5）。1e-3 m / 1e-6 / 1e-3 px 是拿来抓"物点顺序、
    // 单位、Rodrigues"那类量级 1e-1 的错，不是精度指标
    const bool passed = tvec_error_max < 1.0e-3 && rotation_error_max < 1.0e-6 && reprojection_max < 1.0e-3;

    DUST_LOG_INF("pose selfcheck %s: tvec=%.3e m rotation=%.3e reprojection=%.3e px", passed ? "ok" : "failed", tvec_error_max, rotation_error_max, reprojection_max);

    return passed;
}

/**
 * @brief 跑当前这一条测试；失败按 HaltOnFail 中止启动
 */
bool thread_init()
{
    if (!CheckPose())
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
