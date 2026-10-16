/**
 * @file armor_pose.cpp
 * @author qingyu
 * @brief 单块装甲板位姿实现 — 物点生成、IPPE 求解与反投校验
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "framework/algorithm/vision/pose/armor_pose.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <opencv2/calib3d.hpp>
#include <vector>

namespace algorithm::vision
{

namespace
{

/**
 * @brief 检查内参与畸变是否可参与投影
 *
 * fx、fy 必须为正，全部参数须为有限值；否则投影会产出 NaN 而不会被发现。
 *
 * @param camera 内参与畸变
 * @return true 可用于 solvePnP
 */
bool IsCameraUsable(const CameraModel& camera)
{
    return std::isfinite(camera.fx) && std::isfinite(camera.fy) && std::isfinite(camera.cx) && std::isfinite(camera.cy) &&
           std::isfinite(camera.k1) && std::isfinite(camera.k2) && std::isfinite(camera.p1) && std::isfinite(camera.p2) &&
           std::isfinite(camera.k3) && camera.fx > 0.0 && camera.fy > 0.0;
}

/**
 * @brief 拼出 3x3 内参矩阵，行优先
 */
cv::Mat CameraMatrix(const CameraModel& camera)
{
    return (cv::Mat_<double>(3, 3) << camera.fx, 0.0,       camera.cx,
                                                  0.0,      camera.fy, camera.cy,
                                                  0.0,      0.0,        1.0);
}

/**
 * @brief 拼出 1x5 畸变系数，顺序 k1、k2、p1、p2、k3
 */
cv::Mat DistortCoeffs(const CameraModel& camera)
{
    return (cv::Mat_<double>(1, 5) << camera.k1, camera.k2, camera.p1, camera.p2, camera.k3);
}

/**
 * @brief 检查四个点是否都是有限值
 */
bool AllFinite(const std::array<cv::Point2f, 4>& points)
{
    return std::all_of(points.begin(), points.end(), [](const cv::Point2f& point)
    {
        return std::isfinite(point.x) && std::isfinite(point.y);
    });
}

/**
 * @brief 四点平均误差，反投与实测逐点配对
 */
double MeanReprojectionError(const std::array<cv::Point2f, 4>& measured, const std::vector<cv::Point2f>& projected)
{
    double total = 0.0;
    for (std::size_t index = 0; index < measured.size(); ++index)
    {
        const cv::Point2f delta = measured[index] - projected[index];
        total += std::sqrt(delta.dot(delta));
    }

    return total / static_cast<double>(measured.size());
}

/**
 * @brief Rodrigues 旋转向量转 Eigen 3x3 矩阵
 */
Eigen::Matrix3d ToEigenRotation(const cv::Vec3d& rvec)
{
    cv::Matx33d rotation;
    cv::Rodrigues(cv::Mat(rvec), rotation);

    Eigen::Matrix3d result;
    for (std::int32_t row = 0; row < 3; ++row)
    {
        for (std::int32_t col = 0; col < 3; ++col)
        {
            result(row, col) = rotation(row, col);
        }
    }
    return result;
}

/**
 * @brief Eigen 3x3 矩阵转 Rodrigues 旋转向量
 */
cv::Vec3d ToRotationVector(const Eigen::Matrix3d& rotation)
{
    cv::Matx33d matrix;
    for (std::int32_t row = 0; row < 3; ++row)
    {
        for (std::int32_t col = 0; col < 3; ++col)
        {
            matrix(row, col) = rotation(row, col);
        }
    }

    cv::Vec3d rvec;
    cv::Rodrigues(cv::Mat(matrix), rvec);
    return rvec;
}

/**
 * @brief 外参旋转向量转 3x3 旋转矩阵
 */
Eigen::Matrix3d ExtrinsicRotation(const GimbalExtrinsics& extrinsics)
{
    return ToEigenRotation(extrinsics.rotation);
}

} // namespace

/**
 * @brief 相机系点 → 云台系点
 */
Eigen::Vector3d CameraToGimbal(const Eigen::Vector3d& point, const GimbalExtrinsics& extrinsics)
{
    const Eigen::Vector3d translation(extrinsics.translation[0], extrinsics.translation[1], extrinsics.translation[2]);
    return ExtrinsicRotation(extrinsics) * point + translation;
}

/**
 * @brief 相机系方向 → 云台系方向，只转不平移
 */
Eigen::Vector3d CameraToGimbalDirection(const Eigen::Vector3d& direction, const GimbalExtrinsics& extrinsics)
{
    return ExtrinsicRotation(extrinsics) * direction;
}

/**
 * @brief 云台系点 → 相机系点
 */
Eigen::Vector3d GimbalToCamera(const Eigen::Vector3d& point, const GimbalExtrinsics& extrinsics)
{
    const Eigen::Vector3d translation(extrinsics.translation[0], extrinsics.translation[1], extrinsics.translation[2]);
    return ExtrinsicRotation(extrinsics).transpose() * (point - translation);
}

/**
 * @brief 云台系方向 → 相机系方向，只转不平移
 */
Eigen::Vector3d GimbalToCameraDirection(const Eigen::Vector3d& direction, const GimbalExtrinsics& extrinsics)
{
    return ExtrinsicRotation(extrinsics).transpose() * direction;
}

/**
 * @brief 把一块装甲板的姿态从云台系表达换到相机系表达
 */
cv::Vec3d GimbalToCameraRotation(const cv::Vec3d& gimbal_rvec, const GimbalExtrinsics& extrinsics)
{
    const Eigen::Matrix3d camera_from_armor = ExtrinsicRotation(extrinsics).transpose() * ToEigenRotation(gimbal_rvec);
    return ToRotationVector(camera_from_armor);
}

/**
 * @brief 由板法向转角与板面倾角拼出板系到相机系的旋转向量
 *
 * 车的旋转是绕**竖直轴**，而相机系 y 向下，所以水平面是相机系的 x-z 平面，转角从 +x 向
 * +z 量。三个列向量依次是板系 x（法向 n）、y（宽度 w）、z（灯条 l）：
 * 法向在水平面内取 `n = (cos a, 0, sin a)`，面心为 `中心 − r·n`（与参考 `target.cpp` 的
 * `center_x = xyz[0] + r·cos(a)` 一致）；灯条竖直取 `l = (0, −1, 0)`（相机 y 向下，“向上”
 * 是 −y），宽度由右手系 `w = l × n = (−sin a, 0, cos a)` 定出。pitch 是板子绕**自身宽度轴**
 * 的倾角，直接右乘在板系里。
 *
 * @param yaw_rad 板法向在水平面内的方位角，单位 rad
 * @param pitch_rad 板面绕宽度轴的倾角，单位 rad
 * @return 板系到相机系的 Rodrigues 旋转向量
 */
cv::Vec3d ArmorRotation(double yaw_rad, double pitch_rad)
{
    const double cos_yaw   = std::cos(yaw_rad);
    const double sin_yaw   = std::sin(yaw_rad);
    const double cos_pitch = std::cos(pitch_rad);
    const double sin_pitch = std::sin(pitch_rad);

    // 列向量 [法向 n, 宽度 w, 灯条 l]，w = l × n
    const cv::Matx33d base( cos_yaw, -sin_yaw, 0.0,
                            0.0,      0.0,    -1.0,
                            sin_yaw,  cos_yaw, 0.0);

    // 绕板系宽度轴（列 1）的倾角
    const cv::Matx33d tilt(cos_pitch, 0.0, sin_pitch,
                           0.0, 1.0, 0.0,
                           -sin_pitch, 0.0, cos_pitch);

    cv::Vec3d rvec;
    cv::Rodrigues(cv::Mat(base * tilt), rvec);
    return rvec;
}

/**
 * @brief 按装甲板尺寸生成板系物点，顺序固定 LT、RT、RB、LB
 *
 * @param size 装甲板大小；Unknown 表示尺寸不明，返回 std::nullopt 而不是猜一个
 * @param dimensions 实物尺寸，单位 m
 * @return 四个物点，顺序与检测结果的角点顺序一致
 */
std::optional<std::array<cv::Point3f, 4>> MakeObjectPoints(ArmorSize size, const ArmorDimensions& dimensions)
{
    double width = 0.0;
    switch (size)
    {
        case ArmorSize::Small:
            width = dimensions.small_width_m;
            break;
        case ArmorSize::Large:
            width = dimensions.big_width_m;
            break;
        default:
            return std::nullopt; // 尺寸不明就不解：拿错宽度解出的尺度会系统性偏大或偏小
    }

    const double length = dimensions.lightbar_length_m;
    if (!std::isfinite(width) || !std::isfinite(length) || width <= 0.0 || length <= 0.0)
    {
        return std::nullopt;
    }

    const float half_width  = static_cast<float>(width * 0.5);
    const float half_length = static_cast<float>(length * 0.5);

    // 板系 x 为法向，四点的 x 都是 0；y 沿宽度、z 沿灯条方向
    return std::array<cv::Point3f, 4> {{
        cv::Point3f(0.0f, +half_width, +half_length), // LT
        cv::Point3f(0.0f, -half_width, +half_length), // RT
        cv::Point3f(0.0f, -half_width, -half_length), // RB
        cv::Point3f(0.0f, +half_width, -half_length), // LB
    }};
}

/**
 * @brief 由像素四点解出相机系位姿，并把结果反投回像素计算误差
 *
 * @param corners 图像四点，顺序必须与 object_points 的 LT、RT、RB、LB 一致
 * @param object_points 板系物点，单位 m
 * @param camera 内参与畸变
 * @return 位姿与反投结果；角点非有限、内参非法或求解失败时返回 std::nullopt
 */
std::optional<ArmorPose> SolvePose(const std::array<cv::Point2f, 4>& corners, const std::array<cv::Point3f, 4>& object_points, const CameraModel& camera)
{
    if (!IsCameraUsable(camera) || !AllFinite(corners))
    {
        return std::nullopt;
    }

    const cv::Mat camera_matrix = CameraMatrix(camera);
    const cv::Mat distort       = DistortCoeffs(camera);

    const std::vector<cv::Point3f> object_list(object_points.begin(), object_points.end());
    const std::vector<cv::Point2f> image_list(corners.begin(), corners.end());

    cv::Vec3d rvec {};
    cv::Vec3d tvec {};
    // useExtrinsicGuess = false：每帧独立解，不继承上一帧；共面四点用 IPPE
    if (!cv::solvePnP(object_list, image_list, camera_matrix, distort, rvec, tvec, false, cv::SOLVEPNP_IPPE))
    {
        return std::nullopt;
    }

    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_list, rvec, tvec, camera_matrix, distort, projected);
    if (projected.size() != object_points.size())
    {
        return std::nullopt;
    }

    ArmorPose pose;
    pose.rvec = rvec;
    pose.tvec = tvec;
    pose.reprojection_error_px = MeanReprojectionError(corners, projected);
    std::copy(projected.begin(), projected.end(), pose.projected.begin());

    return pose;
}

/**
 * @brief 把板系位姿反投回像素四点
 *
 * @param object_points 板系物点，单位 m
 * @param rvec 板系到相机系的旋转向量
 * @param tvec 板心在相机系的位置，单位 m
 * @param camera 内参与畸变
 * @return 反投的四点；内参或位姿含非有限值时返回 std::nullopt
 */
std::optional<std::array<cv::Point2f, 4>> ProjectPose(const std::array<cv::Point3f, 4>& object_points, const cv::Vec3d& rvec, const cv::Vec3d& tvec, const CameraModel& camera)
{
    if (!IsCameraUsable(camera) || !std::isfinite(tvec[0]) || !std::isfinite(tvec[1]) || !std::isfinite(tvec[2]))
    {
        return std::nullopt;
    }

    const std::vector<cv::Point3f> object_list(object_points.begin(), object_points.end());
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_list, rvec, tvec, CameraMatrix(camera), DistortCoeffs(camera), projected);
    if (projected.size() != object_points.size())
    {
        return std::nullopt;
    }

    std::array<cv::Point2f, 4> result {};
    std::copy(projected.begin(), projected.end(), result.begin());

    return result;
}

} // namespace algorithm::vision
