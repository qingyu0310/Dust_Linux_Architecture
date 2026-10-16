/**
 * @file armor_pose.hpp
 * @author qingyu
 * @brief 单块装甲板的相机系位姿 — 板系约定、物点生成与 PnP 反投
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "framework/algorithm/vision/core/armor_types.hpp"

#include <Eigen/Dense>
#include <array>
#include <opencv2/core.hpp>
#include <optional>

namespace algorithm::vision
{

// 板系：x 为板法向、y 沿装甲板宽度、z 沿灯条方向，原点在板心，板面即 x = 0。
// 四角点按图像的 LT、RT、RB、LB 顺序与物点配对，W 取小/大装甲板宽度、L 取灯条长度，单位一律 m：
//
//   LT = {0, +W/2, +L/2}    RT = {0, -W/2, +L/2}
//   RB = {0, -W/2, -L/2}    LB = {0, +W/2, -L/2}
//
// 符号分配按"图像 LT 在左上、RT 在右上"这一假定。y 或 z 写反会让重投影大一个量级，x 写反只换
// yaw 分支（IPPE 对共面四点本来就有两个解），所以合成自检验不出符号分配与实物板是否一致。
// 尺寸是实物量，取自参考 solver.cpp:14-16；运行值以 trd_vision.yaml 为准，这里只是缺省。

constexpr double kSmallArmorWidthM = 0.135;   // 小装甲板宽度，单位 m
constexpr double kBigArmorWidthM   = 0.230;   // 大装甲板宽度，单位 m
constexpr double kLightBarLengthM  = 0.056;   // 灯条长度，单位 m

/**
 * @brief 装甲板实物尺寸，由调用方从线程 YAML 读入
 */
struct ArmorDimensions
{
    double small_width_m     {kSmallArmorWidthM};  // 小装甲板宽度，单位 m
    double big_width_m       {kBigArmorWidthM};    // 大装甲板宽度，单位 m
    double lightbar_length_m {kLightBarLengthM};   // 灯条长度，单位 m
};

/**
 * @brief 针孔内参与径向、切向畸变
 */
struct CameraModel
{
    double fx {0.0};    // x 方向焦距，单位 pixel
    double fy {0.0};    // y 方向焦距，单位 pixel
    double cx {0.0};    // 主点横坐标，单位 pixel
    double cy {0.0};    // 主点纵坐标，单位 pixel
    double k1 {0.0};    // 径向畸变一阶系数
    double k2 {0.0};    // 径向畸变二阶系数
    double p1 {0.0};    // 切向畸变一阶系数
    double p2 {0.0};    // 切向畸变二阶系数
    double k3 {0.0};    // 径向畸变三阶系数
};

/**
 * @brief 相机相对云台的外参：把相机系的点与向量换到云台系
 *
 * `p_gimbal = R_camera2gimbal · p_camera + t_camera2gimbal`，与参考 `solver.cpp` 一致。
 * 本工程还没有云台设备层、也没做手眼标定，所以默认是单位阵/零平移——此时"云台系"与
 * "相机系"数值上完全相同，跟踪器照常工作。等有了云台、标定出这 12 个数，填进 YAML 即可
 * 让跟踪器工作在云台系，不必改动算法。
 */
struct GimbalExtrinsics
{
    cv::Vec3d rotation {};    // R_camera2gimbal 的 Rodrigues 形式，单位阵时为 (0,0,0)
    cv::Vec3d translation {}; // t_camera2gimbal，单位 m，须与点坐标单位一致
};

/**
 * @brief 单块装甲板的相机系位姿与反投校验
 */
struct ArmorPose
{
    cv::Vec3d rvec {};                       // 板系到相机系的旋转向量，Rodrigues 形式
    cv::Vec3d tvec {};                       // 板心在相机系的位置，单位 m
    double reprojection_error_px {0.0};      // 四点反投回像素的平均误差，单位 pixel
    std::array<cv::Point2f, 4> projected {}; // 反投的四点，顺序与输入 corner 相同
};

/**
 * @brief 相机系点 → 云台系点
 *
 * @param point 相机系下的点，单位 m
 * @param extrinsics 相机相对云台的外参
 * @return 云台系下的点，单位 m
 */
Eigen::Vector3d CameraToGimbal(const Eigen::Vector3d& point, const GimbalExtrinsics& extrinsics);

/**
 * @brief 相机系方向 → 云台系方向，只转不平移
 *
 * @param direction 相机系下的方向向量
 * @param extrinsics 相机相对云台的外参
 * @return 云台系下的方向向量
 */
Eigen::Vector3d CameraToGimbalDirection(const Eigen::Vector3d& direction, const GimbalExtrinsics& extrinsics);

/**
 * @brief 云台系点 → 相机系点
 *
 * @param point 云台系下的点，单位 m
 * @param extrinsics 相机相对云台的外参
 * @return 相机系下的点，单位 m
 */
Eigen::Vector3d GimbalToCamera(const Eigen::Vector3d& point, const GimbalExtrinsics& extrinsics);

/**
 * @brief 云台系方向 → 相机系方向，只转不平移
 *
 * @param direction 云台系下的方向向量
 * @param extrinsics 相机相对云台的外参
 * @return 相机系下的方向向量
 */
Eigen::Vector3d GimbalToCameraDirection(const Eigen::Vector3d& direction, const GimbalExtrinsics& extrinsics);

/**
 * @brief 把一块装甲板的姿态从云台系表达换到相机系表达
 *
 * 板系到云台系的旋转 `R_ag` 换成板系到相机系的 `R_ac = R_cg⁻¹ · R_ag`，只用到外参的旋转
 * 部分（平移对方向无影响）。跟踪器的状态在云台系，投影前要用它把姿态一并换过去。
 *
 * @param gimbal_rvec 板系到云台系的旋转向量
 * @param extrinsics 相机相对云台的外参
 * @return 板系到相机系的旋转向量
 */
cv::Vec3d GimbalToCameraRotation(const cv::Vec3d& gimbal_rvec, const GimbalExtrinsics& extrinsics);

/**
 * @brief 由板法向转角与板面倾角拼出板系到相机系的旋转向量
 *
 * 相机系 x 向右、y 向下、z 向前。板系 x 为法向、y 沿宽度、z 沿灯条方向（见文件头约定）。
 * `yaw_rad` 是板法向在水平面（相机系 x-z 平面）内的方位角，从 +z 向 +x 量；`pitch_rad` 是
 * 板面绕宽度轴的倾角，板子正立时为 0。
 *
 * 跨帧跟踪器用它把状态里的一个转角变成位姿，自检用它造观测，两边必须是同一份约定。
 *
 * @param yaw_rad 板法向在水平面内的方位角，单位 rad
 * @param pitch_rad 板面绕宽度轴的倾角，单位 rad
 * @return 板系到相机系的 Rodrigues 旋转向量
 */
cv::Vec3d ArmorRotation(double yaw_rad, double pitch_rad);

/**
 * @brief 按装甲板尺寸生成板系物点，顺序固定 LT、RT、RB、LB
 *
 * @param size 装甲板大小；Unknown 表示尺寸不明，返回 std::nullopt 而不是猜一个
 * @param dimensions 实物尺寸，单位 m
 * @return 四个物点，顺序与检测结果的角点顺序一致
 */
std::optional<std::array<cv::Point3f, 4>> MakeObjectPoints(ArmorSize size, const ArmorDimensions& dimensions);

/**
 * @brief 由像素四点解出相机系位姿，并把结果反投回像素计算误差
 *
 * 每帧独立解，不继承上一帧；四点共面用 IPPE。位姿只有 6 个自由度而观测有 8 个方程，
 * 冗余很小，精度完全取决于物点尺寸与角点。
 *
 * @param corners 图像四点，顺序必须与 object_points 的 LT、RT、RB、LB 一致
 * @param object_points 板系物点，单位 m
 * @param camera 内参与畸变
 * @return 位姿与反投结果；角点非有限、内参非法或求解失败时返回 std::nullopt
 */
std::optional<ArmorPose> SolvePose(const std::array<cv::Point2f, 4>& corners, const std::array<cv::Point3f, 4>& object_points, const CameraModel& camera);

/**
 * @brief 把板系位姿反投回像素四点
 *
 * 供绘制与跨帧关联使用：已有一位姿时不必再解一遍。
 *
 * @param object_points 板系物点，单位 m
 * @param rvec 板系到相机系的旋转向量
 * @param tvec 板心在相机系的位置，单位 m
 * @param camera 内参与畸变
 * @return 反投的四点；内参或位姿含非有限值时返回 std::nullopt
 */
std::optional<std::array<cv::Point2f, 4>> ProjectPose(const std::array<cv::Point3f, 4>& object_points, const cv::Vec3d& rvec, const cv::Vec3d& tvec, const CameraModel& camera);

} // namespace algorithm::vision
