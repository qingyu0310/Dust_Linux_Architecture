/**
 * @file trd_calib_sample.cpp
 * @author qingyu
 * @brief 标定线程样本处理 — 圆心标记、内参求解、三项验收与结果打印
 * @version 0.1
 * @date 2026-10-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_calib.hpp"
#include "framework/cmd/shell/log/log.hpp"
#include "framework/init/startup/System_startup.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <opencv2/imgproc.hpp>
#include <optional>

namespace thread::calib
{

using algorithm::calib::Intrinsics;

// 主点相对图像几何中心的允许偏移比例；参考那台偏移 26.09 px，合 0.024·H
constexpr double kMaxPrincipalOffsetRatio = 0.05;

// fx 与 fy 的允许相对偏差；方形像素下应为 1，参考那台是 1.00044
constexpr double kMaxFocalRatioDeviation = 0.01;

// 圆心标记的半径与线宽，单位 pixel
constexpr std::int32_t kMarkerRadiusPx    = 8;
constexpr std::int32_t kMarkerThicknessPx = 2;

// 圆心标记按行取色：行序或列序反了（转置）时颜色分布会立刻不对
const std::array<cv::Scalar, 6> kRowColors {{{0, 0, 255}, {0, 200, 255}, {0, 255, 0}, {255, 200, 0}, {255, 0, 0}, {255, 0, 200}}};

/**
 * @brief 在每个认出来的圆心上套一个圈，画在副本上
 *
 * 圈按行换色：圆点阵的行序或列序要是反了（转置），颜色分布会立刻不对，比纯看位置容易发现。
 * 画的是 clone——通道里的像素与显示侧共享，约定要求改图的一方自己复制。
 *
 * @param image 相机帧，整幅 BGR
 * @param centers 本帧识别到的圆心，识别失败时为空
 * @return 叠加好圆心的整幅图像
 */
cv::Mat DrawCenters(const cv::Mat& image, const std::vector<cv::Point2f>& centers)
{
    cv::Mat drawn = image.clone();

    const std::size_t cols = static_cast<std::size_t>(params_.board.cols);
    for (std::size_t index = 0; index < centers.size(); ++index)
    {
        cv::circle(drawn, centers[index], kMarkerRadiusPx, kRowColors[(index / cols) % kRowColors.size()], kMarkerThicknessPx, cv::LINE_AA);
    }

    return drawn;
}

/**
 * @brief 按保留到的样本反解内参、畸变与重投影误差
 *
 * @param intrinsics 输出的求解结果
 * @return true 求解成功
 */
static bool Solve(Intrinsics& intrinsics)
{
    const std::optional<Intrinsics> solved = algorithm::calib::SolveIntrinsics(params_.board, samples_.centers, samples_.image_size);
    if (!solved)
    {
        DUST_LOG_ERR("calib: solve failed, samples=%zu image=%dx%d", samples_.centers.size(), samples_.image_size.width, samples_.image_size.height);
        return false;
    }

    intrinsics = *solved;
    return true;
}

/**
 * @brief 三项验收：重投影误差、方形像素、主点位置
 *
 * 门限从参考那台的数字来：重投影 0.0595 px、fx/fy = 1.00044、主点偏移 0.024·H。
 * 任一项不过都说明这批样本不足以解出可信内参，宁可判失败也不报出坏值。
 *
 * @param intrinsics 求解结果
 * @return true 三项都在界内
 */
static bool VerifyIntrinsics(const Intrinsics& intrinsics)
{
    if (!(intrinsics.reprojection_error_px <= params_.max_reprojection_error_px))
    {
        DUST_LOG_ERR("calib: reprojection_error %.4fpx exceeds %.4fpx", intrinsics.reprojection_error_px, params_.max_reprojection_error_px);
        return false;
    }

    const double fx = intrinsics.camera_matrix[0];
    const double fy = intrinsics.camera_matrix[4];
    if (!(fy > 0.0) || std::abs(fx / fy - 1.0) > kMaxFocalRatioDeviation)
    {
        DUST_LOG_ERR("calib: fx=%.4f fy=%.4f deviate more than %.2f", fx, fy, kMaxFocalRatioDeviation);
        return false;
    }

    const double x_limit = static_cast<double>(intrinsics.image_width) * kMaxPrincipalOffsetRatio;
    const double y_limit = static_cast<double>(intrinsics.image_height) * kMaxPrincipalOffsetRatio;
    if (std::abs(intrinsics.camera_matrix[2] - static_cast<double>(intrinsics.image_width) * 0.5) > x_limit ||
        std::abs(intrinsics.camera_matrix[5] - static_cast<double>(intrinsics.image_height) * 0.5) > y_limit)
    {
        DUST_LOG_ERR("calib: principal point (%.2f,%.2f) is too far from the image centre", intrinsics.camera_matrix[2], intrinsics.camera_matrix[5]);
        return false;
    }

    return true;
}

/**
 * @brief 按配置文件的键名打印一次标定结果，好让人直接复制粘贴
 *
 * 每条日志在彩色模式下只有约 101 字节可用，所以一行一个键，不往一行里堆。
 *
 * @param intrinsics 求解结果
 */
static void LogIntrinsics(const Intrinsics& intrinsics)
{
    DUST_LOG_INF("calib: image_width: %d", intrinsics.image_width);
    DUST_LOG_INF("calib: image_height: %d", intrinsics.image_height);
    DUST_LOG_INF("calib: camera_fx: %.10g", intrinsics.camera_matrix[0]);
    DUST_LOG_INF("calib: camera_fy: %.10g", intrinsics.camera_matrix[4]);
    DUST_LOG_INF("calib: camera_cx: %.10g", intrinsics.camera_matrix[2]);
    DUST_LOG_INF("calib: camera_cy: %.10g", intrinsics.camera_matrix[5]);
    DUST_LOG_INF("calib: distort_k1: %.10g", intrinsics.distort_coeffs[0]);
    DUST_LOG_INF("calib: distort_k2: %.10g", intrinsics.distort_coeffs[1]);
    DUST_LOG_INF("calib: distort_p1: %.10g", intrinsics.distort_coeffs[2]);
    DUST_LOG_INF("calib: distort_p2: %.10g", intrinsics.distort_coeffs[3]);
    DUST_LOG_INF("calib: distort_k3: %.10g", intrinsics.distort_coeffs[4]);
    DUST_LOG_INF("calib: reprojection_error_px: %.6g", intrinsics.reprojection_error_px);
}

/**
 * @brief 保留满样本后求解、验收并打印一次
 *
 * 只在第一次收满时执行：验收不过就请求统一停止（这批样本解不出可信内参，继续转着也没意义），
 * 通过则打印 12 个键供人粘进视觉线程的 YAML，线程继续转着让画面留在屏幕上。
 */
void FinishIfComplete()
{
    if (samples_.solved || static_cast<std::int32_t>(samples_.centers.size()) < params_.sample_count)
    {
        return;
    }
    samples_.solved = true;

    Intrinsics intrinsics;
    if (!Solve(intrinsics) || !VerifyIntrinsics(intrinsics))
    {
        System_RequestStop(1);
        return;
    }

    LogIntrinsics(intrinsics);
    DUST_LOG_OK("calib: paste the keys above into thread/vision/trd_vision.yaml");
}

} // namespace thread::calib
