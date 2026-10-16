/**
 * @file trd_test.cpp
 * @author qingyu
 * @brief 测试线程：初始化注册与当前这一条测试
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "framework/algorithm/calib/camera_calib.hpp"
#include <algorithm>
#include "framework/init/Init_entry.hpp"
#include "framework/cmd/shell/log.hpp"
#include <cmath>
#include <cstddef>
#include <opencv2/calib3d.hpp>
#include <optional>
#include <utility>
#include <vector>

namespace thread::test
{
/**
 * @brief 用合成观测反解一遍内参，验证算法层的求解链
 *
 * 内参与畸变取参考那台相机的值（同一 USB 型号、同一 1440×1080），按六组板姿态用
 * projectPoints 造出圆心像素坐标，再喂给 SolveIntrinsics。输入本来就是用这组内参生成的，
 * 解回去必须落在同一组数上；解不回去说明点序、单位或 flag 写错了，跟相机没关系。
 *
 * 板点在这里重新排一遍而不去调算法层的内部函数：合成样本和求解器各排一次，顺序约定
 * 写错才暴露得出来。真机上的点序（findCirclesGrid 的返回顺序）只能靠实际标定验证。
 *
 * @return true 反解结果与设定值一致
 */
static bool CheckCalibration()
{
    const algorithm::calib::Board board;
    const cv::Size image_size(1440, 1080);

    const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 1808.8304696430803, 0.0, 713.70459283498224,
                                                          0.0, 1808.0411873540195, 566.09347726083934,
                                                          0.0, 0.0, 1.0);
    const cv::Mat distort_coeffs = (cv::Mat_<double>(1, 5) << -0.073241403353599299, 0.14495759982904788,
                                                             0.00028937879061362732, -0.00026770913811837289, 0.0);

    std::vector<cv::Point3f> board_points;
    board_points.reserve(static_cast<std::size_t>(board.cols) * static_cast<std::size_t>(board.rows));
    for (std::int32_t row = 0; row < board.rows; ++row)
    {
        for (std::int32_t col = 0; col < board.cols; ++col)
        {
            board_points.emplace_back(static_cast<float>(col * board.center_distance_mm), static_cast<float>(row * board.center_distance_mm), 0.0f);
        }
    }

    // 板心先挪到光轴附近，z 取 700–950 mm（与规划的 0.5–1.0 m 工作距离同量级）；
    // 每组都带倾角，只靠平移也能凑出观测，那样验不到倾斜方向的约束
    const double poses[6][6] = {
        { 0.00,  0.00, 0.00, -160.0, -100.0, 700.0},
        { 0.30,  0.30, 0.00, -160.0, -100.0, 800.0},
        {-0.30,  0.30, 0.00, -160.0, -100.0, 800.0},
        {-0.30, -0.30, 0.00, -160.0, -100.0, 900.0},
        { 0.30, -0.30, 0.00, -160.0, -100.0, 900.0},
        { 0.45,  0.15, 0.20, -160.0, -100.0, 950.0},
    };

    std::vector<std::vector<cv::Point2f>> image_points;
    image_points.reserve(6);
    for (const auto& pose : poses)
    {
        const cv::Vec3d rotation(pose[0], pose[1], pose[2]);
        const cv::Vec3d translation(pose[3], pose[4], pose[5]);
        std::vector<cv::Point2f> projected;
        cv::projectPoints(board_points, rotation, translation, camera_matrix, distort_coeffs, projected);
        image_points.push_back(std::move(projected));
    }

    const std::optional<algorithm::calib::Intrinsics> solved = algorithm::calib::SolveIntrinsics(board, image_points, image_size);
    if (!solved)
    {
        DUST_LOG_ERR("calib selfcheck: SolveIntrinsics returned nothing");
        return false;
    }

    // 结构项（0 和 1）没有相对误差可算，分母统一取 max(|期望|, 1)；畸变系数一律按绝对值比
    double matrix_error = 0.0;
    for (std::int32_t row = 0; row < 3; ++row)
    {
        for (std::int32_t col = 0; col < 3; ++col)
        {
            const double expected = camera_matrix.at<double>(row, col);
            matrix_error = std::max(matrix_error, std::abs(solved->camera_matrix[row * 3 + col] - expected) / std::max(std::abs(expected), 1.0));
        }
    }

    double coefficient_error = 0.0;
    for (std::int32_t index = 0; index < 5; ++index)
    {
        coefficient_error = std::max(coefficient_error, std::abs(solved->distort_coeffs[index] - distort_coeffs.at<double>(0, index)));
    }

    // 合成观测没有噪声，但圆心存成 float，量化台阶约 1e-4 px，反解误差的地板在 1e-6 量级；
    // 1e-3 是拿来抓"点序/单位/flag 写错"那类量级 1e-1 的错，不是精度指标
    const bool passed = matrix_error < 1.0e-3 && coefficient_error < 1.0e-3 && solved->reprojection_error_px < 1.0e-3;

    DUST_LOG_INF("calib selfcheck %s: fx=%.4f fy=%.4f cx=%.4f cy=%.4f", passed ? "ok" : "failed", solved->camera_matrix[0],
                 solved->camera_matrix[4], solved->camera_matrix[2], solved->camera_matrix[5]);
    DUST_LOG_INF("calib selfcheck: matrix=%.3e coef=%.3e reprojection=%.3e px", matrix_error, coefficient_error, solved->reprojection_error_px);

    return passed;
}

/**
 * @brief 跑当前这一条测试；失败按 HaltOnFail 中止启动
 */
bool thread_init()
{
    if (!CheckCalibration())
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
