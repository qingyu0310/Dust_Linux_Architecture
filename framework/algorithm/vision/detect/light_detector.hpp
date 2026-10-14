/**
 * @file light_detector.hpp
 * @author qingyu
 * @brief 灯条图像检测器
 * @version 0.1
 * @date 2026-10-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "framework/algorithm/vision/core/armor_types.hpp"
#include "framework/algorithm/vision/core/light_result.hpp"

#include <cmath>
#include <cstddef>
#include <opencv2/core.hpp>
#include <stdexcept>

namespace algorithm::vision
{

/**
 * @brief 从 BGR 图像中提取目标颜色灯条
 *
 * 本类只做单帧像素处理，不负责装甲板配对、配置文件、窗口或日志。
 */
class LightDetector final
{
public:
    /**
     * @brief 亮度分割与形态学参数
     */
    struct BinaryConfig
    {
        int brightness_threshold {150}; // 灰度二值化阈值，范围 [0, 255]
        int close_kernel_size    {0};   // 闭运算方核边长；0 表示关闭，否则必须为正奇数

        /**
         * @brief 校验二值化参数
         * @throws std::invalid_argument 阈值越界或闭运算核不合法
         */
        void Validate() const
        {
            const bool kernel_valid = close_kernel_size == 0 || (close_kernel_size > 0 && close_kernel_size % 2 == 1);
            if (brightness_threshold < 0 || brightness_threshold > 255 || !kernel_valid)
            {
                throw std::invalid_argument("invalid light detector binary config");
            }
        }
    };

    /**
     * @brief 单灯条几何、颜色和数量门限
     */
    struct FilterConfig
    {
        float min_contour_area   {8.0f};            // 最小轮廓面积，pixel^2
        float min_length_px      {8.0f};            // 最小长轴长度，pixel
        float min_ratio          {1.5f};            // 最小长宽比
        float max_ratio          {20.0f};           // 最大长宽比
        float min_fill_ratio     {0.5f};            // 轮廓面积 / 旋转矩形面积下限
        float max_tilt_rad       {0.785398f};       // 相对图像竖直方向最大倾角，rad
        float min_color_contrast {0.1f};            // 红蓝归一化差值绝对值下限
        TeamColor target_color   {TeamColor::Blue}; // 只保留的敌方灯条颜色
        std::size_t max_lights   {64};              // 单帧最多保留灯条数

        /**
         * @brief 校验灯条筛选参数
         * @throws std::invalid_argument 数值非有限、范围关系错误或目标颜色无效
         */
        void Validate() const
        {
            const bool target_valid = target_color == TeamColor::Red || target_color == TeamColor::Blue;
            const bool values_valid = std::isfinite(min_contour_area)   && min_contour_area   > 0.0f &&
                                      std::isfinite(min_length_px)      && min_length_px      > 0.0f &&
                                      std::isfinite(min_ratio)          && min_ratio          > 0.0f &&
                                      std::isfinite(max_ratio)          && max_ratio          > min_ratio &&
                                      std::isfinite(min_fill_ratio)     && min_fill_ratio     > 0.0f && min_fill_ratio <= 1.0f &&
                                      std::isfinite(max_tilt_rad)       && max_tilt_rad       > 0.0f && max_tilt_rad <= static_cast<float>(CV_PI) * 0.5f &&
                                      std::isfinite(min_color_contrast) && min_color_contrast > 0.0f && min_color_contrast < 1.0f;

            if (!target_valid || !values_valid || max_lights == 0)
            {
                throw std::invalid_argument("invalid light detector filter config");
            }
        }
    };

    /**
     * @brief 灯条检测器完整配置
     */
    struct Config
    {
        BinaryConfig binary; // 亮度掩膜参数
        FilterConfig filter; // 单灯条筛选参数

        /**
         * @brief 依次校验二值化和灯条筛选参数
         */
        void Validate() const
        {
            binary.Validate();
            filter.Validate();
        }
    };

    /**
     * @brief 创建灯条检测器
     * @param config 完整检测参数
     * @throws std::invalid_argument 配置不满足约束
     */
    explicit LightDetector(const Config& config) : config_(config)
    {
        config_.Validate();
    }

    LightDetectionResult Detect(const cv::Mat& bgr_image, cv::Mat* binary_output = nullptr) const;

    /**
     * @brief 返回构造时校验并保存的配置
     */
    const Config& GetConfig() const noexcept
    {
        return config_;
    }

private:
    Config config_; // 构造后只读的检测参数
};

} // namespace algorithm::vision
