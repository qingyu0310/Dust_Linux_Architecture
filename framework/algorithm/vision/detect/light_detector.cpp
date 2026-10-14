/**
 * @file light_detector.cpp
 * @author qingyu
 * @brief 灯条图像检测器实现
 * @version 0.1
 * @date 2026-10-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "framework/algorithm/vision/detect/light_detector.hpp"

#include "framework/algorithm/vision/core/light_bar.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace algorithm::vision
{

namespace
{

/**
 * @brief 通过全部筛选、尚未分配稳定帧内 ID 的灯条来源
 */
struct LightSource
{
    cv::RotatedRect rectangle;      // 轮廓最小旋转矩形
    TeamColor color;                // 由轮廓内部像素判定的颜色
    float color_score;              // (R-B) / (R+B+epsilon)
    float contour_area;             // 轮廓面积，pixel^2
};

/**
 * @brief 累加一项灯条拒绝统计
 */
void Reject(LightDetectionStats& stats, LightRejectReason reason)
{
    ++stats.rejected[static_cast<std::size_t>(reason)];
}

/**
 * @brief 计算轮廓内部红蓝色差能量的归一化差值
 *
 * 先对每个像素分别累加红、蓝优势色差，再做归一化。接近白色的高亮核心
 * 因 R-B 接近零不会稀释周围有效色差，同时仍只统计轮廓内部像素。
 *
 * @param image 原始 BGR 图像
 * @param contour 当前亮区轮廓，使用整图坐标
 * @return (red_energy-blue_energy)/(red_energy+blue_energy+epsilon)，正值偏红、负值偏蓝
 */
float ColorScore(const cv::Mat& image, const std::vector<cv::Point>& contour)
{
    const cv::Rect image_bounds(0, 0, image.cols, image.rows);
    const cv::Rect bounds = cv::boundingRect(contour) & image_bounds;
    if (bounds.empty())
    {
        return 0.0f;
    }

    std::vector<cv::Point> local_contour;
    local_contour.reserve(contour.size());

    for (const cv::Point& point : contour)
    {
        local_contour.emplace_back(point.x - bounds.x, point.y - bounds.y);
    }

    cv::Mat mask = cv::Mat::zeros(bounds.size(), CV_8UC1);
    const std::vector<std::vector<cv::Point>> contours {local_contour};
    cv::drawContours(mask, contours, 0, cv::Scalar(255), cv::FILLED);

    double red_energy  = 0.0;
    double blue_energy = 0.0;
    const cv::Mat region = image(bounds);

    for (int row = 0; row < region.rows; ++row)
    {
        const cv::Vec3b* pixels = region.ptr<cv::Vec3b>(row);
        const std::uint8_t* selected = mask.ptr<std::uint8_t>(row);
        for (int column = 0; column < region.cols; ++column)
        {
            if (selected[column] == 0)
            {
                continue;
            }

            const int difference = static_cast<int>(pixels[column][2]) - static_cast<int>(pixels[column][0]);
            if (difference > 0)
            {
                red_energy += difference;
            }
            else
            {
                blue_energy -= difference;
            }
        }
    }

    const double denominator = red_energy + blue_energy;
    if (denominator <= 0.0)
    {
        return 0.0f;
    }
    return static_cast<float>((red_energy - blue_energy) / denominator);
}

/**
 * @brief 按带死区的颜色分数判定红、蓝或未知
 *
 * @param score 归一化红蓝差值
 * @param threshold 红蓝判定的最小绝对值
 * @return 灯条颜色；分数落在死区内时返回 Unknown
 */
TeamColor ClassifyColor(float score, float threshold)
{
    if (score >= threshold) {
        return TeamColor::Red;
    }
    if (score <= -threshold) {
        return TeamColor::Blue;
    }

    return TeamColor::Unknown;
}

} // namespace

/**
 * @brief 查询指定原因的灯条拒绝数量
 */
std::size_t LightDetectionStats::GetRejected(LightRejectReason reason) const noexcept
{
    const std::size_t index = static_cast<std::size_t>(reason);
    return index < rejected.size() ? rejected[index] : 0;
}


/**
 * @brief 从 BGR 图像提取按横坐标排序的目标颜色灯条
 *
 * 处理顺序固定为亮度分割、可选闭运算、外轮廓、几何筛选、颜色筛选、
 * 数量限制和稳定 ID 分配；每个亮区只记录第一个失败原因。
 *
 * @param bgr_image 非空 CV_8UC3 图像
 * @param binary_output 可选二值调试图输出
 * @return 灯条集合和逐级拒绝统计
 */
LightDetectionResult LightDetector::Detect(const cv::Mat& bgr_image, cv::Mat* binary_output) const
{
    if (bgr_image.empty() || bgr_image.type() != CV_8UC3)
    {
        throw std::invalid_argument("light detector requires non-empty CV_8UC3 image");
    }

    // 先建立只依赖固定参数的亮度掩膜。
    cv::Mat gray;
    cv::Mat binary;
    cv::cvtColor(bgr_image, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary, config_.binary.brightness_threshold, 255, cv::THRESH_BINARY);

    if (config_.binary.close_kernel_size != 0)
    {
        const int size = config_.binary.close_kernel_size;
        const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {size, size});
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
    }
    if (binary_output != nullptr)
    {
        binary.copyTo(*binary_output);
    }

    // findContours 使用副本，保证可选输出仍是形态学处理后的原始二值图。
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    LightDetectionResult result;
    result.stats.contour_count = contours.size();

    std::vector<LightSource> sources;
    sources.reserve(contours.size());

    // 按固定顺序筛选，每个轮廓只累计第一个拒绝原因。
    for (const auto& contour : contours)
    {
        const float contour_area = static_cast<float>(cv::contourArea(contour));
        const cv::RotatedRect rectangle = cv::minAreaRect(contour);
        if (contour_area < config_.filter.min_contour_area)
        {
            Reject(result.stats, LightRejectReason::ContourArea);
            result.rejections.push_back({rectangle, LightRejectReason::ContourArea});
            continue;
        }

        const auto geometry = LightBar::Create(rectangle, 0);
        if (!geometry)
        {
            Reject(result.stats, LightRejectReason::InvalidRectangle);
            result.rejections.push_back({rectangle, LightRejectReason::InvalidRectangle});
            continue;
        }

        if (geometry->GetLength() < config_.filter.min_length_px)
        {
            Reject(result.stats, LightRejectReason::Length);
            result.rejections.push_back({rectangle, LightRejectReason::Length});
            continue;
        }

        const float ratio = geometry->GetLength() / geometry->GetWidth();
        if (ratio < config_.filter.min_ratio || ratio > config_.filter.max_ratio)
        {
            Reject(result.stats, LightRejectReason::Ratio);
            result.rejections.push_back({rectangle, LightRejectReason::Ratio});
            continue;
        }

        const float rectangle_area = geometry->GetLength() * geometry->GetWidth();
        if (contour_area / rectangle_area < config_.filter.min_fill_ratio)
        {
            Reject(result.stats, LightRejectReason::Fill);
            result.rejections.push_back({rectangle, LightRejectReason::Fill});
            continue;
        }

        if (std::abs(geometry->GetTilt()) > config_.filter.max_tilt_rad)
        {
            Reject(result.stats, LightRejectReason::Tilt);
            result.rejections.push_back({rectangle, LightRejectReason::Tilt});
            continue;
        }

        const float     score = ColorScore(bgr_image, contour);
        const TeamColor color = ClassifyColor(score, config_.filter.min_color_contrast);
        
        if (color == TeamColor::Red)
        {
            ++result.stats.red_count;
        }
        else if (color == TeamColor::Blue)
        {
            ++result.stats.blue_count;
        }
        else
        {
            ++result.stats.unknown_count;
        }

        if (color != config_.filter.target_color)
        {
            Reject(result.stats, LightRejectReason::Color);
            result.rejections.push_back({rectangle, LightRejectReason::Color});
            continue;
        }

        sources.push_back({rectangle, color, score, contour_area});
    }

    // 超限时先按轮廓面积保留更显著的灯条，同面积用横坐标保证确定性。
    std::sort(sources.begin(), sources.end(), [](const LightSource& first, const LightSource& second)
    {
        if (first.contour_area != second.contour_area)
        {
            return first.contour_area > second.contour_area;
        }
        return first.rectangle.center.x < second.rectangle.center.x;
    });

    if (sources.size() > config_.filter.max_lights)
    {
        const std::size_t excess = sources.size() - config_.filter.max_lights;
        result.stats.rejected[static_cast<std::size_t>(LightRejectReason::Limit)] += excess;
        for (std::size_t index = config_.filter.max_lights; index < sources.size(); ++index)
        {
            result.rejections.push_back({sources[index].rectangle, LightRejectReason::Limit});
        }
        sources.resize(config_.filter.max_lights);
    }

    // 最终按画面位置排序，再分配连续且稳定的帧内 ID。
    std::sort(sources.begin(), sources.end(), [](const LightSource& first, const LightSource& second)
    {
        if (first.rectangle.center.x != second.rectangle.center.x)
        {
            return first.rectangle.center.x < second.rectangle.center.x;
        }
        return first.rectangle.center.y < second.rectangle.center.y;
    });

    result.lights.reserve(sources.size());
    for (std::size_t index = 0; index < sources.size(); ++index)
    {
        const LightSource& source = sources[index];
        auto light = LightBar::Create(source.rectangle, static_cast<std::uint32_t>(index),
                                      source.color, source.color_score);
        if (light)
        {
            result.lights.push_back(std::move(light.value()));
        }
    }
    result.stats.light_count = result.lights.size();

    return result;
}

} // namespace algorithm::vision
