/**
 * @file trd_vision_config.cpp
 * @author qingyu
 * @brief 识别线程配置 — 检测器参数、相机内参、云台外参与跟踪参数
 * @version 0.1
 * @date 2026-10-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_vision.hpp"
#include "framework/cmd/config/config.hpp"
#include "framework/cmd/shell/log/log.hpp"
#include <limits>
#include <stdexcept>
#include <string>

#ifdef CONFIG_DUST_VISION_YOLO_REFINER
#include "framework/algorithm/vision/refine/armor_refiner.hpp"
#endif

namespace thread::vision
{

using namespace algorithm::vision;

/**
 * @brief 一次读完视觉线程 YAML：检测器参数与相机内参
 *
 * 内参那批键由标定组件打印、人粘进本 YAML，是标定与视觉之间唯一的接口；
 * 缺键或越界都会在 Load 里直接报错，不带默认值悄悄跑。
 */
void LoadVisionConfig(ArmorDetector::Config& detector_config)
{
    cmd::config::Config yaml;
    yaml.Load("thread/vision/trd_vision.yaml",
              {"engine_path",          "cuda_device",         "score_threshold", "confidence_threshold", "nms_threshold"},
              {"brightness_threshold", "min_light_length_px", "min_light_ratio", "max_light_ratio",      "max_light_tilt_deg", "max_endpoint_distance_px", "image_width", "image_height", "camera_fx", "camera_fy", "camera_cx", "camera_cy", "distort_k1", "distort_k2", "distort_p1", "distort_p2", "distort_k3", "reprojection_error_px", "armor_radius_init_m", "armor_num", "point_sigma_px", "sigma_distance_scale", "accel_variance", "angular_accel_variance", "shape_variance", "gimbal_rvec_x", "gimbal_rvec_y", "gimbal_rvec_z", "gimbal_tx_m", "gimbal_ty_m", "gimbal_tz_m"});

    detector_config.engine_path = yaml.GetPath("engine_path");
    if (detector_config.engine_path.empty())
    {
        throw std::runtime_error("vision config: engine_path must not be empty");
    }
    detector_config.cuda_device          = yaml.GetInt("cuda_device", 0, std::numeric_limits<int>::max());
    detector_config.nms_threshold        = static_cast<float>(yaml.GetDouble("nms_threshold", 0.0, 1.0));
    detector_config.score_threshold      = static_cast<float>(yaml.GetDouble("score_threshold", 0.0, 1.0));
    detector_config.confidence_threshold = static_cast<float>(yaml.GetDouble("confidence_threshold", 0.0, 1.0));

    config_.camera.image_width       = yaml.GetInt("image_width",   1,          100000);
    config_.camera.image_height      = yaml.GetInt("image_height",  1,          100000);

    config_.camera.fx                = yaml.GetDouble("camera_fx",  1.0,        1000000.0);
    config_.camera.fy                = yaml.GetDouble("camera_fy",  1.0,        1000000.0);
    config_.camera.cx                = yaml.GetDouble("camera_cx",  -1000000.0, 1000000.0);
    config_.camera.cy                = yaml.GetDouble("camera_cy",  -1000000.0, 1000000.0);

    config_.camera.k1                = yaml.GetDouble("distort_k1", -1000.0,    1000.0);
    config_.camera.k2                = yaml.GetDouble("distort_k2", -1000.0,    1000.0);
    config_.camera.p1                = yaml.GetDouble("distort_p1", -1000.0,    1000.0);
    config_.camera.p2                = yaml.GetDouble("distort_p2", -1000.0,    1000.0);
    config_.camera.k3                = yaml.GetDouble("distort_k3", -1000.0,    1000.0);

    config_.camera.reprojection_error_px = yaml.GetDouble("reprojection_error_px", 0.0, 1000000.0);

    config_.extrinsics.rotation[0]    = yaml.GetDouble("gimbal_rvec_x", -4.0, 4.0);
    config_.extrinsics.rotation[1]    = yaml.GetDouble("gimbal_rvec_y", -4.0, 4.0);
    config_.extrinsics.rotation[2]    = yaml.GetDouble("gimbal_rvec_z", -4.0, 4.0);
    config_.extrinsics.translation[0] = yaml.GetDouble("gimbal_tx_m", -10.0, 10.0);
    config_.extrinsics.translation[1] = yaml.GetDouble("gimbal_ty_m", -10.0, 10.0);
    config_.extrinsics.translation[2] = yaml.GetDouble("gimbal_tz_m", -10.0, 10.0);

    // 跟踪器参数：Q/R 的量级都在这里调，不改代码
    config_.tracker.tracker.armor_num              = yaml.GetInt("armor_num", 1, 8);
    config_.tracker.tracker.radius_init_m          = yaml.GetDouble("armor_radius_init_m", 0.05, 0.5);
    config_.tracker.tracker.point_sigma_px         = yaml.GetDouble("point_sigma_px", 0.1, 100.0);
    config_.tracker.tracker.sigma_distance_scale   = yaml.GetDouble("sigma_distance_scale", 0.0, 10.0);
    config_.tracker.tracker.accel_variance         = yaml.GetDouble("accel_variance", 1.0e-6, 1.0e6);
    config_.tracker.tracker.angular_accel_variance = yaml.GetDouble("angular_accel_variance", 1.0e-6, 1.0e6);
    config_.tracker.tracker.shape_variance         = yaml.GetDouble("shape_variance", 1.0e-6, 1.0e6);
    config_.tracker.tracker.extrinsics             = config_.extrinsics;

    DUST_LOG_INF("vision: intrinsics %dx%d fx=%.3f fy=%.3f cx=%.3f cy=%.3f", config_.camera.image_width, config_.camera.image_height, config_.camera.fx, config_.camera.fy, config_.camera.cx, config_.camera.cy);
}

#ifdef CONFIG_DUST_VISION_YOLO_REFINER
/**
 * @brief 从视觉线程 YAML 读取精修参数
 *
 * 精修器编进程序就生效，参数必须齐备。
 */
void LoadRefinerConfig(ArmorRefiner::Config& refiner_config)
{
    cmd::config::Config yaml;
    yaml.Load("thread/vision/trd_vision.yaml",
              {"engine_path", "cuda_device", "score_threshold", "confidence_threshold", "nms_threshold"},
              {"brightness_threshold", "min_light_length_px", "min_light_ratio", "max_light_ratio", "max_light_tilt_deg", "max_endpoint_distance_px"});

    const char* const refiner_keys[] = {
        "brightness_threshold",
        "min_light_length_px",
        "min_light_ratio",
        "max_light_ratio",
        "max_light_tilt_deg",
        "max_endpoint_distance_px"
    };
    for (const char* key : refiner_keys)
    {
        if (!yaml.Has(key))
        {
            throw std::runtime_error(std::string("vision config: missing refiner field: ") + key);
        }
    }

    refiner_config.brightness_threshold = yaml.GetInt("brightness_threshold", 0, 255);
    refiner_config.min_light_ratio      = static_cast<float>(yaml.GetDouble("min_light_ratio",     0.0, 1000000.0));
    refiner_config.max_light_ratio      = static_cast<float>(yaml.GetDouble("max_light_ratio",     0.0, 1000000.0));
    refiner_config.min_light_length_px  = static_cast<float>(yaml.GetDouble("min_light_length_px", 0.0, 1000000.0));

    const double max_tilt_deg = yaml.GetDouble("max_light_tilt_deg", 0.0, 90.0);
    refiner_config.max_light_tilt_rad       = static_cast<float>(max_tilt_deg * 3.14159265358979323846 / 180.0);
    refiner_config.max_endpoint_distance_px = static_cast<float>(yaml.GetDouble("max_endpoint_distance_px", 0.0, 1000000.0));

    refiner_config.Validate();
}
#endif

} // namespace thread::vision
