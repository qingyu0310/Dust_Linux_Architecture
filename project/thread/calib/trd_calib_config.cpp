/**
 * @file trd_calib_config.cpp
 * @author qingyu
 * @brief 标定线程配置 — 标定板规格、样本门限与采集时限
 * @version 0.1
 * @date 2026-10-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_calib.hpp"
#include "framework/cmd/config/config.hpp"

namespace thread::calib
{

/**
 * @brief 读取本组件 YAML：字段全部必需
 *
 * @throws std::exception 文件缺失、字段不符或数值越界
 */
void LoadParams()
{
    cmd::config::Config yaml;
    yaml.Load("thread/calib/trd_calib.yaml",
              {"pattern_cols", "pattern_rows", "center_distance_mm", "sample_count", "min_shift_px", "min_scale_change", "min_ratio_change", "sample_interval_ms", "capture_timeout_s", "max_reprojection_error_px", "samples_dir"});

    params_.board.cols                = yaml.GetInt("pattern_cols",          2,          1000);
    params_.board.rows                = yaml.GetInt("pattern_rows",          2,          1000);
    params_.board.center_distance_mm  = yaml.GetDouble("center_distance_mm", 0.001,      100000.0);

    params_.sample_count              = yaml.GetInt("sample_count",          3,          1000);
    params_.sample_interval_ms        = yaml.GetInt("sample_interval_ms",    0,          600000);
    params_.capture_timeout_s         = yaml.GetInt("capture_timeout_s",     1,          86400);
    params_.samples_dir               = yaml.GetPath("samples_dir");
    params_.max_reprojection_error_px = yaml.GetDouble("max_reprojection_error_px", 0.0, 1000.0);

    params_.gate.min_shift_px         = static_cast<float>(yaml.GetDouble("min_shift_px",     0.0, 100000.0));
    params_.gate.min_span_change      = static_cast<float>(yaml.GetDouble("min_scale_change", 0.0, 1.0));
    params_.gate.min_ratio_change     = static_cast<float>(yaml.GetDouble("min_ratio_change", 0.0, 100.0));
}

} // namespace thread::calib
