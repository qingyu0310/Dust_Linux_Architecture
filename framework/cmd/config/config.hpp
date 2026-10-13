/**
 * @file config.hpp
 * @author qingyu
 * @brief YAML 参数读取类 — 文件定位、字段校验与数值读取
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <initializer_list>
#include <string>
#include <yaml-cpp/yaml.h>

namespace cmd::config
{

/**
 * @brief YAML 参数读取对象
 *
 * 每个调用方加载自己的 YAML，类不持有全局参数或线程业务字段。
 * 相对路径以可执行文件所在目录为基准，加载与读取在初始化阶段完成。
 */
class Config final
{
public:
    void Load(const char* path, std::initializer_list<const char*> keys); // 加载 YAML，校验必需字段、未知字段和重复字段
    int  GetInt(const char* key, int minimum, int maximum) const;         // 读取整数，检查类型和允许范围
    bool GetBool(const char* key) const;                                  // 读取布尔，检查类型

private:
    YAML::Node  yaml_; // 当前对象加载的 YAML 根节点
    std::string name_; // 文件名，用于参数错误定位
};

} // namespace cmd::config
