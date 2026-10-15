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
 * 相对路径优先以 DUST_CONFIG_ROOT 为根；未设置时退回可执行文件所在目录。
 * 加载与读取均在初始化阶段完成。
 */
class Config final
{
public:
    void   Load(const char* path, std::initializer_list<const char*> required, std::initializer_list<const char*> optional = {});  // 加载 YAML，校验字段集合和重复字段
    bool   Has(const char* key) const;                                          // 查询可选字段是否存在
    
    int    GetInt(const char* key, int minimum, int maximum) const;             // 读取整数，检查类型和允许范围
    double GetDouble(const char* key, double minimum, double maximum) const;    // 读取有限浮点数，检查范围
    bool   GetBool(const char* key) const;                                      // 读取布尔，检查类型
    std::string GetString(const char* key) const;                               // 读取非结构化字符串
    std::string GetPath(const char* key) const;                                 // 解析相对配置根的路径

private:
    YAML::Node  yaml_; // 当前对象加载的 YAML 根节点
    std::string name_; // 文件名，用于参数错误定位
    std::string root_; // 当前配置根，用于解析运行资源相对路径
};

} // namespace cmd::config
