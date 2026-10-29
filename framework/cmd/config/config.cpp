/**
 * @file config.cpp
 * @author qingyu
 * @brief YAML 参数读取实现 — 文件定位、字段校验与数值读取
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "config.hpp"
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <set>
#include <stdexcept>

namespace cmd::config
{

/**
 * @brief 加载调用方指定 YAML，校验根节点和字段集合
 *
 * 相对路径优先以 DUST_CONFIG_ROOT 为根，否则以当前工作目录为根。
 * required 是调用方声明的必需字段，optional 是允许但可以省略的字段；拒绝未知与重复字段。
 * 加载失败清空当前节点，避免继续读取上一次加载的数据。
 *
 * @param path YAML 路径，可为绝对路径或工作目录下的相对路径
 * @param required 调用方所需的必需字段名
 * @param optional 调用方允许但可以省略的字段名
 * @throws std::exception 文件无法读取、YAML 格式或字段不符合要求
 */
void Config::Load(const char* path, std::initializer_list<const char*> required, std::initializer_list<const char*> optional)
{
    yaml_.reset();
    name_.clear();
    root_.clear();

    if (path == nullptr || *path == '\0')
    {
        throw std::invalid_argument("YAML path is empty");
    }

    auto file = std::filesystem::path(path);
    if (file.is_relative())
    {
        const char* root = std::getenv("DUST_CONFIG_ROOT");
        if (root != nullptr && *root != '\0')
        {
            const auto config_root = std::filesystem::path(root);
            if (!config_root.is_absolute())
            {
                throw std::invalid_argument("DUST_CONFIG_ROOT must be an absolute path");
            }
            root_ = config_root.lexically_normal().string();
            file = std::filesystem::path(root_) / file;
        }
        else
        {
            root_ = std::filesystem::current_path().lexically_normal().string();
            file = std::filesystem::path(root_) / file;
        }
    }
    else
    {
        root_ = file.parent_path().lexically_normal().string();
    }
    name_ = file.string();

    const auto yaml = YAML::LoadFile(file.string());
    if (!yaml.IsMap())
    {
        throw std::runtime_error(name_ + ": expected YAML mapping");
    }

    std::set<std::string> required_keys;
    std::set<std::string> allowed_keys;
    for (const auto key : required)
    {
        if (key == nullptr || *key == '\0' || !required_keys.insert(key).second ||
            !allowed_keys.insert(key).second)
        {
            throw std::invalid_argument(name_ + ": invalid required key list");
        }
    }

    for (const auto key : optional)
    {
        if (key == nullptr || *key == '\0' || !allowed_keys.insert(key).second)
        {
            throw std::invalid_argument(name_ + ": invalid optional key list");
        }
    }

    std::set<std::string> seen;
    for (const auto& field : yaml)
    {
        const auto key = field.first.as<std::string>();
        if (allowed_keys.count(key) == 0)
        {
            throw std::runtime_error(name_ + ": unknown key: " + key);
        }
        if (!seen.insert(key).second)
        {
            throw std::runtime_error(name_ + ": duplicate key: " + key);
        }
    }

    for (const auto& key : required_keys)
    {
        if (seen.count(key) == 0)
        {
            throw std::runtime_error(name_ + ": required key: " + key);
        }
    }

    yaml_.reset(yaml);
}

/**
 * @brief 查询当前 YAML 是否包含可选字段
 *
 * @param key 字段名
 * @return true 表示字段存在
 */
bool Config::Has(const char* key) const
{
    if (key == nullptr || *key == '\0')
    {
        throw std::invalid_argument("invalid YAML field request");
    }
    if (!yaml_.IsMap())
    {
        throw std::logic_error("YAML not loaded");
    }
    return static_cast<bool>(yaml_[key]);
}

/**
 * @brief 读取整数参数，检查类型与调用方声明的取值范围
 *
 * @param key 字段名
 * @param minimum 最小允许值，包含端点
 * @param maximum 最大允许值，包含端点
 * @return 已校验的整数值
 * @throws std::exception 未加载、字段不存在、类型错误或数值越界
 */
int Config::GetInt(const char* key, int minimum, int maximum) const
{
    if (key == nullptr || *key == '\0' || minimum > maximum)
    {
        throw std::invalid_argument("invalid YAML integer request");
    }
    if (!yaml_.IsMap())
    {
        throw std::logic_error("YAML not loaded");
    }

    const auto value = yaml_[key];
    if (!value.IsScalar())
    {
        throw std::runtime_error(name_ + ": integer required: " + key);
    }

    int number;
    try
    {
        number = value.as<int>();
    }
    catch (const YAML::Exception&)
    {
        throw std::runtime_error(name_ + ": integer required: " + key);
    }

    if (number < minimum || number > maximum)
    {
        throw std::runtime_error(name_ + ": " + key + " must be within [" + std::to_string(minimum) + ", " + std::to_string(maximum) + "]");
    }
    return number;
}

/**
 * @brief 读取浮点参数，检查类型、有限性与调用方声明的范围
 *
 * @param key 字段名
 * @param minimum 最小允许值，包含端点
 * @param maximum 最大允许值，包含端点
 * @return 已校验的 double 值
 * @throws std::exception 未加载、字段不存在、类型错误、非有限或数值越界
 */
double Config::GetDouble(const char* key, double minimum, double maximum) const
{
    if (key == nullptr || *key == '\0' || !std::isfinite(minimum) ||
        !std::isfinite(maximum) || minimum > maximum)
    {
        throw std::invalid_argument("invalid YAML double request");
    }
    if (!yaml_.IsMap())
    {
        throw std::logic_error("YAML not loaded");
    }

    const auto value = yaml_[key];
    if (!value.IsScalar())
    {
        throw std::runtime_error(name_ + ": double required: " + key);
    }

    double number;
    try
    {
        number = value.as<double>();
    }
    catch (const YAML::Exception&)
    {
        throw std::runtime_error(name_ + ": double required: " + key);
    }

    if (!std::isfinite(number) || number < minimum || number > maximum)
    {
        throw std::runtime_error(name_ + ": " + key + " must be within [" + std::to_string(minimum) + ", " + std::to_string(maximum) + "]");
    }
    return number;
}

/**
 * @brief 读取布尔参数，检查类型
 *
 * @param key 字段名
 * @return 已校验的布尔值
 * @throws std::exception 未加载、字段不存在或类型错误
 */
bool Config::GetBool(const char* key) const
{
    if (key == nullptr || *key == '\0')
    {
        throw std::invalid_argument("invalid YAML boolean request");
    }
    if (!yaml_.IsMap())
    {
        throw std::logic_error("YAML not loaded");
    }

    bool flag = false;
    try
    {
        flag = yaml_[key].as<bool>();
    }
    catch (const YAML::Exception&)
    {
        throw std::runtime_error(name_ + ": boolean required: " + key);
    }
    return flag;
}

/**
 * @brief 读取字符串参数，检查字段存在且为标量
 *
 * @param key 字段名
 * @return 字符串值，允许由调用方对空字符串作业务校验
 * @throws std::exception 未加载、字段不存在或类型错误
 */
std::string Config::GetString(const char* key) const
{
    if (key == nullptr || *key == '\0')
    {
        throw std::invalid_argument("invalid YAML string request");
    }
    if (!yaml_.IsMap())
    {
        throw std::logic_error("YAML not loaded");
    }

    const auto value = yaml_[key];
    if (!value.IsScalar())
    {
        throw std::runtime_error(name_ + ": string required: " + key);
    }

    try
    {
        return value.as<std::string>();
    }
    catch (const YAML::Exception&)
    {
        throw std::runtime_error(name_ + ": string required: " + key);
    }
}

/**
 * @brief 读取并解析相对配置根的运行资源路径
 *
 * @param key 字段名
 * @return 绝对或相对路径；相对路径以加载配置时的配置根为基准
 * @throws std::exception 未加载、字段不存在或类型错误
 */
std::string Config::GetPath(const char* key) const
{
    const std::string value = GetString(key);
    if (value.empty())
    {
        throw std::runtime_error(name_ + ": path must not be empty: " + key);
    }
    const auto path = std::filesystem::path(value);
    if (path.is_absolute())
    {
        return path.lexically_normal().string();
    }
    if (root_.empty())
    {
        throw std::logic_error("configuration root not initialized");
    }
    return (std::filesystem::path(root_) / path).lexically_normal().string();
}

} // namespace cmd::config
