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
#include <filesystem>
#include <set>
#include <stdexcept>

namespace cmd::config
{

/**
 * @brief 加载调用方指定 YAML，校验根节点和字段集合
 *
 * 相对路径以 /proc/self/exe 对应程序所在目录为基准，不依赖工作目录。
 * keys 是调用方声明的必需字段；拒绝缺失、未知与重复字段。
 * 加载失败清空当前节点，避免继续读取上一次加载的数据。
 *
 * @param path YAML 路径，可为绝对路径或程序旁的相对路径
 * @param keys 调用方所需的字段名
 * @throws std::exception 文件无法读取、YAML 格式或字段不符合要求
 */
void Config::Load(const char* path, std::initializer_list<const char*> keys)
{
    yaml_.reset();
    name_.clear();

    if (path == nullptr || *path == '\0')
    {
        throw std::invalid_argument("YAML path is empty");
    }

    auto file = std::filesystem::path(path);
    if (file.is_relative())
    {
        file = std::filesystem::read_symlink("/proc/self/exe").parent_path() / file;
    }
    name_ = file.filename().string();

    const auto yaml = YAML::LoadFile(file.string());
    if (!yaml.IsMap())
    {
        throw std::runtime_error(name_ + ": expected YAML mapping");
    }

    std::set<std::string> required;
    for (const auto key : keys)
    {
        if (key == nullptr || *key == '\0' || !required.insert(key).second)
        {
            throw std::invalid_argument(name_ + ": invalid required key list");
        }
    }

    std::set<std::string> seen;
    for (const auto& field : yaml)
    {
        const auto key = field.first.as<std::string>();
        if (required.count(key) == 0)
        {
            throw std::runtime_error(name_ + ": unknown key: " + key);
        }
        if (!seen.insert(key).second)
        {
            throw std::runtime_error(name_ + ": duplicate key: " + key);
        }
    }

    for (const auto& key : required)
    {
        if (seen.count(key) == 0)
        {
            throw std::runtime_error(name_ + ": required key: " + key);
        }
    }

    yaml_.reset(yaml);
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
        throw std::runtime_error(name_ + ": " + key + " must be within [" +
                                 std::to_string(minimum) + ", " + std::to_string(maximum) + "]");
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

} // namespace cmd::config
