/**
 * @file var.cpp
 * @author qingyu
 * @brief var 命令：类型化读写、范围校验和同步业务设置
 * @version 0.2
 * @date 2026-10-13
 *
 * @copyright Copyright (c) 2026
 */

#include "var.hpp"
#include "log.hpp"
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

extern "C"
{

extern const debug::VarEntry __start_shell_var[] __attribute__((weak));
extern const debug::VarEntry __stop_shell_var [] __attribute__((weak));

}

namespace debug
{

static std::unordered_map<std::string, const VarEntry*> entries_;    // 启动时建立，运行时只读

/**
 * @brief 按类型读取变量，返回统一取值容器
 */
static VarValue LoadValue(VarType type, const void* pointer)
{
    VarValue value {};
    switch (type)
    {
        case VarType::Uint8:  value.u8  = *static_cast<const uint8_t*> (pointer);   break;
        case VarType::Int8:   value.i8  = *static_cast<const int8_t*>  (pointer);   break;
        case VarType::Uint16: value.u16 = *static_cast<const uint16_t*>(pointer);   break;
        case VarType::Int16:  value.i16 = *static_cast<const int16_t*> (pointer);   break;
        case VarType::Uint32: value.u32 = *static_cast<const uint32_t*>(pointer);   break;
        case VarType::Int32:  value.i32 = *static_cast<const int32_t*> (pointer);   break;
        case VarType::Uint64: value.u64 = *static_cast<const uint64_t*>(pointer);   break;
        case VarType::Int64:  value.i64 = *static_cast<const int64_t*> (pointer);   break;
        case VarType::Float:  value.f   = *static_cast<const float*>   (pointer);   break;
        case VarType::Double: value.d   = *static_cast<const double*>  (pointer);   break;
        case VarType::Bool:   value.b   = *static_cast<const bool*>    (pointer);   break;
    }
    return value;
}

/**
 * @brief 按类型把取值容器写入变量
 */
static void StoreValue(VarType type, void* pointer, const VarValue& value)
{
    switch (type)
    {
        case VarType::Uint8:  *static_cast<uint8_t*>(pointer)  = value.u8;  break;
        case VarType::Int8:   *static_cast<int8_t*>(pointer)   = value.i8;  break;
        case VarType::Uint16: *static_cast<uint16_t*>(pointer) = value.u16; break;
        case VarType::Int16:  *static_cast<int16_t*>(pointer)  = value.i16; break;
        case VarType::Uint32: *static_cast<uint32_t*>(pointer) = value.u32; break;
        case VarType::Int32:  *static_cast<int32_t*>(pointer)  = value.i32; break;
        case VarType::Uint64: *static_cast<uint64_t*>(pointer) = value.u64; break;
        case VarType::Int64:  *static_cast<int64_t*>(pointer)  = value.i64; break;
        case VarType::Float:  *static_cast<float*>(pointer)    = value.f;   break;
        case VarType::Double: *static_cast<double*>(pointer)   = value.d;   break;
        case VarType::Bool:   *static_cast<bool*>(pointer)     = value.b;   break;
    }
}

/**
 * @brief 把取值容器按类型转成 double
 *
 * 用于范围比较；超过 2^53 的整数只能近似表示，需要精确值就用带范围的整型时留意。
 */
static double ToDouble(VarType type, const VarValue& value)
{
    switch (type)
    {
        case VarType::Uint8:  return static_cast<double>(value.u8);
        case VarType::Int8:   return static_cast<double>(value.i8);
        case VarType::Uint16: return static_cast<double>(value.u16);
        case VarType::Int16:  return static_cast<double>(value.i16);
        case VarType::Uint32: return static_cast<double>(value.u32);
        case VarType::Int32:  return static_cast<double>(value.i32);
        case VarType::Uint64: return static_cast<double>(value.u64);
        case VarType::Int64:  return static_cast<double>(value.i64);
        case VarType::Float:  return static_cast<double>(value.f);
        case VarType::Double: return value.d;
        case VarType::Bool:   return value.b ? 1.0 : 0.0;
    }
    return 0.0;
}

/**
 * @brief 由 double 构造取值容器，用于范围钳制后的写入
 */
static VarValue FromDouble(VarType type, double value)
{
    VarValue out {};
    switch (type)
    {
        case VarType::Uint8:  out.u8  = static_cast<uint8_t>(value);  break;
        case VarType::Int8:   out.i8  = static_cast<int8_t>(value);   break;
        case VarType::Uint16: out.u16 = static_cast<uint16_t>(value); break;
        case VarType::Int16:  out.i16 = static_cast<int16_t>(value);  break;
        case VarType::Uint32: out.u32 = static_cast<uint32_t>(value); break;
        case VarType::Int32:  out.i32 = static_cast<int32_t>(value);  break;
        case VarType::Uint64: out.u64 = static_cast<uint64_t>(value); break;
        case VarType::Int64:  out.i64 = static_cast<int64_t>(value);  break;
        case VarType::Float:  out.f   = static_cast<float>(value);    break;
        case VarType::Double: out.d   = value;                        break;
        case VarType::Bool:   out.b   = value != 0.0;                 break;
    }
    return out;
}

/**
 * @brief 按类型解析文本，bool 接受 true/false/1/0
 *
 * 整型按本类型边界判断，超出直接判为坏值，不做静默截断。
 *
 * @param type 变量类型
 * @param text 待解析文本
 * @param out 解析结果
 * @return true 解析成功且整段文本被消费
 */
static bool ParseValue(VarType type, const char* text, VarValue& out)
{
    char* end = nullptr;

    switch (type)
    {
        case VarType::Uint8:
        case VarType::Uint16:
        case VarType::Uint32:
        case VarType::Uint64:
        {
            if (text[0] == '-')
            {
                return false; // 无符号类型不接受负号
            }
            errno = 0;
            const unsigned long long value = std::strtoull(text, &end, 0);
            if (end == text || *end != '\0' || errno == ERANGE)
            {
                return false;
            }
            switch (type)
            {
                case VarType::Uint8:  if (value > std::numeric_limits<uint8_t>::max())  { return false; } out.u8  = static_cast<uint8_t>(value);  break;
                case VarType::Uint16: if (value > std::numeric_limits<uint16_t>::max()) { return false; } out.u16 = static_cast<uint16_t>(value); break;
                case VarType::Uint32: if (value > std::numeric_limits<uint32_t>::max()) { return false; } out.u32 = static_cast<uint32_t>(value); break;
                default: out.u64 = value; break;
            }
            return true;
        }
        case VarType::Int8:
        case VarType::Int16:
        case VarType::Int32:
        case VarType::Int64:
        {
            errno = 0;
            const long long value = std::strtoll(text, &end, 0);
            if (end == text || *end != '\0' || errno == ERANGE)
            {
                return false;
            }
            switch (type)
            {
                case VarType::Int8:  if (value < std::numeric_limits<int8_t>::min()  || value > std::numeric_limits<int8_t>::max())  { return false; } out.i8  = static_cast<int8_t>(value);  break;
                case VarType::Int16: if (value < std::numeric_limits<int16_t>::min() || value > std::numeric_limits<int16_t>::max()) { return false; } out.i16 = static_cast<int16_t>(value); break;
                case VarType::Int32: if (value < std::numeric_limits<int32_t>::min() || value > std::numeric_limits<int32_t>::max()) { return false; } out.i32 = static_cast<int32_t>(value); break;
                default: out.i64 = value; break;
            }
            return true;
        }
        case VarType::Float:
        {
            out.f = std::strtof(text, &end);
            return end != text && *end == '\0';
        }
        case VarType::Double:
        {
            out.d = std::strtod(text, &end);
            return end != text && *end == '\0';
        }
        case VarType::Bool:
        {
            if (std::strcmp(text, "true") == 0 || std::strcmp(text, "1") == 0)
            {
                out.b = true;
                return true;
            }
            if (std::strcmp(text, "false") == 0 || std::strcmp(text, "0") == 0)
            {
                out.b = false;
                return true;
            }
            return false;
        }
    }
    return false;
}

/**
 * @brief 按类型格式化取值，供 var list/get 输出
 */
static void FormatValue(VarType type, const VarValue& value, char* out, size_t size)
{
    switch (type)
    {
        case VarType::Uint8:  std::snprintf(out, size, "%u", static_cast<unsigned int>(value.u8));  break;
        case VarType::Int8:   std::snprintf(out, size, "%d", static_cast<int>(value.i8));           break;
        case VarType::Uint16: std::snprintf(out, size, "%u", static_cast<unsigned int>(value.u16)); break;
        case VarType::Int16:  std::snprintf(out, size, "%d", static_cast<int>(value.i16));          break;
        case VarType::Uint32: std::snprintf(out, size, "%u", value.u32);                            break;
        case VarType::Int32:  std::snprintf(out, size, "%d", value.i32);                            break;
        case VarType::Uint64: std::snprintf(out, size, "%llu", static_cast<unsigned long long>(value.u64)); break;
        case VarType::Int64:  std::snprintf(out, size, "%lld", static_cast<long long>(value.i64));  break;
        case VarType::Float:  std::snprintf(out, size, "%.9g", static_cast<double>(value.f));       break;
        case VarType::Double: std::snprintf(out, size, "%.9g", value.d);                            break;
        case VarType::Bool:   std::snprintf(out, size, "%s", value.b ? "true" : "false");           break;
    }
}

/**
 * @brief 取范围端点或当前值的 double 表示，调用方须持有条目锁
 */
static double ValueAsDouble(const VarEntry& entry, const void* pointer)
{
    return ToDouble(entry.type, LoadValue(entry.type, pointer));
}

/**
 * @brief 检查条目并建立索引，失败时不发布部分结果
 */
void Var::Init()
{
    std::unordered_map<std::string, const VarEntry*> entries;

    if (__start_shell_var != nullptr && __stop_shell_var != nullptr)
    {
        for (auto* entry = __start_shell_var; entry != __stop_shell_var; ++entry)
        {
            if (entry->name == nullptr || *entry->name == '\0' || entry->value == nullptr || entry->mutex == nullptr)
            {
                throw std::logic_error("invalid shell variable entry");
            }
            if (!entries.emplace(entry->name, entry).second)
            {
                throw std::logic_error(std::string("duplicate shell variable: ") + entry->name);
            }
        }
    }
    entries_ = std::move(entries);
}

/**
 * @brief 哈希查找已注册变量，未找到返回空指针
 */
const VarEntry* Var::Find(const std::string& name)
{
    const auto found = entries_.find(name);
    return found == entries_.end() ? nullptr : found->second;
}

/**
 * @brief 统一有限数值与动态范围校验，调用方持有条目锁
 */
void Var::ValidateValue(const VarEntry& entry, double value)
{
    if (!std::isfinite(value))
    {
        throw std::invalid_argument("finite number required");
    }
    if ((entry.minimum != nullptr && value < ValueAsDouble(entry, entry.minimum)) ||
        (entry.maximum != nullptr && value > ValueAsDouble(entry, entry.maximum)))
    {
        throw std::invalid_argument("out of range; use var get to inspect limits");
    }
}

/**
 * @brief 输出当前值及可选范围，调用方须持有条目的互斥锁
 *
 * @param entry 待显示的注册条目
 */
static void PrintVar(const VarEntry& entry)
{
    char value_text[64];
    FormatValue(entry.type, LoadValue(entry.type, entry.value), value_text, sizeof(value_text));

    char text[256];
    if (entry.minimum != nullptr && entry.maximum != nullptr)
    {
        char minimum_text[64];
        char maximum_text[64];
        FormatValue(entry.type, LoadValue(entry.type, entry.minimum), minimum_text, sizeof(minimum_text));
        FormatValue(entry.type, LoadValue(entry.type, entry.maximum), maximum_text, sizeof(maximum_text));
        std::snprintf(text, sizeof(text), "%s = %s [%s, %s]", entry.name, value_text, minimum_text, maximum_text);
    }
    else
    {
        std::snprintf(text, sizeof(text), "%s = %s", entry.name, value_text);
    }
    Log::SendCommandLine(text);
}

/**
 * @brief 通知型写入事务，业务成功后提交变量值
 *
 * 带范围的通知条目先在 double 域钳制再调用业务设置；业务侧收 double。
 *
 * @param entry 注册条目
 * @param value 有限候选值
 */
static void SetNotifiedValue(const VarEntry& entry, double value)
{
    {
        std::lock_guard<std::mutex> lock(*entry.mutex);
        const double minimum = entry.minimum != nullptr ? ValueAsDouble(entry, entry.minimum) : 0.0;
        const double maximum = entry.maximum != nullptr ? ValueAsDouble(entry, entry.maximum) : 0.0;
        if ((entry.minimum != nullptr && !std::isfinite(minimum)) || (entry.maximum != nullptr && !std::isfinite(maximum)) ||
            (entry.minimum != nullptr && entry.maximum != nullptr && minimum > maximum))
        {
            throw std::invalid_argument("invalid variable range");
        }

        if (entry.minimum != nullptr && value < minimum)
        {
            value = minimum;
        }

        if (entry.maximum != nullptr && value > maximum)
        {
            value = maximum;
        }
    }

    entry.setter(entry, value, entry.context);

    std::lock_guard<std::mutex> lock(*entry.mutex);
    StoreValue(entry.type, entry.value, FromDouble(entry.type, value));
}

/**
 * @brief 普通变量直接写入，通知变量同步设置成功后返回 ok
 *
 * @param line 不含 var 前缀的子命令
 */
void Var::Process(uint8_t* line)
{
    std::istringstream input(reinterpret_cast<char*>(line));
    std::string command, name, value, extra;
    input >> command;
    if (command == "list" && !(input >> extra))
    {
        if (entries_.empty())
        {
            Log::SendCommandLine("(none)");
            return;
        }
        for (auto* entry = __start_shell_var; entry != __stop_shell_var; ++entry)
        {
            std::lock_guard<std::mutex> lock(*entry->mutex);
            PrintVar(*entry);
        }
        return;
    }
    if ((command != "get" && command != "set") || !(input >> name) ||
        (command == "set" && !(input >> value)) || (input >> extra))
    {
        Log::SendCommandLine("?: var list|get <name>|set <name> <value>");
        return;
    }
    const auto* entry = Find(name);
    if (entry == nullptr)
    {
        Log::SendCommandLine("variable not found");
        return;
    }
    if (command == "get")
    {
        std::lock_guard<std::mutex> lock(*entry->mutex);
        PrintVar(*entry);
        return;
    }

    VarValue parsed {};
    if (!ParseValue(entry->type, value.c_str(), parsed))
    {
        Log::SendCommandLine("bad value");
        return;
    }
    const double number = ToDouble(entry->type, parsed);
    if (!std::isfinite(number))
    {
        Log::SendCommandLine("bad value: finite number required");
        return;
    }
    try
    {
        if (entry->setter != nullptr)
        {
            SetNotifiedValue(*entry, number);
        }
        else
        {
            std::lock_guard<std::mutex> lock(*entry->mutex);
            ValidateValue(*entry, number);
            StoreValue(entry->type, entry->value, parsed); // 直接写入的类型：存解析结果，不经过 double
        }
        Log::SendCommandLine("ok");
    }
    catch (const std::exception& error)
    {
        Log::SendCommandLine(error.what());
    }
    catch (...)
    {
        Log::SendCommandLine("variable setter failed");
    }
}

} // namespace debug
