/**
 * @file var.hpp
 * @author qingyu
 * @brief 调试变量注册 — 类型推导、可选范围、同步设置回调和哈希查找
 * @version 0.2
 * @date 2026-10-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include <mutex>
#include <string>

namespace debug
{

/**
 * @brief 调试变量类型，注册时由 decltype 推导，运行期决定解析、打印与写入分支
 */
enum class VarType : uint8_t
{
    Uint8,
    Int8,
    Uint16,
    Int16,
    Uint32,
    Int32,
    Uint64,
    Int64,
    Float,
    Double,
    Bool,
};

/**
 * @brief 各类型统一的取值容器
 *
 * 只作解析、比较和打印的临时容器；变量本身仍按真实类型存储，VarEntry::value 指向它。
 */
union VarValue
{
    uint8_t  u8;
    int8_t   i8;
    uint16_t u16;
    int16_t  i16;
    uint32_t u32;
    int32_t  i32;
    uint64_t u64;
    int64_t  i64;
    float    f;
    double   d;
    bool     b;
};

struct VarEntry;

/**
 * @brief 业务设置回调，收 double
 *
 * shell 的输入是文本，回调按业务需要的量纲转换；需要精确 64 位整数时不要用通知型注册。
 */
using VarSetter = void (*)(const VarEntry&, double, void*);

/**
 * @brief 类型映射：编译期把变量类型折算成 VarType
 *
 * 未支持的类型在编译期报错，不会退化到某个默认类型。
 */
template <typename T>
struct TypeMap
{
    static_assert(sizeof(T) == 0, "unsupported shell variable type");
};

template <typename T>
struct TypeMap<T&> : TypeMap<T> {};

template <>
struct TypeMap<uint8_t>  { static constexpr VarType type = VarType::Uint8;  };
template <>
struct TypeMap<int8_t>   { static constexpr VarType type = VarType::Int8;   };
template <>
struct TypeMap<uint16_t> { static constexpr VarType type = VarType::Uint16; };
template <>
struct TypeMap<int16_t>  { static constexpr VarType type = VarType::Int16;  };
template <>
struct TypeMap<uint32_t> { static constexpr VarType type = VarType::Uint32; };
template <>
struct TypeMap<int32_t>  { static constexpr VarType type = VarType::Int32;  };
template <>
struct TypeMap<uint64_t> { static constexpr VarType type = VarType::Uint64; };
template <>
struct TypeMap<int64_t>  { static constexpr VarType type = VarType::Int64;  };
template <>
struct TypeMap<float>    { static constexpr VarType type = VarType::Float;  };
template <>
struct TypeMap<double>   { static constexpr VarType type = VarType::Double; };
template <>
struct TypeMap<bool>     { static constexpr VarType type = VarType::Bool;   };

/**
 * @brief 链接段变量条目，范围和数值由同一互斥锁保护
 */
struct VarEntry
{
    const char* name;      // 调试变量名称
    VarType     type;      // 变量类型，注册时由 TypeMap 推导
    void*       value;     // 当前变量或设备成功值
    void*       minimum;   // 与 value 同类型的下限；空指针表示不设置
    void*       maximum;   // 与 value 同类型的上限；空指针表示不设置
    std::mutex* mutex;     // 保护数值和动态范围
    VarSetter   setter;    // 空指针则直接写变量，否则同步调用业务设置
    void*       context;   // 调用方持有的设置上下文
};

/**
 * @brief 变量命令入口，静态注册后建立只读名称索引
 */
class Var final
{
public:
    /**
     * @brief 建立名称索引，须在 shell 线程启动前调用
     * @throws std::logic_error 注册元数据无效或名称重复
     */
    static void Init();

    /**
     * @brief 从已建立的索引查找变量
     * @param name 调试变量名称
     * @return 注册条目，未找到返回 nullptr
     */
    static const VarEntry* Find(const std::string& name);

    /**
     * @brief 校验有限数值及条目范围，调用方须持有条目锁
     * @param entry 注册条目
     * @param value 候选数值
     * @throws std::invalid_argument 数值无效或越界
     */
    static void ValidateValue(const VarEntry& entry, double value);

    /**
     * @brief 处理 var list/get/set，设置回调失败只返回命令错误
     * @param line 不含 var 前缀的子命令，调用期间保持有效
     */
    static void Process(uint8_t* line);
};

} // namespace debug

#define DUST_VAR_CAT_IMPL(a, b) a##b
#define DUST_VAR_CAT(a, b) DUST_VAR_CAT_IMPL(a, b)

/**
 * @brief 注册无范围变量，类型由 decltype 推导，所有关联存储须覆盖条目生命周期
 * @param name_ 调试名称
 * @param var_ 变量，类型自动推导
 * @param mutex_ 数值保护锁
 */
#define REGISTER_SHELL_VAR(name_, var_, mutex_) \
    static ::debug::VarEntry DUST_VAR_CAT(s_shell_var_, __COUNTER__) \
    __attribute__((used, section("shell_var"), aligned(sizeof(void*)))) = \
    {name_, ::debug::TypeMap<decltype(var_)>::type, &(var_), nullptr, nullptr, &(mutex_), nullptr, nullptr}

/**
 * @brief 注册带动态范围的变量，校验成功后直接修改
 * @param name_ 调试名称
 * @param var_ 变量，类型自动推导
 * @param min_ 下限，与 var_ 同类型
 * @param max_ 上限，与 var_ 同类型
 * @param mutex_ 数值及范围保护锁
 */
#define REGISTER_SHELL_VAR_RANGE(name_, var_, min_, max_, mutex_) \
    static ::debug::VarEntry DUST_VAR_CAT(s_shell_var_, __COUNTER__) \
    __attribute__((used, section("shell_var"), aligned(sizeof(void*)))) = \
    {name_, ::debug::TypeMap<decltype(var_)>::type, &(var_), &(min_), &(max_), &(mutex_), nullptr, nullptr}

/**
 * @brief 注册同步设置回调，业务成功后提交变量值
 * @param name_ 调试名称
 * @param var_ 变量，类型自动推导，业务设置成功后更新
 * @param mutex_ 数值保护锁
 * @param setter_ var 不持条目锁调用，返回时业务设置必须已经完成
 * @param context_ 调用方持有的上下文，覆盖线程运行周期
 */
#define REGISTER_SHELL_VAR_NOTIFY(name_, var_, mutex_, setter_, context_) \
    static ::debug::VarEntry DUST_VAR_CAT(s_shell_var_, __COUNTER__) \
    __attribute__((used, section("shell_var"), aligned(sizeof(void*)))) = \
    {name_, ::debug::TypeMap<decltype(var_)>::type, &(var_), nullptr, nullptr, &(mutex_), setter_, context_}

/**
 * @brief 注册带范围同步设置回调，范围钳制后由业务同步设置
 * @param name_ 调试名称
 * @param var_ 变量，类型自动推导，业务设置成功后更新
 * @param min_ 下限，与 var_ 同类型
 * @param max_ 上限，与 var_ 同类型
 * @param mutex_ 数值及范围保护锁
 * @param setter_ var 不持条目锁调用，返回时业务设置必须已经完成
 * @param context_ 调用方持有的上下文，覆盖线程运行周期
 */
#define REGISTER_SHELL_VAR_RANGE_NOTIFY(name_, var_, min_, max_, mutex_, setter_, context_) \
    static ::debug::VarEntry DUST_VAR_CAT(s_shell_var_, __COUNTER__) \
    __attribute__((used, section("shell_var"), aligned(sizeof(void*)))) = \
    {name_, ::debug::TypeMap<decltype(var_)>::type, &(var_), &(min_), &(max_), &(mutex_), setter_, context_}
