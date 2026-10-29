/**
 * @file Init_entry.hpp
 * @author qingyu
 * @brief 初始化与线程注册项：阶段、优先级、失败策略、清理回调与启动耗时槽
 * @version 0.2
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include "boot_timing.hpp"

namespace init
{
/**
 * @brief 初始化或线程启动回调
 *
 * @return true 成功；false 失败
 */
using InitFunc = bool (*)();

/**
 * @brief 资源释放、停止通知或线程等待回调
 */
using CleanupFunc = void (*)();

/**
 * @brief 初始化阶段：按 Pre、Early、Mid、Late、App 顺序执行
 */
enum class InitStage
{
    PreInit = 0,
    EarlyInit,
    MidInit,
    LateInit,
    AppInit
};

/**
 * @brief 线程启动阶段：对应阶段初始化完成后执行
 */
enum class ThreadStage
{
    PreThread = 0,
    EarlyThread,
    MidThread,
    LateThread,
    AppThread
};

/**
 * @brief 初始化等级（优先级：同一阶段内 High → Mid → Low）
 */
enum class InitLevel
{
    High = 0,
    Mid,
    Low
};

/**
 * @brief 初始化失败策略：停止后续启动或继续执行
 */
enum class InitFailPolicy
{
    HaltOnFail = 0,
    ContinueOnFail
};

/**
 * @brief 初始化注册项：资源所属组件声明，启动层统一遍历
 */
struct InitEntry
{
    InitFunc        function;   // 初始化函数，返回成功状态
    CleanupFunc     cleanup;    // 部分初始化失败与退出时的资源释放
    BootTimingSlot* timing;     // 本项耗时槽（.boot_timing 段）
    InitStage       stage;      // 所属初始化阶段
    InitLevel       level;      // 同阶段初始化优先级：High → Mid → Low
    InitFailPolicy  fail;       // 初始化失败后的处理策略
    const char*     name;       // 日志中的组件标识
};

/**
 * @brief 线程注册项：启动、停止通知和等待退出构成完整生命周期
 */
struct ThreadEntry
{
    InitFunc        start;        // 创建线程后立即返回
    CleanupFunc     request_stop; // 通知停止并解除阻塞
    CleanupFunc     join;         // 等待线程退出
    BootTimingSlot* timing;       // 本项耗时槽（.boot_timing 段）
    ThreadStage     stage;        // 所属线程启动阶段
    const char*     name;         // 日志中的组件标识
};
} // namespace init

// 段内存放指针，保持固定大小和对齐；注册结构本身使用正常 C++ 布局。
#define DUST_JOIN_IMPL(a, b) a##b
#define DUST_JOIN(a, b) DUST_JOIN_IMPL(a, b)

/**
 * @brief 注册初始化项到 .user_init 指针表
 *
 * @param fn 初始化函数，返回 true 表示成功
 * @param stage_ 初始化阶段
 * @param level_ 同阶段初始化优先级（High/Mid/Low）
 * @param fail_ 失败策略
 * @param cleanup_ 清理函数，须支持部分初始化和未初始化状态
 * @param name_ 组件名称，用于启动日志
 */
#define REGISTER_INIT(fn, stage_, level_, fail_, cleanup_, name_)                                  \
    static BootTimingSlot DUST_JOIN(dust_timing_, fn)                                              \
        __attribute__((used, section(".boot_timing"), aligned(8))) = { name_, 0, false };          \
    static const init::InitEntry DUST_JOIN(dust_init_, fn){fn,                                     \
                                                           cleanup_,                               \
                                                           &DUST_JOIN(dust_timing_, fn),           \
                                                           init::InitStage::stage_,                \
                                                           init::InitLevel::level_,                \
                                                           init::InitFailPolicy::fail_,            \
                                                           name_};                                 \
    static const init::InitEntry* const DUST_JOIN(dust_init_ptr_, fn)                              \
        __attribute__((used, section(".user_init"), aligned(sizeof(void*)))) =                     \
            &DUST_JOIN(dust_init_, fn)

/**
 * @brief 注册线程启动项到 .user_thread 指针表
 *
 * @param fn 线程启动函数，启动后立即返回，不能在此直接阻塞运行
 * @param stage_ 线程启动阶段
 * @param stop_ 请求停止并解除等待的回调
 * @param join_ 等待线程退出的回调
 * @param name_ 线程名称，用于启动日志
 */
#define REGISTER_THREAD(fn, stage_, stop_, join_, name_)                                           \
    static BootTimingSlot DUST_JOIN(dust_timing_, fn)                                              \
        __attribute__((used, section(".boot_timing"), aligned(8))) = { name_, 0, false };          \
    static const init::ThreadEntry DUST_JOIN(dust_thread_, fn){fn, stop_, join_,                   \
                                                               &DUST_JOIN(dust_timing_, fn),       \
                                                               init::ThreadStage::stage_, name_};  \
    static const init::ThreadEntry* const DUST_JOIN(dust_thread_ptr_, fn)                          \
        __attribute__((used, section(".user_thread"), aligned(sizeof(void*)))) =                   \
            &DUST_JOIN(dust_thread_, fn)
