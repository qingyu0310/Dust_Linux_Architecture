/**
 * @file boot_timing.cpp
 * @author qingyu
 * @brief 启动期耗时槽的段遍历实现
 * @version 0.1
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "boot_timing.hpp"

extern const BootTimingSlot __boot_timing_start[];
extern const BootTimingSlot __boot_timing_end[];

/**
 * @brief 槽总数（链接段长度 / 槽大小）
 */
uint8_t BootTiming::Count()
{
    return static_cast<uint8_t>(__boot_timing_end - __boot_timing_start);
}

/**
 * @brief 按序号取槽
 * @param index 序号（0 起）
 * @return 槽指针；越界返回 nullptr
 */
const BootTimingSlot* BootTiming::At(uint8_t index)
{
    if (index >= Count()) return nullptr;

    return &__boot_timing_start[index];
}

/**
 * @brief 全部槽的总耗时（纳秒）
 */
uint64_t BootTiming::TotalNs()
{
    uint64_t sum = 0;
    const uint8_t count = Count();

    for (uint8_t i = 0; i < count; ++i)
    {
        sum += __boot_timing_start[i].ns;
    }

    return sum;
}
