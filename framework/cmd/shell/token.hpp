/**
 * @file token.hpp
 * @author qingyu
 * @brief shell 行切分工具 — 空格分隔的 token 取出
 * @version 0.1
 * @date 2026-10-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>

namespace debug {

/**
 * @brief 跳过多余空格
 * @param p 输入指针
 * @return 第一个非空格字符的位置
 */
inline uint8_t* SkipSpace(uint8_t* p)
{
    while (*p == ' ') ++p;
    return p;
}

/**
 * @brief 取出下一个空格分隔的 token
 *
 * 原地把 token 末尾的空格改成 '\0'，便于直接用作 C 字符串。
 * 连续空格按一个分隔符处理；行尾空格不影响结果。
 *
 * @param line 输入指针（原地修改）
 * @param out  输出：token 起始地址（无 token 时指向空串）
 * @return 下一个 token 的起始位置
 */
inline uint8_t* NextToken(uint8_t* line, const char*& out)
{
    line = SkipSpace(line);
    out  = reinterpret_cast<const char*>(line);

    while (*line && *line != ' ') ++line;
    if (*line == ' ') {
        *line = '\0';
        ++line;
    }

    return SkipSpace(line);
}

} // namespace debug
