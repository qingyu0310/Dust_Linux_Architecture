/**
 * @file token.hpp
 * @author qingyu
 * @brief shell 行切分工具 — 空格分隔的 token 取出与子命令查表分派
 * @version 0.2
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

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

/**
 * @brief 子命令表项：子命令名与处理函数
 *
 * 处理函数收到的是「子命令之后剩余的行」（已跳过空格，可再取 token）。
 */
struct SubCmdEntry
{
    const char* name;                    // 子命令名（"list" / "on" / ...）
    void (*handler)(uint8_t* args);      // 处理函数，收到子命令之后的剩余行
};

/**
 * @brief 词法解析与查表分派：取首 token 作子命令名，命中则调处理函数
 *
 * 未命中时调用 on_unknown（传子命令之后的剩余行），由命令族自己打印用法。
 *
 * @param line       命令行（原地修改，token 间空格变 '\0'）
 * @param table      子命令表
 * @param count      表项数
 * @param on_unknown 未命中时的处理函数；nullptr 表示静默
 * @return true=命中；false=未命中
 */
inline bool DispatchSubCmd(uint8_t* line, const SubCmdEntry* table, size_t count, void (*on_unknown)(uint8_t* args))
{
    const char* sub = nullptr;
    line = NextToken(line, sub);

    for (size_t i = 0; i < count; ++i)
    {
        if (std::strcmp(sub, table[i].name) == 0)
        {
            table[i].handler(line);
            return true;
        }
    }

    if (on_unknown != nullptr)
    {
        on_unknown(line);
    }

    return false;
}

} // namespace debug
