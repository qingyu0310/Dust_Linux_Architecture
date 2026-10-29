/**
 * @file log_cmd.hpp
 * @author qingyu
 * @brief [① 命令层] DUST_LOG 命令层 — log 子命令词法解析、查表匹配与处理
 * @version 0.1
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>

namespace debug::logcmd
{

/**
 * @brief log 命令入口（词法解析 + 查表匹配）
 * @param line 子命令参数（不含 "log" 前缀）
 */
void Process(uint8_t* line);

} // namespace debug::logcmd
