/**
 * @file log_cmd.cpp
 * @author qingyu
 * @brief [① 命令层] DUST_LOG 命令层实现 — log list/on/off
 * @version 0.2
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "log_cmd.hpp"
#include "log_process.hpp"
#include "shell_cmd.hpp"
#include "token.hpp"
#include <cstdio>
#include <cstring>

namespace debug::logcmd
{

/**
 * @brief log list：遍历所有已注册 DBG 条目输出
 * @param args 未使用
 */
static void CmdList(uint8_t* args)
{
    (void)args;
    logproc::PrintLogList();
}

/**
 * @brief log on <name> [N]：选中 DBG 条目，可选分频周期
 * @param args 剩余行（"<name> [N]"）
 */
static void CmdOn(uint8_t* args)
{
    const char* name = nullptr;
    uint8_t* rest = NextToken(args, name);

    if (*name == '\0')
    {
        logproc::CommandLine("usage: log on <name> [divisor]");
        return;
    }

    uint16_t divisor = 1;
    if (*rest != '\0')
    {
        uint32_t v = 0;
        for (const uint8_t* p = rest; *p >= '0' && *p <= '9'; ++p)
        {
            v = v * 10u + static_cast<uint32_t>(*p - '0');
            if (v > 0xFFFFu)
            {
                v = 0xFFFFu;
                break;
            }
        }
        divisor = (v == 0) ? 1 : static_cast<uint16_t>(v);   // 0 或非数字 → 不分频
    }

    if (logproc::SelectDebugEntry(name, divisor))
    {
        char ack[32];
        std::snprintf(ack, sizeof(ack), "log on: ok x%u", static_cast<unsigned>(divisor));
        logproc::CommandLine(ack);
    }
    else
    {
        logproc::CommandLine("log on: not found");
    }
}

/**
 * @brief log off：取消选中（代数递增作废在途）
 * @param args 未使用
 */
static void CmdOff(uint8_t* args)
{
    (void)args;
    logproc::DeselectDebugEntry();
    logproc::CommandLine("log off: ok");
}

/**
 * @brief 未命中子命令：打印用法
 * @param args 未使用
 */
static void CmdUsage(uint8_t* args)
{
    (void)args;
    logproc::CommandLine("log list|on <name> [N]|off");
}

/**
 * @brief log 命令入口（词法解析 + 查表匹配）
 * @param line 子命令参数（不含 "log" 前缀）
 */
void Process(uint8_t* line)
{
    static constexpr SubCmdEntry kSubCmds[] = {
        {"list", CmdList},
        {"on",   CmdOn  },
        {"off",  CmdOff },
    };

    DispatchSubCmd(line, kSubCmds, sizeof(kSubCmds) / sizeof(kSubCmds[0]), CmdUsage);
}

} // namespace debug::logcmd

REGISTER_SHELL_CMD("log", "log list|on <name> [N]|off", ::debug::logcmd::Process);
