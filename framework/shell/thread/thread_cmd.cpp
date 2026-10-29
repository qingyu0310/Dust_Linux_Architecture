/**
 * @file thread_cmd.cpp
 * @author qingyu
 * @brief thread 命令实现 — list/show/pause/resume/exit/reset（输出走 DUST_LOG 命令响应）
 * @version 0.2
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "shell_cmd.hpp"
#include "token.hpp"
#include "log.hpp"
#include "framework/init/thread/thread_registry.hpp"
#include <cstdio>
#include <cstring>

namespace debug
{

/**
 * @brief 线程状态字符串
 * @param s 统计快照
 * @return 固定字符串
 */
static const char* StateName(const init::ThreadStats& s)
{
    if (!s.running) return "dead";
    if (s.paused)   return "paused";
    return "running";
}

/**
 * @brief 把纳秒拆成毫秒整数与三位小数
 */
static void FormatMs(uint64_t ns, char* out, size_t size)
{
    const uint64_t us = ns / 1000ULL;
    std::snprintf(out, size, "%u.%03u", static_cast<unsigned>(us / 1000ULL), static_cast<unsigned>(us % 1000ULL));
}

/**
 * @brief thread list：遍历注册表，每个线程一行
 */
static void CmdList()
{
    Log::SendCommandLine("THREAD          STATE     RUN      RETRY   EXIT    DROP");

    const uint8_t count = thread_registry::Count();
    for (uint8_t i = 0; i < count; ++i)
    {
        init::ThreadBase* t = thread_registry::At(i);
        if (t == nullptr) continue;

        init::ThreadStats s {};
        if (!t->GetStats(s)) continue;

        char line[128];
        std::snprintf(line, sizeof(line), "%-15s %-9s %-8u %-7u %-7u %u",
                      s.name, StateName(s),
                      static_cast<unsigned>(s.run_count), static_cast<unsigned>(s.retry_count),
                      static_cast<unsigned>(s.exit_count), static_cast<unsigned>(s.drop_count));
        Log::SendCommandLine(line);
    }

    char tail[32];
    std::snprintf(tail, sizeof(tail), "--- %u threads ---", static_cast<unsigned>(count));
    Log::SendCommandLine(tail);
}

/**
 * @brief thread show：单线程详细统计
 * @param name 线程名
 */
static void CmdShow(const char* name)
{
    if (name == nullptr || *name == '\0') {
        Log::SendCommandLine("usage: thread show <name>");
        return;
    }

    init::ThreadBase* t = thread_registry::Find(name);
    if (t == nullptr) {
        Log::SendCommandLine("thread show: not found");
        return;
    }

    init::ThreadStats s {};
    if (!t->GetStats(s)) {
        Log::SendCommandLine("thread show: not started");
        return;
    }

    char line[128];
    char ms[32];

    std::snprintf(line, sizeof(line), "%s:", s.name);
    Log::SendCommandLine(line);

    std::snprintf(line, sizeof(line), "  state        %s", StateName(s));
    Log::SendCommandLine(line);

    std::snprintf(line, sizeof(line), "  run          %u", static_cast<unsigned>(s.run_count));
    Log::SendCommandLine(line);

    std::snprintf(line, sizeof(line), "  retry/exit   %u/%u", static_cast<unsigned>(s.retry_count), static_cast<unsigned>(s.exit_count));
    Log::SendCommandLine(line);

    std::snprintf(line, sizeof(line), "  drop         %u", static_cast<unsigned>(s.drop_count));
    Log::SendCommandLine(line);

    FormatMs(s.last_run_ns, ms, sizeof(ms));
    std::snprintf(line, sizeof(line), "  last         %s ms", ms);
    Log::SendCommandLine(line);

    FormatMs(s.max_run_ns, ms, sizeof(ms));
    std::snprintf(line, sizeof(line), "  max          %s ms", ms);
    Log::SendCommandLine(line);

    FormatMs(s.run_ns, ms, sizeof(ms));
    std::snprintf(line, sizeof(line), "  total        %s ms", ms);
    Log::SendCommandLine(line);
}

/**
 * @brief thread pause：暂停线程（在下一拍前挂起）
 * @param name 线程名
 */
static void CmdPause(const char* name)
{
    if (name == nullptr || *name == '\0') {
        Log::SendCommandLine("usage: thread pause <name>");
        return;
    }

    init::ThreadBase* t = thread_registry::Find(name);
    if (t == nullptr) {
        Log::SendCommandLine("thread pause: not found");
        return;
    }

    t->Pause();
    Log::SendCommandLine("thread pause: ok");
}

/**
 * @brief thread resume：恢复被暂停的线程
 * @param name 线程名
 */
static void CmdResume(const char* name)
{
    if (name == nullptr || *name == '\0') {
        Log::SendCommandLine("usage: thread resume <name>");
        return;
    }

    init::ThreadBase* t = thread_registry::Find(name);
    if (t == nullptr) {
        Log::SendCommandLine("thread resume: not found");
        return;
    }

    t->Resume();
    Log::SendCommandLine("thread resume: ok");
}

/**
 * @brief thread exit：请求终止线程（协作式，异步生效）
 *
 * 带保留标记的线程（shell）拒绝终止。
 *
 * @param name 线程名
 */
static void CmdExit(const char* name)
{
    if (name == nullptr || *name == '\0') {
        Log::SendCommandLine("usage: thread exit <name>");
        return;
    }

    init::ThreadBase* t = thread_registry::Find(name);
    if (t == nullptr) {
        Log::SendCommandLine("thread exit: not found");
        return;
    }

    if (t->KeepAlive()) {
        Log::SendCommandLine("thread exit: reserved, refused");
        return;
    }

    t->Exit();
    Log::SendCommandLine("thread exit: requested");
}

/**
 * @brief thread reset：清零线程计数器
 * @param name 线程名
 */
static void CmdReset(const char* name)
{
    if (name == nullptr || *name == '\0') {
        Log::SendCommandLine("usage: thread reset <name>");
        return;
    }

    init::ThreadBase* t = thread_registry::Find(name);
    if (t == nullptr) {
        Log::SendCommandLine("thread reset: not found");
        return;
    }

    t->ResetStats();
    Log::SendCommandLine("thread reset: ok");
}

/**
 * @brief thread list：遍历注册表，每个线程一行
 * @param args 未使用
 */
static void CmdListSub(uint8_t* args)
{
    (void)args;
    CmdList();
}

/**
 * @brief thread show <name>：单线程详细统计
 * @param args 剩余行（"<name>"）
 */
static void CmdShowSub(uint8_t* args)
{
    CmdShow(reinterpret_cast<const char*>(args));
}

/**
 * @brief thread pause <name>：暂停线程
 * @param args 剩余行（"<name>"）
 */
static void CmdPauseSub(uint8_t* args)
{
    CmdPause(reinterpret_cast<const char*>(args));
}

/**
 * @brief thread resume <name>：恢复线程
 * @param args 剩余行（"<name>"）
 */
static void CmdResumeSub(uint8_t* args)
{
    CmdResume(reinterpret_cast<const char*>(args));
}

/**
 * @brief thread exit <name>：请求终止线程
 * @param args 剩余行（"<name>"）
 */
static void CmdExitSub(uint8_t* args)
{
    CmdExit(reinterpret_cast<const char*>(args));
}

/**
 * @brief thread reset <name>：清零计数器
 * @param args 剩余行（"<name>"）
 */
static void CmdResetSub(uint8_t* args)
{
    CmdReset(reinterpret_cast<const char*>(args));
}

/**
 * @brief 未命中子命令：打印用法
 * @param args 未使用
 */
static void CmdUsage(uint8_t* args)
{
    (void)args;
    Log::SendCommandLine("thread list|show|pause|resume|exit|reset <name>");
}

/**
 * @brief thread 命令入口：查表分派子命令
 * @param line 子命令参数（不含 "thread" 前缀）
 */
static void Process(uint8_t* line)
{
    static constexpr SubCmdEntry kSubCmds[] = {
        {"list",   CmdListSub  },
        {"show",   CmdShowSub  },
        {"pause",  CmdPauseSub },
        {"resume", CmdResumeSub},
        {"exit",   CmdExitSub  },
        {"reset",  CmdResetSub },
    };

    DispatchSubCmd(line, kSubCmds, sizeof(kSubCmds) / sizeof(kSubCmds[0]), CmdUsage);
}

REGISTER_SHELL_CMD("thread", "thread list|show|pause|resume|exit|reset", Process);

} // namespace debug
