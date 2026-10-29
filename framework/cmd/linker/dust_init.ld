/**
 * @file dust_init.ld
 * @author qingyu
 * @brief ELF 注册指针表：保留初始化、线程、shell 命令与启动耗时段，导出遍历边界
 * @version 0.3
 * @date 2026-10-29
 *
 * @copyright Copyright (c) 2026
 *
 */

SECTIONS
{
    .user_init : ALIGN(8)
    {
        __user_init_start = .;
        KEEP(*(.user_init))
        __user_init_end = .;
    }
    .user_thread : ALIGN(8)
    {
        __user_thread_start = .;
        KEEP(*(.user_thread))
        __user_thread_end = .;
    }
    shell_var : ALIGN(8)
    {
        __start_shell_var = .;
        KEEP(*(shell_var))
        __stop_shell_var = .;
    }
    shell_cmd : ALIGN(8)
    {
        __start_shell_cmd = .;
        KEEP(*(shell_cmd))
        __stop_shell_cmd = .;
    }
    .boot_timing : ALIGN(8)
    {
        __boot_timing_start = .;
        KEEP(*(.boot_timing))
        __boot_timing_end = .;
    }
}
/* 补充默认 Linux 链接布局，不替换系统的完整链接脚本。 */
INSERT AFTER .data;
