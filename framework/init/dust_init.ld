/**
 * @file dust_init.ld
 * @author qingyu
 * @brief ELF 注册指针表：保留初始化与线程段，导出遍历边界
 * @version 0.1
 * @date 2026-10-12
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
}
/* 补充默认 Linux 链接布局，不替换系统的完整链接脚本。 */
INSERT AFTER .data;
