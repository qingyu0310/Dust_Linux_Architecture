# shell 层：控制台底座 + 子命令分层（boot / log / var / thread）
#
# 带 REGISTER_SHELL_CMD 的源（shell.cpp / boot_cmd.cpp / thread_cmd.cpp /
# var.cpp / log_cmd.cpp）由 project/CMakeLists.txt 直接加进可执行程序：
# 静态库里没人引用的对象会被链接器整份丢弃，注册项就进不了 .shell_cmd 段。
# 这里只加 include 目录和「纯实现、无注册」的源。

target_include_directories(framework_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/boot
    ${CMAKE_CURRENT_SOURCE_DIR}/log
    ${CMAKE_CURRENT_SOURCE_DIR}/log/cmd
    ${CMAKE_CURRENT_SOURCE_DIR}/log/process
    ${CMAKE_CURRENT_SOURCE_DIR}/log/queue
    ${CMAKE_CURRENT_SOURCE_DIR}/log/transport
    ${CMAKE_CURRENT_SOURCE_DIR}/log/editor
    ${CMAKE_CURRENT_SOURCE_DIR}/var
    ${CMAKE_CURRENT_SOURCE_DIR}/thread
)

if(CONFIG_DUST_CMD_SHELL_LOG)
    target_sources(framework_core PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/log/log.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/log/process/log_process.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/log/transport/log_transport.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/log/editor/log_editor.cpp)
endif()
