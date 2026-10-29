# ============================================================================
#
# project/CMakeLists.txt —— 业务工程装配
#
# 注册源直接进入可执行程序，避免静态库未引用对象被裁剪
#
# ============================================================================

# ============================================================================
#
# 可执行程序与注册源
#
# ============================================================================

add_executable(dust_app ${PROJECT_SOURCE_DIR}/framework/init/startup/main.cpp)

# 带 REGISTER_SHELL_CMD 的源必须直接进可执行程序：静态库里没人引用的对象
# 会被链接器整份丢弃，注册项就进不了 .shell_cmd 段。
if(CONFIG_DUST_CMD_SHELL)
    target_sources(dust_app PRIVATE
        ${PROJECT_SOURCE_DIR}/framework/shell/shell.cpp
        ${PROJECT_SOURCE_DIR}/framework/shell/boot/boot_cmd.cpp
        ${PROJECT_SOURCE_DIR}/framework/shell/thread/thread_cmd.cpp)
endif()

if(CONFIG_DUST_CMD_SHELL_VAR)
    target_sources(dust_app PRIVATE ${PROJECT_SOURCE_DIR}/framework/shell/var/var.cpp)
endif()

if(CONFIG_DUST_CMD_SHELL_LOG)
    target_sources(dust_app PRIVATE ${PROJECT_SOURCE_DIR}/framework/shell/log/cmd/log_cmd.cpp)
endif()

# 业务线程：各线程按自己的 Kconfig 开关装配
add_subdirectory(thread)

# ============================================================================
#
# 链接依赖
#
# ============================================================================

target_link_libraries(dust_app PRIVATE
    framework_core framework_algorithm framework_topic)

if(CONFIG_DUST_COM_ETHERCAT)
    target_link_libraries(dust_app PRIVATE framework_ethercat)
endif()

# ============================================================================
#
# 编译与链接选项：注册段靠链接脚本保留
#
# ============================================================================

target_compile_options(dust_app PRIVATE
    -Wall -Wextra -Wpedantic -ffunction-sections -fdata-sections)
target_link_options(dust_app PRIVATE "LINKER:--disable-new-dtags")
target_link_options(dust_app PRIVATE
    "LINKER:-T,${PROJECT_SOURCE_DIR}/framework/cmd/linker/dust_init.ld"
    "LINKER:--gc-sections"
    "LINKER:-Map,${CMAKE_CURRENT_BINARY_DIR}/dust_app.map")
set_property(TARGET dust_app APPEND PROPERTY LINK_DEPENDS
    ${PROJECT_SOURCE_DIR}/framework/cmd/linker/dust_init.ld)
