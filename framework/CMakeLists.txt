# ============================================================================
#
# framework/CMakeLists.txt —— 架构层各层装配
#
# 每层一个 target；可裁的源按 CONFIG_* 决定（开关定义在各层 Kconfig）
#
# ============================================================================

# 架构层内部依赖
find_package(yaml-cpp REQUIRED)

# ============================================================================
#
# algorithm：算法能力（纯头文件能力）
#
# ============================================================================

add_library(framework_algorithm INTERFACE)
target_include_directories(framework_algorithm INTERFACE ${PROJECT_SOURCE_DIR})
target_link_libraries(framework_algorithm INTERFACE Eigen3::Eigen)

# ============================================================================
#
# topic：线程间数据通道契约（纯头文件）
#
# ============================================================================

add_library(framework_topic INTERFACE)
target_include_directories(framework_topic INTERFACE ${PROJECT_SOURCE_DIR})

# ============================================================================
#
# core：init 注册与启动（始终编译）+ 按 cmd 层能力裁剪的调试/配置源
#
# ============================================================================

add_library(framework_core STATIC
    init/startup/Init_entry.cpp
    init/startup/boot_timing.cpp
    init/signal_handler.cpp
    init/thread/thread_fault.cpp)
if(CONFIG_DUST_CMD_CONFIG)
    target_sources(framework_core PRIVATE cmd/config/config.cpp)
endif()
# 各子目录加入全局 include：业务线程仍然只写 #include "thread.hpp" / "Init_entry.hpp"
target_include_directories(framework_core PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/init/startup
    ${CMAKE_CURRENT_SOURCE_DIR}/init/thread)
target_link_libraries(framework_core PUBLIC Threads::Threads yaml-cpp)
target_compile_options(framework_core PRIVATE -Wall -Wextra -Wpedantic)

# ============================================================================
#
# shell：调试台底座 + 子命令族（boot / log / var / thread）
#
# ============================================================================

add_subdirectory(shell)

# ============================================================================
#
# modules：设备模块能力（骨架无设备模块，保留空装配位）
#
# ============================================================================

add_subdirectory(modules)
