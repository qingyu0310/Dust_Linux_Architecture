cmake_minimum_required(VERSION 3.16)
project(linux_vision LANGUAGES CXX)

# ============================================================================
#
# 编译环境
#
# ============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================================
#
# Kconfig 功能选择：开关写在 Kconfig 文件里，CMake 只消费 CONFIG_*
#
# 生成物：<build>/kconfig/{.config,kconfig.cmake,autoconf.h}
# KCONFIG_CONFIG 可指定一份已存在的 .config 覆盖 Kconfig 里的 default
#
# ============================================================================

set(KCONFIG_CONFIG "" CACHE FILEPATH
    "已存在的 .config；留空表示直接用 Kconfig 文件里的 default")
set(KCONFIG_OUT "${CMAKE_BINARY_DIR}/kconfig" CACHE PATH "Kconfig 生成目录")
set(KCONFIG_PYTHON "python3" CACHE FILEPATH "运行 scripts/kconfig.py 的解释器")

set(_kconfig_inputs "")
if(KCONFIG_CONFIG)
    if(NOT EXISTS "${KCONFIG_CONFIG}")
        message(FATAL_ERROR "Kconfig configuration file not found: ${KCONFIG_CONFIG}")
    endif()
    set(_kconfig_inputs --config "${KCONFIG_CONFIG}")
endif()

execute_process(
    COMMAND "${KCONFIG_PYTHON}" "${PROJECT_SOURCE_DIR}/scripts/kconfig.py" --kconfig "${PROJECT_SOURCE_DIR}/Kconfig" ${_kconfig_inputs} --out "${KCONFIG_OUT}"
    RESULT_VARIABLE _kconfig_result
    OUTPUT_VARIABLE _kconfig_summary
    ERROR_VARIABLE _kconfig_errors)
if(NOT _kconfig_result EQUAL 0)
    message(FATAL_ERROR "Kconfig 求值失败：\n${_kconfig_summary}${_kconfig_errors}")
endif()
message(STATUS "${_kconfig_summary}")

include("${KCONFIG_OUT}/kconfig.cmake")
include_directories("${KCONFIG_OUT}")
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-include "${KCONFIG_OUT}/autoconf.h")
endif()

# ============================================================================
#
# 通用依赖
#
# ============================================================================

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

# ============================================================================
#
# 视觉依赖（CONFIG_DUST_VISION_DETECT）：OpenCV / CUDA / TensorRT
#
# 骨架阶段没有视觉业务，相关符号在 framework/algorithm/Kconfig 中已移除，
# 这里整段不再触发。新增视觉能力时把对应 find_package 段加回来。
#
# ============================================================================

if(CONFIG_DUST_VISION_DETECT)
    message(FATAL_ERROR "Dust vision capability is not part of the Linux skeleton")
endif()

# ============================================================================
#
# 装配
#
# ============================================================================

add_subdirectory(framework)
add_subdirectory(project)
