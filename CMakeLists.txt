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
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)

if(CONFIG_DUST_VISION_YOLO)
    find_package(OpenCV REQUIRED COMPONENTS dnn)
endif()

if(CONFIG_DUST_CALIB OR CONFIG_DUST_VISION_POSE)
    find_package(OpenCV REQUIRED COMPONENTS calib3d imgcodecs)
endif()

# ============================================================================
#
# 视觉依赖（CONFIG_DUST_VISION_YOLO）：CUDA / TensorRT
#
# ============================================================================

if(CONFIG_DUST_VISION_YOLO)
    set(DUST_DEPENDENCY_ROOT "${PROJECT_SOURCE_DIR}/.venv" CACHE PATH
        "Root of the project-local CUDA and TensorRT environment")
    if(NOT DEFINED CUDAToolkit_ROOT)
        set(CUDAToolkit_ROOT "${DUST_DEPENDENCY_ROOT}" CACHE PATH "CUDA Toolkit root")
    endif()
    find_package(CUDAToolkit 12 REQUIRED)

    set(TensorRT_ROOT "${DUST_DEPENDENCY_ROOT}" CACHE PATH "TensorRT installation root")
    find_path(TensorRT_INCLUDE_DIR
        NAMES NvInfer.h
        HINTS "${TensorRT_ROOT}/include"
        REQUIRED)
    find_library(TensorRT_LIBRARY
        NAMES nvinfer
        HINTS "${TensorRT_ROOT}/lib" "${TensorRT_ROOT}/lib64"
        REQUIRED)

    set(_tensorrt_version_header "${TensorRT_INCLUDE_DIR}/NvInferVersion.h")
    if(NOT EXISTS "${_tensorrt_version_header}")
        message(FATAL_ERROR "TensorRT version header not found: ${_tensorrt_version_header}")
    endif()
    file(STRINGS "${_tensorrt_version_header}" _tensorrt_major_line
        REGEX "^#define (NV_TENSORRT_MAJOR|TRT_MAJOR_ENTERPRISE) [0-9]+")
    file(STRINGS "${_tensorrt_version_header}" _tensorrt_minor_line
        REGEX "^#define (NV_TENSORRT_MINOR|TRT_MINOR_ENTERPRISE) [0-9]+")
    file(STRINGS "${_tensorrt_version_header}" _tensorrt_patch_line
        REGEX "^#define (NV_TENSORRT_PATCH|TRT_PATCH_ENTERPRISE) [0-9]+")
    string(REGEX MATCH "[0-9]+$" TensorRT_VERSION_MAJOR "${_tensorrt_major_line}")
    string(REGEX MATCH "[0-9]+$" TensorRT_VERSION_MINOR "${_tensorrt_minor_line}")
    string(REGEX MATCH "[0-9]+$" TensorRT_VERSION_PATCH "${_tensorrt_patch_line}")
    if(NOT TensorRT_VERSION_MAJOR STREQUAL "10")
        message(FATAL_ERROR
            "TensorRT 10.x is required, found ${TensorRT_VERSION_MAJOR}.${TensorRT_VERSION_MINOR}.${TensorRT_VERSION_PATCH}")
    endif()

    add_library(TensorRT::nvinfer SHARED IMPORTED GLOBAL)
    set_target_properties(TensorRT::nvinfer PROPERTIES
        IMPORTED_LOCATION "${TensorRT_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${TensorRT_INCLUDE_DIR}")

    message(STATUS "CUDA Toolkit: ${CUDAToolkit_VERSION} (${CUDAToolkit_ROOT})")
    message(STATUS
        "TensorRT: ${TensorRT_VERSION_MAJOR}.${TensorRT_VERSION_MINOR}.${TensorRT_VERSION_PATCH}; include=${TensorRT_INCLUDE_DIR}; library=${TensorRT_LIBRARY}")
else()
    message(STATUS "CONFIG_DUST_VISION_YOLO=OFF: CUDA/TensorRT discovery is disabled")
endif()

# ============================================================================
#
# 装配
#
# ============================================================================

add_subdirectory(framework)
add_subdirectory(project)
