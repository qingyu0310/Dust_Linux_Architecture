cmake_minimum_required(VERSION 3.16)
project(linux_vision LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc dnn)

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

set(ARMOR_ENGINE_PATH "${CMAKE_BINARY_DIR}/0526.engine" CACHE FILEPATH
    "TensorRT engine compiled from temp/Dust_infantry_vision-main/assets/0526.onnx")
get_filename_component(ARMOR_ENGINE_PATH "${ARMOR_ENGINE_PATH}" ABSOLUTE
    BASE_DIR "${CMAKE_BINARY_DIR}")
if(NOT EXISTS "${ARMOR_ENGINE_PATH}")
    message(FATAL_ERROR
        "Armor TensorRT engine not found: ${ARMOR_ENGINE_PATH}. Build 0526.onnx with .venv/bin/trtexec first.")
endif()

message(STATUS "CUDA Toolkit: ${CUDAToolkit_VERSION} (${CUDAToolkit_ROOT})")
message(STATUS
    "TensorRT: ${TensorRT_VERSION_MAJOR}.${TensorRT_VERSION_MINOR}.${TensorRT_VERSION_PATCH}; include=${TensorRT_INCLUDE_DIR}; library=${TensorRT_LIBRARY}")
message(STATUS "Armor engine: ${ARMOR_ENGINE_PATH}")
add_subdirectory(framework)
add_subdirectory(project)
