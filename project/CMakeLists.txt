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

add_executable(vision_app ${PROJECT_SOURCE_DIR}/framework/init/main.cpp)

if(CONFIG_DUST_CMD_SHELL)
    target_sources(vision_app PRIVATE ${PROJECT_SOURCE_DIR}/framework/cmd/shell/shell.cpp)
endif()

# 业务线程：各线程按自己的 Kconfig 开关装配
add_subdirectory(thread)

# ============================================================================
#
# 链接依赖
#
# ============================================================================

target_link_libraries(vision_app PRIVATE
    framework_core framework_algorithm framework_topic)

if(CONFIG_DUST_MOD_DEV_HIKROBOT)
    target_link_libraries(vision_app PRIVATE framework_modules)
endif()

if(CONFIG_DUST_VISION_YOLO)
    target_link_libraries(vision_app PRIVATE framework_vision)
endif()

if(CONFIG_DUST_CALIB)
    target_link_libraries(vision_app PRIVATE framework_calib)
endif()

# ============================================================================
#
# 运行库路径：MVS 与 CUDA/TensorRT 都从各自 SDK 目录解析
#
# ============================================================================

get_filename_component(_hikrobot_runtime_dir "${HIKROBOT_LIBRARY}" DIRECTORY)
set_property(TARGET vision_app APPEND PROPERTY BUILD_RPATH "${_hikrobot_runtime_dir}")

if(CONFIG_DUST_VISION_YOLO)
    get_filename_component(_tensorrt_runtime_dir "${TensorRT_LIBRARY}" DIRECTORY)
    set_property(TARGET vision_app APPEND PROPERTY BUILD_RPATH
        "${_tensorrt_runtime_dir}" "${CUDAToolkit_LIBRARY_DIR}")
endif()

# ============================================================================
#
# 编译与链接选项：注册段靠链接脚本保留
#
# ============================================================================

target_compile_options(vision_app PRIVATE
    -Wall -Wextra -Wpedantic -ffunction-sections -fdata-sections)
target_link_options(vision_app PRIVATE "LINKER:--disable-new-dtags")
target_link_options(vision_app PRIVATE
    "LINKER:-T,${PROJECT_SOURCE_DIR}/framework/init/dust_init.ld"
    "LINKER:--gc-sections"
    "LINKER:-Map,${CMAKE_CURRENT_BINARY_DIR}/vision_app.map")
set_property(TARGET vision_app APPEND PROPERTY LINK_DEPENDS
    ${PROJECT_SOURCE_DIR}/framework/init/dust_init.ld)
