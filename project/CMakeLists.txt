# 注册源直接进入可执行程序，避免静态库未引用对象被裁剪。
add_executable(vision_app ${PROJECT_SOURCE_DIR}/framework/init/main.cpp
    ${PROJECT_SOURCE_DIR}/framework/cmd/shell/shell.cpp)
add_subdirectory(thread)
target_link_libraries(vision_app PRIVATE framework_core framework_algorithm framework_topic framework_modules)
# MVS 动态加载 USB 传输层，其依赖也须从同一 SDK 目录解析。
get_filename_component(_hikrobot_runtime_dir "${HIKROBOT_LIBRARY}" DIRECTORY)
set_property(TARGET vision_app APPEND PROPERTY BUILD_RPATH "${_hikrobot_runtime_dir}")
target_link_options(vision_app PRIVATE "LINKER:--disable-new-dtags")
target_compile_options(vision_app PRIVATE -Wall -Wextra -Wpedantic -ffunction-sections -fdata-sections)
target_link_options(vision_app PRIVATE
    "LINKER:-T,${PROJECT_SOURCE_DIR}/framework/init/dust_init.ld"
    "LINKER:--gc-sections" "LINKER:-Map,${CMAKE_CURRENT_BINARY_DIR}/vision_app.map")
set_property(TARGET vision_app APPEND PROPERTY LINK_DEPENDS
    ${PROJECT_SOURCE_DIR}/framework/init/dust_init.ld)
