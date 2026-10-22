# ============================================================================
#
# project/thread/CMakeLists.txt —— 业务线程装配
#
# 每个线程一个目录 + 自己的 Kconfig 开关；注册源直接进可执行程序
#
# ============================================================================

# ============================================================================
#
# 线程源文件
#
# ============================================================================

if(CONFIG_TRD_TEST)
    target_sources(vision_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test/trd_test.cpp)
endif()

if(CONFIG_TRD_CAMERA)
    target_sources(vision_app PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/camera/trd_camera.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/camera/trd_camera_cmd.cpp)
    target_include_directories(vision_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/camera)
endif()

if(CONFIG_TRD_CALIB)
    target_sources(vision_app PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/calib/trd_calib.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/calib/trd_calib_config.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/calib/trd_calib_sample.cpp)
    target_include_directories(vision_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/calib)
endif()

if(CONFIG_TRD_DISPLAY)
    target_sources(vision_app PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/display/trd_display.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/display/trd_display_var.cpp)
    target_include_directories(vision_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/display)
endif()

if(CONFIG_TRD_VISION)
    target_sources(vision_app PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/vision/trd_vision.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/vision/trd_vision_config.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/vision/trd_vision_draw.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/vision/trd_vision_report.cpp)
    target_include_directories(vision_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/vision)
endif()

# ============================================================================
#
# 线程运行 YAML：放到可执行文件旁，程序按可执行文件目录解析相对路径
#
# ============================================================================

foreach(_thread_yaml camera/trd_camera.yaml calib/trd_calib.yaml display/trd_display.yaml vision/trd_vision.yaml)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${_thread_yaml}
                   ${CMAKE_BINARY_DIR}/project/thread/${_thread_yaml} COPYONLY)
endforeach()
