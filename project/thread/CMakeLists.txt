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
    target_sources(vision_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/camera/trd_camera.cpp)
endif()

if(CONFIG_TRD_CALIB)
    target_sources(vision_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/calib/trd_calib.cpp)
endif()

if(CONFIG_TRD_DISPLAY)
    target_sources(vision_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/display/trd_display.cpp)
endif()

if(CONFIG_TRD_VISION)
    target_sources(vision_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/vision/trd_vision.cpp)
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
