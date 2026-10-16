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
# algorithm：算法能力（纯头文件能力 + 视觉、标定静态库）
#
# ============================================================================

add_library(framework_algorithm INTERFACE)
target_include_directories(framework_algorithm INTERFACE ${PROJECT_SOURCE_DIR})
target_link_libraries(framework_algorithm INTERFACE Eigen3::Eigen)

if(CONFIG_DUST_VISION_YOLO OR CONFIG_DUST_VISION_POSE)
    add_library(framework_vision STATIC)
    if(CONFIG_DUST_VISION_YOLO)
        target_sources(framework_vision PRIVATE
            algorithm/vision/core/detections.cpp
            algorithm/vision/core/light_bar.cpp
            algorithm/vision/detect/model_decode.cpp
            algorithm/vision/detect/armor_detector.cpp)
    endif()
    if(CONFIG_DUST_VISION_YOLO_REFINER)
        target_sources(framework_vision PRIVATE algorithm/vision/refine/armor_refiner.cpp)
    endif()
    if(CONFIG_DUST_VISION_POSE)
        target_sources(framework_vision PRIVATE algorithm/vision/pose/armor_pose.cpp)
    endif()
    if(CONFIG_DUST_VISION_TRACK)
        target_sources(framework_vision PRIVATE algorithm/vision/track/armor_tracker.cpp)
    endif()
    target_include_directories(framework_vision PUBLIC ${PROJECT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(framework_vision PUBLIC opencv_core opencv_imgproc)
    if(CONFIG_DUST_VISION_YOLO)
        target_link_libraries(framework_vision PUBLIC opencv_dnn PRIVATE CUDA::cudart TensorRT::nvinfer)
    endif()
    if(CONFIG_DUST_VISION_POSE)
        target_link_libraries(framework_vision PUBLIC opencv_calib3d)
    endif()
    target_link_libraries(framework_vision PUBLIC Eigen3::Eigen)
    target_compile_options(framework_vision PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(CONFIG_DUST_CALIB)
    add_library(framework_calib STATIC
        algorithm/calib/camera_calib.cpp)
    target_include_directories(framework_calib PUBLIC ${PROJECT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(framework_calib PUBLIC opencv_core opencv_calib3d)
    target_compile_options(framework_calib PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ============================================================================
#
# topic：线程间数据通道契约（纯头文件）
#
# ============================================================================

add_library(framework_topic INTERFACE)
target_include_directories(framework_topic INTERFACE ${PROJECT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(framework_topic INTERFACE ${OpenCV_LIBS})

# ============================================================================
#
# core：init 注册与启动（始终编译）+ 按 cmd 层能力裁剪的调试/配置源
#
# ============================================================================

add_library(framework_core STATIC
    init/Init_entry.cpp 
    init/signal_handler.cpp)
if(CONFIG_DUST_CMD_SHELL_LOG)
    target_sources(framework_core PRIVATE cmd/shell/log.cpp cmd/shell/log_transport.cpp)
endif()
if(CONFIG_DUST_CMD_SHELL_VAR)
    target_sources(framework_core PRIVATE cmd/shell/var.cpp)
endif()
if(CONFIG_DUST_CMD_CONFIG)
    target_sources(framework_core PRIVATE cmd/config/config.cpp)
endif()
target_include_directories(framework_core PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(framework_core PUBLIC Threads::Threads yaml-cpp)
target_compile_options(framework_core PRIVATE -Wall -Wextra -Wpedantic)

# ============================================================================
#
# modules：设备模块能力
#
# ============================================================================

add_subdirectory(modules)
