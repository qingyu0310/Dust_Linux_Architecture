# 海康模块只提供设备能力，不注册线程或读取项目配置。
set(_hikrobot_default_root "/opt/MVS")
if(NOT EXISTS "${_hikrobot_default_root}/include/MvCameraControl.h" AND
   EXISTS "$ENV{HOME}/.local/opt/hikrobot/MvCamCtrlSDK/include/MvCameraControl.h")
    set(_hikrobot_default_root "$ENV{HOME}/.local/opt/hikrobot/MvCamCtrlSDK")
endif()
set(HIKROBOT_MVS_ROOT "${_hikrobot_default_root}" CACHE PATH "Complete HikRobot MVS SDK root")
find_path(HIKROBOT_INCLUDE_DIR MvCameraControl.h
    HINTS "${HIKROBOT_MVS_ROOT}/include")
find_library(HIKROBOT_LIBRARY NAMES MvCameraControl
    HINTS "${HIKROBOT_MVS_ROOT}/lib/64" "${HIKROBOT_MVS_ROOT}/lib/aarch64")
if(NOT HIKROBOT_INCLUDE_DIR OR NOT HIKROBOT_LIBRARY)
    message(FATAL_ERROR "HikRobot SDK missing: set HIKROBOT_MVS_ROOT to a complete MVS SDK")
endif()
get_filename_component(HIKROBOT_LIBRARY_DIR "${HIKROBOT_LIBRARY}" DIRECTORY)
if(NOT EXISTS "${HIKROBOT_LIBRARY_DIR}/libMvUsb3vTL.so")
    message(FATAL_ERROR "HikRobot USB transport missing under ${HIKROBOT_LIBRARY_DIR}")
endif()

add_library(hikrobot_mvs SHARED IMPORTED)
set_target_properties(hikrobot_mvs PROPERTIES
    IMPORTED_LOCATION "${HIKROBOT_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${HIKROBOT_INCLUDE_DIR}")
add_library(framework_modules STATIC camera/hikrobot.cpp)
target_include_directories(framework_modules PUBLIC ${PROJECT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(framework_modules PUBLIC framework_core ${OpenCV_LIBS} PRIVATE hikrobot_mvs)
target_compile_options(framework_modules PRIVATE -Wall -Wextra -Wpedantic)
