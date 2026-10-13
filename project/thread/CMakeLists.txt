# 项目业务线程统一装配，注册源直接加入可执行程序。

target_sources(vision_app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test/trd_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/camera/trd_camera.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/display/trd_display.cpp)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/test/trd_test.yaml
    ${CMAKE_CURRENT_BINARY_DIR}/test/trd_test.yaml
    COPYONLY)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/camera/trd_camera.yaml
    ${CMAKE_CURRENT_BINARY_DIR}/camera/trd_camera.yaml
    COPYONLY)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/display/trd_display.yaml
    ${CMAKE_CURRENT_BINARY_DIR}/display/trd_display.yaml
    COPYONLY)
