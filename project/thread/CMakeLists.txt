# ============================================================================
#
# project/thread/CMakeLists.txt —— 业务线程装配
#
# 每个线程一个目录 + 自己的 Kconfig 开关；注册源直接进可执行程序
#
# ============================================================================

if(CONFIG_TRD_TEST)
    target_sources(dust_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test/trd_test.cpp)
    target_include_directories(dust_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
endif()

if(CONFIG_TRD_ETHERCAT)
    target_sources(dust_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ethercat/trd_ethercat.cpp)
    target_include_directories(dust_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ethercat)
endif()
