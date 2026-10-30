# ============================================================================
#
# framework/drivers/CMakeLists.txt —— 底层通讯/设备驱动装配
#
# 骨架阶段没有驱动实现，只导出 include 路径。
# 新增驱动时照 CONFIG_DUST_COM_* / CONFIG_DUST_DEV_* 加块，
# 源文件挂到 framework_drivers 上。
#
# ============================================================================

add_library(framework_drivers INTERFACE)
target_include_directories(framework_drivers INTERFACE ${PROJECT_SOURCE_DIR})

# ============================================================================
#
# CONFIG_DUST_COM_ETHERCAT：用户态 EtherCAT 主站（SOEM）
#
# ============================================================================

if(CONFIG_DUST_COM_ETHERCAT)
    find_path(SOEM_INCLUDE_DIR soem/soem.h HINTS /usr/local/include /usr/include)
    find_library(SOEM_LIBRARY NAMES soem HINTS /usr/local/lib /usr/lib)

    if(NOT SOEM_INCLUDE_DIR OR NOT SOEM_LIBRARY)
        message(FATAL_ERROR
            "SOEM not found. Build and install SOEM first:\n"
            "  git clone https://github.com/OpenEtherCATsociety/SOEM.git ~/src/SOEM\n"
            "  cmake -S ~/src/SOEM -B ~/src/SOEM/build -DCMAKE_BUILD_TYPE=Release\n"
            "  cmake --build ~/src/SOEM/build -j\n"
            "  sudo cmake --install ~/src/SOEM/build")
    endif()

    add_library(framework_ethercat INTERFACE)
    target_include_directories(framework_ethercat INTERFACE
        ${PROJECT_SOURCE_DIR}
        ${SOEM_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/framework/shell/log
        ${PROJECT_SOURCE_DIR}/framework/shell/log/process
        ${PROJECT_SOURCE_DIR}/framework/shell/log/queue
        ${PROJECT_SOURCE_DIR}/framework/shell/log/transport
        ${PROJECT_SOURCE_DIR}/framework/shell/log/editor)
    target_link_libraries(framework_ethercat INTERFACE
        ${SOEM_LIBRARY} Threads::Threads)
endif()
