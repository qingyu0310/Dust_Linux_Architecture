find_package(yaml-cpp REQUIRED)

add_library(framework_algorithm INTERFACE)
target_include_directories(framework_algorithm INTERFACE ${PROJECT_SOURCE_DIR})
target_link_libraries(framework_algorithm INTERFACE Eigen3::Eigen)

add_library(framework_topic INTERFACE)
target_include_directories(framework_topic INTERFACE ${PROJECT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(framework_topic INTERFACE ${OpenCV_LIBS})

add_library(framework_core STATIC
    cmd/shell/log.cpp 
    cmd/shell/log_transport.cpp 
    cmd/shell/var.cpp 
    cmd/config/config.cpp
    init/Init_entry.cpp 
    init/signal_handler.cpp)
target_include_directories(framework_core PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(framework_core PUBLIC Threads::Threads yaml-cpp)
target_compile_options(framework_core PRIVATE -Wall -Wextra -Wpedantic)

add_subdirectory(modules)
