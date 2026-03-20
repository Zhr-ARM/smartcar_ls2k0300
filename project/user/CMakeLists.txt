# 包含交叉编译工具链
include(cross.cmake)

# 设置CMake的最低版本要求
cmake_minimum_required(VERSION 3.5.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -Wall")  # 对于 C++ 编译器
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall")      # 对于 C 编译器

# 仅在开启该选项时编译逐飞助手网络传图逻辑
option(ENABLE_SEEKFREE_ASSISTANT_CAMERA_STREAM "Enable Seekfree assistant camera streaming" OFF)
if(ENABLE_SEEKFREE_ASSISTANT_CAMERA_STREAM)
    add_compile_definitions(ENABLE_SEEKFREE_ASSISTANT_CAMERA_STREAM=1)
endif()

# 仅在开启该选项时编译和启用车载屏显线程
option(ENABLE_CAR_SCREEN_DISPLAY "Enable on-board screen display thread" ON)
if(ENABLE_CAR_SCREEN_DISPLAY)
    add_compile_definitions(ENABLE_CAR_SCREEN_DISPLAY=1)
endif()

# 关屏模式下默认启用图传，避免屏显开关影响图传链路。
if(NOT ENABLE_CAR_SCREEN_DISPLAY)
    set(ENABLE_SEEKFREE_ASSISTANT_CAMERA_STREAM ON CACHE BOOL "Enable Seekfree assistant camera streaming" FORCE)
endif()

# E99视觉栈开关：开启时尝试启用ncnn相关模块（pipeline/client_sender/ncnn）。
option(ENABLE_E99_VISION_STACK "Enable E99 vision full stack" ON)
set(NCNN_ROOT "" CACHE PATH "Path to an NCNN install prefix containing include/ and lib/")

set(E99_VISION_STACK_READY OFF)
if(ENABLE_E99_VISION_STACK)
    set(_ncnn_search_roots "")
    if(NCNN_ROOT)
        list(APPEND _ncnn_search_roots "${NCNN_ROOT}")
    endif()
    if(DEFINED ENV{NCNN_ROOT} AND NOT "$ENV{NCNN_ROOT}" STREQUAL "")
        list(APPEND _ncnn_search_roots "$ENV{NCNN_ROOT}")
    endif()

    list(APPEND _ncnn_search_roots
        "${CMAKE_CURRENT_LIST_DIR}/../../libraries/zf_components/ncnn"
        "/home/zhr/.local/share/Trash/files/LS2K0300_Library/LS2K300_Library/Seekfree_LS2K0300_Opensource_Library/libraries/zf_components/ncnn")
    list(REMOVE_DUPLICATES _ncnn_search_roots)

    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(NCNN QUIET ncnn)
    endif()

    if(NOT NCNN_FOUND)
        find_path(NCNN_INCLUDE_DIR
            NAMES ncnn/net.h
            PATHS ${_ncnn_search_roots}
            PATH_SUFFIXES include
            NO_CMAKE_FIND_ROOT_PATH)
        find_library(NCNN_LIBRARY
            NAMES ncnn
            PATHS ${_ncnn_search_roots}
            PATH_SUFFIXES lib
            NO_CMAKE_FIND_ROOT_PATH)

        if(NCNN_INCLUDE_DIR AND NCNN_LIBRARY)
            set(NCNN_FOUND TRUE)
            set(NCNN_INCLUDE_DIRS ${NCNN_INCLUDE_DIR})
            get_filename_component(NCNN_LIBRARY_DIR "${NCNN_LIBRARY}" DIRECTORY)
            set(NCNN_LIBRARY_DIRS ${NCNN_LIBRARY_DIR})
            set(NCNN_LIBRARIES ${NCNN_LIBRARY})
        endif()
    endif()

    if(NCNN_FOUND)
        set(E99_VISION_STACK_READY ON)
        add_compile_definitions(ENABLE_E99_VISION_STACK=1)
        include_directories(${NCNN_INCLUDE_DIRS})
        link_directories(${NCNN_LIBRARY_DIRS})
        message(STATUS "NCNN_INCLUDE_DIRS: ${NCNN_INCLUDE_DIRS}")
        message(STATUS "NCNN_LIBRARIES: ${NCNN_LIBRARIES}")
    else()
        message(STATUS "NCNN not found, build will skip ncnn-related E99 vision sources. Set NCNN_ROOT to an install prefix if needed.")
    endif()
endif()

# 获取当前 CMakeLists.txt 文件所在目录的完整路径
set(CURRENT_DIR ${CMAKE_CURRENT_LIST_DIR})

# 获取上上级目录的完整路径
get_filename_component(PARENT_PARENT_DIR ${CURRENT_DIR}/.. ABSOLUTE)

# 从完整路径中提取文件夹名称
get_filename_component(PARENT_PARENT_FOLDER_NAME ${PARENT_PARENT_DIR} NAME)

# 打印上上级文件夹名称
message("Parent parent folder name: ${PARENT_PARENT_FOLDER_NAME}")

# 定义项目名称和版本，并指定使用C和C++语言
project(${PARENT_PARENT_FOLDER_NAME} VERSION 0.1.0 LANGUAGES C CXX)

# 设置OpenCV的安装路径
set(CMAKE_PREFIX_PATH "/opt/ls_2k0300_env/opencv_4_10_build")

# 查找OpenCV库，确保安装了所需的依赖
find_package(OpenCV REQUIRED)

# 包含OpenCV的头文件路径
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")

# 项目目录
set(PROJECT_CODE_DIR ../code)
set(PROJECT_DRIVER_DIR ${PROJECT_CODE_DIR}/driver)
set(PROJECT_APP_DIR ${PROJECT_CODE_DIR}/app)

# 包含项目头文件路径
include_directories(${PROJECT_CODE_DIR})
include_directories(${PROJECT_APP_DIR}/uart_thread)
include_directories(${PROJECT_APP_DIR}/motor_thread)
include_directories(${PROJECT_APP_DIR}/imu_thread)
include_directories(${PROJECT_APP_DIR}/line_follow_thread)
include_directories(${PROJECT_APP_DIR}/vision_thread)
include_directories(${PROJECT_APP_DIR}/screen_display_thread)
include_directories(${PROJECT_DRIVER_DIR}/battery)
include_directories(${PROJECT_DRIVER_DIR}/brushless)
include_directories(${PROJECT_DRIVER_DIR}/imu660ra)
include_directories(${PROJECT_DRIVER_DIR}/motor)
include_directories(${PROJECT_DRIVER_DIR}/pid)
include_directories(${PROJECT_DRIVER_DIR}/uart)
include_directories(${PROJECT_DRIVER_DIR}/uart_pid_protocol)
include_directories(${PROJECT_DRIVER_DIR}/vision)
include_directories(../../libraries/zf_common)
include_directories(../../libraries/zf_device)
include_directories(../../libraries/zf_driver)
include_directories(../../libraries/zf_components)

# 收集源文件
file(GLOB_RECURSE PROJECT_SRCS CONFIGURE_DEPENDS
    ${PROJECT_DRIVER_DIR}/*.cpp
    ${PROJECT_APP_DIR}/*.cpp)

if(NOT E99_VISION_STACK_READY)
    list(FILTER PROJECT_SRCS EXCLUDE REGEX ".*/driver/vision/lq_ncnn\\.cpp$")
    list(FILTER PROJECT_SRCS EXCLUDE REGEX ".*/driver/vision/vision_ncnn\\.cpp$")
    list(FILTER PROJECT_SRCS EXCLUDE REGEX ".*/driver/vision/vision_pipeline\\.cpp$")
    list(FILTER PROJECT_SRCS EXCLUDE REGEX ".*/driver/vision/vision_client_sender\\.cpp$")
endif()

file(GLOB LIBRARY_SRCS CONFIGURE_DEPENDS
    ../../libraries/zf_common/*.cpp
    ../../libraries/zf_device/*.cpp
    ../../libraries/zf_driver/*.cpp
    ../../libraries/zf_components/*.cpp)

# 创建可执行文件，使用项目名称作为目标名称
add_executable(${PROJECT_NAME}
    ../user/main.cpp
    ${PROJECT_SRCS}
    ${LIBRARY_SRCS})

# 链接OpenCV库到可执行文件
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

if(E99_VISION_STACK_READY)
    target_link_libraries(${PROJECT_NAME} ${NCNN_LIBRARIES})
endif()
