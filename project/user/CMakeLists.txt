# 包含交叉编译工具链
include(cross.cmake)

# 设置CMake的最低版本要求
cmake_minimum_required(VERSION 3.5.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -Wall")  # 对于 C++ 编译器
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall")      # 对于 C 编译器

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

# 统一视觉主链：始终启用 ncnn 相关模块（pipeline/client_sender/ncnn）。
set(_local_ncnn_root "${CMAKE_CURRENT_LIST_DIR}/../../lib-ncnn")
set(_ncnn_install_root "${_local_ncnn_root}/tools/LQ_Dep_libs/ncnn_install")

    find_package(ncnn CONFIG QUIET
        HINTS "${_ncnn_install_root}"
        PATH_SUFFIXES lib/cmake/ncnn ncnn/cmake)
    if(ncnn_FOUND)
        set(NCNN_FOUND TRUE)
        if(TARGET ncnn::ncnn)
            set(NCNN_LIBRARIES ncnn::ncnn)
            get_target_property(_ncnn_iface_includes ncnn::ncnn INTERFACE_INCLUDE_DIRECTORIES)
        elseif(TARGET ncnn)
            set(NCNN_LIBRARIES ncnn)
            get_target_property(_ncnn_iface_includes ncnn INTERFACE_INCLUDE_DIRECTORIES)
        endif()
        if(_ncnn_iface_includes AND NOT NCNN_INCLUDE_DIRS)
            set(_ncnn_fixed_includes "")
            foreach(_inc ${_ncnn_iface_includes})
                list(APPEND _ncnn_fixed_includes "${_inc}")
                if(_inc MATCHES "/include/ncnn$")
                    string(REGEX REPLACE "/include/ncnn$" "/include" _inc_parent "${_inc}")
                    list(APPEND _ncnn_fixed_includes "${_inc_parent}")
                endif()
            endforeach()
            list(REMOVE_DUPLICATES _ncnn_fixed_includes)
            set(NCNN_INCLUDE_DIRS ${_ncnn_fixed_includes})
        endif()
    else()
        find_path(NCNN_INCLUDE_DIR
            NAMES ncnn/net.h
            PATHS "${_ncnn_install_root}"
            PATH_SUFFIXES include)
        find_library(NCNN_LIBRARY
            NAMES ncnn
            PATHS "${_ncnn_install_root}"
            PATH_SUFFIXES lib)

        if(NCNN_INCLUDE_DIR AND NCNN_LIBRARY)
            set(NCNN_FOUND TRUE)
            set(NCNN_INCLUDE_DIRS ${NCNN_INCLUDE_DIR})
            get_filename_component(NCNN_LIBRARY_DIR "${NCNN_LIBRARY}" DIRECTORY)
            set(NCNN_LIBRARY_DIRS ${NCNN_LIBRARY_DIR})
            set(NCNN_LIBRARIES ${NCNN_LIBRARY})
        endif()
    endif()

if(NCNN_FOUND)
    include_directories(${NCNN_INCLUDE_DIRS})
    link_directories(${NCNN_LIBRARY_DIRS})
    message(STATUS "NCNN_INCLUDE_DIRS: ${NCNN_INCLUDE_DIRS}")
    message(STATUS "NCNN_LIBRARIES: ${NCNN_LIBRARIES}")
else()
    message(FATAL_ERROR "NCNN not found under ${_local_ncnn_root}. Unified vision stack requires ncnn.")
endif()


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

target_link_libraries(${PROJECT_NAME} ${NCNN_LIBRARIES})
