cmake_minimum_required(VERSION 3.16)

project(vision_web_bridge_stub LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# 这个目录的职责不是重写算法，而是：
# 1) 只读引用 project/ 里的现有算法源码
# 2) 在 tools/ 下提供适配层和兼容桩
# 3) 后续由 Emscripten 或其他 WASM 工具链编译

set(ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/../../..")
set(PROJECT_CODE_DIR "${ROOT_DIR}/project/code")

add_library(vision_web_bridge_stub STATIC
    web_vision_bridge.cpp
)

target_include_directories(vision_web_bridge_stub
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}"
        "${PROJECT_CODE_DIR}"
)

target_compile_definitions(vision_web_bridge_stub
    PUBLIC
        VISION_WEB_BRIDGE_STUB=1
)

message(STATUS "vision_web_bridge_stub is a scaffold only.")
message(STATUS "It is intentionally placed under tools/ so project/ algorithms remain untouched.")
