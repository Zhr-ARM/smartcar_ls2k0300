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
set(COMPAT_DIR "${CMAKE_CURRENT_LIST_DIR}/compat")

find_package(OpenCV QUIET COMPONENTS core imgproc)

add_library(vision_web_bridge_stub STATIC
    web_vision_bridge.cpp
    web_vision_abi.cpp
)

target_include_directories(vision_web_bridge_stub
    PUBLIC
        "${COMPAT_DIR}"
        "${CMAKE_CURRENT_LIST_DIR}"
        "${PROJECT_CODE_DIR}"
)

target_compile_definitions(vision_web_bridge_stub
    PUBLIC
        VISION_WEB_BRIDGE_STUB=1
)

message(STATUS "vision_web_bridge_stub is a scaffold only.")
message(STATUS "It is intentionally placed under tools/ so project/ algorithms remain untouched.")

add_library(vision_web_bridge_probe STATIC
    web_vision_bridge.cpp
    web_vision_abi.cpp
    compat/line_follow_thread.cpp
    "${PROJECT_CODE_DIR}/driver/vision/vision_config.c"
    "${PROJECT_CODE_DIR}/driver/vision/vision_line_error_layer.cpp"
)

target_include_directories(vision_web_bridge_probe
    PUBLIC
        "${COMPAT_DIR}"
        "${CMAKE_CURRENT_LIST_DIR}"
        "${PROJECT_CODE_DIR}"
)

target_compile_definitions(vision_web_bridge_probe
    PUBLIC
        VISION_WEB_BRIDGE_PROBE=1
)

message(STATUS "vision_web_bridge_probe currently validates the first same-source bridge slice:")
message(STATUS "vision_config.c + vision_line_error_layer.cpp + compat shims")

if(OpenCV_FOUND)
    add_library(vision_web_bridge_image_probe STATIC
        web_vision_bridge.cpp
        web_vision_abi.cpp
        compat/line_follow_thread.cpp
        compat/vision_frame_capture.cpp
        "${PROJECT_CODE_DIR}/driver/vision/vision_config.c"
        "${PROJECT_CODE_DIR}/driver/vision/vision_line_error_layer.cpp"
        "${PROJECT_CODE_DIR}/driver/vision/vision_image_processor.cpp"
    )

    target_include_directories(vision_web_bridge_image_probe
        PUBLIC
            "${COMPAT_DIR}"
            "${CMAKE_CURRENT_LIST_DIR}"
            "${PROJECT_CODE_DIR}"
            ${OpenCV_INCLUDE_DIRS}
    )

    target_link_libraries(vision_web_bridge_image_probe
        PUBLIC
            ${OpenCV_LIBS}
    )

    target_compile_definitions(vision_web_bridge_image_probe
        PUBLIC
            VISION_WEB_BRIDGE_IMAGE_PROBE=1
    )

    message(STATUS "vision_web_bridge_image_probe enabled with OpenCV core/imgproc.")

    add_executable(vision_web_bridge_smoke
        smoke_test.cpp
    )

    target_include_directories(vision_web_bridge_smoke
        PUBLIC
            "${COMPAT_DIR}"
            "${CMAKE_CURRENT_LIST_DIR}"
            "${PROJECT_CODE_DIR}"
            ${OpenCV_INCLUDE_DIRS}
    )

    target_link_libraries(vision_web_bridge_smoke
        PRIVATE
            vision_web_bridge_image_probe
    )

    if(EMSCRIPTEN)
        add_executable(vision_pipeline
            web_vision_bridge.cpp
            web_vision_abi.cpp
            compat/line_follow_thread.cpp
            compat/vision_frame_capture.cpp
            "${PROJECT_CODE_DIR}/driver/vision/vision_config.c"
            "${PROJECT_CODE_DIR}/driver/vision/vision_line_error_layer.cpp"
            "${PROJECT_CODE_DIR}/driver/vision/vision_image_processor.cpp"
        )

        target_include_directories(vision_pipeline
            PUBLIC
                "${COMPAT_DIR}"
                "${CMAKE_CURRENT_LIST_DIR}"
                "${PROJECT_CODE_DIR}"
                ${OpenCV_INCLUDE_DIRS}
        )

        target_link_libraries(vision_pipeline
            PRIVATE
                ${OpenCV_LIBS}
        )

        set_target_properties(vision_pipeline PROPERTIES OUTPUT_NAME "vision_pipeline")
        target_link_options(vision_pipeline PRIVATE
            "-sWASM=1"
            "-sALLOW_MEMORY_GROWTH=1"
            "-sMODULARIZE=1"
            "-sEXPORT_NAME=createVisionPipelineModule"
            "-sENVIRONMENT=web,worker"
            "-sEXPORTED_FUNCTIONS=['_malloc','_free','_web_vision_abi_process_bgr','_web_vision_abi_last_result_json','_web_vision_abi_version']"
            "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','UTF8ToString','HEAPU8']"
        )
    endif()
else()
    message(WARNING "OpenCV core/imgproc not found. vision_web_bridge_image_probe is skipped for now.")
endif()
