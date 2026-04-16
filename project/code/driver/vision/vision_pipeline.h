#ifndef VISION_PIPELINE_H_
#define VISION_PIPELINE_H_

#include "zf_common_headfile.h"
#include "driver/vision/vision_infer_async.h"
#include "driver/vision/vision_transport.h"

// 作用：初始化视觉主链（采图/处理/推理/发送）。
// 意义：对上层线程暴露统一入口，隐藏内部模块拆分细节。
// 如何修改：
// - camera_path: 相机设备路径（默认常用 /dev/video0）；
// - ncnn/ncnn_enabled: 推理对象与是否启用推理。
// 是否调用：是，vision_thread_init 调用。
bool vision_pipeline_init(const char *camera_path, LQ_NCNN *ncnn, bool ncnn_enabled);

// 作用：清理视觉主链资源。
// 意义：确保线程退出时不遗留推理线程与采图资源。
// 是否调用：是，vision_thread_cleanup 调用。
void vision_pipeline_cleanup();

// 作用：执行一帧“采图+处理+检测+推理任务提交/取结果”。
// 意义：处理与发送解耦，便于独立测性能。
// 是否调用：是，vision_thread 每帧调用。
bool vision_pipeline_process_step();

// 作用：执行一帧发送（助手/UDP/TCP）。
// 是否调用：是，vision_thread 每帧调用。
void vision_pipeline_send_step();

// 作用：兼容旧接口（process+send 一起执行）。
// 是否调用：当前主链通常不直接用，但保留兼容性。
bool vision_pipeline_step();

// 作用：获取当前处理后的 BGR 图像指针（160x120）。
// 意义：供调试显示/发送等模块直接复用。
const uint8 *vision_pipeline_bgr_image();

// 作用：发送配置透传给 transport。
// 如何修改：在 main.cpp 通过 vision_thread 接口设置。
void vision_pipeline_set_send_mode(vision_send_mode_enum mode);
vision_send_mode_enum vision_pipeline_get_send_mode();
void vision_pipeline_set_send_max_fps(uint32 max_fps);
uint32 vision_pipeline_get_send_max_fps();
void vision_pipeline_set_send_enabled(bool enabled);
bool vision_pipeline_is_send_enabled();

// 作用：推理总开关（关闭后跳过红色检测与 ncnn 推理）。
// 如何修改：运行时可动态切换。
// 是否调用：是，main.cpp -> vision_thread -> pipeline 调用。
void vision_pipeline_set_infer_enabled(bool enabled);
bool vision_pipeline_infer_enabled();
void vision_pipeline_set_ncnn_enabled(bool enabled);
bool vision_pipeline_ncnn_enabled();

// 作用：读取红色矩形结果（来自 image_processor 全局状态）。
// 意义：供外部状态上报/UI显示使用。
void vision_pipeline_get_red_rect(bool *found, int *x, int *y, int *w, int *h, int *cx, int *cy);
int vision_pipeline_get_red_rect_area();

#endif
