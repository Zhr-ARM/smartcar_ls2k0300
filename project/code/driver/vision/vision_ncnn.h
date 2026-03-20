#ifndef VISION_NCNN_H_
#define VISION_NCNN_H_

#include "driver/vision/lq_ncnn.hpp"
#include "zf_common_headfile.h"

// 使用默认参数初始化模型
bool vision_ncnn_init_default_model(LQ_NCNN &ncnn);

// 绑定推理运行时上下文（可开关）
void vision_ncnn_bind(LQ_NCNN *ncnn, bool enabled);

// 推理开关
void vision_ncnn_set_enabled(bool enabled);
bool vision_ncnn_is_enabled();

// 对当前帧执行一次推理
void vision_ncnn_step(const uint8 *bgr_data, int width, int height);

#endif
