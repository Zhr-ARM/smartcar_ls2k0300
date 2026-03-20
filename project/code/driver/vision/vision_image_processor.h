#ifndef VISION_IMAGE_PROCESSOR_H_
#define VISION_IMAGE_PROCESSOR_H_

#include "zf_common_headfile.h"

// 轮廓线误差：在图像高度(line_sample_ratio_num/line_sample_ratio_den)处，中线x与图像中心x的差值
extern int line_error;
extern int line_sample_ratio_num;
extern int line_sample_ratio_den;

// 初始化图像处理模块（内部初始化UVC摄像头）
bool vision_image_processor_init(const char *camera_path);

// 执行一帧图像采集 + OpenCV处理（灰度、二值化、边界提取）
bool vision_image_processor_process_step();

// 图像数据访问接口
const uint8 *vision_image_processor_gray_image();
const uint8 *vision_image_processor_binary_u8_image();
const uint8 *vision_image_processor_bgr_image();
const uint8 *vision_image_processor_rgb565_image();

// 边界数据访问接口
void vision_image_processor_get_boundaries(uint8 **x1, uint8 **x2, uint8 **x3,
                                           uint8 **y1, uint8 **y2, uint8 **y3,
                                           uint16 *dot_num);

// 红色实心矩形检测结果（坐标与尺寸）
void vision_image_processor_get_red_rect(bool *found, int *x, int *y, int *w, int *h, int *cx, int *cy);
int vision_image_processor_get_red_rect_area();

// ncnn输入ROI（用于发送图像叠加调试框）
void vision_image_processor_set_ncnn_roi(bool valid, int x, int y, int w, int h);
void vision_image_processor_get_ncnn_roi(bool *valid, int *x, int *y, int *w, int *h);

#endif
