#ifndef VISION_IMAGE_PROCESSOR_H_
#define VISION_IMAGE_PROCESSOR_H_

#include "zf_common_headfile.h"

#define VISION_DOWNSAMPLED_WIDTH 160
#define VISION_DOWNSAMPLED_HEIGHT 120

// 轮廓线误差：在图像高度(line_sample_ratio_num/line_sample_ratio_den)处，中线x与图像中心x的差值
extern int line_error;
extern int line_sample_ratio_num;
extern int line_sample_ratio_den;

// 初始化图像处理模块（内部初始化UVC摄像头）
bool vision_image_processor_init(const char *camera_path);
// 释放图像处理模块资源（停止采集线程）
void vision_image_processor_cleanup();

// 执行一帧图像采集 + OpenCV处理（灰度、二值化、边界提取）
bool vision_image_processor_process_step();

// 读取最近一帧处理耗时（单位：us）
// capture_wait_us: 等待相机新帧
// preprocess_us  : 160x120 Gray/RGB565 生成
// otsu_us        : 全图OTSU二值化耗时
// maze_us        : 下60%区域迷宫法双边线提取耗时
// total_us       : process_step总耗时
void vision_image_processor_get_last_perf_us(uint32 *capture_wait_us,
                                             uint32 *preprocess_us,
                                             uint32 *otsu_us,
                                             uint32 *maze_us,
                                             uint32 *total_us);

// 图像数据访问接口
// 当前阶段图像统一为160x120（无翻转）。
const uint8 *vision_image_processor_gray_image();
const uint8 *vision_image_processor_binary_u8_image();
const uint8 *vision_image_processor_bgr_image();
const uint8 *vision_image_processor_rgb565_image();
// 降采样接口与主接口一致（当前输入即160x120）
const uint8 *vision_image_processor_gray_downsampled_image();
const uint8 *vision_image_processor_binary_downsampled_u8_image();
const uint8 *vision_image_processor_bgr_downsampled_image();
const uint8 *vision_image_processor_rgb565_downsampled_image();
// 调试图像：逆透视彩色图与逆透视边线图（黑底白线）
const uint8 *vision_image_processor_ipm_bgr_downsampled_image();
const uint8 *vision_image_processor_ipm_edge_gray_downsampled_image();

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
