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

// 迷宫法起始搜索行（0~H-1），按单行搜索左右起点。
void vision_image_processor_set_maze_start_row(int row);
int vision_image_processor_get_maze_start_row();

// 读取最近一帧处理耗时（单位：us）
// capture_wait_us: 等待相机新帧
// preprocess_us  : 当前分辨率 Gray/RGB565 生成
// otsu_us        : 全图OTSU二值化耗时
// maze_us        : 下60%区域迷宫法双边线提取耗时
// total_us       : process_step总耗时
void vision_image_processor_get_last_perf_us(uint32 *capture_wait_us,
                                             uint32 *preprocess_us,
                                             uint32 *otsu_us,
                                             uint32 *maze_us,
                                             uint32 *total_us);
// 读取最近一帧红色矩形识别耗时（单位：us）
void vision_image_processor_get_last_red_detect_us(uint32 *red_detect_us);

// 读取最近一帧 maze 阶段子流程耗时（单位：us）与有效性统计
// maze_setup_us       : maze阶段初始化（参数准备）
// maze_start_us       : 左右起点搜索（find_maze_start_from_bottom）
// maze_trace_left_us  : 左边线巡线耗时
// maze_trace_right_us : 右边线巡线耗时
// maze_post_us        : 后处理（边线填充/逆透视与中线生成）
// left_points/right_points : 左右边线输出点数
// left_ok/right_ok         : 左右起点是否找到
void vision_image_processor_get_last_maze_detail_us(uint32 *maze_setup_us,
                                                    uint32 *maze_start_us,
                                                    uint32 *maze_trace_left_us,
                                                    uint32 *maze_trace_right_us,
                                                    uint32 *maze_post_us,
                                                    uint16 *left_points,
                                                    uint16 *right_points,
                                                    bool *left_ok,
                                                    bool *right_ok);

// 图像数据访问接口
// 当前阶段处理图像统一为 VISION_DOWNSAMPLED_WIDTH x VISION_DOWNSAMPLED_HEIGHT。
const uint8 *vision_image_processor_gray_image();
const uint8 *vision_image_processor_binary_u8_image();
const uint8 *vision_image_processor_bgr_image();
const uint8 *vision_image_processor_bgr_full_image();
const uint8 *vision_image_processor_rgb565_image();
// 降采样接口与主接口一致（当前输入即处理分辨率）
const uint8 *vision_image_processor_gray_downsampled_image();
const uint8 *vision_image_processor_binary_downsampled_u8_image();
const uint8 *vision_image_processor_bgr_downsampled_image();
const uint8 *vision_image_processor_rgb565_downsampled_image();
// 调试图像：逆透视彩色图与逆透视边线图（黑底白线）
const uint8 *vision_image_processor_ipm_bgr_downsampled_image();
const uint8 *vision_image_processor_ipm_edge_gray_downsampled_image();

// 边界数据访问接口
// 当前返回原图(无逆透视)坐标系下的边界数组:
// x1/y1 左边线, x2/y2 左右均值中线, x3/y3 右边线。
void vision_image_processor_get_boundaries(uint16 **x1, uint16 **x2, uint16 **x3,
                                           uint16 **y1, uint16 **y2, uint16 **y3,
                                           uint16 *dot_num);

// 逆透视后边界数据（另存，供控制等后续模块使用）
void vision_image_processor_get_ipm_boundaries(uint16 **x1, uint16 **x2, uint16 **x3,
                                               uint16 **y1, uint16 **y2, uint16 **y3,
                                               uint16 *dot_num);

// 逆透视后拟合中线（另存）
void vision_image_processor_get_ipm_fitted_centerline(uint16 **x, uint16 **y, uint16 *dot_num);

// 红色实心矩形检测结果（坐标与尺寸）
void vision_image_processor_get_red_rect(bool *found, int *x, int *y, int *w, int *h, int *cx, int *cy);
int vision_image_processor_get_red_rect_area();
void vision_image_processor_set_red_rect(bool found, int x, int y, int w, int h, int cx, int cy, int area);
void vision_image_processor_set_last_red_detect_us(uint32 red_detect_us);

// ncnn输入ROI（用于发送图像叠加调试框）
void vision_image_processor_set_ncnn_roi(bool valid, int x, int y, int w, int h);
void vision_image_processor_get_ncnn_roi(bool *valid, int *x, int *y, int *w, int *h);

#endif
