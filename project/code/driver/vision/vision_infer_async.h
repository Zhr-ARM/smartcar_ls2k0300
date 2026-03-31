#ifndef VISION_INFER_ASYNC_H_
#define VISION_INFER_ASYNC_H_

#include "zf_common_headfile.h"

#include <opencv2/core.hpp>
#include <ncnn/net.h>

#include <string>
#include <vector>

class LQ_NCNN
{
public:
    LQ_NCNN();
    bool Init();
    std::string Infer(const cv::Mat &bgr_image);
    void SetModelPath(const std::string &param_path, const std::string &bin_path);
    void SetInputSize(int width, int height);
    void SetLabels(const std::vector<std::string> &labels);
    void SetNormalize(const float mean_vals[3], const float norm_vals[3]);
    ~LQ_NCNN();

private:
    int Argmax(const ncnn::Mat &logits);

private:
    ncnn::Net m_net;
    std::vector<std::string> m_labels;
    bool m_initialized;
    std::string m_param_path;
    std::string m_bin_path;
    int m_input_width;
    int m_input_height;
    float m_mean_vals[3];
    float m_norm_vals[3];
    std::string m_input_name;
    std::string m_output_name;
};

// 使用默认参数初始化模型。
bool vision_infer_init_default_model(LQ_NCNN &ncnn);

typedef struct
{
    bool found;
    int red_x;
    int red_y;
    int red_w;
    int red_h;
    int red_cx;
    int red_cy;
    int red_area;
    uint32 red_detect_us;
    bool ncnn_roi_valid;
    int ncnn_roi_x;
    int ncnn_roi_y;
    int ncnn_roi_w;
    int ncnn_roi_h;
} vision_infer_async_result_t;

bool vision_infer_async_init(LQ_NCNN *ncnn, bool enabled);
void vision_infer_async_cleanup();

void vision_infer_async_set_enabled(bool enabled);
bool vision_infer_async_enabled();

void vision_infer_async_set_roi_capture_mode(bool enabled);
bool vision_infer_async_roi_capture_mode_enabled();

void vision_infer_async_submit_frame(const uint8 *bgr_proc_data,
                                     int proc_width,
                                     int proc_height,
                                     const uint8 *bgr_full_data,
                                     int full_width,
                                     int full_height);

bool vision_infer_async_fetch_latest(vision_infer_async_result_t *out);

#endif
