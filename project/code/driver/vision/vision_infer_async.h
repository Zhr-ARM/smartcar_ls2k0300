#ifndef VISION_INFER_ASYNC_H_
#define VISION_INFER_ASYNC_H_

#include "zf_common_headfile.h"

#include <opencv2/core.hpp>
#include <ncnn/net.h>

#include <string>
#include <vector>

static constexpr int VISION_NCNN_MAX_CLASSES = 32;
static constexpr int VISION_NCNN_LABEL_MAX_LEN = 32;

class LQ_NCNN
{
public:
    // 作用：构造推理对象，填充默认输入参数。
    // 是否调用：是，main.cpp 栈上创建对象。
    LQ_NCNN();

    // 作用：加载 NCNN 模型。
    // 意义：只有 Init 成功后 Infer 才可调用。
    // 如何修改：模型路径、输入尺寸、归一化在 Init 前通过 Set* 配置。
    bool Init();

    // 作用：单次推理。
    // 参数：bgr_image 为 BGR 三通道图像。
    // 返回：Top1 标签字符串。
    // 是否调用：是，由 infer worker 在 ROI 上调用。
    std::string Infer(const cv::Mat &bgr_image);
    bool InferWithProbs(const cv::Mat &bgr_image,
                       int *top_class_id,
                       float *top_score,
                       std::string *top_label,
                       std::vector<std::string> *labels,
                       std::vector<float> *probs,
                       uint32 *infer_us);

    // 作用：设置模型 param/bin 路径。
    // 如何修改：切换模型文件时改这里。
    void SetModelPath(const std::string &param_path, const std::string &bin_path);

    // 作用：设置模型输入尺寸。
    // 如何修改：必须与模型训练输入一致。
    void SetInputSize(int width, int height);

    // 作用：设置类别标签表（索引->文本）。
    void SetLabels(const std::vector<std::string> &labels);

    // 作用：设置均值/方差归一化参数。
    // 如何修改：需和训练预处理保持一致。
    void SetNormalize(const float mean_vals[3], const float norm_vals[3]);

    // 作用：析构推理对象。
    ~LQ_NCNN();

private:
    // 作用：输出向量取最大值索引（Top1）。
    int Argmax(const ncnn::Mat &logits);

private:
    ncnn::Net m_net;
    std::vector<std::string> m_labels;
    bool m_initialized;         // 是否已 Init 成功。
    std::string m_param_path;   // .param 路径。
    std::string m_bin_path;     // .bin 路径。
    int m_input_width;          // 模型输入宽。
    int m_input_height;         // 模型输入高。
    float m_mean_vals[3];       // 归一化均值。
    float m_norm_vals[3];       // 归一化缩放。
    std::string m_input_name;   // 输入 blob 名。
    std::string m_output_name;  // 输出 blob 名。
};

// 作用：按项目默认模型参数初始化 ncnn 对象。
// 如何修改：若替换模型，优先改此函数里的默认参数。
// 是否调用：是，main.cpp 启动阶段调用。
bool vision_infer_init_default_model(LQ_NCNN &ncnn);

typedef struct
{
    uint32 result_seq;     // 异步结果序号；每产生一帧新结果递增。
    bool found;            // 是否找到红色矩形。
    int red_x;             // 红框左上角 x（处理分辨率坐标）。
    int red_y;             // 红框左上角 y。
    int red_w;             // 红框宽。
    int red_h;             // 红框高。
    int red_cx;            // 红框中心 x。
    int red_cy;            // 红框中心 y。
    int red_area;          // 红框面积。
    uint32 red_detect_us;  // 红框检测耗时（us）。
    bool ncnn_roi_valid;   // ncnn ROI 是否有效。
    int ncnn_roi_x;        // ncnn ROI 左上角 x（处理分辨率坐标）。
    int ncnn_roi_y;        // ncnn ROI 左上角 y。
    int ncnn_roi_w;        // ncnn ROI 宽。
    int ncnn_roi_h;        // ncnn ROI 高。
    bool ncnn_enabled;     // 当前是否启用 ncnn 推理。
    bool ncnn_infer_valid; // 当前帧是否有有效 ncnn 结果。
    uint32 ncnn_infer_us;  // ncnn 推理耗时（us）。
    int ncnn_top_class_id; // Top1 类别 id。
    float ncnn_top_score;  // Top1 概率。
    char ncnn_top_label[VISION_NCNN_LABEL_MAX_LEN]; // Top1 类别标签。
    int ncnn_class_count;  // 当前返回的类别数。
    char ncnn_labels[VISION_NCNN_MAX_CLASSES][VISION_NCNN_LABEL_MAX_LEN]; // 各类别标签。
    float ncnn_probs[VISION_NCNN_MAX_CLASSES]; // 各类别概率。
} vision_infer_async_result_t;

// 作用：初始化异步推理模块并启动 worker 线程。
// 参数：
// - ncnn: 推理对象（可为空，空时跳过 ncnn 调用）；
// - enabled: 是否默认开启推理。
// 是否调用：是，vision_pipeline_init 调用。
bool vision_infer_async_init(LQ_NCNN *ncnn, bool enabled);

// 作用：停止异步推理线程并清理共享状态。
void vision_infer_async_cleanup();

// 作用：推理开关（运行时可改）。
// 如何修改：false 时仅巡线不推理。
void vision_infer_async_set_enabled(bool enabled);
bool vision_infer_async_enabled();
void vision_infer_async_set_ncnn_enabled(bool enabled);
bool vision_infer_async_ncnn_enabled();

// 作用：ROI 抓图开关（检测到目标时按节流保存样本图）。
void vision_infer_async_set_roi_capture_mode(bool enabled);
bool vision_infer_async_roi_capture_mode_enabled();

// 作用：提交一帧给异步推理线程。
// 参数：proc/full 图像指针与尺寸；尺寸必须匹配当前固定分辨率。
// 是否调用：是，vision_pipeline_process_step 每帧调用。
void vision_infer_async_submit_frame(const uint8 *bgr_proc_data,
                                     int proc_width,
                                     int proc_height,
                                     const uint8 *bgr_full_data,
                                     int full_width,
                                     int full_height);

// 作用：获取最近一次异步推理结果。
// 返回：true=有可用结果，false=尚无结果。
// 是否调用：是，vision_pipeline_process_step 每帧尝试调用。
bool vision_infer_async_fetch_latest(vision_infer_async_result_t *out);

#endif
