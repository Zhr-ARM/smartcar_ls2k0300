#include "driver/vision/vision_ncnn.h"

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

// 外部注入的模型对象，生命周期由调用方管理
static LQ_NCNN *g_ncnn = nullptr;
static bool g_infer_enabled = false;

bool vision_ncnn_init_default_model(LQ_NCNN &ncnn)
{
    const std::string model_param = "tiny_classifier_fp32.ncnn.param";
    const std::string model_bin   = "tiny_classifier_fp32.ncnn.bin";
    const int input_width  = 25;
    const int input_height = 25;

    const std::vector<std::string> labels = {
        "aid", "amb", "arm", "axe", "baton", "bulletproof", "explosive",
        "fire", "flashlight", "gun", "helmet", "knife", "motor", "tele", "walkie"
    };

    float mean_vals[3] = {123.675f, 116.28f, 103.53f};
    float norm_vals[3] = {0.01712475f, 0.017507f, 0.01742919f};

    ncnn.SetModelPath(model_param, model_bin);
    ncnn.SetInputSize(input_width, input_height);
    ncnn.SetLabels(labels);
    ncnn.SetNormalize(mean_vals, norm_vals);

    printf("[NCNN] loading model...\n");
    if (!ncnn.Init())
    {
        printf("[NCNN] model init failed\n");
        return false;
    }
    printf("[NCNN] model init ok\n");
    return true;
}

void vision_ncnn_bind(LQ_NCNN *ncnn, bool enabled)
{
    g_ncnn = ncnn;
    g_infer_enabled = enabled;
}

void vision_ncnn_set_enabled(bool enabled)
{
    g_infer_enabled = enabled;
}

bool vision_ncnn_is_enabled()
{
    return g_infer_enabled;
}

void vision_ncnn_step(const uint8 *bgr_data, int width, int height)
{
    if (!g_infer_enabled || g_ncnn == nullptr)
    {
        return;
    }
    if (bgr_data == nullptr || width <= 0 || height <= 0)
    {
        return;
    }

    cv::Mat bgr(height, width, CV_8UC3, const_cast<uint8 *>(bgr_data));
    g_ncnn->Infer(bgr);
}
