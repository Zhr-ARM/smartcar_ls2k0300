#include "driver/vision/vision_infer_async.h"

#include "driver/vision/vision_config.h"
#include "driver/vision/vision_image_processor.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <limits.h>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unistd.h>

namespace
{
// 固定分辨率参数：主链处理图为 160x120，红框/ROI/ncnn 使用 320x240 full 图。
static constexpr int kProcWidth = VISION_DOWNSAMPLED_WIDTH;
static constexpr int kProcHeight = VISION_DOWNSAMPLED_HEIGHT;
static constexpr int kFullWidth = UVC_WIDTH;
static constexpr int kFullHeight = UVC_HEIGHT;

struct infer_job_t
{
    // 异步任务输入：处理分辨率 BGR + full 分辨率 BGR。
    cv::Mat proc_bgr;
    cv::Mat full_bgr;
};

struct infer_worker_result_t
{
    // 异步任务输出：红框信息 + ncnn ROI（full 分辨率坐标）。
    bool found = false;
    int red_x = 0;
    int red_y = 0;
    int red_w = 0;
    int red_h = 0;
    int red_cx = 0;
    int red_cy = 0;
    int red_area = 0;
    uint32 red_detect_us = 0;
    cv::Rect ncnn_roi_full;
    bool ncnn_enabled = false;
    bool ncnn_infer_valid = false;
    uint32 ncnn_infer_us = 0;
    int ncnn_top_class_id = -1;
    float ncnn_top_score = 0.0f;
    std::string ncnn_top_label;
    std::vector<std::string> ncnn_labels;
    std::vector<float> ncnn_probs;
};

// 推理运行态（原 vision_ncnn 逻辑并入）。
static LQ_NCNN *g_ncnn = nullptr;
static std::atomic<bool> g_infer_enabled(false);
static std::atomic<bool> g_ncnn_enabled(false);

// 异步 worker 线程与共享任务/结果缓冲。
static std::mutex g_infer_mutex;
static std::condition_variable g_infer_cv;
static std::thread g_infer_thread;
static bool g_infer_worker_running = false;
static bool g_infer_worker_stop = false;
static bool g_infer_job_ready = false;
static infer_job_t g_infer_job;
static infer_worker_result_t g_latest_infer_result;
static bool g_latest_infer_result_valid = false;
static uint32 g_latest_infer_result_seq = 0;

static cv::Rect build_ncnn_roi_from_red_rect(int red_x,
                                             int red_y,
                                             int red_w,
                                             int red_h,
                                             int image_width,
                                             int image_height)
{
    const float ratio_w = std::max(0.01f, g_vision_runtime_config.red_roi_ratio_w);
    const float ratio_h = std::max(0.01f, g_vision_runtime_config.red_roi_ratio_h);
    const float offset_ratio = std::max(0.0f, g_vision_runtime_config.red_roi_offset_ratio);
    const int bw = std::max(1, red_w);
    const int lift = static_cast<int>(std::lround(static_cast<float>(bw) * offset_ratio));
    const int anchor_y = red_y + red_h - lift;
    const int roi_w = std::max(3, static_cast<int>(std::lround(static_cast<float>(bw) * ratio_w)));
    const int roi_h = std::max(3, static_cast<int>(std::lround(static_cast<float>(bw) * ratio_h)));
    int roi_x = static_cast<int>(std::lround(static_cast<float>(red_x) + (static_cast<float>(bw - roi_w) * 0.5f)));
    int roi_y = anchor_y - roi_h;

    roi_x = std::clamp(roi_x, 0, std::max(0, image_width - roi_w));
    roi_y = std::clamp(roi_y, 0, std::max(0, image_height - roi_h));
    return cv::Rect(roi_x, roi_y, roi_w, roi_h) & cv::Rect(0, 0, image_width, image_height);
}

// 作用：按 data_gen fine_crop_debug.py 的 HSV 双红区间算法找最大红色轮廓。
static bool detect_red_rectangle_bbox(const cv::Mat &bgr, cv::Rect *bbox, int *area_px)
{
    if (bbox == nullptr || area_px == nullptr || bgr.empty() || bgr.type() != CV_8UC3)
    {
        return false;
    }

    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    const int h_span = std::clamp(g_vision_runtime_config.red_roi_h_span, 0, 90);
    const int s_min = std::clamp(g_vision_runtime_config.red_roi_s_min, 0, 255);
    const int v_min = std::clamp(g_vision_runtime_config.red_roi_v_min, 0, 255);

    cv::Mat mask1;
    cv::Mat mask2;
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(0, s_min, v_min), cv::Scalar(h_span, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(180 - h_span, s_min, v_min), cv::Scalar(180, 255, 255), mask2);
    cv::bitwise_or(mask1, mask2, mask);

    const cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
    const int close_iter = std::max(0, g_vision_runtime_config.red_roi_close_iter);
    const int open_iter = std::max(0, g_vision_runtime_config.red_roi_open_iter);
    if (close_iter > 0)
    {
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), close_iter);
    }
    if (open_iter > 0)
    {
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), open_iter);
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty())
    {
        return false;
    }

    int best_index = -1;
    double best_area = 0.0;
    for (int i = 0; i < static_cast<int>(contours.size()); ++i)
    {
        const double area = cv::contourArea(contours[i]);
        if (area > best_area)
        {
            best_area = area;
            best_index = i;
        }
    }

    if (best_index < 0 || best_area <= static_cast<double>(std::max(1, g_vision_runtime_config.red_roi_area_min)))
    {
        return false;
    }

    *bbox = cv::boundingRect(contours[best_index]) & cv::Rect(0, 0, bgr.cols, bgr.rows);
    *area_px = static_cast<int>(std::lround(best_area));
    return true;
}

// 作用：执行一帧 ncnn 推理（输出 top1 与各类别概率）。
static bool ncnn_step(const uint8 *bgr_data,
                      int width,
                      int height,
                      int *top_class_id,
                      float *top_score,
                      std::string *top_label,
                      std::vector<std::string> *labels,
                      std::vector<float> *probs,
                      uint32 *infer_us)
{
    if (!g_ncnn_enabled.load() || g_ncnn == nullptr || bgr_data == nullptr || width <= 0 || height <= 0)
    {
        return false;
    }

    cv::Mat bgr(height, width, CV_8UC3, const_cast<uint8 *>(bgr_data));
    return g_ncnn->InferWithProbs(bgr, top_class_id, top_score, top_label, labels, probs, infer_us);
}

static bool file_exists(const std::string &path)
{
    return access(path.c_str(), R_OK) == 0;
}

static std::string executable_dir()
{
    char exe_path[PATH_MAX] = {0};
    const ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (len <= 0)
    {
        return "";
    }
    exe_path[len] = '\0';
    const char *slash = std::strrchr(exe_path, '/');
    if (slash == nullptr)
    {
        return "";
    }
    return std::string(exe_path, static_cast<size_t>(slash - exe_path));
}

static std::string join_path(const std::string &dir, const char *name)
{
    if (dir.empty())
    {
        return name;
    }
    return dir + "/" + name;
}

static void resolve_default_model_paths(std::string *param_path, std::string *bin_path)
{
    const char *param_name = "tiny_classifier_fp32.ncnn.param";
    const char *bin_name = "tiny_classifier_fp32.ncnn.bin";
    const std::string exe_dir = executable_dir();
    if (!exe_dir.empty())
    {
        const std::string model_dir = exe_dir + "/ncnn_model";
        const std::string exe_param = join_path(model_dir, param_name);
        const std::string exe_bin = join_path(model_dir, bin_name);
        if (file_exists(exe_param) && file_exists(exe_bin))
        {
            *param_path = exe_param;
            *bin_path = exe_bin;
            return;
        }
    }

    const std::string cwd_param = join_path("ncnn_model", param_name);
    const std::string cwd_bin = join_path("ncnn_model", bin_name);
    if (file_exists(cwd_param) && file_exists(cwd_bin))
    {
        *param_path = cwd_param;
        *bin_path = cwd_bin;
        return;
    }

    *param_path = param_name;
    *bin_path = bin_name;
}

// 作用：清空异步任务与结果共享状态。
static void reset_infer_shared_state()
{
    std::lock_guard<std::mutex> lock(g_infer_mutex);
    g_infer_job.proc_bgr.release();
    g_infer_job.full_bgr.release();
    g_infer_job_ready = false;
    g_latest_infer_result = infer_worker_result_t{};
    g_latest_infer_result_valid = false;
    g_latest_infer_result_seq = 0;
}

// 作用：异步推理工作线程主循环。
static void run_infer_worker()
{
    for (;;)
    {
        infer_job_t job;
        {
            std::unique_lock<std::mutex> lock(g_infer_mutex);
            g_infer_cv.wait(lock, []() { return g_infer_worker_stop || g_infer_job_ready; });
            if (g_infer_worker_stop)
            {
                break;
            }
            job.proc_bgr = g_infer_job.proc_bgr;
            job.full_bgr = g_infer_job.full_bgr;
            g_infer_job_ready = false;
        }

        infer_worker_result_t result{};
        result.ncnn_enabled = g_ncnn_enabled.load();
        auto detect_start = std::chrono::steady_clock::now();
        cv::Rect red_bbox;
        int red_area = 0;
        const cv::Mat &detect_bgr = !job.full_bgr.empty() ? job.full_bgr : job.proc_bgr;
        const int detect_width = detect_bgr.cols;
        const int detect_height = detect_bgr.rows;
        const bool found = detect_red_rectangle_bbox(detect_bgr, &red_bbox, &red_area);
        auto detect_end = std::chrono::steady_clock::now();
        result.red_detect_us = static_cast<uint32>(
            std::chrono::duration_cast<std::chrono::microseconds>(detect_end - detect_start).count());

        if (found)
        {
            red_bbox &= cv::Rect(0, 0, detect_width, detect_height);
            result.found = true;
            result.red_x = red_bbox.x;
            result.red_y = red_bbox.y;
            result.red_w = red_bbox.width;
            result.red_h = red_bbox.height;
            result.red_cx = red_bbox.x + red_bbox.width / 2;
            result.red_cy = red_bbox.y + red_bbox.height / 2;
            result.red_area = red_area;

            result.ncnn_roi_full = build_ncnn_roi_from_red_rect(result.red_x,
                                                                result.red_y,
                                                                result.red_w,
                                                                result.red_h,
                                                                detect_width,
                                                                detect_height);

            if (g_ncnn_enabled.load())
            {
                cv::Rect safe_roi = result.ncnn_roi_full & cv::Rect(0, 0, detect_width, detect_height);
                if (safe_roi.width > 0 && safe_roi.height > 0)
                {
                    cv::Mat roi_bgr;
                    roi_bgr = detect_bgr(safe_roi).clone();
                    result.ncnn_infer_valid = ncnn_step(reinterpret_cast<const uint8 *>(roi_bgr.data),
                                                        roi_bgr.cols,
                                                        roi_bgr.rows,
                                                        &result.ncnn_top_class_id,
                                                        &result.ncnn_top_score,
                                                        &result.ncnn_top_label,
                                                        &result.ncnn_labels,
                                                        &result.ncnn_probs,
                                                        &result.ncnn_infer_us);
                }
            }
        }

        {
            std::lock_guard<std::mutex> lock(g_infer_mutex);
            g_latest_infer_result = result;
            ++g_latest_infer_result_seq;
            g_latest_infer_result_valid = true;
        }
    }
}

} // namespace

LQ_NCNN::LQ_NCNN()
    : m_initialized(false)
    , m_input_width(64)
    , m_input_height(64)
    , m_input_name("in0")
    , m_output_name("out0")
{
    m_mean_vals[0] = 123.675f;
    m_mean_vals[1] = 116.28f;
    m_mean_vals[2] = 103.53f;
    m_norm_vals[0] = 1.0f / 58.395f;
    m_norm_vals[1] = 1.0f / 57.12f;
    m_norm_vals[2] = 1.0f / 57.375f;
}

bool LQ_NCNN::Init()
{
#ifdef VISION_ENABLE_NCNN
    m_net.opt.use_vulkan_compute = false;
    m_net.opt.num_threads = 1;
    if (m_param_path.empty() || m_net.load_param(m_param_path.c_str()) != 0)
    {
        printf("NCNN: load param failed: %s\n", m_param_path.c_str());
        return false;
    }
    if (m_bin_path.empty() || m_net.load_model(m_bin_path.c_str()) != 0)
    {
        printf("NCNN: load bin failed: %s\n", m_bin_path.c_str());
        return false;
    }
    m_initialized = true;
    return true;
#else
    m_initialized = false;
    return false;
#endif
}

std::string LQ_NCNN::Infer(const cv::Mat &bgr_image)
{
    int top_class_id = -1;
    float top_score = 0.0f;
    uint32 infer_us = 0;
    std::string top_label;
    std::vector<std::string> labels;
    std::vector<float> probs;
    if (!InferWithProbs(bgr_image, &top_class_id, &top_score, &top_label, &labels, &probs, &infer_us))
    {
        throw std::runtime_error("InferWithProbs failed");
    }
    return top_label;
}

bool LQ_NCNN::InferWithProbs(const cv::Mat &bgr_image,
                             int *top_class_id,
                             float *top_score,
                             std::string *top_label,
                             std::vector<std::string> *labels,
                             std::vector<float> *probs,
                             uint32 *infer_us)
{
#ifdef VISION_ENABLE_NCNN
    if (!m_initialized)
    {
        throw std::runtime_error("NCNN not initialized");
    }
    if (bgr_image.empty())
    {
        throw std::invalid_argument("Input image is empty");
    }

    const auto t0 = std::chrono::steady_clock::now();

    cv::Mat resized;
    cv::resize(bgr_image, resized, cv::Size(m_input_width, m_input_height));
    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

    ncnn::Mat input = ncnn::Mat::from_pixels(
        rgb.data,
        ncnn::Mat::PIXEL_RGB,
        m_input_width,
        m_input_height);
    input.substract_mean_normalize(m_mean_vals, m_norm_vals);

    ncnn::Extractor ex = m_net.create_extractor();
    int ret = ex.input(m_input_name.c_str(), input);
    if (ret != 0)
    {
        throw std::runtime_error("ex.input failed");
    }

    ncnn::Mat logits;
    ret = ex.extract(m_output_name.c_str(), logits);
    if (ret != 0 || logits.w <= 0)
    {
        throw std::runtime_error("ex.extract failed");
    }

    const int class_id = Argmax(logits);
    float max_logit = -std::numeric_limits<float>::infinity();
    for (int i = 0; i < logits.w; ++i)
    {
        max_logit = std::max(max_logit, logits[i]);
    }

    std::vector<float> local_probs(logits.w, 0.0f);
    float sum = 0.0f;
    for (int i = 0; i < logits.w; ++i)
    {
        const float v = std::exp(logits[i] - max_logit);
        local_probs[i] = v;
        sum += v;
    }
    if (sum > 0.0f)
    {
        for (float &v : local_probs)
        {
            v /= sum;
        }
    }

    std::vector<std::string> local_labels;
    local_labels.reserve(logits.w);
    for (int i = 0; i < logits.w; ++i)
    {
        if (i >= 0 && i < static_cast<int>(m_labels.size()))
        {
            local_labels.push_back(m_labels[i]);
        }
        else
        {
            local_labels.push_back(std::to_string(i));
        }
    }

    if (top_class_id)
    {
        *top_class_id = class_id;
    }
    if (top_score)
    {
        *top_score = (class_id >= 0 && class_id < static_cast<int>(local_probs.size())) ? local_probs[class_id] : 0.0f;
    }
    if (top_label)
    {
        if (class_id >= 0 && class_id < static_cast<int>(local_labels.size()))
        {
            *top_label = local_labels[class_id];
        }
        else
        {
            *top_label = std::to_string(class_id);
        }
    }
    if (labels)
    {
        *labels = local_labels;
    }
    if (probs)
    {
        *probs = local_probs;
    }
    if (infer_us)
    {
        *infer_us = static_cast<uint32>(
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count());
    }
    return true;
#else
    (void)bgr_image;
    if (top_class_id)
    {
        *top_class_id = -1;
    }
    if (top_score)
    {
        *top_score = 0.0f;
    }
    if (top_label)
    {
        top_label->clear();
    }
    if (labels)
    {
        labels->clear();
    }
    if (probs)
    {
        probs->clear();
    }
    if (infer_us)
    {
        *infer_us = 0;
    }
    return false;
#endif
}

void LQ_NCNN::SetModelPath(const std::string &param_path, const std::string &bin_path)
{
    m_param_path = param_path;
    m_bin_path = bin_path;
}

void LQ_NCNN::SetInputSize(int width, int height)
{
    m_input_width = width;
    m_input_height = height;
}

void LQ_NCNN::SetLabels(const std::vector<std::string> &labels)
{
    m_labels = labels;
}

void LQ_NCNN::SetNormalize(const float mean_vals[3], const float norm_vals[3])
{
    m_mean_vals[0] = mean_vals[0];
    m_mean_vals[1] = mean_vals[1];
    m_mean_vals[2] = mean_vals[2];
    m_norm_vals[0] = norm_vals[0];
    m_norm_vals[1] = norm_vals[1];
    m_norm_vals[2] = norm_vals[2];
}

#ifdef VISION_ENABLE_NCNN
int LQ_NCNN::Argmax(const ncnn::Mat &logits)
{
    if (logits.w <= 0)
    {
        return -1;
    }
    int best_index = 0;
    float best_value = -std::numeric_limits<float>::infinity();
    for (int i = 0; i < logits.w; ++i)
    {
        if (logits[i] > best_value)
        {
            best_value = logits[i];
            best_index = i;
        }
    }
    return best_index;
}
#endif

LQ_NCNN::~LQ_NCNN() = default;

bool vision_infer_init_default_model(LQ_NCNN &ncnn)
{
#ifndef VISION_ENABLE_NCNN
    (void)ncnn;
    printf("[NCNN] disabled at build time, skip model init\n");
    return false;
#else
    // 默认模型配置：集中在此，替换模型时优先修改这里。
    std::string model_param;
    std::string model_bin;
    resolve_default_model_paths(&model_param, &model_bin);
    const int input_width = g_vision_runtime_config.ncnn_input_width;
    const int input_height = g_vision_runtime_config.ncnn_input_height;
    std::vector<std::string> labels;
    labels.reserve(g_vision_runtime_config.ncnn_label_count);
    for (size_t i = 0; i < g_vision_runtime_config.ncnn_label_count &&
                       i < VISION_NCNN_CONFIG_MAX_LABELS; ++i)
    {
        const char *label = g_vision_runtime_config.ncnn_labels[i];
        if (label == nullptr || label[0] == '\0')
        {
            break;
        }
        labels.emplace_back(label);
    }
    float mean_vals[3] = {123.675f, 116.28f, 103.53f};
    float norm_vals[3] = {1.0f / 58.395f, 1.0f / 57.12f, 1.0f / 57.375f};

    ncnn.SetModelPath(model_param, model_bin);
    ncnn.SetInputSize(input_width, input_height);
    ncnn.SetLabels(labels);
    ncnn.SetNormalize(mean_vals, norm_vals);
    printf("[NCNN] loading model: %s / %s\n", model_param.c_str(), model_bin.c_str());
    if (!ncnn.Init())
    {
        printf("[NCNN] model init failed\n");
        return false;
    }
    printf("[NCNN] model init ok\n");
    return true;
#endif
}

bool vision_infer_async_init(LQ_NCNN *ncnn, bool enabled)
{
    // 模块初始化：绑定 ncnn 运行态并启动 worker。
    g_ncnn = ncnn;
    g_infer_enabled.store(enabled);
    g_ncnn_enabled.store(enabled);
    reset_infer_shared_state();

    {
        std::lock_guard<std::mutex> lock(g_infer_mutex);
        g_infer_worker_stop = false;
    }
    if (!g_infer_worker_running)
    {
        g_infer_thread = std::thread(run_infer_worker);
        g_infer_worker_running = true;
    }
    return true;
}

void vision_infer_async_cleanup()
{
    // 停止 worker 并清理共享状态。
    if (g_infer_worker_running)
    {
        {
            std::lock_guard<std::mutex> lock(g_infer_mutex);
            g_infer_worker_stop = true;
            g_infer_job_ready = false;
        }
        g_infer_cv.notify_all();
        if (g_infer_thread.joinable())
        {
            g_infer_thread.join();
        }
        g_infer_worker_running = false;
    }
    reset_infer_shared_state();
}

void vision_infer_async_set_enabled(bool enabled)
{
    g_infer_enabled.store(enabled);
    if (!enabled)
    {
        reset_infer_shared_state();
    }
}

bool vision_infer_async_enabled()
{
    return g_infer_enabled.load();
}

void vision_infer_async_set_ncnn_enabled(bool enabled)
{
    g_ncnn_enabled.store(enabled);
}

bool vision_infer_async_ncnn_enabled()
{
    return g_ncnn_enabled.load();
}

void vision_infer_async_submit_frame(const uint8 *bgr_proc_data,
                                     int proc_width,
                                     int proc_height,
                                     const uint8 *bgr_full_data,
                                     int full_width,
                                     int full_height)
{
    if (!g_infer_enabled.load() || bgr_proc_data == nullptr)
    {
        return;
    }
    // 尺寸不匹配时直接丢弃（保护当前固定分辨率流程）。
    if (proc_width != kProcWidth || proc_height != kProcHeight || full_width != kFullWidth || full_height != kFullHeight)
    {
        return;
    }

    cv::Mat proc_frame(kProcHeight, kProcWidth, CV_8UC3, const_cast<uint8 *>(bgr_proc_data));
    {
        std::lock_guard<std::mutex> lock(g_infer_mutex);
        g_infer_job.proc_bgr = proc_frame.clone();
        if (bgr_full_data != nullptr)
        {
            cv::Mat full_frame(kFullHeight, kFullWidth, CV_8UC3, const_cast<uint8 *>(bgr_full_data));
            g_infer_job.full_bgr = full_frame.clone();
        }
        else
        {
            g_infer_job.full_bgr.release();
        }
        g_infer_job_ready = true;
    }
    g_infer_cv.notify_one();
}

bool vision_infer_async_fetch_latest(vision_infer_async_result_t *out)
{
    if (out == nullptr)
    {
        return false;
    }

    infer_worker_result_t result{};
    uint32 result_seq = 0;
    {
        std::lock_guard<std::mutex> lock(g_infer_mutex);
        if (!g_latest_infer_result_valid)
        {
            return false;
        }
        result = g_latest_infer_result;
        result_seq = g_latest_infer_result_seq;
    }

    out->result_seq = result_seq;
    out->found = result.found;
    out->red_x = result.red_x;
    out->red_y = result.red_y;
    out->red_w = result.red_w;
    out->red_h = result.red_h;
    out->red_cx = result.red_cx;
    out->red_cy = result.red_cy;
    out->red_area = result.red_area;
    out->red_detect_us = result.red_detect_us;
    out->ncnn_roi_valid = result.found && result.ncnn_roi_full.width > 0 && result.ncnn_roi_full.height > 0;
    out->ncnn_roi_x = result.ncnn_roi_full.x;
    out->ncnn_roi_y = result.ncnn_roi_full.y;
    out->ncnn_roi_w = result.ncnn_roi_full.width;
    out->ncnn_roi_h = result.ncnn_roi_full.height;
    out->ncnn_enabled = result.ncnn_enabled;
    out->ncnn_infer_valid = result.ncnn_infer_valid;
    out->ncnn_infer_us = result.ncnn_infer_us;
    out->ncnn_top_class_id = result.ncnn_top_class_id;
    out->ncnn_top_score = result.ncnn_top_score;
    std::memset(out->ncnn_top_label, 0, sizeof(out->ncnn_top_label));
    std::snprintf(out->ncnn_top_label, sizeof(out->ncnn_top_label), "%s", result.ncnn_top_label.c_str());
    out->ncnn_class_count = std::min(static_cast<int>(result.ncnn_probs.size()), VISION_NCNN_MAX_CLASSES);
    for (int i = 0; i < VISION_NCNN_MAX_CLASSES; ++i)
    {
        out->ncnn_probs[i] = 0.0f;
        std::memset(out->ncnn_labels[i], 0, sizeof(out->ncnn_labels[i]));
    }
    for (int i = 0; i < out->ncnn_class_count; ++i)
    {
        out->ncnn_probs[i] = result.ncnn_probs[i];
        const std::string &label = (i < static_cast<int>(result.ncnn_labels.size()))
                                       ? result.ncnn_labels[i]
                                       : std::to_string(i);
        std::snprintf(out->ncnn_labels[i], sizeof(out->ncnn_labels[i]), "%s", label.c_str());
    }
    return true;
}
