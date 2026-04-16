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
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace
{
// 固定分辨率参数：当前工程推理 ROI 直接基于 160x120 处理图裁剪。
static constexpr int kProcWidth = VISION_DOWNSAMPLED_WIDTH;
static constexpr int kProcHeight = VISION_DOWNSAMPLED_HEIGHT;
static constexpr int kFullWidth = UVC_WIDTH;
static constexpr int kFullHeight = UVC_HEIGHT;

static constexpr int kRefProcWidth = 160;
static constexpr int kRefProcHeight = 120;
static constexpr int kRefProcPixels = kRefProcWidth * kRefProcHeight;

// 红框搜索区域参数（以 x=80 竖线附近搜索）。
static constexpr int kRedSearchExpandXRef = 28;
static constexpr int kRedSearchCenterXRef = 80;
static constexpr int kRedSearchMinWRef = 48;
static constexpr int kRedSearchMinHRef = 36;

// 红框连通域筛选阈值（面积范围与目标面积）。
static constexpr int kRedExpectedAreaPxRef = 60;
static constexpr int kRedAreaMinPxRef = 20;
static constexpr int kRedAreaMaxPxRef = 450;

struct infer_job_t
{
    // 异步任务输入：处理分辨率 BGR + full 分辨率 BGR。
    cv::Mat proc_bgr;
    cv::Mat full_bgr;
};

struct infer_worker_result_t
{
    // 异步任务输出：红框信息 + ncnn ROI（处理分辨率坐标）。
    bool found = false;
    int red_x = 0;
    int red_y = 0;
    int red_w = 0;
    int red_h = 0;
    int red_cx = 0;
    int red_cy = 0;
    int red_area = 0;
    uint32 red_detect_us = 0;
    cv::Rect ncnn_roi_proc;
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

// 作用：参考分辨率参数缩放到当前处理分辨率。
static inline int scale_by_width(int ref_px)
{
    if (ref_px <= 0)
    {
        return 0;
    }
    return std::max(1, (ref_px * kProcWidth + kRefProcWidth / 2) / kRefProcWidth);
}

static inline int scale_by_height(int ref_px)
{
    if (ref_px <= 0)
    {
        return 0;
    }
    return std::max(1, (ref_px * kProcHeight + kRefProcHeight / 2) / kRefProcHeight);
}

static inline int scale_by_area(int ref_area_px)
{
    if (ref_area_px <= 0)
    {
        return 0;
    }
    const int64_t scaled = static_cast<int64_t>(ref_area_px) * kProcWidth * kProcHeight;
    return std::max(1, static_cast<int>((scaled + kRefProcPixels / 2) / kRefProcPixels));
}

static cv::Rect build_ncnn_proc_roi_from_red_rect(int red_x_proc,
                                                  int red_y_proc,
                                                  int red_w_proc,
                                                  int red_h_proc)
{
    // 对齐 vision_specific：
    // 1. 以红框上边中点作为 ROI 中心；
    // 2. ROI 为正方形；
    // 3. 边长 = 红框宽 * 3 / 2。
    const int side_proc = std::max(1, (std::max(1, red_w_proc) * 3) / 2);
    const int center_x_proc = red_x_proc + red_w_proc / 2;
    const int center_y_proc = red_y_proc;

    int roi_x_proc = center_x_proc - side_proc / 2;
    int roi_y_proc = center_y_proc - side_proc / 2;
    roi_x_proc = std::clamp(roi_x_proc, 0, std::max(0, kProcWidth - side_proc));
    roi_y_proc = std::clamp(roi_y_proc, 0, std::max(0, kProcHeight - side_proc));
    return cv::Rect(roi_x_proc, roi_y_proc, side_proc, side_proc) & cv::Rect(0, 0, kProcWidth, kProcHeight);
}

static cv::Rect fallback_center_roi()
{
    const int rw = std::clamp(kProcWidth / 2, scale_by_width(kRedSearchMinWRef), kProcWidth);
    const int rh = std::clamp(kProcHeight / 2, scale_by_height(kRedSearchMinHRef), kProcHeight);
    int rx = (kProcWidth - rw) / 2;
    int ry = (kProcHeight - rh) / 2;
    rx = std::clamp(rx, 0, std::max(0, kProcWidth - rw));
    ry = std::clamp(ry, 0, std::max(0, kProcHeight - rh));
    return cv::Rect(rx, ry, rw, rh);
}

// 作用：在 x=80 附近生成红框搜索 ROI。
static cv::Rect build_red_search_roi_from_x_line()
{
    const int kExpandX = scale_by_width(kRedSearchExpandXRef);
    const int kMinRoiW = scale_by_width(kRedSearchMinWRef);
    const int kMinRoiH = std::min(kProcHeight, scale_by_height(kRedSearchMinHRef));
    const int center_x = std::clamp(scale_by_width(kRedSearchCenterXRef), 0, kProcWidth - 1);

    int x0 = std::max(0, center_x - kExpandX);
    int x1 = std::min(kProcWidth - 1, center_x + kExpandX);
    int y0 = 0;
    int y1 = kProcHeight - 1;

    int rw = x1 - x0 + 1;
    int rh = y1 - y0 + 1;
    if (rw < kMinRoiW)
    {
        const int cx = (x0 + x1) / 2;
        x0 = std::max(0, cx - kMinRoiW / 2);
        x1 = std::min(kProcWidth - 1, x0 + kMinRoiW - 1);
        x0 = std::max(0, x1 - kMinRoiW + 1);
        rw = x1 - x0 + 1;
    }
    if (rh < kMinRoiH)
    {
        const int cy = (y0 + y1) / 2;
        y0 = std::max(0, cy - kMinRoiH / 2);
        y1 = std::min(kProcHeight - 1, y0 + kMinRoiH - 1);
        y0 = std::max(0, y1 - kMinRoiH + 1);
        rh = y1 - y0 + 1;
    }
    if (rw <= 0 || rh <= 0)
    {
        return fallback_center_roi();
    }
    return cv::Rect(x0, y0, rw, rh);
}

// 作用：在搜索 ROI 内找红色实心矩形。
// 如何修改：可调颜色阈值、面积阈值、形状约束。
static bool detect_red_rectangle_bbox(const cv::Mat &bgr, const cv::Rect &search_roi, cv::Rect *bbox, int *area_px)
{
    if (bbox == nullptr || area_px == nullptr || bgr.empty() || bgr.type() != CV_8UC3)
    {
        return false;
    }

    cv::Rect roi = search_roi & cv::Rect(0, 0, bgr.cols, bgr.rows);
    if (roi.width <= 0 || roi.height <= 0)
    {
        return false;
    }

    cv::Mat mask(roi.height, roi.width, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < roi.height; ++y)
    {
        const cv::Vec3b *src = bgr.ptr<cv::Vec3b>(roi.y + y);
        uint8 *dst = mask.ptr<uint8>(y);
        for (int x = 0; x < roi.width; ++x)
        {
            const cv::Vec3b &px = src[roi.x + x];
            const int b = static_cast<int>(px[0]);
            const int g = static_cast<int>(px[1]);
            const int r = static_cast<int>(px[2]);
            if (r >= 80 && (r - g) >= 28 && (r - b) >= 22)
            {
                dst[x] = 255;
            }
        }
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int comp_num = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
    if (comp_num <= 1)
    {
        return false;
    }

    int best_label = -1;
    int best_score = std::numeric_limits<int>::max();
    int best_area = 0;
    const int kAreaMinPx = scale_by_area(kRedAreaMinPxRef);
    const int kAreaMaxPx = scale_by_area(kRedAreaMaxPxRef);
    const int kExpectedAreaPx = scale_by_area(kRedExpectedAreaPxRef);
    const int kMinCompW = scale_by_width(3);
    const int kMinCompH = scale_by_height(3);
    for (int i = 1; i < comp_num; ++i)
    {
        const int area = stats.at<int>(i, cv::CC_STAT_AREA);
        const int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
        const int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        if (area < kAreaMinPx || area > kAreaMaxPx || w < kMinCompW || h < kMinCompH)
        {
            continue;
        }

        const int side_min = std::min(w, h);
        const int side_max = std::max(w, h);
        if (side_min <= 0 || side_max > side_min * 6)
        {
            continue;
        }

        const int area_cost = std::abs(area - kExpectedAreaPx);
        const int shape_cost = std::abs((w * 10) - (h * 24));
        const int score = area_cost * 4 + shape_cost;
        if (score < best_score || (score == best_score && area > best_area))
        {
            best_score = score;
            best_label = i;
            best_area = area;
        }
    }

    if (best_label < 0)
    {
        return false;
    }

    *bbox = cv::Rect(stats.at<int>(best_label, cv::CC_STAT_LEFT) + roi.x,
                     stats.at<int>(best_label, cv::CC_STAT_TOP) + roi.y,
                     stats.at<int>(best_label, cv::CC_STAT_WIDTH),
                     stats.at<int>(best_label, cv::CC_STAT_HEIGHT));
    *area_px = best_area;
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
        const cv::Rect search_roi = build_red_search_roi_from_x_line();
        const bool found = detect_red_rectangle_bbox(job.proc_bgr, search_roi, &red_bbox, &red_area);
        auto detect_end = std::chrono::steady_clock::now();
        result.red_detect_us = static_cast<uint32>(
            std::chrono::duration_cast<std::chrono::microseconds>(detect_end - detect_start).count());

        if (found)
        {
            red_bbox &= cv::Rect(0, 0, kProcWidth, kProcHeight);
            result.found = true;
            result.red_x = red_bbox.x;
            result.red_y = red_bbox.y;
            result.red_w = red_bbox.width;
            result.red_h = red_bbox.height;
            result.red_cx = red_bbox.x + red_bbox.width / 2;
            result.red_cy = red_bbox.y + red_bbox.height / 2;
            result.red_area = red_area;

            result.ncnn_roi_proc = build_ncnn_proc_roi_from_red_rect(result.red_x,
                                                                     result.red_y,
                                                                     result.red_w,
                                                                     result.red_h);

            if (g_ncnn_enabled.load())
            {
                cv::Rect safe_roi = result.ncnn_roi_proc & cv::Rect(0, 0, kProcWidth, kProcHeight);
                if (safe_roi.width > 0 && safe_roi.height > 0)
                {
                    cv::Mat roi_bgr = job.proc_bgr(safe_roi).clone();
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
    , m_input_width(96)
    , m_input_height(96)
    , m_input_name("in0")
    , m_output_name("out0")
{
    m_mean_vals[0] = 123.675f;
    m_mean_vals[1] = 116.28f;
    m_mean_vals[2] = 103.53f;
    m_norm_vals[0] = 0.01712475f;
    m_norm_vals[1] = 0.017507f;
    m_norm_vals[2] = 0.01742919f;
}

bool LQ_NCNN::Init()
{
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

LQ_NCNN::~LQ_NCNN() = default;

bool vision_infer_init_default_model(LQ_NCNN &ncnn)
{
    // 默认模型配置：集中在此，替换模型时优先修改这里。
    const std::string model_param = "tiny_classifier_fp32.ncnn.param";
    const std::string model_bin = "tiny_classifier_fp32.ncnn.bin";
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
    out->ncnn_roi_valid = result.found && result.ncnn_roi_proc.width > 0 && result.ncnn_roi_proc.height > 0;
    out->ncnn_roi_x = result.ncnn_roi_proc.x;
    out->ncnn_roi_y = result.ncnn_roi_proc.y;
    out->ncnn_roi_w = result.ncnn_roi_proc.width;
    out->ncnn_roi_h = result.ncnn_roi_proc.height;
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
