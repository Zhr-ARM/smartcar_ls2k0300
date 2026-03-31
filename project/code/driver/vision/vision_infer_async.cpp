#include "driver/vision/vision_infer_async.h"

#include "driver/vision/vision_image_processor.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cerrno>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <sys/stat.h>
#include <sys/types.h>

namespace
{
static constexpr int kProcWidth = VISION_DOWNSAMPLED_WIDTH;
static constexpr int kProcHeight = VISION_DOWNSAMPLED_HEIGHT;
static constexpr int kFullWidth = UVC_WIDTH;
static constexpr int kFullHeight = UVC_HEIGHT;

static constexpr int kRefProcWidth = 160;
static constexpr int kRefProcHeight = 120;
static constexpr int kRefProcPixels = kRefProcWidth * kRefProcHeight;
static constexpr int kRefFullWidth = 320;
static constexpr int kRefFullHeight = 240;

static constexpr int kInferRoiSizeProcRef = 40;
static constexpr int kInferBottomOffsetFullRef = 10;

static constexpr int kRedSearchExpandXRef = 28;
static constexpr int kRedSearchCenterXRef = 80;
static constexpr int kRedSearchMinWRef = 48;
static constexpr int kRedSearchMinHRef = 36;

static constexpr int kRedExpectedAreaPxRef = 60;
static constexpr int kRedAreaMinPxRef = 20;
static constexpr int kRedAreaMaxPxRef = 450;

static constexpr int kCaptureMinIntervalMs = 1000;
static constexpr int kCaptureMaxCount = 20;
static constexpr const char *kCaptureDir = "vision_roi_captures";

struct infer_job_t
{
    cv::Mat proc_bgr;
    cv::Mat full_bgr;
};

struct infer_worker_result_t
{
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
};

// 原 vision_ncnn + lq_ncnn 的运行态并入 infer 模块。
static LQ_NCNN *g_ncnn = nullptr;
static std::atomic<bool> g_infer_enabled(false);

static std::atomic<bool> g_roi_capture_mode_enabled(false);
static std::atomic<int> g_roi_capture_saved_count(0);
static std::atomic<int64_t> g_roi_capture_last_save_ms(-(int64_t)kCaptureMinIntervalMs);
static std::atomic<bool> g_roi_capture_done_notified(false);

static std::mutex g_infer_mutex;
static std::condition_variable g_infer_cv;
static std::thread g_infer_thread;
static bool g_infer_worker_running = false;
static bool g_infer_worker_stop = false;
static bool g_infer_job_ready = false;
static infer_job_t g_infer_job;
static infer_worker_result_t g_latest_infer_result;
static bool g_latest_infer_result_valid = false;

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

static inline int scale_full_height(int ref_px)
{
    if (ref_px <= 0)
    {
        return 0;
    }
    return std::max(1, (ref_px * kFullHeight + kRefFullHeight / 2) / kRefFullHeight);
}

static inline int map_proc_to_full_x(int x_proc)
{
    if (kProcWidth <= 1)
    {
        return 0;
    }
    return std::clamp((x_proc * (kFullWidth - 1)) / (kProcWidth - 1), 0, kFullWidth - 1);
}

static inline int map_proc_to_full_y(int y_proc)
{
    if (kProcHeight <= 1)
    {
        return 0;
    }
    return std::clamp((y_proc * (kFullHeight - 1)) / (kProcHeight - 1), 0, kFullHeight - 1);
}

static inline int map_full_to_proc_x(int x_full)
{
    if (kFullWidth <= 1)
    {
        return 0;
    }
    return std::clamp((x_full * (kProcWidth - 1)) / (kFullWidth - 1), 0, kProcWidth - 1);
}

static inline int map_full_to_proc_y(int y_full)
{
    if (kFullHeight <= 1)
    {
        return 0;
    }
    return std::clamp((y_full * (kProcHeight - 1)) / (kFullHeight - 1), 0, kProcHeight - 1);
}

static cv::Rect build_ncnn_full_roi_from_red_rect_proc(int cx_proc, int red_y_proc, int red_h_proc)
{
    const int roi_w_proc = std::min(scale_by_width(kInferRoiSizeProcRef), kProcWidth);
    const int roi_h_proc = std::min(scale_by_height(kInferRoiSizeProcRef), kProcHeight);
    const int roi_w = std::max(1, std::min((roi_w_proc * kFullWidth + kProcWidth / 2) / kProcWidth, kFullWidth));
    const int roi_h = std::max(1, std::min((roi_h_proc * kFullHeight + kProcHeight / 2) / kProcHeight, kFullHeight));
    const int cx_full = map_proc_to_full_x(cx_proc);
    const int red_top_full = map_proc_to_full_y(red_y_proc);
    const int red_h_full = std::max(1, (red_h_proc * kFullHeight + kProcHeight / 2) / kProcHeight);
    const int red_bottom_full = red_top_full + std::max(0, red_h_full - 1);
    const int roi_bottom = std::min(kFullHeight - 1, red_bottom_full + scale_full_height(kInferBottomOffsetFullRef));

    int roi_x = cx_full - roi_w / 2;
    int roi_y = roi_bottom - roi_h + 1;
    roi_x = std::clamp(roi_x, 0, std::max(0, kFullWidth - roi_w));
    roi_y = std::clamp(roi_y, 0, std::max(0, kFullHeight - roi_h));
    return cv::Rect(roi_x, roi_y, roi_w, roi_h);
}

static cv::Rect map_full_roi_to_proc(const cv::Rect &full_roi)
{
    cv::Rect safe = full_roi & cv::Rect(0, 0, kFullWidth, kFullHeight);
    if (safe.width <= 0 || safe.height <= 0)
    {
        return cv::Rect(0, 0, 0, 0);
    }
    const int x0 = map_full_to_proc_x(safe.x);
    const int y0 = map_full_to_proc_y(safe.y);
    const int x1 = map_full_to_proc_x(safe.x + safe.width - 1);
    const int y1 = map_full_to_proc_y(safe.y + safe.height - 1);
    const int w = std::max(1, x1 - x0 + 1);
    const int h = std::max(1, y1 - y0 + 1);
    return cv::Rect(x0, y0, w, h);
}

static int64_t steady_now_ms()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

static void ensure_capture_dir_once()
{
    static std::once_flag flag;
    std::call_once(flag, []() {
        if (mkdir(kCaptureDir, 0777) != 0 && errno != EEXIST)
        {
            printf("[VISION CAPTURE] mkdir failed: %s err=%d\r\n", kCaptureDir, errno);
        }
    });
}

static void try_save_infer_roi_png(const cv::Mat &frame, const cv::Rect &ncnn_roi)
{
    if (!g_roi_capture_mode_enabled.load())
    {
        return;
    }

    const int saved = g_roi_capture_saved_count.load();
    if (saved >= kCaptureMaxCount)
    {
        if (!g_roi_capture_done_notified.exchange(true))
        {
            printf("[VISION CAPTURE] done count=%d\r\n", saved);
        }
        return;
    }

    const int64_t now_ms = steady_now_ms();
    const int64_t last_ms = g_roi_capture_last_save_ms.load();
    if ((now_ms - last_ms) < kCaptureMinIntervalMs)
    {
        return;
    }
    g_roi_capture_last_save_ms.store(now_ms);

    const int idx = g_roi_capture_saved_count.fetch_add(1);
    if (idx >= kCaptureMaxCount)
    {
        g_roi_capture_saved_count.store(kCaptureMaxCount);
        return;
    }

    const cv::Rect safe_roi = ncnn_roi & cv::Rect(0, 0, frame.cols, frame.rows);
    if (safe_roi.width <= 0 || safe_roi.height <= 0)
    {
        return;
    }

    ensure_capture_dir_once();
    cv::Mat roi = frame(safe_roi).clone();
    char path[160] = {0};
    std::snprintf(path, sizeof(path), "%s/roi_%02d.png", kCaptureDir, idx + 1);
    if (cv::imwrite(path, roi))
    {
        printf("[VISION CAPTURE] saved %d/%d %s\r\n", idx + 1, kCaptureMaxCount, path);
        if (idx + 1 >= kCaptureMaxCount && !g_roi_capture_done_notified.exchange(true))
        {
            printf("[VISION CAPTURE] done count=%d\r\n", kCaptureMaxCount);
        }
    }
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

static void ncnn_step(const uint8 *bgr_data, int width, int height)
{
    if (!g_infer_enabled.load() || g_ncnn == nullptr || bgr_data == nullptr || width <= 0 || height <= 0)
    {
        return;
    }
    cv::Mat bgr(height, width, CV_8UC3, const_cast<uint8 *>(bgr_data));
    g_ncnn->Infer(bgr);
}

static void reset_infer_shared_state()
{
    std::lock_guard<std::mutex> lock(g_infer_mutex);
    g_infer_job.proc_bgr.release();
    g_infer_job.full_bgr.release();
    g_infer_job_ready = false;
    g_latest_infer_result = infer_worker_result_t{};
    g_latest_infer_result_valid = false;
}

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

            const cv::Rect ncnn_full_roi = build_ncnn_full_roi_from_red_rect_proc(result.red_cx, result.red_y, result.red_h);
            result.ncnn_roi_proc = map_full_roi_to_proc(ncnn_full_roi);

            try_save_infer_roi_png(job.full_bgr, ncnn_full_roi);
            if (g_infer_enabled.load())
            {
                cv::Rect safe_full_roi = ncnn_full_roi & cv::Rect(0, 0, kFullWidth, kFullHeight);
                if (safe_full_roi.width > 0 && safe_full_roi.height > 0)
                {
                    cv::Mat roi_bgr = job.full_bgr(safe_full_roi).clone();
                    ncnn_step(reinterpret_cast<const uint8 *>(roi_bgr.data), roi_bgr.cols, roi_bgr.rows);
                }
            }
        }

        {
            std::lock_guard<std::mutex> lock(g_infer_mutex);
            g_latest_infer_result = result;
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
    if (!m_initialized)
    {
        throw std::runtime_error("NCNN not initialized");
    }
    if (bgr_image.empty())
    {
        throw std::invalid_argument("Input image is empty");
    }

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

    int class_id = Argmax(logits);
    if (class_id >= 0 && class_id < static_cast<int>(m_labels.size()))
    {
        return m_labels[class_id];
    }
    return std::to_string(class_id);
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
    const std::string model_param = "tiny_classifier_fp32.ncnn.param";
    const std::string model_bin = "tiny_classifier_fp32.ncnn.bin";
    const int input_width = 64;
    const int input_height = 64;
    const std::vector<std::string> labels = {
        "gun", "explosive", "knife", "baton", "axe", "aid", "flashlight",
        "walkie", "bulletproof", "tele", "helmet", "fire", "amb", "arm", "motor"};
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
    g_ncnn = ncnn;
    g_infer_enabled.store(enabled);

    g_roi_capture_saved_count.store(0);
    g_roi_capture_last_save_ms.store(steady_now_ms() - kCaptureMinIntervalMs);
    g_roi_capture_done_notified.store(false);
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
    g_roi_capture_saved_count.store(0);
    g_roi_capture_done_notified.store(false);
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

void vision_infer_async_set_roi_capture_mode(bool enabled)
{
    const bool prev = g_roi_capture_mode_enabled.exchange(enabled);
    if (enabled && !prev)
    {
        g_roi_capture_saved_count.store(0);
        g_roi_capture_last_save_ms.store(steady_now_ms() - kCaptureMinIntervalMs);
        g_roi_capture_done_notified.store(false);
        ensure_capture_dir_once();
        printf("[VISION CAPTURE] mode=ON interval=%dms max=%d dir=%s\r\n",
               kCaptureMinIntervalMs,
               kCaptureMaxCount,
               kCaptureDir);
    }
    else if (!enabled && prev)
    {
        printf("[VISION CAPTURE] mode=OFF saved=%d\r\n", g_roi_capture_saved_count.load());
    }
}

bool vision_infer_async_roi_capture_mode_enabled()
{
    return g_roi_capture_mode_enabled.load();
}

void vision_infer_async_submit_frame(const uint8 *bgr_proc_data,
                                     int proc_width,
                                     int proc_height,
                                     const uint8 *bgr_full_data,
                                     int full_width,
                                     int full_height)
{
    if (!g_infer_enabled.load() || bgr_proc_data == nullptr || bgr_full_data == nullptr)
    {
        return;
    }
    if (proc_width != kProcWidth || proc_height != kProcHeight || full_width != kFullWidth || full_height != kFullHeight)
    {
        return;
    }

    cv::Mat proc_frame(kProcHeight, kProcWidth, CV_8UC3, const_cast<uint8 *>(bgr_proc_data));
    cv::Mat full_frame(kFullHeight, kFullWidth, CV_8UC3, const_cast<uint8 *>(bgr_full_data));
    {
        std::lock_guard<std::mutex> lock(g_infer_mutex);
        g_infer_job.proc_bgr = proc_frame.clone();
        g_infer_job.full_bgr = full_frame.clone();
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
    {
        std::lock_guard<std::mutex> lock(g_infer_mutex);
        if (!g_latest_infer_result_valid)
        {
            return false;
        }
        result = g_latest_infer_result;
    }

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
    return true;
}
