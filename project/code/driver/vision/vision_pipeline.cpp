#include "driver/vision/vision_pipeline.h"

#include "driver/vision/vision_client_sender.h"
#include "driver/vision/vision_image_processor.h"
#include "driver/vision/vision_ncnn.h"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <limits>
#include <mutex>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>

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

static inline int scale_full_width(int ref_px)
{
    if (ref_px <= 0)
    {
        return 0;
    }
    return std::max(1, (ref_px * kFullWidth + kRefFullWidth / 2) / kRefFullWidth);
}

static inline int scale_full_height(int ref_px)
{
    if (ref_px <= 0)
    {
        return 0;
    }
    return std::max(1, (ref_px * kFullHeight + kRefFullHeight / 2) / kRefFullHeight);
}

// 以处理分辨率(160x120)为基准的推理ROI边长。
// 设为40表示：在160x120图上，ROI画框/识别窗口为40x40。
static constexpr int kInferRoiSizeProcRef = 40;
static constexpr int kInferBottomOffsetFullRef = 10;
static constexpr int kInferMinIntervalMs = 200;
static constexpr int kInferMaxStaleMs = 280;
static constexpr int kInferCenterMovePxRef = 4;
static constexpr int kInferAreaChangePercent = 20;
static constexpr int kCaptureMinIntervalMs = 1000;
static constexpr int kCaptureMaxCount = 20;
static constexpr const char *kCaptureDir = "vision_roi_captures";

static constexpr int kRedSearchExpandXRef = 28;
static constexpr int kRedSearchExpandYRef = 24;
static constexpr int kRedSearchMinWRef = 48;
static constexpr int kRedSearchMinHRef = 36;

static constexpr int kRedExpectedAreaPxRef = 60;
static constexpr int kRedAreaMinPxRef = 20;
static constexpr int kRedAreaMaxPxRef = 450;

struct red_ncnn_result_t
{
    bool ready = false;
    bool found = false;
    int red_x = 0;
    int red_y = 0;
    int red_w = 0;
    int red_h = 0;
    int red_cx = 0;
    int red_cy = 0;
    int red_area = 0;
    uint32 red_detect_us = 0;
    cv::Rect ncnn_roi;
};

static std::thread g_red_ncnn_thread;
static std::atomic<bool> g_red_ncnn_running(false);

static std::mutex g_red_ncnn_task_mutex;
static std::condition_variable g_red_ncnn_task_cv;
static std::array<uint8, kProcWidth * kProcHeight * 3> g_red_ncnn_proc_frame{};
static std::array<uint8, kFullWidth * kFullHeight * 3> g_red_ncnn_full_frame{};
static cv::Rect g_red_search_roi(0, 0, kProcWidth, kProcHeight);
static uint32 g_red_ncnn_task_seq = 0;
static uint32 g_red_ncnn_handled_seq = 0;

static std::mutex g_red_ncnn_result_mutex;
static red_ncnn_result_t g_red_ncnn_result{};

static std::atomic<bool> g_roi_capture_mode_enabled(false);
static std::atomic<int> g_roi_capture_saved_count(0);
static std::atomic<int64_t> g_roi_capture_last_save_ms(-(int64_t)kCaptureMinIntervalMs);
static std::atomic<bool> g_roi_capture_done_notified(false);

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
    // 先在处理分辨率坐标系里确定40x40，再映射到full图。
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
    const bool ok = cv::imwrite(path, roi);
    if (ok)
    {
        printf("[VISION CAPTURE] saved %d/%d %s\r\n", idx + 1, kCaptureMaxCount, path);
        if (idx + 1 >= kCaptureMaxCount && !g_roi_capture_done_notified.exchange(true))
        {
            printf("[VISION CAPTURE] done count=%d\r\n", kCaptureMaxCount);
        }
    }
    else
    {
        printf("[VISION CAPTURE] imwrite failed %s\r\n", path);
    }
}

static cv::Rect fallback_center_roi()
{
    const int width = kProcWidth;
    const int height = kProcHeight;
    const int rw = std::clamp(width / 2, scale_by_width(kRedSearchMinWRef), width);
    const int rh = std::clamp(height / 2, scale_by_height(kRedSearchMinHRef), height);
    int rx = (width - rw) / 2;
    int ry = (height - rh) / 2;
    rx = std::clamp(rx, 0, std::max(0, width - rw));
    ry = std::clamp(ry, 0, std::max(0, height - rh));
    return cv::Rect(rx, ry, rw, rh);
}

static cv::Rect build_red_search_roi_from_midline()
{
    uint16 *x2 = nullptr;
    uint16 *y2 = nullptr;
    uint16 dot_num = 0;
    vision_image_processor_get_boundaries(nullptr, &x2, nullptr, nullptr, &y2, nullptr, &dot_num);
    if (x2 == nullptr || y2 == nullptr || dot_num == 0)
    {
        return fallback_center_roi();
    }

    const int width = kProcWidth;
    const int height = kProcHeight;
    int min_x = width - 1;
    int max_x = 0;
    int min_y = height - 1;
    int max_y = 0;
    for (int i = 0; i < static_cast<int>(dot_num); ++i)
    {
        const int x = std::clamp(static_cast<int>(x2[i]), 0, width - 1);
        const int y = std::clamp(static_cast<int>(y2[i]), 0, height - 1);
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
    }

    const int kExpandX = scale_by_width(kRedSearchExpandXRef);
    const int kExpandY = scale_by_height(kRedSearchExpandYRef);
    const int kMinRoiW = scale_by_width(kRedSearchMinWRef);
    const int kMinRoiH = scale_by_height(kRedSearchMinHRef);

    int x0 = std::max(0, min_x - kExpandX);
    int y0 = std::max(0, min_y - kExpandY);
    int x1 = std::min(width - 1, max_x + kExpandX);
    int y1 = std::min(height - 1, max_y + kExpandY);

    int rw = x1 - x0 + 1;
    int rh = y1 - y0 + 1;
    if (rw < kMinRoiW)
    {
        const int cx = (x0 + x1) / 2;
        x0 = std::max(0, cx - kMinRoiW / 2);
        x1 = std::min(width - 1, x0 + kMinRoiW - 1);
        x0 = std::max(0, x1 - kMinRoiW + 1);
        rw = x1 - x0 + 1;
    }
    if (rh < kMinRoiH)
    {
        const int cy = (y0 + y1) / 2;
        y0 = std::max(0, cy - kMinRoiH / 2);
        y1 = std::min(height - 1, y0 + kMinRoiH - 1);
        y0 = std::max(0, y1 - kMinRoiH + 1);
        rh = y1 - y0 + 1;
    }

    if (rw <= 0 || rh <= 0)
    {
        return fallback_center_roi();
    }
    return cv::Rect(x0, y0, rw, rh);
}

static bool detect_red_rectangle_bbox(const cv::Mat &bgr,
                                      const cv::Rect &search_roi,
                                      cv::Rect *bbox,
                                      int *area_px)
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
        const int shape_cost = std::abs((w * 10) - (h * 24)); // 12:5 ~ 24:10
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

static void red_ncnn_worker_loop()
{
    std::array<uint8, kProcWidth * kProcHeight * 3> local_proc_frame{};
    std::array<uint8, kFullWidth * kFullHeight * 3> local_full_frame{};
    bool last_detect_found = false;
    bool last_infer_valid = false;
    int last_infer_cx = 0;
    int last_infer_cy = 0;
    int last_infer_area = 0;
    auto last_infer_tp = std::chrono::steady_clock::now() - std::chrono::milliseconds(kInferMaxStaleMs);

    while (g_red_ncnn_running.load())
    {
        uint32 task_seq = 0;
        cv::Rect search_roi;
        {
            std::unique_lock<std::mutex> lk(g_red_ncnn_task_mutex);
            g_red_ncnn_task_cv.wait(lk, []() {
                return !g_red_ncnn_running.load() || (g_red_ncnn_task_seq != g_red_ncnn_handled_seq);
            });
            if (!g_red_ncnn_running.load())
            {
                break;
            }

            task_seq = g_red_ncnn_task_seq;
            g_red_ncnn_handled_seq = task_seq;
            search_roi = g_red_search_roi;
            std::memcpy(local_proc_frame.data(), g_red_ncnn_proc_frame.data(), local_proc_frame.size());
            std::memcpy(local_full_frame.data(), g_red_ncnn_full_frame.data(), local_full_frame.size());
        }

        cv::Mat proc_frame(kProcHeight, kProcWidth, CV_8UC3, local_proc_frame.data());
        cv::Mat full_frame(kFullHeight, kFullWidth, CV_8UC3, local_full_frame.data());
        auto t0 = std::chrono::steady_clock::now();
        cv::Rect red_bbox;
        int red_area = 0;
        const bool found = detect_red_rectangle_bbox(proc_frame, search_roi, &red_bbox, &red_area);
        auto t1 = std::chrono::steady_clock::now();
        const uint32 red_detect_us = static_cast<uint32>(std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());

        red_ncnn_result_t result{};
        result.ready = true;
        result.found = found;
        result.red_detect_us = red_detect_us;
        if (found)
        {
            red_bbox &= cv::Rect(0, 0, kProcWidth, kProcHeight);
            result.red_x = red_bbox.x;
            result.red_y = red_bbox.y;
            result.red_w = red_bbox.width;
            result.red_h = red_bbox.height;
            result.red_cx = red_bbox.x + red_bbox.width / 2;
            result.red_cy = red_bbox.y + red_bbox.height / 2;
            result.red_area = red_area;
            const cv::Rect ncnn_full_roi = build_ncnn_full_roi_from_red_rect_proc(result.red_cx,
                                                                                   result.red_y,
                                                                                   result.red_h);
            result.ncnn_roi = map_full_roi_to_proc(ncnn_full_roi);
            try_save_infer_roi_png(full_frame, ncnn_full_roi);

            if (vision_ncnn_is_enabled())
            {
                const auto now_tp = std::chrono::steady_clock::now();
                const int64_t elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - last_infer_tp).count();

                const int dx = result.red_cx - last_infer_cx;
                const int dy = result.red_cy - last_infer_cy;
                const int64_t center_move_sq = static_cast<int64_t>(dx) * dx + static_cast<int64_t>(dy) * dy;
                const int center_move_px = scale_by_width(kInferCenterMovePxRef);
                const bool center_changed = center_move_sq >= static_cast<int64_t>(center_move_px) * center_move_px;
                const int safe_prev_area = std::max(1, last_infer_area);
                const int area_diff = std::abs(result.red_area - last_infer_area);
                const bool area_changed = (area_diff * 100) >= (safe_prev_area * kInferAreaChangePercent);

                bool should_infer = false;
                if (!last_infer_valid)
                {
                    // 首次识别立即触发。
                    should_infer = true;
                }
                else if (elapsed_ms >= kInferMinIntervalMs)
                {
                    // 严格限频：没达到最小间隔，无论目标变化多大都不推理。
                    if (!last_detect_found)
                    {
                        should_infer = true;
                    }
                    else if (center_changed || area_changed)
                    {
                        should_infer = true;
                    }
                    else if (elapsed_ms >= kInferMaxStaleMs)
                    {
                        should_infer = true; // 长时间未刷新，强制推理
                    }
                }

                if (should_infer)
                {
                    cv::Mat roi_bgr = full_frame(ncnn_full_roi).clone();
                    vision_ncnn_step(reinterpret_cast<const uint8 *>(roi_bgr.data), roi_bgr.cols, roi_bgr.rows);
                    last_infer_valid = true;
                    last_infer_tp = now_tp;
                    last_infer_cx = result.red_cx;
                    last_infer_cy = result.red_cy;
                    last_infer_area = result.red_area;
                }
            }
        }
        last_detect_found = found;

        vision_image_processor_set_last_red_detect_us(red_detect_us);
        if (found)
        {
            vision_image_processor_set_red_rect(true,
                                                result.red_x,
                                                result.red_y,
                                                result.red_w,
                                                result.red_h,
                                                result.red_cx,
                                                result.red_cy,
                                                result.red_area);
        }
        else
        {
            vision_image_processor_set_red_rect(false, 0, 0, 0, 0, 0, 0, 0);
        }

        (void)task_seq;
        {
            std::lock_guard<std::mutex> lk(g_red_ncnn_result_mutex);
            g_red_ncnn_result = result;
        }
    }
}

static void submit_async_task(const uint8 *bgr_proc_data,
                              const uint8 *bgr_full_data,
                              const cv::Rect &search_roi)
{
    if (bgr_proc_data == nullptr || bgr_full_data == nullptr)
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lk(g_red_ncnn_task_mutex);
        std::memcpy(g_red_ncnn_proc_frame.data(), bgr_proc_data, g_red_ncnn_proc_frame.size());
        std::memcpy(g_red_ncnn_full_frame.data(), bgr_full_data, g_red_ncnn_full_frame.size());
        g_red_search_roi = search_roi;
        ++g_red_ncnn_task_seq;
    }
    g_red_ncnn_task_cv.notify_one();
}

static red_ncnn_result_t get_latest_async_result()
{
    std::lock_guard<std::mutex> lk(g_red_ncnn_result_mutex);
    return g_red_ncnn_result;
}
} // namespace

// 初始化视觉总线：图像处理 + ncnn推理 + 客户端发送
bool vision_pipeline_init(const char *camera_path, LQ_NCNN *ncnn, bool ncnn_enabled)
{
    if (!vision_image_processor_init(camera_path))
    {
        return false;
    }

    vision_ncnn_bind(ncnn, ncnn_enabled);
    vision_client_sender_init();

    vision_image_processor_set_red_rect(false, 0, 0, 0, 0, 0, 0, 0);
    vision_image_processor_set_last_red_detect_us(0);
    vision_image_processor_set_ncnn_roi(false, 0, 0, 0, 0);

    g_red_ncnn_result = red_ncnn_result_t{};
    g_red_ncnn_task_seq = 0;
    g_red_ncnn_handled_seq = 0;
    g_roi_capture_saved_count.store(0);
    g_roi_capture_last_save_ms.store(steady_now_ms() - kCaptureMinIntervalMs);
    g_roi_capture_done_notified.store(false);

    g_red_ncnn_running.store(true);
    g_red_ncnn_thread = std::thread(red_ncnn_worker_loop);
    return true;
}

void vision_pipeline_cleanup()
{
    if (g_red_ncnn_running.exchange(false))
    {
        g_red_ncnn_task_cv.notify_all();
        if (g_red_ncnn_thread.joinable())
        {
            g_red_ncnn_thread.join();
        }
    }

    vision_image_processor_set_ncnn_roi(false, 0, 0, 0, 0);
    vision_image_processor_set_red_rect(false, 0, 0, 0, 0, 0, 0, 0);
    vision_image_processor_set_last_red_detect_us(0);
    g_roi_capture_saved_count.store(0);
    g_roi_capture_done_notified.store(false);
    vision_image_processor_cleanup();
}

// 仅做采图和处理，不做发送
bool vision_pipeline_process_step()
{
    if (!vision_image_processor_process_step())
    {
        return false;
    }

    const uint8 *bgr_proc_data = vision_image_processor_bgr_image();
    const uint8 *bgr_full_data = vision_image_processor_bgr_full_image();
    if (bgr_proc_data != nullptr && bgr_full_data != nullptr)
    {
        submit_async_task(bgr_proc_data, bgr_full_data, build_red_search_roi_from_midline());
    }

    const red_ncnn_result_t result = get_latest_async_result();
    if (result.ready && result.found)
    {
        vision_image_processor_set_ncnn_roi(true,
                                            result.ncnn_roi.x,
                                            result.ncnn_roi.y,
                                            result.ncnn_roi.width,
                                            result.ncnn_roi.height);

        if (bgr_proc_data != nullptr)
        {
            cv::Mat frame(kProcHeight, kProcWidth, CV_8UC3, const_cast<uint8 *>(bgr_proc_data));
            cv::rectangle(frame, result.ncnn_roi, cv::Scalar(0, 255, 0), 1, cv::LINE_8);
        }

        const uint8 *gray_data = vision_image_processor_gray_image();
        if (gray_data != nullptr)
        {
            cv::Mat gray(kProcHeight, kProcWidth, CV_8UC1, const_cast<uint8 *>(gray_data));
            cv::rectangle(gray, result.ncnn_roi, cv::Scalar(255), 1, cv::LINE_8);
        }
    }
    else
    {
        vision_image_processor_set_ncnn_roi(false, 0, 0, 0, 0);
    }
    return true;
}

// 仅做发送
void vision_pipeline_send_step()
{
    vision_client_sender_send_step();
}

// 兼容旧接口：处理 + 发送
bool vision_pipeline_step()
{
    if (!vision_pipeline_process_step())
    {
        return false;
    }
    vision_pipeline_send_step();
    return true;
}

const uint8 *vision_pipeline_bgr_image()
{
    return vision_image_processor_bgr_image();
}

void vision_pipeline_set_send_mode(vision_send_mode_enum mode)
{
    vision_client_sender_set_mode(mode);
}

vision_send_mode_enum vision_pipeline_get_send_mode()
{
    return vision_client_sender_get_mode();
}

void vision_pipeline_set_send_max_fps(uint32 max_fps)
{
    vision_client_sender_set_max_fps(max_fps);
}

uint32 vision_pipeline_get_send_max_fps()
{
    return vision_client_sender_get_max_fps();
}

void vision_pipeline_set_send_enabled(bool enabled)
{
    vision_client_sender_set_enabled(enabled);
}

bool vision_pipeline_is_send_enabled()
{
    return vision_client_sender_is_enabled();
}

void vision_pipeline_set_roi_capture_mode(bool enabled)
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

bool vision_pipeline_roi_capture_mode_enabled()
{
    return g_roi_capture_mode_enabled.load();
}

void vision_pipeline_get_red_rect(bool *found, int *x, int *y, int *w, int *h, int *cx, int *cy)
{
    vision_image_processor_get_red_rect(found, x, y, w, h, cx, cy);
}

int vision_pipeline_get_red_rect_area()
{
    return vision_image_processor_get_red_rect_area();
}
