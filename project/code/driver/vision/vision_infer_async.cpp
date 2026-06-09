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
// 固定分辨率参数：主链处理图宽固定为 160，高度允许因预裁剪动态变化。
static constexpr int kProcWidth = VISION_DOWNSAMPLED_WIDTH;
static constexpr int kProcHeight = VISION_DOWNSAMPLED_HEIGHT;
static constexpr int kFullWidth = UVC_WIDTH;
static constexpr int kFullHeight = UVC_HEIGHT;
static constexpr int kBoardRoiSize = 32;
static constexpr int kCenterlineCapacity = VISION_DOWNSAMPLED_HEIGHT * 2;

struct infer_job_t
{
    // 异步任务输入：处理分辨率 BGR + full 分辨率 BGR + 二值图/中线快照。
    cv::Mat proc_bgr;
    cv::Mat full_bgr;
    int proc_width = 0;
    int proc_height = 0;
    int full_width = 0;
    int full_height = 0;
    int crop_x = 0;
    int crop_y = 0;
    int crop_w = 0;
    int crop_h = 0;
    uint8 binary_u8[VISION_DOWNSAMPLED_HEIGHT * VISION_DOWNSAMPLED_WIDTH] = {0};
    uint16 center_x[kCenterlineCapacity] = {0};
    uint16 center_y[kCenterlineCapacity] = {0};
    uint16 center_count = 0;
};

struct infer_worker_result_t
{
    // 异步任务输出：兼容红框信息 + ncnn ROI（full 分辨率坐标）。
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
    bool board_debug_valid = false;
    std::string board_fail_reason = "not_run";
    float board_dist_ipm = 0.0f;
    float board_target_bottom_gap_ipm = 0.0f;
    int board_bottom_cx = 0;
    int board_bottom_cy = 0;
    int board_hit_x = 0;
    int board_hit_y = 0;
    int board_height_px = 0;
    int board_width_px = 0;
    int board_corner_bl_x = 0;
    int board_corner_bl_y = 0;
    int board_corner_br_x = 0;
    int board_corner_br_y = 0;
    int board_corner_tr_x = 0;
    int board_corner_tr_y = 0;
    int board_corner_tl_x = 0;
    int board_corner_tl_y = 0;
    int board_ipm_bottom_x = 0;
    int board_ipm_bottom_y = 0;
    int board_ipm_top_x = 0;
    int board_ipm_top_y = 0;
    int board_ipm_red_bottom_x = 0;
    int board_ipm_red_bottom_y = 0;
    int board_ipm_bl_x = 0;
    int board_ipm_bl_y = 0;
    int board_ipm_br_x = 0;
    int board_ipm_br_y = 0;
    int board_ipm_tr_x = 0;
    int board_ipm_tr_y = 0;
    int board_ipm_tl_x = 0;
    int board_ipm_tl_y = 0;
    int board_src_bl_x = 0;
    int board_src_bl_y = 0;
    int board_src_br_x = 0;
    int board_src_br_y = 0;
    int board_src_tr_x = 0;
    int board_src_tr_y = 0;
    int board_src_tl_x = 0;
    int board_src_tl_y = 0;
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

static cv::Point2f proc_point_to_full_point(const infer_job_t &job, double proc_x, double proc_y)
{
    if (job.proc_width <= 0 || job.proc_height <= 0 || job.crop_w <= 0 || job.crop_h <= 0)
    {
        return cv::Point2f(0.0f, 0.0f);
    }
    const float full_x = static_cast<float>(job.crop_x + (proc_x * static_cast<double>(job.crop_w) / job.proc_width));
    const float full_y = static_cast<float>(job.crop_y + (proc_y * static_cast<double>(job.crop_h) / job.proc_height));
    return cv::Point2f(full_x, full_y);
}

static cv::Point2f proc_point_to_crop_local_point(const infer_job_t &job, double proc_x, double proc_y)
{
    const cv::Point2f full = proc_point_to_full_point(job, proc_x, proc_y);
    return cv::Point2f(full.x - static_cast<float>(job.crop_x),
                       full.y - static_cast<float>(job.crop_y));
}

static cv::Rect full_rect_from_proc_quad(const infer_job_t &job, const cv::Point2f quad[4])
{
    if (job.full_width <= 0 || job.full_height <= 0)
    {
        return cv::Rect();
    }
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
    for (int i = 0; i < 4; ++i)
    {
        const cv::Point2f full = proc_point_to_full_point(job, quad[i].x, quad[i].y);
        min_x = std::min(min_x, full.x);
        min_y = std::min(min_y, full.y);
        max_x = std::max(max_x, full.x);
        max_y = std::max(max_y, full.y);
    }
    const int x0 = static_cast<int>(std::floor(min_x));
    const int y0 = static_cast<int>(std::floor(min_y));
    const int x1 = static_cast<int>(std::ceil(max_x));
    const int y1 = static_cast<int>(std::ceil(max_y));
    return cv::Rect(x0, y0, std::max(0, x1 - x0), std::max(0, y1 - y0)) &
           cv::Rect(0, 0, job.full_width, job.full_height);
}

static void fill_compat_red_rect(infer_worker_result_t *result,
                                 int anchor_full_x,
                                 int anchor_full_y,
                                 int full_width,
                                 int full_height)
{
    if (result == nullptr || full_width <= 0 || full_height <= 0)
    {
        return;
    }
    const int side = 8;
    const int x0 = std::clamp(anchor_full_x - side / 2, 0, std::max(0, full_width - 1));
    const int y0 = std::clamp(anchor_full_y - side / 2, 0, std::max(0, full_height - 1));
    const int x1 = std::clamp(x0 + side, 0, full_width);
    const int y1 = std::clamp(y0 + side, 0, full_height);
    result->red_x = x0;
    result->red_y = y0;
    result->red_w = std::max(1, x1 - x0);
    result->red_h = std::max(1, y1 - y0);
    result->red_cx = x0 + result->red_w / 2;
    result->red_cy = y0 + result->red_h / 2;
    result->red_area = result->red_w * result->red_h;
}

static bool detect_bottom_edge_via_centerline(const infer_job_t &job,
                                              int *out_bottom_index,
                                              int *out_bottom_x,
                                              int *out_bottom_y)
{
    if (out_bottom_index == nullptr || out_bottom_x == nullptr || out_bottom_y == nullptr ||
        job.center_count == 0 || job.proc_width <= 0 || job.proc_height <= 0)
    {
        return false;
    }

    auto try_hit = [&](int index, int x, int y) -> bool {
        if (x < 0 || x >= job.proc_width || y < 0 || y >= job.proc_height)
        {
            return false;
        }
        if (job.binary_u8[y * job.proc_width + x] != 0)
        {
            return false;
        }
        *out_bottom_index = index;
        *out_bottom_x = x;
        *out_bottom_y = y;
        return true;
    };

    int prev_x = static_cast<int>(job.center_x[0]);
    int prev_y = static_cast<int>(job.center_y[0]);
    if (try_hit(0, prev_x, prev_y))
    {
        return true;
    }

    for (int i = 1; i < static_cast<int>(job.center_count); ++i)
    {
        const int curr_x = static_cast<int>(job.center_x[i]);
        const int curr_y = static_cast<int>(job.center_y[i]);
        int x = prev_x;
        int y = prev_y;
        const int dx = std::abs(curr_x - prev_x);
        const int dy = std::abs(curr_y - prev_y);
        const int sx = (prev_x < curr_x) ? 1 : -1;
        const int sy = (prev_y < curr_y) ? 1 : -1;
        int err = dx - dy;
        while (x != curr_x || y != curr_y)
        {
            const int e2 = 2 * err;
            if (e2 > -dy)
            {
                err -= dy;
                x += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y += sy;
            }
            if (try_hit(i, x, y))
            {
                return true;
            }
        }
        prev_x = curr_x;
        prev_y = curr_y;
    }
    return false;
}

static double accumulate_ipm_distance(const infer_job_t &job,
                                      int start_index,
                                      int end_index,
                                      int *out_ipm_x,
                                      int *out_ipm_y)
{
    double dist = 0.0;
    int prev_ipm_x = 0;
    int prev_ipm_y = 0;
    bool has_prev = false;
    const int safe_end = std::min(end_index, std::max(0, static_cast<int>(job.center_count) - 1));
    for (int i = start_index; i <= safe_end; ++i)
    {
        int ipm_x = 0;
        int ipm_y = 0;
        if (!vision_image_processor_src_to_ipm_point(static_cast<int>(job.center_x[i]),
                                                     static_cast<int>(job.center_y[i]),
                                                     &ipm_x,
                                                     &ipm_y))
        {
            continue;
        }
        if (has_prev)
        {
            const double dx = static_cast<double>(ipm_x - prev_ipm_x);
            const double dy = static_cast<double>(ipm_y - prev_ipm_y);
            dist += std::sqrt(dx * dx + dy * dy);
        }
        prev_ipm_x = ipm_x;
        prev_ipm_y = ipm_y;
        has_prev = true;
        if (i == safe_end && out_ipm_x != nullptr && out_ipm_y != nullptr)
        {
            *out_ipm_x = ipm_x;
            *out_ipm_y = ipm_y;
        }
    }
    return dist;
}

static bool walk_ipm_centerline_distance(const infer_job_t &job,
                                         int next_index,
                                         int start_ipm_x,
                                         int start_ipm_y,
                                         double distance_ipm,
                                         int *out_ipm_x,
                                         int *out_ipm_y,
                                         int *out_next_index)
{
    if (out_ipm_x == nullptr || out_ipm_y == nullptr || out_next_index == nullptr ||
        next_index < 0 || distance_ipm < 0.0)
    {
        return false;
    }
    if (distance_ipm <= 0.0)
    {
        *out_ipm_x = start_ipm_x;
        *out_ipm_y = start_ipm_y;
        *out_next_index = std::min(next_index, std::max(0, static_cast<int>(job.center_count) - 1));
        return true;
    }
    if (next_index >= static_cast<int>(job.center_count))
    {
        return false;
    }

    double walked = 0.0;
    int prev_x = start_ipm_x;
    int prev_y = start_ipm_y;
    int last_x = start_ipm_x;
    int last_y = start_ipm_y;
    int last_index = next_index;
    for (int i = next_index; i < static_cast<int>(job.center_count); ++i)
    {
        int ipm_x = 0;
        int ipm_y = 0;
        if (!vision_image_processor_src_to_ipm_point(static_cast<int>(job.center_x[i]),
                                                     static_cast<int>(job.center_y[i]),
                                                     &ipm_x,
                                                     &ipm_y))
        {
            continue;
        }
        const double dx = static_cast<double>(ipm_x - prev_x);
        const double dy = static_cast<double>(ipm_y - prev_y);
        const double seg = std::sqrt(dx * dx + dy * dy);
        if (walked + seg >= distance_ipm)
        {
            const double t = (seg > 0.0) ? ((distance_ipm - walked) / seg) : 0.0;
            *out_ipm_x = static_cast<int>(std::lround(prev_x + dx * t));
            *out_ipm_y = static_cast<int>(std::lround(prev_y + dy * t));
            *out_next_index = i;
            return true;
        }
        walked += seg;
        prev_x = ipm_x;
        prev_y = ipm_y;
        last_x = ipm_x;
        last_y = ipm_y;
        last_index = i + 1;
    }
    if (walked < distance_ipm * 0.5)
    {
        return false;
    }
    *out_ipm_x = last_x;
    *out_ipm_y = last_y;
    *out_next_index = std::min(last_index, std::max(0, static_cast<int>(job.center_count) - 1));
    return true;
}

static double resolve_target_bottom_gap_ipm(double dist_ipm)
{
    const double x1 = static_cast<double>(g_vision_runtime_config.red_roi_red_to_target_bottom_x1);
    const double y1 = static_cast<double>(g_vision_runtime_config.red_roi_red_to_target_bottom_y1);
    const double x2 = static_cast<double>(g_vision_runtime_config.red_roi_red_to_target_bottom_x2);
    const double y2 = static_cast<double>(g_vision_runtime_config.red_roi_red_to_target_bottom_y2);
    const double dx = x2 - x1;
    if (std::fabs(dx) < 1.0e-9)
    {
        return -1.0;
    }
    const double k = (y2 - y1) / dx;
    const double b = y1 - k * x1;
    const double dist_clamped = std::clamp(dist_ipm, std::min(x1, x2), std::max(x1, x2));
    return k * dist_clamped + b;
}

static bool detect_and_extract_target_board(const infer_job_t &job,
                                            infer_worker_result_t *result,
                                            cv::Mat *warped_roi_out)
{
    if (result == nullptr || warped_roi_out == nullptr)
    {
        return false;
    }
    result->board_fail_reason = "unknown";

    int bottom_index = -1;
    int bottom_src_x = 0;
    int bottom_src_y = 0;
    if (!detect_bottom_edge_via_centerline(job, &bottom_index, &bottom_src_x, &bottom_src_y))
    {
        result->board_fail_reason = (job.center_count == 0) ? "no_centerline" : "no_black_hit";
        return false;
    }

    int bottom_ipm_x = 0;
    int bottom_ipm_y = 0;
    const double dist_ipm = accumulate_ipm_distance(job,
                                                    0,
                                                    bottom_index,
                                                    &bottom_ipm_x,
                                                    &bottom_ipm_y);
    const double target_bottom_gap_ipm = resolve_target_bottom_gap_ipm(dist_ipm);
    if (target_bottom_gap_ipm < 0.0)
    {
        result->board_fail_reason = "target_bottom_gap_invalid";
        return false;
    }

    int target_bottom_ipm_x = bottom_ipm_x;
    int target_bottom_ipm_y = bottom_ipm_y;
    int target_bottom_next_index = bottom_index + 1;
    if (!walk_ipm_centerline_distance(job,
                                      bottom_index + 1,
                                      bottom_ipm_x,
                                      bottom_ipm_y,
                                      target_bottom_gap_ipm,
                                      &target_bottom_ipm_x,
                                      &target_bottom_ipm_y,
                                      &target_bottom_next_index))
    {
        result->board_fail_reason = "target_bottom_centerline_too_short";
        return false;
    }

    const double square_side_ipm = static_cast<double>(g_vision_runtime_config.red_roi_square_side_px);
    if (square_side_ipm <= 0.0)
    {
        result->board_fail_reason = "square_side_invalid";
        return false;
    }

    int top_anchor_ipm_x = target_bottom_ipm_x;
    int top_anchor_ipm_y = target_bottom_ipm_y;
    int top_walk_next_index = target_bottom_next_index;
    if (!walk_ipm_centerline_distance(job,
                                      target_bottom_next_index,
                                      target_bottom_ipm_x,
                                      target_bottom_ipm_y,
                                      square_side_ipm,
                                      &top_anchor_ipm_x,
                                      &top_anchor_ipm_y,
                                      &top_walk_next_index))
    {
        result->board_fail_reason = "target_top_centerline_too_short";
        return false;
    }

    const double dx_ipm = static_cast<double>(top_anchor_ipm_x - target_bottom_ipm_x);
    const double dy_ipm = static_cast<double>(top_anchor_ipm_y - target_bottom_ipm_y);
    const double axis_len = std::sqrt(dx_ipm * dx_ipm + dy_ipm * dy_ipm);
    if (axis_len < 1.0)
    {
        result->board_fail_reason = "height_too_small";
        return false;
    }

    const double ux = dx_ipm / axis_len;
    const double uy = dy_ipm / axis_len;
    const double nx = -uy;
    const double ny = ux;
    const double half_side = square_side_ipm * 0.5;
    const double top_ipm_x = static_cast<double>(target_bottom_ipm_x) + ux * square_side_ipm;
    const double top_ipm_y = static_cast<double>(target_bottom_ipm_y) + uy * square_side_ipm;
    struct pointd_t { double x; double y; };
    const pointd_t ipm_corners[4] = {
        {static_cast<double>(target_bottom_ipm_x) - nx * half_side, static_cast<double>(target_bottom_ipm_y) - ny * half_side},
        {static_cast<double>(target_bottom_ipm_x) + nx * half_side, static_cast<double>(target_bottom_ipm_y) + ny * half_side},
        {top_ipm_x + nx * half_side, top_ipm_y + ny * half_side},
        {top_ipm_x - nx * half_side, top_ipm_y - ny * half_side},
    };

    cv::Point2f src_quad_proc[4];
    cv::Point2f src_quad_crop_local[4];
    int src_corners_x[4] = {0};
    int src_corners_y[4] = {0};
    for (int i = 0; i < 4; ++i)
    {
        int src_x = 0;
        int src_y = 0;
        if (!vision_image_processor_ipm_to_src_point(static_cast<int>(std::lround(ipm_corners[i].x)),
                                                     static_cast<int>(std::lround(ipm_corners[i].y)),
                                                     &src_x,
                                                     &src_y))
        {
            result->board_fail_reason = "corner_project_failed";
            return false;
        }
        src_corners_x[i] = src_x;
        src_corners_y[i] = src_y;
        src_quad_proc[i] = cv::Point2f(static_cast<float>(src_x), static_cast<float>(src_y));
        src_quad_crop_local[i] = proc_point_to_crop_local_point(job,
                                                                static_cast<double>(src_x),
                                                                static_cast<double>(src_y));
    }

    const cv::Rect roi_full = full_rect_from_proc_quad(job, src_quad_proc);
    if (roi_full.width <= 0 || roi_full.height <= 0)
    {
        result->board_fail_reason = "roi_full_invalid";
        return false;
    }

    const cv::Point2f dst_quad[4] = {
        cv::Point2f(0.0f, static_cast<float>(kBoardRoiSize)),
        cv::Point2f(static_cast<float>(kBoardRoiSize), static_cast<float>(kBoardRoiSize)),
        cv::Point2f(static_cast<float>(kBoardRoiSize), 0.0f),
        cv::Point2f(0.0f, 0.0f),
    };
    cv::Mat warp_source;
    if (!job.full_bgr.empty() &&
        job.crop_w > 0 && job.crop_h > 0 &&
        job.crop_x >= 0 && job.crop_y >= 0 &&
        job.crop_x + job.crop_w <= job.full_bgr.cols &&
        job.crop_y + job.crop_h <= job.full_bgr.rows)
    {
        warp_source = job.full_bgr(cv::Rect(job.crop_x, job.crop_y, job.crop_w, job.crop_h));
    }
    else
    {
        warp_source = job.proc_bgr;
        for (int i = 0; i < 4; ++i)
        {
            src_quad_crop_local[i] = src_quad_proc[i];
        }
    }
    const cv::Mat perspective = cv::getPerspectiveTransform(src_quad_crop_local, dst_quad);
    cv::warpPerspective(warp_source,
                        *warped_roi_out,
                        perspective,
                        cv::Size(kBoardRoiSize, kBoardRoiSize),
                        cv::INTER_LINEAR,
                        cv::BORDER_CONSTANT,
                        cv::Scalar(0, 0, 0));
    if (warped_roi_out->empty())
    {
        result->board_fail_reason = "warp_failed";
        return false;
    }

    result->found = true;
    result->ncnn_roi_full = roi_full;
    result->board_debug_valid = true;
    result->board_fail_reason = "ok";
    result->board_dist_ipm = static_cast<float>(dist_ipm);
    result->board_target_bottom_gap_ipm = static_cast<float>(target_bottom_gap_ipm);
    result->board_hit_x = bottom_src_x;
    result->board_hit_y = bottom_src_y;
    int target_bottom_src_x = 0;
    int target_bottom_src_y = 0;
    if (vision_image_processor_ipm_to_src_point(target_bottom_ipm_x,
                                                target_bottom_ipm_y,
                                                &target_bottom_src_x,
                                                &target_bottom_src_y))
    {
        result->board_bottom_cx = target_bottom_src_x;
        result->board_bottom_cy = target_bottom_src_y;
    }
    const int top_src_x = static_cast<int>(std::lround((src_corners_x[2] + src_corners_x[3]) * 0.5));
    const int top_src_y = static_cast<int>(std::lround((src_corners_y[2] + src_corners_y[3]) * 0.5));
    const double hdx = static_cast<double>(top_src_x - result->board_bottom_cx);
    const double hdy = static_cast<double>(top_src_y - result->board_bottom_cy);
    const double wdx = static_cast<double>(src_corners_x[1] - src_corners_x[0]);
    const double wdy = static_cast<double>(src_corners_y[1] - src_corners_y[0]);
    result->board_height_px = static_cast<int>(std::lround(std::sqrt(hdx * hdx + hdy * hdy)));
    result->board_width_px = static_cast<int>(std::lround(std::sqrt(wdx * wdx + wdy * wdy)));
    result->board_corner_bl_x = src_corners_x[0];
    result->board_corner_bl_y = src_corners_y[0];
    result->board_corner_br_x = src_corners_x[1];
    result->board_corner_br_y = src_corners_y[1];
    result->board_corner_tr_x = src_corners_x[2];
    result->board_corner_tr_y = src_corners_y[2];
    result->board_corner_tl_x = src_corners_x[3];
    result->board_corner_tl_y = src_corners_y[3];
    result->board_ipm_red_bottom_x = bottom_ipm_x;
    result->board_ipm_red_bottom_y = bottom_ipm_y;
    result->board_ipm_bottom_x = target_bottom_ipm_x;
    result->board_ipm_bottom_y = target_bottom_ipm_y;
    result->board_ipm_top_x = static_cast<int>(std::lround(top_ipm_x));
    result->board_ipm_top_y = static_cast<int>(std::lround(top_ipm_y));
    result->board_ipm_bl_x = static_cast<int>(std::lround(ipm_corners[0].x));
    result->board_ipm_bl_y = static_cast<int>(std::lround(ipm_corners[0].y));
    result->board_ipm_br_x = static_cast<int>(std::lround(ipm_corners[1].x));
    result->board_ipm_br_y = static_cast<int>(std::lround(ipm_corners[1].y));
    result->board_ipm_tr_x = static_cast<int>(std::lround(ipm_corners[2].x));
    result->board_ipm_tr_y = static_cast<int>(std::lround(ipm_corners[2].y));
    result->board_ipm_tl_x = static_cast<int>(std::lround(ipm_corners[3].x));
    result->board_ipm_tl_y = static_cast<int>(std::lround(ipm_corners[3].y));
    result->board_src_bl_x = src_corners_x[0];
    result->board_src_bl_y = src_corners_y[0];
    result->board_src_br_x = src_corners_x[1];
    result->board_src_br_y = src_corners_y[1];
    result->board_src_tr_x = src_corners_x[2];
    result->board_src_tr_y = src_corners_y[2];
    result->board_src_tl_x = src_corners_x[3];
    result->board_src_tl_y = src_corners_y[3];

    const cv::Point2f hit_full = proc_point_to_full_point(job,
                                                          static_cast<double>(bottom_src_x),
                                                          static_cast<double>(bottom_src_y));
    fill_compat_red_rect(result,
                         static_cast<int>(std::lround(hit_full.x)),
                         static_cast<int>(std::lround(hit_full.y)),
                         job.full_width,
                         job.full_height);
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

#ifdef VISION_ENABLE_NCNN
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
#endif

static void reset_infer_shared_state()
{
    std::lock_guard<std::mutex> lock(g_infer_mutex);
    g_infer_job.proc_bgr.release();
    g_infer_job.full_bgr.release();
    g_infer_job = infer_job_t{};
    g_infer_job_ready = false;
    g_latest_infer_result = infer_worker_result_t{};
    g_latest_infer_result_valid = false;
    g_latest_infer_result_seq = 0;
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
            job = g_infer_job;
            g_infer_job_ready = false;
        }

        infer_worker_result_t result{};
        result.ncnn_enabled = g_ncnn_enabled.load();
        const auto detect_start = std::chrono::steady_clock::now();
        cv::Mat warped_roi;
        const bool found = detect_and_extract_target_board(job, &result, &warped_roi);
        const auto detect_end = std::chrono::steady_clock::now();
        result.red_detect_us = static_cast<uint32>(
            std::chrono::duration_cast<std::chrono::microseconds>(detect_end - detect_start).count());

        if (found && g_ncnn_enabled.load() && !warped_roi.empty())
        {
            result.ncnn_infer_valid = ncnn_step(reinterpret_cast<const uint8 *>(warped_roi.data),
                                                warped_roi.cols,
                                                warped_roi.rows,
                                                &result.ncnn_top_class_id,
                                                &result.ncnn_top_score,
                                                &result.ncnn_top_label,
                                                &result.ncnn_labels,
                                                &result.ncnn_probs,
                                                &result.ncnn_infer_us);
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

    ncnn::Mat input = ncnn::Mat::from_pixels(rgb.data,
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

    if (top_class_id) *top_class_id = class_id;
    if (top_score) *top_score = (class_id >= 0 && class_id < static_cast<int>(local_probs.size())) ? local_probs[class_id] : 0.0f;
    if (top_label)
    {
        *top_label = (class_id >= 0 && class_id < static_cast<int>(local_labels.size()))
                         ? local_labels[class_id]
                         : std::to_string(class_id);
    }
    if (labels) *labels = local_labels;
    if (probs) *probs = local_probs;
    if (infer_us)
    {
        *infer_us = static_cast<uint32>(
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count());
    }
    return true;
#else
    (void)bgr_image;
    if (top_class_id) *top_class_id = -1;
    if (top_score) *top_score = 0.0f;
    if (top_label) top_label->clear();
    if (labels) labels->clear();
    if (probs) probs->clear();
    if (infer_us) *infer_us = 0;
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
    // 宽度与 full 尺寸仍需匹配；处理图高度允许在预裁剪后动态变化。
    if (proc_width != kProcWidth || proc_height <= 0 || proc_height > kProcHeight ||
        full_width != kFullWidth || full_height != kFullHeight)
    {
        return;
    }
    const uint8 *binary_data = vision_image_processor_binary_downsampled_u8_image();
    if (binary_data == nullptr)
    {
        return;
    }

    int crop_x = 0;
    int crop_y = 0;
    int crop_w = full_width;
    int crop_h = full_height;
    vision_image_processor_get_full_crop_rect(&crop_x, &crop_y, &crop_w, &crop_h);

    uint16 *selected_center_x = nullptr;
    uint16 *selected_center_y = nullptr;
    uint16 selected_center_num = 0;
    vision_image_processor_get_src_infer_centerline(&selected_center_x, &selected_center_y, &selected_center_num);

    cv::Mat proc_frame(proc_height, kProcWidth, CV_8UC3, const_cast<uint8 *>(bgr_proc_data));
    {
        std::lock_guard<std::mutex> lock(g_infer_mutex);
        g_infer_job = infer_job_t{};
        g_infer_job.proc_bgr = proc_frame.clone();
        g_infer_job.proc_width = proc_width;
        g_infer_job.proc_height = proc_height;
        g_infer_job.full_width = full_width;
        g_infer_job.full_height = full_height;
        g_infer_job.crop_x = crop_x;
        g_infer_job.crop_y = crop_y;
        g_infer_job.crop_w = crop_w;
        g_infer_job.crop_h = crop_h;
        if (bgr_full_data != nullptr)
        {
            cv::Mat full_frame(kFullHeight, kFullWidth, CV_8UC3, const_cast<uint8 *>(bgr_full_data));
            g_infer_job.full_bgr = full_frame.clone();
        }
        else
        {
            g_infer_job.full_bgr.release();
        }
        std::memcpy(g_infer_job.binary_u8,
                    binary_data,
                    static_cast<size_t>(proc_width * proc_height));
        g_infer_job.center_count = static_cast<uint16>(std::min<int>(selected_center_num, kCenterlineCapacity));
        if (g_infer_job.center_count > 0 && selected_center_x != nullptr && selected_center_y != nullptr)
        {
            std::memcpy(g_infer_job.center_x,
                        selected_center_x,
                        static_cast<size_t>(g_infer_job.center_count) * sizeof(uint16));
            std::memcpy(g_infer_job.center_y,
                        selected_center_y,
                        static_cast<size_t>(g_infer_job.center_count) * sizeof(uint16));
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
    out->board_debug_valid = result.board_debug_valid;
    out->board_dist_ipm = result.board_dist_ipm;
    out->board_target_bottom_gap_ipm = result.board_target_bottom_gap_ipm;
    out->board_bottom_cx = result.board_bottom_cx;
    out->board_bottom_cy = result.board_bottom_cy;
    out->board_hit_x = result.board_hit_x;
    out->board_hit_y = result.board_hit_y;
    out->board_height_px = result.board_height_px;
    out->board_width_px = result.board_width_px;
    out->board_corner_bl_x = result.board_corner_bl_x;
    out->board_corner_bl_y = result.board_corner_bl_y;
    out->board_corner_br_x = result.board_corner_br_x;
    out->board_corner_br_y = result.board_corner_br_y;
    out->board_corner_tr_x = result.board_corner_tr_x;
    out->board_corner_tr_y = result.board_corner_tr_y;
    out->board_corner_tl_x = result.board_corner_tl_x;
    out->board_corner_tl_y = result.board_corner_tl_y;
    out->board_ipm_bottom_x = result.board_ipm_bottom_x;
    out->board_ipm_bottom_y = result.board_ipm_bottom_y;
    out->board_ipm_top_x = result.board_ipm_top_x;
    out->board_ipm_top_y = result.board_ipm_top_y;
    out->board_ipm_red_bottom_x = result.board_ipm_red_bottom_x;
    out->board_ipm_red_bottom_y = result.board_ipm_red_bottom_y;
    out->board_ipm_bl_x = result.board_ipm_bl_x;
    out->board_ipm_bl_y = result.board_ipm_bl_y;
    out->board_ipm_br_x = result.board_ipm_br_x;
    out->board_ipm_br_y = result.board_ipm_br_y;
    out->board_ipm_tr_x = result.board_ipm_tr_x;
    out->board_ipm_tr_y = result.board_ipm_tr_y;
    out->board_ipm_tl_x = result.board_ipm_tl_x;
    out->board_ipm_tl_y = result.board_ipm_tl_y;
    out->board_src_bl_x = result.board_src_bl_x;
    out->board_src_bl_y = result.board_src_bl_y;
    out->board_src_br_x = result.board_src_br_x;
    out->board_src_br_y = result.board_src_br_y;
    out->board_src_tr_x = result.board_src_tr_x;
    out->board_src_tr_y = result.board_src_tr_y;
    out->board_src_tl_x = result.board_src_tl_x;
    out->board_src_tl_y = result.board_src_tl_y;
    std::memset(out->board_fail_reason, 0, sizeof(out->board_fail_reason));
    std::snprintf(out->board_fail_reason, sizeof(out->board_fail_reason), "%s", result.board_fail_reason.c_str());
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
