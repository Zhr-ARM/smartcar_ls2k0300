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
    // 二值图快照（160×60 uint8，每帧深拷贝）。
    uint8 binary_u8[VISION_DOWNSAMPLED_HEIGHT * VISION_DOWNSAMPLED_WIDTH];
    // src 中线快照（每帧深拷贝）。
    uint16 center_x[VISION_DOWNSAMPLED_HEIGHT * 2];
    uint16 center_y[VISION_DOWNSAMPLED_HEIGHT * 2];
    uint16 center_count;
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
    // 目标板检测调试信息（crop 坐标系）。
    bool board_debug_valid = false;
    std::string board_fail_reason = "not_run";
    float board_dist_ipm = 0.0f;
    int board_bottom_cx = 0;
    int board_bottom_cy = 0;
    // 下底边命中点（proc 坐标系，供网页端二值图中线标注）。
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
    // IPM 坐标系：上下底边中点 + 四角点。
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
    // 回投原图角点（proc 坐标系，供灰度图标注）。
    int board_src_bl_x = 0;
    int board_src_bl_y = 0;
    int board_src_br_x = 0;
    int board_src_br_y = 0;
    int board_src_tr_x = 0;
    int board_src_tr_y = 0;
    int board_src_tl_x = 0;
    int board_src_tl_y = 0;
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

// 作用：沿 src 中线搜索，在二值图中找到第一个黑色像素点，作为目标板下底边中点。
// 对相邻中线点之间的连线逐像素采样，每个采样像素仍是单点黑白判定。
static bool detect_bottom_edge_via_centerline(const uint16 *center_x,
                                              const uint16 *center_y,
                                              uint16 center_count,
                                              const uint8 *binary_u8,
                                              int *out_bottom_index,
                                              int *out_bottom_x,
                                              int *out_bottom_y)
{
    if (center_x == nullptr || center_y == nullptr || binary_u8 == nullptr ||
        out_bottom_index == nullptr || out_bottom_x == nullptr || out_bottom_y == nullptr ||
        center_count == 0)
    {
        return false;
    }

    constexpr int kSearchStartIndex = 0;

    auto try_hit = [&](int index, int x, int y) -> bool {
        if (x < 0 || x >= kProcWidth || y < 0 || y >= kProcHeight)
        {
            return false;
        }

        if (binary_u8[y * kProcWidth + x] != 0)
        {
            return false;
        }

        *out_bottom_index = index;
        *out_bottom_x = x;
        *out_bottom_y = y;
        return true;
    };

    int prev_x = static_cast<int>(center_x[kSearchStartIndex]);
    int prev_y = static_cast<int>(center_y[kSearchStartIndex]);
    if (try_hit(kSearchStartIndex, prev_x, prev_y))
    {
        return true;
    }

    for (int i = kSearchStartIndex + 1; i < static_cast<int>(center_count); ++i)
    {
        const int curr_x = static_cast<int>(center_x[i]);
        const int curr_y = static_cast<int>(center_y[i]);

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

static void current_crop_geometry(int *crop_y, int *crop_w, int *crop_h)
{
    const int d = std::max(1, g_vision_processor_config.crop_denominator);
    const int top = std::clamp(g_vision_processor_config.crop_top, 0, d - 1);
    const int bottom = std::clamp(g_vision_processor_config.crop_bottom, top + 1, d);

    if (crop_y != nullptr)
    {
        *crop_y = (kFullHeight * top) / d;
    }
    if (crop_w != nullptr)
    {
        *crop_w = kFullWidth;
    }
    if (crop_h != nullptr)
    {
        *crop_h = std::max(1, (kFullHeight * (bottom - top)) / d);
    }
}

// 作用：proc 坐标 → crop 相对坐标换算（无 full 图 y 偏移）。
// crop 图尺寸来自 vision.processor.crop，与主处理链保持一致。
static void proc_to_crop(int proc_x, int proc_y, int *crop_x, int *crop_y)
{
    int crop_w = 0;
    int crop_h = 0;
    current_crop_geometry(nullptr, &crop_w, &crop_h);
    *crop_x = proc_x * crop_w / kProcWidth;
    *crop_y = proc_y * crop_h / kProcHeight;
}

// 作用：推进中线 IPM 累计距离，返回从 start_index 到 end_index 的累计 IPM 欧氏距离。
// 同时将 end_index 处的 IPM 坐标写入 out_ipm_x/out_ipm_y。
static double accumulate_ipm_distance(const uint16 *center_x,
                                      const uint16 *center_y,
                                      uint16 center_count,
                                      int start_index,
                                      int end_index,
                                      int *out_ipm_x,
                                      int *out_ipm_y)
{
    double dist = 0.0;
    int prev_ipm_x = 0;
    int prev_ipm_y = 0;
    bool has_prev = false;

    const int safe_end = std::min(static_cast<int>(center_count) - 1, end_index);
    for (int i = start_index; i <= safe_end; ++i)
    {
        int ipm_x = 0;
        int ipm_y = 0;
        if (!vision_image_processor_src_to_ipm_point(static_cast<int>(center_x[i]),
                                                     static_cast<int>(center_y[i]),
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
            const double remaining = distance_ipm - walked;
            const double t = (seg > 0.0) ? (remaining / seg) : 0.0;
            *out_ipm_x = static_cast<int>(std::lround(static_cast<double>(prev_x) + dx * t));
            *out_ipm_y = static_cast<int>(std::lround(static_cast<double>(prev_y) + dy * t));
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
    *out_next_index = std::min(last_index, static_cast<int>(job.center_count) - 1);
    return true;
}

// 作用：沿中线搜索 + IPM 空间推算 + 四角点回投原图 + perspective warp 提取 ROI。
// 返回 true 表示成功检测到目标板并提取了 ROI。
static bool detect_and_extract_target_board(const infer_job_t &job,
                                            infer_worker_result_t *result,
                                            cv::Mat *warped_roi_out)
{
    if (result == nullptr || warped_roi_out == nullptr)
    {
        return false;
    }
    result->board_fail_reason = "unknown";

    // ---- 步骤 1：沿 src 中线在二值图中搜索下底边中点 ----
    int bottom_index = -1;
    int bottom_src_x = 0;
    int bottom_src_y = 0;
    if (!detect_bottom_edge_via_centerline(job.center_x,
                                           job.center_y,
                                           job.center_count,
                                           job.binary_u8,
                                           &bottom_index,
                                           &bottom_src_x,
                                           &bottom_src_y))
    {
        result->board_fail_reason = (job.center_count == 0) ? "no_centerline" : "no_black_hit";
        return false;
    }

    // ---- 步骤 2：src 中线 index 0→bottom_index 转 IPM，累计距离 ----
    int bottom_ipm_x = 0;
    int bottom_ipm_y = 0;
    const double dist_ipm = accumulate_ipm_distance(job.center_x,
                                                    job.center_y,
                                                    job.center_count,
                                                    0,
                                                    bottom_index,
                                                    &bottom_ipm_x,
                                                    &bottom_ipm_y);

    // ---- 步骤 3：在 IPM 空间推算目标物下底边中点 ----
    const double target_bottom_gap_ipm =
        static_cast<double>(g_vision_runtime_config.red_roi_red_to_target_bottom_k) * dist_ipm +
        static_cast<double>(g_vision_runtime_config.red_roi_red_to_target_bottom_b);
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

    // ---- 步骤 4：在 IPM 空间推算目标物上底边中点 ----
    const double target_height_ipm =
        static_cast<double>(g_vision_runtime_config.red_roi_target_height_k) * dist_ipm +
        static_cast<double>(g_vision_runtime_config.red_roi_target_height_b);
    if (target_height_ipm <= 0.0)
    {
        result->board_fail_reason = "target_height_invalid";
        return false;
    }

    int top_ipm_x = target_bottom_ipm_x;
    int top_ipm_y = target_bottom_ipm_y;
    int top_next_index = target_bottom_next_index;
    if (!walk_ipm_centerline_distance(job,
                                      target_bottom_next_index,
                                      target_bottom_ipm_x,
                                      target_bottom_ipm_y,
                                      target_height_ipm,
                                      &top_ipm_x,
                                      &top_ipm_y,
                                      &top_next_index))
    {
        result->board_fail_reason = "target_top_centerline_too_short";
        return false;
    }
    (void)top_next_index;

    // ---- 步骤 5：在 IPM 空间计算目标物正方形四个角点 ----
    const double dx_ipm = static_cast<double>(top_ipm_x - target_bottom_ipm_x);
    const double dy_ipm = static_cast<double>(top_ipm_y - target_bottom_ipm_y);
    const double H_ipm = std::sqrt(dx_ipm * dx_ipm + dy_ipm * dy_ipm);
    if (H_ipm < 1.0)
    {
        result->board_fail_reason = "height_too_small";
        return false;
    }
    const double W_ipm = H_ipm;
    const double px = -dy_ipm / H_ipm * (W_ipm * 0.5);
    const double py = dx_ipm / H_ipm * (W_ipm * 0.5);

    struct { double x; double y; } ipm_corners[4] = {
        {static_cast<double>(target_bottom_ipm_x) - px, static_cast<double>(target_bottom_ipm_y) - py}, // bl
        {static_cast<double>(target_bottom_ipm_x) + px, static_cast<double>(target_bottom_ipm_y) + py}, // br
        {static_cast<double>(top_ipm_x)    + px, static_cast<double>(top_ipm_y)    + py}, // tr
        {static_cast<double>(top_ipm_x)    - px, static_cast<double>(top_ipm_y)    - py}, // tl
    };

    // ---- 步骤 6：四角点回投 crop 图（320×crop_h 或 160×crop_h）----
    int crop_y_offset = 0;
    int crop_w = 0;
    int crop_h = 0;
    current_crop_geometry(&crop_y_offset, &crop_w, &crop_h);

    cv::Point2f src_quad[4];
    for (int i = 0; i < 4; ++i)
    {
        int src_x = 0;
        int src_y = 0;
        if (!vision_image_processor_ipm_to_src_point(
                static_cast<int>(std::lround(ipm_corners[i].x)),
                static_cast<int>(std::lround(ipm_corners[i].y)),
                &src_x,
                &src_y))
        {
            result->board_fail_reason = "corner_project_failed";
            return false;
        }
        int cx = 0;
        int cy = 0;
        proc_to_crop(src_x, src_y, &cx, &cy);
        src_quad[i] = cv::Point2f(static_cast<float>(cx), static_cast<float>(cy));

    }
    result->found = true;

    // ---- 填充调试信息 ----
    {
        // 下底边命中点（proc 坐标）。
        result->board_hit_x = bottom_src_x;
        result->board_hit_y = bottom_src_y;

        // 目标物下底边中点（crop 坐标）。
        int bottom_cx = 0;
        int bottom_cy = 0;
        int target_bottom_src_x = 0;
        int target_bottom_src_y = 0;
        if (vision_image_processor_ipm_to_src_point(target_bottom_ipm_x,
                                                    target_bottom_ipm_y,
                                                    &target_bottom_src_x,
                                                    &target_bottom_src_y))
        {
            proc_to_crop(target_bottom_src_x, target_bottom_src_y, &bottom_cx, &bottom_cy);
        }
        result->board_bottom_cx = bottom_cx;
        result->board_bottom_cy = bottom_cy;

        // 上底边中点（crop 坐标）。
        int top_src_x = 0;
        int top_src_y = 0;
        if (vision_image_processor_ipm_to_src_point(top_ipm_x, top_ipm_y, &top_src_x, &top_src_y))
        {
            int top_cx = 0;
            int top_cy = 0;
            proc_to_crop(top_src_x, top_src_y, &top_cx, &top_cy);
            const double dh = static_cast<double>(top_cy - bottom_cy);
            const double dw = static_cast<double>(top_cx - bottom_cx);
            result->board_height_px = static_cast<int>(std::lround(std::sqrt(dh * dh + dw * dw)));
        }
        result->board_width_px = result->board_height_px;

        // 四个角点（crop 坐标，顺序 bl / br / tr / tl）。
        result->board_corner_bl_x = static_cast<int>(std::lround(src_quad[0].x));
        result->board_corner_bl_y = static_cast<int>(std::lround(src_quad[0].y));
        result->board_corner_br_x = static_cast<int>(std::lround(src_quad[1].x));
        result->board_corner_br_y = static_cast<int>(std::lround(src_quad[1].y));
        result->board_corner_tr_x = static_cast<int>(std::lround(src_quad[2].x));
        result->board_corner_tr_y = static_cast<int>(std::lround(src_quad[2].y));
        result->board_corner_tl_x = static_cast<int>(std::lround(src_quad[3].x));
        result->board_corner_tl_y = static_cast<int>(std::lround(src_quad[3].y));

        // IPM 坐标系：上下底边中点。
        result->board_dist_ipm = static_cast<float>(dist_ipm);
        result->board_ipm_red_bottom_x = bottom_ipm_x;
        result->board_ipm_red_bottom_y = bottom_ipm_y;
        result->board_ipm_bottom_x = target_bottom_ipm_x;
        result->board_ipm_bottom_y = target_bottom_ipm_y;
        result->board_ipm_top_x = top_ipm_x;
        result->board_ipm_top_y = top_ipm_y;

        // IPM 坐标系：四角点（顺序 bl / br / tr / tl）。
        result->board_ipm_bl_x = static_cast<int>(std::lround(ipm_corners[0].x));
        result->board_ipm_bl_y = static_cast<int>(std::lround(ipm_corners[0].y));
        result->board_ipm_br_x = static_cast<int>(std::lround(ipm_corners[1].x));
        result->board_ipm_br_y = static_cast<int>(std::lround(ipm_corners[1].y));
        result->board_ipm_tr_x = static_cast<int>(std::lround(ipm_corners[2].x));
        result->board_ipm_tr_y = static_cast<int>(std::lround(ipm_corners[2].y));
        result->board_ipm_tl_x = static_cast<int>(std::lround(ipm_corners[3].x));
        result->board_ipm_tl_y = static_cast<int>(std::lround(ipm_corners[3].y));

        // 回投原图角点（proc 坐标系，供灰度图标注，顺序 bl / br / tr / tl）。
        {
            int sx = 0, sy = 0;
            if (vision_image_processor_ipm_to_src_point(
                    static_cast<int>(std::lround(ipm_corners[0].x)),
                    static_cast<int>(std::lround(ipm_corners[0].y)), &sx, &sy))
            { result->board_src_bl_x = sx; result->board_src_bl_y = sy; }
            if (vision_image_processor_ipm_to_src_point(
                    static_cast<int>(std::lround(ipm_corners[1].x)),
                    static_cast<int>(std::lround(ipm_corners[1].y)), &sx, &sy))
            { result->board_src_br_x = sx; result->board_src_br_y = sy; }
            if (vision_image_processor_ipm_to_src_point(
                    static_cast<int>(std::lround(ipm_corners[2].x)),
                    static_cast<int>(std::lround(ipm_corners[2].y)), &sx, &sy))
            { result->board_src_tr_x = sx; result->board_src_tr_y = sy; }
            if (vision_image_processor_ipm_to_src_point(
                    static_cast<int>(std::lround(ipm_corners[3].x)),
                    static_cast<int>(std::lround(ipm_corners[3].y)), &sx, &sy))
            { result->board_src_tl_x = sx; result->board_src_tl_y = sy; }
        }

        result->board_debug_valid = true;
        result->board_fail_reason = "ok";
    }

    // ---- 步骤 6：裁剪 crop 图 + warpPerspective → 64×64 ----
    const cv::Point2f dst_quad[4] = {
        cv::Point2f(0.0f,           64.0f),          // bl → dst bottom-left
        cv::Point2f(64.0f,          64.0f),          // br → dst bottom-right
        cv::Point2f(64.0f,          0.0f),           // tr → dst top-right
        cv::Point2f(0.0f,           0.0f),           // tl → dst top-left
    };
    cv::Mat M = cv::getPerspectiveTransform(src_quad, dst_quad);

    // 从 full 图裁出与主处理链一致的 crop 区域作为 warp 源图。
    cv::Mat crop_bgr;
    if (!job.full_bgr.empty())
    {
        crop_bgr = job.full_bgr(cv::Rect(0, crop_y_offset, crop_w, crop_h));
    }
    else
    {
        crop_bgr = job.proc_bgr;
    }
    cv::warpPerspective(crop_bgr, *warped_roi_out, M, cv::Size(64, 64),
                        cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

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
            std::memcpy(job.binary_u8, g_infer_job.binary_u8, sizeof(job.binary_u8));
            job.center_count = g_infer_job.center_count;
            std::memcpy(job.center_x, g_infer_job.center_x, job.center_count * sizeof(uint16));
            std::memcpy(job.center_y, g_infer_job.center_y, job.center_count * sizeof(uint16));
            g_infer_job_ready = false;
        }

        infer_worker_result_t result{};
        result.ncnn_enabled = g_ncnn_enabled.load();
        auto detect_start = std::chrono::steady_clock::now();

        cv::Mat warped_roi;
        const bool found = detect_and_extract_target_board(job, &result, &warped_roi);

        auto detect_end = std::chrono::steady_clock::now();
        result.red_detect_us = static_cast<uint32>(
            std::chrono::duration_cast<std::chrono::microseconds>(detect_end - detect_start).count());

        // 将 warp 后的 64×64 BGR 图像写入 image_processor，供网页 ROI 显示。
        if (found && !warped_roi.empty())
        {
            vision_image_processor_set_warp_roi(true, warped_roi.data);
        }
        else
        {
            vision_image_processor_set_warp_roi(false, nullptr);
        }

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

        // 快照二值图（160×60 uint8）。
        const uint8 *binary = vision_image_processor_binary_downsampled_u8_image();
        if (binary != nullptr)
        {
            std::memcpy(g_infer_job.binary_u8, binary, sizeof(g_infer_job.binary_u8));
        }
        else
        {
            std::memset(g_infer_job.binary_u8, 255, sizeof(g_infer_job.binary_u8));
        }

        // 快照 src 中线（与 line_error / 网页显示一致的当前选中版本）。
        {
            uint16 *cx = nullptr;
            uint16 *cy = nullptr;
            uint16 count = 0;
            if (vision_image_processor_ipm_line_error_source() == VISION_IPM_LINE_ERROR_FROM_RIGHT_SHIFT)
            {
                vision_image_processor_get_src_shifted_centerline_from_right(&cx, &cy, &count);
            }
            else
            {
                vision_image_processor_get_src_shifted_centerline_from_left(&cx, &cy, &count);
            }
            if (cx != nullptr && cy != nullptr && count > 0)
            {
                const uint16 safe_count = std::min<uint16>(count, static_cast<uint16>(sizeof(g_infer_job.center_x) / sizeof(uint16)));
                g_infer_job.center_count = safe_count;
                std::memcpy(g_infer_job.center_x, cx, safe_count * sizeof(uint16));
                std::memcpy(g_infer_job.center_y, cy, safe_count * sizeof(uint16));
            }
            else
            {
                g_infer_job.center_count = 0;
            }
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
    // 目标板检测调试信息。
    out->board_debug_valid = result.board_debug_valid;
    std::memset(out->board_fail_reason, 0, sizeof(out->board_fail_reason));
    std::snprintf(out->board_fail_reason, sizeof(out->board_fail_reason), "%s", result.board_fail_reason.c_str());
    out->board_dist_ipm = result.board_dist_ipm;
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
    return true;
}
