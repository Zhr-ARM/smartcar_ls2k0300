#include "driver/vision/vision_image_processor.h"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

// 当前阶段目标：
// 1) 直接采集 160x120 BGR 图像（不翻转）；
// 2) 采集与处理解耦为两个线程；
// 3) 使用全图 OTSU 二值化；
// 4) 在图像下方区域，基于迷宫法提取左右边线；
// 5) 左/右边线经逆透视后，按单边偏移方式生成中线。

#define VISION_BOUNDARY_NUM (VISION_DOWNSAMPLED_HEIGHT * 2)
static constexpr int MAZE_LOWER_REGION_PERCENT = 90;
// 临时性能开关：
// true  : 按需OTSU（迷宫法按灰度+阈值即取即判，不依赖全图二值图）
// false : 传统全图OTSU二值化后再迷宫法
static constexpr bool kTempDemandOtsuEnable = true;
// 按需OTSU时，是否仍然额外生成完整二值图缓存（给二值图发送/调试用）。
// 关闭后性能更好，但 binary 图像接口会返回全黑图。
static constexpr bool kTempDemandOtsuKeepFullBinaryCache = false;
// 后处理逆透视开关：先迷宫法，再将边线点做逆透视并渲染黑底白线图。
static constexpr bool kEnableGrayPointerInversePerspective = true;
// 逆透视输出域（用户反馈按 100x100 标定）。
static constexpr int kIpmOutputWidth = 100;
static constexpr int kIpmOutputHeight = 100;
// 赛道参数：当前标定为 70px 对应 45cm。
static constexpr float kLaneWidthCm = 45.0f;
static constexpr float kLaneWidthPx = 70.0f;
static constexpr float kHalfLaneOffsetPx = kLaneWidthPx * 0.5f;
// 边线/中线等距采样步长：按 2cm 采样。
static constexpr float kSampleStepCm = 2.0f;
static constexpr float kSampleStepPx = (kSampleStepCm * kLaneWidthPx) / kLaneWidthCm;
// 中线起点归一化固定点（降低左右切换导致的抖动）。
static constexpr float kCenterNormalizeStartX = static_cast<float>(UVC_WIDTH / 2);
static constexpr float kCenterNormalizeStartY = static_cast<float>(UVC_HEIGHT - 1);

enum center_track_mode_t
{
    CENTER_TRACK_LEFT = 0,
    CENTER_TRACK_RIGHT = 1,
    CENTER_TRACK_AUTO = 2,
};
// AUTO: 左优先，左线丢失时自动切右线。
static constexpr center_track_mode_t kCenterTrackMode = CENTER_TRACK_AUTO;
// 逆透视矩阵（用户提供）。
static constexpr double change_un_Mat[3][3] = {
    {-0.404545, 0.346369, -27.015230},
    {0.000000, 0.068428, -23.873436},
    {0.000000, 0.004613, -0.609445}
};
// 畸变参数接口暂保留（当前流程未使用）。
[[maybe_unused]] static constexpr double cameraMatrix[3][3] = {
    {81.742134, 0.000000, 73.715838},
    {0.000000, 81.615092, 56.550951},
    {0.000000, 0.000000, 1.000000}
};
[[maybe_unused]] static constexpr double distCoeffs[5] = {0.191611, -0.146004, -0.010946, -0.008742, -0.030884};
[[maybe_unused]] static constexpr int move_xy[2] = {0, 162};

static uint8 g_xy_x1_boundary[VISION_BOUNDARY_NUM];
static uint8 g_xy_x2_boundary[VISION_BOUNDARY_NUM];
static uint8 g_xy_x3_boundary[VISION_BOUNDARY_NUM];
static uint8 g_xy_y1_boundary[VISION_BOUNDARY_NUM];
static uint8 g_xy_y2_boundary[VISION_BOUNDARY_NUM];
static uint8 g_xy_y3_boundary[VISION_BOUNDARY_NUM];
static int g_boundary_count = 0;

static uint8 g_image_bgr[UVC_HEIGHT * UVC_WIDTH * 3];
static uint8 g_image_gray[UVC_HEIGHT * UVC_WIDTH];
static uint8 g_image_binary_u8[UVC_HEIGHT * UVC_WIDTH];
static uint8 g_image_rgb565[UVC_HEIGHT * UVC_WIDTH * 2];
// 调试输出缓存：逆透视彩色图、逆透视边线图（黑底白线）。
static uint8 g_image_ipm_bgr[UVC_HEIGHT * UVC_WIDTH * 3];
static uint8 g_image_ipm_edge_gray[UVC_HEIGHT * UVC_WIDTH];
// 逆透视矩阵正向映射（原图->俯视）及初始化标志。
static double g_ipm_forward_mat[3][3] = {{0.0}};
static bool g_ipm_forward_ready = false;

static bool g_red_rect_found = false;
static int g_red_rect_x = 0;
static int g_red_rect_y = 0;
static int g_red_rect_w = 0;
static int g_red_rect_h = 0;
static int g_red_rect_cx = 0;
static int g_red_rect_cy = 0;
static int g_red_rect_area = 0;

static bool g_ncnn_roi_valid = false;
static int g_ncnn_roi_x = 0;
static int g_ncnn_roi_y = 0;
static int g_ncnn_roi_w = 0;
static int g_ncnn_roi_h = 0;

// 最近一帧处理耗时（us）
static uint32 g_last_capture_wait_us = 0;
static uint32 g_last_preprocess_us = 0;
static uint32 g_last_otsu_us = 0;
static uint32 g_last_maze_us = 0;
static uint32 g_last_total_us = 0;

// 采集线程：只负责拿相机最新 BGR 帧并写入共享缓冲。
static std::thread g_capture_thread;
static std::atomic<bool> g_capture_running(false);
static std::mutex g_capture_mutex;
static std::condition_variable g_capture_cv;
static std::array<uint8, UVC_HEIGHT * UVC_WIDTH * 3> g_capture_frame{};
static uint32 g_capture_seq = 0;
static uint32 g_processed_seq = 0;

// 对外暴露控制量
int line_error = 0;
int line_sample_ratio_num = 1;
int line_sample_ratio_den = 2;

struct maze_point_t
{
    int x;
    int y;
};

static void fill_boundary_arrays_from_maze(const maze_point_t *left_pts,
                                           int left_num,
                                           const maze_point_t *right_pts,
                                           int right_num);

/* 前进方向定义：
 *   0
 * 3   1
 *   2
 */
static constexpr int kDirFront[4][2] = {
    {0, -1},
    {1, 0},
    {0, 1},
    {-1, 0}
};
static constexpr int kDirFrontLeft[4][2] = {
    {-1, -1},
    {1, -1},
    {1, 1},
    {-1, 1}
};
static constexpr int kDirFrontRight[4][2] = {
    {1, -1},
    {1, 1},
    {-1, 1},
    {-1, -1}
};

static void clear_boundary_arrays()
{
    std::fill_n(g_xy_x1_boundary, VISION_BOUNDARY_NUM, static_cast<uint8>(0));
    std::fill_n(g_xy_x2_boundary, VISION_BOUNDARY_NUM, static_cast<uint8>(0));
    std::fill_n(g_xy_x3_boundary, VISION_BOUNDARY_NUM, static_cast<uint8>(0));
    std::fill_n(g_xy_y1_boundary, VISION_BOUNDARY_NUM, static_cast<uint8>(0));
    std::fill_n(g_xy_y2_boundary, VISION_BOUNDARY_NUM, static_cast<uint8>(0));
    std::fill_n(g_xy_y3_boundary, VISION_BOUNDARY_NUM, static_cast<uint8>(0));
    g_boundary_count = 0;
}

static uint8 compute_global_otsu_threshold_u8(const uint8 *gray_img)
{
    if (gray_img == nullptr)
    {
        return 127;
    }

    std::array<uint32, 256> histogram{};
    uint64_t gray_sum = 0;
    constexpr int kPixelCount = UVC_WIDTH * UVC_HEIGHT;

    for (int i = 0; i < kPixelCount; ++i)
    {
        uint8 v = gray_img[i];
        ++histogram[v];
        gray_sum += static_cast<uint64_t>(v);
    }

    uint32 weight_bg = 0;
    uint64_t sum_bg = 0;
    double best_between_var = -1.0;
    uint8 best_threshold = 127;

    for (int t = 0; t < 256; ++t)
    {
        weight_bg += histogram[t];
        if (weight_bg == 0)
        {
            continue;
        }

        uint32 weight_fg = static_cast<uint32>(kPixelCount) - weight_bg;
        if (weight_fg == 0)
        {
            break;
        }

        sum_bg += static_cast<uint64_t>(histogram[t]) * static_cast<uint64_t>(t);

        double mean_bg = static_cast<double>(sum_bg) / static_cast<double>(weight_bg);
        double mean_fg = static_cast<double>(gray_sum - sum_bg) / static_cast<double>(weight_fg);
        double diff = mean_bg - mean_fg;
        double between_var = static_cast<double>(weight_bg) * static_cast<double>(weight_fg) * diff * diff;

        if (between_var > best_between_var)
        {
            best_between_var = between_var;
            best_threshold = static_cast<uint8>(t);
        }
    }

    return best_threshold;
}

static inline bool pixel_is_white(const uint8 *img, int x, int y, uint8 white_threshold)
{
    return img[y * UVC_WIDTH + x] > white_threshold;
}

static inline bool pixel_is_wall(const uint8 *img, int x, int y, uint8 white_threshold, bool wall_is_white)
{
    return wall_is_white ? pixel_is_white(img, x, y, white_threshold) : !pixel_is_white(img, x, y, white_threshold);
}

static void build_binary_image_from_gray_threshold(const uint8 *gray_img, uint8 threshold)
{
    if (gray_img == nullptr)
    {
        std::fill_n(g_image_binary_u8, UVC_WIDTH * UVC_HEIGHT, static_cast<uint8>(0));
        return;
    }

    constexpr int kPixelCount = UVC_WIDTH * UVC_HEIGHT;
    for (int i = 0; i < kPixelCount; ++i)
    {
        g_image_binary_u8[i] = (gray_img[i] > threshold) ? static_cast<uint8>(255) : static_cast<uint8>(0);
    }
}

static bool init_ipm_forward_matrix()
{
    cv::Mat h_inv(3, 3, CV_64F, const_cast<double *>(&change_un_Mat[0][0]));
    cv::Mat h(3, 3, CV_64F);
    const double det = cv::invert(h_inv, h, cv::DECOMP_LU);
    if (std::fabs(det) < 1e-12)
    {
        g_ipm_forward_ready = false;
        return false;
    }

    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            g_ipm_forward_mat[r][c] = h.at<double>(r, c);
        }
    }
    g_ipm_forward_ready = true;
    return true;
}

static bool src_point_to_ipm_point(int src_x, int src_y, int *ipm_x, int *ipm_y)
{
    if (ipm_x == nullptr || ipm_y == nullptr)
    {
        return false;
    }

    if (!g_ipm_forward_ready && !init_ipm_forward_matrix())
    {
        return false;
    }

    const double w = g_ipm_forward_mat[2][0] * src_x + g_ipm_forward_mat[2][1] * src_y + g_ipm_forward_mat[2][2];
    if (std::fabs(w) < 1e-9)
    {
        return false;
    }

    const int x = static_cast<int>(std::lround((g_ipm_forward_mat[0][0] * src_x + g_ipm_forward_mat[0][1] * src_y + g_ipm_forward_mat[0][2]) / w));
    const int y = static_cast<int>(std::lround((g_ipm_forward_mat[1][0] * src_x + g_ipm_forward_mat[1][1] * src_y + g_ipm_forward_mat[1][2]) / w));
    if (x < 0 || x >= kIpmOutputWidth || y < 0 || y >= kIpmOutputHeight)
    {
        return false;
    }

    *ipm_x = x;
    *ipm_y = y;
    return true;
}

static inline void ipm_to_proc_point(int ipm_x, int ipm_y, int *proc_x, int *proc_y)
{
    const int denom_x = std::max(1, kIpmOutputWidth - 1);
    const int denom_y = std::max(1, kIpmOutputHeight - 1);
    *proc_x = std::clamp((ipm_x * (UVC_WIDTH - 1)) / denom_x, 0, UVC_WIDTH - 1);
    *proc_y = std::clamp((ipm_y * (UVC_HEIGHT - 1)) / denom_y, 0, UVC_HEIGHT - 1);
}

static void draw_line_gray(uint8 *img, int width, int height, int x0, int y0, int x1, int y1, uint8 value)
{
    if (img == nullptr || width <= 0 || height <= 0)
    {
        return;
    }

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true)
    {
        if (x0 >= 0 && x0 < width && y0 >= 0 && y0 < height)
        {
            img[y0 * width + x0] = value;
        }
        if (x0 == x1 && y0 == y1)
        {
            break;
        }
        int e2 = err * 2;
        if (e2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

static std::vector<cv::Point2f> maze_points_to_polyline(const maze_point_t *pts, int num)
{
    std::vector<cv::Point2f> out;
    if (pts == nullptr || num <= 0)
    {
        return out;
    }

    out.reserve(static_cast<size_t>(num));
    for (int i = 0; i < num; ++i)
    {
        out.emplace_back(static_cast<float>(pts[i].x), static_cast<float>(pts[i].y));
    }
    return out;
}

static void clamp_polyline_to_image(std::vector<cv::Point2f> *pts)
{
    if (pts == nullptr)
    {
        return;
    }

    for (auto &p : *pts)
    {
        p.x = std::clamp(p.x, 0.0f, static_cast<float>(UVC_WIDTH - 1));
        p.y = std::clamp(p.y, 0.0f, static_cast<float>(UVC_HEIGHT - 1));
    }
}

static void remove_near_duplicate_points(std::vector<cv::Point2f> *pts, float min_dist)
{
    if (pts == nullptr || pts->size() <= 1)
    {
        return;
    }

    const float min_dist_sq = min_dist * min_dist;
    std::vector<cv::Point2f> filtered;
    filtered.reserve(pts->size());
    filtered.push_back((*pts)[0]);
    for (size_t i = 1; i < pts->size(); ++i)
    {
        cv::Point2f d = (*pts)[i] - filtered.back();
        if ((d.x * d.x + d.y * d.y) >= min_dist_sq)
        {
            filtered.push_back((*pts)[i]);
        }
    }
    pts->swap(filtered);
}

static std::vector<cv::Point2f> triangle_filter_polyline(const std::vector<cv::Point2f> &pts)
{
    if (pts.size() < 3)
    {
        return pts;
    }

    std::vector<cv::Point2f> out = pts;
    for (size_t i = 1; i + 1 < pts.size(); ++i)
    {
        out[i] = (pts[i - 1] + pts[i] * 2.0f + pts[i + 1]) * 0.25f;
    }
    return out;
}

static std::vector<cv::Point2f> resample_polyline_equal_spacing(const std::vector<cv::Point2f> &pts, float step)
{
    std::vector<cv::Point2f> out;
    if (pts.empty())
    {
        return out;
    }

    if (pts.size() == 1 || step <= 0.01f)
    {
        return pts;
    }

    out.reserve(pts.size());
    out.push_back(pts[0]);

    float dist_acc = 0.0f;
    for (size_t i = 1; i < pts.size(); ++i)
    {
        cv::Point2f a = pts[i - 1];
        cv::Point2f b = pts[i];
        cv::Point2f v = b - a;
        float seg_len = std::sqrt(v.x * v.x + v.y * v.y);
        if (seg_len < 1e-5f)
        {
            continue;
        }

        float used = 0.0f;
        while ((dist_acc + (seg_len - used)) >= step)
        {
            float remain = step - dist_acc;
            float t = (used + remain) / seg_len;
            cv::Point2f p = a + v * t;
            out.push_back(p);
            used += remain;
            dist_acc = 0.0f;
        }

        dist_acc += (seg_len - used);
    }

    if (cv::norm(out.back() - pts.back()) > (step * 0.5f))
    {
        out.push_back(pts.back());
    }

    return out;
}

static std::vector<cv::Point2f> preprocess_boundary_polyline(const maze_point_t *pts, int num)
{
    std::vector<cv::Point2f> line = maze_points_to_polyline(pts, num);
    if (line.size() <= 1)
    {
        clamp_polyline_to_image(&line);
        return line;
    }

    remove_near_duplicate_points(&line, 0.5f);
    line = triangle_filter_polyline(line);
    line = resample_polyline_equal_spacing(line, kSampleStepPx);
    line = triangle_filter_polyline(line);
    clamp_polyline_to_image(&line);
    remove_near_duplicate_points(&line, 0.5f);
    return line;
}

static std::vector<cv::Point2f> offset_centerline_from_side(const std::vector<cv::Point2f> &side_line, bool from_left, float offset_px)
{
    std::vector<cv::Point2f> center;
    if (side_line.size() < 2)
    {
        return center;
    }

    center.reserve(side_line.size());
    for (size_t i = 0; i < side_line.size(); ++i)
    {
        const cv::Point2f prev = side_line[(i == 0) ? 0 : (i - 1)];
        const cv::Point2f next = side_line[(i + 1 >= side_line.size()) ? (side_line.size() - 1) : (i + 1)];
        const cv::Point2f t_raw = next - prev;
        const float t_norm = std::sqrt(t_raw.x * t_raw.x + t_raw.y * t_raw.y);
        if (t_norm < 1e-5f)
        {
            continue;
        }

        const cv::Point2f t = t_raw * (1.0f / t_norm);
        // 图像坐标系下，左边线向右偏移用 n=( -dy, dx )；右边线向左偏移用 n=( dy, -dx )。
        cv::Point2f n = from_left ? cv::Point2f(-t.y, t.x) : cv::Point2f(t.y, -t.x);
        center.push_back(side_line[i] + n * offset_px);
    }

    clamp_polyline_to_image(&center);
    remove_near_duplicate_points(&center, 0.5f);
    return center;
}

static std::vector<cv::Point2f> midpoint_centerline(const std::vector<cv::Point2f> &left_line,
                                                    const std::vector<cv::Point2f> &right_line)
{
    std::vector<cv::Point2f> center;
    if (left_line.empty() || right_line.empty())
    {
        return center;
    }

    const size_t out_n = std::max(left_line.size(), right_line.size());
    center.reserve(out_n);
    for (size_t i = 0; i < out_n; ++i)
    {
        const size_t li = (left_line.size() <= 1 || out_n <= 1) ? 0 : (i * (left_line.size() - 1)) / (out_n - 1);
        const size_t ri = (right_line.size() <= 1 || out_n <= 1) ? 0 : (i * (right_line.size() - 1)) / (out_n - 1);
        center.push_back((left_line[li] + right_line[ri]) * 0.5f);
    }
    clamp_polyline_to_image(&center);
    remove_near_duplicate_points(&center, 0.5f);
    return center;
}

static void normalize_centerline_start(std::vector<cv::Point2f> *center_line,
                                       const cv::Point2f &start_point)
{
    if (center_line == nullptr || center_line->empty())
    {
        return;
    }

    int best_idx = 0;
    float best_dist = std::numeric_limits<float>::max();
    for (int i = 0; i < static_cast<int>(center_line->size()); ++i)
    {
        float d = cv::norm((*center_line)[i] - start_point);
        if (d < best_dist)
        {
            best_dist = d;
            best_idx = i;
        }
    }

    if (best_idx > 0)
    {
        center_line->erase(center_line->begin(), center_line->begin() + best_idx);
    }

    if (center_line->empty())
    {
        center_line->push_back(start_point);
        return;
    }

    if (cv::norm(center_line->front() - start_point) > 1.0f)
    {
        center_line->insert(center_line->begin(), start_point);
    }
    else
    {
        center_line->front() = start_point;
    }
}

static bool choose_center_from_left(size_t left_n, size_t right_n)
{
    if (kCenterTrackMode == CENTER_TRACK_LEFT)
    {
        return true;
    }
    if (kCenterTrackMode == CENTER_TRACK_RIGHT)
    {
        return false;
    }

    // AUTO: 左优先，左边线不足时切到右边线。
    if (left_n >= 2)
    {
        return true;
    }
    return right_n < 2;
}

static int compute_line_error_from_centerline(const std::vector<cv::Point2f> &center_line)
{
    if (center_line.empty())
    {
        return 0;
    }

    const int den = std::max(1, line_sample_ratio_den);
    const int num = std::clamp(line_sample_ratio_num, 0, den);
    const float sample_y = static_cast<float>((UVC_HEIGHT - 1) * num) / static_cast<float>(den);

    int best_idx = 0;
    float best_dy = std::fabs(center_line[0].y - sample_y);
    for (int i = 1; i < static_cast<int>(center_line.size()); ++i)
    {
        const float dy = std::fabs(center_line[i].y - sample_y);
        if (dy < best_dy)
        {
            best_dy = dy;
            best_idx = i;
        }
    }

    const int cx = std::clamp(static_cast<int>(std::lround(center_line[best_idx].x)), 0, UVC_WIDTH - 1);
    return cx - (UVC_WIDTH / 2);
}

static inline uint8 point_x_to_u8(const cv::Point2f &p)
{
    const int x = std::clamp(static_cast<int>(std::lround(p.x)), 0, UVC_WIDTH - 1);
    return static_cast<uint8>(x);
}

static inline uint8 point_y_to_u8(const cv::Point2f &p)
{
    const int y = std::clamp(static_cast<int>(std::lround(p.y)), 0, UVC_HEIGHT - 1);
    return static_cast<uint8>(y);
}

static void fill_boundary_arrays_from_lines(const std::vector<cv::Point2f> &left_line,
                                            const std::vector<cv::Point2f> &center_line,
                                            const std::vector<cv::Point2f> &right_line)
{
    clear_boundary_arrays();

    const int left_num = static_cast<int>(left_line.size());
    const int center_num = static_cast<int>(center_line.size());
    const int right_num = static_cast<int>(right_line.size());
    if (left_num <= 0 && center_num <= 0 && right_num <= 0)
    {
        return;
    }

    int out_num = std::max({left_num, center_num, right_num});
    out_num = std::clamp(out_num, 0, VISION_BOUNDARY_NUM);
    if (out_num <= 0)
    {
        return;
    }

    for (int i = 0; i < out_num; ++i)
    {
        auto pick_point = [i, out_num](const std::vector<cv::Point2f> &line) -> cv::Point2f {
            if (line.empty())
            {
                return cv::Point2f(0.0f, 0.0f);
            }
            if (line.size() == 1 || out_num <= 1)
            {
                return line[0];
            }
            size_t idx = (static_cast<size_t>(i) * (line.size() - 1)) / static_cast<size_t>(out_num - 1);
            return line[idx];
        };

        cv::Point2f lp = pick_point(left_line);
        cv::Point2f rp = pick_point(right_line);
        cv::Point2f cp = pick_point(center_line);

        if (left_line.empty())
        {
            lp = right_line.empty() ? cp : rp;
        }
        if (right_line.empty())
        {
            rp = left_line.empty() ? cp : lp;
        }
        if (center_line.empty())
        {
            cp = (lp + rp) * 0.5f;
        }

        g_xy_x1_boundary[i] = point_x_to_u8(lp);
        g_xy_y1_boundary[i] = point_y_to_u8(lp);
        g_xy_x2_boundary[i] = point_x_to_u8(cp);
        g_xy_y2_boundary[i] = point_y_to_u8(cp);
        g_xy_x3_boundary[i] = point_x_to_u8(rp);
        g_xy_y3_boundary[i] = point_y_to_u8(rp);
    }

    g_boundary_count = out_num;
}

static void draw_polyline_gray(uint8 *img,
                               int width,
                               int height,
                               const std::vector<cv::Point2f> &line,
                               uint8 value)
{
    if (img == nullptr || line.empty())
    {
        return;
    }

    if (line.size() == 1)
    {
        const int x = std::clamp(static_cast<int>(std::lround(line[0].x)), 0, width - 1);
        const int y = std::clamp(static_cast<int>(std::lround(line[0].y)), 0, height - 1);
        img[y * width + x] = value;
        return;
    }

    for (size_t i = 1; i < line.size(); ++i)
    {
        const int x0 = static_cast<int>(std::lround(line[i - 1].x));
        const int y0 = static_cast<int>(std::lround(line[i - 1].y));
        const int x1 = static_cast<int>(std::lround(line[i].x));
        const int y1 = static_cast<int>(std::lround(line[i].y));
        draw_line_gray(img, width, height, x0, y0, x1, y1, value);
    }
}

static int transform_boundary_points_to_proc(const maze_point_t *src_pts, int src_num, maze_point_t *dst_pts, int max_pts)
{
    if (src_pts == nullptr || dst_pts == nullptr || src_num <= 0 || max_pts <= 0)
    {
        return 0;
    }

    int out = 0;
    for (int i = 0; i < src_num && out < max_pts; ++i)
    {
        int ipm_x = 0;
        int ipm_y = 0;
        if (!src_point_to_ipm_point(src_pts[i].x, src_pts[i].y, &ipm_x, &ipm_y))
        {
            continue;
        }

        int proc_x = 0;
        int proc_y = 0;
        ipm_to_proc_point(ipm_x, ipm_y, &proc_x, &proc_y);
        dst_pts[out++] = {proc_x, proc_y};
    }
    return out;
}

static void build_ipm_bgr_debug_image()
{
    std::memset(g_image_ipm_bgr, 0, UVC_WIDTH * UVC_HEIGHT * 3);
    if (!g_ipm_forward_ready && !init_ipm_forward_matrix())
    {
        return;
    }

    cv::Mat src(UVC_HEIGHT, UVC_WIDTH, CV_8UC3, g_image_bgr);
    cv::Mat ipm_small(kIpmOutputHeight, kIpmOutputWidth, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat ipm_full(UVC_HEIGHT, UVC_WIDTH, CV_8UC3, g_image_ipm_bgr);
    cv::Mat h(3, 3, CV_64F, &g_ipm_forward_mat[0][0]);

    cv::warpPerspective(src, ipm_small, h, ipm_small.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::resize(ipm_small, ipm_full, ipm_full.size(), 0.0, 0.0, cv::INTER_LINEAR);
}

static int render_ipm_boundary_image_and_update_boundaries(const maze_point_t *left_pts,
                                                           int left_num,
                                                           const maze_point_t *right_pts,
                                                           int right_num)
{
    std::memset(g_image_ipm_edge_gray, 0, UVC_WIDTH * UVC_HEIGHT);
    build_ipm_bgr_debug_image();

    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_proc{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> right_proc{};
    const int left_proc_num = transform_boundary_points_to_proc(left_pts, left_num, left_proc.data(), static_cast<int>(left_proc.size()));
    const int right_proc_num = transform_boundary_points_to_proc(right_pts, right_num, right_proc.data(), static_cast<int>(right_proc.size()));

    std::vector<cv::Point2f> left_line = preprocess_boundary_polyline(left_proc.data(), left_proc_num);
    std::vector<cv::Point2f> right_line = preprocess_boundary_polyline(right_proc.data(), right_proc_num);

    const bool use_left = choose_center_from_left(left_line.size(), right_line.size());
    std::vector<cv::Point2f> center_line;
    if (use_left)
    {
        center_line = offset_centerline_from_side(left_line, true, kHalfLaneOffsetPx);
        if (center_line.size() < 2)
        {
            center_line = offset_centerline_from_side(right_line, false, kHalfLaneOffsetPx);
        }
    }
    else
    {
        center_line = offset_centerline_from_side(right_line, false, kHalfLaneOffsetPx);
        if (center_line.size() < 2)
        {
            center_line = offset_centerline_from_side(left_line, true, kHalfLaneOffsetPx);
        }
    }

    if (center_line.empty())
    {
        center_line = midpoint_centerline(left_line, right_line);
    }

    if (!center_line.empty())
    {
        center_line = triangle_filter_polyline(center_line);
        center_line = resample_polyline_equal_spacing(center_line, kSampleStepPx);
        normalize_centerline_start(&center_line, cv::Point2f(kCenterNormalizeStartX, kCenterNormalizeStartY));
        clamp_polyline_to_image(&center_line);
        remove_near_duplicate_points(&center_line, 0.5f);
    }

    draw_polyline_gray(g_image_ipm_edge_gray, UVC_WIDTH, UVC_HEIGHT, left_line, 255);
    draw_polyline_gray(g_image_ipm_edge_gray, UVC_WIDTH, UVC_HEIGHT, right_line, 255);
    draw_polyline_gray(g_image_ipm_edge_gray, UVC_WIDTH, UVC_HEIGHT, center_line, 255);

    fill_boundary_arrays_from_lines(left_line, center_line, right_line);
    return compute_line_error_from_centerline(center_line);
}

static bool find_maze_start_from_bottom(const uint8 *classify_img,
                                        uint8 white_threshold,
                                        bool search_left,
                                        int *start_x,
                                        int *start_y,
                                        bool *wall_is_white)
{
    if (classify_img == nullptr || start_x == nullptr || start_y == nullptr || wall_is_white == nullptr)
    {
        return false;
    }

    const int width = UVC_WIDTH;
    const int height = UVC_HEIGHT;
    if (width < 4 || height < 3)
    {
        return false;
    }

    const int center_x = width / 2;
    const int scan_y = height - 1;   // 从图像最底部行找起始边界
    const int track_y = height - 2;  // 实际迷宫法从倒数第二行开始，避免访问越界

    int x_found = -1;
    bool path_is_white = false;
    bool wall_is_white_local = false;
    if (search_left)
    {
        for (int x = center_x; x >= 2; --x)
        {
            bool inner_white = pixel_is_white(classify_img, x, scan_y, white_threshold);
            bool outer_white = pixel_is_white(classify_img, x - 1, scan_y, white_threshold);
            if (inner_white != outer_white)
            {
                x_found = x; // 靠近图像中心的一侧
                path_is_white = inner_white;
                wall_is_white_local = outer_white;
                break;
            }
        }
    }
    else
    {
        for (int x = center_x; x <= width - 3; ++x)
        {
            bool inner_white = pixel_is_white(classify_img, x, scan_y, white_threshold);
            bool outer_white = pixel_is_white(classify_img, x + 1, scan_y, white_threshold);
            if (inner_white != outer_white)
            {
                x_found = x; // 靠近图像中心的一侧
                path_is_white = inner_white;
                wall_is_white_local = outer_white;
                break;
            }
        }
    }

    if (x_found < 1 || x_found > width - 2)
    {
        return false;
    }

    // 如果起点误落在墙侧，向图像中心方向微调到路径侧。
    int x_adjust = x_found;
    int center_step = search_left ? 1 : -1;
    for (int i = 0; i < 8; ++i)
    {
        bool cur_white = pixel_is_white(classify_img, x_adjust, track_y, white_threshold);
        if (cur_white == path_is_white)
        {
            break;
        }

        int next_x = x_adjust + center_step;
        if (next_x < 1 || next_x > width - 2)
        {
            break;
        }
        x_adjust = next_x;
    }

    *start_x = x_adjust;
    *start_y = track_y;
    *wall_is_white = wall_is_white_local;
    return true;
}

static int maze_trace_left_hand(const uint8 *classify_img,
                                uint8 white_threshold,
                                bool wall_is_white,
                                int start_x,
                                int start_y,
                                int y_min,
                                maze_point_t *pts,
                                int max_pts)
{
    if (classify_img == nullptr || pts == nullptr || max_pts <= 0)
    {
        return 0;
    }

    int x = start_x;
    int y = start_y;
    int dir = 0;
    int turn = 0;
    int step = 0;

    while (step < max_pts &&
           x > 0 && x < UVC_WIDTH - 1 &&
           y > 0 && y < UVC_HEIGHT - 1 &&
           y >= y_min &&
           turn < 4)
    {
        int fx = x + kDirFront[dir][0];
        int fy = y + kDirFront[dir][1];
        int flx = x + kDirFrontLeft[dir][0];
        int fly = y + kDirFrontLeft[dir][1];

        bool front_is_wall = pixel_is_wall(classify_img, fx, fy, white_threshold, wall_is_white);
        bool frontleft_is_wall = pixel_is_wall(classify_img, flx, fly, white_threshold, wall_is_white);

        if (front_is_wall)
        {
            dir = (dir + 1) % 4;
            turn++;
        }
        else if (frontleft_is_wall)
        {
            x = fx;
            y = fy;
            pts[step++] = {x, y};
            turn = 0;
        }
        else
        {
            x = flx;
            y = fly;
            dir = (dir + 3) % 4;
            pts[step++] = {x, y};
            turn = 0;
        }
    }

    return step;
}

static int maze_trace_right_hand(const uint8 *classify_img,
                                 uint8 white_threshold,
                                 bool wall_is_white,
                                 int start_x,
                                 int start_y,
                                 int y_min,
                                 maze_point_t *pts,
                                 int max_pts)
{
    if (classify_img == nullptr || pts == nullptr || max_pts <= 0)
    {
        return 0;
    }

    int x = start_x;
    int y = start_y;
    int dir = 0;
    int turn = 0;
    int step = 0;

    while (step < max_pts &&
           x > 0 && x < UVC_WIDTH - 1 &&
           y > 0 && y < UVC_HEIGHT - 1 &&
           y >= y_min &&
           turn < 4)
    {
        int fx = x + kDirFront[dir][0];
        int fy = y + kDirFront[dir][1];
        int frx = x + kDirFrontRight[dir][0];
        int fry = y + kDirFrontRight[dir][1];

        bool front_is_wall = pixel_is_wall(classify_img, fx, fy, white_threshold, wall_is_white);
        bool frontright_is_wall = pixel_is_wall(classify_img, frx, fry, white_threshold, wall_is_white);

        if (front_is_wall)
        {
            dir = (dir + 3) % 4;
            turn++;
        }
        else if (frontright_is_wall)
        {
            x = fx;
            y = fy;
            pts[step++] = {x, y};
            turn = 0;
        }
        else
        {
            x = frx;
            y = fry;
            dir = (dir + 1) % 4;
            pts[step++] = {x, y};
            turn = 0;
        }
    }

    return step;
}

static void fill_boundary_arrays_from_maze(const maze_point_t *left_pts,
                                           int left_num,
                                           const maze_point_t *right_pts,
                                           int right_num)
{
    clear_boundary_arrays();
    if ((left_pts == nullptr && right_pts == nullptr) || (left_num <= 0 && right_num <= 0))
    {
        return;
    }

    int out_num = std::max(left_num, right_num);
    if (out_num > VISION_BOUNDARY_NUM)
    {
        out_num = VISION_BOUNDARY_NUM;
    }

    for (int i = 0; i < out_num; ++i)
    {
        int li = 0;
        int ri = 0;
        if (left_num > 0)
        {
            li = (i < left_num) ? i : (left_num - 1);
        }
        if (right_num > 0)
        {
            ri = (i < right_num) ? i : (right_num - 1);
        }

        int lx = (left_num > 0) ? left_pts[li].x : ((right_num > 0) ? right_pts[ri].x : 0);
        int ly = (left_num > 0) ? left_pts[li].y : ((right_num > 0) ? right_pts[ri].y : 0);
        int rx = (right_num > 0) ? right_pts[ri].x : ((left_num > 0) ? left_pts[li].x : 0);
        int ry = (right_num > 0) ? right_pts[ri].y : ((left_num > 0) ? left_pts[li].y : 0);

        g_xy_x1_boundary[i] = static_cast<uint8>(lx);
        g_xy_y1_boundary[i] = static_cast<uint8>(ly);

        // 中线使用左右边界的均值。
        int mx = (lx + rx) / 2;
        int my = (ly + ry) / 2;
        g_xy_x2_boundary[i] = static_cast<uint8>(mx);
        g_xy_y2_boundary[i] = static_cast<uint8>(my);

        g_xy_x3_boundary[i] = static_cast<uint8>(rx);
        g_xy_y3_boundary[i] = static_cast<uint8>(ry);
    }

    g_boundary_count = out_num;
}

static void capture_loop()
{
    while (g_capture_running.load())
    {
        if (wait_image_refresh() < 0 || bgr_image == nullptr)
        {
            system_delay_ms(1);
            continue;
        }

        {
            std::lock_guard<std::mutex> lk(g_capture_mutex);
            std::memcpy(g_capture_frame.data(), bgr_image, g_capture_frame.size());
            ++g_capture_seq;
        }
        g_capture_cv.notify_one();
    }
}

bool vision_image_processor_init(const char *camera_path)
{
    if (g_capture_running.load())
    {
        return true;
    }

    if (camera_path == nullptr || camera_path[0] == '\0')
    {
        camera_path = "/dev/video0";
    }

    if (uvc_camera_init(camera_path) != 0)
    {
        return false;
    }

    {
        std::lock_guard<std::mutex> lk(g_capture_mutex);
        g_capture_seq = 0;
        g_processed_seq = 0;
    }

    clear_boundary_arrays();
    line_error = 0;

    g_capture_running.store(true);
    g_capture_thread = std::thread(capture_loop);
    return true;
}

void vision_image_processor_cleanup()
{
    if (!g_capture_running.load())
    {
        return;
    }

    g_capture_running.store(false);
    g_capture_cv.notify_all();

    if (g_capture_thread.joinable())
    {
        g_capture_thread.join();
    }
}

bool vision_image_processor_process_step()
{
    auto t0 = std::chrono::steady_clock::now();

    {
        std::unique_lock<std::mutex> lk(g_capture_mutex);
        bool ok = g_capture_cv.wait_for(
            lk,
            std::chrono::milliseconds(100),
            []() {
                return (g_capture_seq != g_processed_seq) || !g_capture_running.load();
            });

        if (!ok || g_capture_seq == g_processed_seq)
        {
            g_last_capture_wait_us = static_cast<uint32>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count());
            g_last_preprocess_us = 0;
            g_last_otsu_us = 0;
            g_last_maze_us = 0;
            g_last_total_us = g_last_capture_wait_us;
            return false;
        }

        std::memcpy(g_image_bgr, g_capture_frame.data(), sizeof(g_image_bgr));
        g_processed_seq = g_capture_seq;
    }

    auto t1 = std::chrono::steady_clock::now();

    auto t_pre_start = t1;

    // 灰度图（160x120）
    cv::Mat bgr(UVC_HEIGHT, UVC_WIDTH, CV_8UC3, g_image_bgr);
    cv::Mat gray(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, g_image_gray);
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

    // RGB565（160x120）
    for (int y = 0; y < UVC_HEIGHT; ++y)
    {
        const cv::Vec3b *row = bgr.ptr<cv::Vec3b>(y);
        for (int x = 0; x < UVC_WIDTH; ++x)
        {
            uint8 bb = row[x][0];
            uint8 gg = row[x][1];
            uint8 rr = row[x][2];
            uint16 v = static_cast<uint16>(((rr >> 3) << 11) | ((gg >> 2) << 5) | (bb >> 3));
            int idx = (y * UVC_WIDTH + x) * 2;
            g_image_rgb565[idx] = static_cast<uint8>(v >> 8);
            g_image_rgb565[idx + 1] = static_cast<uint8>(v & 0xFF);
        }
    }

    auto t_pre_end = std::chrono::steady_clock::now();

    auto t_otsu_start = t_pre_end;
    uint8 otsu_threshold = 127;
    if constexpr (kTempDemandOtsuEnable)
    {
        otsu_threshold = compute_global_otsu_threshold_u8(g_image_gray);
        if constexpr (kTempDemandOtsuKeepFullBinaryCache)
        {
            build_binary_image_from_gray_threshold(g_image_gray, otsu_threshold);
        }
        else
        {
            std::fill_n(g_image_binary_u8, UVC_WIDTH * UVC_HEIGHT, static_cast<uint8>(0));
        }
    }
    else
    {
        cv::Mat gray_ipm(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, g_image_gray);
        cv::Mat binary(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, g_image_binary_u8);
        cv::threshold(gray_ipm, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    }
    auto t_otsu_end = std::chrono::steady_clock::now();

    auto t_maze_start = t_otsu_end;
    const int y_min = std::max(1, UVC_HEIGHT - (UVC_HEIGHT * MAZE_LOWER_REGION_PERCENT) / 100);
    const uint8 *classify_img = nullptr;
    uint8 classify_white_threshold = 127;
    if constexpr (kTempDemandOtsuEnable)
    {
        classify_img = g_image_gray;
        classify_white_threshold = otsu_threshold;
    }
    else
    {
        classify_img = g_image_binary_u8;
        classify_white_threshold = 127;
    }

    int left_start_x = 0;
    int left_start_y = 0;
    int right_start_x = 0;
    int right_start_y = 0;
    bool left_wall_is_white = false;
    bool right_wall_is_white = false;

    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_pts{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> right_pts{};

    int left_num = 0;
    int right_num = 0;

    bool left_ok = find_maze_start_from_bottom(classify_img,
                                               classify_white_threshold,
                                               true,
                                               &left_start_x,
                                               &left_start_y,
                                               &left_wall_is_white);
    bool right_ok = find_maze_start_from_bottom(classify_img,
                                                classify_white_threshold,
                                                false,
                                                &right_start_x,
                                                &right_start_y,
                                                &right_wall_is_white);

    if (left_ok)
    {
        left_num = maze_trace_left_hand(classify_img,
                                        classify_white_threshold,
                                        left_wall_is_white,
                                        left_start_x,
                                        left_start_y,
                                        y_min,
                                        left_pts.data(),
                                        static_cast<int>(left_pts.size()));
    }
    if (right_ok)
    {
        right_num = maze_trace_right_hand(classify_img,
                                          classify_white_threshold,
                                          right_wall_is_white,
                                          right_start_x,
                                          right_start_y,
                                          y_min,
                                          right_pts.data(),
                                          static_cast<int>(right_pts.size()));
    }

    if constexpr (kEnableGrayPointerInversePerspective)
    {
        line_error = render_ipm_boundary_image_and_update_boundaries(left_pts.data(), left_num, right_pts.data(), right_num);
    }
    else
    {
        std::memset(g_image_ipm_bgr, 0, UVC_WIDTH * UVC_HEIGHT * 3);
        std::memset(g_image_ipm_edge_gray, 0, UVC_WIDTH * UVC_HEIGHT);
        fill_boundary_arrays_from_maze(left_pts.data(), left_num, right_pts.data(), right_num);
        line_error = 0;
    }
    auto t_maze_end = std::chrono::steady_clock::now();

    g_red_rect_found = false;
    g_red_rect_x = g_red_rect_y = g_red_rect_w = g_red_rect_h = 0;
    g_red_rect_cx = g_red_rect_cy = 0;
    g_red_rect_area = 0;

    g_last_capture_wait_us = static_cast<uint32>(std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());
    g_last_preprocess_us = static_cast<uint32>(std::chrono::duration_cast<std::chrono::microseconds>(t_pre_end - t_pre_start).count());
    g_last_otsu_us = static_cast<uint32>(std::chrono::duration_cast<std::chrono::microseconds>(t_otsu_end - t_otsu_start).count());
    g_last_maze_us = static_cast<uint32>(std::chrono::duration_cast<std::chrono::microseconds>(t_maze_end - t_maze_start).count());
    g_last_total_us = static_cast<uint32>(std::chrono::duration_cast<std::chrono::microseconds>(t_maze_end - t0).count());

    return true;
}

void vision_image_processor_get_last_perf_us(uint32 *capture_wait_us,
                                             uint32 *preprocess_us,
                                             uint32 *otsu_us,
                                             uint32 *maze_us,
                                             uint32 *total_us)
{
    if (capture_wait_us) *capture_wait_us = g_last_capture_wait_us;
    if (preprocess_us) *preprocess_us = g_last_preprocess_us;
    if (otsu_us) *otsu_us = g_last_otsu_us;
    if (maze_us) *maze_us = g_last_maze_us;
    if (total_us) *total_us = g_last_total_us;
}

const uint8 *vision_image_processor_gray_image()
{
    return g_image_gray;
}

const uint8 *vision_image_processor_binary_u8_image()
{
    return g_image_binary_u8;
}

const uint8 *vision_image_processor_bgr_image()
{
    return g_image_bgr;
}

const uint8 *vision_image_processor_rgb565_image()
{
    return g_image_rgb565;
}

const uint8 *vision_image_processor_gray_downsampled_image()
{
    return g_image_gray;
}

const uint8 *vision_image_processor_binary_downsampled_u8_image()
{
    return g_image_binary_u8;
}

const uint8 *vision_image_processor_bgr_downsampled_image()
{
    return g_image_bgr;
}

const uint8 *vision_image_processor_rgb565_downsampled_image()
{
    return g_image_rgb565;
}

const uint8 *vision_image_processor_ipm_bgr_downsampled_image()
{
    return g_image_ipm_bgr;
}

const uint8 *vision_image_processor_ipm_edge_gray_downsampled_image()
{
    return g_image_ipm_edge_gray;
}

void vision_image_processor_get_boundaries(uint8 **x1, uint8 **x2, uint8 **x3,
                                           uint8 **y1, uint8 **y2, uint8 **y3,
                                           uint16 *dot_num)
{
    if (x1) *x1 = g_xy_x1_boundary;
    if (x2) *x2 = g_xy_x2_boundary;
    if (x3) *x3 = g_xy_x3_boundary;
    if (y1) *y1 = g_xy_y1_boundary;
    if (y2) *y2 = g_xy_y2_boundary;
    if (y3) *y3 = g_xy_y3_boundary;
    if (dot_num) *dot_num = static_cast<uint16>(g_boundary_count);
}

void vision_image_processor_get_red_rect(bool *found, int *x, int *y, int *w, int *h, int *cx, int *cy)
{
    if (found) *found = g_red_rect_found;
    if (x) *x = g_red_rect_x;
    if (y) *y = g_red_rect_y;
    if (w) *w = g_red_rect_w;
    if (h) *h = g_red_rect_h;
    if (cx) *cx = g_red_rect_cx;
    if (cy) *cy = g_red_rect_cy;
}

int vision_image_processor_get_red_rect_area()
{
    return g_red_rect_area;
}

void vision_image_processor_set_ncnn_roi(bool valid, int x, int y, int w, int h)
{
    g_ncnn_roi_valid = valid;
    g_ncnn_roi_x = x;
    g_ncnn_roi_y = y;
    g_ncnn_roi_w = w;
    g_ncnn_roi_h = h;
}

void vision_image_processor_get_ncnn_roi(bool *valid, int *x, int *y, int *w, int *h)
{
    if (valid) *valid = g_ncnn_roi_valid;
    if (x) *x = g_ncnn_roi_x;
    if (y) *y = g_ncnn_roi_y;
    if (w) *w = g_ncnn_roi_w;
    if (h) *h = g_ncnn_roi_h;
}
