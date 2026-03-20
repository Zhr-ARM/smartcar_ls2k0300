#include "driver/vision/vision_image_processor.h"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cstring>
#include <queue>
#include <vector>

// ============================== 参数区 ==============================
// 边界点容量：预留更大空间，避免回弯/复杂场景时点数不够
#define VISION_BOUNDARY_NUM (UVC_HEIGHT * 4 / 2)
// 8邻域追踪起始行：从图像底部向上偏移高度比例（1/3）
static const int VISION_TRACE_START_OFFSET_DIVISOR = 3;

// ============================== 全局缓存区 ==============================
// 边界数组：用于发送到上位机显示
static uint8 g_xy_x1_boundary[VISION_BOUNDARY_NUM];
static uint8 g_xy_x2_boundary[VISION_BOUNDARY_NUM];
static uint8 g_xy_x3_boundary[VISION_BOUNDARY_NUM];
static uint8 g_xy_y1_boundary[VISION_BOUNDARY_NUM];
static uint8 g_xy_y2_boundary[VISION_BOUNDARY_NUM];
static uint8 g_xy_y3_boundary[VISION_BOUNDARY_NUM];
static int g_boundary_count = 0;

// 图像缓存：灰度图和8bit二值图
static uint8 g_image_gray[UVC_HEIGHT * UVC_WIDTH];
static uint8 g_image_binary_u8[UVC_HEIGHT * UVC_WIDTH];

// 红色实心矩形检测结果（全局坐标变量）
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
// 巡线线程最终只消费这一个量：
// line_error = 采样行处赛道中线 x - 图像中心 x。
// 也就是说，视觉线程把复杂图像压缩成了一个“横向偏差”。
int line_error = 0;

// 采样高度比例，当前默认取图像高度的 1/2 处。
// 取样越靠下，控制越灵敏但更容易抖；取样越靠上，预瞄更远但转向反应更慢。
int line_sample_ratio_num = 1;
int line_sample_ratio_den = 2;

static void detect_red_rect_hsv(const uint8 *bgr_data, int width, int height)
{
    // 参数：仅使用颜色、面积、实心度筛选，不限制长宽比
    const int area_min = 20;
    const int area_max = 400;
    const float fill_min = 0.72f;

    g_red_rect_found = false;
    g_red_rect_x = g_red_rect_y = g_red_rect_w = g_red_rect_h = 0;
    g_red_rect_cx = g_red_rect_cy = 0;
    g_red_rect_area = 0;

    if (bgr_data == nullptr || width <= 0 || height <= 0)
    {
        return;
    }

    cv::Mat bgr(height, width, CV_8UC3, const_cast<uint8 *>(bgr_data));
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // 红色双区间
    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, cv::Scalar(0, 80, 80), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(170, 80, 80), cv::Scalar(180, 255, 255), mask2);
    cv::bitwise_or(mask1, mask2, mask);

    // 仅在图像上2/3区域做红色识别
    int detect_h = (height * 2) / 3;
    if (detect_h > 0 && detect_h < height)
    {
        cv::Rect bottom_roi(0, detect_h, width, height - detect_h);
        mask(bottom_roi).setTo(0);
    }

    // 轻量去噪
    static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    cv::Mat labels, stats, centroids;
    int n = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
    int best_id = -1;
    int best_area = -1;

    for (int i = 1; i < n; ++i)
    {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < area_min || area > area_max)
        {
            continue;
        }

        int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        if (w <= 0 || h <= 0)
        {
            continue;
        }

        float fill = static_cast<float>(area) / static_cast<float>(w * h);
        if (fill < fill_min)
        {
            continue;
        }

        if (area > best_area)
        {
            best_area = area;
            best_id = i;
        }
    }

    if (best_id < 0)
    {
        return;
    }

    g_red_rect_x = stats.at<int>(best_id, cv::CC_STAT_LEFT);
    g_red_rect_y = stats.at<int>(best_id, cv::CC_STAT_TOP);
    g_red_rect_w = stats.at<int>(best_id, cv::CC_STAT_WIDTH);
    g_red_rect_h = stats.at<int>(best_id, cv::CC_STAT_HEIGHT);
    g_red_rect_cx = g_red_rect_x + g_red_rect_w / 2;
    g_red_rect_cy = g_red_rect_y + g_red_rect_h / 2;
    g_red_rect_area = best_area;
    g_red_rect_found = true;
}

static bool find_seed_point(const uint8 *img, int width, int height, int start_x, int start_y, int &seed_x, int &seed_y)
{
    if (start_x < 0) start_x = 0;
    if (start_x >= width) start_x = width - 1;
    if (start_y < 0) start_y = 0;
    if (start_y >= height) start_y = height - 1;

    if (img[start_y * width + start_x] > 0)
    {
        seed_x = start_x;
        seed_y = start_y;
        return true;
    }

    for (int y = start_y; y >= 0; --y)
    {
        if (img[y * width + start_x] > 0)
        {
            seed_x = start_x;
            seed_y = y;
            return true;
        }
    }

    for (int y = start_y; y >= 0; --y)
    {
        for (int x = 0; x < width; ++x)
        {
            if (img[y * width + x] > 0)
            {
                seed_x = x;
                seed_y = y;
                return true;
            }
        }
    }
    return false;
}

static void trace_component_8n(const uint8 *img, int width, int height, std::vector<cv::Point> &points)
{
    points.clear();
    int seed_x = width / 2;
    int seed_y = height - 1 - (height / VISION_TRACE_START_OFFSET_DIVISOR);
    if (!find_seed_point(img, width, height, seed_x, seed_y, seed_x, seed_y))
    {
        return;
    }

    std::vector<uint8> visited(width * height, 0);
    std::queue<cv::Point> q;
    q.push(cv::Point(seed_x, seed_y));
    visited[seed_y * width + seed_x] = 1;

    static const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static const int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

    while (!q.empty())
    {
        cv::Point p = q.front();
        q.pop();
        points.push_back(p);

        for (int k = 0; k < 8; ++k)
        {
            int nx = p.x + dx[k];
            int ny = p.y + dy[k];
            if (nx < 0 || nx >= width || ny < 0 || ny >= height)
            {
                continue;
            }
            int idx = ny * width + nx;
            if (visited[idx])
            {
                continue;
            }
            if (img[idx] > 0)
            {
                visited[idx] = 1;
                q.push(cv::Point(nx, ny));
            }
        }
    }
}

static void fill_boundary_arrays(const std::vector<cv::Point> &points)
{
    for (int i = 0; i < VISION_BOUNDARY_NUM; ++i)
    {
        g_xy_x1_boundary[i] = 0;
        g_xy_y1_boundary[i] = 0;
        g_xy_x2_boundary[i] = 0;
        g_xy_y2_boundary[i] = 0;
        g_xy_x3_boundary[i] = 0;
        g_xy_y3_boundary[i] = 0;
    }

    g_boundary_count = 0;
    if (points.empty())
    {
        return;
    }

    // 把追踪得到的散点按“每一行最左 / 最右”重新压缩。
    // 这样后面既能给上位机画左中右三条边界，也能很方便地计算每一行的中线。
    std::vector<int> min_x(UVC_HEIGHT, UVC_WIDTH);
    std::vector<int> max_x(UVC_HEIGHT, -1);

    for (const auto &p : points)
    {
        if (p.y >= 0 && p.y < UVC_HEIGHT)
        {
            if (p.x < min_x[p.y]) min_x[p.y] = p.x;
            if (p.x > max_x[p.y]) max_x[p.y] = p.x;
        }
    }

    int count = 0;
    for (int y = UVC_HEIGHT - 1; y >= 0 && count < VISION_BOUNDARY_NUM; --y)
    {
        if (max_x[y] < 0)
        {
            continue;
        }
        int left = min_x[y];
        int right = max_x[y];
        int mid = (left + right) / 2;

        g_xy_x1_boundary[count] = static_cast<uint8>(left);
        g_xy_y1_boundary[count] = static_cast<uint8>(y);
        g_xy_x2_boundary[count] = static_cast<uint8>(mid);
        g_xy_y2_boundary[count] = static_cast<uint8>(y);
        g_xy_x3_boundary[count] = static_cast<uint8>(right);
        g_xy_y3_boundary[count] = static_cast<uint8>(y);
        ++count;
    }
    g_boundary_count = count;
}

static void update_line_error()
{
    if (g_boundary_count <= 0)
    {
        // 没看到有效边界时，先把误差清零，避免沿用旧值造成盲目纠偏。
        line_error = 0;
        return;
    }

    int den = (line_sample_ratio_den <= 0) ? 1 : line_sample_ratio_den;
    int num = std::max(0, line_sample_ratio_num);

    // 根据比例选一条“控制采样横线”。
    // 你的巡线思路不是对整条边界做复杂拟合，而是选定一个关键高度，直接看该高度的中线偏差。
    const int target_y = std::min(UVC_HEIGHT - 1, (UVC_HEIGHT * num) / den);
    int best_idx = -1;
    int best_dy = UVC_HEIGHT + 1;

    for (int i = 0; i < g_boundary_count; ++i)
    {
        // 由于边界数组并不保证恰好包含 target_y 这一行，
        // 这里选择距离采样横线最近的一行来代表当前控制视角。
        int dy = std::abs(static_cast<int>(g_xy_y2_boundary[i]) - target_y);
        if (dy < best_dy)
        {
            best_dy = dy;
            best_idx = i;
        }
    }

    if (best_idx < 0)
    {
        line_error = 0;
        return;
    }

    int mid_x = static_cast<int>(g_xy_x2_boundary[best_idx]);
    // 这里给出的正负号定义是：
    // 正值 => 赛道中线在图像中心右侧；
    // 负值 => 赛道中线在图像中心左侧。
    // 巡线线程会在进入控制链时再统一成自己的历史方向约定。
    line_error = mid_x - (UVC_WIDTH / 2);
}

bool vision_image_processor_init(const char *camera_path)
{
    if (camera_path == nullptr || camera_path[0] == '\0')
    {
        camera_path = "/dev/video0";
    }
    return (uvc_camera_init(camera_path) == 0);
}

bool vision_image_processor_process_step()
{
    if (wait_image_refresh() < 0)
    {
        return false;
    }

    std::memcpy(g_image_gray, rgay_image, UVC_WIDTH * UVC_HEIGHT);

    cv::Mat gray(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, g_image_gray);
    cv::Mat binary(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, g_image_binary_u8);
    cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // 红色矩形坐标检测（HSV + 连通域）
    detect_red_rect_hsv(bgr_image, UVC_WIDTH, UVC_HEIGHT);

    std::vector<cv::Point> traced_points;
    trace_component_8n(g_image_binary_u8, UVC_WIDTH, UVC_HEIGHT, traced_points);
    fill_boundary_arrays(traced_points);
    update_line_error();
    //printf("[LINE] error=%d\n", line_error);
    return true;
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
    return bgr_image;
}

const uint8 *vision_image_processor_rgb565_image()
{
    return rgb565_image;
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
