#include "driver/vision/vision_pipeline.h"

#include "driver/vision/vision_client_sender.h"
#include "driver/vision/vision_image_processor.h"
#include "driver/vision/vision_ncnn.h"

#include <opencv2/opencv.hpp>

#include <algorithm>

// ncnn输入ROI配置：按红色矩形长度线性缩放
// 约束点：len=15 -> roi=15, len=27 -> roi=45
static const float VISION_ROI_LEN_MIN = 15.0f;
static const float VISION_ROI_LEN_MAX = 27.0f;
static const float VISION_ROI_SIZE_MIN = 25.0f;
static const float VISION_ROI_SIZE_MAX = 45.0f;

static int vision_calc_dynamic_roi_size(int red_rect_len)
{
    float len = static_cast<float>(red_rect_len);
    if (len <= VISION_ROI_LEN_MIN) return static_cast<int>(VISION_ROI_SIZE_MIN);
    if (len >= VISION_ROI_LEN_MAX) return static_cast<int>(VISION_ROI_SIZE_MAX);

    float t = (len - VISION_ROI_LEN_MIN) / (VISION_ROI_LEN_MAX - VISION_ROI_LEN_MIN);
    float size = VISION_ROI_SIZE_MIN + t * (VISION_ROI_SIZE_MAX - VISION_ROI_SIZE_MIN);
    return static_cast<int>(size + 0.5f);
}

// 初始化视觉总线：图像处理 + ncnn推理 + 客户端发送
bool vision_pipeline_init(const char *camera_path, LQ_NCNN *ncnn, bool ncnn_enabled)
{
    if (!vision_image_processor_init(camera_path))
    {
        return false;
    }

    vision_ncnn_bind(ncnn, ncnn_enabled);
    vision_client_sender_init();
    return true;
}

// 仅做采图和处理，不做发送
bool vision_pipeline_process_step()
{
    if (!vision_image_processor_process_step())
    {
        return false;
    }
    vision_image_processor_set_ncnn_roi(false, 0, 0, 0, 0);

    bool found = false;
    int x = 0, y = 0, w = 0, h = 0, cx = 0, cy = 0;
    vision_image_processor_get_red_rect(&found, &x, &y, &w, &h, &cx, &cy);

    if (found)
    {
        if (vision_ncnn_is_enabled())
        {
            const uint8 *bgr_data = vision_image_processor_bgr_image();
            if (bgr_data != nullptr)
            {
                cv::Mat frame(UVC_HEIGHT, UVC_WIDTH, CV_8UC3, const_cast<uint8 *>(bgr_data));

                int red_rect_len = std::max(w, h);
                int roi_size = vision_calc_dynamic_roi_size(red_rect_len);
                int roi_w = std::min(roi_size, UVC_WIDTH);
                int roi_h = std::min(roi_size, UVC_HEIGHT);

                // 以红色矩形上边中点 (cx, y) 为中心取ROI
                int roi_x = cx - roi_w / 2;
                int roi_y = y - roi_h / 2;

                roi_x = std::max(0, std::min(roi_x, UVC_WIDTH - roi_w));
                roi_y = std::max(0, std::min(roi_y, UVC_HEIGHT - roi_h));

                vision_image_processor_set_ncnn_roi(true, roi_x, roi_y, roi_w, roi_h);
                cv::Rect roi(roi_x, roi_y, roi_w, roi_h);
                cv::Mat roi_bgr = frame(roi).clone();
                vision_ncnn_step(reinterpret_cast<const uint8 *>(roi_bgr.data), roi_bgr.cols, roi_bgr.rows);
            }
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

void vision_pipeline_get_red_rect(bool *found, int *x, int *y, int *w, int *h, int *cx, int *cy)
{
    vision_image_processor_get_red_rect(found, x, y, w, h, cx, cy);
}

int vision_pipeline_get_red_rect_area()
{
    return vision_image_processor_get_red_rect_area();
}
