#include "driver/vision/vision_pipeline.h"

#include "driver/vision/vision_image_processor.h"
#include "driver/vision/vision_infer_async.h"
#include "driver/vision/vision_transport.h"

#include <opencv2/opencv.hpp>

namespace
{
static constexpr int kProcWidth = VISION_DOWNSAMPLED_WIDTH;
static constexpr int kProcHeight = VISION_DOWNSAMPLED_HEIGHT;
static constexpr int kFullWidth = UVC_WIDTH;
static constexpr int kFullHeight = UVC_HEIGHT;

static void clear_infer_result_in_image_processor()
{
    vision_image_processor_set_last_red_detect_us(0);
    vision_image_processor_set_red_rect(false, 0, 0, 0, 0, 0, 0, 0);
    vision_image_processor_set_ncnn_roi(false, 0, 0, 0, 0);
}

static void apply_infer_result_to_image(vision_infer_async_result_t *result)
{
    if (result == nullptr)
    {
        clear_infer_result_in_image_processor();
        return;
    }

    vision_image_processor_set_last_red_detect_us(result->red_detect_us);
    if (!result->found)
    {
        vision_image_processor_set_red_rect(false, 0, 0, 0, 0, 0, 0, 0);
        vision_image_processor_set_ncnn_roi(false, 0, 0, 0, 0);
        return;
    }

    vision_image_processor_set_red_rect(true,
                                        result->red_x,
                                        result->red_y,
                                        result->red_w,
                                        result->red_h,
                                        result->red_cx,
                                        result->red_cy,
                                        result->red_area);

    if (!result->ncnn_roi_valid)
    {
        vision_image_processor_set_ncnn_roi(false, 0, 0, 0, 0);
        return;
    }

    vision_image_processor_set_ncnn_roi(true,
                                        result->ncnn_roi_x,
                                        result->ncnn_roi_y,
                                        result->ncnn_roi_w,
                                        result->ncnn_roi_h);

    const uint8 *bgr_proc_data = vision_image_processor_bgr_image();
    const uint8 *gray_data = vision_image_processor_gray_image();
    if (bgr_proc_data != nullptr)
    {
        cv::Mat proc_frame(kProcHeight, kProcWidth, CV_8UC3, const_cast<uint8 *>(bgr_proc_data));
        cv::rectangle(proc_frame,
                      cv::Rect(result->ncnn_roi_x, result->ncnn_roi_y, result->ncnn_roi_w, result->ncnn_roi_h),
                      cv::Scalar(0, 255, 0),
                      1,
                      cv::LINE_8);
    }
    if (gray_data != nullptr)
    {
        cv::Mat gray(kProcHeight, kProcWidth, CV_8UC1, const_cast<uint8 *>(gray_data));
        cv::rectangle(gray,
                      cv::Rect(result->ncnn_roi_x, result->ncnn_roi_y, result->ncnn_roi_w, result->ncnn_roi_h),
                      cv::Scalar(255),
                      1,
                      cv::LINE_8);
    }
}

} // namespace

bool vision_pipeline_init(const char *camera_path, LQ_NCNN *ncnn, bool ncnn_enabled)
{
    if (!vision_image_processor_init(camera_path))
    {
        return false;
    }

    if (!vision_infer_async_init(ncnn, ncnn_enabled))
    {
        vision_image_processor_cleanup();
        return false;
    }

    vision_transport_init();
    clear_infer_result_in_image_processor();
    return true;
}

void vision_pipeline_cleanup()
{
    vision_infer_async_cleanup();
    clear_infer_result_in_image_processor();
    vision_image_processor_cleanup();
}

bool vision_pipeline_process_step()
{
    if (!vision_image_processor_process_step())
    {
        return false;
    }

    const uint8 *bgr_proc_data = vision_image_processor_bgr_image();
    const uint8 *bgr_full_data = vision_image_processor_bgr_full_image();
    if (!vision_infer_async_enabled() || bgr_proc_data == nullptr || bgr_full_data == nullptr)
    {
        clear_infer_result_in_image_processor();
        return true;
    }

    vision_infer_async_submit_frame(bgr_proc_data,
                                    kProcWidth,
                                    kProcHeight,
                                    bgr_full_data,
                                    kFullWidth,
                                    kFullHeight);

    vision_infer_async_result_t latest{};
    if (!vision_infer_async_fetch_latest(&latest))
    {
        clear_infer_result_in_image_processor();
        return true;
    }

    apply_infer_result_to_image(&latest);
    return true;
}

void vision_pipeline_send_step()
{
    vision_transport_send_step();
}

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
    vision_transport_set_send_mode(mode);
}

vision_send_mode_enum vision_pipeline_get_send_mode()
{
    return vision_transport_get_send_mode();
}

void vision_pipeline_set_send_max_fps(uint32 max_fps)
{
    vision_transport_set_send_max_fps(max_fps);
}

uint32 vision_pipeline_get_send_max_fps()
{
    return vision_transport_get_send_max_fps();
}

void vision_pipeline_set_send_enabled(bool enabled)
{
    vision_transport_set_send_enabled(enabled);
}

bool vision_pipeline_is_send_enabled()
{
    return vision_transport_is_send_enabled();
}

void vision_pipeline_set_infer_enabled(bool enabled)
{
    vision_infer_async_set_enabled(enabled);
    if (!enabled)
    {
        clear_infer_result_in_image_processor();
    }
}

bool vision_pipeline_infer_enabled()
{
    return vision_infer_async_enabled();
}

void vision_pipeline_set_roi_capture_mode(bool enabled)
{
    vision_infer_async_set_roi_capture_mode(enabled);
}

bool vision_pipeline_roi_capture_mode_enabled()
{
    return vision_infer_async_roi_capture_mode_enabled();
}

void vision_pipeline_get_red_rect(bool *found, int *x, int *y, int *w, int *h, int *cx, int *cy)
{
    vision_image_processor_get_red_rect(found, x, y, w, h, cx, cy);
}

int vision_pipeline_get_red_rect_area()
{
    return vision_image_processor_get_red_rect_area();
}
