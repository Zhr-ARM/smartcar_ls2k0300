#ifndef VISION_FRAME_CAPTURE_H_
#define VISION_FRAME_CAPTURE_H_

#include "zf_common_headfile.h"

#include <cstddef>

// 作用：初始化采图模块并启动采图线程（当前由 vision_image_processor_init 调用）。
// 意义：将“采图”与“图像处理”解耦，后续可单独替换采图驱动。
// 如何修改：可修改 camera_path（如 /dev/video2）切换相机设备。
// 是否调用：是，主链必调（通过 vision_image_processor_init 间接调用）。
bool vision_frame_capture_init(const char *camera_path);

// 作用：停止采图线程并清理采图模块状态。
// 意义：保证程序退出时不会遗留采图线程或阻塞等待。
// 如何修改：通常无需改；如新增底层 close 接口，可在实现里补充。
// 是否调用：是，主链退出时调用（通过 vision_image_processor_cleanup 间接调用）。
void vision_frame_capture_cleanup();

// 作用：阻塞等待新帧并拷贝到 out_bgr。
// 意义：为处理层提供“最新帧 + 可统计等待耗时”的统一入口。
// 如何修改：
// - out_bgr_bytes 需至少为 UVC_WIDTH * UVC_HEIGHT * 3；
// - timeout_ms 可调等待超时（大值更稳，小值更实时）；
// - wait_us 返回本次等待耗时（可用于性能上报）。
// 是否调用：是，每帧调用（由 vision_image_processor_process_step 调用）。
bool vision_frame_capture_wait_next_bgr(uint8 *out_bgr, size_t out_bgr_bytes, uint32 timeout_ms, uint32 *wait_us);

// 作用：读取采图线程最近 1 秒窗口统计得到的采图帧率。
// 返回：frames per second（整数，0 表示当前无有效统计）。
uint32 vision_frame_capture_fps();

#endif
