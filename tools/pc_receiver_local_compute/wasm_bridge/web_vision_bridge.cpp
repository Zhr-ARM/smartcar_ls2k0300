#include "web_vision_bridge.h"

bool web_vision_bridge_process(const web_vision_frame_t *frame,
                               const web_vision_raw_status_t *status,
                               web_vision_result_t *result)
{
    if (result == nullptr)
    {
        return false;
    }

    result->line_error = 0;
    result->valid = false;

    if (frame == nullptr || status == nullptr || frame->data == nullptr || frame->data_size == 0)
    {
        return false;
    }

    // 当前只放一个最小桩实现，用来固定接口。
    // 真正落地时，这里应改成：
    // - 调用 tools/ 下的适配层
    // - 由适配层只读引用 project/ 中现有算法源码
    // - 编译为 WebAssembly 后供浏览器 worker 调用
    result->line_error = 0;
    result->valid = true;
    return true;
}
