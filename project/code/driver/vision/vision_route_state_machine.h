#ifndef VISION_ROUTE_STATE_MACHINE_H_
#define VISION_ROUTE_STATE_MACHINE_H_

#include "zf_common_headfile.h"

typedef enum
{
    VISION_ROUTE_MAIN_NORMAL = 0,
    VISION_ROUTE_MAIN_CIRCLE_LEFT,
    VISION_ROUTE_MAIN_CIRCLE_RIGHT,
    VISION_ROUTE_MAIN_CROSS
} vision_route_main_state_enum;

typedef enum
{
    VISION_ROUTE_SUB_NONE = 0,
    VISION_ROUTE_SUB_CROSS_BEGIN,
    VISION_ROUTE_SUB_CROSS_IN,
    VISION_ROUTE_SUB_CIRCLE_LEFT_BEGIN,
    VISION_ROUTE_SUB_CIRCLE_LEFT_IN,
    VISION_ROUTE_SUB_CIRCLE_LEFT_RUNNING,
    VISION_ROUTE_SUB_CIRCLE_LEFT_OUT,
    VISION_ROUTE_SUB_CIRCLE_LEFT_END,
    VISION_ROUTE_SUB_CIRCLE_RIGHT_BEGIN,
    VISION_ROUTE_SUB_CIRCLE_RIGHT_IN,
    VISION_ROUTE_SUB_CIRCLE_RIGHT_RUNNING,
    VISION_ROUTE_SUB_CIRCLE_RIGHT_OUT,
    VISION_ROUTE_SUB_CIRCLE_RIGHT_END
} vision_route_sub_state_enum;

typedef enum
{
    VISION_ROUTE_PREFERRED_SOURCE_AUTO = -1,
    VISION_ROUTE_PREFERRED_SOURCE_LEFT = 0,
    VISION_ROUTE_PREFERRED_SOURCE_RIGHT = 1
} vision_route_preferred_source_enum;

typedef struct
{
    int base_preferred_source;
    bool left_corner_found;
    bool right_corner_found;
    int left_corner_x;
    int left_corner_y;
    int left_corner_src_y;
    int left_corner_index;
    int right_corner_x;
    int right_corner_y;
    int right_corner_src_y;
    int right_corner_index;
    bool left_straight;
    bool right_straight;
    int cross_detected_stop_row;
    int left_boundary_count;
    int right_boundary_count;
    uint32 frame_encoder_delta;
} vision_route_state_input_t;

typedef struct
{
    vision_route_main_state_enum main_state;
    vision_route_sub_state_enum sub_state;
    int preferred_source;
    uint32 encoder_since_state_enter;
    int cross_loss_count;
    int left_loss_count;
    int left_gain_count;
    int right_loss_count;
    int right_gain_count;
} vision_route_state_snapshot_t;

void vision_route_state_machine_reset();
void vision_route_state_machine_update(const vision_route_state_input_t *input);
vision_route_state_snapshot_t vision_route_state_machine_snapshot();
vision_route_main_state_enum vision_route_state_machine_main_state();
vision_route_sub_state_enum vision_route_state_machine_sub_state();
int vision_route_state_machine_preferred_source();
uint32 vision_route_state_machine_encoder_since_state_enter();
int vision_route_state_machine_cross_loss_count();

#endif
