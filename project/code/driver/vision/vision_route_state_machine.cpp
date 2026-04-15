#include "driver/vision/vision_route_state_machine.h"
#include "driver/vision/vision_config.h"

#include <algorithm>

namespace
{

static vision_route_main_state_enum g_main_state = VISION_ROUTE_MAIN_NORMAL;
static vision_route_sub_state_enum g_sub_state = VISION_ROUTE_SUB_NONE;
static int g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_AUTO;
static uint32 g_encoder_since_state_enter = 0U;
static int g_cross_loss_count = 0;
static int g_left_loss_count = 0;
static int g_left_gain_count = 0;
static int g_right_loss_count = 0;
static int g_right_gain_count = 0;
static int g_straight_ready_consecutive_count = 0;

static int normalize_preferred_source(int preferred_source)
{
    if (preferred_source == VISION_ROUTE_PREFERRED_SOURCE_LEFT)
    {
        return VISION_ROUTE_PREFERRED_SOURCE_LEFT;
    }

    if (preferred_source == VISION_ROUTE_PREFERRED_SOURCE_RIGHT)
    {
        return VISION_ROUTE_PREFERRED_SOURCE_RIGHT;
    }

    return VISION_ROUTE_PREFERRED_SOURCE_AUTO;
}

static void clear_runtime_counters()
{
    g_encoder_since_state_enter = 0U;
    g_cross_loss_count = 0;
    g_left_loss_count = 0;
    g_left_gain_count = 0;
    g_right_loss_count = 0;
    g_right_gain_count = 0;
    g_straight_ready_consecutive_count = 0;
}

static void enter_state(vision_route_main_state_enum main_state,
                        vision_route_sub_state_enum sub_state,
                        int preferred_source)
{
    g_main_state = main_state;
    g_sub_state = sub_state;
    g_preferred_source = normalize_preferred_source(preferred_source);
    clear_runtime_counters();
}

static bool left_circle_entry_ready(const vision_route_state_input_t *input)
{
    if (input == nullptr)
    {
        return false;
    }

    return input->right_straight &&
           input->left_corner_found &&
           input->left_corner_y > 60 &&
           input->left_corner_index >= 0 &&
           input->right_boundary_count > g_vision_runtime_config.route_circle_entry_min_boundary_count &&
           input->left_circle_entry_raw_gap_ok &&
           input->left_corner_index < (input->right_boundary_count - g_vision_runtime_config.route_circle_entry_corner_tail_margin);
}

static bool right_circle_entry_ready(const vision_route_state_input_t *input)
{
    if (input == nullptr)
    {
        return false;
    }

    return input->left_straight &&
           input->right_corner_found &&
           input->right_corner_y > 60 &&
           input->right_corner_index >= 0 &&
           input->left_boundary_count > g_vision_runtime_config.route_circle_entry_min_boundary_count &&
           input->right_circle_entry_raw_gap_ok &&
           input->right_corner_index < (input->left_boundary_count - g_vision_runtime_config.route_circle_entry_corner_tail_margin);
}

static bool straight_state_ready(const vision_route_state_input_t *input)
{
    if (input == nullptr)
    {
        return false;
    }

    return input->straight_required_last_index >= 0 &&
           input->selected_centerline_count > input->straight_required_last_index &&
           input->straight_abs_error_sum < g_vision_runtime_config.route_straight_abs_error_sum_max;
}

} // namespace

void vision_route_state_machine_reset()
{
    g_main_state = VISION_ROUTE_MAIN_NORMAL;
    g_sub_state = VISION_ROUTE_SUB_NONE;
    g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_AUTO;
    clear_runtime_counters();
}

void vision_route_state_machine_update(const vision_route_state_input_t *input)
{
    if (input == nullptr)
    {
        return;
    }

    g_encoder_since_state_enter += input->frame_encoder_delta;

    if (!g_vision_runtime_config.route_circle_detection_enabled &&
        (g_main_state == VISION_ROUTE_MAIN_CIRCLE_LEFT || g_main_state == VISION_ROUTE_MAIN_CIRCLE_RIGHT))
    {
        enter_state(VISION_ROUTE_MAIN_NORMAL, VISION_ROUTE_SUB_NONE, input->base_preferred_source);
    }

    switch (g_main_state)
    {
    case VISION_ROUTE_MAIN_NORMAL:
        g_preferred_source = normalize_preferred_source(input->base_preferred_source);
        if (straight_state_ready(input))
        {
            g_straight_ready_consecutive_count += 1;
            if (g_straight_ready_consecutive_count >=
                std::max(1, g_vision_runtime_config.route_straight_enter_consecutive_frames))
            {
                enter_state(VISION_ROUTE_MAIN_STRAIGHT,
                            VISION_ROUTE_SUB_NONE,
                            input->base_preferred_source);
            }
        }
        else
        {
            g_straight_ready_consecutive_count = 0;
            if (g_vision_runtime_config.route_circle_detection_enabled)
            {
                if (left_circle_entry_ready(input))
                {
                    enter_state(VISION_ROUTE_MAIN_CIRCLE_LEFT,
                                VISION_ROUTE_SUB_CIRCLE_LEFT_1,
                                VISION_ROUTE_PREFERRED_SOURCE_RIGHT);
                }
                else if (right_circle_entry_ready(input))
                {
                    enter_state(VISION_ROUTE_MAIN_CIRCLE_RIGHT,
                                VISION_ROUTE_SUB_CIRCLE_RIGHT_1,
                                VISION_ROUTE_PREFERRED_SOURCE_LEFT);
                }
            }
        }
        break;
    case VISION_ROUTE_MAIN_STRAIGHT:
        g_preferred_source = normalize_preferred_source(input->base_preferred_source);
        if (!straight_state_ready(input))
        {
            enter_state(VISION_ROUTE_MAIN_NORMAL,
                        VISION_ROUTE_SUB_NONE,
                        input->base_preferred_source);
        }
        break;
    case VISION_ROUTE_MAIN_CIRCLE_LEFT:
        g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_RIGHT;
        switch (g_sub_state)
        {
        case VISION_ROUTE_SUB_CIRCLE_LEFT_1:
            if (input->left_start_frame_wall_rows >= g_vision_runtime_config.route_circle_stage_frame_wall_rows_enter)
            {
                enter_state(VISION_ROUTE_MAIN_CIRCLE_LEFT,
                            VISION_ROUTE_SUB_CIRCLE_LEFT_2,
                            VISION_ROUTE_PREFERRED_SOURCE_RIGHT);
            }
            break;
        case VISION_ROUTE_SUB_CIRCLE_LEFT_2:
            if (input->left_start_frame_wall_rows <= 0)
            {
                enter_state(VISION_ROUTE_MAIN_CIRCLE_LEFT,
                            VISION_ROUTE_SUB_CIRCLE_LEFT_3,
                            VISION_ROUTE_PREFERRED_SOURCE_RIGHT);
            }
            break;
        case VISION_ROUTE_SUB_CIRCLE_LEFT_3:
            g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_LEFT;
            if (input->right_start_frame_wall_rows > g_vision_runtime_config.route_circle_stage3_frame_wall_rows_trigger)
            {
                enter_state(VISION_ROUTE_MAIN_CIRCLE_LEFT,
                            VISION_ROUTE_SUB_CIRCLE_LEFT_4,
                            VISION_ROUTE_PREFERRED_SOURCE_RIGHT);
            }
            break;
        case VISION_ROUTE_SUB_CIRCLE_LEFT_4:
            g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_LEFT;
            if (input->right_corner_found)
            {
                enter_state(VISION_ROUTE_MAIN_CIRCLE_LEFT,
                            VISION_ROUTE_SUB_CIRCLE_LEFT_5,
                            VISION_ROUTE_PREFERRED_SOURCE_RIGHT);
            }
            break;
        case VISION_ROUTE_SUB_CIRCLE_LEFT_5:
            g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_LEFT;
            if (input->right_start_frame_wall_rows >= g_vision_runtime_config.route_circle_stage_frame_wall_rows_enter)
            {
                enter_state(VISION_ROUTE_MAIN_CIRCLE_LEFT,
                            VISION_ROUTE_SUB_CIRCLE_LEFT_6,
                            VISION_ROUTE_PREFERRED_SOURCE_RIGHT);
            }
            break;
        case VISION_ROUTE_SUB_CIRCLE_LEFT_6:
            if (input->left_straight && input->right_straight)
            {
                enter_state(VISION_ROUTE_MAIN_NORMAL,
                            VISION_ROUTE_SUB_NONE,
                            input->base_preferred_source);
            }
            break;
        default:
            enter_state(VISION_ROUTE_MAIN_CIRCLE_LEFT,
                        VISION_ROUTE_SUB_CIRCLE_LEFT_1,
                        VISION_ROUTE_PREFERRED_SOURCE_RIGHT);
            break;
        }
        break;
    case VISION_ROUTE_MAIN_CIRCLE_RIGHT:
        g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_LEFT;
        switch (g_sub_state)
        {
        case VISION_ROUTE_SUB_CIRCLE_RIGHT_1:
            if (input->right_start_frame_wall_rows >= g_vision_runtime_config.route_circle_stage_frame_wall_rows_enter)
            {
                enter_state(VISION_ROUTE_MAIN_CIRCLE_RIGHT,
                            VISION_ROUTE_SUB_CIRCLE_RIGHT_2,
                            VISION_ROUTE_PREFERRED_SOURCE_LEFT);
            }
            break;
        case VISION_ROUTE_SUB_CIRCLE_RIGHT_2:
            if (input->right_start_frame_wall_rows <= 0)
            {
                enter_state(VISION_ROUTE_MAIN_CIRCLE_RIGHT,
                            VISION_ROUTE_SUB_CIRCLE_RIGHT_3,
                            VISION_ROUTE_PREFERRED_SOURCE_LEFT);
            }
            break;
        case VISION_ROUTE_SUB_CIRCLE_RIGHT_3:
            g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_RIGHT;
            if (input->left_start_frame_wall_rows > g_vision_runtime_config.route_circle_stage3_frame_wall_rows_trigger)
            {
                enter_state(VISION_ROUTE_MAIN_CIRCLE_RIGHT,
                            VISION_ROUTE_SUB_CIRCLE_RIGHT_4,
                            VISION_ROUTE_PREFERRED_SOURCE_LEFT);
            }
            break;
        case VISION_ROUTE_SUB_CIRCLE_RIGHT_4:
            g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_RIGHT;
            if (input->left_corner_found)
            {
                enter_state(VISION_ROUTE_MAIN_CIRCLE_RIGHT,
                            VISION_ROUTE_SUB_CIRCLE_RIGHT_5,
                            VISION_ROUTE_PREFERRED_SOURCE_LEFT);
            }
            break;
        case VISION_ROUTE_SUB_CIRCLE_RIGHT_5:
            g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_RIGHT;
            if (input->left_start_frame_wall_rows >= g_vision_runtime_config.route_circle_stage_frame_wall_rows_enter)
            {
                enter_state(VISION_ROUTE_MAIN_CIRCLE_RIGHT,
                            VISION_ROUTE_SUB_CIRCLE_RIGHT_6,
                            VISION_ROUTE_PREFERRED_SOURCE_LEFT);
            }
            break;
        case VISION_ROUTE_SUB_CIRCLE_RIGHT_6:
            if (input->left_straight && input->right_straight)
            {
                enter_state(VISION_ROUTE_MAIN_NORMAL,
                            VISION_ROUTE_SUB_NONE,
                            input->base_preferred_source);
            }
            break;
        default:
            enter_state(VISION_ROUTE_MAIN_CIRCLE_RIGHT,
                        VISION_ROUTE_SUB_CIRCLE_RIGHT_1,
                        VISION_ROUTE_PREFERRED_SOURCE_LEFT);
            break;
        }
        break;
    case VISION_ROUTE_MAIN_CROSS:
    default:
        g_preferred_source = normalize_preferred_source(input->base_preferred_source);
        break;
    }
}

vision_route_state_snapshot_t vision_route_state_machine_snapshot()
{
    vision_route_state_snapshot_t snapshot{};
    snapshot.main_state = g_main_state;
    snapshot.sub_state = g_sub_state;
    snapshot.preferred_source = g_preferred_source;
    snapshot.encoder_since_state_enter = g_encoder_since_state_enter;
    snapshot.cross_loss_count = g_cross_loss_count;
    snapshot.left_loss_count = g_left_loss_count;
    snapshot.left_gain_count = g_left_gain_count;
    snapshot.right_loss_count = g_right_loss_count;
    snapshot.right_gain_count = g_right_gain_count;
    return snapshot;
}

vision_route_main_state_enum vision_route_state_machine_main_state()
{
    return g_main_state;
}

vision_route_sub_state_enum vision_route_state_machine_sub_state()
{
    return g_sub_state;
}

int vision_route_state_machine_preferred_source()
{
    return g_preferred_source;
}

uint32 vision_route_state_machine_encoder_since_state_enter()
{
    return g_encoder_since_state_enter;
}

int vision_route_state_machine_cross_loss_count()
{
    return g_cross_loss_count;
}
