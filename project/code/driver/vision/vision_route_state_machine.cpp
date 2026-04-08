#include "driver/vision/vision_route_state_machine.h"
#include "driver/vision/vision_config.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
static constexpr int kCircleEdgeVisibleMinCount = 2;
static constexpr int kCircleLossTrigger = 3;
static constexpr int kCircleGainTrigger = 3;
static constexpr uint32 kCircleEncoderSwitchThreshold = 40000;
static constexpr int kDualStraightEscapeFrames = 5;
static constexpr int kCircleEntryWindowFrames = 10;
static constexpr int kCircleEntryHitThreshold = 8;

static vision_route_main_state_enum g_main_state = VISION_ROUTE_MAIN_NORMAL;
static vision_route_sub_state_enum g_sub_state = VISION_ROUTE_SUB_NONE;
static int g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_AUTO;
static uint32 g_encoder_since_state_enter = 0U;
static int g_cross_loss_count = 0;
static bool g_cross_begin_to_in_pending = false;
static int g_left_loss_count = 0;
static int g_left_gain_count = 0;
static int g_left_corner_count = 0;
static int g_right_loss_count = 0;
static int g_right_gain_count = 0;
static int g_right_corner_count = 0;
static int g_dual_straight_count = 0;
static int g_left_circle_entry_window = 0;
static int g_left_circle_entry_hits = 0;
static int g_left_circle_entry_history[kCircleEntryWindowFrames] = {0};
static int g_left_circle_entry_history_index = 0;
static int g_right_circle_entry_window = 0;
static int g_right_circle_entry_hits = 0;
static int g_right_circle_entry_history[kCircleEntryWindowFrames] = {0};
static int g_right_circle_entry_history_index = 0;

static inline int safe_count(int count)
{
    return std::max(0, count);
}

static inline int corner_distance_px(const vision_route_state_input_t &input)
{
    if (!input.left_corner_found || !input.right_corner_found)
    {
        return std::numeric_limits<int>::max();
    }

    const int dx = input.left_corner_x - input.right_corner_x;
    const int dy = input.left_corner_y - input.right_corner_y;
    return static_cast<int>(std::lround(std::sqrt(static_cast<double>(dx * dx + dy * dy))));
}

static inline bool corner_index_valid(int index)
{
    return index >= 0 && index <= 35;
}

static void update_sliding_window(int *history, int &history_index, int &window_sum, bool hit)
{
    if (history == nullptr)
    {
        return;
    }

    window_sum -= history[history_index];
    history[history_index] = hit ? 1 : 0;
    window_sum += history[history_index];
    history_index = (history_index + 1) % kCircleEntryWindowFrames;
}

static void reset_circle_counters_left()
{
    g_left_loss_count = 0;
    g_left_gain_count = 0;
    g_left_corner_count = 0;
    g_left_circle_entry_window = 0;
    g_left_circle_entry_hits = 0;
    g_left_circle_entry_history_index = 0;
    for (int &value : g_left_circle_entry_history)
    {
        value = 0;
    }
}

static void reset_circle_counters_right()
{
    g_right_loss_count = 0;
    g_right_gain_count = 0;
    g_right_corner_count = 0;
    g_right_circle_entry_window = 0;
    g_right_circle_entry_hits = 0;
    g_right_circle_entry_history_index = 0;
    for (int &value : g_right_circle_entry_history)
    {
        value = 0;
    }
}

static void reset_circle_entry_windows()
{
    g_left_circle_entry_window = 0;
    g_left_circle_entry_hits = 0;
    g_left_circle_entry_history_index = 0;
    for (int &value : g_left_circle_entry_history)
    {
        value = 0;
    }

    g_right_circle_entry_window = 0;
    g_right_circle_entry_hits = 0;
    g_right_circle_entry_history_index = 0;
    for (int &value : g_right_circle_entry_history)
    {
        value = 0;
    }
}

static void enter_main_state(vision_route_main_state_enum main_state,
                             vision_route_sub_state_enum sub_state,
                             int preferred_source)
{
    g_main_state = main_state;
    g_sub_state = sub_state;
    g_preferred_source = preferred_source;
    g_encoder_since_state_enter = 0U;
    g_cross_loss_count = 0;
    g_cross_begin_to_in_pending = false;
    reset_circle_counters_left();
    reset_circle_counters_right();
    reset_circle_entry_windows();
    g_dual_straight_count = 0;
}

static void set_circle_left_state(vision_route_sub_state_enum sub_state, int preferred_source)
{
    g_main_state = VISION_ROUTE_MAIN_CIRCLE_LEFT;
    g_sub_state = sub_state;
    g_preferred_source = preferred_source;
    g_encoder_since_state_enter = 0U;
    g_cross_loss_count = 0;
    reset_circle_counters_right();
    reset_circle_entry_windows();
}

static void set_circle_right_state(vision_route_sub_state_enum sub_state, int preferred_source)
{
    g_main_state = VISION_ROUTE_MAIN_CIRCLE_RIGHT;
    g_sub_state = sub_state;
    g_preferred_source = preferred_source;
    g_encoder_since_state_enter = 0U;
    g_cross_loss_count = 0;
    reset_circle_counters_left();
    reset_circle_entry_windows();
}

static void step_circle_left(const vision_route_state_input_t &input)
{
    const int left_count = safe_count(input.left_boundary_count);

    switch (g_sub_state)
    {
        case VISION_ROUTE_SUB_CIRCLE_LEFT_BEGIN:
            g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_RIGHT;
            if (left_count <= 0)
            {
                ++g_left_loss_count;
            }
            else if (g_left_loss_count > kCircleLossTrigger)
            {
                if (left_count >= kCircleEdgeVisibleMinCount)
                {
                    ++g_left_gain_count;
                }
                else
                {
                    g_left_gain_count = 0;
                }

                if (g_left_gain_count > kCircleGainTrigger)
                {
                    g_sub_state = VISION_ROUTE_SUB_CIRCLE_LEFT_IN;
                    g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_LEFT;
                    g_encoder_since_state_enter = 0U;
                    g_left_gain_count = 0;
                }
            }
            else
            {
                g_left_gain_count = 0;
            }
            break;

        case VISION_ROUTE_SUB_CIRCLE_LEFT_IN:
            g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_LEFT;
            g_encoder_since_state_enter += input.frame_encoder_delta;
            if (g_encoder_since_state_enter > kCircleEncoderSwitchThreshold || left_count <= kCircleEdgeVisibleMinCount)
            {
                g_sub_state = VISION_ROUTE_SUB_CIRCLE_LEFT_RUNNING;
                g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_RIGHT;
                g_encoder_since_state_enter = 0U;
            }
            break;

        case VISION_ROUTE_SUB_CIRCLE_LEFT_RUNNING:
            g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_RIGHT;
            if (input.right_corner_found && corner_index_valid(input.right_corner_index))
            {
                ++g_left_corner_count;
                if (g_left_corner_count >= 3)
                {
                    g_sub_state = VISION_ROUTE_SUB_CIRCLE_LEFT_OUT;
                    g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_LEFT;
                    g_left_corner_count = 0;
                }
            }
            break;

        case VISION_ROUTE_SUB_CIRCLE_LEFT_OUT:
            g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_LEFT;
            if (input.right_straight)
            {
                g_sub_state = VISION_ROUTE_SUB_CIRCLE_LEFT_END;
                g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_RIGHT;
            }
            break;

        case VISION_ROUTE_SUB_CIRCLE_LEFT_END:
            g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_RIGHT;
            if (input.left_straight && input.right_straight)
            {
                enter_main_state(VISION_ROUTE_MAIN_NORMAL, VISION_ROUTE_SUB_NONE, input.base_preferred_source);
            }
            break;

        default:
            set_circle_left_state(VISION_ROUTE_SUB_CIRCLE_LEFT_BEGIN, VISION_ROUTE_PREFERRED_SOURCE_RIGHT);
            break;
    }
}

static void step_circle_right(const vision_route_state_input_t &input)
{
    const int right_count = safe_count(input.right_boundary_count);

    switch (g_sub_state)
    {
        case VISION_ROUTE_SUB_CIRCLE_RIGHT_BEGIN:
            g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_LEFT;
            if (right_count <= 0)
            {
                ++g_right_loss_count;
            }
            else if (g_right_loss_count > kCircleLossTrigger)
            {
                if (right_count >= kCircleEdgeVisibleMinCount)
                {
                    ++g_right_gain_count;
                }
                else
                {
                    g_right_gain_count = 0;
                }

                if (g_right_gain_count > kCircleGainTrigger)
                {
                    g_sub_state = VISION_ROUTE_SUB_CIRCLE_RIGHT_IN;
                    g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_RIGHT;
                    g_encoder_since_state_enter = 0U;
                    g_right_gain_count = 0;
                }
            }
            else
            {
                g_right_gain_count = 0;
            }
            break;

        case VISION_ROUTE_SUB_CIRCLE_RIGHT_IN:
            g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_RIGHT;
            g_encoder_since_state_enter += input.frame_encoder_delta;
            if (g_encoder_since_state_enter > kCircleEncoderSwitchThreshold || right_count <= kCircleEdgeVisibleMinCount)
            {
                g_sub_state = VISION_ROUTE_SUB_CIRCLE_RIGHT_RUNNING;
                g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_LEFT;
                g_encoder_since_state_enter = 0U;
            }
            break;

        case VISION_ROUTE_SUB_CIRCLE_RIGHT_RUNNING:
            g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_LEFT;
            if (input.left_corner_found && corner_index_valid(input.left_corner_index))
            {
                ++g_right_corner_count;
                if (g_right_corner_count >= 3)
                {
                    g_sub_state = VISION_ROUTE_SUB_CIRCLE_RIGHT_OUT;
                    g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_RIGHT;
                    g_right_corner_count = 0;
                }
            }
            break;

        case VISION_ROUTE_SUB_CIRCLE_RIGHT_OUT:
            g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_RIGHT;
            if (input.left_straight)
            {
                g_sub_state = VISION_ROUTE_SUB_CIRCLE_RIGHT_END;
                g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_LEFT;
            }
            break;

        case VISION_ROUTE_SUB_CIRCLE_RIGHT_END:
            g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_LEFT;
            if (input.left_straight && input.right_straight)
            {
                enter_main_state(VISION_ROUTE_MAIN_NORMAL, VISION_ROUTE_SUB_NONE, input.base_preferred_source);
            }
            break;

        default:
            set_circle_right_state(VISION_ROUTE_SUB_CIRCLE_RIGHT_BEGIN, VISION_ROUTE_PREFERRED_SOURCE_LEFT);
            break;
    }
}

} // namespace

void vision_route_state_machine_reset()
{
    g_main_state = VISION_ROUTE_MAIN_NORMAL;
    g_sub_state = VISION_ROUTE_SUB_NONE;
    g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_AUTO;
    g_encoder_since_state_enter = 0U;
    g_cross_loss_count = 0;
    g_cross_begin_to_in_pending = false;
    g_left_loss_count = 0;
    g_left_gain_count = 0;
    g_left_corner_count = 0;
    g_right_loss_count = 0;
    g_right_gain_count = 0;
    g_right_corner_count = 0;
    g_dual_straight_count = 0;
    g_left_circle_entry_window = 0;
    g_left_circle_entry_hits = 0;
    g_right_circle_entry_window = 0;
    g_right_circle_entry_hits = 0;
    g_cross_begin_to_in_pending = false;
}

void vision_route_state_machine_update(const vision_route_state_input_t *input)
{
    if (input == nullptr)
    {
        return;
    }

    const bool cross_enabled = g_vision_runtime_config.route_cross_enabled;
    const int cross_corner_distance_threshold_px = vision_runtime_config_route_cross_corner_distance_px();
    const int cross_begin_corner_src_y_threshold_px = vision_runtime_config_route_cross_begin_corner_src_y_px();
    const int cross_stop_row_exit_threshold_px = vision_runtime_config_route_cross_stop_row_exit_px();

    const bool left_corner_found = input->left_corner_found;
    const bool right_corner_found = input->right_corner_found;
    const bool left_corner_valid = left_corner_found && corner_index_valid(input->left_corner_index);
    const bool right_corner_valid = right_corner_found && corner_index_valid(input->right_corner_index);
    const bool both_straight = input->left_straight && input->right_straight;
    const int corner_distance = corner_distance_px(*input);
    const bool left_circle_entry_hit = left_corner_valid && input->right_straight;
    const bool right_circle_entry_hit = right_corner_valid && input->left_straight;

    if (both_straight)
    {
        ++g_dual_straight_count;
    }
    else
    {
        g_dual_straight_count = 0;
    }

    if (g_dual_straight_count >= kDualStraightEscapeFrames)
    {
        if (g_main_state != VISION_ROUTE_MAIN_NORMAL || g_sub_state != VISION_ROUTE_SUB_NONE)
        {
            enter_main_state(VISION_ROUTE_MAIN_NORMAL, VISION_ROUTE_SUB_NONE, input->base_preferred_source);
        }
        return;
    }

    g_encoder_since_state_enter += input->frame_encoder_delta;

    if (g_main_state == VISION_ROUTE_MAIN_NORMAL)
    {
        g_preferred_source = input->base_preferred_source;

        update_sliding_window(g_left_circle_entry_history,
                              g_left_circle_entry_history_index,
                              g_left_circle_entry_window,
                              left_circle_entry_hit);
        update_sliding_window(g_right_circle_entry_history,
                              g_right_circle_entry_history_index,
                              g_right_circle_entry_window,
                              right_circle_entry_hit);

        if (cross_enabled && left_corner_valid && right_corner_valid && corner_distance < cross_corner_distance_threshold_px)
        {
            enter_main_state(VISION_ROUTE_MAIN_CROSS, VISION_ROUTE_SUB_CROSS_BEGIN, input->base_preferred_source);
            return;
        }

        if (g_left_circle_entry_window >= kCircleEntryHitThreshold)
        {
            set_circle_left_state(VISION_ROUTE_SUB_CIRCLE_LEFT_BEGIN, VISION_ROUTE_PREFERRED_SOURCE_RIGHT);
            return;
        }

        if (g_right_circle_entry_window >= kCircleEntryHitThreshold)
        {
            set_circle_right_state(VISION_ROUTE_SUB_CIRCLE_RIGHT_BEGIN, VISION_ROUTE_PREFERRED_SOURCE_LEFT);
            return;
        }

        return;
    }

    if (g_main_state == VISION_ROUTE_MAIN_CROSS)
    {
        if (!cross_enabled)
        {
            enter_main_state(VISION_ROUTE_MAIN_NORMAL, VISION_ROUTE_SUB_NONE, input->base_preferred_source);
            return;
        }

        g_preferred_source = input->base_preferred_source;
        const bool left_corner_src_ready = left_corner_valid && (input->left_corner_src_y > cross_begin_corner_src_y_threshold_px);
        const bool right_corner_src_ready = right_corner_valid && (input->right_corner_src_y > cross_begin_corner_src_y_threshold_px);

        if (g_sub_state == VISION_ROUTE_SUB_NONE)
        {
            g_sub_state = VISION_ROUTE_SUB_CROSS_BEGIN;
            g_cross_begin_to_in_pending = false;
        }

        if (g_sub_state == VISION_ROUTE_SUB_CROSS_BEGIN)
        {
            g_cross_loss_count = 0;
            if (g_cross_begin_to_in_pending)
            {
                g_sub_state = VISION_ROUTE_SUB_CROSS_IN;
                g_cross_begin_to_in_pending = false;
                return;
            }

            if (left_corner_src_ready || right_corner_src_ready)
            {
                g_cross_begin_to_in_pending = true;
            }
            return;
        }

        if (g_sub_state == VISION_ROUTE_SUB_CROSS_IN)
        {
            g_cross_loss_count = std::max(input->cross_detected_stop_row, 0);
            if (input->cross_detected_stop_row >= cross_stop_row_exit_threshold_px)
            {
                enter_main_state(VISION_ROUTE_MAIN_NORMAL, VISION_ROUTE_SUB_NONE, input->base_preferred_source);
            }
            return;
        }

        g_sub_state = VISION_ROUTE_SUB_CROSS_BEGIN;
        g_cross_begin_to_in_pending = false;
        g_cross_loss_count = 0;
        return;
    }

    if (g_main_state == VISION_ROUTE_MAIN_CIRCLE_LEFT)
    {
        step_circle_left(*input);
        return;
    }

    if (g_main_state == VISION_ROUTE_MAIN_CIRCLE_RIGHT)
    {
        step_circle_right(*input);
        return;
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
