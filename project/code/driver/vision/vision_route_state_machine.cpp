#include "driver/vision/vision_route_state_machine.h"

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
    g_main_state = VISION_ROUTE_MAIN_NORMAL;
    g_sub_state = VISION_ROUTE_SUB_NONE;
    g_preferred_source = VISION_ROUTE_PREFERRED_SOURCE_AUTO;
    clear_runtime_counters();

    if (input == nullptr)
    {
        return;
    }

    g_preferred_source = normalize_preferred_source(input->base_preferred_source);
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
