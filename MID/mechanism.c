#include "mechanism.h"
#include "step_motor.h"

static int32_t s_level_zero = 0;

int32_t ball_tilt_to_step_pos(float tilt_deg)
{
    float pulses = tilt_deg * ((float)STEP_ENCODER_PPR / 360.0f);
    return (int32_t)(pulses + (float)s_level_zero);
}

void mechanism_set_level_zero(int32_t pos)
{
    s_level_zero = pos;
}

int32_t mechanism_get_level_zero(void)
{
    return s_level_zero;
}
