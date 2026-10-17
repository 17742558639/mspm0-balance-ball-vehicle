#ifndef __MECHANISM_H
#define __MECHANISM_H

#include <stdint.h>

#define MECH_TILT_MAX_DEG   20.0f

int32_t ball_tilt_to_step_pos(float tilt_deg);
void mechanism_set_level_zero(int32_t pos);
int32_t mechanism_get_level_zero(void);

#endif /* __MECHANISM_H */
