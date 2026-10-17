#ifndef COURSE_SPEED_H
#define COURSE_SPEED_H

#include <stdint.h>

#define COURSE_START_SPEED             12
#define COURSE_RAMP_STEP_MS            100U
#define COURSE_SLOWDOWN_DELAY_MS       10000U
#define COURSE_SLOW_SPEED              6

typedef enum {
    COURSE_SPEED_IDLE = 0,
    COURSE_SPEED_WAIT_DELAY,
    COURSE_SPEED_SLOW
} CourseSpeedState;

typedef struct {
    CourseSpeedState state;
    uint32_t runStartMs;
} CourseSpeedController;

void CourseSpeed_init(CourseSpeedController *controller);
void CourseSpeed_start(CourseSpeedController *controller, uint32_t nowMs);
void CourseSpeed_cancel(CourseSpeedController *controller);
int16_t CourseSpeed_update(
    CourseSpeedController *controller, uint32_t nowMs, int16_t requestedSpeed);

#endif /* COURSE_SPEED_H */
