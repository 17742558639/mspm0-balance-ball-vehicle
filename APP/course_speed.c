#include "course_speed.h"

#include <stddef.h>

static int16_t CourseSpeed_limit(int16_t requestedSpeed)
{
    if (requestedSpeed > COURSE_SLOW_SPEED) {
        return COURSE_SLOW_SPEED;
    }
    return requestedSpeed;
}

static int16_t CourseSpeed_ramp(int16_t requestedSpeed, uint32_t elapsedMs)
{
    uint32_t rampedSpeed;

    if (requestedSpeed <= COURSE_START_SPEED) {
        return requestedSpeed;
    }

    rampedSpeed = (uint32_t)COURSE_START_SPEED +
        (elapsedMs / COURSE_RAMP_STEP_MS);
    if (rampedSpeed >= (uint32_t)requestedSpeed) {
        return requestedSpeed;
    }
    return (int16_t)rampedSpeed;
}

void CourseSpeed_init(CourseSpeedController *controller)
{
    CourseSpeed_cancel(controller);
}

void CourseSpeed_start(CourseSpeedController *controller, uint32_t nowMs)
{
    if (controller == NULL) {
        return;
    }

    controller->state = COURSE_SPEED_WAIT_DELAY;
    controller->runStartMs = nowMs;
}

void CourseSpeed_cancel(CourseSpeedController *controller)
{
    if (controller == NULL) {
        return;
    }

    controller->state = COURSE_SPEED_IDLE;
    controller->runStartMs = 0U;
}

int16_t CourseSpeed_update(
    CourseSpeedController *controller, uint32_t nowMs, int16_t requestedSpeed)
{
    uint32_t elapsedMs;

    if (controller == NULL || controller->state == COURSE_SPEED_IDLE) {
        return requestedSpeed;
    }

    elapsedMs = (uint32_t)(nowMs - controller->runStartMs);
    if (controller->state == COURSE_SPEED_WAIT_DELAY &&
        elapsedMs >= COURSE_SLOWDOWN_DELAY_MS) {
        controller->state = COURSE_SPEED_SLOW;
    }

    if (controller->state == COURSE_SPEED_SLOW) {
        return CourseSpeed_limit(requestedSpeed);
    }
    return CourseSpeed_ramp(requestedSpeed, elapsedMs);
}
