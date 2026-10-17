#include "task4_ab.h"

#include <stddef.h>

static uint32_t Task4Ab_addPositiveSaturated(
    uint32_t total, int16_t delta)
{
    uint32_t increment;

    if (delta <= 0) {
        return total;
    }

    increment = (uint32_t)delta;
    if (total > (UINT32_MAX - increment)) {
        return UINT32_MAX;
    }
    return total + increment;
}

void Task4Ab_init(Task4AbController *controller)
{
    Task4Ab_cancel(controller);
}

void Task4Ab_start(Task4AbController *controller, uint32_t nowMs)
{
    if (controller == NULL) {
        return;
    }

    controller->state = TASK4_AB_RUNNING;
    controller->runStartMs = nowMs;
    controller->elapsedMs = 0U;
    controller->leftPulses = 0U;
    controller->rightPulses = 0U;
}

int16_t Task4Ab_commandSpeed(
    const Task4AbController *controller, uint32_t nowMs)
{
    uint32_t elapsedMs;
    uint32_t speed;

    if (controller == NULL || controller->state != TASK4_AB_RUNNING) {
        return 0;
    }

    elapsedMs = (uint32_t)(nowMs - controller->runStartMs);
    speed = (uint32_t)TASK4_AB_START_SPEED +
        (elapsedMs / TASK4_AB_RAMP_STEP_MS);
    if (speed >= (uint32_t)TASK4_AB_SPEED) {
        speed = (uint32_t)TASK4_AB_SPEED;
    }
    return (int16_t)speed;
}

void Task4Ab_cancel(Task4AbController *controller)
{
    if (controller == NULL) {
        return;
    }

    controller->state = TASK4_AB_IDLE;
    controller->runStartMs = 0U;
    controller->elapsedMs = 0U;
    controller->leftPulses = 0U;
    controller->rightPulses = 0U;
}

void Task4Ab_recordEncoder(
    Task4AbController *controller, int16_t leftDelta, int16_t rightDelta)
{
    if (controller == NULL || controller->state != TASK4_AB_RUNNING) {
        return;
    }

    controller->leftPulses = Task4Ab_addPositiveSaturated(
        controller->leftPulses, leftDelta);
    controller->rightPulses = Task4Ab_addPositiveSaturated(
        controller->rightPulses, rightDelta);
}

uint32_t Task4Ab_distancePulses(const Task4AbController *controller)
{
    uint32_t left;
    uint32_t right;

    if (controller == NULL) {
        return 0U;
    }

    left = controller->leftPulses;
    right = controller->rightPulses;
    return (left / 2U) + (right / 2U) +
        (((left & 1U) + (right & 1U)) / 2U);
}

void Task4Ab_update(Task4AbController *controller, uint32_t nowMs)
{
    if (controller == NULL || controller->state != TASK4_AB_RUNNING) {
        return;
    }

    controller->elapsedMs =
        (uint32_t)(nowMs - controller->runStartMs);
}

bool Task4Ab_hasResult(const Task4AbController *controller)
{
    return controller != NULL &&
        (controller->state == TASK4_AB_PASSED ||
         controller->state == TASK4_AB_TIMEOUT);
}

bool Task4Ab_succeeded(const Task4AbController *controller)
{
    return controller != NULL && controller->state == TASK4_AB_PASSED;
}
