#include "parking.h"

#include <stddef.h>

static bool Parking_targetDetected(
    uint8_t activeCount, bool relaxedTarget)
{
    if (relaxedTarget) {
        return activeCount >= PARK_SLOW_TARGET_MIN_ACTIVE_COUNT;
    }
    return activeCount == PARK_TARGET_ACTIVE_COUNT;
}

static ParkingAction Parking_finish(
    ParkingController *controller, ParkingState state, uint32_t nowMs)
{
    controller->state = state;
    controller->elapsedMs = nowMs - controller->runStartMs;
    return PARK_ACTION_STOP;
}

void Parking_init(ParkingController *controller)
{
    Parking_cancel(controller);
}

void Parking_start(ParkingController *controller, uint32_t nowMs)
{
    if (controller == NULL) {
        return;
    }

    controller->state = PARK_STATE_WAIT_LEAVE_A;
    controller->runStartMs = nowMs;
    controller->stateStartMs = nowMs;
    controller->elapsedMs = 0U;
    controller->leaveConfirmTicks = 0U;
    controller->forwardConfirmTicks = 0U;
}

void Parking_cancel(ParkingController *controller)
{
    if (controller == NULL) {
        return;
    }

    controller->state = PARK_STATE_IDLE;
    controller->runStartMs = 0U;
    controller->stateStartMs = 0U;
    controller->elapsedMs = 0U;
    controller->leaveConfirmTicks = 0U;
    controller->forwardConfirmTicks = 0U;
}

ParkingAction Parking_update(
    ParkingController *controller,
    uint8_t activeCount,
    bool relaxedTarget,
    uint32_t nowMs)
{
    if (controller == NULL) {
        return PARK_ACTION_STOP;
    }

    switch (controller->state) {
    case PARK_STATE_WAIT_LEAVE_A:
        if (activeCount == PARK_TARGET_ACTIVE_COUNT) {
            controller->leaveConfirmTicks = 0U;
        } else {
            if (controller->leaveConfirmTicks < PARK_LEAVE_CONFIRM_TICKS) {
                controller->leaveConfirmTicks++;
            }
            if (controller->leaveConfirmTicks >= PARK_LEAVE_CONFIRM_TICKS) {
                controller->state = PARK_STATE_RUNNING;
                controller->forwardConfirmTicks = 0U;
            }
        }
        return PARK_ACTION_FORWARD;

    case PARK_STATE_RUNNING:
        if (Parking_targetDetected(activeCount, relaxedTarget)) {
            if (controller->forwardConfirmTicks <
                PARK_FORWARD_CONFIRM_TICKS) {
                controller->forwardConfirmTicks++;
            }
            if (controller->forwardConfirmTicks >=
                PARK_FORWARD_CONFIRM_TICKS) {
                controller->state = PARK_STATE_BRAKE_CHECK;
                controller->stateStartMs = nowMs;
                return PARK_ACTION_STOP;
            }
        } else {
            controller->forwardConfirmTicks = 0U;
        }
        return PARK_ACTION_FORWARD;

    case PARK_STATE_BRAKE_CHECK:
        if ((uint32_t)(nowMs - controller->stateStartMs) <
            PARK_BRAKE_HOLD_MS) {
            return PARK_ACTION_STOP;
        }
        return Parking_finish(controller, PARK_STATE_STOPPED, nowMs);

    case PARK_STATE_IDLE:
    case PARK_STATE_STOPPED:
    case PARK_STATE_FAILED:
    default:
        return PARK_ACTION_STOP;
    }
}

bool Parking_hasResult(const ParkingController *controller)
{
    return controller != NULL &&
        (controller->state == PARK_STATE_STOPPED ||
         controller->state == PARK_STATE_FAILED);
}

bool Parking_succeeded(const ParkingController *controller)
{
    return controller != NULL && controller->state == PARK_STATE_STOPPED;
}
