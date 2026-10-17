#include "vision_motor_trigger.h"

void VisionMotorTrigger_init(VisionMotorTrigger *trigger)
{
    trigger->state          = VISION_TRIGGER_WAIT_CLEAR;
    trigger->stopDeadlineMs = 0U;
}

VisionMotorAction VisionMotorTrigger_onRedState(
    VisionMotorTrigger *trigger, bool present, uint32_t nowMs)
{
    if (!present) {
        if (trigger->state == VISION_TRIGGER_WAIT_CLEAR) {
            trigger->state = VISION_TRIGGER_ARMED;
        }
        return VISION_MOTOR_ACTION_NONE;
    }

    if (trigger->state == VISION_TRIGGER_ARMED) {
        trigger->state          = VISION_TRIGGER_RUNNING;
        trigger->stopDeadlineMs = nowMs + VISION_MOTOR_RUN_MS;
        return VISION_MOTOR_ACTION_START;
    }

    return VISION_MOTOR_ACTION_NONE;
}

VisionMotorAction VisionMotorTrigger_tick(
    VisionMotorTrigger *trigger, uint32_t nowMs)
{
    if ((trigger->state == VISION_TRIGGER_RUNNING) &&
        ((int32_t) (nowMs - trigger->stopDeadlineMs) >= 0)) {
        trigger->state = VISION_TRIGGER_WAIT_CLEAR;
        return VISION_MOTOR_ACTION_STOP;
    }

    return VISION_MOTOR_ACTION_NONE;
}
