#ifndef VISION_MOTOR_TRIGGER_H_
#define VISION_MOTOR_TRIGGER_H_

#include <stdbool.h>
#include <stdint.h>

#define VISION_MOTOR_RUN_MS (1000U)

typedef enum {
    VISION_TRIGGER_WAIT_CLEAR = 0,
    VISION_TRIGGER_ARMED,
    VISION_TRIGGER_RUNNING,
} VisionMotorState;

typedef enum {
    VISION_MOTOR_ACTION_NONE = 0,
    VISION_MOTOR_ACTION_START,
    VISION_MOTOR_ACTION_STOP,
} VisionMotorAction;

typedef struct {
    VisionMotorState state;
    uint32_t stopDeadlineMs;
} VisionMotorTrigger;

void VisionMotorTrigger_init(VisionMotorTrigger *trigger);
VisionMotorAction VisionMotorTrigger_onRedState(
    VisionMotorTrigger *trigger, bool present, uint32_t nowMs);
VisionMotorAction VisionMotorTrigger_tick(
    VisionMotorTrigger *trigger, uint32_t nowMs);

#endif /* VISION_MOTOR_TRIGGER_H_ */
