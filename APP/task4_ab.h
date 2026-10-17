#ifndef TASK4_AB_H
#define TASK4_AB_H

#include <stdbool.h>
#include <stdint.h>

#define TASK4_AB_SPEED             16
#define TASK4_AB_START_SPEED        4
#define TASK4_AB_RAMP_STEP_MS     100U
#define TASK4_AB_TARGET_PULSES  12500U
#define TASK4_AB_TIMEOUT_MS      8000U

typedef enum {
    TASK4_AB_IDLE = 0,
    TASK4_AB_RUNNING,
    TASK4_AB_PASSED,
    TASK4_AB_TIMEOUT
} Task4AbState;

typedef struct {
    volatile Task4AbState state;
    volatile uint32_t runStartMs;
    volatile uint32_t elapsedMs;
    volatile uint32_t leftPulses;
    volatile uint32_t rightPulses;
} Task4AbController;

void Task4Ab_init(Task4AbController *controller);
void Task4Ab_start(Task4AbController *controller, uint32_t nowMs);
int16_t Task4Ab_commandSpeed(
    const Task4AbController *controller, uint32_t nowMs);
void Task4Ab_cancel(Task4AbController *controller);
void Task4Ab_recordEncoder(
    Task4AbController *controller, int16_t leftDelta, int16_t rightDelta);
void Task4Ab_update(Task4AbController *controller, uint32_t nowMs);
uint32_t Task4Ab_distancePulses(const Task4AbController *controller);
bool Task4Ab_hasResult(const Task4AbController *controller);
bool Task4Ab_succeeded(const Task4AbController *controller);

#endif /* TASK4_AB_H */
