#ifndef BALL_TARGET_TASK_H
#define BALL_TARGET_TASK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BALL_TARGET_TASK_IDLE = 0,
    BALL_TARGET_TASK_CENTER_HOLD,
    BALL_TARGET_TASK_WAIT_POSITIVE,
    BALL_TARGET_TASK_WAIT_NEGATIVE,
    BALL_TARGET_TASK_HOLD
} BallTargetTaskState;

void BallTargetTask_init(void);
void BallTargetTask_start(void);
void BallTargetTask_startCenterHold(void);
void BallTargetTask_stop(void);
void BallTargetTask_update(uint32_t nowMs);
bool BallTargetTask_active(void);
BallTargetTaskState BallTargetTask_state(void);

#endif /* BALL_TARGET_TASK_H */
