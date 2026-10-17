#include "ball_target_task.h"

#include "ball.h"
#include "led_buzzer.h"

#define BALL_TARGET_BUZZER_MS   80U

static BallTargetTaskState s_state = BALL_TARGET_TASK_IDLE;
static bool s_buzzer_active = false;
static uint32_t s_buzzer_started_ms = 0U;

static void BallTargetTask_startBuzzer(uint32_t nowMs)
{
    buzzer_on();
    s_buzzer_active = true;
    s_buzzer_started_ms = nowMs;
}

static void BallTargetTask_serviceBuzzer(uint32_t nowMs)
{
    if (s_buzzer_active &&
        ((uint32_t)(nowMs - s_buzzer_started_ms) >=
         BALL_TARGET_BUZZER_MS)) {
        buzzer_off();
        s_buzzer_active = false;
    }
}

void BallTargetTask_init(void)
{
    s_state = BALL_TARGET_TASK_IDLE;
    s_buzzer_active = false;
    buzzer_off();
}

void BallTargetTask_start(void)
{
    buzzer_off();
    led_off();
    s_buzzer_active = false;
    ball_loop_stop();
    ball_loop_run();
    set_ball_position(50);
    s_state = BALL_TARGET_TASK_WAIT_POSITIVE;
}

void BallTargetTask_startCenterHold(void)
{
    buzzer_off();
    led_off();
    s_buzzer_active = false;
    ball_loop_stop();
    set_ball_position(0);
    ball_loop_run();
    s_state = BALL_TARGET_TASK_CENTER_HOLD;
}

void BallTargetTask_stop(void)
{
    s_state = BALL_TARGET_TASK_IDLE;
    s_buzzer_active = false;
    buzzer_off();
    led_off();
    ball_loop_stop();
}

void BallTargetTask_update(uint32_t nowMs)
{
    BallTargetTask_serviceBuzzer(nowMs);

    switch (s_state) {
    case BALL_TARGET_TASK_WAIT_POSITIVE:
        if (wait_ball_pos(50)) {
            BallTargetTask_startBuzzer(nowMs);
            set_ball_position(-50);
            s_state = BALL_TARGET_TASK_WAIT_NEGATIVE;
        }
        break;

    case BALL_TARGET_TASK_WAIT_NEGATIVE:
        if (wait_ball_pos(-50)) {
            led_on();
            BallTargetTask_startBuzzer(nowMs);
            s_state = BALL_TARGET_TASK_HOLD;
        }
        break;

    case BALL_TARGET_TASK_IDLE:
    case BALL_TARGET_TASK_CENTER_HOLD:
    case BALL_TARGET_TASK_HOLD:
    default:
        break;
    }
}

bool BallTargetTask_active(void)
{
    return s_state != BALL_TARGET_TASK_IDLE;
}

BallTargetTaskState BallTargetTask_state(void)
{
    return s_state;
}
