/*
 * pid.c - PID 控制器 (位置式 + 增量式)
 *
 * 详见 pid.h. 纯算法, 无硬件依赖.
 */
#include "pid.h"

#include <stddef.h>

/* ---- PID 实例 (全局变量定义, pid.h 有 extern 声明) ----
 * 移植时曾漏写这些定义, 导致链接 unresolved. 必须在 .c 里定义, .h 里 extern. */
PidObject PID_Speed_bL;
PidObject PID_Speed_bR;
PidObject PID_Speed_aL;
PidObject PID_Speed_aR;
PidObject PID_AngleTurn;
PidObject PID_YawTurn;
PidObject PID_StepPos;
PidObject PID_BallPos;


/* ========================= 位置式 PID ========================= */

void PidInit(PidObject *pid, float desired, float KP, float KI, float KD, float Max, float Min)
{
    pid->error      = 0;
    pid->preverror  = 0;
    pid->integ      = 0;
    pid->prev_integ = 0;
    pid->deriv      = 0;
    pid->desired    = desired;
    pid->kp         = KP;
    pid->ki         = KI;
    pid->kd         = KD;
    pid->MaxValue   = Max;
    pid->MinValue   = Min;
}

void PidResetState(PidObject *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->error      = 0.0f;
    pid->preverror  = 0.0f;
    pid->integ      = 0.0f;
    pid->prev_integ = 0.0f;
    pid->deriv      = 0.0f;
    pid->out        = 0.0f;
}

float pidout(PidObject *pid, float measure)
{
    pid->error    = pid->desired - measure;
    pid->integ   += pid->error;
    pid->deriv    = pid->error - pid->preverror;
    pid->preverror = pid->error;

    pid->out = pid->kp * pid->error + pid->ki * pid->integ + pid->kd * pid->deriv;

    /* 输出限幅 */
    if (pid->out > pid->MaxValue)
        pid->out = pid->MaxValue;
    else if (pid->out < pid->MinValue)
        pid->out = pid->MinValue;

    return pid->out;
}


/* ========================= 增量式 PID ========================= */

void PidInit_z(PidObject_Z *pid, float desired, float KP, float KI, float Max, float Min)
{
    pid->error     = 0;
    pid->preverror = 0;
    pid->integ     = 0;
    pid->deriv     = 0;
    pid->desired   = desired;
    pid->kp        = KP;
    pid->ki        = KI;
    pid->MaxValue  = Max;
    pid->MinValue  = Min;
    pid->out_last  = 0;
    pid->out_flag  = 0;
}

float pidout_z(PidObject_Z *pid, float measure)
{
    pid->error    = pid->desired - measure;
    pid->deriv    = pid->error - pid->preverror;
    pid->preverror = pid->error;
    pid->out      = pid->out_last + (pid->kp * pid->deriv + pid->ki * pid->error);
    pid->out_last = pid->out;

    /* 输出限幅 */
    if (pid->out >= pid->MaxValue) { pid->out = pid->MaxValue; pid->out_last = pid->MaxValue; }
    if (pid->out <= pid->MinValue) { pid->out = pid->MinValue; pid->out_last = pid->MinValue; }

    return pid->out;
}
