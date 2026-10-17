#ifndef __PID_H
#define __PID_H

#include <stdint.h>

/* ========================= PID 控制器 =========================
 * 两种形式:
 *   PidObject   - 位置式 PID (输出 = Kp*e + Ki*∫e + Kd*de)
 *   PidObject_Z - 增量式 PID (输出 = 上次输出 + Kp*de + Ki*e)
 *
 * 用法:
 *   PidObject pid;
 *   PidInit(&pid, 期望值, Kp, Ki, Kd, 输出上限, 输出下限);
 *   float u = pidout(&pid, 当前测量值);
 *
 * 纯算法, 无硬件依赖.
 */

/* 位置式 PID */
typedef struct {
    float kp;           /* 比例 */
    float ki;           /* 积分 */
    float kd;           /* 微分 */
    float desired;      /* 期望值 */
    float error;        /* 当前误差 */
    float preverror;    /* 上一次误差 */
    float integ;        /* 误差积分 */
    float prev_integ;   /* 积分项累加前的值 (备用) */
    float deriv;        /* 误差微分 */
    float MaxValue;     /* 输出最大值 */
    float MinValue;     /* 输出最小值 */
    float out;          /* 输出 */
} PidObject;

/* 增量式 PID */
typedef struct {
    float kp;           /* 比例系数 */
    float ki;           /* 积分系数 */
    float desired;      /* 目标值 */
    float error;        /* 误差 */
    float preverror;    /* 前一个误差 */
    float integ;        /* 误差积分 */
    float deriv;        /* 误差微分 */
    float MaxValue;     /* 输出最大值 */
    float MinValue;     /* 输出最小值 */
    float out;          /* 输出值 */
    float out_last;     /* 上一次输出值 */
    float out_flag;
} PidObject_Z;

void  PidInit(PidObject *pid, float desired, float KP, float KI, float KD, float Max, float Min);
void  PidResetState(PidObject *pid);
float pidout(PidObject *pid, float measure);

void  PidInit_z(PidObject_Z *pid, float desired, float KP, float KI, float Max, float Min);
float pidout_z(PidObject_Z *pid, float measure);

/* ---- PID 实例 (定义在 pid.c) ----
 * bL/bR: 后轮速度环; aL/aR: 前轮速度环 (预留, 前轮已精简未用);
 * AngleTurn: 循迹转向环; YawTurn: yaw 转向环 (预留). */
extern PidObject PID_Speed_bL;
extern PidObject PID_Speed_bR;
extern PidObject PID_Speed_aL;
extern PidObject PID_Speed_aR;
extern PidObject PID_AngleTurn;
extern PidObject PID_YawTurn;
extern PidObject PID_StepPos;
extern PidObject PID_BallPos;

#endif /* __PID_H */
