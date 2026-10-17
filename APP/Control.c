/*
 * control.c - 控制核心 (转向环 + 速度环)
 *
 * 移植自队友工程, 适配 empty:
 *   - yaw 用 attitude.yaw (自写 Mahony, 见 attitude.h / imu_select.h)
 *   - 精简到后轮 bL/bR (前轮代码已删)
 *   - start_run 启用: ISR 顶部 if(!start_run) return, 上电不跑, 按键触发
 *   - 控制定时器 TIMER_0 (TIMA0, 10ms), ISR = TIMER_0_INST_IRQHandler
 *   - 编码器 ISR (GROUP1_IRQHandler) 在 Encoder.c 实现
 *
 * 对外接口: control_init / control_task / control_toggle_run (见 Control.h).
 */
#include "ti_msp_dl_config.h"
#include "Control.h"
#include "attitude.h"
#include "Encoder.h"
#include "Motor.h"
#include "pid.h"
#include "track.h"
#include "led_buzzer.h"
#include "scheduler.h"
#include "uart.h"
#include "parking.h"
#include "course_speed.h"
#include "task4_ab.h"
#include "step_motor.h"

static ParkingController g_parking;
static CourseSpeedController g_courseSpeed;
static Task4AbController g_task4Ab;
static volatile ControlTask g_activeTask = CONTROL_TASK_NONE;

int16_t basicSpeed = 28;       /* 速度环目标 (编码器脉冲数/10ms), 实测调 */

int16_t track_error = 0;
int16_t track_angle = 0;
int16_t track_turn = 0;
int16_t gyro_z_speed = 0;

float    yaw_error = 0.0f;
volatile bool start_run = false;
uint32_t isr_cnt = 0;

volatile int16_t control_desired_left = 0;
volatile int16_t control_desired_right = 0;
volatile int16_t control_pwm_left = 0;
volatile int16_t control_pwm_right = 0;

/* 100 Hz 控制周期连续 20 次没有新的、完整的 IMU 快照（约 200 ms）即锁死输出。 */
#define CONTROL_IMU_STALE_TICKS  (20U)
#define CONTROL_LOOP_PERIOD_MS   (10U)

static volatile bool g_control_motion_enabled = false;
static uint32_t g_last_imu_sequence = 0U;
static uint32_t g_last_imu_change_ms = 0U;

static void control_force_stop(void)
{
    start_run = false;
    control_desired_left = 0;
    control_desired_right = 0;
    control_pwm_left = 0;
    control_pwm_right = 0;
    motor_behind_control(0, 0);
    step_off(X);
    step_off(Y);
}

static bool control_imu_heartbeat_ok(void)
{
    uint32_t sequence = attitude_imu_sequence;

    if ((sequence != 0U) && ((sequence & 1U) == 0U) &&
        (sequence != g_last_imu_sequence)) {
        g_last_imu_sequence = sequence;
        g_last_imu_change_ms = (uint32_t)g_tms;
    }

    /* 上电后给首个 200 Hz 样本最多 200 ms；在此期间仍不允许启动电机。 */
    return (uint32_t)((uint32_t)g_tms - g_last_imu_change_ms) <
           (CONTROL_IMU_STALE_TICKS * CONTROL_LOOP_PERIOD_MS);
}

/* ---- 循迹转向环: track_error -> track_turn ---- */
void turn_control(void)
{
    track_error = track_error_get();
    track_turn = (int16_t)(-pidout(&PID_AngleTurn, track_error));
    track_angle = track_turn;
}

/* ---- yaw 转向环 (预留, 当前未在 ISR 启用): attitude.yaw -> track_turn ---- */
void yaw_control(void)
{
    PID_YawTurn.desired = 90;
    track_turn = (int16_t)pidout(&PID_YawTurn, attitude.yaw);
    yaw_error = PID_YawTurn.error;
}

/* ---- 速度环: 编码器反馈 -> 电机 PWM ---- */
static void speed_control_targets(int16_t desiredLeft, int16_t desiredRight)
{
    int16_t bl, br;

    get_speed();
    control_desired_left = desiredLeft;
    control_desired_right = desiredRight;
    PID_Speed_bL.desired = control_desired_left;
    PID_Speed_bR.desired = control_desired_right;

    bl = pidout(&PID_Speed_bL, encoder_bl_speed);
    br = pidout(&PID_Speed_bR, encoder_br_speed);
    control_pwm_left = bl;
    control_pwm_right = br;

    motor_behind_control(bl, br);
}

static void speed_control_base(int16_t baseSpeed)
{
    speed_control_targets(
        (int16_t)(baseSpeed - track_turn),
        (int16_t)(baseSpeed + track_turn));
}

void speed_control(void)
{
    speed_control_base(basicSpeed);
}

/* ---- 控制初始化: PID 参数 + 启动 100Hz 控制环 ----
 * BSP 子模块 init (encoder/motor/step) 已在 bsp_init() 里, 这里只 Control 自己的. */
void control_init(void)
{
    PidInit(&PID_Speed_bL, 0, 20, 0, 1, 1000, -1000);
    PidInit(&PID_Speed_bR, 0, 20, 0, 1, 1000, -1000);
    PidInit(&PID_AngleTurn, 0, 4, 0, 15, 35, -35);
    //PidInit(&PID_YawTurn, 0, 1, 0, 0, 40, -40);   /* yaw 环预留, 启用 yaw_control 时打开 */

    Parking_init(&g_parking);
    CourseSpeed_init(&g_courseSpeed);
    Task4Ab_init(&g_task4Ab);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_Timer_startCounter(TIMER_0_INST);
    motor_behind_control(0, 0);
    g_control_motion_enabled = false;
    g_activeTask = CONTROL_TASK_NONE;
    g_last_imu_sequence = attitude_imu_sequence;
    g_last_imu_change_ms = (uint32_t)g_tms;
}

void control_set_motion_enabled(bool enabled)
{
    g_control_motion_enabled = enabled;
    g_last_imu_sequence = attitude_imu_sequence;
    g_last_imu_change_ms = (uint32_t)g_tms;

    if (!enabled) {
        Parking_cancel(&g_parking);
        CourseSpeed_cancel(&g_courseSpeed);
        Task4Ab_cancel(&g_task4Ab);
        g_activeTask = CONTROL_TASK_NONE;
        control_force_stop();
    }
}

bool control_motion_enabled(void)
{
    return g_control_motion_enabled;
}

ControlTask control_active_task(void)
{
    return g_activeTask;
}


/* ---- 切换运行状态: start_run 翻转 + led 指示 ----
 * K3 长按调. ISR 顶部 if(!start_run) return 控制上电不跑. */
void control_set_run(bool enabled)
{
    if (enabled) {
        if (!g_control_motion_enabled || (attitude_imu_sequence == 0U) ||
            !control_imu_heartbeat_ok()) {
            Parking_cancel(&g_parking);
            CourseSpeed_cancel(&g_courseSpeed);
            Task4Ab_cancel(&g_task4Ab);
            g_activeTask = CONTROL_TASK_NONE;
            control_force_stop();
            return;
        }
        encoder_bl = 0;
        encoder_br = 0;
        encoder_bl_speed = 0;
        encoder_br_speed = 0;
        PidResetState(&PID_Speed_bL);
        PidResetState(&PID_Speed_bR);
        Task4Ab_cancel(&g_task4Ab);
        Parking_start(&g_parking, (uint32_t)g_tms);
        CourseSpeed_start(&g_courseSpeed, (uint32_t)g_tms);
        g_activeTask = CONTROL_TASK_A_STOP;
        start_run = true;
        led_on();
    } else {
        start_run = false;
        Parking_cancel(&g_parking);
        CourseSpeed_cancel(&g_courseSpeed);
        Task4Ab_cancel(&g_task4Ab);
        g_activeTask = CONTROL_TASK_NONE;
        control_desired_left = 0;
        control_desired_right = 0;
        control_pwm_left = 0;
        control_pwm_right = 0;
        control_force_stop();
        led_off();
    }
}

bool control_start_ab(void)
{
    if (!g_control_motion_enabled || (attitude_imu_sequence == 0U) ||
        !control_imu_heartbeat_ok()) {
        Parking_cancel(&g_parking);
        CourseSpeed_cancel(&g_courseSpeed);
        Task4Ab_cancel(&g_task4Ab);
        g_activeTask = CONTROL_TASK_NONE;
        control_force_stop();
        led_off();
        return false;
    }

    encoder_bl = 0;
    encoder_br = 0;
    encoder_bl_speed = 0;
    encoder_br_speed = 0;
    PidResetState(&PID_Speed_bL);
    PidResetState(&PID_Speed_bR);
    Parking_cancel(&g_parking);
    CourseSpeed_cancel(&g_courseSpeed);
    Task4Ab_start(&g_task4Ab, (uint32_t)g_tms);
    g_activeTask = CONTROL_TASK_AB;
    start_run = true;
    led_on();
    return true;
}

void control_toggle_run(void)
{
    if (g_activeTask == CONTROL_TASK_AB) {
        control_set_run(false);
        return;
    }

    if (g_activeTask == CONTROL_TASK_NONE ||
        g_activeTask == CONTROL_TASK_A_STOP) {
        control_set_run(!start_run);
    }
}

bool control_parking_result_ready(void)
{
    return Parking_hasResult(&g_parking);
}

bool control_parking_result_success(void)
{
    return Parking_succeeded(&g_parking);
}

uint32_t control_parking_elapsed_ms(void)
{
    if (start_run) {
        if (g_activeTask == CONTROL_TASK_A_STOP) {
            return (uint32_t)g_tms - g_parking.runStartMs;
        }
    }

    return g_parking.elapsedMs;
}

bool control_ab_result_ready(void)
{
    return Task4Ab_hasResult(&g_task4Ab);
}

bool control_ab_result_success(void)
{
    return Task4Ab_succeeded(&g_task4Ab);
}

uint32_t control_ab_elapsed_ms(void)
{
    if (start_run && g_activeTask == CONTROL_TASK_AB) {
        return (uint32_t)g_tms - g_task4Ab.runStartMs;
    }

    return g_task4Ab.elapsedMs;
}

uint32_t control_ab_distance_pulses(void)
{
    return Task4Ab_distancePulses(&g_task4Ab);
}

/* 读取并清零编码器累计值 -> 每控制周期的脉冲数 (速度) */
void get_speed(void)
{
    encoder_bl_speed = encoder_bl;      /* ISR 已调为正转递增, 无需取反 */
    encoder_br_speed = encoder_br;      /* ISR 已调为正转递增, 无需取反 */
    encoder_bl = 0;
    encoder_br = 0;
}

/* ---- 控制环 ISR (10ms) ---- */
void TIMER_0_INST_IRQHandler(void)
{
    if (DL_Timer_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO)
    {
        DL_Timer_clearInterruptStatus(TIMER_0_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);

        if (!start_run) return;         /* 安全开关: 上电不跑, 按键触发 start_run=true */

        if (!g_control_motion_enabled || !control_imu_heartbeat_ok()) {
            g_control_motion_enabled = false;  /* 故障锁存，需重新上电并通过 IMU 初始化 */
            Parking_cancel(&g_parking);
            CourseSpeed_cancel(&g_courseSpeed);
            Task4Ab_cancel(&g_task4Ab);
            g_activeTask = CONTROL_TASK_NONE;
            control_force_stop();
            led_off();
            return;
        }

        static uint8_t cnt = 0;
        cnt++;

        // if (cnt % 3 == 0) {             /* ~30ms 一次转向环 */
            turn_control();
        //}
        if (cnt % 2 == 0) 
        {             /* ~20ms 一次 yaw 环 (预留) */
             //yaw_control();
        }
        if (g_activeTask == CONTROL_TASK_A_STOP) {
            int16_t effectiveSpeed = CourseSpeed_update(
                &g_courseSpeed, (uint32_t)g_tms, basicSpeed);
            bool relaxedParkingTarget =
                g_courseSpeed.state == COURSE_SPEED_SLOW;
            ParkingState oldState = g_parking.state;
            ParkingAction action = Parking_update(
                &g_parking,
                track_active_count(),
                relaxedParkingTarget,
                (uint32_t)g_tms);

            if (oldState != g_parking.state &&
                g_parking.state == PARK_STATE_BRAKE_CHECK) {
                PidResetState(&PID_Speed_bL);
                PidResetState(&PID_Speed_bR);
            }

            switch (action) {
            case PARK_ACTION_FORWARD:
                speed_control_base(effectiveSpeed);
                break;

            case PARK_ACTION_STOP:
            default:
                get_speed();
                control_desired_left = 0;
                control_desired_right = 0;
                control_pwm_left = 0;
                control_pwm_right = 0;
                motor_behind_control(0, 0);
                if (Parking_hasResult(&g_parking)) {
                    start_run = false;
                    led_off();
                }
                break;
            }
        } else if (g_activeTask == CONTROL_TASK_AB) {
            int16_t abSpeed = Task4Ab_commandSpeed(
                &g_task4Ab, (uint32_t)g_tms);
            speed_control_base(abSpeed);
            Task4Ab_recordEncoder(
                &g_task4Ab, encoder_bl_speed, encoder_br_speed);
            Task4Ab_update(&g_task4Ab, (uint32_t)g_tms);
        } else {
            control_force_stop();
            led_off();
        }

        if (cnt >= 36) cnt = 0;
    }
}

/* ---- 主循环周期任务 (非 ISR) ----
 * 在 system_run 的 while(1) 里调. 用于放非实时的任务逻辑 (状态机/任务调度等),
 * 实时控制环在 TIMER_0 ISR 里跑. */
void control_task(void)
{
    if (!g_control_motion_enabled) {
        return;
    }
    if (!control_imu_heartbeat_ok()) {
        g_control_motion_enabled = false;
        Parking_cancel(&g_parking);
        CourseSpeed_cancel(&g_courseSpeed);
        Task4Ab_cancel(&g_task4Ab);
        g_activeTask = CONTROL_TASK_NONE;
        control_force_stop();
        led_off();
        return;
    }
    if (attitude_imu_sequence == 0U) {
        return;
    }
}
