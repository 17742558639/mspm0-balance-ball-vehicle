#include "ti_msp_dl_config.h"
#include "ball.h"
#include "Encoder.h"
#include "step_motor.h"
#include "pid.h"
#if BALL_LOOP_SINGLE && BALL_SINGLE_FRAME_PD
#include "scheduler.h"
#endif
#if !BALL_LOOP_SINGLE
#include "mechanism.h"
#endif

#define BALL_LOOP_DIV             3U
#define BALL_DIR_INVERT          (1)

#define BALL_ARRIVE_TOL           8
#define BALL_ARRIVE_FRAMES       13U

#define BALL_PID_SINGLE_KP       10.0f
#define BALL_PID_SINGLE_KI        0.1f
#define BALL_PID_SINGLE_KD       17.0f
#define BALL_STEP_HZ          12000

#define BALL_OUT_MAX           2000
#define BALL_DELTA_MAX_FAR     1500
#define BALL_DELTA_MAX_MID      800
#define BALL_DELTA_MAX_NEAR     300
#define BALL_DELTA_FAR_THRESH   800
#define BALL_DELTA_MID_THRESH   250
#define BALL_DELTA_DEAD           5

#if BALL_LOOP_SINGLE && BALL_SINGLE_FRAME_PD
#define BALL_PID_SINGLE_KV         2.60f
#define BALL_VEL_FILTER_TAU_MS    30.0f
#define BALL_FRAME_DT_MIN_MS       5U
#define BALL_FRAME_DT_MAX_MS     100U
#define BALL_FRAME_TIMEOUT_MS    100U
#define BALL_D_OUT_MAX          2200.0f
#define BALL_ARRIVE_SPEED_MAX    100.0f
#define BALL_OLD_NOMINAL_DT_MS    30.0f
#define BALL_I_BAND                60.0f
#define BALL_I_MAX               3000.0f
#endif

#define BALL_VIRT_POS_MAX      2500
#define BALL_VIRT_POS_MIN     -2500

#define BALL_PID_DUAL_KP         0.1f
#define BALL_PID_DUAL_KI         0.0f
#define BALL_PID_DUAL_KD         0.9f

volatile int32_t step_target_pos = 0;
volatile int16_t ball_pos = 0;
volatile int16_t ball_target_pos = 0;

static volatile bool ball_loop_enable = false;
static volatile uint16_t ball_stable_cnt = 0;
static volatile bool ball_arrived = false;
#if BALL_LOOP_SINGLE
static int32_t step_virt_pos = 0;
#endif

float g_out = 0.0f;

volatile uint32_t g_ball_frame_seq = 0U;
volatile uint32_t g_ball_dt_ms = 0U;
volatile float g_ball_velocity_raw = 0.0f;
volatile float g_ball_velocity_filtered = 0.0f;
volatile float g_ball_p_out = 0.0f;
volatile float g_ball_i_out = 0.0f;
volatile float g_ball_d_out = 0.0f;

#if BALL_LOOP_SINGLE && BALL_SINGLE_FRAME_PD
static volatile uint32_t s_ball_frame_time_ms = 0U;
static uint32_t s_last_frame_seq = 0U;
static uint32_t s_previous_frame_time_ms = 0U;
static int16_t s_previous_ball_pos = 0;
static bool s_velocity_initialized = false;
static bool s_frame_timeout_active = false;
#endif

#if !BALL_LOOP_SINGLE
static void step_pos_control(void);
#endif
static void ball_pos_control(void);

void ball_init(void)
{
    PidInit(&PID_StepPos, 0.0f, 50.0f, 0.0f, 0.0f,
            STEP_SPEED_MAX_HZ, -STEP_SPEED_MAX_HZ);

#if BALL_LOOP_SINGLE
    PidInit(&PID_BallPos, 0.0f, BALL_PID_SINGLE_KP,
            BALL_PID_SINGLE_KI, BALL_PID_SINGLE_KD,
            (float)BALL_OUT_MAX, -(float)BALL_OUT_MAX);
#else
    PidInit(&PID_BallPos, 0.0f, BALL_PID_DUAL_KP,
            BALL_PID_DUAL_KI, BALL_PID_DUAL_KD,
            MECH_TILT_MAX_DEG, -MECH_TILT_MAX_DEG);
    step_target_pos = encoder_step;
#endif

    NVIC_EnableIRQ(TIMER_BALL_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_BALL_INST);
}

void ball_isr_10ms(void)
{
    static uint8_t ball_cnt = 0U;
    ball_cnt++;

#if BALL_LOOP_SINGLE && BALL_SINGLE_FRAME_PD
    ball_pos_control();
#else
    if ((ball_cnt % BALL_LOOP_DIV) == 0U) {
        ball_pos_control();
    }
#endif

#if !BALL_LOOP_SINGLE
    step_pos_control();
#endif

    if (ball_cnt >= 36U) {
        ball_cnt = 0U;
    }
}

void set_step_target(int input_pos)
{
    step_target_pos = input_pos;
}

void ball_measurement_update(int16_t position)
{
    ball_pos = position;
#if BALL_LOOP_SINGLE && BALL_SINGLE_FRAME_PD
    s_ball_frame_time_ms = (uint32_t)g_tms;
    g_ball_frame_seq++;
#endif
}

bool wait_ball_pos(int16_t pos)
{
    (void)pos;
    return ball_arrived;
}

void set_ball_position(int16_t pos)
{
    ball_target_pos = pos;
    ball_stable_cnt = 0U;
    ball_arrived = false;
#if BALL_LOOP_SINGLE && BALL_SINGLE_FRAME_PD
    PID_BallPos.integ = 0.0f;
#endif
}

#if !BALL_LOOP_SINGLE
static void step_pos_control(void)
{
    int32_t pos = encoder_step;
    float out_hz;

    PID_StepPos.desired = (float)step_target_pos;
    out_hz = pidout(&PID_StepPos, (float)pos);
    step_drive_hz((int32_t)out_hz);
}
#endif

#if BALL_LOOP_SINGLE
static void ball_single_drive_target(float out)
{
    int32_t target = (int32_t)out;
    int32_t delta;
    int32_t delta_abs;
    int32_t delta_cap;

    if (target > BALL_VIRT_POS_MAX) {
        target = BALL_VIRT_POS_MAX;
    } else if (target < BALL_VIRT_POS_MIN) {
        target = BALL_VIRT_POS_MIN;
    }

    delta = target - step_virt_pos;
    delta_abs = (delta < 0) ? -delta : delta;
    delta_cap = BALL_DELTA_MAX_NEAR;
    if (delta_abs >= BALL_DELTA_FAR_THRESH) {
        delta_cap = BALL_DELTA_MAX_FAR;
    } else if (delta_abs >= BALL_DELTA_MID_THRESH) {
        delta_cap = BALL_DELTA_MAX_MID;
    }

    if (delta > delta_cap) {
        delta = delta_cap;
    } else if (delta < -delta_cap) {
        delta = -delta_cap;
    }

    if ((delta > BALL_DELTA_DEAD) || (delta < -BALL_DELTA_DEAD)) {
        step_on_at_freq(delta, BALL_STEP_HZ);
        step_virt_pos += delta;
    } else {
        step_off(Y);
    }
}
#endif

static void ball_pos_control(void)
{
    int16_t ball;

    if (!ball_loop_enable) {
        return;
    }

#if BALL_LOOP_SINGLE && BALL_SINGLE_FRAME_PD
    uint32_t seq_before;
    uint32_t seq_after;
    uint32_t frame_time_ms;
    uint32_t dt_ms = 0U;
    float velocity_raw = 0.0f;
    float velocity_filtered = g_ball_velocity_filtered;
    float integral_scale = 1.0f;
    float error;
    float p_out;
    float d_out;
    float i_out;
    float out;

    do {
        seq_before = g_ball_frame_seq;
        ball = ball_pos;
        frame_time_ms = s_ball_frame_time_ms;
        seq_after = g_ball_frame_seq;
    } while (seq_before != seq_after);

    if ((seq_before == 0U) || (seq_before == s_last_frame_seq)) {
        if ((seq_before != 0U) &&
            (((uint32_t)g_tms - frame_time_ms) > BALL_FRAME_TIMEOUT_MS)) {
            if (!s_frame_timeout_active) {
                s_frame_timeout_active = true;
                s_velocity_initialized = false;
                PID_BallPos.integ = 0.0f;
                g_out = 0.0f;
                g_ball_dt_ms = 0U;
                g_ball_velocity_raw = 0.0f;
                g_ball_velocity_filtered = 0.0f;
                g_ball_p_out = 0.0f;
                g_ball_i_out = 0.0f;
                g_ball_d_out = 0.0f;
            }
            if (step1_count == 0U) {
                ball_single_drive_target(0.0f);
            }
        }
        return;
    }

    if (step1_count > 0U) {
        return;
    }

    s_last_frame_seq = seq_before;
    s_frame_timeout_active = false;

    if (s_velocity_initialized) {
        dt_ms = frame_time_ms - s_previous_frame_time_ms;
        if ((dt_ms >= BALL_FRAME_DT_MIN_MS) &&
            (dt_ms <= BALL_FRAME_DT_MAX_MS)) {
            int32_t delta_pos = (int32_t)ball - (int32_t)s_previous_ball_pos;
            float alpha;

            velocity_raw = (float)delta_pos * 1000.0f / (float)dt_ms;
            alpha = (float)dt_ms /
                    (BALL_VEL_FILTER_TAU_MS + (float)dt_ms);
            velocity_filtered +=
                alpha * (velocity_raw - velocity_filtered);
            integral_scale = (float)dt_ms / BALL_OLD_NOMINAL_DT_MS;
        } else {
            velocity_filtered = 0.0f;
            integral_scale = 0.0f;
        }
    } else {
        s_velocity_initialized = true;
        velocity_filtered = 0.0f;
    }

    s_previous_ball_pos = ball;
    s_previous_frame_time_ms = frame_time_ms;
    g_ball_dt_ms = dt_ms;
    g_ball_velocity_raw = velocity_raw;
    g_ball_velocity_filtered = velocity_filtered;

    error = (float)ball_target_pos - (float)ball;
    p_out = BALL_PID_SINGLE_KP * error;
    d_out = -BALL_PID_SINGLE_KV * velocity_filtered;
    if (d_out > BALL_D_OUT_MAX) {
        d_out = BALL_D_OUT_MAX;
    } else if (d_out < -BALL_D_OUT_MAX) {
        d_out = -BALL_D_OUT_MAX;
    }

    if (integral_scale > 0.0f) {
        float err_abs = (error < 0.0f) ? -error : error;
        float pd = p_out + d_out;
        float pd_abs = (pd < 0.0f) ? -pd : pd;

        if ((err_abs < BALL_I_BAND) &&
            (pd_abs < (float)BALL_OUT_MAX)) {
            PID_BallPos.integ += error * integral_scale;
        } else if (pd_abs >= (float)BALL_OUT_MAX) {
            PID_BallPos.integ *= 0.9f;
        }
        if (PID_BallPos.integ > BALL_I_MAX) {
            PID_BallPos.integ = BALL_I_MAX;
        } else if (PID_BallPos.integ < -BALL_I_MAX) {
            PID_BallPos.integ = -BALL_I_MAX;
        }
    }
    i_out = BALL_PID_SINGLE_KI * PID_BallPos.integ;

    out = p_out + i_out + d_out;
    if (out > (float)BALL_OUT_MAX) {
        out = (float)BALL_OUT_MAX;
    } else if (out < -(float)BALL_OUT_MAX) {
        out = -(float)BALL_OUT_MAX;
    }
    out *= BALL_DIR_INVERT;

    g_ball_p_out = p_out;
    g_ball_i_out = i_out;
    g_ball_d_out = d_out;
    g_out = out;
#else
    float out;

    ball = ball_pos;
    PID_BallPos.desired = (float)ball_target_pos;
    out = pidout(&PID_BallPos, (float)ball);
    out *= BALL_DIR_INVERT;
    g_out = out;
#endif

    {
        int16_t diff = (int16_t)(ball - ball_target_pos);
        bool ball_is_stable =
            (diff <= BALL_ARRIVE_TOL) && (diff >= -BALL_ARRIVE_TOL);
#if BALL_LOOP_SINGLE && BALL_SINGLE_FRAME_PD
        float speed_abs = g_ball_velocity_filtered;
        if (speed_abs < 0.0f) {
            speed_abs = -speed_abs;
        }
        ball_is_stable = ball_is_stable &&
                         (speed_abs <= BALL_ARRIVE_SPEED_MAX);
#endif
        if (ball_is_stable) {
            if (ball_stable_cnt < 0xFFFFU) {
                ball_stable_cnt++;
            }
            if (ball_stable_cnt >= BALL_ARRIVE_FRAMES) {
                ball_arrived = true;
            }
        } else {
            ball_stable_cnt = 0U;
            ball_arrived = false;
        }
    }

#if BALL_LOOP_SINGLE
    ball_single_drive_target(out);
#else
    step_target_pos = ball_tilt_to_step_pos(out);
#endif
}

void ball_loop_run(void)
{
#if BALL_LOOP_SINGLE
    step_virt_pos = 0;
#if BALL_SINGLE_FRAME_PD
    s_last_frame_seq = (g_ball_frame_seq == 0U)
        ? 0U : (g_ball_frame_seq - 1U);
    s_previous_ball_pos = ball_pos;
    s_previous_frame_time_ms = s_ball_frame_time_ms;
    s_velocity_initialized = false;
    s_frame_timeout_active = false;
    PID_BallPos.integ = 0.0f;
    g_ball_dt_ms = 0U;
    g_ball_velocity_raw = 0.0f;
    g_ball_velocity_filtered = 0.0f;
    g_ball_p_out = 0.0f;
    g_ball_i_out = 0.0f;
    g_ball_d_out = 0.0f;
#endif
#else
    PID_BallPos.integ = 0.0f;
    PID_BallPos.preverror = 0.0f;
#endif
    ball_loop_enable = true;
}

void ball_loop_stop(void)
{
    ball_loop_enable = false;
#if BALL_LOOP_SINGLE
    step_off(Y);
#if BALL_SINGLE_FRAME_PD
    s_velocity_initialized = false;
    s_frame_timeout_active = false;
    PID_BallPos.integ = 0.0f;
    g_ball_velocity_raw = 0.0f;
    g_ball_velocity_filtered = 0.0f;
    g_ball_p_out = 0.0f;
    g_ball_i_out = 0.0f;
    g_ball_d_out = 0.0f;
#endif
#else
    step_target_pos = mechanism_get_level_zero();
#endif
}

void TIMER_BALL_INST_IRQHandler(void)
{
    DL_Timer_clearInterruptStatus(
        TIMER_BALL_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    ball_isr_10ms();
}
