/*
 * step_motor.c - 2 路步进电机驱动 (PWM 脉冲 + 方向 GPIO, CC0_DN 中断计步)
 * 详见 step_motor.h. 移植自队友工程, 仅改 include 和宏名.
 *
 * 原理: PWM 输出脉冲, 每个脉冲下降沿触发 CC0_DN 中断, ISR 里 step_count--
 * 到 0 调 step_off 停定时器. step_on 设方向 + 装步数 + startCounter.
 */
#include "ti_msp_dl_config.h"
#include "step_motor.h"

volatile uint32_t step1_count = 0;
volatile uint32_t step2_count = 0;
int step1_sum = 0;
int step2_sum = 0;

/* 只作用于第一路步进电机；第二路仍由原有 step_on(X) 控制。 */
static void step1_frequency_apply(int32_t frequency_hz)
{
    uint32_t frequency;
    uint32_t period;

    DL_Timer_stopCounter(STEP_MOTOR_1_INST);
    if (frequency_hz == 0) {
        return;
    }

    if (frequency_hz < 0) {
        frequency = (uint32_t)(-frequency_hz);
        DL_GPIO_setPins(GPIO_STEP_MOTOR_DIR1_PORT,
                        GPIO_STEP_MOTOR_DIR1_PIN);
    } else {
        frequency = (uint32_t)frequency_hz;
        DL_GPIO_clearPins(GPIO_STEP_MOTOR_DIR1_PORT,
                          GPIO_STEP_MOTOR_DIR1_PIN);
    }

    period = STEP_MOTOR_1_INST_CLK_FREQ / frequency;
    DL_Timer_setLoadValue(STEP_MOTOR_1_INST, period);
    DL_Timer_setTimerCount(STEP_MOTOR_1_INST, period);
    DL_Timer_setCaptureCompareValue(
        STEP_MOTOR_1_INST, period / 2U, GPIO_STEP_MOTOR_1_C0_IDX);
    DL_Timer_startCounter(STEP_MOTOR_1_INST);
}

void Step_Init(void)
{
    DL_Timer_setCaptureCompareValue(STEP_MOTOR_1_INST, 2500, GPIO_STEP_MOTOR_1_C0_IDX);
    DL_Timer_setCaptureCompareValue(STEP_MOTOR_2_INST, 2500, GPIO_STEP_MOTOR_2_C0_IDX);

    NVIC_ClearPendingIRQ(STEP_MOTOR_1_INST_INT_IRQN);
    NVIC_EnableIRQ(STEP_MOTOR_1_INST_INT_IRQN);

    NVIC_ClearPendingIRQ(STEP_MOTOR_2_INST_INT_IRQN);
    NVIC_EnableIRQ(STEP_MOTOR_2_INST_INT_IRQN);
}

void step_off(uint8_t id)
{
    if (id == Y) {
        DL_Timer_stopCounter(STEP_MOTOR_1_INST);
        step1_count = 0;
    } else if (id == X) {
        DL_Timer_stopCounter(STEP_MOTOR_2_INST);
        step2_count = 0;
    }
}

void step_on(int step_num, uint8_t id)
{
    switch (id) {
        case Y:
            DL_Timer_clearInterruptStatus(
                STEP_MOTOR_1_INST, DL_TIMER_INTERRUPT_CC0_DN_EVENT);
            DL_Timer_enableInterrupt(
                STEP_MOTOR_1_INST, DL_TIMER_INTERRUPT_CC0_DN_EVENT);
            if (step_num < 0) {
                step1_sum += step_num;
                step1_count = -step_num;
                DL_GPIO_setPins(GPIO_STEP_MOTOR_DIR1_PORT, GPIO_STEP_MOTOR_DIR1_PIN);
                DL_Timer_startCounter(STEP_MOTOR_1_INST);
            } else if (step_num > 0) {
                step1_sum += step_num;
                step1_count = step_num;
                DL_GPIO_clearPins(GPIO_STEP_MOTOR_DIR1_PORT, GPIO_STEP_MOTOR_DIR1_PIN);
                DL_Timer_startCounter(STEP_MOTOR_1_INST);
            } else {
                step_off(Y);
            }
            break;

        case X:
            if (step_num < 0) {
                step2_sum += step_num;
                step2_count = -step_num;
                DL_GPIO_setPins(GPIO_STEP_MOTOR_DIR2_PORT, GPIO_STEP_MOTOR_DIR2_PIN);
                DL_Timer_startCounter(STEP_MOTOR_2_INST);
            } else if (step_num > 0) {
                step2_sum += step_num;
                step2_count = step_num;
                DL_GPIO_clearPins(GPIO_STEP_MOTOR_DIR2_PORT, GPIO_STEP_MOTOR_DIR2_PIN);
                DL_Timer_startCounter(STEP_MOTOR_2_INST);
            } else {
                step_off(X);
            }
            break;

        default:
            break;
    }
}

void step_on_at_freq(int32_t step_num, int32_t hz)
{
    int32_t signed_hz;

    if (step_num == 0) {
        step_off(Y);
        return;
    }

    if (step1_count > 0U) {
        step_off(Y);
    }

    if (hz < 0) {
        hz = -hz;
    }
    if (hz > STEP_SPEED_MAX_HZ) {
        hz = STEP_SPEED_MAX_HZ;
    } else if (hz < STEP_SPEED_MIN_HZ) {
        hz = STEP_SPEED_MIN_HZ;
    }

    DL_Timer_clearInterruptStatus(
        STEP_MOTOR_1_INST, DL_TIMER_INTERRUPT_CC0_DN_EVENT);
    DL_Timer_enableInterrupt(
        STEP_MOTOR_1_INST, DL_TIMER_INTERRUPT_CC0_DN_EVENT);

    step1_count = (uint32_t)((step_num > 0) ? step_num : -step_num);
    step1_sum += step_num;
    signed_hz = (step_num > 0) ? hz : -hz;
    step1_frequency_apply(signed_hz);
}

void step_drive_hz(int32_t hz)
{
    if (hz > STEP_SPEED_MAX_HZ) {
        hz = STEP_SPEED_MAX_HZ;
    } else if (hz < -STEP_SPEED_MAX_HZ) {
        hz = -STEP_SPEED_MAX_HZ;
    } else if ((hz > 0) && (hz < STEP_SPEED_MIN_HZ)) {
        hz = STEP_SPEED_MIN_HZ;
    } else if ((hz < 0) && (hz > -STEP_SPEED_MIN_HZ)) {
        hz = -STEP_SPEED_MIN_HZ;
    }

    step1_count = 0U;
    DL_Timer_disableInterrupt(
        STEP_MOTOR_1_INST, DL_TIMER_INTERRUPT_CC0_DN_EVENT);
    DL_Timer_clearInterruptStatus(
        STEP_MOTOR_1_INST, DL_TIMER_INTERRUPT_CC0_DN_EVENT);
    step1_frequency_apply(hz);
}

void STEP_MOTOR_1_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(STEP_MOTOR_1_INST)) {
        case DL_TIMER_IIDX_CC0_DN:
            if (step1_count > 0) step1_count--;
            if (step1_count == 0) step_off(Y);
            break;
        default:
            break;
    }
}

void STEP_MOTOR_2_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(STEP_MOTOR_2_INST)) {
        case DL_TIMER_IIDX_CC0_DN:
            if (step2_count > 0) step2_count--;
            if (step2_count == 0) step_off(X);
            break;
        default:
            break;
    }
}
