#ifndef __BALL_H
#define __BALL_H

#include <stdbool.h>
#include <stdint.h>

/* 独立的滚球闭环，使用 TIMER_BALL，不受小车 start_run 状态约束。 */
#ifndef BALL_LOOP_SINGLE
#define BALL_LOOP_SINGLE   1
#endif

#ifndef BALL_SINGLE_FRAME_PD
#define BALL_SINGLE_FRAME_PD   1
#endif

void ball_init(void);
void ball_isr_10ms(void);
void ball_loop_run(void);
void ball_loop_stop(void);
void set_ball_position(int16_t pos);
bool wait_ball_pos(int16_t pos);
void set_step_target(int input_pos);
void ball_measurement_update(int16_t position);

extern volatile int16_t ball_pos;
extern volatile int16_t ball_target_pos;
extern volatile int32_t step_target_pos;
extern float g_out;

extern volatile uint32_t g_ball_frame_seq;
extern volatile uint32_t g_ball_dt_ms;
extern volatile float g_ball_velocity_raw;
extern volatile float g_ball_velocity_filtered;
extern volatile float g_ball_p_out;
extern volatile float g_ball_i_out;
extern volatile float g_ball_d_out;

#endif /* __BALL_H */
