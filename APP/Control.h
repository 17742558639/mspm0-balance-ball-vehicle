#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    CONTROL_TASK_NONE = 0,
    CONTROL_TASK_A_STOP,
    CONTROL_TASK_AB
} ControlTask;

/* ========================= 控制核心 =========================
 * 三环控制: 循迹转向环 (turn) + yaw 转向环 (yaw, 预留) + 速度环 (speed).
 * 控制周期 10ms (TIMER_0 = TIMA0, 100Hz), 在 TIMER_0_INST_IRQHandler 里跑.
 *
 * 移植自队友工程, 适配:
 *   - yaw 来源: attitude.yaw (自写 Mahony, 不用 MPU6050 DMP)
 *   - 精简: 只后轮 bL/bR (前轮 ahind / al / ar 已删)
 *   - start_run 安全开关启用 (ISR 顶部判断, 上电不跑)
 *
 * 对外接口 (system.c 调用):
 *   control_init()        - 初始化 PID 参数 + 启动 100Hz 控制环 (TIMER_0)
 *   control_task()        - 主循环周期任务 (非 ISR), 后续写任务逻辑, 当前空
 *   control_toggle_run()  - 切换 start_run 运行状态 + led 指示 (K3 长按调)
 *
 * 中断:
 *   TIMER_0_INST_IRQHandler - 控制环 10ms (turn + speed, yaw 预留)
 *   (编码器 GROUP1_IRQHandler 在 Encoder.c 实现)
 */

void control_init(void);
void control_task(void);
void control_set_run(bool enabled);
bool control_start_ab(void);
void control_toggle_run(void);
void control_set_motion_enabled(bool enabled);
bool control_motion_enabled(void);
ControlTask control_active_task(void);
bool control_parking_result_ready(void);
bool control_parking_result_success(void);
uint32_t control_parking_elapsed_ms(void);
bool control_ab_result_ready(void);
bool control_ab_result_success(void);
uint32_t control_ab_elapsed_ms(void);
uint32_t control_ab_distance_pulses(void);

void get_speed(void);
void yaw_control(void);
void turn_control(void);
void speed_control(void);

extern int16_t track_error;
extern int16_t track_angle;
extern int16_t track_turn;
extern int16_t gyro_z_speed;
extern int16_t basicSpeed;

extern float   yaw_error;
extern volatile bool start_run;
extern uint32_t isr_cnt;

extern volatile int16_t control_desired_left;
extern volatile int16_t control_desired_right;
extern volatile int16_t control_pwm_left;
extern volatile int16_t control_pwm_right;

#endif /* __CONTROL_H */
