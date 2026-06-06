#ifndef __TASKS_H__
#define __TASKS_H__

#ifdef __cplusplus
extern "C" {
#endif

// Time-triggered cooperative task scheduler.
// Each function self-manages its period via HAL_GetTick().

void task_motor_1khz(void);
void task_imu_200hz(void);
void task_can_tx_imu_200hz(void);
void task_can_tx_motor_100hz(void);
void task_can_test(void);
void task_rtt_scope_10hz(void);
void task_rtt_telemetry_2hz(void);
void task_command_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __TASKS_H__ */
