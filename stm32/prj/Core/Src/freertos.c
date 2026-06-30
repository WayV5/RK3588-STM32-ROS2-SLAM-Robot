/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "imu.h"
#include "can_protocol.h"
#include "rtt_console.h"
#include "rtt_debug.h"
#include "tick_jitter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// IMU mutex — protects g_imu_data from concurrent read/write by vIMUAcquireTask
// and vCANTxSchedulerTask.  Priority inheritance enabled to prevent inversion
// if a low-prio task (e.g. telemetry) ever holds this lock.
osMutexId_t g_imu_mutex;
const osMutexAttr_t g_imu_mutex_attr = {
	.name      = "IMU",
	.attr_bits = osMutexPrioInherit,
};

// External: CAN ISR uses this handle to wake the command dispatch task
extern TaskHandle_t g_cmd_task_handle;

// DWT-cycle jitter measurement contexts (see tick_jitter.h)
static JitterCtx g_jitter_motor = { .name = "motor", .expected_us = 1000 };
static JitterCtx g_jitter_imu   = { .name = "imu",   .expected_us = 4000 };
static JitterCtx g_jitter_cantx = { .name = "cantx", .expected_us = 1000 };

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void vMotorControlTask(void *argument);
void vIMUAcquireTask(void *argument);
void vCANTxSchedulerTask(void *argument);
void vCommandDispatchTask(void *argument);
void vRTTTelemetryTask(void *argument);
/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

// ---------------------------------------------------------------------------
// Task attribute structs
// ---------------------------------------------------------------------------

// [1kHz] Motor PID + encoder — highest priority
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
	.name       = "motor",
	.stack_size = 768,
	.priority   = osPriorityHigh,
};

// [250Hz] IMU acquisition — above normal priority
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
	.name       = "imu",
	.stack_size = 1024,
	.priority   = osPriorityAboveNormal,
};

// [1kHz] CAN TX slot scheduler — normal priority
osThreadId_t cantxTaskHandle;
const osThreadAttr_t cantxTask_attributes = {
	.name       = "cantx",
	.stack_size = 768,
	.priority   = osPriorityNormal,
};

// Event-driven CAN command dispatch — normal priority
osThreadId_t cmdTaskHandle;
const osThreadAttr_t cmdTask_attributes = {
	.name       = "cmd",
	.stack_size = 1024,
	.priority   = osPriorityNormal,
};

// [2Hz] RTT telemetry — lowest priority
osThreadId_t rttTaskHandle;
const osThreadAttr_t rttTask_attributes = {
	.name       = "rtt",
	.stack_size = 768,
	.priority   = osPriorityLow,
};

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */
	tick_jitter_init();
	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	g_imu_mutex = osMutexNew(&g_imu_mutex_attr);
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* USER CODE BEGIN RTOS_THREADS */
	motorTaskHandle = osThreadNew(vMotorControlTask, NULL, &motorTask_attributes);
	imuTaskHandle   = osThreadNew(vIMUAcquireTask,   NULL, &imuTask_attributes);
	cantxTaskHandle = osThreadNew(vCANTxSchedulerTask, NULL, &cantxTask_attributes);
	cmdTaskHandle   = osThreadNew(vCommandDispatchTask, NULL, &cmdTask_attributes);
	rttTaskHandle   = osThreadNew(vRTTTelemetryTask, NULL, &rttTask_attributes);
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* USER CODE END RTOS_EVENTS */

}

// ---------------------------------------------------------------------------
// Task implementations
// ---------------------------------------------------------------------------

/*
 * [1kHz] Motor PID + encoder control.
 *
 * vTaskDelayUntil provides precise 1ms period.  On a 168 MHz Cortex-M4 the
 * motor_control_update() takes ~20 µs, so this task is active < 2 % of the
 * time and blocks (sleeps) the remaining ~98 %.
 */
void vMotorControlTask(void *argument)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	RTT_INF("[TASK] motor started, prio=%lu\n",
		(unsigned long)osPriorityHigh);

	for (;;) {
		tick_jitter_sample(&g_jitter_motor, 1000);  // 1kHz → 1000µs
		motor_control_update();
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
	}
}

/*
 * [250Hz] IMU data acquisition.
 *
 * Reads accelerometer + gyroscope every 4 ms (14-byte burst, ~300 µs on
 * I2C1 @ 400 kHz).  Magnetometer is read at 20 Hz (every 50 ms) internally
 * via HAL_GetTick gating.
 *
 * Holds g_imu_mutex while writing g_imu_data.  IMU I2C read is done outside
 * the lock to keep the critical section short.
 */
void vIMUAcquireTask(void *argument)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	RTT_INF("[TASK] imu started, prio=%lu\n",
		(unsigned long)osPriorityAboveNormal);

	for (;;) {
		tick_jitter_sample(&g_jitter_imu, 4000);  // 250Hz → 4000us
		if (g_imu_ready) {
			ImuRaw6Axis raw;
			if (imu_read_6axis(&raw) == 0) {
				// Lock → write → unlock (critical section ~2 µs)
				osMutexAcquire(g_imu_mutex, osWaitForever);
				g_imu_data.accel[0] = -raw.accel[0];
				g_imu_data.accel[1] = +raw.accel[1];
				g_imu_data.accel[2] = -raw.accel[2];
				g_imu_data.gyro[0]  = -raw.gyro[0] - g_gyro_bias[0];
				g_imu_data.gyro[1]  = +raw.gyro[1] - g_gyro_bias[1];
				g_imu_data.gyro[2]  = -raw.gyro[2] - g_gyro_bias[2];
				g_imu_data.temp      = raw.temp;
				osMutexRelease(g_imu_mutex);

				// Mag @ 20Hz
				static uint32_t last_mag_ms;
				uint32_t now = HAL_GetTick();
				if (g_mag_available && now - last_mag_ms >= MAG_READ_MS) {
					ImuRawMag raw_mag;
					if (imu_read_mag(&raw_mag) == 0) {
						osMutexAcquire(g_imu_mutex, osWaitForever);
						g_imu_data.mag[0] = -raw_mag.mag[1];
						g_imu_data.mag[1] = +raw_mag.mag[0];
						g_imu_data.mag[2] = +raw_mag.mag[2];
						osMutexRelease(g_imu_mutex);
					}
					last_mag_ms = now;
				}
			}
		}
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4));
	}
}

/*
 * [1kHz] CAN TX slot scheduler.
 *
 * 8-slot round-robin at 1 ms per slot (8 ms full cycle):
 *   0: 0x201 motor speeds 125 Hz
 *   1: 0x202 motor PWM     10 Hz
 *   2,6: 0x203 accel      250 Hz
 *   3,7: 0x204 gyro       250 Hz
 *   4: 0x205 mag+temp      20 Hz
 *   5: 0x206 system status  1 Hz
 *
 * Takes stack snapshots of IMU and motor data under brief locks to avoid
 * reading partially-updated data mid-encode.
 */
void vCANTxSchedulerTask(void *argument)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	static uint8_t  slot;
	static uint8_t  cnt_slot1;   // 0x202 phase counter
	static uint8_t  cnt_slot4;   // 0x205 phase counter
	static uint16_t cnt_slot5;   // 0x206 phase counter
	uint8_t buf[8];
	Motor *m;

	RTT_INF("[TASK] cantx started, prio=%lu\n",
		(unsigned long)osPriorityNormal);

	for (;;) {
		tick_jitter_sample(&g_jitter_cantx, 1000);  // 1kHz → 1000us
		if (g_can_mode == 0) {
			// Snapshot IMU data under mutex
			ImuData imu_snap;
			osMutexAcquire(g_imu_mutex, osWaitForever);
			imu_snap = g_imu_data;
			osMutexRelease(g_imu_mutex);

			// Snapshot motor speed/PWM (int16 atomic on M4; copy together
			// to avoid racing across slots)
			int16_t spd[4], pwm[4];
			for (int i = 0; i < 4; i++) {
				m = motor_get((MotorID)i);
				spd[i] = m->actual_speed;
				pwm[i] = (int16_t)m->pwm_output;
			}

			switch (slot) {
			case 0: // 0x201: all 4 wheel speeds — 125 Hz
				for (int i = 0; i < 4; i++)
					put_i16(&buf[i * 2], spd[i]);
				can_send_frame(CAN_ID_MOTOR_SPEED, buf, 8);
				break;

			case 1: // 0x202: all 4 PWM — ~10 Hz
				cnt_slot1++;
				if (cnt_slot1 >= 12) {
					cnt_slot1 = 0;
					for (int i = 0; i < 4; i++)
						put_i16(&buf[i * 2], pwm[i]);
					can_send_frame(CAN_ID_MOTOR_PWM, buf, 8);
				}
				break;

			case 2: case 6: // 0x203: Accel — 250 Hz
				if (g_imu_ready) {
					put_i16(&buf[0], imu_snap.accel[0]);
					put_i16(&buf[2], imu_snap.accel[1]);
					put_i16(&buf[4], imu_snap.accel[2]);
					can_send_frame(CAN_ID_IMU_ACCEL, buf, 6);
				}
				break;

			case 3: case 7: // 0x204: Gyro — 250 Hz
				if (g_imu_ready) {
					put_i16(&buf[0], imu_snap.gyro[0]);
					put_i16(&buf[2], imu_snap.gyro[1]);
					put_i16(&buf[4], imu_snap.gyro[2]);
					can_send_frame(CAN_ID_IMU_GYRO, buf, 6);
				}
				break;

			case 4: // 0x205: Mag+Temp — ~20 Hz
				cnt_slot4++;
				if (cnt_slot4 >= 6) {
					cnt_slot4 = 0;
					if (g_imu_ready) {
						put_i16(&buf[0], imu_snap.mag[0]);
						put_i16(&buf[2], imu_snap.mag[1]);
						put_i16(&buf[4], imu_snap.mag[2]);
						put_i16(&buf[6], imu_snap.temp);
						can_send_frame(CAN_ID_IMU_MAG, buf, 8);
					}
				}
				break;

			case 5: // 0x206: System status — 1 Hz
				cnt_slot5++;
				if (cnt_slot5 >= 125) {
					cnt_slot5 = 0;
					put_i16(&buf[2], 0); // reserved (battery ADC TODO)
					buf[0] = can_get_status_flags();
					buf[1] = can_get_fault_code();
					can_send_frame(CAN_ID_SYS_STATUS, buf, 4);
				}
				break;
			}
			slot = (slot + 1) & 7;

		} else if (g_can_mode == 1) {
			can_send_test();
		}

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
	}
}

/*
 * Event-driven CAN command dispatch.
 *
 * Blocks on ulTaskNotifyTake() — woken by CAN RX ISR via
 * vTaskNotifyGiveFromISR().  Each wakeup drains the ring buffer completely
 * (can_command_process() runs while(ringbuf_pop) internally).
 *
 * Also polls RTT console input for debug commands.
 */
void vCommandDispatchTask(void *argument)
{
	// Register handle so the CAN ISR can wake us
	g_cmd_task_handle = xTaskGetCurrentTaskHandle();

	RTT_INF("[TASK] cmd started, prio=%lu\n",
		(unsigned long)osPriorityNormal);

	for (;;) {
		// Block until CAN ISR notifies us (consumes the notification)
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		// Drain all pending CAN frames
		can_command_process();

		// Also poll RTT console input
		rtt_console_poll();
	}
}

/*
 * [2Hz] RTT telemetry output.
 *
 * Prints motor/IMU telemetry text and stack high-water marks every 500 ms.
 * Lowest priority — if the system is busy this task simply skips a beat.
 */
void vRTTTelemetryTask(void *argument)
{
	RTT_INF("[TASK] rtt started, prio=%lu\n",
		(unsigned long)osPriorityLow);

	for (;;) {
		vTaskDelay(pdMS_TO_TICKS(500));

		// Print stack high-water marks for all tasks (debug / tuning)
		RTT_INF("=== STACK ===\n");
		RTT_INF("  motor %5lu  imu %5lu  cantx %5lu  cmd %5lu  rtt %5lu\n",
			(unsigned long)uxTaskGetStackHighWaterMark(motorTaskHandle),
			(unsigned long)uxTaskGetStackHighWaterMark(imuTaskHandle),
			(unsigned long)uxTaskGetStackHighWaterMark(cantxTaskHandle),
			(unsigned long)uxTaskGetStackHighWaterMark(cmdTaskHandle),
			(unsigned long)uxTaskGetStackHighWaterMark(rttTaskHandle));

		// Print task period jitter (DWT cycle counter, 168 MHz)
		RTT_INF("=== JITTER ===\n");
		RTT_INF("  %s\n", tick_jitter_report(&g_jitter_motor));
		RTT_INF("  %s\n", tick_jitter_report(&g_jitter_imu));
		RTT_INF("  %s\n", tick_jitter_report(&g_jitter_cantx));

		// Existing telemetry output
		rtt_telemetry_output();
	}
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
