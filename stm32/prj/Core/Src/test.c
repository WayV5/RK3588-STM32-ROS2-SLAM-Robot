#include "test.h"

void led_test(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(500);
}

void motor_test(void) {
    // 正转 50% 占空比, 2秒
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);   // IN1=1
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); // IN2=0
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 8399);   // 50% duty
    HAL_Delay(2000);
    // 刹车
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}