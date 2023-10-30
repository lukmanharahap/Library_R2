/*
 * invicto_motor.h
 *
 *  Created on: Oct 16, 2023
 *      Author: Lukman Harahap
 *     Version: v2.0
 */

#ifndef INC_INVICTO_MOTOR_H_
#define INC_INVICTO_MOTOR_H_

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define LPWM1_Pin GPIO_PIN_14
#define LPWM1_GPIO_Port GPIOC
#define RPWM1_Pin GPIO_PIN_15
#define RPWM1_GPIO_Port GPIOC
#define LPWM2_Pin GPIO_PIN_6
#define LPWM2_GPIO_Port GPIOE
#define RPWM2_Pin GPIO_PIN_13
#define RPWM2_GPIO_Port GPIOC
#define LPWM3_Pin GPIO_PIN_4
#define LPWM3_GPIO_Port GPIOE
#define RPWM3_Pin GPIO_PIN_5
#define RPWM3_GPIO_Port GPIOE
#define LPWM4_Pin GPIO_PIN_12
#define LPWM4_GPIO_Port GPIOC
#define RPWM4_Pin GPIO_PIN_0
#define RPWM4_GPIO_Port GPIOD
#define LPWM5_Pin GPIO_PIN_2
#define LPWM5_GPIO_Port GPIOE
#define RPWM5_Pin GPIO_PIN_3
#define RPWM5_GPIO_Port GPIOE
#define LPWM6_Pin GPIO_PIN_1
#define LPWM6_GPIO_Port GPIOD
#define RPWM6_Pin GPIO_PIN_2
#define RPWM6_GPIO_Port GPIOD
#define LPWM7_Pin GPIO_PIN_0
#define LPWM7_GPIO_Port GPIOE
#define RPWM7_Pin GPIO_PIN_1
#define RPWM7_GPIO_Port GPIOE
#define RPWM8_Pin GPIO_PIN_4
#define RPWM8_GPIO_Port GPIOC
#define LPWM8_Pin GPIO_PIN_5
#define LPWM8_GPIO_Port GPIOC
#define LPWM9_Pin GPIO_PIN_3
#define LPWM9_GPIO_Port GPIOD
#define RPWM9_Pin GPIO_PIN_4
#define RPWM9_GPIO_Port GPIOD
#define RPWM10_Pin GPIO_PIN_0
#define RPWM10_GPIO_Port GPIOB
#define LPWM10_Pin GPIO_PIN_1
#define LPWM10_GPIO_Port GPIOB
#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3
#define MOTOR4 4
#define MOTOR5 5
#define MOTOR6 6
#define MOTOR7 7
#define MOTOR8 8
#define MOTOR9 9
#define MOTOR10 10
#define CW	1
#define CCW	2

// Motor parameters
typedef struct
{
    GPIO_TypeDef* lpwm_port;
    uint16_t lpwm_pin;
    GPIO_TypeDef* rpwm_port;
    uint16_t rpwm_pin;
    double* motorf;
    uint32_t channel;
    TIM_HandleTypeDef* timer;
} Motor;

void motorDirection(uint8_t motor, uint8_t direction);
void Motor_Spd(Motor* motor, double parameter);
void setMotorSpeed(uint8_t motor, double speed);
void Inverse_Kinematics(double Vx, double Vy, double W);

#endif /* INC_INVICTO_MOTOR_H_ */
