/*
 * odometry.h
 *
 *  Created on: Oct 16, 2023
 *      Author: Lukman Harahap
 *     Version: v1.0
 */

#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define DT_ENC1_Pin GPIO_PIN_15
#define DT_ENC1_GPIO_Port GPIOA
#define CLK_ENC1_Pin GPIO_PIN_3
#define CLK_ENC1_GPIO_Port GPIOB
#define DT_ENC2_Pin GPIO_PIN_4
#define DT_ENC2_GPIO_Port GPIOB
#define CLK_ENC2_Pin GPIO_PIN_5
#define CLK_ENC2_GPIO_Port GPIOB
#define DT_ENC3_Pin GPIO_PIN_12
#define DT_ENC3_GPIO_Port GPIOD
#define CLK_ENC3_Pin GPIO_PIN_13
#define CLK_ENC3_GPIO_Port GPIOD
#define DT_ENC4_Pin GPIO_PIN_0
#define DT_ENC4_GPIO_Port GPIOA
#define CLK_ENC4_Pin GPIO_PIN_1
#define CLK_ENC4_GPIO_Port GPIOA

typedef struct {
	double x;
	double y;
	double h;
} robotPosition;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void encoder_init();
int16_t encoderRead(uint8_t encoder);
double PID_controller(double setpoint, double actual_position);
robotPosition odometry(double setpoint_x, double setpoint_y, double setpoint_h);


#endif /* INC_ODOMETRY_H_ */
