/*
 * invicto_motor.c
 *
 *  Created on: Oct 16, 2023
 *      Author: Lukman Harahap
 *     Version: v2.0
 */

#include "invicto_motor.h"

#define NUM_MOTORS 10
//extern
extern TIM_HandleTypeDef htim1, htim8, htim9;
double motor1f, motor2f, motor3f, motor4f, motor5f, motor6f, motor7f, motor8f, motor9f, motor10f;

Motor motors[NUM_MOTORS] =
{
    {LPWM1_GPIO_Port, LPWM1_Pin, RPWM1_GPIO_Port, RPWM1_Pin, &motor1f, TIM_CHANNEL_1, &htim1},
    {LPWM2_GPIO_Port, LPWM2_Pin, RPWM2_GPIO_Port, RPWM2_Pin, &motor2f, TIM_CHANNEL_2, &htim1},
    {LPWM3_GPIO_Port, LPWM3_Pin, RPWM3_GPIO_Port, RPWM3_Pin, &motor3f, TIM_CHANNEL_3, &htim1},
    {LPWM4_GPIO_Port, LPWM4_Pin, RPWM4_GPIO_Port, RPWM4_Pin, &motor4f, TIM_CHANNEL_4, &htim1},
    {LPWM5_GPIO_Port, LPWM5_Pin, RPWM5_GPIO_Port, RPWM5_Pin, &motor5f, TIM_CHANNEL_1, &htim8},
    {LPWM6_GPIO_Port, LPWM6_Pin, RPWM6_GPIO_Port, RPWM6_Pin, &motor6f, TIM_CHANNEL_2, &htim8},
    {LPWM7_GPIO_Port, LPWM7_Pin, RPWM7_GPIO_Port, RPWM7_Pin, &motor7f, TIM_CHANNEL_3, &htim8},
    {LPWM8_GPIO_Port, LPWM8_Pin, RPWM8_GPIO_Port, RPWM8_Pin, &motor8f, TIM_CHANNEL_4, &htim8},
    {LPWM9_GPIO_Port, LPWM9_Pin, RPWM9_GPIO_Port, RPWM9_Pin, &motor9f, TIM_CHANNEL_1, &htim9},
    {LPWM10_GPIO_Port, LPWM10_Pin, RPWM10_GPIO_Port, RPWM10_Pin, &motor10f, TIM_CHANNEL_2, &htim9}
};

void motorDirection(uint8_t motor, uint8_t direction)
{
    if (motor >= 1 && motor <= NUM_MOTORS)
    {
        Motor* currentMotor = &motors[motor - 1];
        GPIO_PinState lpwm_state = GPIO_PIN_RESET;
        GPIO_PinState rpwm_state = GPIO_PIN_RESET;

        switch (direction)
        {
            case 1:  // CW
                lpwm_state = GPIO_PIN_SET;
                rpwm_state = GPIO_PIN_RESET;
                break;
            case 2:  // CCW
                lpwm_state = GPIO_PIN_RESET;
                rpwm_state = GPIO_PIN_SET;
                break;
            default:
                break;
        }
        HAL_GPIO_WritePin(currentMotor->lpwm_port, currentMotor->lpwm_pin, lpwm_state);
        HAL_GPIO_WritePin(currentMotor->rpwm_port, currentMotor->rpwm_pin, rpwm_state);
    }
}

void Motor_Spd(Motor* motor, double parameter)
{
//    *(motor->motorf) = (uint16_t)(parameter);
    __HAL_TIM_SET_COMPARE(motor->timer, motor->channel, parameter);
}

void setMotorSpeed(uint8_t motor, double speed)
{
    if (motor >= 1 && motor <= NUM_MOTORS)
    {
        Motor* currentMotor = &motors[motor - 1];
        if (speed > 0)
        {
            motorDirection(motor, CW);
        }
        else if (speed < 0)
        {
            motorDirection(motor, CCW);
            speed = -speed;
        }
        else
        {
            motorDirection(motor, 0);
        }
        Motor_Spd(currentMotor, speed);
    }
}

void Inverse_Kinematics(double Vx, double Vy, double W)
{
	double R = 76.0;	//omni radius

	double V1 = sin(M_PI_4)*Vx + cos(M_PI_4)*Vy + R*W;
	double V2 = sin(3*M_PI_4)*Vx + cos(3*M_PI_4)*Vy + R*W;
	double V3 = sin(5*M_PI_4)*Vx + cos(5*M_PI_4)*Vy + R*W;
	double V4 = sin(7*M_PI_4)*Vx + cos(7*M_PI_4)*Vy + R*W;

	setMotorSpeed(MOTOR5, V1);
	setMotorSpeed(MOTOR6, V2);
	setMotorSpeed(MOTOR7, V3);
	setMotorSpeed(MOTOR8, V4);
}

//void motorDirection(uint8_t motor, uint8_t direction)
//{
//    if(motor == 1)
//    {
//        switch (direction)
//        {
//        case 1/*CW*/:
//            HAL_GPIO_WritePin(LPWM1_GPIO_Port, LPWM1_Pin, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(RPWM1_GPIO_Port, RPWM1_Pin, GPIO_PIN_RESET);
//            break;
//        case 2/*CCW*/:
//            HAL_GPIO_WritePin(LPWM1_GPIO_Port, LPWM1_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RPWM1_GPIO_Port, RPWM1_Pin, GPIO_PIN_SET);
//            break;
//        default:
//            HAL_GPIO_WritePin(LPWM1_GPIO_Port, LPWM1_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RPWM1_GPIO_Port, RPWM1_Pin, GPIO_PIN_RESET);
//            break;
//        }
//    }
//    else if(motor == 2)
//    {
//        switch (direction)
//        {
//        case 1/*CW*/:
//            HAL_GPIO_WritePin(LPWM2_GPIO_Port, LPWM2_Pin, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(RPWM2_GPIO_Port, RPWM2_Pin, GPIO_PIN_RESET);
//            break;
//        case 2/*CCW*/:
//            HAL_GPIO_WritePin(LPWM2_GPIO_Port, LPWM2_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RPWM2_GPIO_Port, RPWM2_Pin, GPIO_PIN_SET);
//            break;
//        default:
//            HAL_GPIO_WritePin(LPWM2_GPIO_Port, LPWM2_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RPWM2_GPIO_Port, RPWM2_Pin, GPIO_PIN_RESET);
//            break;
//        }
//    }
//    else if(motor == 3)
//    {
//        switch (direction)
//        {
//        case 1/*CW*/:
//            HAL_GPIO_WritePin(LPWM3_GPIO_Port, LPWM3_Pin, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(RPWM3_GPIO_Port, RPWM3_Pin, GPIO_PIN_RESET);
//            break;
//        case 2/*CCW*/:
//            HAL_GPIO_WritePin(LPWM3_GPIO_Port, LPWM3_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RPWM3_GPIO_Port, RPWM3_Pin, GPIO_PIN_SET);
//            break;
//        default:
//            HAL_GPIO_WritePin(LPWM3_GPIO_Port, LPWM3_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RPWM3_GPIO_Port, RPWM3_Pin, GPIO_PIN_RESET);
//            break;
//        }
//    }
//    else if(motor == 4)
//    {
//        switch (direction)
//        {
//        case 1/*CW*/:
//            HAL_GPIO_WritePin(LPWM4_GPIO_Port, LPWM4_Pin, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(RPWM4_GPIO_Port, RPWM4_Pin, GPIO_PIN_RESET);
//            break;
//        case 2/*CCW*/:
//            HAL_GPIO_WritePin(LPWM4_GPIO_Port, LPWM4_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RPWM4_GPIO_Port, RPWM4_Pin, GPIO_PIN_SET);
//            break;
//        default:
//            HAL_GPIO_WritePin(LPWM4_GPIO_Port, LPWM4_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RPWM4_GPIO_Port, RPWM4_Pin, GPIO_PIN_RESET);
//            break;
//        }
//    }
//    else if(motor == 5)
//    {
//        switch (direction)
//        {
//        case 1/*CW*/:
//            HAL_GPIO_WritePin(LPWM5_GPIO_Port, LPWM5_Pin, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(RPWM5_GPIO_Port, RPWM5_Pin, GPIO_PIN_RESET);
//            break;
//        case 2/*CCW*/:
//            HAL_GPIO_WritePin(LPWM5_GPIO_Port, LPWM5_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RPWM5_GPIO_Port, RPWM5_Pin, GPIO_PIN_SET);
//            break;
//        default:
//            HAL_GPIO_WritePin(LPWM5_GPIO_Port, LPWM5_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RPWM5_GPIO_Port, RPWM5_Pin, GPIO_PIN_RESET);
//            break;
//        }
//    }
//    else if(motor == 6)
//    {
//        switch (direction)
//        {
//        case 1/*CW*/:
//            HAL_GPIO_WritePin(LPWM6_GPIO_Port, LPWM6_Pin, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(RPWM6_GPIO_Port, RPWM6_Pin, GPIO_PIN_RESET);
//            break;
//        case 2/*CCW*/:
//            HAL_GPIO_WritePin(LPWM6_GPIO_Port, LPWM6_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RPWM6_GPIO_Port, RPWM6_Pin, GPIO_PIN_SET);
//            break;
//        default:
//            HAL_GPIO_WritePin(LPWM6_GPIO_Port, LPWM6_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RPWM6_GPIO_Port, RPWM6_Pin, GPIO_PIN_RESET);
//            break;
//        }
//    }
//    else if(motor == 7)
//    {
//        switch (direction)
//        {
//        case 1/*CW*/:
//            HAL_GPIO_WritePin(LPWM7_GPIO_Port, LPWM7_Pin, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(RPWM7_GPIO_Port, RPWM7_Pin, GPIO_PIN_RESET);
//            break;
//        case 2/*CCW*/:
//            HAL_GPIO_WritePin(LPWM7_GPIO_Port, LPWM7_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RPWM7_GPIO_Port, RPWM7_Pin, GPIO_PIN_SET);
//            break;
//        default:
//            HAL_GPIO_WritePin(LPWM7_GPIO_Port, LPWM7_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RPWM7_GPIO_Port, RPWM7_Pin, GPIO_PIN_RESET);
//            break;
//        }
//    }
//    else if(motor == 8)
//    {
//        switch (direction)
//        {
//        case 1/*CW*/:
//            HAL_GPIO_WritePin(LPWM8_GPIO_Port, LPWM8_Pin, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(RPWM8_GPIO_Port, RPWM8_Pin, GPIO_PIN_RESET);
//            break;
//        case 2/*CCW*/:
//            HAL_GPIO_WritePin(LPWM8_GPIO_Port, LPWM8_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RPWM8_GPIO_Port, RPWM8_Pin, GPIO_PIN_SET);
//            break;
//        default:
//            HAL_GPIO_WritePin(LPWM8_GPIO_Port, LPWM8_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RPWM8_GPIO_Port, RPWM8_Pin, GPIO_PIN_RESET);
//            break;
//        }
//    }
//    else if(motor == 9)
//    {
//        switch (direction)
//        {
//        case 1/*CW*/:
//            HAL_GPIO_WritePin(LPWM9_GPIO_Port, LPWM9_Pin, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(RPWM9_GPIO_Port, RPWM9_Pin, GPIO_PIN_RESET);
//            break;
//        case 2/*CCW*/:
//            HAL_GPIO_WritePin(LPWM9_GPIO_Port, LPWM9_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RPWM9_GPIO_Port, RPWM9_Pin, GPIO_PIN_SET);
//            break;
//        default:
//            HAL_GPIO_WritePin(LPWM9_GPIO_Port, LPWM9_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RPWM9_GPIO_Port, RPWM9_Pin, GPIO_PIN_RESET);
//            break;
//        }
//    }
//    else if(motor == 10)
//    {
//        switch (direction)
//        {
//        case 1/*CW*/:
//            HAL_GPIO_WritePin(LPWM10_GPIO_Port, LPWM10_Pin, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(RPWM10_GPIO_Port, RPWM10_Pin, GPIO_PIN_RESET);
//            break;
//        case 2/*CCW*/:
//            HAL_GPIO_WritePin(LPWM10_GPIO_Port, LPWM10_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RPWM10_GPIO_Port, RPWM10_Pin, GPIO_PIN_SET);
//            break;
//        default:
//            HAL_GPIO_WritePin(LPWM10_GPIO_Port, LPWM10_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RPWM10_GPIO_Port, RPWM10_Pin, GPIO_PIN_RESET);
//            break;
//        }
//    }
//}

//void Motor1_Spd(int parameter)
//{
//	motor1f = (uint16_t)(parameter);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor1f);
//}
//
//void Motor2_Spd(int parameter)
//{
//	motor2f = (uint16_t)(parameter);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor2f);
//}
//
//void Motor3_Spd(int parameter)
//{
//	motor3f = (uint16_t)(parameter);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, motor3f);
//}
//
//void Motor4_Spd(int parameter)
//{
//	motor4f = (uint16_t)(parameter);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, motor4f);
//}
//
//void Motor5_Spd(int parameter)
//{
//	motor5f = (uint16_t)(parameter);
//    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, motor5f);
//}
//
//void Motor6_Spd(int parameter)
//{
//	motor6f = (uint16_t)(parameter);
//    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, motor6f);
//}
//
//void Motor7_Spd(int parameter)
//{
//	motor7f = (uint16_t)(parameter);
//    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, motor7f);
//}
//
//void Motor8_Spd(int parameter)
//{
//	motor8f = (uint16_t)(parameter);
//    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, motor8f);
//}
//
//void Motor9_Spd(int parameter)
//{
//	motor9f = (uint16_t)(parameter);
//    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, motor9f);
//}
//
//void Motor10_Spd(int parameter)
//{
//	motor10f = (uint16_t)(parameter);
//    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, motor10f);
//}

//void setmotor(uint8_t motor, long speed)
//{
//	if(speed == 0)
//	{
//		switch(motor)
//		{
//		case 1: direksi_motor(MOTOR1, 0); Motor1_Spd(spd_motor); break;
//		case 2: direksi_motor(MOTOR2, 0); Motor2_Spd(spd_motor); break;
//		case 3: direksi_motor(MOTOR3, 0); Motor3_Spd(spd_motor); break;
//		case 4: direksi_motor(MOTOR4, 0); Motor4_Spd(spd_motor); break;
//		case 5: direksi_motor(MOTOR5, 0); Motor5_Spd(spd_motor); break;
//		case 6: direksi_motor(MOTOR6, 0); Motor6_Spd(spd_motor); break;
//		case 7: direksi_motor(MOTOR7, 0); Motor7_Spd(spd_motor); break;
//		case 8: direksi_motor(MOTOR8, 0); Motor8_Spd(spd_motor); break;
//		case 9: direksi_motor(MOTOR9, 0); Motor9_Spd(spd_motor); break;
//		case 10: direksi_motor(MOTOR10, 0); Motor10_Spd(spd_motor); break;
//		}
//	}
//	else if (speed > 0)
//	{
//		switch(motor)
//		{
//		case 1: direksi_motor(MOTOR1, CW); Motor1_Spd(spd_motor); break;
//		case 2: direksi_motor(MOTOR2, CW); Motor2_Spd(spd_motor); break;
//		case 3: direksi_motor(MOTOR3, CW); Motor3_Spd(spd_motor); break;
//		case 4: direksi_motor(MOTOR4, CW); Motor4_Spd(spd_motor); break;
//		case 5: direksi_motor(MOTOR5, CW); Motor5_Spd(spd_motor); break;
//		case 6: direksi_motor(MOTOR6, CW); Motor6_Spd(spd_motor); break;
//		case 7: direksi_motor(MOTOR7, CW); Motor7_Spd(spd_motor); break;
//		case 8: direksi_motor(MOTOR8, CW); Motor8_Spd(spd_motor); break;
//		case 9: direksi_motor(MOTOR9, CW); Motor9_Spd(spd_motor); break;
//		case 10: direksi_motor(MOTOR10, CW); Motor10_Spd(spd_motor); break;
//		}
//	}
//	else if (speed < 0)
//	{
//		switch(motor)
//		{
//		case 1: direksi_motor(MOTOR1, CCW); Motor1_Spd(abs(spd_motor)); break;
//		case 2: direksi_motor(MOTOR2, CCW); Motor2_Spd(abs(spd_motor)); break;
//		case 3: direksi_motor(MOTOR3, CCW); Motor3_Spd(abs(spd_motor)); break;
//		case 4: direksi_motor(MOTOR4, CCW); Motor4_Spd(abs(spd_motor)); break;
//		case 5: direksi_motor(MOTOR5, CCW); Motor5_Spd(abs(spd_motor)); break;
//		case 6: direksi_motor(MOTOR6, CCW); Motor6_Spd(abs(spd_motor)); break;
//		case 7: direksi_motor(MOTOR7, CCW); Motor7_Spd(abs(spd_motor)); break;
//		case 8: direksi_motor(MOTOR8, CCW); Motor8_Spd(abs(spd_motor)); break;
//		case 9: direksi_motor(MOTOR9, CCW); Motor9_Spd(abs(spd_motor)); break;
//		case 10: direksi_motor(MOTOR10, CCW); Motor10_Spd(abs(spd_motor)); break;
//		}
//	}
//}

//void Inverse_Kinematics(double Vx, double Vy, double theta)
//{
//	V1 = sin(M_PI_4)*Vx + cos(M_PI_4) + R*theta;
//	V2 = sin(3*M_PI_4)*Vx + cos(3*M_PI_4) + R*theta;
//	V3 = sin(5*M_PI_4)*Vx + cos(5*M_PI_4) + R*theta;
//	V4 = sin(7*M_PI_4)*Vx + cos(7*M_PI_4) + R*theta;
//
//	setmotor(MOTOR1, V1);
//	setmotor(MOTOR2, V2);
//	setmotor(MOTOR3, V3);
//	setmotor(MOTOR4, V4);
//}
