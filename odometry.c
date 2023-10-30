/*
 * odometry.c
 *
 *  Created on: Oct 16, 2023
 *      Author: Lukman Harahap
 *     Version: v1.0
 */

#include "odometry.h"

//extern TIM_HandleTypeDef htim2, htim3, htim4;
//
//const int PPR = 2000;
//const double R = 76.0; // Replace with the actual radius of the encoder wheel
//const double cm_per_tick = 2.0 * M_PI * R / PPR;
//const double e1_e2; // distance between encoder 1 and encoder 2
//const double e12_e3; // distance between the midpoint of encoder 1 & 2 and encoder 3
//
//double xPos = 0.0, yPos = 0.0, heading = 0.0;
//int16_t encoderValues[3] = {0}; // Store encoder values for 3 encoders
//static int oldEncoderValues[3] = {0}; // Store previous encoder values
//
///****** PID control parameters ******/
//double Kp_x = 0.0, Ki_x = 0.0, Kd_x = 0.0;
//double Kp_y = 0.0, Ki_y = 0.0, Kd_y = 0.0;
//double Kp_h = 0.0, Ki_h = 0.0, Kd_h = 0.0;
//
//double integral_x = 0.0, integral_y = 0.0, integral_h = 0.0;
//const int integral_limit = 1000;
//double prevError_x = 0.0, prevError_y = 0.0, prevError_h = 0.0;
//double xSetpoint = 0.0;
//double ySetpoint = 0.0;
//double hSetpoint = 0.0;
//
//typedef struct
//{
//    double x;
//    double y;
//    double h;
//} robotPosition;

//double updatePID(double error, double* integral, double* prevError, double Kp, double Ki, double Kd)
//{
//	*integral += error;
//	if(*integral > integral_limit) *integral = integral_limit;	//prevent integral windup
//	else if(*integral < -integral_limit) *integral = -integral_limit;
//
//	double derivative = error - *prevError;
//	double control = Kp*error + Ki*(*integral) + Kd*derivative;
//	*prevError = error;
//	return control;
//}

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//    if (htim == &htim2 || htim == &htim3 || htim == &htim4)
//    {
//        int encoderIndex = htim == &htim2 ? 0 : (htim == &htim3 ? 1 : 2);
//        int count = __HAL_TIM_GET_COUNTER(htim);
//        encoderValues[encoderIndex] = (int16_t)(count / 4);
//    }
//}
//
//void encoder_init()
//{
//    HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
//    HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
//    HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
//}

//int16_t encoderRead(uint8_t encoder)
//{
//    if (encoder >= 1 && encoder <= 3)
//    {
//        return encoderValues[encoder - 1];
//    }
//    else
//    {
//        return -1; // Invalid encoder number
//    }
//}

//robotPosition odometry()
//{
//    robotPosition currentPosition;
//
//    double xError = xSetpoint - xPos;
//    double yError = ySetpoint - yPos;
//    double hError = hSetpoint - heading;
//
//    double xControl = updatePID(xError, &integral_x, &prevError_x, Kp_x, Ki_x, Kd_x);
//    double yControl = updatePID(yError, &integral_y, &prevError_y, Kp_y, Ki_y, Kd_y);
//    double hControl = updatePID(hError, &integral_h, &prevError_h, Kp_h, Ki_h, Kd_h);
//
//    int dn1 = encoderValues[1] - oldEncoderValues[1];
//    int dn2 = encoderValues[2] - oldEncoderValues[2];
//    int dn3 = encoderValues[3] - oldEncoderValues[3];
//
//    double dtheta = cm_per_tick * (dn2 - dn1) / e1_e2;
//    double dx = cm_per_tick * (dn1 + dn2) / 2.0;
//    double dy = cm_per_tick * (dn3 - (dn2 - dn1) * e12_e3/e1_e2);
//    double theta = heading + (dtheta / 2.0);
//
//    xPos += dx * cos(theta) - dy * sin(theta);
//    yPos += dx * sin(theta) + dy * cos(theta);
//    heading += dtheta;
//
//    currentPosition.x = xPos;
//    currentPosition.y = yPos;
//    currentPosition.h = heading;
//
//    oldEncoderValues[1] = encoderValues[1];
//    oldEncoderValues[2] = encoderValues[2];
//    oldEncoderValues[3] = encoderValues[3];
//
//    return currentPosition;
//}

extern TIM_HandleTypeDef htim2, htim3, htim4;

//declare
int16_t count1 = 0, count2 = 0, count3 = 0;
int16_t counter1 = 0, counter2 = 0, counter3 = 0;
int enc1 = 0, enc2 = 0, enc3 = 0;
static int oldEnc1 = 0, oldEnc2 = 0, oldEnc3 = 0;

//encoder detail
const int PPR = 2000;
const double R = 76.0; //radius of encoder wheel
const double cm_per_tick = 2.0 * M_PI * R / PPR;
double e1_e2; //distance between encoder 1 & 2
double e12_e3; //distance between the midpoint of encoder 1 & 2 and encoder 3
double xPos = 0.0, yPos = 0.0, heading = 0.0;

// PID constants
double Kp = 0.0, Ki = 0.0, Kd = 0.0;
double integral = 0.0, derivative = 0.0, prev_error = 0.0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2)
	{
		count1 = __HAL_TIM_GET_COUNTER(htim);
		counter1 = count1/4;
	}
	else if(htim == &htim3)
	{
		count2 = __HAL_TIM_GET_COUNTER(htim);
		counter2 = count2/4;
	}
	else if(htim == &htim4)
	{
		count3 = __HAL_TIM_GET_COUNTER(htim);
		counter3 = count3/4;
	}
}

void encoder_init()
{
	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
}

int16_t encoderRead(uint8_t encoder)
{
	if(encoder == 1)
	{
		return counter1;
	}
	else if(encoder == 2)
	{
		return counter2;
	}
	else if(encoder == 3)
	{
		return counter3;
	}
	else
	{
		return -1;
	}
}

double PID_controller(double setpoint, double actual_position)
{
    double error = setpoint - actual_position;

    //Proportional
    double P = Kp * error;

    //Integral
    integral += error;
    double I = Ki * integral;

    //Derivative
    derivative = error - prev_error;
    double D = Kd * derivative;

    double output = P + I + D;

    prev_error = error;

    return output;
}

robotPosition odometry(double setpoint_x, double setpoint_y, double setpoint_h)
{
	robotPosition currentPosition;

	enc1 = encoderRead(1);
	enc2 = encoderRead(2);
	enc3 = encoderRead(3);

	int dn1 = enc1 - oldEnc1;
	int dn2 = enc2 - oldEnc2;
	int dn3 = enc3 - oldEnc3;
	double dtheta = cm_per_tick * (dn2 - dn1)/e1_e2;
	double dx = cm_per_tick * (dn1 + dn2)/2.0;
	double dy = cm_per_tick * (dn3 - (dn2 - dn1) * e12_e3/e1_e2);
	double theta = heading + (dtheta/2.0);

	xPos += dx * cos(theta) - dy * sin(theta);
	yPos += dx * sin(theta) + dy * cos(theta);
	heading += dtheta;

	double pid_x = PID_controller(setpoint_x, xPos);
	double pid_y = PID_controller(setpoint_y, yPos);
	double pid_h = PID_controller(setpoint_h, heading);

	xPos += pid_x;
	yPos += pid_y;
	heading += pid_h;

	currentPosition.x = xPos;
	currentPosition.y = yPos;
	currentPosition.h = heading;

	oldEnc1 = enc1;
	oldEnc2 = enc2;
	oldEnc3 = enc3;

	return currentPosition;
}
