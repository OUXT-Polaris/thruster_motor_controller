/*
 * motor.c
 *
 *  Created on: May 21, 2024
 *      Author: KobayashiSouta
 */

#include "motor.h"

void motorHandleStructInit(TIM_HandleTypeDef* pTimer, MotorHandleType* pMotor)
{
	uint16_t		period		= (uint16_t)pTimer->Instance->ARR + 1;
	//const uint32_t
	uint32_t		frequency	=  1 / ((uint32_t)period * pTimer->Instance->PSC + 1);
	MotorHandleType	motor_ = {pTimer, period, frequency, 0.0, 0.0};
	memcpy(pMotor, &motor_, sizeof(motor_));
}
