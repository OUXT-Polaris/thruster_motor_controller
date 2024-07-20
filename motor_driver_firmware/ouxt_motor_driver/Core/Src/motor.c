/*
 * motor.c
 *
 *  Created on: May 21, 2024
 *      Author: KobayashiSouta
 */

#include "motor.h"

/**
  * @brief  Initialize for motor handler structure
  * @param  pTimer : Pointer of timer handler for PWM pins to control motor.
  * 		pMotor : Pointer of motor handler is initialized
  * @retval None
  */
void motorHandleStructInit(TIM_HandleTypeDef* pTimer, MotorHandleType* pMotor)
{
	uint16_t		period			= __HAL_TIM_GET_AUTORELOAD(pTimer) + 1; //(uint16_t)pTimer->Instance->ARR + 1;
	/*Timer clock is double of peripheral clock */
	const uint32_t	source_clk_freq	= 2 * ((pTimer->Instance == TIM1 || pTimer->Instance == TIM8) ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq());
	uint32_t		frequency		= source_clk_freq / ((uint32_t)period * pTimer->Instance->PSC + 1);
	MotorHandleType	motor_			= {pTimer, period, frequency, 0.0, 0.0};
	memcpy(pMotor, &motor_, sizeof(motor_));
}

/**
  * @brief  Enable PWM output
  * @param  pMotor : Pointer of motor handler is enabled PWM
  * @retval None
  */
void motorEnablePWM_Output(MotorHandleType* pMotor)
{
	/*Enable channel1 outputs*/
	HAL_TIM_PWM_Start(pMotor->pTimer, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(pMotor->pTimer, TIM_CHANNEL_1);

	/*Enable channel2 outputs*/
	HAL_TIM_PWM_Start(pMotor->pTimer, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(pMotor->pTimer, TIM_CHANNEL_2);
}

/**
  * @brief  Enable PWM output
  * @param  pMotor : Pointer of motor handler is enabled PWM
  * @retval returned 0xFF is duty was out of range
  */
int8_t motorSetSpeed(MotorHandleType* pMotor, const double duty)
{
	if(duty > 1.0 || -1.0 > duty) return -1; // out of input range;

	/*update duty information in motor handler*/
	pMotor->previous_duty = pMotor->current_duty;
	pMotor->current_duty = duty;

	/*calculate capture-compare resistor value with duty and period*/
	const uint16_t ccr_val = (uint16_t) ( (double)pMotor->period * fabs(duty) );

	/*set capture-compare resistor. control method is sign magnitude break.*/
	if(pMotor->current_duty >= 0.0)
	{
		/*positive direction*/
		__HAL_TIM_SET_COMPARE(pMotor->pTimer, TIM_CHANNEL_1, ccr_val);
		__HAL_TIM_SET_COMPARE(pMotor->pTimer, TIM_CHANNEL_2, 0);
	}
	else
	{
		/*negative direction*/
		__HAL_TIM_SET_COMPARE(pMotor->pTimer, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(pMotor->pTimer, TIM_CHANNEL_2, ccr_val);
	}

	/*Enable code below if pwm signal was not output */
	motorEnablePWM_Output(pMotor);

	return 0;
}
