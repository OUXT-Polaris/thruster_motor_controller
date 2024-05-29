#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32f7xx_hal.h"
#include <memory.h>

typedef struct
{
	TIM_HandleTypeDef* 	pTimer;
	const uint16_t		period;
	const uint32_t		frequency;
	double				current_duty;	// duty is input -1.0 ~ 1.0
	double				previous_duty;	// duty is input -1.0 ~ 1.0

}MotorHandleType;

void motorHandleStructInit(TIM_HandleTypeDef* pTimer, MotorHandleType* pMotor);

#endif /* INC_MOTOR_H_ */
